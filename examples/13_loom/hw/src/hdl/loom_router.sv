import lynxTypes::*;
import loom_pkg::*;

module loom_router (
  input  logic          aclk,
  input  logic          aresetn,

  // uniform transaction in (from the orchestrator)
  input  loom_txn_t     txn,
  input  logic          txn_valid,
  output logic          txn_ready,

  // completion out (to the orchestrator)
  output logic          rsp_valid,
  output logic [AXIL_DATA_BITS-1:0] rsp_data,
  output logic          rsp_err,

  // config in
  input  logic [AXIL_DATA_BITS-1:0] range_tbl [N_RANGE*RANGE_W],
  input  logic [PID_BITS-1:0]       coyote_pid,
  input  logic          cnt_clear,

  // debug out
  output logic [AXIL_DATA_BITS-1:0] dbg_cnt [N_CNT*2],
  output logic [63:0]   dbg_tick,
  output logic [31:0]   drop_count,
  output logic          busy,
  output logic          done_valid,

  // shell descriptor queues
  metaIntf.m            sq_rd,
  metaIntf.m            sq_wr,
  metaIntf.s            cq_rd,
  metaIntf.s            cq_wr,
  metaIntf.s            rq_rd,
  metaIntf.s            rq_wr,

  // shell data streams (constant-indexed elements from vfpga_top)
  AXI4SR.s             host_recv0,
  AXI4SR.s             host_recv1,
  AXI4SR.m             host_send0,
  AXI4SR.m             host_send1,
  AXI4SR.m             rreq_send0,
  AXI4SR.s             rreq_recv0,
  AXI4SR.m             rrsp_send0,
  AXI4SR.s             rrsp_recv0
);

// ============================================================================
// 1. CYCLE COUNTERS 
// ============================================================================

// free-running tick, exposed as a sanity CSR
logic [47:0] tick;
always_ff @(posedge aclk) begin
  if (!aresetn) tick <= '0;
  else tick <= tick + 1'b1;
end
assign dbg_tick = {{(64-48){1'b0}}, tick};

// eight stopwatches: each counts cycles between its start and stop edge
localparam integer EL_BITS  = 32;   // one interval
localparam integer ACC_BITS = 40;   // sum of intervals
logic [N_CNT-1:0]    c_start, c_stop;
logic [ACC_BITS-1:0] acc [N_CNT];
logic [EL_BITS-1:0]  el  [N_CNT];
logic [31:0]         nsm [N_CNT];
logic [N_CNT-1:0]    run;

always_ff @(posedge aclk) begin
  if (!aresetn || cnt_clear) begin
    run <= '0;
    for (int i = 0; i < N_CNT; i++) begin acc[i] <= '0; el[i] <= '0; nsm[i] <= '0; end
  end else begin
    for (int i = 0; i < N_CNT; i++) begin
      if (c_start[i] && c_stop[i]) begin              // zero-length sample
        nsm[i] <= nsm[i] + 1'b1; run[i] <= 1'b0; el[i] <= '0;
      end else if (c_start[i]) begin                  // start timing
        run[i] <= 1'b1; el[i] <= '0;
      end else if (c_stop[i] && run[i]) begin         // stop: bank the interval
        acc[i] <= acc[i] + {{(ACC_BITS-EL_BITS){1'b0}}, el[i]};
        nsm[i] <= nsm[i] + 1'b1; run[i] <= 1'b0; el[i] <= '0;
      end else if (run[i]) begin                      // still timing
        el[i] <= el[i] + 1'b1;
      end
    end
  end
end

genvar gc;
generate
  for (gc = 0; gc < N_CNT; gc++) begin : gen_cnt
    assign dbg_cnt[gc*2]     = {{(AXIL_DATA_BITS-ACC_BITS){1'b0}}, acc[gc]};
    assign dbg_cnt[gc*2 + 1] = {{(AXIL_DATA_BITS-32){1'b0}},      nsm[gc]};
  end
endgenerate

// ============================================================================
// RANGE LOOKUP  (the routing decision)
// ============================================================================

logic [N_RANGE-1:0] hit, src_ok;
loom_txn_t cur;

// the address the range describes is the peer side of the transfer
wire [VADDR_BITS-1:0] key      = (cur.op == OP_READ) ? cur.src_vaddr : cur.dst_vaddr;
wire [PID_BITS-1:0]   peer_pid = (cur.op == OP_READ) ? cur.src_pid   : cur.dst_pid;

// match the DESTINATION address against all 8 entries in parallel
generate
  for (gc = 0; gc < N_RANGE; gc++) begin : gen_match
    wire [VADDR_BITS-1:0] e_base = range_tbl[gc*RANGE_W][VADDR_BITS-1:0];
    wire [LEN_BITS-1:0]   e_len  = range_tbl[gc*RANGE_W+1][LEN_BITS-1:0];
    wire [PID_BITS-1:0]   e_pid  = range_tbl[gc*RANGE_W+1][39:34];
    wire [1:0]            e_ing  = range_tbl[gc*RANGE_W+1][41:40];
    wire                  e_vld  = range_tbl[gc*RANGE_W+1][42];
    // +1 bit so addr+len cannot wrap past the range top
    wire [VADDR_BITS:0] e_top = {1'b0, e_base} + {{(VADDR_BITS+1-LEN_BITS){1'b0}}, e_len};
    wire [VADDR_BITS:0] k_top = {1'b0, key}    + {{(VADDR_BITS+1-LEN_BITS){1'b0}}, cur.len};
    assign hit[gc]    = e_vld && (key >= e_base) && (k_top <= e_top);
    assign src_ok[gc] = (e_pid == peer_pid)
                        && ((cur.src == SRC_APERTURE) ? e_ing[0] : e_ing[1]);
  end
endgenerate

// match the SOURCE address too: a bulk read from a remote source is a pull
logic [N_RANGE-1:0] s_hit, s_ok;
generate
  for (gc = 0; gc < N_RANGE; gc++) begin : gen_smatch
    wire [VADDR_BITS-1:0] b2 = range_tbl[gc*RANGE_W][VADDR_BITS-1:0];
    wire [LEN_BITS-1:0]   l2 = range_tbl[gc*RANGE_W+1][LEN_BITS-1:0];
    wire [PID_BITS-1:0]   p2 = range_tbl[gc*RANGE_W+1][39:34];
    wire [1:0]            i2 = range_tbl[gc*RANGE_W+1][41:40];
    wire                  v2 = range_tbl[gc*RANGE_W+1][42];
    wire [VADDR_BITS:0] t2 = {1'b0, b2} + {{(VADDR_BITS+1-LEN_BITS){1'b0}}, l2};
    wire [VADDR_BITS:0] s2 = {1'b0, cur.src_vaddr} + {{(VADDR_BITS+1-LEN_BITS){1'b0}}, cur.len};
    assign s_hit[gc] = v2 && (cur.src_vaddr >= b2) && (s2 <= t2);
    assign s_ok[gc]  = (p2 == cur.src_pid) && i2[1];
  end
endgenerate

// pick the source match (lowest index wins) and translate the source address
logic s_found, s_srcok; logic [1:0] s_route; logic [VADDR_BITS-1:0] s_raddr;
always_comb begin
  s_found = 1'b0; s_srcok = 1'b0; s_route = 2'd0; s_raddr = '0;
  for (int i = N_RANGE-1; i >= 0; i--)
    if (s_hit[i]) begin
      s_found = 1'b1; s_srcok = s_ok[i];
      s_route = range_tbl[i*RANGE_W+1][33:32];
      s_raddr = range_tbl[i*RANGE_W+2][VADDR_BITS-1:0]
                + (cur.src_vaddr - range_tbl[i*RANGE_W][VADDR_BITS-1:0]);
    end
end
wire want_pull = !cur.inline_data && s_found && s_srcok && (s_route == 2'd1);

// pick the destination match (lowest index wins) and translate the dst address
logic m_found, m_srcok; logic [1:0] m_route;
logic [DEST_BITS-1:0] m_dest; logic [VADDR_BITS-1:0] m_raddr;
always_comb begin
  m_found = 1'b0; m_srcok = 1'b0; m_route = 2'd0; m_dest = '0; m_raddr = '0;
  for (int i = N_RANGE-1; i >= 0; i--)
    if (hit[i]) begin
      m_found = 1'b1; m_srcok = src_ok[i];
      m_route = range_tbl[i*RANGE_W+1][33:32];
      m_dest  = range_tbl[i*RANGE_W+1][31:28];
      m_raddr = range_tbl[i*RANGE_W+2][VADDR_BITS-1:0]     // remote = base + offset
                + (key - range_tbl[i*RANGE_W][VADDR_BITS-1:0]);
    end
end

// no entries programmed -> behave as plain local
logic tbl_any_valid;
always_comb begin
  tbl_any_valid = 1'b0;
  for (int i = 0; i < N_RANGE; i++)
    if (range_tbl[i*RANGE_W+1][42]) tbl_any_valid = 1'b1;
end
wire tbl_empty = ~tbl_any_valid;

// ============================================================================
// EGRESS FSM  (run one transfer: read + write concurrently)
// ============================================================================

typedef enum logic [1:0] {ST_IDLE, ST_LOOKUP, ST_RUN, ST_DROP} state_t;
state_t state_C;

logic rd_issued, wr_issued, ap_sent, ap_got;
logic [AXIL_DATA_BITS-1:0] ap_rdata_r;
loom_route_t cur_route;
logic [DEST_BITS-1:0] cur_dest;      // destination stream
logic [VADDR_BITS-1:0] cur_raddr;    // translated peer address
logic [LEN_BITS-1:0] pull_bytes;
logic [15:0]         pull_pend;      // fragment writes issued but not yet completed

wire is_ap     = cur.inline_data;    // aperture (8 B)
wire is_ap_wr  = is_ap && (cur.op == OP_WRITE);
wire is_ap_rd  = is_ap && (cur.op == OP_READ);
wire is_remote = (cur_route == RT_REMOTE);   // push
wire is_pull   = (cur_route == RT_PULL);

// descriptor acks + write completion
wire rd_ack = sq_rd.valid && sq_rd.ready;
wire wr_ack = sq_wr.valid && sq_wr.ready;
wire wr_cq  = cq_wr.valid;

// stream handshakes (wired in section 6)
logic recv_beat, send_beat;
logic pull_frag_ack;
// pull is done when all bytes are fetched AND all fragment writes have committed
wire pull_all_bytes = (pull_bytes >= cur.len);
wire pull_done      = pull_all_bytes && (pull_pend == 16'd0);

assign busy      = (state_C != ST_IDLE) && !is_ap;   // DMA only (STATUS_REG)
assign txn_ready = (state_C == ST_IDLE) && !sel_rx;  // idle and not serving RX
assign rsp_data  = ap_rdata_r;

always_ff @(posedge aclk) begin
  if (!aresetn) begin
    state_C <= ST_IDLE; cur <= '0; cur_route <= RT_LOCAL; cur_dest <= '0; cur_raddr <= '0;
    rd_issued <= 1'b0; wr_issued <= 1'b0; ap_sent <= 1'b0; ap_got <= 1'b0;
    ap_rdata_r <= '0; pull_bytes <= '0; pull_pend <= '0;
    rsp_valid <= 1'b0; rsp_err <= 1'b0; done_valid <= 1'b0;
  end else begin
    rsp_valid <= 1'b0; done_valid <= 1'b0;

    case (state_C)
      // take a new txn from the orchestrator
      ST_IDLE: if (txn_valid && txn_ready) begin
        cur <= txn;
        rd_issued <= 1'b0; wr_issued <= 1'b0; ap_sent <= 1'b0; ap_got <= 1'b0;
        pull_bytes <= '0; pull_pend <= '0;
        state_C <= ST_LOOKUP;
      end

      // one cycle: turn the range-match into a route, or drop
      ST_LOOKUP: begin
        if (cur.err_bounds
            || (!tbl_empty && want_pull && m_found && m_srcok && m_route == 2'd1)) begin
          state_C <= ST_DROP;                    // remote src + remote dst -> can't
        end else if (!tbl_empty && want_pull) begin
          cur_route <= RT_PULL; cur_raddr <= s_raddr;
          cur_dest  <= (cur.dst_strm < DEST_BITS'(N_STRM_AXI)) ? cur.dst_strm : '0;
          state_C   <= ST_RUN;
        end else if (!tbl_empty && (!m_found || !m_srcok)) begin
          state_C <= ST_DROP;                    // no match / not permitted
        end else if (!tbl_empty && (m_route == 2'd2)) begin
          state_C <= ST_DROP;                    // route == RT_DROP
        end else begin
          cur_route <= tbl_empty ? RT_LOCAL : loom_route_t'(m_route);
          cur_dest  <= tbl_empty ? cur.dst_strm
                       : ((m_dest < DEST_BITS'(N_STRM_AXI)) ? m_dest : '0);
          cur_raddr <= m_raddr;
          state_C   <= ST_RUN;
        end
      end

      // issue read+write; payload streams recv->send in section 6
      ST_RUN: begin
        if (rd_ack) rd_issued <= 1'b1;
        if (wr_ack) wr_issued <= 1'b1;

        if (is_ap_rd) begin                      // capture one 8 B beat, then done
          if (recv_beat) begin ap_rdata_r <= ap_recv_word; ap_got <= 1'b1; end
          if (ap_got || recv_beat) begin
            rsp_valid <= 1'b1; rsp_err <= 1'b0; state_C <= ST_IDLE;
          end
        end else if (is_ap_wr) begin             // posted: done when the beat is out
          if (send_beat) ap_sent <= 1'b1;
          if ((wr_ack || wr_issued) && (send_beat || ap_sent)) begin
            rsp_valid <= 1'b1; rsp_err <= 1'b0; state_C <= ST_IDLE;
          end
        end else if (is_pull) begin              // fragments -> local writes
          pull_bytes <= pull_bytes + (pull_frag_ack ? rq_wr.data.len : '0);
          pull_pend  <= pull_pend + (pull_frag_ack ? 16'd1 : 16'd0) - (wr_cq ? 16'd1 : 16'd0);
          if (pull_done) begin
            done_valid <= 1'b1; rsp_valid <= 1'b1; rsp_err <= 1'b0; state_C <= ST_IDLE;
          end
        end else begin                           // bulk local / push: done on cq_wr
          if (wr_cq) begin
            done_valid <= 1'b1; rsp_valid <= 1'b1; rsp_err <= 1'b0; state_C <= ST_IDLE;
          end
        end
      end

      // bad address: complete with an error so AXI-Lite never hangs
      ST_DROP: begin
        rsp_valid <= 1'b1; rsp_err <= 1'b1; state_C <= ST_IDLE;
      end

      default: state_C <= ST_IDLE;
    endcase

    // watchdog: force-complete a stalled transfer
    if (rt_wd_expired && (state_C == ST_RUN)) begin
      done_valid <= !is_ap; rsp_valid <= 1'b1; rsp_err <= 1'b1; state_C <= ST_IDLE;
    end
  end
end

// watchdog counter
localparam integer RT_WD_LIMIT = 1 << 18;
logic [18:0] rt_wd;
wire rt_wd_expired = (rt_wd == RT_WD_LIMIT[18:0]);
always_ff @(posedge aclk) begin
  if (!aresetn) rt_wd <= '0;
  else if (state_C == ST_IDLE || state_C == ST_LOOKUP) rt_wd <= '0;
  else if (!rt_wd_expired) rt_wd <= rt_wd + 1;
end

wire running = (state_C == ST_RUN);
// a remote aperture read is an RDMA READ (response returns on rreq_recv)
wire ap_rd_remote = is_ap_rd && is_remote;
wire rd_is_rdma   = running && (ap_rd_remote || is_pull);

// ============================================================================
// RX LANDING  (inbound remote requests)
// ============================================================================

// The request metadata (rq_*) arrives in one cycle but its payload streams for
// many, so grab a grant on accept and hold it until the payload's last beat.
logic rx_hold, rx_is_wr, rx_is_rd;
wire  rq_wr_is_rd_resp = is_opcode_rd_resp(rq_wr.data.opcode);
wire  rq_wr_fwd = rq_wr.valid && !rq_wr_is_rd_resp;   // a real inbound write
// one kind of inbound request per grant: a write (which lands payload) wins, so
// only serve a read when no write is being forwarded (rx_payload_last tracks one)
wire  rq_rd_fwd = rq_rd.valid && !rq_wr_fwd;
wire  is_pull_fwd = is_pull && rd_issued;             // pull: sq_rd goes quiet after issue
wire  rx_new = (rq_wr_fwd || rq_rd_fwd) && (state_C == ST_IDLE) && !rx_hold;

// payload-done flags (wired in section 6)
logic rx_wr_last;   // inbound write landed
logic rx_rd_last;   // outbound read response finished
wire  rx_payload_last = rx_is_wr ? rx_wr_last : rx_is_rd ? rx_rd_last : 1'b1;

wire  sel_rx        = rx_hold || rx_new;
wire  rx_serving_rd = rx_hold && rx_is_rd;

// hold the grant across the payload, with a ~4 ms escape hatch
localparam integer RX_TIMEOUT = 1 << 20;
logic [20:0] rx_wd;
wire rx_wd_expired = (rx_wd == RX_TIMEOUT[20:0]);
always_ff @(posedge aclk) begin
  if (!aresetn) begin rx_hold <= 1'b0; rx_wd <= '0; end
  else begin
    if (rx_new && (wr_ack || rd_ack)) begin rx_hold <= 1'b1; rx_wd <= '0; end
    else if (rx_hold && (rx_payload_last || rx_wd_expired)) begin rx_hold <= 1'b0; rx_wd <= '0; end
    else if (rx_hold) rx_wd <= rx_wd + 1;
  end
end
// remember which kind of inbound request we accepted
always_ff @(posedge aclk) begin
  if (!aresetn) begin rx_is_wr <= 1'b0; rx_is_rd <= 1'b0; end
  else if (rx_new && (wr_ack || rd_ack)) begin
    rx_is_wr <= rq_wr_fwd  && wr_ack;
    rx_is_rd <= rq_rd_fwd && rd_ack;
  end
end

// ============================================================================
// DESCRIPTOR STAMPING  (route -> sq_rd / sq_wr fields)
// ============================================================================

wire pull_active = running && is_pull;
always_comb begin
  sq_rd.data = 0;
  sq_wr.data = 0;

  if (sel_rx) begin
    // inbound request: forward the metadata, force the data onto the host
    sq_rd.data      = rq_rd.data;
    sq_rd.data.strm = STRM_HOST;  sq_rd.data.dest = DEST_BITS'(1);
    sq_rd.valid     = rq_rd_fwd;
    sq_wr.data      = rq_wr.data;
    sq_wr.data.strm = STRM_HOST;
    sq_wr.data.dest = rq_wr_is_rd_resp ? DEST_BITS'(0) : DEST_BITS'(1);
    sq_wr.valid     = rq_wr_fwd;
  end else begin
    // --- read side ---
    // an aperture WRITE emits its 8 B directly and reads nothing (no sq_rd)
    if (rd_is_rdma) begin                        // pull / remote aperture read
      sq_rd.data.opcode = APP_READ;   sq_rd.data.strm = STRM_RDMA;
      sq_rd.data.mode = RDMA_MODE_PARSE; sq_rd.data.rdma = 1'b1;
      sq_rd.data.remote = 1'b1; sq_rd.data.actv = 1'b1; sq_rd.data.host = 1'b0;
      sq_rd.data.last = 1'b1; sq_rd.data.dest = DEST_BITS'(0);
      sq_rd.data.pid = coyote_pid; sq_rd.data.vaddr = cur_raddr; sq_rd.data.len = cur.len;
      sq_rd.valid = running && ~rd_issued && !is_pull_fwd && !is_ap_wr;
    end else begin                               // local read from host memory
      sq_rd.data.opcode = LOCAL_READ; sq_rd.data.strm = STRM_HOST; sq_rd.data.last = 1'b1;
      sq_rd.data.dest = cur.src_strm; sq_rd.data.pid = cur.src_pid;
      sq_rd.data.vaddr = cur.src_vaddr; sq_rd.data.len = cur.len;
      sq_rd.valid = running && ~rd_issued && !is_pull_fwd && !is_ap_wr;
    end

    // --- write side ---
    if (pull_active) begin                       // fragment -> local write
      sq_wr.data.opcode = LOCAL_WRITE; sq_wr.data.strm = STRM_HOST; sq_wr.data.last = 1'b1;
      sq_wr.data.dest = cur_dest; sq_wr.data.pid = cur.dst_pid;
      sq_wr.data.vaddr = cur.dst_vaddr + rq_wr.data.vaddr; sq_wr.data.len = rq_wr.data.len;
      sq_wr.valid = rq_wr.valid && rq_wr_is_rd_resp;
    end else if (is_remote) begin                // push = RDMA write
      sq_wr.data.opcode = APP_WRITE; sq_wr.data.strm = STRM_RDMA;
      sq_wr.data.mode = RDMA_MODE_PARSE; sq_wr.data.rdma = 1'b1;
      sq_wr.data.remote = 1'b1; sq_wr.data.actv = 1'b1; sq_wr.data.host = 1'b0;
      sq_wr.data.last = 1'b1; sq_wr.data.dest = DEST_BITS'(0);
      sq_wr.data.pid = coyote_pid; sq_wr.data.vaddr = cur_raddr; sq_wr.data.len = cur.len;
      sq_wr.valid = running && ~wr_issued;
    end else begin                               // local write (aperture read has none)
      sq_wr.data.opcode = LOCAL_WRITE; sq_wr.data.strm = STRM_HOST; sq_wr.data.last = 1'b1;
      sq_wr.data.dest = cur_dest; sq_wr.data.pid = cur.dst_pid;
      sq_wr.data.vaddr = cur.dst_vaddr; sq_wr.data.len = cur.len;
      sq_wr.valid = running && ~wr_issued && !is_ap_rd;
    end
  end

  cq_rd.ready = 1'b1;
  cq_wr.ready = 1'b1;
end

// rq acks: forward during RX / pull, drain a stray read-response only when idle
assign rq_rd.ready = sel_rx && rd_ack;
assign rq_wr.ready = (sel_rx && wr_ack)
                   || (pull_active && rq_wr_is_rd_resp && wr_ack)
                   || (rq_wr_is_rd_resp && (state_C == ST_IDLE) && !sel_rx);
assign pull_frag_ack = pull_active && wr_ack;

// ============================================================================
// STREAM MUXING  (wire the payload source to its sink)
// ============================================================================

wire [DEST_BITS-1:0] rd_strm = cur.src_strm;
wire [DEST_BITS-1:0] wr_strm = cur_dest;

// backpressure for a recv->send passthrough comes from the actual sink
wire send_ready_for_recv = is_remote ? rreq_send0.tready
                         : (wr_strm==DEST_BITS'(1)) ? host_send1.tready
                         : host_send0.tready;

// --- payload SOURCE: the chosen recv stream (or rreq_recv for a pull) ---
logic [AXI_DATA_BITS-1:0]   recv_tdata;
logic [AXI_DATA_BITS/8-1:0] recv_tkeep;
logic recv_tvalid, recv_tlast;
always_comb begin
  recv_tdata  = (rd_strm == DEST_BITS'(1)) ? host_recv1.tdata  : host_recv0.tdata;
  recv_tkeep  = (rd_strm == DEST_BITS'(1)) ? host_recv1.tkeep  : host_recv0.tkeep;
  recv_tvalid = (rd_strm == DEST_BITS'(1)) ? host_recv1.tvalid : host_recv0.tvalid;
  recv_tlast  = (rd_strm == DEST_BITS'(1)) ? host_recv1.tlast  : host_recv0.tlast;
  if (pull_active) begin
    recv_tdata  = rreq_recv0.tdata;  recv_tkeep  = rreq_recv0.tkeep;
    recv_tvalid = rreq_recv0.tvalid; recv_tlast  = rreq_recv0.tlast;
  end
end
wire [AXIL_DATA_BITS-1:0] ap_recv_word = recv_tdata[AXIL_DATA_BITS-1:0];  // aperture read

// --- aperture write: one 8 B beat instead of a stream ---
wire ap_wr_beat_valid = running && is_ap_wr && ~ap_sent;
wire [AXI_DATA_BITS-1:0]   ap_beat_data = {{(AXI_DATA_BITS-AXIL_DATA_BITS){1'b0}}, cur.data};
wire [AXI_DATA_BITS/8-1:0] ap_beat_keep = {{(AXI_DATA_BITS/8-AP_BYTES){1'b0}}, {AP_BYTES{1'b1}}};

wire send_is_rreq = running && is_remote;   // push -> rreq_send0
wire recv_is_host = !pull_active;           // pull reads rreq_recv, not host recv
wire sel1_rx      = sel_rx;                 // inbound write lands on host_send1

// --- SINK: host_send0 (local egress / pull landing) ---
assign host_send0.tvalid = (pull_active && (wr_strm==DEST_BITS'(0))) ? rreq_recv0.tvalid
                         : (running && !is_remote && (wr_strm==DEST_BITS'(0)))
                           ? (is_ap_wr ? ap_wr_beat_valid : recv_tvalid) : 1'b0;
assign host_send0.tdata  = (pull_active && (wr_strm==DEST_BITS'(0))) ? rreq_recv0.tdata
                         : is_ap_wr ? ap_beat_data : recv_tdata;
assign host_send0.tkeep  = (pull_active && (wr_strm==DEST_BITS'(0))) ? rreq_recv0.tkeep
                         : is_ap_wr ? ap_beat_keep : recv_tkeep;
assign host_send0.tlast  = (pull_active && (wr_strm==DEST_BITS'(0))) ? rreq_recv0.tlast
                         : is_ap_wr ? 1'b1 : recv_tlast;
assign host_send0.tid    = '0;

// --- SINK: host_send1 (local egress / pull landing / RX inbound-write landing) ---
assign host_send1.tvalid = sel1_rx ? rrsp_recv0.tvalid
                         : (pull_active && (wr_strm==DEST_BITS'(1))) ? rreq_recv0.tvalid
                         : (running && !is_remote && (wr_strm==DEST_BITS'(1)))
                           ? (is_ap_wr ? ap_wr_beat_valid : recv_tvalid) : 1'b0;
assign host_send1.tdata  = sel1_rx ? rrsp_recv0.tdata
                         : (pull_active && (wr_strm==DEST_BITS'(1))) ? rreq_recv0.tdata
                         : is_ap_wr ? ap_beat_data : recv_tdata;
assign host_send1.tkeep  = sel1_rx ? rrsp_recv0.tkeep
                         : (pull_active && (wr_strm==DEST_BITS'(1))) ? rreq_recv0.tkeep
                         : is_ap_wr ? ap_beat_keep : recv_tkeep;
assign host_send1.tlast  = sel1_rx ? rrsp_recv0.tlast
                         : (pull_active && (wr_strm==DEST_BITS'(1))) ? rreq_recv0.tlast
                         : is_ap_wr ? 1'b1 : recv_tlast;
assign host_send1.tid    = '0;

// --- SINK: rreq_send0 (outgoing RDMA write payload) ---
assign rreq_send0.tvalid = send_is_rreq ? (is_ap_wr ? ap_wr_beat_valid : recv_tvalid) : 1'b0;
assign rreq_send0.tdata  = is_ap_wr ? ap_beat_data : recv_tdata;
assign rreq_send0.tkeep  = is_ap_wr ? ap_beat_keep : recv_tkeep;
assign rreq_send0.tlast  = is_ap_wr ? 1'b1 : recv_tlast;
assign rreq_send0.tid    = '0;

// --- SINK: rrsp_send0 (serving an inbound read: host_recv1 streams back out) ---
assign rrsp_send0.tvalid = rx_serving_rd && host_recv1.tvalid;
assign rrsp_send0.tdata  = host_recv1.tdata;
assign rrsp_send0.tkeep  = host_recv1.tkeep;
assign rrsp_send0.tlast  = host_recv1.tlast;
assign rrsp_send0.tid    = '0;

// --- recv treadys: aperture read self-asserts; else follows the send sink ---
wire ap_rd_local = is_ap_rd && !is_remote;
assign host_recv0.tready = (running && ap_rd_local && (rd_strm==DEST_BITS'(0))) ? 1'b1
                         : (running && recv_is_host && !is_ap_rd && (rd_strm==DEST_BITS'(0)))
                           ? send_ready_for_recv : 1'b0;
assign host_recv1.tready = rx_serving_rd ? rrsp_send0.tready
                         : (running && ap_rd_local && (rd_strm==DEST_BITS'(1))) ? 1'b1
                         : (running && recv_is_host && !is_ap_rd && (rd_strm==DEST_BITS'(1)))
                           ? send_ready_for_recv : 1'b0;
assign rreq_recv0.tready = pull_active ? ((wr_strm==DEST_BITS'(1)) ? host_send1.tready
                                                                   : host_send0.tready)
                         : (rd_is_rdma && is_ap_rd) ? 1'b1 : 1'b0;
assign rrsp_recv0.tready = sel1_rx ? host_send1.tready : 1'b0;

// --- beat observers the FSM watches ---
always_comb begin
  recv_beat = 1'b0;                            // one aperture-read beat
  if (is_ap_rd && running) begin
    if (rd_is_rdma) recv_beat = rreq_recv0.tvalid && rreq_recv0.tready;
    else recv_beat = recv_tvalid &&
                 ((rd_strm==DEST_BITS'(1)) ? host_recv1.tready : host_recv0.tready);
  end
  send_beat = 1'b0;                            // one aperture-write beat
  if (is_ap_wr && running) begin
    if (is_remote) send_beat = rreq_send0.tvalid && rreq_send0.tready;
    else if (wr_strm==DEST_BITS'(1)) send_beat = host_send1.tvalid && host_send1.tready;
    else send_beat = host_send0.tvalid && host_send0.tready;
  end
end

// RX payload-done: last beat on the landing / response stream
assign rx_wr_last = sel1_rx && rrsp_recv0.tvalid && rrsp_recv0.tready && rrsp_recv0.tlast;
assign rx_rd_last = rx_serving_rd && rrsp_send0.tvalid && rrsp_send0.tready && rrsp_send0.tlast;

// ============================================================================
// DROP COUNTER + COUNTER GATING  
// ============================================================================

always_ff @(posedge aclk) begin
  if (!aresetn) drop_count <= '0;
  else if (cnt_clear) drop_count <= '0;
  else if (state_C == ST_DROP) drop_count <= drop_count + 1;   // ST_DROP is 1 cycle
end

wire txn_accept = txn_valid && txn_ready && (state_C == ST_IDLE);
wire lookup_ok  = (state_C == ST_LOOKUP) && !cur.err_bounds
                  && (tbl_empty || (m_found && m_srcok && m_route != 2'd2));
wire lk_remote  = !tbl_empty && (m_route == 2'd1);
wire rdma_ack   = wr_ack && is_remote;

// edge one-shots for the counter start/stop
logic st_run_q, st_lookup_q, rq_seen_q;
wire  rq_seen = rq_wr_fwd || rq_rd_fwd;
always_ff @(posedge aclk) begin
  if (!aresetn) begin st_run_q <= 1'b0; st_lookup_q <= 1'b0; rq_seen_q <= 1'b0; end
  else begin
    st_run_q    <= (state_C == ST_RUN);
    st_lookup_q <= (state_C == ST_LOOKUP);
    rq_seen_q   <= rq_seen;
  end
end
wire enter_run = (state_C == ST_RUN) && !st_run_q;

always_comb begin
  c_start = '0; c_stop = '0;
  c_start[C_TRANSLATE]  = txn_accept;                                // txn latched
  c_stop [C_TRANSLATE]  = (state_C == ST_LOOKUP) && !st_lookup_q;
  c_start[C_LOOKUP]     = (state_C == ST_LOOKUP) && !st_lookup_q;    // range match
  c_stop [C_LOOKUP]     = (state_C != ST_LOOKUP) && st_lookup_q;
  c_start[C_FORWARD]    = lookup_ok && !lk_remote && !st_lookup_q;   // local transfer
  c_stop [C_FORWARD]    = done_valid;
  c_start[C_QUEUE]      = lookup_ok && lk_remote && !st_lookup_q;    // remote: -> RUN
  c_stop [C_QUEUE]      = enter_run && is_remote;
  c_start[C_ENCAP]      = enter_run && is_remote;                    // -> descriptor out
  c_stop [C_ENCAP]      = rdma_ack;
  c_start[C_ROCE_TX]    = rdma_ack;                                  // stack + wire + land
  c_stop [C_ROCE_TX]    = wr_cq && is_remote;
  c_start[C_RX_LAND]    = rq_seen && !rq_seen_q;                     // inbound seen -> accepted
  c_stop [C_RX_LAND]    = rx_new && (wr_ack || rd_ack);
  c_start[C_RX_FORWARD] = rx_new && wr_ack;                          // accepted -> payload done
  c_stop [C_RX_FORWARD] = rx_wr_last;
end

endmodule
