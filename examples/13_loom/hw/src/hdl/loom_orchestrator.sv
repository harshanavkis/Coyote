import lynxTypes::*;
import loom_pkg::*;

module loom_orchestrator (
  input  logic                      aclk,
  input  logic                      aresetn,
  AXI4L.s                           axi_ctrl,

  // uniform transaction out (to the router)
  output loom_txn_t                 txn,
  output logic                      txn_valid,
  input  logic                      txn_ready,

  // completion back from the router
  input  logic                      rsp_valid,
  input  logic [AXIL_DATA_BITS-1:0] rsp_data,
  input  logic                      rsp_err,

  // status + debug readback (driven by the router, read over AXI-Lite)
  input  logic                      busy,
  input  logic                      done_valid,
  input  logic [AXIL_DATA_BITS-1:0] dbg_cnt [N_CNT*2],
  input  logic [63:0]               dbg_tick,
  input  logic [31:0]               drop_count,

  // config out (CSR slices handed to the router)
  output logic [AXIL_DATA_BITS-1:0] range_tbl [N_RANGE*RANGE_W],
  output logic [PID_BITS-1:0]       coyote_pid,
  output logic                      cnt_clear
);

localparam integer ADDR_MSB      = $clog2(N_REGS);
localparam integer ADDR_LSB      = $clog2(AXIL_DATA_BITS/8);
localparam integer AXI_ADDR_BITS = ADDR_LSB + ADDR_MSB;

logic [AXI_ADDR_BITS-1:0]  axi_awaddr, axi_araddr;
logic                      axi_awready, axi_arready, axi_bvalid, axi_wready, axi_rvalid, aw_en;
logic [1:0]                axi_bresp, axi_rresp;
logic [AXIL_DATA_BITS-1:0] axi_rdata;

logic [N_REGS-1:0][AXIL_DATA_BITS-1:0] ctrl_reg;   // the register file
logic [31:0]               done_count;

// which register is being written / read, and whether it is the aperture window
wire [ADDR_MSB-1:0] wr_idx = axi_awaddr[ADDR_LSB+:ADDR_MSB];
wire [ADDR_MSB-1:0] rd_idx = axi_araddr[ADDR_LSB+:ADDR_MSB];
wire wr_is_ap = (wr_idx >= AP_LO) && (wr_idx <= AP_HI);
wire rd_is_ap = (rd_idx >= AP_LO) && (rd_idx <= AP_HI);

wire [VADDR_BITS-1:0] ap_base = {ctrl_reg[AP_BASE_HI_REG][VADDR_BITS-32-1:0],
                                 ctrl_reg[AP_BASE_LO_REG][31:0]};

logic dma_clear_start;

// register file
// One always_ff drives ctrl_reg (a packed 2D array MUST have a single driver).
// Aperture writes are trapped, not stored.
wire ctrl_reg_wren = axi_wready && axi_ctrl.wvalid && axi_awready && axi_ctrl.awvalid;

always_ff @(posedge aclk) begin
  if (!aresetn) ctrl_reg <= 0;
  else begin
    if (dma_clear_start) ctrl_reg[CMD_REG][0] <= 1'b0;   // auto-clear the start bit
    if (ctrl_reg_wren && !wr_is_ap)
      for (int i = 0; i < (AXIL_DATA_BITS/8); i++)
        if (axi_ctrl.wstrb[i])
          ctrl_reg[wr_idx][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
  end
end

always_ff @(posedge aclk) begin
  if (!aresetn)         done_count <= 0;
  else if (done_valid)  done_count <= done_count + 1;
end

// config outputs
genvar gt;
generate
  for (gt = 0; gt < N_RANGE*RANGE_W; gt++)
    assign range_tbl[gt] = ctrl_reg[RANGE_BASE_REG+gt];
endgenerate

assign coyote_pid = ctrl_reg[COYOTE_PID_REG][PID_BITS-1:0];
assign cnt_clear  = ctrl_reg[CNT_CTRL_REG][0];

wire [VADDR_BITS-1:0] win_base  = ctrl_reg[WIN_BASE_REG][VADDR_BITS-1:0];
wire [VADDR_BITS-1:0] win_limit = ctrl_reg[WIN_LIMIT_REG][VADDR_BITS-1:0];

// ingress triggers
// Three things can start a transaction, only one at a time (or_state == OR_IDLE).
wire dma_start = ctrl_reg[CMD_REG][0];
wire rd_accept = axi_arready && axi_ctrl.arvalid && ~axi_rvalid;

typedef enum logic [1:0] {OR_IDLE, OR_ISSUE, OR_WAIT} or_state_t;
or_state_t or_state;

logic armed;                                   // one txn per CMD write (edge, not level)
wire ap_wr_trig = ctrl_reg_wren && wr_is_ap && (or_state == OR_IDLE);
wire ap_rd_trig = rd_accept     && rd_is_ap && (or_state == OR_IDLE);
assign dma_clear_start = dma_start && armed && (or_state == OR_IDLE)
                       && !ap_wr_trig && !ap_rd_trig;

// address translation + bounds
// aperture peer addr = ap_base + (idx-AP_LO)*8. Offset computed in the index
// width (idx >= AP_LO is guaranteed by the trap), then zero-extended.
wire [ADDR_MSB-1:0]   ap_off_wr  = wr_idx - ADDR_MSB'(AP_LO);
wire [ADDR_MSB-1:0]   ap_off_rd  = rd_idx - ADDR_MSB'(AP_LO);
wire [VADDR_BITS-1:0] ap_addr_wr = ap_base + ({{(VADDR_BITS-ADDR_MSB){1'b0}}, ap_off_wr} << 3);
wire [VADDR_BITS-1:0] ap_addr_rd = ap_base + ({{(VADDR_BITS-ADDR_MSB){1'b0}}, ap_off_rd} << 3);

// enforced only when a window is set; +1 bit so addr+len cannot wrap past the limit
function automatic logic bounds_bad(input logic [VADDR_BITS-1:0] a, input logic [LEN_BITS-1:0] l);
  logic [VADDR_BITS:0] top;
  top = {1'b0, a} + {{(VADDR_BITS+1-LEN_BITS){1'b0}}, l};
  bounds_bad = (win_limit != 0) && ((a < win_base) || (top > {1'b0, win_limit}));
endfunction

// clamp a SW-supplied stream index to a real stream
function automatic logic [DEST_BITS-1:0] clamp_strm(input logic [DEST_BITS-1:0] s);
  clamp_strm = (s < DEST_BITS'(N_STRM_AXI)) ? s : '0;
endfunction

// ingress FSM
// OR_IDLE  : a trigger fires -> fill in txn_r, go to OR_ISSUE
// OR_ISSUE : hold txn_valid until the router takes it
// OR_WAIT  : wait for the router's response (or the watchdog), then complete
loom_txn_t txn_r;
logic ap_pending_wr, ap_pending_rd;            // which aperture op is in flight
logic ap_wr_done, ap_rd_done;                  // 1-cycle pulses -> release bvalid/rvalid
logic [AXIL_DATA_BITS-1:0] ap_rdata;
logic ap_err_r;                                // registered with ap_*_done (drives SLVERR)

// watchdog completes a stalled txn before a PCIe completion timeout
localparam integer WD_LIMIT = 1 << 18;
logic [18:0] wd_cnt;
wire  wd_expired = (wd_cnt == WD_LIMIT[18:0]);

always_ff @(posedge aclk) begin
  if (!aresetn)                     wd_cnt <= '0;
  else if (or_state == OR_IDLE)     wd_cnt <= '0;
  else if (!wd_expired)             wd_cnt <= wd_cnt + 1;
end

always_ff @(posedge aclk) begin
  if (!aresetn) begin
    or_state <= OR_IDLE; txn_r <= '0; armed <= 1'b1;
    ap_pending_wr <= 1'b0; ap_pending_rd <= 1'b0;
    ap_wr_done <= 1'b0; ap_rd_done <= 1'b0; ap_err_r <= 1'b0;
    ap_rdata <= '0;
  end else begin
    ap_wr_done <= 1'b0; ap_rd_done <= 1'b0;

    if (dma_clear_start) armed <= 1'b0;
    else if (!dma_start) armed <= 1'b1;

    case (or_state)
      OR_IDLE: begin
        // aperture WRITE: poke 8 B into the peer's memory
        if (ap_wr_trig) begin
          ap_pending_wr     <= 1'b1;
          txn_r.src         <= SRC_APERTURE;
          txn_r.op          <= OP_WRITE;
          txn_r.inline_data <= 1'b1;
          txn_r.data        <= axi_ctrl.wdata;
          txn_r.src_pid     <= '0;
          txn_r.dst_pid     <= ctrl_reg[AP_PID_REG][PID_BITS-1:0];
          txn_r.src_vaddr   <= '0;
          txn_r.dst_vaddr   <= ap_addr_wr;
          txn_r.len         <= LEN_BITS'(AP_BYTES);
          txn_r.src_strm    <= '0;
          txn_r.dst_strm    <= clamp_strm(ctrl_reg[AP_STRM_REG][DEST_BITS-1:0]);
          txn_r.err_bounds  <= bounds_bad(ap_addr_wr, LEN_BITS'(AP_BYTES));
          or_state          <= OR_ISSUE;
        // aperture READ: fetch 8 B from the peer's memory
        end else if (ap_rd_trig) begin
          ap_pending_rd     <= 1'b1;
          txn_r.src         <= SRC_APERTURE;
          txn_r.op          <= OP_READ;
          txn_r.inline_data <= 1'b1;
          txn_r.data        <= '0;
          txn_r.src_pid     <= ctrl_reg[AP_PID_REG][PID_BITS-1:0];
          txn_r.dst_pid     <= '0;
          txn_r.src_vaddr   <= ap_addr_rd;
          txn_r.dst_vaddr   <= ap_addr_rd;
          txn_r.len         <= LEN_BITS'(AP_BYTES);
          txn_r.src_strm    <= clamp_strm(ctrl_reg[AP_STRM_REG][DEST_BITS-1:0]);
          txn_r.dst_strm    <= '0;
          txn_r.err_bounds  <= bounds_bad(ap_addr_rd, LEN_BITS'(AP_BYTES));
          or_state          <= OR_ISSUE;
        // DMA: the big transfer described by the SRC/DST/LEN registers
        end else if (dma_clear_start) begin
          txn_r.src         <= SRC_DMA;
          txn_r.op          <= OP_WRITE;
          txn_r.inline_data <= 1'b0;
          txn_r.data        <= '0;
          txn_r.src_pid     <= ctrl_reg[SRC_PID_REG][PID_BITS-1:0];
          txn_r.dst_pid     <= ctrl_reg[DST_PID_REG][PID_BITS-1:0];
          txn_r.src_vaddr   <= ctrl_reg[SRC_ADDR_REG][VADDR_BITS-1:0];
          txn_r.dst_vaddr   <= ctrl_reg[DST_ADDR_REG][VADDR_BITS-1:0];
          txn_r.len         <= ctrl_reg[LEN_REG][LEN_BITS-1:0];
          txn_r.src_strm    <= clamp_strm(ctrl_reg[SRC_STRM_REG][DEST_BITS-1:0]);
          txn_r.dst_strm    <= clamp_strm(ctrl_reg[DST_STRM_REG][DEST_BITS-1:0]);
          txn_r.err_bounds  <= bounds_bad(ctrl_reg[DST_ADDR_REG][VADDR_BITS-1:0],
                                          ctrl_reg[LEN_REG][LEN_BITS-1:0]);
          or_state          <= OR_ISSUE;
        end
      end

      OR_ISSUE: if (txn_ready) or_state <= OR_WAIT;

      // response arrived: pulse the matching done, latch read data
      OR_WAIT: if (rsp_valid || wd_expired) begin
        ap_err_r <= rsp_err || wd_expired;
        if (ap_pending_wr) begin
          ap_wr_done <= 1'b1; ap_pending_wr <= 1'b0;
        end
        if (ap_pending_rd) begin
          ap_rdata   <= (rsp_err || wd_expired) ? 64'hDEAD_DEAD_DEAD_DEAD : rsp_data;
          ap_rd_done <= 1'b1; ap_pending_rd <= 1'b0;
        end
        or_state <= OR_IDLE;
      end

      default: or_state <= OR_IDLE;
    endcase
  end
end

assign txn       = txn_r;
assign txn_valid = (or_state == OR_ISSUE);

// read data
wire ctrl_reg_rden = axi_arready & axi_ctrl.arvalid & ~axi_rvalid;
always_ff @(posedge aclk) begin
  if (!aresetn)        axi_rdata <= 0;
  else if (ap_rd_done) axi_rdata <= ap_rdata;
  else if (ctrl_reg_rden && !rd_is_ap) begin
    axi_rdata <= 0;
    if      (rd_idx == STATUS_REG)   axi_rdata[0]    <= busy;
    else if (rd_idx == DONE_CNT_REG) axi_rdata[31:0] <= done_count;
    else if (rd_idx == DROP_CNT_REG) axi_rdata[31:0] <= drop_count;
    else if (rd_idx == TICK_REG)     axi_rdata       <= dbg_tick;
    else if (rd_idx >= CNT_BASE_REG && rd_idx < CNT_BASE_REG + N_CNT*2)
                                     axi_rdata       <= dbg_cnt[rd_idx - CNT_BASE_REG];
    else                             axi_rdata       <= ctrl_reg[rd_idx];
  end
end

// AXI-Lite handshake
assign axi_ctrl.awready = axi_awready;
assign axi_ctrl.arready = axi_arready;
assign axi_ctrl.bresp   = axi_bresp;
assign axi_ctrl.bvalid  = axi_bvalid;
assign axi_ctrl.wready  = axi_wready;
assign axi_ctrl.rdata   = axi_rdata;
assign axi_ctrl.rresp   = axi_rresp;
assign axi_ctrl.rvalid  = axi_rvalid;

// Accept-gate: never accept a NEW aperture access while one is in flight 
wire or_idle = (or_state == OR_IDLE);
wire [ADDR_MSB-1:0] aw_idx_bus = axi_ctrl.awaddr[ADDR_LSB+:ADDR_MSB];
wire [ADDR_MSB-1:0] ar_idx_bus = axi_ctrl.araddr[ADDR_LSB+:ADDR_MSB];
wire aw_is_ap_bus = (aw_idx_bus >= AP_LO) && (aw_idx_bus <= AP_HI);
wire ar_is_ap_bus = (ar_idx_bus >= AP_LO) && (ar_idx_bus <= AP_HI);

logic wr_ap_pending, rd_ap_pending;            // a bvalid/rvalid is waiting on the router
wire aw_accept_ok = or_idle || (!aw_is_ap_bus && !wr_ap_pending);
wire ar_accept_ok = or_idle || (!ar_is_ap_bus && !rd_ap_pending);

// write: address/data accept
always_ff @(posedge aclk) begin
  if (!aresetn) begin axi_awready<=0; axi_awaddr<=0; aw_en<=1; end
  else begin
    if (~axi_awready && axi_ctrl.awvalid && axi_ctrl.wvalid && aw_en && aw_accept_ok) begin
      axi_awready<=1; aw_en<=0; axi_awaddr<=axi_ctrl.awaddr;
    end else if (axi_ctrl.bready && axi_bvalid) begin
      aw_en<=1; axi_awready<=0;
    end else axi_awready<=0;
  end
end
always_ff @(posedge aclk) begin
  if (!aresetn) axi_wready<=0;
  else axi_wready <= (~axi_wready && axi_ctrl.wvalid && axi_ctrl.awvalid && aw_en);
end
// write response: immediate for config, wait for ap_wr_done for aperture
always_ff @(posedge aclk) begin
  if (!aresetn) begin axi_bvalid<=0; axi_bresp<=0; wr_ap_pending<=0; end
  else begin
    if (ap_wr_trig) wr_ap_pending <= 1'b1;
    if (axi_awready && axi_ctrl.awvalid && ~axi_bvalid && axi_wready && axi_ctrl.wvalid && !wr_is_ap) begin
      axi_bvalid<=1; axi_bresp<=2'b00;
    end else if (wr_ap_pending && ap_wr_done) begin
      axi_bvalid<=1; axi_bresp<=ap_err_r ? 2'b10 : 2'b00; wr_ap_pending<=0;  // SLVERR on drop
    end else if (axi_ctrl.bready && axi_bvalid) axi_bvalid<=0;
  end
end

// read: address accept
always_ff @(posedge aclk) begin
  if (!aresetn) begin axi_arready<=0; axi_araddr<=0; end
  else begin
    if (~axi_arready && axi_ctrl.arvalid && ar_accept_ok) begin
      axi_arready<=1; axi_araddr<=axi_ctrl.araddr;
    end else axi_arready<=0;
  end
end
// -- read data valid: immediate for config, wait for ap_rd_done for aperture --
always_ff @(posedge aclk) begin
  if (!aresetn) begin axi_rvalid<=0; axi_rresp<=0; rd_ap_pending<=0; end
  else begin
    if (ap_rd_trig) rd_ap_pending <= 1'b1;
    if (axi_arready && axi_ctrl.arvalid && ~axi_rvalid && !rd_is_ap) begin
      axi_rvalid<=1; axi_rresp<=2'b00;
    end else if (rd_ap_pending && ap_rd_done) begin
      axi_rvalid<=1; axi_rresp<=ap_err_r ? 2'b10 : 2'b00; rd_ap_pending<=0;  // SLVERR on drop
    end else if (axi_rvalid && axi_ctrl.rready) axi_rvalid<=0;
  end
end

endmodule
