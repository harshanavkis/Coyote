/**
 * loom_reg_intf — AXI-Lite slave + peer-aperture controller.
 *
 * Config CSRs drive the DMA copy engine (large transfers) and configure the
 * aperture peer {ctid, base, stream}. The APERTURE register window is trapped:
 * an AXI-Lite access to it is performed against the PEER thread's memory (8 B).
 *   write: sq_wr 8 B to peer  -> bvalid once sq_wr accepted (posted, fast).
 *   read : sq_rd 8 B from peer -> rvalid held until data returns (blocking, slow).
 * Each trapped access is one 64-bit word; peer_addr = ap_base + (idx-AP_LO)*8.
 */

import lynxTypes::*;

module loom_reg_intf (
  input  logic                        aclk,
  input  logic                        aresetn,
  AXI4L.s                             axi_ctrl,

  // DMA copy-engine path (large)
  output logic                        start,
  input  logic                        clear_start,
  input  logic                        busy,
  input  logic                        done_valid,
  output logic [LEN_BITS-1:0]         len,
  output logic [PID_BITS-1:0]         src_pid,
  output logic [VADDR_BITS-1:0]       src_addr,
  output logic [3:0]                  src_stream,
  output logic [PID_BITS-1:0]         dst_pid,
  output logic [VADDR_BITS-1:0]       dst_addr,
  output logic [3:0]                  dst_stream,

  // Aperture path (small) — abstract sq + one-beat payload to/from peer
  output logic                        ap_busy,
  output logic                        ap_rd_req, ap_wr_req,
  output logic [PID_BITS-1:0]         ap_pid,
  output logic [VADDR_BITS-1:0]       ap_addr,
  output logic [3:0]                  ap_stream,
  input  logic                        ap_rd_ack, ap_wr_ack,
  input  logic                        ap_rd_cq,
  output logic [AXI_DATA_BITS-1:0]    ap_send_tdata,
  output logic                        ap_send_tvalid, ap_send_tlast,
  input  logic                        ap_send_tready,
  input  logic                        ap_recv_tvalid,
  input  logic [AXI_DATA_BITS-1:0]    ap_recv_tdata,
  output logic                        ap_recv_tready
);

localparam integer N_REGS   = 32;
localparam integer ADDR_MSB = $clog2(N_REGS);
localparam integer ADDR_LSB = $clog2(AXIL_DATA_BITS/8);
localparam integer AXI_ADDR_BITS = ADDR_LSB + ADDR_MSB;

// Register map
localparam integer CMD_REG=0, STATUS_REG=1, DONE_CNT_REG=2;
localparam integer SRC_PID_REG=3, SRC_ADDR_REG=4, SRC_STRM_REG=5;
localparam integer DST_PID_REG=6, DST_ADDR_REG=7, DST_STRM_REG=8, LEN_REG=9;
localparam integer AP_PID_REG=10, AP_BASE_LO_REG=11, AP_BASE_HI_REG=12, AP_STRM_REG=13;
// Aperture window: reg indices [AP_LO..AP_HI] map to peer memory (128 B).
localparam integer AP_LO=16, AP_HI=31;

logic [AXI_ADDR_BITS-1:0] axi_awaddr, axi_araddr;
logic axi_awready, axi_arready, axi_bvalid, axi_wready, axi_rvalid, aw_en;
logic [1:0] axi_bresp, axi_rresp;
logic [AXIL_DATA_BITS-1:0] axi_rdata;

logic [N_REGS-1:0][AXIL_DATA_BITS-1:0] ctrl_reg;
logic [31:0] done_count;

wire [ADDR_MSB-1:0] wr_idx = axi_awaddr[ADDR_LSB+:ADDR_MSB];
wire [ADDR_MSB-1:0] rd_idx = axi_araddr[ADDR_LSB+:ADDR_MSB];
wire wr_is_ap = (wr_idx >= AP_LO) && (wr_idx <= AP_HI);
wire rd_is_ap = (rd_idx >= AP_LO) && (rd_idx <= AP_HI);

wire [VADDR_BITS-1:0] ap_base = {ctrl_reg[AP_BASE_HI_REG][VADDR_BITS-32-1:0], ctrl_reg[AP_BASE_LO_REG][31:0]};

// ---------------- Config register writes (non-aperture) ----------------
wire ctrl_reg_wren = axi_wready && axi_ctrl.wvalid && axi_awready && axi_ctrl.awvalid;

always_ff @(posedge aclk) begin
  if (!aresetn) ctrl_reg <= 0;
  else begin
    if (clear_start) ctrl_reg[CMD_REG][0] <= 1'b0;
    if (ctrl_reg_wren && !wr_is_ap)   // aperture writes are trapped, not stored
      for (int i = 0; i < (AXIL_DATA_BITS/8); i++)
        if (axi_ctrl.wstrb[i])
          ctrl_reg[wr_idx][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
  end
end

always_ff @(posedge aclk) begin
  if (!aresetn) done_count <= 0;
  else if (done_valid) done_count <= done_count + 1;
end

// ---------------- Aperture FSM ----------------
typedef enum logic [1:0] {AP_IDLE, AP_WR, AP_RD} ap_state_t;
ap_state_t ap_state;
logic ap_sq_issued, ap_sent, ap_wr_done, ap_rd_done;
logic [AXIL_DATA_BITS-1:0] ap_wdata, ap_rdata;
logic [VADDR_BITS-1:0]     ap_paddr;

assign ap_busy = (ap_state != AP_IDLE);
assign ap_pid    = ctrl_reg[AP_PID_REG][PID_BITS-1:0];
assign ap_stream = ctrl_reg[AP_STRM_REG][3:0];
assign ap_addr   = ap_paddr;

// triggers: write/read accepted to an aperture address
wire ap_wr_trig = ctrl_reg_wren && wr_is_ap && (ap_state == AP_IDLE);
wire rd_accept  = axi_arready && axi_ctrl.arvalid && ~axi_rvalid;
wire ap_rd_trig = rd_accept && rd_is_ap && (ap_state == AP_IDLE);

wire ap_send_beat = ap_send_tvalid && ap_send_tready;
wire ap_recv_beat = ap_recv_tvalid && ap_recv_tready;

always_ff @(posedge aclk) begin
  if (!aresetn) begin
    ap_state <= AP_IDLE; ap_sq_issued <= 0; ap_sent <= 0;
    ap_wr_done <= 0; ap_rd_done <= 0; ap_wdata <= 0; ap_rdata <= 0; ap_paddr <= 0;
  end else begin
    ap_wr_done <= 0; ap_rd_done <= 0;
    case (ap_state)
      AP_IDLE: begin
        ap_sq_issued <= 0; ap_sent <= 0;
        if (ap_wr_trig) begin
          ap_wdata <= axi_ctrl.wdata;
          ap_paddr <= ap_base + ((wr_idx - AP_LO) << 3);   // *8 bytes
          ap_state <= AP_WR;
        end else if (ap_rd_trig) begin
          ap_paddr <= ap_base + ((rd_idx - AP_LO) << 3);
          ap_state <= AP_RD;
        end
      end
      AP_WR: begin  // posted: done once sq_wr accepted AND the 8B beat sent
        if (ap_wr_ack)   ap_sq_issued <= 1;
        if (ap_send_beat) ap_sent <= 1;
        if ((ap_wr_ack || ap_sq_issued) && (ap_send_beat || ap_sent)) begin
          ap_wr_done <= 1; ap_state <= AP_IDLE;
        end
      end
      AP_RD: begin  // blocking: wait for the peer data beat
        if (ap_rd_ack) ap_sq_issued <= 1;
        if (ap_recv_beat) begin
          ap_rdata <= ap_recv_tdata[AXIL_DATA_BITS-1:0];   // low 8 B
          ap_rd_done <= 1; ap_state <= AP_IDLE;
        end
      end
      default: ap_state <= AP_IDLE;
    endcase
  end
end

// aperture sq + payload beat
assign ap_wr_req      = (ap_state == AP_WR) && ~ap_sq_issued;
assign ap_rd_req      = (ap_state == AP_RD) && ~ap_sq_issued;
assign ap_send_tvalid = (ap_state == AP_WR) && ~ap_sent;
assign ap_send_tdata  = {{(AXI_DATA_BITS-AXIL_DATA_BITS){1'b0}}, ap_wdata};
assign ap_send_tlast  = 1'b1;
assign ap_recv_tready = (ap_state == AP_RD);

// ---------------- Read data ----------------
wire ctrl_reg_rden = axi_arready & axi_ctrl.arvalid & ~axi_rvalid;
always_ff @(posedge aclk) begin
  if (!aresetn) axi_rdata <= 0;
  else if (ap_rd_done) axi_rdata <= ap_rdata;      // aperture read result
  else if (ctrl_reg_rden && !rd_is_ap) begin
    axi_rdata <= 0;
    unique case (rd_idx)
      STATUS_REG:   axi_rdata[0]    <= busy;
      DONE_CNT_REG: axi_rdata[31:0] <= done_count;
      default:      axi_rdata       <= ctrl_reg[rd_idx];
    endcase
  end
end

// ---------------- Config outputs (DMA) ----------------
always_comb begin
  start      = ctrl_reg[CMD_REG][0];
  src_pid    = ctrl_reg[SRC_PID_REG][PID_BITS-1:0];
  src_addr   = ctrl_reg[SRC_ADDR_REG][VADDR_BITS-1:0];
  src_stream = ctrl_reg[SRC_STRM_REG][3:0];
  dst_pid    = ctrl_reg[DST_PID_REG][PID_BITS-1:0];
  dst_addr   = ctrl_reg[DST_ADDR_REG][VADDR_BITS-1:0];
  dst_stream = ctrl_reg[DST_STRM_REG][3:0];
  len        = ctrl_reg[LEN_REG][LEN_BITS-1:0];
end

// ---------------- AXI-Lite handshakes (bvalid/rvalid gated on aperture) ----------------
assign axi_ctrl.awready = axi_awready;
assign axi_ctrl.arready = axi_arready;
assign axi_ctrl.bresp   = axi_bresp;
assign axi_ctrl.bvalid  = axi_bvalid;
assign axi_ctrl.wready  = axi_wready;
assign axi_ctrl.rdata   = axi_rdata;
assign axi_ctrl.rresp   = axi_rresp;
assign axi_ctrl.rvalid  = axi_rvalid;

// write address/data accept
always_ff @(posedge aclk) begin
  if (!aresetn) begin axi_awready<=0; axi_awaddr<=0; aw_en<=1; end
  else begin
    if (~axi_awready && axi_ctrl.awvalid && axi_ctrl.wvalid && aw_en) begin
      axi_awready<=1; aw_en<=0; axi_awaddr<=axi_ctrl.awaddr;
    end else if (axi_ctrl.bready && axi_bvalid) begin aw_en<=1; axi_awready<=0;
    end else axi_awready<=0;
  end
end
always_ff @(posedge aclk) begin
  if (!aresetn) axi_wready<=0;
  else axi_wready <= (~axi_wready && axi_ctrl.wvalid && axi_ctrl.awvalid && aw_en);
end
// bvalid: immediate for config; for aperture wait until ap_wr_done
logic wr_ap_pending;
always_ff @(posedge aclk) begin
  if (!aresetn) begin axi_bvalid<=0; axi_bresp<=0; wr_ap_pending<=0; end
  else begin
    if (ap_wr_trig) wr_ap_pending <= 1'b1;
    if (axi_awready && axi_ctrl.awvalid && ~axi_bvalid && axi_wready && axi_ctrl.wvalid && !wr_is_ap) begin
      axi_bvalid<=1; axi_bresp<=0;                 // config write completes now
    end else if (wr_ap_pending && ap_wr_done) begin
      axi_bvalid<=1; axi_bresp<=0; wr_ap_pending<=0;  // aperture write posted
    end else if (axi_ctrl.bready && axi_bvalid) axi_bvalid<=0;
  end
end

// read address accept
always_ff @(posedge aclk) begin
  if (!aresetn) begin axi_arready<=0; axi_araddr<=0; end
  else begin
    if (~axi_arready && axi_ctrl.arvalid) begin axi_arready<=1; axi_araddr<=axi_ctrl.araddr;
    end else axi_arready<=0;
  end
end
// rvalid: immediate for config; for aperture wait until ap_rd_done
logic rd_ap_pending;
always_ff @(posedge aclk) begin
  if (!aresetn) begin axi_rvalid<=0; axi_rresp<=0; rd_ap_pending<=0; end
  else begin
    if (ap_rd_trig) rd_ap_pending <= 1'b1;
    if (axi_arready && axi_ctrl.arvalid && ~axi_rvalid && !rd_is_ap) begin
      axi_rvalid<=1; axi_rresp<=0;                 // config read completes now
    end else if (rd_ap_pending && ap_rd_done) begin
      axi_rvalid<=1; axi_rresp<=0; rd_ap_pending<=0;  // aperture read data ready
    end else if (axi_rvalid && axi_ctrl.rready) axi_rvalid<=0;
  end
end

endmodule
