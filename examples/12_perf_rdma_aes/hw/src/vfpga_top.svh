/**
 * Example 12 - AES-GCM over RDMA, encrypt on one node, decrypt on the other.
 *
 *   TX  axis_host_recv[0] -> aes_gcm_bank(ENCRYPT) -> axis_rreq_send[0]
 *   RX  axis_rrsp_recv[0] -> aes_gcm_bank(DECRYPT) -> axis_host_send[1]
 *
 * Control path is stock 09_perf_rdma; only the payload is touched. Both nodes
 * load the same bitstream and differ by CTRL.role. See README.md.
 *
 * Two things you cannot change without breaking it:
 *
 * 1. Byte count per fragment is fixed. dreq_rdma_parser_wr splits a WRITE into
 *    PMTU_BYTES fragments and commits each length to the RoCE header before the
 *    vFPGA sees data, so the last 16 B of every fragment are the tag slot:
 *    TX strips a pad beat and the tag refills it, RX strips the received tag
 *    and the computed tag refills it. Needs every fragment to be a multiple of
 *    16 B and >= 32 B; any transfer size that is a multiple of 64 works.
 *
 * 2. Role must be written before enable. GCM breaks on (key, IV) reuse and both
 *    directions share the zero key, so iv_dir separates them via IV bit 95
 *    (TX = role, RX = ~role). It is sampled when the bank leaves reset, and
 *    CTRL.enable is what releases reset.
 *
 * CSRs: 0 CTRL [0]=enable (0 = bypass) [1]=role | 1 STATUS [0]=rx quarantine
 * [1]=sticky tag-verified | 2 TAG_OK | 3 TX_FRAMES | 4 RX_FRAMES | 5 MEAS_CTRL
 * [0]=arm | 6-7 FIRST_CYCLE | 8-9 LAST_CYCLE (48-bit receive-side stamps).
 */

// =========================================================================
// AXI-Lite CSR block
// =========================================================================
localparam integer N_CSR_REGS = 10;
localparam integer CSR_ADDR_MSB = $clog2(N_CSR_REGS);
localparam integer CSR_ADDR_LSB = $clog2(AXIL_DATA_BITS/8);
localparam integer CSR_AXI_ADDR_BITS = CSR_ADDR_LSB + CSR_ADDR_MSB;

localparam integer REG_CTRL           = 0;
localparam integer REG_STATUS         = 1;
localparam integer REG_TAG_OK         = 2;
localparam integer REG_TX_FRAMES      = 3;
localparam integer REG_RX_FRAMES      = 4;
localparam integer REG_MEAS_CTRL      = 5;
localparam integer REG_FIRST_CYCLE_LO = 6;
localparam integer REG_FIRST_CYCLE_HI = 7;
localparam integer REG_LAST_CYCLE_LO  = 8;
localparam integer REG_LAST_CYCLE_HI  = 9;

logic [N_CSR_REGS-1:0][AXIL_DATA_BITS-1:0] csr_reg;
logic csr_wren;
logic csr_rden;

logic aes_enable;
logic aes_role;
logic meas_armed;

always_comb begin
    aes_enable = csr_reg[REG_CTRL][0];
    aes_role   = csr_reg[REG_CTRL][1];
    meas_armed = csr_reg[REG_MEAS_CTRL][0];
end

logic [CSR_AXI_ADDR_BITS-1:0] csr_awaddr;
logic csr_awready;
logic [CSR_AXI_ADDR_BITS-1:0] csr_araddr;
logic csr_arready;
logic [1:0] csr_bresp;
logic csr_bvalid;
logic csr_wready;
logic [AXIL_DATA_BITS-1:0] csr_rdata;
logic [1:0] csr_rresp;
logic csr_rvalid;
logic csr_aw_en;

assign axi_ctrl.awready = csr_awready;
assign axi_ctrl.arready = csr_arready;
assign axi_ctrl.bresp   = csr_bresp;
assign axi_ctrl.bvalid  = csr_bvalid;
assign axi_ctrl.wready  = csr_wready;
assign axi_ctrl.rdata   = csr_rdata;
assign axi_ctrl.rresp   = csr_rresp;
assign axi_ctrl.rvalid  = csr_rvalid;

assign csr_wren = csr_wready && axi_ctrl.wvalid && csr_awready && axi_ctrl.awvalid;

// Only CTRL and MEAS_CTRL are writable; the rest are hardware status.
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        csr_reg <= '0;
    end else if (csr_wren) begin
        case (csr_awaddr[CSR_ADDR_LSB+:CSR_ADDR_MSB])
            REG_CTRL:
                for (int i = 0; i < (AXIL_DATA_BITS/8); i++)
                    if (axi_ctrl.wstrb[i]) csr_reg[REG_CTRL][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            REG_MEAS_CTRL:
                for (int i = 0; i < (AXIL_DATA_BITS/8); i++)
                    if (axi_ctrl.wstrb[i]) csr_reg[REG_MEAS_CTRL][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            default: ;
        endcase
    end
end

assign csr_rden = csr_arready & axi_ctrl.arvalid & ~csr_rvalid;

logic [31:0] tag_ok_count;
logic [31:0] tx_frame_count;
logic [31:0] rx_frame_count;
logic [47:0] first_cycle;
logic [47:0] last_cycle;
logic        rx_quarantine;
logic        tag_seen;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        csr_rdata <= '0;
    end else if (csr_rden) begin
        csr_rdata <= '0;
        case (csr_araddr[CSR_ADDR_LSB+:CSR_ADDR_MSB])
            REG_CTRL:           csr_rdata <= csr_reg[REG_CTRL];
            REG_STATUS:         csr_rdata <= {{(AXIL_DATA_BITS-2){1'b0}}, tag_seen, rx_quarantine};
            REG_TAG_OK:         csr_rdata <= {{(AXIL_DATA_BITS-32){1'b0}}, tag_ok_count};
            REG_TX_FRAMES:      csr_rdata <= {{(AXIL_DATA_BITS-32){1'b0}}, tx_frame_count};
            REG_RX_FRAMES:      csr_rdata <= {{(AXIL_DATA_BITS-32){1'b0}}, rx_frame_count};
            REG_MEAS_CTRL:      csr_rdata <= csr_reg[REG_MEAS_CTRL];
            REG_FIRST_CYCLE_LO: csr_rdata <= {{(AXIL_DATA_BITS-32){1'b0}}, first_cycle[31:0]};
            REG_FIRST_CYCLE_HI: csr_rdata <= {{(AXIL_DATA_BITS-16){1'b0}}, first_cycle[47:32]};
            REG_LAST_CYCLE_LO:  csr_rdata <= {{(AXIL_DATA_BITS-32){1'b0}}, last_cycle[31:0]};
            REG_LAST_CYCLE_HI:  csr_rdata <= {{(AXIL_DATA_BITS-16){1'b0}}, last_cycle[47:32]};
            default: ;
        endcase
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        csr_awready <= 1'b0;
        csr_awaddr  <= '0;
        csr_aw_en   <= 1'b1;
    end else begin
        if (~csr_awready && axi_ctrl.awvalid && axi_ctrl.wvalid && csr_aw_en) begin
            csr_awready <= 1'b1;
            csr_aw_en   <= 1'b0;
            csr_awaddr  <= axi_ctrl.awaddr;
        end else if (axi_ctrl.bready && csr_bvalid) begin
            csr_aw_en   <= 1'b1;
            csr_awready <= 1'b0;
        end else begin
            csr_awready <= 1'b0;
        end
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        csr_arready <= 1'b0;
        csr_araddr  <= '0;
    end else begin
        if (~csr_arready && axi_ctrl.arvalid) begin
            csr_arready <= 1'b1;
            csr_araddr  <= axi_ctrl.araddr;
        end else begin
            csr_arready <= 1'b0;
        end
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        csr_bvalid <= 1'b0;
        csr_bresp  <= 2'b0;
    end else begin
        if (csr_awready && axi_ctrl.awvalid && ~csr_bvalid && csr_wready && axi_ctrl.wvalid) begin
            csr_bvalid <= 1'b1;
            csr_bresp  <= 2'b0;
        end else if (axi_ctrl.bready && csr_bvalid) begin
            csr_bvalid <= 1'b0;
        end
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        csr_wready <= 1'b0;
    end else begin
        if (~csr_wready && axi_ctrl.wvalid && axi_ctrl.awvalid && csr_aw_en) begin
            csr_wready <= 1'b1;
        end else begin
            csr_wready <= 1'b0;
        end
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        csr_rvalid <= 1'b0;
        csr_rresp  <= 2'b0;
    end else begin
        if (csr_arready && axi_ctrl.arvalid && ~csr_rvalid) begin
            csr_rvalid <= 1'b1;
            csr_rresp  <= 2'b0;
        end else if (csr_rvalid && axi_ctrl.rready) begin
            csr_rvalid <= 1'b0;
        end
    end
end

// =========================================================================
// Control path -- unchanged from 09_perf_rdma
// =========================================================================
always_comb begin
    sq_wr.valid = rq_wr.valid;
    rq_wr.ready = sq_wr.ready;
    sq_wr.data  = rq_wr.data;
    sq_wr.data.strm = STRM_HOST;
    sq_wr.data.dest = is_opcode_rd_resp(rq_wr.data.opcode) ? 0 : 1;

    sq_rd.valid = rq_rd.valid;
    rq_rd.ready = sq_rd.ready;
    sq_rd.data  = rq_rd.data;
    sq_rd.data.strm = STRM_HOST;
    sq_rd.data.dest = 1;
end

// =========================================================================
// AES banks
//
// Held in reset while disabled, so iv_dir is captured on the 0->1 edge of
// CTRL.enable. While disabled the datapath falls back to the stock
// 09_perf_rdma pass-through, which is the baseline this example is measured
// against -- same bitstream, same shell, same software, crypto off.
// =========================================================================
localparam integer KEEP_W = AXI_DATA_BITS/8;

// Engines per direction. Each is a full AES-256-GCM core: the previous
// single-engine build measured ~119k LUTs per core, so with the ~287k LUT
// RDMA shell a U280 realistically takes 2 per direction (4 total). One
// engine sustains 128 bit/cycle = 32 Gb/s at 250 MHz.
localparam integer N_AES_ENGINES = 2;

wire aes_on  = aes_enable;
wire aes_rst = ~aresetn | ~aes_enable;

// ---- TX: encrypt -------------------------------------------------------
wire                    enc_in_tvalid = axis_host_recv[0].tvalid & aes_on;
wire                    enc_in_tready;
wire [AXI_DATA_BITS-1:0] enc_out_tdata;
wire [KEEP_W-1:0]       enc_out_tkeep;
wire                    enc_out_tvalid;
wire                    enc_out_tlast;
wire                    enc_out_tready = axis_rreq_send[0].tready & aes_on;

aes_gcm_bank #(
    .AXI_DATA_WIDTH  (AXI_DATA_BITS),
    .KEEP_WIDTH      (KEEP_W),
    .NUM_AES_ENGINES (N_AES_ENGINES),
    .DIRECTION       (0)
) inst_aes_tx (
    .clk            (aclk),
    .rst            (aes_rst),
    .iv_dir         (aes_role),
    .s_axis_tdata   (axis_host_recv[0].tdata),
    .s_axis_tkeep   (axis_host_recv[0].tkeep),
    .s_axis_tvalid  (enc_in_tvalid),
    .s_axis_tready  (enc_in_tready),
    .s_axis_tlast   (axis_host_recv[0].tlast),
    .m_axis_tdata   (enc_out_tdata),
    .m_axis_tkeep   (enc_out_tkeep),
    .m_axis_tvalid  (enc_out_tvalid),
    .m_axis_tready  (enc_out_tready),
    .m_axis_tlast   (enc_out_tlast),
    .tag_ok_cnt     (),
    .tag_quarantine ()
);

assign axis_host_recv[0].tready = aes_on ? enc_in_tready : axis_rreq_send[0].tready;

assign axis_rreq_send[0].tdata  = aes_on ? enc_out_tdata  : axis_host_recv[0].tdata;
assign axis_rreq_send[0].tkeep  = aes_on ? enc_out_tkeep  : axis_host_recv[0].tkeep;
assign axis_rreq_send[0].tlast  = aes_on ? enc_out_tlast  : axis_host_recv[0].tlast;
assign axis_rreq_send[0].tvalid = aes_on ? enc_out_tvalid : axis_host_recv[0].tvalid;
assign axis_rreq_send[0].tid    = '0;   // dropped by the shell on this path

// ---- RX: decrypt -------------------------------------------------------
wire                    dec_in_tvalid = axis_rrsp_recv[0].tvalid & aes_on;
wire                    dec_in_tready;
wire [AXI_DATA_BITS-1:0] dec_out_tdata;
wire [KEEP_W-1:0]       dec_out_tkeep;
wire                    dec_out_tvalid;
wire                    dec_out_tlast;
wire                    dec_out_tready = axis_host_send[1].tready & aes_on;
wire [$clog2(N_AES_ENGINES+1)-1:0] dec_tag_ok_cnt;
wire                    dec_quarantine;

aes_gcm_bank #(
    .AXI_DATA_WIDTH  (AXI_DATA_BITS),
    .KEEP_WIDTH      (KEEP_W),
    .NUM_AES_ENGINES (N_AES_ENGINES),
    .DIRECTION       (1)
) inst_aes_rx (
    .clk            (aclk),
    .rst            (aes_rst),
    .iv_dir         (~aes_role),
    .s_axis_tdata   (axis_rrsp_recv[0].tdata),
    .s_axis_tkeep   (axis_rrsp_recv[0].tkeep),
    .s_axis_tvalid  (dec_in_tvalid),
    .s_axis_tready  (dec_in_tready),
    .s_axis_tlast   (axis_rrsp_recv[0].tlast),
    .m_axis_tdata   (dec_out_tdata),
    .m_axis_tkeep   (dec_out_tkeep),
    .m_axis_tvalid  (dec_out_tvalid),
    .m_axis_tready  (dec_out_tready),
    .m_axis_tlast   (dec_out_tlast),
    .tag_ok_cnt     (dec_tag_ok_cnt),
    .tag_quarantine (dec_quarantine)
);

assign axis_rrsp_recv[0].tready = aes_on ? dec_in_tready : axis_host_send[1].tready;

assign axis_host_send[1].tdata  = aes_on ? dec_out_tdata  : axis_rrsp_recv[0].tdata;
assign axis_host_send[1].tkeep  = aes_on ? dec_out_tkeep  : axis_rrsp_recv[0].tkeep;
assign axis_host_send[1].tlast  = aes_on ? dec_out_tlast  : axis_rrsp_recv[0].tlast;
assign axis_host_send[1].tvalid = aes_on ? dec_out_tvalid : axis_rrsp_recv[0].tvalid;
assign axis_host_send[1].tid    = '0;   // dropped by the shell on this path

// ---- RDMA READ streams: untouched pass-through -------------------------
`AXISR_ASSIGN(axis_rreq_recv[0], axis_host_send[0])
`AXISR_ASSIGN(axis_host_recv[1], axis_rrsp_send[0])

// =========================================================================
// Status counters and the receive-side cycle stamps
//
// Pure observers of the handshakes; they drive nothing on the datapath.
// Everything holds at zero while MEAS_CTRL.arm is low, so writing 0 then 1
// gives a clean reset before each measurement.
// =========================================================================
logic [47:0] cycle_cnt;
always_ff @(posedge aclk) begin
    if (!aresetn) cycle_cnt <= '0;
    else          cycle_cnt <= cycle_cnt + 1;
end

logic first_seen;

wire rx_beat  = axis_rrsp_recv[0].tvalid && axis_rrsp_recv[0].tready;
wire rx_frame = axis_host_send[1].tvalid && axis_host_send[1].tready && axis_host_send[1].tlast;
wire tx_frame = axis_rreq_send[0].tvalid && axis_rreq_send[0].tready && axis_rreq_send[0].tlast;

always_ff @(posedge aclk) begin
    if (!aresetn || !meas_armed) begin
        first_seen     <= 1'b0;
        first_cycle    <= '0;
        last_cycle     <= '0;
        tag_ok_count   <= '0;
        tx_frame_count <= '0;
        rx_frame_count <= '0;
        tag_seen       <= 1'b0;
    end else begin
        if (rx_beat && !first_seen) begin
            first_cycle <= cycle_cnt;
            first_seen  <= 1'b1;
        end
        if (rx_frame) begin
            last_cycle     <= cycle_cnt;
            rx_frame_count <= rx_frame_count + 1;
        end
        if (tx_frame)   tx_frame_count <= tx_frame_count + 1;
        if (dec_tag_ok_cnt != '0) begin
            tag_ok_count <= tag_ok_count + dec_tag_ok_cnt;
            tag_seen     <= 1'b1;
        end
    end
end

assign rx_quarantine = dec_quarantine;

// =========================================================================
// Tie off unused interfaces
// =========================================================================
always_comb notify.tie_off_m();
always_comb cq_rd.tie_off_s();
always_comb cq_wr.tie_off_s();

// =========================================================================
// Debug
// =========================================================================
ila_perf_rdma_aes inst_ila_perf_rdma_aes (
    .clk(aclk),
    .probe0 (axis_host_recv[0].tvalid),
    .probe1 (axis_host_recv[0].tready),
    .probe2 (axis_host_recv[0].tlast),
    .probe3 (axis_rreq_send[0].tvalid),
    .probe4 (axis_rreq_send[0].tready),
    .probe5 (axis_rreq_send[0].tlast),
    .probe6 (axis_rrsp_recv[0].tvalid),
    .probe7 (axis_rrsp_recv[0].tready),
    .probe8 (axis_rrsp_recv[0].tlast),
    .probe9 (axis_host_send[1].tvalid),
    .probe10(axis_host_send[1].tready),
    .probe11(axis_host_send[1].tlast),
    .probe12(aes_on),
    .probe13(aes_role),
    .probe14(|dec_tag_ok_cnt),
    .probe15(dec_quarantine)
);
