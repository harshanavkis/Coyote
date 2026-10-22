import lynxTypes::*;

/**
 * stamp_csr
 *
 * AXI4-Lite CSR page for stamp_rec in the perf_rdma vFPGA, at the same word
 * indices as loom_switch's loom_ctrl, so one tool reads both:
 *   224 ST_CTL  (W) arm: clear the counts, restart the cycle count, record
 *               from now on; (R) {DEPTH [63:32], N_ST [31:0]}
 *   225 ST_SEL  (RW) [19:16] point, [15:0] ring index
 *   226 ST_DATA (RO) the selected stamp (cycles since the arm)
 *   227 ST_NOW  (RO) cycles since the arm
 *   232+p       (RO) events at point p since the arm
 * Every other word reads 0.
 */
module stamp_csr #(
    parameter integer N_ST     = 9,
    parameter integer ST_DEPTH = 4096
) (
    input  logic                        aclk,
    input  logic                        aresetn,

    AXI4L.s                             axi_ctrl,

    output logic                        st_arm,
    output logic [3:0]                  st_pt,
    output logic [$clog2(ST_DEPTH)-1:0] st_idx,
    input  logic [31:0]                 st_data,
    input  logic [N_ST-1:0][31:0]       st_cnt,
    input  logic [31:0]                 st_now
);

localparam integer ADDR_LSB = $clog2(AXIL_DATA_BITS/8);   // 3
localparam integer CSR_BITS = 9;
localparam integer R_ST_CTL      = 224;
localparam integer R_ST_SEL      = 225;
localparam integer R_ST_DATA     = 226;
localparam integer R_ST_NOW      = 227;
localparam integer R_ST_CNT_BASE = 232;

logic [15:0] axi_awaddr, axi_araddr;
logic        axi_awready, axi_arready, axi_wready, axi_bvalid, axi_rvalid, aw_en;
logic [1:0]  axi_bresp, axi_rresp;
logic [AXIL_DATA_BITS-1:0] axi_rdata;

wire ctrl_reg_wren = axi_wready && axi_ctrl.wvalid && axi_awready && axi_ctrl.awvalid;
wire ctrl_reg_rden = axi_arready && axi_ctrl.arvalid && ~axi_rvalid;
wire [CSR_BITS-1:0] wr_idx = axi_awaddr[ADDR_LSB +: CSR_BITS];
wire [CSR_BITS-1:0] rd_idx = axi_araddr[ADDR_LSB +: CSR_BITS];
wire csr_wr = ctrl_reg_wren && (axi_awaddr[15:12] == 4'd0) && (&axi_ctrl.wstrb);
wire csr_rd = (axi_araddr[15:12] == 4'd0);

logic [63:0] r_st_sel;
always_ff @(posedge aclk) begin
    if (!aresetn) r_st_sel <= 64'd0;
    else if (csr_wr && (wr_idx == R_ST_SEL)) r_st_sel <= axi_ctrl.wdata;
end

assign st_arm = csr_wr && (wr_idx == R_ST_CTL);
assign st_pt  = r_st_sel[19:16];
assign st_idx = r_st_sel[$clog2(ST_DEPTH)-1:0];

always_ff @(posedge aclk) begin
    if (!aresetn) axi_rdata <= 0;
    else if (ctrl_reg_rden) begin
        axi_rdata <= 0;
        if (csr_rd) case (rd_idx)
            R_ST_CTL:  axi_rdata <= {32'(ST_DEPTH), 32'(N_ST)};
            R_ST_SEL:  axi_rdata <= r_st_sel;
            R_ST_DATA: axi_rdata <= {32'b0, st_data};
            R_ST_NOW:  axi_rdata <= {32'b0, st_now};
            default:
                if (rd_idx >= R_ST_CNT_BASE && rd_idx < R_ST_CNT_BASE + N_ST)
                    axi_rdata <= {32'b0, st_cnt[rd_idx - R_ST_CNT_BASE]};
        endcase
    end
end

// Standard AXI4-Lite control (Coyote boilerplate, as loom_ctrl)
assign axi_ctrl.awready = axi_awready;
assign axi_ctrl.arready = axi_arready;
assign axi_ctrl.bresp   = axi_bresp;
assign axi_ctrl.bvalid  = axi_bvalid;
assign axi_ctrl.wready  = axi_wready;
assign axi_ctrl.rdata   = axi_rdata;
assign axi_ctrl.rresp   = axi_rresp;
assign axi_ctrl.rvalid  = axi_rvalid;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_awready <= 1'b0; axi_awaddr <= 0; aw_en <= 1'b1;
    end else begin
        if (~axi_awready && axi_ctrl.awvalid && axi_ctrl.wvalid && aw_en) begin
            axi_awready <= 1'b1; aw_en <= 1'b0;
            axi_awaddr  <= axi_ctrl.awaddr[15:0];
        end else if (axi_ctrl.bready && axi_bvalid) begin
            aw_en <= 1'b1; axi_awready <= 1'b0;
        end else begin
            axi_awready <= 1'b0;
        end
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_arready <= 1'b0; axi_araddr <= 0;
    end else begin
        if (~axi_arready && axi_ctrl.arvalid) begin
            axi_arready <= 1'b1; axi_araddr <= axi_ctrl.araddr[15:0];
        end else begin
            axi_arready <= 1'b0;
        end
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_bvalid <= 0; axi_bresp <= 2'b0;
    end else begin
        if (axi_awready && axi_ctrl.awvalid && ~axi_bvalid && axi_wready && axi_ctrl.wvalid) begin
            axi_bvalid <= 1'b1; axi_bresp <= 2'b0;
        end else if (axi_ctrl.bready && axi_bvalid) begin
            axi_bvalid <= 1'b0;
        end
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_wready <= 1'b0;
    end else begin
        if (~axi_wready && axi_ctrl.wvalid && axi_ctrl.awvalid && aw_en)
            axi_wready <= 1'b1;
        else
            axi_wready <= 1'b0;
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_rvalid <= 0; axi_rresp <= 0;
    end else begin
        if (axi_arready && axi_ctrl.arvalid && ~axi_rvalid) begin
            axi_rvalid <= 1'b1; axi_rresp <= 2'b0;
        end else if (axi_rvalid && axi_ctrl.rready) begin
            axi_rvalid <= 1'b0;
        end
    end
end

endmodule
