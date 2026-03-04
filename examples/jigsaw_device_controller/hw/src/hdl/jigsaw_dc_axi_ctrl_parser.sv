import lynxTypes::*;

/**
 * jigsaw_dc_axi_ctrl_parser
 * @brief Minimal AXI Lite register parser for the device controller.
 *        Only provides RDMA config registers (no MMIO control needed on device side).
 *
 * @param[in] aclk Clock signal
 * @param[in] aresetn Active low reset signal
 * @param[in/out] axi_ctrl AXI Lite Control signal
 * @param[out] coyote_pid: PID of the Coyote process
 */
module jigsaw_dc_axi_ctrl_parser (
  input  logic                        aclk,
  input  logic                        aresetn,

  AXI4L.s                             axi_ctrl,

  output logic [PID_BITS - 1:0] coyote_pid,
  output logic [VADDR_BITS-1:0] remote_vaddr
);

/////////////////////////////////////
//          CONSTANTS             //
///////////////////////////////////
localparam integer N_REGS = 2; // minimum 2 to keep $clog2 >= 1 (reg 1 unused)
localparam integer ADDR_MSB = $clog2(N_REGS);
localparam integer ADDR_LSB = $clog2(AXIL_DATA_BITS/8);
localparam integer AXI_ADDR_BITS = ADDR_LSB + ADDR_MSB;

/////////////////////////////////////
//          REGISTERS             //
///////////////////////////////////
logic [AXI_ADDR_BITS-1:0] axi_awaddr;
logic axi_awready;
logic [AXI_ADDR_BITS-1:0] axi_araddr;
logic axi_arready;
logic [1:0] axi_bresp;
logic axi_bvalid;
logic axi_wready;
logic [AXIL_DATA_BITS-1:0] axi_rdata;
logic [1:0] axi_rresp;
logic axi_rvalid;
logic aw_en;

logic [N_REGS-1:0][AXIL_DATA_BITS-1:0] ctrl_reg;
logic ctrl_reg_rden;
logic ctrl_reg_wren;

// Register map:
// 0 (RW) - Coyote PID
localparam COYOTE_PID_REG = 0;
// 1 (RW) - Remote virtual address for RDMA WRITE
localparam REMOTE_VADDR_REG = 1;

/////////////////////////////////////
//         WRITE PROCESS          //
///////////////////////////////////
assign ctrl_reg_wren = axi_wready && axi_ctrl.wvalid && axi_awready && axi_ctrl.awvalid;

always_ff @(posedge aclk) begin
  if (aresetn == 1'b0) begin
    ctrl_reg <= 0;
  end
  else begin
    if(ctrl_reg_wren) begin
      case (axi_awaddr[ADDR_LSB+:ADDR_MSB])
        COYOTE_PID_REG:
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[COYOTE_PID_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        REMOTE_VADDR_REG:
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[REMOTE_VADDR_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        default: ;
      endcase
    end
  end
end

/////////////////////////////////////
//         READ PROCESS           //
///////////////////////////////////
assign ctrl_reg_rden = axi_arready & axi_ctrl.arvalid & ~axi_rvalid;

always_ff @(posedge aclk) begin
  if(aresetn == 1'b0) begin
    axi_rdata <= 0;
  end
  else begin
    if(ctrl_reg_rden) begin
      axi_rdata <= 0;
      case (axi_araddr[ADDR_LSB+:ADDR_MSB])
        COYOTE_PID_REG:
          axi_rdata <= ctrl_reg[COYOTE_PID_REG];
        REMOTE_VADDR_REG:
          axi_rdata <= ctrl_reg[REMOTE_VADDR_REG];
        default: ;
      endcase
    end
  end
end

/////////////////////////////////////
//       OUTPUT ASSIGNMENT        //
///////////////////////////////////
always_comb begin
  coyote_pid = ctrl_reg[COYOTE_PID_REG];
  remote_vaddr = ctrl_reg[REMOTE_VADDR_REG];
end

/////////////////////////////////////
//     STANDARD AXI CONTROL       //
///////////////////////////////////

assign axi_ctrl.awready = axi_awready;
assign axi_ctrl.arready = axi_arready;
assign axi_ctrl.bresp = axi_bresp;
assign axi_ctrl.bvalid = axi_bvalid;
assign axi_ctrl.wready = axi_wready;
assign axi_ctrl.rdata = axi_rdata;
assign axi_ctrl.rresp = axi_rresp;
assign axi_ctrl.rvalid = axi_rvalid;

always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_awready <= 1'b0;
      axi_awaddr <= 0;
      aw_en <= 1'b1;
    end
  else
    begin
      if (~axi_awready && axi_ctrl.awvalid && axi_ctrl.wvalid && aw_en)
        begin
          axi_awready <= 1'b1;
          aw_en <= 1'b0;
          axi_awaddr <= axi_ctrl.awaddr;
        end
      else if (axi_ctrl.bready && axi_bvalid)
        begin
          aw_en <= 1'b1;
          axi_awready <= 1'b0;
        end
      else
        begin
          axi_awready <= 1'b0;
        end
    end
end

always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_arready <= 1'b0;
      axi_araddr  <= 0;
    end
  else
    begin
      if (~axi_arready && axi_ctrl.arvalid)
        begin
          axi_arready <= 1'b1;
          axi_araddr  <= axi_ctrl.araddr;
        end
      else
        begin
          axi_arready <= 1'b0;
        end
    end
end

always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_bvalid  <= 0;
      axi_bresp   <= 2'b0;
    end
  else
    begin
      if (axi_awready && axi_ctrl.awvalid && ~axi_bvalid && axi_wready && axi_ctrl.wvalid)
        begin
          axi_bvalid <= 1'b1;
          axi_bresp  <= 2'b0;
        end
      else
        begin
          if (axi_ctrl.bready && axi_bvalid)
            begin
              axi_bvalid <= 1'b0;
            end
        end
    end
end

always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_wready <= 1'b0;
    end
  else
    begin
      if (~axi_wready && axi_ctrl.wvalid && axi_ctrl.awvalid && aw_en )
        begin
          axi_wready <= 1'b1;
        end
      else
        begin
          axi_wready <= 1'b0;
        end
    end
end

always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_rvalid <= 0;
      axi_rresp  <= 0;
    end
  else
    begin
      if (axi_arready && axi_ctrl.arvalid && ~axi_rvalid)
        begin
          axi_rvalid <= 1'b1;
          axi_rresp  <= 2'b0;
        end
      else if (axi_rvalid && axi_ctrl.rready)
        begin
          axi_rvalid <= 1'b0;
        end
    end
end

endmodule
