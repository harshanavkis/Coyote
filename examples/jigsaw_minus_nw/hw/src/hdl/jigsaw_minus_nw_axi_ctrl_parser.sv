import lynxTypes::*;

/**
 * jigsaw_minus_nw_axi_ctrl_parser
 * @brief Reads from/wites to the AXI Lite stream containing the benchmark data
 * 
 * @param[in] aclk Clock signal
 * @param[in] aresetn Active low reset signal
 
 * @param[in/out] axi_ctrl AXI Lite Control signal, from/to the host via PCIe and XDMA

 * @param[out] mmio_vaddr: virtual address of the MMIO
 * @param[out] mmio_ctrl: control signal for MMIO
 * @param[in] mmio_clear: clear signal for MMIO
 * @param[in] mmio_write_done: write done signal for MMIO write request
 * @param[in] mmio_read_done: read done signal for MMIO read response
 * @param[out] coyote_pid: PID of the Coyote process on the host
 */
module jigsaw_minus_nw_axi_ctrl_parser (
  input  logic                        aclk,
  input  logic                        aresetn,
  
  AXI4L.s                             axi_ctrl,

  output logic [VADDR_BITS-1:0] mmio_vaddr,
  output logic mmio_ctrl,
  input logic mmio_clear,
  input logic mmio_write_done,
  input logic mmio_read_done,
  output logic [PID_BITS - 1:0] coyote_pid
);

/////////////////////////////////////
//          CONSTANTS             //
///////////////////////////////////
localparam integer N_REGS = 5;
localparam integer ADDR_MSB = $clog2(N_REGS);
localparam integer ADDR_LSB = $clog2(AXIL_DATA_BITS/8);
localparam integer AXI_ADDR_BITS = ADDR_LSB + ADDR_MSB;

/////////////////////////////////////
//          REGISTERS             //
///////////////////////////////////
// Internal AXI registers
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

// Registers for holding the values read from/to be written to the AXI Lite interface
// These are synchronous but the outputs are combinatorial
logic [N_REGS-1:0][AXIL_DATA_BITS-1:0] ctrl_reg;
logic ctrl_reg_rden;
logic ctrl_reg_wren;

// Register map:
//  0 (RW) - Store beginning address of MMIO virtual address on host:
localparam MMIO_VADDR_REG = 0;
//  1 (RW) - Store control signal for MMIO: it is set by the host and cleared by the jigsaw host-side controller
localparam MMIO_CTRL_REG = 1;
//  2 (RW) - Store write status signal for MMIO: it is set by the jigsaw host-side controller and cleared by the host
localparam MMIO_WRITE_STATUS_REG = 2;
//  3 (RW) - Store read status signal for MMIO: it is set by the jigsaw host-side controller and cleared by the host
localparam MMIO_READ_STATUS_REG = 3;
// 4 (RW) - Store PID for coyote
localparam COYOTE_PID_REG = 4;

/////////////////////////////////////
//         WRITE PROCESS          //
///////////////////////////////////
// Data coming in from host to the vFPGA vie PCIe and XDMA
assign ctrl_reg_wren = axi_wready && axi_ctrl.wvalid && axi_awready && axi_ctrl.awvalid;

always_ff @(posedge aclk) begin
  if (aresetn == 1'b0) begin
    ctrl_reg <= 0;
  end
  else begin
    // Host writes via AXI Lite
    if(ctrl_reg_wren) begin
      case (axi_awaddr[ADDR_LSB+:ADDR_MSB])
        MMIO_VADDR_REG:     // MMIO virtual address register
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[MMIO_VADDR_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        MMIO_CTRL_REG:    // MMIO control register
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[MMIO_CTRL_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        MMIO_WRITE_STATUS_REG:  // MMIO write status register
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[MMIO_WRITE_STATUS_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        MMIO_READ_STATUS_REG:  // MMIO read status register
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[MMIO_READ_STATUS_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        COYOTE_PID_REG:  // Coyote PID register
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[COYOTE_PID_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        default: ;
      endcase
    end

    // Hardware overrides (higher priority than AXI)
    if (mmio_clear) begin
        ctrl_reg[MMIO_CTRL_REG] <= 0;
    end

    if (mmio_write_done) begin
        ctrl_reg[MMIO_WRITE_STATUS_REG] <= 1;
    end

    if (mmio_read_done) begin
      ctrl_reg[MMIO_READ_STATUS_REG] <= 1;
    end
  end
end

/////////////////////////////////////
//         READ PROCESS           //
///////////////////////////////////
// Data going to the host from the vFPGA via XDMA and PCIe
assign ctrl_reg_rden = axi_arready & axi_ctrl.arvalid & ~axi_rvalid;

always_ff @(posedge aclk) begin
  if(aresetn == 1'b0) begin
    axi_rdata <= 0;
  end
  else begin
    if(ctrl_reg_rden) begin
      axi_rdata <= 0;

      case (axi_araddr[ADDR_LSB+:ADDR_MSB])
        MMIO_VADDR_REG:  // MMIO virtual address register
          axi_rdata <= ctrl_reg[MMIO_VADDR_REG];
        MMIO_CTRL_REG:  // MMIO control register
          axi_rdata <= ctrl_reg[MMIO_CTRL_REG];
        MMIO_WRITE_STATUS_REG:  // MMIO write status register
          axi_rdata <= ctrl_reg[MMIO_WRITE_STATUS_REG];
        MMIO_READ_STATUS_REG:  // MMIO read status register
          axi_rdata <= ctrl_reg[MMIO_READ_STATUS_REG];
        COYOTE_PID_REG:   // Coyote PID register
          axi_rdata <= ctrl_reg[COYOTE_PID_REG];
        default: ;
      endcase
    end
  end 
end

/////////////////////////////////////
//       OUTPUT ASSIGNMENT        //
///////////////////////////////////
always_comb begin
  mmio_vaddr = ctrl_reg[MMIO_VADDR_REG];
  mmio_ctrl = ctrl_reg[MMIO_CTRL_REG][0];
  coyote_pid = ctrl_reg[COYOTE_PID_REG];
end

/////////////////////////////////////
//     STANDARD AXI CONTROL       //
///////////////////////////////////
// NOT TO BE EDITED

// I/O
assign axi_ctrl.awready = axi_awready;
assign axi_ctrl.arready = axi_arready;
assign axi_ctrl.bresp = axi_bresp;
assign axi_ctrl.bvalid = axi_bvalid;
assign axi_ctrl.wready = axi_wready;
assign axi_ctrl.rdata = axi_rdata;
assign axi_ctrl.rresp = axi_rresp;
assign axi_ctrl.rvalid = axi_rvalid;

// awready and awaddr
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

// arready and araddr
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

// bvalid and bresp
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

// wready
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

// rvalid and rresp
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