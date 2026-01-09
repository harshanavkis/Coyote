import lynxTypes::*;

/**
 * jigsaw_baseline_axi_ctrl_parser
 * @brief Reads from/wites to the AXI Lite stream containing the benchmark data
 * 
 * @param[in] aclk Clock signal
 * @param[in] aresetn Active low reset signal
 
 * @param[in/out] axi_ctrl AXI Lite Control signal, from/to the host via PCIe and XDMA

 * @param[out] dma_start: start a DMA transfer
 * @param[out] dma_direction: 0 is H2D and 1 is D2H
 * @param[out] dma_src_addr: source address for H2D transfer
 * @param[out] dma_dst_addr: destination address for D2H transfer
 * @param[out] dma_len: length of the DMA transfer in bytes
 * @param[in] dma_status: status of the DMA transfer
 * @param[in] dma_status_valid
 * @param[in] computation_status
 * @param[in] computation_status_valid
 * @param[in] clear_dma_start
 */
module jigsaw_baseline_axi_ctrl_parser (
  input  logic                        aclk,
  input  logic                        aresetn,
  
  AXI4L.s                             axi_ctrl,

  output logic dma_start,
  output logic dma_direction,
  output logic [VADDR_BITS-1:0] dma_src_addr,
  output logic [VADDR_BITS-1:0] dma_dst_addr,
  output logic [LEN_BITS-1:0] dma_len,
  output logic [PID_BITS-1:0] coyote_pid,
  input logic dma_status,
  input logic dma_status_valid,
  input logic computation_status,
  input logic computation_status_valid,
  input logic clear_dma_start,
  input logic coyote_dma_tx_len_valid,
  input logic [63:0] coyote_dma_tx_len
);

/////////////////////////////////////
//          CONSTANTS             //
///////////////////////////////////
localparam integer N_REGS = 9;
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
//  0 (RW) - DMA command register is bitwise OR of the following:
//    bit 0 - Start transfer
//    bit 1 - Direction (0 = H2D, 1 = D2H)
localparam DMA_CMD_REG = 0;
//  1 (RW) - DMA source address
localparam DMA_SRC_ADDR_REG = 1;
//  2 (RW) - DMA destination address
localparam DMA_DST_ADDR_REG = 2;
//  3 (RW) - DMA length
localparam DMA_LEN_REG = 3;
//  4 (RW) - DMA status register: 0th bit for DMA completion, 1st bit for computation completion
localparam DMA_STATUS_REG = 4;
//  5 (RW) - Start computation
localparam START_COMPUTATION_REG = 5;
//  6 (RW) - Cycles per computation
localparam CYCLES_PER_COMPUTATION_REG = 6;
//  7 (RW) - Coyote specific: PID
localparam COYOTE_PID_REG = 7;
//  8 (RW) - Coyote specific: DMA transfer length
localparam COYOTE_DMA_TX_LEN_REG = 8;

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
    // Latch DMA status when valid
    if (dma_status_valid) begin
        ctrl_reg[DMA_STATUS_REG][0] <= dma_status;
    end

    // Latch Coyote DMA transfer length when valid
    if (coyote_dma_tx_len_valid) begin
        ctrl_reg[COYOTE_DMA_TX_LEN_REG] <= coyote_dma_tx_len;
    end

    if (clear_dma_start) begin
      ctrl_reg[DMA_CMD_REG] <= 0;
    end else if(ctrl_reg_wren) begin
      case (axi_awaddr[ADDR_LSB+:ADDR_MSB])
        DMA_CMD_REG:     // DMA command register
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[DMA_CMD_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        DMA_SRC_ADDR_REG:    // DMA source address
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[DMA_SRC_ADDR_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        DMA_DST_ADDR_REG:      // DMA destination address
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[DMA_DST_ADDR_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        DMA_LEN_REG:      // DMA length
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[DMA_LEN_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        DMA_STATUS_REG:   // DMA status register
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[DMA_STATUS_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        START_COMPUTATION_REG:  // Start computation register
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[START_COMPUTATION_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        CYCLES_PER_COMPUTATION_REG:  // Cycles per computation register
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[CYCLES_PER_COMPUTATION_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        COYOTE_PID_REG:  // Coyote specific: PID register
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[COYOTE_PID_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
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
        DMA_CMD_REG:  // DMA command register
          axi_rdata <= ctrl_reg[DMA_CMD_REG];
        DMA_SRC_ADDR_REG:  // DMA source address register
          axi_rdata <= ctrl_reg[DMA_SRC_ADDR_REG];
        DMA_DST_ADDR_REG:  // DMA destination address register
          axi_rdata <= ctrl_reg[DMA_DST_ADDR_REG];
        DMA_LEN_REG:  // DMA length register
          axi_rdata <= ctrl_reg[DMA_LEN_REG];
        DMA_STATUS_REG:   // DMA status register
          axi_rdata <= ctrl_reg[DMA_STATUS_REG];
        START_COMPUTATION_REG:  // Start computation register
          axi_rdata <= ctrl_reg[START_COMPUTATION_REG];
        CYCLES_PER_COMPUTATION_REG:  // Cycles per computation register
          axi_rdata <= ctrl_reg[CYCLES_PER_COMPUTATION_REG];
        COYOTE_PID_REG:  // Coyote specific: PID register
          axi_rdata <= ctrl_reg[COYOTE_PID_REG];
        COYOTE_DMA_TX_LEN_REG:  // Coyote specific: DMA transfer length register
          axi_rdata <= ctrl_reg[COYOTE_DMA_TX_LEN_REG];
        default: ;
      endcase
    end
  end 
end

/////////////////////////////////////
//       OUTPUT ASSIGNMENT        //
///////////////////////////////////
always_comb begin
  dma_start = ctrl_reg[DMA_CMD_REG][0];
  dma_direction = ctrl_reg[DMA_CMD_REG][1];
  dma_src_addr = ctrl_reg[DMA_SRC_ADDR_REG];
  dma_dst_addr = ctrl_reg[DMA_DST_ADDR_REG];
  dma_len = ctrl_reg[DMA_LEN_REG];
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