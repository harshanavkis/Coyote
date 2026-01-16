/**
 * This file is part of the Coyote <https://github.com/fpgasystems/Coyote>
 *
 * MIT Licence
 * Copyright (c) 2025, Systems Group, ETH Zurich
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Simple pipeline stages, buffering the input/output signals (not really needed, but nice to have for easier timing closure)
AXI4SR axis_in_int (.*);
axisr_reg inst_reg_in  (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_host_recv[0]), .m_axis(axis_in_int));

AXI4SR axis_out_int (.*);
axisr_reg inst_reg_out (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_out_int), .m_axis(axis_host_send[0]));

// Control signals
(* mark_debug = "true" *) logic [VADDR_BITS-1:0] mmio_vaddr;
(* mark_debug = "true" *) logic mmio_ctrl;
(* mark_debug = "true" *) logic mmio_clear;
(* mark_debug = "true" *) logic mmio_write_done;
(* mark_debug = "true" *) logic mmio_read_done;
(* mark_debug = "true" *) logic [PID_BITS-1:0] coyote_pid;

// SQ signals
(* mark_debug = "true" *) logic sq_valid_write;
(* mark_debug = "true" *) logic sq_dir_write;
(* mark_debug = "true" *) logic [VADDR_BITS-1:0] sq_addr_write;
(* mark_debug = "true" *) logic [LEN_BITS-1:0] sq_len_write;
(* mark_debug = "true" *) logic sq_valid_read;
(* mark_debug = "true" *) logic sq_dir_read;
(* mark_debug = "true" *) logic [VADDR_BITS-1:0] sq_addr_read;
(* mark_debug = "true" *) logic [LEN_BITS-1:0] sq_len_read;

// Interface signals between controller and txn_generator
(* mark_debug = "true" *) logic [AXI_DATA_BITS-1:0] net_out_tdata;
(* mark_debug = "true" *) logic [AXI_DATA_BITS-1:0] net_in_tdata;
(* mark_debug = "true" *) logic [AXI_DATA_BITS/8-1:0] net_out_tkeep;
(* mark_debug = "true" *) logic [AXI_DATA_BITS/8-1:0] net_in_tkeep;
(* mark_debug = "true" *) logic net_out_tvalid;
(* mark_debug = "true" *) logic net_out_tready;
(* mark_debug = "true" *) logic net_out_tlast;
(* mark_debug = "true" *) logic net_out_tuser;
(* mark_debug = "true" *) logic net_in_tvalid;
(* mark_debug = "true" *) logic net_in_tready;
(* mark_debug = "true" *) logic net_in_tlast;
(* mark_debug = "true" *) logic net_in_tuser;

jigsaw_minus_nw_axi_ctrl_parser inst_axi_ctrl_parser (
    .aclk(aclk),
    .aresetn(aresetn),
    .axi_ctrl(axi_ctrl),
    .mmio_vaddr(mmio_vaddr),
    .mmio_ctrl(mmio_ctrl),
    .mmio_clear(mmio_clear),
    .mmio_write_done(mmio_write_done),
    .mmio_read_done(mmio_read_done),
    .coyote_pid(coyote_pid)
);

jigsaw_host_controller #(
    .ADDR_WIDTH(64),
    .LEN_WIDTH(64)
) inst_host_controller (
    .aclk(aclk),
    .aresetn(aresetn),
    
    // Network side
    .network_in_tdata(net_in_tdata),
    .network_in_tkeep(net_in_tkeep),
    .network_in_tvalid(net_in_tvalid),
    .network_in_tready(net_in_tready),
    .network_in_tlast(net_in_tlast),
    .network_in_tuser(net_in_tuser),
    
    .network_out_tdata(net_out_tdata),
    .network_out_tkeep(net_out_tkeep),
    .network_out_tvalid(net_out_tvalid),
    .network_out_tready(net_out_tready),
    .network_out_tlast(net_out_tlast),
    .network_out_tuser(net_out_tuser),
    
    // Host side (AXI4SR axis_in_int/axis_out_int)
    .host_in_tdata(axis_in_int.tdata),
    .host_in_tkeep(axis_in_int.tkeep),
    .host_in_tvalid(axis_in_int.tvalid),
    .host_in_tready(axis_in_int.tready),
    .host_in_tlast(axis_in_int.tlast),
    .host_in_tuser(1'b0), // AXI4SR doesn't have tuser
    
    .host_out_tdata(axis_out_int.tdata),
    .host_out_tkeep(axis_out_int.tkeep),
    .host_out_tvalid(axis_out_int.tvalid),
    .host_out_tready(axis_out_int.tready),
    .host_out_tlast(axis_out_int.tlast),
    .host_out_tuser(), // AXI4SR doesn't have tuser
    
    // Submission queue interfaces
    .sq_valid_write(sq_valid_write),
    .sq_dir_write(sq_dir_write),
    .sq_addr_write(sq_addr_write),
    .sq_len_write(sq_len_write),
    
    .sq_valid_read(sq_valid_read),
    .sq_dir_read(sq_dir_read),
    .sq_addr_read(sq_addr_read),
    .sq_len_read(sq_len_read),
    
    // MMIO specific control signals
    .mmio_vaddr(mmio_vaddr),
    .mmio_ctrl(mmio_ctrl),
    .mmio_clear(mmio_clear),
    .mmio_write_done(mmio_write_done),
    .mmio_read_done(mmio_read_done)
);

txn_generator inst_txn_generator (
    .aclk(aclk),
    .aresetn(aresetn),
    
    .txn_generator_in_tdata(net_out_tdata),
    .txn_generator_in_tkeep(net_out_tkeep),
    .txn_generator_in_tvalid(net_out_tvalid),
    .txn_generator_in_tready(net_out_tready),
    .txn_generator_in_tlast(net_out_tlast),
    .txn_generator_in_tuser(net_out_tuser),
    
    .txn_generator_out_tdata(net_in_tdata),
    .txn_generator_out_tkeep(net_in_tkeep),
    .txn_generator_out_tvalid(net_in_tvalid),
    .txn_generator_out_tready(net_in_tready),
    .txn_generator_out_tlast(net_in_tlast),
    .txn_generator_out_tuser(net_in_tuser)
);

always_comb begin
    ///////////////////////////////
    //          READS           //
    /////////////////////////////
    // Requests
    sq_rd.data = 0;
    sq_rd.data.last = 1'b1; // Each request from jigsaw host controller is a single request
    sq_rd.data.pid = coyote_pid;
    sq_rd.data.len = sq_len_read;
    sq_rd.data.vaddr = sq_addr_read;
    sq_rd.data.strm = STRM_HOST;
    sq_rd.data.opcode = LOCAL_READ;
    sq_rd.valid = sq_valid_read;

    cq_rd.ready = 1'b1;

    ///////////////////////////////
    //          WRITES          //
    /////////////////////////////
    // Requests
    sq_wr.data = 0;
    sq_wr.data.last = 1'b1; // Each request from jigsaw host controller is a single request
    sq_wr.data.pid = coyote_pid;
    sq_wr.data.len = sq_len_write;
    sq_wr.data.vaddr = sq_addr_write;
    sq_wr.data.strm = STRM_HOST;
    sq_wr.data.opcode = LOCAL_WRITE;
    sq_wr.valid = sq_valid_write;

    cq_wr.ready = 1'b1;
end

// Tie off unused interfaces
always_comb notify.tie_off_m();

// ILA for debugging
// Probes key signals from top-level and submodules via hierarchical paths
// ila_jigsaw inst_ila_jigsaw (
//     .clk(aclk),
    
//     // === Top-level control signals ===
//     .probe0(mmio_vaddr),                // 48 bits (VADDR_BITS)
//     .probe1(mmio_ctrl),                 // 1 bit
//     .probe2(mmio_write_done),           // 1 bit
//     .probe3(mmio_read_done),            // 1 bit
    
//     // === SQ signals ===
//     .probe4(sq_addr_write),             // 48 bits (VADDR_BITS)
//     .probe5(sq_len_write),              // 28 bits (LEN_BITS)
//     .probe6(sq_len_read),               // 28 bits (LEN_BITS)
//     .probe7(sq_valid_write),            // 1 bit
//     .probe8(sq_valid_read),             // 1 bit
    
//     // === txn_generator signals (hierarchical) ===
//     .probe9(inst_txn_generator.mmio_read_valid),           // 1 bit
//     .probe10(inst_txn_generator.mmio_read_data),           // 72 bits
//     .probe11(inst_txn_generator.dma_src_addr),             // 64 bits
//     .probe12(inst_txn_generator.dma_dst_addr),             // 64 bits
//     .probe13(inst_txn_generator.dma_len),                  // 64 bits
//     .probe14(inst_txn_generator.dma_start),                // 1 bit
//     .probe15(inst_txn_generator.dma_status),               // 1 bit
//     .probe16(inst_txn_generator.dma_status_valid),         // 1 bit
//     .probe17(inst_txn_generator.dma_output_active),        // 1 bit
    
//     // === State machines (hierarchical) ===
//     .probe18(inst_txn_generator.payload_to_dma.state),     // 3 bits - payload_to_dma FSM
//     .probe19(inst_host_controller.mmio_state_cur),         // 2 bits - MMIO FSM
//     .probe20(inst_host_controller.dma_rd_state_cur),       // 2 bits - DMA read FSM
//     .probe21(inst_host_controller.dma_wr_state_cur),       // 2 bits - DMA write FSM
//     .probe22(inst_txn_generator.state),                    // 1 bit - txn_generator FSM
//     .probe23(sq_rd.ready),                                 // 1 bit
//     .probe24(sq_wr.ready),                                 // 1 bit
    
//     // === Network / Host Interface signals ===
//     .probe25(net_out_tvalid),                              // 1 bit (Host->FPGA Req)
//     .probe26(net_out_tready),                              // 1 bit
//     .probe27(net_out_tlast),                               // 1 bit
//     .probe28(net_in_tvalid),                               // 1 bit (FPGA->Host Data)
//     .probe29(net_in_tready),                               // 1 bit
//     .probe30(net_in_tlast),                                // 1 bit
//     .probe31(axis_in_int.tvalid),                          // 1 bit (Host Controller In)
//     .probe32(axis_in_int.tready),                          // 1 bit
//     .probe33(axis_in_int.tlast),                           // 1 bit
//     .probe34(axis_out_int.tvalid),                         // 1 bit (Host Controller Out)
//     .probe35(axis_out_int.tready),                         // 1 bit
//     .probe36(axis_out_int.tlast),                          // 1 bit
    
//     // === Detailed DMA / FIFO signals ===
//     .probe37(inst_txn_generator.payload_to_dma.dma_d2h_count), // 64 bits
//     .probe38(inst_txn_generator.dma_fifo_out_tvalid),      // 1 bit
//     .probe39(inst_txn_generator.dma_fifo_out_tready),      // 1 bit
//     .probe40(inst_txn_generator.payload_to_dma_out_tready), // 1 bit - Key signal for DMA completion
    
//     // === payload_to_dma next state (for debugging state transitions) ===
//     .probe41(inst_txn_generator.payload_to_dma.next_state),  // 3 bits - payload_to_dma next FSM state
    
//     // === Additional debug signals for second-run MMIO failure ===
//     .probe42(inst_txn_generator.clear_dma_start),            // 1 bit - DMA command register clear
//     .probe43(inst_txn_generator.payload_to_mmio.payload_valid), // 1 bit - MMIO commands arriving
//     .probe44(inst_txn_generator.payload_to_mmio.read_data_pending_valid), // 1 bit - MMIO reads backing up
//     .probe45(inst_host_controller.mmio_ctrl_reg)            // 1 bit - Registered mmio_ctrl (gates network_in)
// );
