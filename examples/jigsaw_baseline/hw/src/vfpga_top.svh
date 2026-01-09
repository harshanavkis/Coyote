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

logic dma_start;
logic dma_direction;
logic [VADDR_BITS-1:0] dma_src_addr;
logic [VADDR_BITS-1:0] dma_dst_addr;
logic [LEN_BITS-1:0] dma_len;
logic [PID_BITS-1:0] coyote_pid;
logic dma_status;
logic dma_status_valid;
logic computation_status;
logic computation_status_valid;
logic clear_dma_start;

logic coyote_dma_d2h;
logic [VADDR_BITS-1:0] coyote_dma_addr;
logic [LEN_BITS-1:0] coyote_dma_len;
logic coyote_dma_req;
logic coyote_dma_tx_valid;
logic [LEN_BITS-1:0] coyote_dma_tx_len;

jigsaw_baseline_axi_ctrl_parser inst_axi_ctrl_parser (
    .aclk(aclk),
    .aresetn(aresetn),
    .axi_ctrl(axi_ctrl),
    .dma_start(dma_start),
    .dma_direction(dma_direction),
    .dma_src_addr(dma_src_addr),
    .dma_dst_addr(dma_dst_addr),
    .dma_len(dma_len),
    .coyote_pid(coyote_pid),
    .dma_status(dma_status),
    .dma_status_valid(dma_status_valid),
    .computation_status(computation_status),
    .computation_status_valid(computation_status_valid),
    .clear_dma_start(clear_dma_start),
    .coyote_dma_tx_len_valid(coyote_dma_tx_valid),
    .coyote_dma_tx_len(coyote_dma_tx_len)
);

dma_engine inst_dma_engine (
    .aclk(aclk),
    .aresetn(aresetn),
    .dma_in_tdata(axis_in_int.tdata),
    .dma_in_tkeep(axis_in_int.tkeep),
    .dma_in_tvalid(axis_in_int.tvalid),
    .dma_in_tready(axis_in_int.tready),
    .dma_in_tlast(axis_in_int.tlast),
    .dma_out_tdata(axis_out_int.tdata),
    .dma_out_tkeep(axis_out_int.tkeep),
    .dma_out_tvalid(axis_out_int.tvalid),
    .dma_out_tready(axis_out_int.tready),
    .dma_out_tlast(axis_out_int.tlast),
    .dma_start(dma_start),
    .dma_direction(dma_direction),
    .dma_src_addr(dma_src_addr),
    .dma_dst_addr(dma_dst_addr),
    .dma_len(dma_len),
    .dma_status(dma_status),
    .dma_status_valid(dma_status_valid),
    .clear_dma_start(clear_dma_start),
    .coyote_dma_d2h(coyote_dma_d2h),
    .coyote_dma_addr(coyote_dma_addr),
    .coyote_dma_len(coyote_dma_len),
    .coyote_dma_req(coyote_dma_req),
    .coyote_dma_tx_len_valid(coyote_dma_tx_valid),
    .coyote_dma_tx_len(coyote_dma_tx_len)
);

always_comb begin
    ///////////////////////////////
    //          READS           //
    /////////////////////////////
    // Requests
    sq_rd.data = 0;
    sq_rd.data.last = coyote_dma_req && !coyote_dma_d2h; // Only a single request
    sq_rd.data.pid = coyote_pid;
    sq_rd.data.len = coyote_dma_len;
    sq_rd.data.vaddr = coyote_dma_addr;
    sq_rd.data.strm = STRM_HOST;
    sq_rd.data.opcode = LOCAL_READ;
    sq_rd.valid = coyote_dma_req && !coyote_dma_d2h;

    cq_rd.ready = 1'b1;

    ///////////////////////////////
    //          WRITES          //
    /////////////////////////////
    // Requests
    sq_wr.data = 0;
    sq_wr.data.last = coyote_dma_req && coyote_dma_d2h; // Only a single request
    sq_wr.data.pid = coyote_pid;
    sq_wr.data.len = coyote_dma_len;
    sq_wr.data.vaddr = coyote_dma_addr;
    sq_wr.data.strm = STRM_HOST;
    sq_wr.data.opcode = LOCAL_WRITE;
    sq_wr.valid = coyote_dma_req && coyote_dma_d2h;

    cq_wr.ready = 1'b1;
end

// Tie off unused interfaces
always_comb notify.tie_off_m();
