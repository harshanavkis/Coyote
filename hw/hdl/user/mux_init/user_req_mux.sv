/**
 * This file is part of the Coyote <https://github.com/fpgasystems/Coyote>
 *
 * MIT Licence
 * Copyright (c) 2021-2025, Systems Group, ETH Zurich
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

`timescale 1ns / 1ps

import lynxTypes::*;

// QDEPTH (2026-09-27): 0 keeps the structure this module has always had -
// one register in front of a combinational demux, so whichever request is at
// the head of sq_wr owns the port until ITS OWN path takes it. That is
// head-of-line blocking between two independent downstream paths: a remote
// write waiting for the RoCE stack (~84 cycles for a 4 KB packet at 100 GbE
// line rate) stalls a local write bound for the host DMA behind it, and the
// other way round. A vFPGA that transmits and receives at the same time
// therefore serializes the two - measured on Loom as 5.9 GB/s each way
// against 11.8 for one direction alone, and visible in stock perf_rdma too
// (4.1 + 6.3 GB/s).
//
// QDEPTH > 0 gives each path its own queue, so sq_wr.ready means "the path
// THIS request needs has room" and neither path waits for the other. Order
// within each path is preserved, which is all the shell relies on: local
// writes carry their data on axis_host_send[dest] and are sequenced per path
// by dest_req_seq, remote writes carry theirs on axis_rreq_send, and
// completions are told apart by ack_t.remote. Reordering BETWEEN the two
// paths is the point of the change.
module user_req_mux #(
    parameter integer                   ID_REG = 0,
    parameter integer                   QDEPTH = 0
) (
    // USER 
    metaIntf.s                          user_sq_rd,
    metaIntf.s                          user_sq_wr,

    //
    metaIntf.m                          user_local_rd,
    metaIntf.m                          user_local_wr,
`ifdef EN_NET
    metaIntf.m                          user_remote_rd,
    metaIntf.m                          user_remote_wr,
`endif

    input  logic    					aclk,    
	input  logic    					aresetn
);

// Override
metaIntf #(.STYPE(req_t)) user_sq_rd_or (.*);
metaIntf #(.STYPE(req_t)) user_sq_wr_or (.*);

always_comb begin
    user_sq_rd_or.valid = user_sq_rd.valid;
    user_sq_rd.ready = user_sq_rd_or.ready;
    user_sq_rd_or.data = user_sq_rd.data;
    user_sq_rd_or.data.actv = 1'b1;
    user_sq_rd_or.data.host = 1'b0;
    user_sq_rd_or.data.vfid = ID_REG;

    user_sq_wr_or.valid = user_sq_wr.valid;
    user_sq_wr.ready = user_sq_wr_or.ready;
    user_sq_wr_or.data = user_sq_wr.data;
    user_sq_wr_or.data.actv = 1'b1;
    user_sq_wr_or.data.host = 1'b0;
    user_sq_wr_or.data.vfid = ID_REG;
end

// Internal
metaIntf #(.STYPE(req_t)) user_sq_rd_int (.*);
metaIntf #(.STYPE(req_t)) user_sq_wr_int (.*);

metaIntf #(.STYPE(req_t)) user_local_rd_int (.*);
metaIntf #(.STYPE(req_t)) user_local_wr_int (.*);
`ifdef EN_NET
logic remote_strm_1;
logic remote_strm_2;

metaIntf #(.STYPE(dreq_t)) user_remote_rd_int (.*);
metaIntf #(.STYPE(dreq_t)) user_remote_wr_int (.*);
`endif

// Sink reg
meta_reg #(.DATA_BITS($bits(req_t))) inst_reg_sq_rd (.aclk(aclk), .aresetn(aresetn), .s_meta(user_sq_rd_or), .m_meta(user_sq_rd_int));
meta_reg #(.DATA_BITS($bits(req_t))) inst_reg_sq_wr (.aclk(aclk), .aresetn(aresetn), .s_meta(user_sq_wr_or), .m_meta(user_sq_wr_int));

`ifdef EN_NET
if (QDEPTH > 0) begin : g_decoupled
    // Demux by strm straight into a per-path queue: a request is accepted as
    // soon as its own path has room.
    logic q_rd_remote, q_wr_remote;
    metaIntf #(.STYPE(req_t))  q_local_rd_in (.*);
    metaIntf #(.STYPE(req_t))  q_local_wr_in (.*);
    metaIntf #(.STYPE(dreq_t)) q_remote_rd_in (.*);
    metaIntf #(.STYPE(dreq_t)) q_remote_wr_in (.*);

    assign q_rd_remote = ~(is_strm_local(user_sq_rd_int.data.strm));
    assign q_wr_remote = ~(is_strm_local(user_sq_wr_int.data.strm));

    always_comb begin
        q_local_rd_in.valid  = user_sq_rd_int.valid & ~q_rd_remote;
        q_remote_rd_in.valid = user_sq_rd_int.valid &  q_rd_remote;
        user_sq_rd_int.ready = q_rd_remote ? q_remote_rd_in.ready : q_local_rd_in.ready;

        q_local_wr_in.valid  = user_sq_wr_int.valid & ~q_wr_remote;
        q_remote_wr_in.valid = user_sq_wr_int.valid &  q_wr_remote;
        user_sq_wr_int.ready = q_wr_remote ? q_remote_wr_in.ready : q_local_wr_in.ready;

        q_local_rd_in.data = user_sq_rd_int.data;
        q_local_wr_in.data = user_sq_wr_int.data;
        q_remote_rd_in.data.req_1 = user_sq_rd_int.data;
        q_remote_rd_in.data.req_2 = 0;
        q_remote_wr_in.data.req_1 = 0;
        q_remote_wr_in.data.req_2 = user_sq_wr_int.data;
    end

    queue_meta #(.QDEPTH(QDEPTH)) inst_q_local_rd  (.aclk(aclk), .aresetn(aresetn), .s_meta(q_local_rd_in),  .m_meta(user_local_rd));
    queue_meta #(.QDEPTH(QDEPTH)) inst_q_local_wr  (.aclk(aclk), .aresetn(aresetn), .s_meta(q_local_wr_in),  .m_meta(user_local_wr));
    queue_meta #(.QDEPTH(QDEPTH)) inst_q_remote_rd (.aclk(aclk), .aresetn(aresetn), .s_meta(q_remote_rd_in), .m_meta(user_remote_rd));
    queue_meta #(.QDEPTH(QDEPTH)) inst_q_remote_wr (.aclk(aclk), .aresetn(aresetn), .s_meta(q_remote_wr_in), .m_meta(user_remote_wr));

    // The shared-head path's internal interfaces are unused here. They are
    // declared at module scope (the no-EN_NET path needs them), so tie them
    // off: metaIntf asserts that valid and ready are never undefined.
    always_comb begin
        user_local_rd_int.valid  = 1'b0;  user_local_rd_int.data  = '0;  user_local_rd_int.ready  = 1'b1;
        user_local_wr_int.valid  = 1'b0;  user_local_wr_int.data  = '0;  user_local_wr_int.ready  = 1'b1;
        user_remote_rd_int.valid = 1'b0;  user_remote_rd_int.data = '0;  user_remote_rd_int.ready = 1'b1;
        user_remote_wr_int.valid = 1'b0;  user_remote_wr_int.data = '0;  user_remote_wr_int.ready = 1'b1;
    end
end else begin : g_shared_head

assign remote_strm_1 = ~(is_strm_local(user_sq_rd_int.data.strm));
assign remote_strm_2 = ~(is_strm_local(user_sq_wr_int.data.strm));

always_comb begin
    user_sq_rd_int.ready = 1'b0;
    user_sq_wr_int.ready = 1'b0;

    user_local_rd_int.valid = 1'b0;
    user_local_wr_int.valid = 1'b0;
    user_remote_rd_int.valid = 1'b0;
    user_remote_wr_int.valid = 1'b0;

    if(user_sq_rd_int.valid) begin
        user_remote_rd_int.valid = remote_strm_1;
        user_local_rd_int.valid = ~remote_strm_1;
        
        user_sq_rd_int.ready = remote_strm_1 ? (user_remote_rd_int.ready) : (user_local_rd_int.ready);
    end

    if(user_sq_wr_int.valid) begin
        user_remote_wr_int.valid = remote_strm_2;
        user_local_wr_int.valid = ~remote_strm_2;

        user_sq_wr_int.ready = remote_strm_2 ? (user_remote_wr_int.ready) : (user_local_wr_int.ready);
    end
end

assign user_local_rd_int.data = user_sq_rd_int.data;
assign user_local_wr_int.data = user_sq_wr_int.data;
always_comb begin
    user_remote_rd_int.data.req_1 = user_sq_rd_int.data;
    user_remote_rd_int.data.req_2 = 0;

    user_remote_wr_int.data.req_1 = 0;
    user_remote_wr_int.data.req_2 = user_sq_wr_int.data;
end

meta_reg #(.DATA_BITS($bits(req_t))) inst_reg_local_rd  (.aclk(aclk), .aresetn(aresetn), .s_meta(user_local_rd_int), .m_meta(user_local_rd));
meta_reg #(.DATA_BITS($bits(req_t))) inst_reg_local_wr  (.aclk(aclk), .aresetn(aresetn), .s_meta(user_local_wr_int), .m_meta(user_local_wr));
meta_reg #(.DATA_BITS($bits(dreq_t))) inst_reg_remote_rd (.aclk(aclk), .aresetn(aresetn), .s_meta(user_remote_rd_int), .m_meta(user_remote_rd));
meta_reg #(.DATA_BITS($bits(dreq_t))) inst_reg_remote_wr (.aclk(aclk), .aresetn(aresetn), .s_meta(user_remote_wr_int), .m_meta(user_remote_wr));
end

`else

always_comb begin
    user_sq_rd_int.ready = 1'b0;
    user_sq_wr_int.ready = 1'b0;

    user_local_rd_int.valid = 1'b0;
    user_local_wr_int.valid = 1'b0;

    if(user_sq_rd_int.valid) begin
        user_local_rd_int.valid = 1'b1;

        user_sq_rd_int.ready = user_local_rd_int.ready;
    end

    if(user_sq_wr_int.valid) begin
        user_local_wr_int.valid = 1'b1;

        user_sq_wr_int.ready = user_local_wr_int.ready;
    end
end

assign user_local_rd_int.data = user_sq_rd_int.data;
assign user_local_wr_int.data = user_sq_wr_int.data;

meta_reg #(.DATA_BITS($bits(req_t))) inst_reg_local_rd  (.aclk(aclk), .aresetn(aresetn), .s_meta(user_local_rd_int), .m_meta(user_local_rd));
meta_reg #(.DATA_BITS($bits(req_t))) inst_reg_local_wr  (.aclk(aclk), .aresetn(aresetn), .s_meta(user_local_wr_int), .m_meta(user_local_wr));

`endif 
    
endmodule
