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

always_comb begin 
    /*
     * CONTROL SIGNALS
     * 
     * rq_(wr|rd) are two more Coyote interfaces, which act as inputs to the user application.
     * They are driven by Coyote's RDMA stack in reaction to *incoming* network traffic.
     * Their meaning is:
     *   rq_wr - payload has arrived from the remote node and has to be placed in local memory.
     *           This covers incoming RDMA WRITEs/SENDs as well as the RDMA READ RESPONSEs that
     *           come back for a READ we issued ourselves.
     *   rq_rd - a remote RDMA READ REQUEST has arrived, so local payload has to be fetched in
     *           order to build the RDMA READ RESPONSE that answers it.
     * Here, they are used to set Coyote's generic hardware send queues, previously discussed in Example 7.
     * Note that rq_(wr|rd) only carry the *request*; the corresponding payload travels on the
     * axis_rreq_* / axis_rrsp_* streams that are wired up further below, and the dest field
     * chosen here selects which axis_host_(send|recv) stream the transfer is paired with.
     */
    // Write
    sq_wr.valid = rq_wr.valid;
    rq_wr.ready = sq_wr.ready;
    sq_wr.data = rq_wr.data;            // Data field holds information such as remote, virtual address, buffer length etc.
    sq_wr.data.strm = STRM_HOST;        // Marks the transfer as local and targets host memory; this example is built without card memory
    sq_wr.data.dest = is_opcode_rd_resp(rq_wr.data.opcode) ? 0 : 1;   // READ RESPONSEs to axis_host_send[0], WRITEs/SENDs to axis_host_send[1]

    // Reads
    sq_rd.valid = rq_rd.valid;
    rq_rd.ready = sq_rd.ready;
    sq_rd.data = rq_rd.data;           // Data field holds information such as remote, virtual address, buffer length etc.
    sq_rd.data.strm = STRM_HOST;       // Marks the transfer as local and targets host memory; this example is built without card memory
    sq_rd.data.dest = 1;               // READ RESPONSE payload is fetched on axis_host_recv[1]
end

/*
 * CONTROL SIGNALS FOR HOST-INITIATED OPERATIONS (not visible in this file)
 *
 * The block above only handles traffic that the *remote* node initiates. Operations that the
 * local host software starts - i.e. coyote_thread.invoke(REMOTE_RDMA_WRITE/READ, sg) - never
 * pass through the vFPGA's sq_(wr|rd) interfaces. They are written into Coyote's host send
 * queue and split up inside the shell, outside of the vFPGA.
 *   REMOTE_RDMA_WRITE - the shell issues the descriptor to the RDMA stack *and*, in parallel,
 *                       a local host-memory read. The payload of that read shows up in the
 *                       vFPGA on axis_host_recv[rdmaSg.local_dest] (default 0) and we are
 *                       expected to push it into axis_rreq_send below. This is why
 *                       axis_host_recv[0] has no matching sq_rd anywhere in this file.
 *   REMOTE_RDMA_READ  - only the descriptor goes to the RDMA stack. The requested data comes
 *                       back later as RDMA READ RESPONSEs, which the remote node's responder
 *                       logic sends and which we then handle here via rq_wr / axis_rreq_recv.
 * Consequently, the index of a stream tells you who started the transfer: index 0 belongs to
 * the requester side (we initiated), index 1 to the responder side (the remote node initiated).
 */

/*
 * DATA SIGNALS
 *
 * Naming convention, always seen from the vFPGA: 'recv' is a stream flowing *into* the user
 * logic, 'send' a stream flowing *out* of it. The prefix names the other endpoint: 'host' is
 * host memory reached via DMA, while 'rreq'/'rrsp' are the requester and responder sides of
 * the RDMA stack, i.e. towards the network and the remote node.
 */
// Data streams for outgoing RDMA WRITEs and SENDs (from local host to network stack to remote node)
`AXISR_ASSIGN(axis_host_recv[0], axis_rreq_send[0])

// Data streams for incoming RDMA READ RESPONSEs (from remote node to network stack to local host)
`AXISR_ASSIGN(axis_rreq_recv[0], axis_host_send[0])

// Data streams for outgoing RDMA READ RESPONSEs (from local host to network stack to remote node)
`AXISR_ASSIGN(axis_host_recv[1], axis_rrsp_send[0])

// Data streams for incoming RDMA WRITEs and SENDs (from remote node to network stack to local host)
`AXISR_ASSIGN(axis_rrsp_recv[0], axis_host_send[1])

/*
 * STAMPS (Loom latency breakdown): stamp_rec records the cycle of every event at 9 points, read through
 * axi_ctrl (stamp_csr) at the same CSR words as loom_switch. Points: 0 first payload beat of a packet back
 * from the host read (axis_host_recv[0]), 1 unused (the post happens in the shell), 2 first beat of a packet
 * to the stack, 3 remote ack (cq_wr), 4 rq_wr taken, 5 first beat of a packet from the stack, 6 landing write
 * posted (sq_wr, dest 1), 7 first landing beat out (axis_host_send[1]), 8 landing write completed (cq_wr,
 * local, dest 1). Landing beats are paired with the landing writes in post order, as in loom_switch.
 */
localparam integer N_ST = 9, ST_DEPTH = 4096;
logic                        st_arm;
logic [3:0]                  st_pt;
logic [$clog2(ST_DEPTH)-1:0] st_idx;
logic [31:0]                 st_data, st_now;
logic [N_ST-1:0][31:0]       st_cnt;

stamp_csr #(.N_ST(N_ST), .ST_DEPTH(ST_DEPTH)) inst_stamp_csr (
    .aclk(aclk), .aresetn(aresetn), .axi_ctrl(axi_ctrl),
    .st_arm(st_arm), .st_pt(st_pt), .st_idx(st_idx), .st_data(st_data), .st_cnt(st_cnt), .st_now(st_now)
);

logic st_tx_mid, st_rx_mid;
wire  st_hr   = axis_host_recv[0].tvalid && axis_host_recv[0].tready;
wire  st_tx   = axis_rreq_send[0].tvalid && axis_rreq_send[0].tready;
wire  st_rx   = axis_rrsp_recv[0].tvalid && axis_rrsp_recv[0].tready;
wire  st_h    = axis_host_send[1].tvalid && axis_host_send[1].tready;
wire  st_land = sq_wr.valid && sq_wr.ready && (sq_wr.data.dest == 1);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        st_tx_mid <= 1'b0; st_rx_mid <= 1'b0;
    end else begin
        if (st_tx) st_tx_mid <= !axis_rreq_send[0].tlast;
        if (st_rx) st_rx_mid <= !axis_rrsp_recv[0].tlast;
    end
end

localparam integer ST_HQ = 16;
logic [LEN_BITS-1:0]     st_hq [ST_HQ];
logic [$clog2(ST_HQ):0]  st_hq_wp, st_hq_rp;
logic [LEN_BITS-1:0]     st_h_seen;
wire  [LEN_BITS-1:0]     st_h_beats = st_hq[st_hq_rp[$clog2(ST_HQ)-1:0]];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        st_hq_wp <= '0; st_hq_rp <= '0; st_h_seen <= '0;
    end else begin
        if (st_land) begin
            st_hq[st_hq_wp[$clog2(ST_HQ)-1:0]] <= (sq_wr.data.len + 63) >> 6;
            st_hq_wp <= st_hq_wp + 1'b1;
        end
        if (st_h) begin
            if (st_h_seen + 1'b1 == st_h_beats) begin
                st_h_seen <= '0;
                st_hq_rp  <= st_hq_rp + 1'b1;
            end else st_h_seen <= st_h_seen + 1'b1;
        end
    end
end

stamp_rec #(.N_PTS(N_ST), .DEPTH(ST_DEPTH)) inst_stamp_rec (
    .aclk(aclk), .aresetn(aresetn),
    .ev({cq_wr.valid && !cq_wr.data.remote && (cq_wr.data.dest == 1),   // 8 landing write completed
         st_h && (st_h_seen == '0),                                     // 7 first landing beat out
         st_land,                                                       // 6 landing write posted
         st_rx && !st_rx_mid,                                           // 5 first beat from the stack
         rq_wr.valid && rq_wr.ready,                                    // 4 rq_wr taken
         cq_wr.valid && cq_wr.data.remote,                              // 3 remote ack
         st_tx && !st_tx_mid,                                           // 2 first beat to the stack
         1'b0,                                                          // 1 unused
         st_hr && !st_tx_mid}),                                         // 0 first beat back from the host read
    .arm(st_arm), .sel_pt(st_pt), .sel_idx(st_idx), .sel_data(st_data), .cnt(st_cnt), .now(st_now)
);

// Tie off unused interfaces
always_comb notify.tie_off_m();
always_comb cq_rd.tie_off_s();
always_comb cq_wr.tie_off_s();
