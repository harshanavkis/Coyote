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

/**
 * AXI4 write master (full width, INCR bursts), as the host's bypass BAR
 * drives a vFPGA's user data window (axi_udata). Reads are not issued.
 */
class c_axi4;
    virtual AXI4 axi;
    logic [AXI_ID_BITS-1:0] id = 0;

    function new(virtual AXI4 axi);
        this.axi = axi;
    endfunction

    task reset_m;
        axi.awaddr   <= 0; axi.awburst <= 2'b01; axi.awcache <= 0; axi.awid <= 0;
        axi.awlen    <= 0; axi.awlock  <= 0;     axi.awprot  <= 0; axi.awqos <= 0;
        axi.awregion <= 0; axi.awsize  <= 3'd6;  axi.awvalid <= 0;
        axi.araddr   <= 0; axi.arburst <= 2'b01; axi.arcache <= 0; axi.arid <= 0;
        axi.arlen    <= 0; axi.arlock  <= 0;     axi.arprot  <= 0; axi.arqos <= 0;
        axi.arregion <= 0; axi.arsize  <= 3'd6;  axi.arvalid <= 0;
        axi.wdata    <= 0; axi.wstrb   <= 0;     axi.wlast   <= 0; axi.wvalid <= 0;
        axi.bready   <= 0; axi.rready  <= 0;
        `DEBUG(("c_axi4 reset_m() completed."))
    endtask

    // One burst at a 64 B-aligned address; waits for its response.
    //
    // Driven like the block TBs: aligned to a clock edge first, so every
    // value set here is seen by the DUT at the next edge, and each handshake
    // completes on an edge where the DUT has sampled valid (the AXI4 clocking
    // block does not cover awlen/awid/wlast/bid). Without the alignment a
    // caller resuming in the same time step as an edge could raise and drop
    // awvalid before the DUT ever saw it.
    task write_burst(
        input logic [AXI_ADDR_BITS-1:0]   addr,
        input logic [AXI_DATA_BITS-1:0]   data [$],
        input logic [AXI_DATA_BITS/8-1:0] strb [$],
        output logic [1:0]                resp
    );
        @(posedge axi.aclk);
        axi.awaddr  <= addr;
        axi.awlen   <= 8'(data.size() - 1);
        axi.awid    <= id;
        axi.awvalid <= 1'b1;
        do @(posedge axi.aclk); while (!axi.awready);
        axi.awvalid <= 1'b0;
        foreach (data[i]) begin
            axi.wdata  <= data[i];
            axi.wstrb  <= strb[i];
            axi.wlast  <= (i == data.size() - 1);
            axi.wvalid <= 1'b1;
            do @(posedge axi.aclk); while (!axi.wready);
        end
        axi.wvalid <= 1'b0;
        axi.wlast  <= 1'b0;
        axi.bready <= 1'b1;
        do @(posedge axi.aclk); while (!axi.bvalid);
        resp = axi.bresp;
        `ASSERT(axi.bid == id, ("uwin B id %0d, expected %0d", axi.bid, id))
        axi.bready <= 1'b0;
        id++;
    endtask
endclass
