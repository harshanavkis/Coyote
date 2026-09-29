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

`include "log.svh"
`include "c_axi4.svh"

/**
 * Host writes into the vFPGA's user data window (cThread::uwinWrite). The
 * bytes go out as the PCIe bridge would hand them over: bursts of at most
 * one PCIe write (256 B) that never cross a 4 KB boundary, at 64 B-aligned
 * addresses, with byte strobes marking the bytes written.
 */
class uwin_simulation;
    localparam integer MAX_BURST = 256;
    c_axi4 drv;

    function new(c_axi4 drv);
        this.drv = drv;
    endfunction

    task initialize();
        drv.reset_m();
    endtask

    task write(input longint offset, input byte data[]);
        longint pos = 0;
        while (pos < data.size()) begin
            logic [AXI_DATA_BITS-1:0]   beats [$];
            logic [AXI_DATA_BITS/8-1:0] strbs [$];
            logic [1:0] resp;
            longint start = offset + pos;
            longint base  = start & ~64'h3F;
            // this burst ends at MAX_BURST past its aligned start, the next 4 KB
            // boundary, or the end of the data, whichever is first
            longint lim   = base + MAX_BURST;
            longint pg    = (start & ~64'hFFF) + 4096;
            longint stop  = offset + data.size();
            if (pg < lim)   lim = pg;
            if (stop < lim) lim = stop;
            for (longint a = base; a < lim; a += 64) begin
                logic [AXI_DATA_BITS-1:0]   d = '0;
                logic [AXI_DATA_BITS/8-1:0] s = '0;
                for (int b = 0; b < 64; b++)
                    if (a + b >= start && a + b < lim) begin
                        d[8*b +: 8] = data[a + b - offset];
                        s[b] = 1'b1;
                    end
                beats.push_back(d);
                strbs.push_back(s);
            end
            drv.write_burst(AXI_ADDR_BITS'(64'h0800_0000 + base), beats, strbs, resp);
            `ASSERT(resp == 2'b00, ("uwin write at %x: resp %b", base, resp))
            pos = lim - offset;
        end
        `DEBUG(("uwin: wrote %0d bytes at offset %x", data.size(), offset))
    endtask
endclass
