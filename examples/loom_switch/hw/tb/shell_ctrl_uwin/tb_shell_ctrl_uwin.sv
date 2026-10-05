`timescale 1ns / 1ps

/**
 * tb_shell_ctrl_uwin - the generated shell_ctrl block design (design_ctrl,
 * hw/bd/ultrascale_plus/cr_ctrl.tcl) with EN_UWIN=1, EN_AVX=1, driven on
 * axi_main (the XDMA bypass BAR) as the host and a peer device would.
 * Every master port has a recording slave model.
 *
 * Checks: uwin bursts arrive whole, in order, on axim_udata_<i> (and only
 * there), up to the uwin's last byte; the existing regions (shell config,
 * axi_ctrl_<i>, axim_ctrl_<i>) still decode where they did; holes and
 * addresses past the uwin get DECERR; uwin reads come back; and the rate of
 * back-to-back 4-beat bursts through the interconnect.
 *
 * `define N2 builds the two-region variant (uwin split into 2 x 64 MB).
 */

// ---------------------------------------------------------------------------
// Slave models
// ---------------------------------------------------------------------------
module axi4_sink #(parameter integer DW = 512) (
    input  logic            aclk,
    input  logic [63:0]     awaddr, araddr,
    input  logic [7:0]      awlen, arlen,
    input  logic [2:0]      awsize, arsize,
    input  logic [1:0]      awburst, arburst,
    input  logic [3:0]      awcache, arcache, awqos, arqos, awregion, arregion,
    input  logic [2:0]      awprot, arprot,
    input  logic [0:0]      awlock, arlock,
    input  logic            awvalid, arvalid,
    output logic            awready, arready,
    input  logic [DW-1:0]   wdata,
    input  logic [DW/8-1:0] wstrb,
    input  logic            wlast, wvalid,
    output logic            wready,
    output logic [1:0]      bresp, rresp,
    output logic            bvalid, rvalid, rlast,
    input  logic            bready, rready,
    output logic [DW-1:0]   rdata,
    input  logic [5:0]      awid, arid,
    output logic [5:0]      bid, rid
);
    longint aw_addr [$];
    int     aw_len  [$];
    logic [DW-1:0] w_data [$];
    bit     w_last  [$];
    int     n_b = 0;         // B responses owed
    int     b_ids [$];       // their IDs, in AW order
    logic [5:0] bid_r = 0;
    longint first_w = -1, last_w = 0, cyc = 0;

    assign awready = 1'b1;
    assign wready  = 1'b1;
    assign bresp   = 2'b00;
    assign rresp   = 2'b00;
    always @(posedge aclk) begin
        cyc++;
        if (awvalid) begin aw_addr.push_back(awaddr); aw_len.push_back(awlen); b_ids.push_back(awid); end
        if (wvalid) begin
            w_data.push_back(wdata); w_last.push_back(wlast);
            if (first_w < 0) first_w = cyc;
            last_w = cyc;
            if (wlast) n_b++;
        end
        if (bvalid && bready) begin n_b--; void'(b_ids.pop_front()); end
        bid_r = (b_ids.size() != 0) ? 6'(b_ids[0]) : '0;
    end
    assign bvalid = (n_b > 0);
    assign bid    = bid_r;

    // Reads: one burst at a time; beat k of a read at A returns {k, A}.
    // While r_hold is set every read is accepted and queued, none answered
    // (n_ar counts the reads accepted); they are answered in order after.
    logic   r_act = 0;
    longint r_addr;
    int     r_left, r_k;
    logic [5:0] r_id;
    bit     r_hold = 0;
    int     n_ar = 0;
    longint q_addr [$];
    int     q_len  [$];
    logic [5:0] q_id [$];
    assign rid     = r_id;
    assign arready = r_hold || (!r_act && q_addr.size() == 0);
    assign rvalid  = r_act;
    assign rlast   = r_act && (r_left == 0);
    assign rdata   = DW'({32'(r_k), r_addr[31:0]});
    always @(posedge aclk) begin
        if (arvalid && arready) n_ar++;
        if (r_hold && arvalid) begin q_addr.push_back(araddr); q_len.push_back(arlen); q_id.push_back(arid); end
        if (!r_act) begin
            if (!r_hold && q_addr.size() != 0) begin
                r_act <= 1; r_addr <= q_addr.pop_front(); r_left <= q_len.pop_front(); r_k <= 0; r_id <= q_id.pop_front();
            end else if (!r_hold && arvalid) begin
                r_act <= 1; r_addr <= araddr; r_left <= arlen; r_k <= 0; r_id <= arid;
            end
        end else if (rready) begin
            if (r_left == 0) r_act <= 0;
            r_left <= r_left - 1; r_k <= r_k + 1;
        end
    end
endmodule

module axil_sink (
    input  logic        aclk,
    input  logic [63:0] awaddr, araddr,
    input  logic [2:0]  awprot, arprot,
    input  logic        awvalid, arvalid,
    output logic        awready, arready,
    input  logic [63:0] wdata,
    input  logic [7:0]  wstrb,
    input  logic        wvalid,
    output logic        wready,
    output logic [1:0]  bresp, rresp,
    output logic        bvalid, rvalid,
    input  logic        bready, rready,
    output logic [63:0] rdata
);
    longint aw_addr [$];
    longint w_data  [$];
    int     n_b = 0;
    assign awready = 1'b1;
    assign wready  = 1'b1;
    assign bresp   = 2'b00;
    assign rresp   = 2'b00;
    assign rdata   = 64'hA5A5_0000_0000_0000;
    assign arready = 1'b1;
    logic  r_pend = 0;
    assign rvalid  = r_pend;
    always @(posedge aclk) begin
        if (awvalid) aw_addr.push_back(awaddr);
        if (wvalid) begin w_data.push_back(wdata); n_b++; end
        if (bvalid && bready) n_b--;
        if (arvalid) r_pend <= 1; else if (rready) r_pend <= 0;
    end
    assign bvalid = (n_b > 0);
endmodule

// ---------------------------------------------------------------------------
// TB
// ---------------------------------------------------------------------------
module tb_shell_ctrl_uwin;

`ifdef N2
localparam integer NREG = 2;
`else
localparam integer NREG = 1;
`endif
localparam longint UWIN      = 64'h0800_0000;
localparam longint UWIN_SIZE = 64'h0800_0000 / NREG;
localparam integer RD_IN_FLIGHT = 32;     // the shell's reads in flight into a uwin (cr_ctrl.tcl)

logic xclk = 0, xresetn = 0, sys_reset = 1;
always #2 xclk = ~xclk;
logic aclk, aresetn, nclk, nresetn, uclk, uresetn, lckresetn;

// axi_main (driven here)
logic [63:0]  m_awaddr = 0, m_araddr = 0;
logic [7:0]   m_awlen = 0, m_arlen = 0;
logic [2:0]   m_awsize = 6, m_arsize = 6;
logic [5:0]   m_awid = 0, m_arid = 0;
logic         m_awvalid = 0, m_arvalid = 0, m_wvalid = 0, m_wlast = 0;
logic [511:0] m_wdata = 0;
logic [63:0]  m_wstrb = '1;
logic         m_awready, m_arready, m_wready, m_bvalid, m_rvalid, m_rlast;
logic [5:0]   m_bid, m_rid;
logic [1:0]   m_bresp, m_rresp;
logic [511:0] m_rdata;
logic         m_bready = 1, m_rready = 1;

`define AXIL_PORT(p) \
    logic [63:0] p``_awaddr, p``_araddr, p``_wdata, p``_rdata; \
    logic [2:0]  p``_awprot, p``_arprot; \
    logic [7:0]  p``_wstrb; \
    logic [1:0]  p``_bresp, p``_rresp; \
    logic p``_awvalid, p``_awready, p``_arvalid, p``_arready, p``_wvalid, p``_wready, \
          p``_bvalid, p``_bready, p``_rvalid, p``_rready;
`define AXI4_PORT(p, DW) \
    logic [63:0] p``_awaddr, p``_araddr; \
    logic [7:0]  p``_awlen, p``_arlen; \
    logic [2:0]  p``_awsize, p``_arsize, p``_awprot, p``_arprot; \
    logic [1:0]  p``_awburst, p``_arburst, p``_bresp, p``_rresp; \
    logic [3:0]  p``_awcache, p``_arcache, p``_awqos, p``_arqos, p``_awregion, p``_arregion; \
    logic [0:0]  p``_awlock, p``_arlock; \
    logic [DW-1:0] p``_wdata, p``_rdata; \
    logic [DW/8-1:0] p``_wstrb; \
    logic p``_awvalid, p``_awready, p``_arvalid, p``_arready, p``_wvalid, p``_wready, p``_wlast, \
          p``_bvalid, p``_bready, p``_rvalid, p``_rready, p``_rlast;

`AXIL_PORT(cnfg)
`AXIL_PORT(ctrl0)
`AXI4_PORT(avx0, 256)
logic [5:0] avx0_awid = 0, avx0_arid = 0, avx0_bid, avx0_rid;
`AXI4_PORT(ud0, 512)
logic [5:0] ud0_awid, ud0_arid, ud0_bid, ud0_rid;
`ifdef N2
`AXIL_PORT(ctrl1)
`AXI4_PORT(avx1, 256)
logic [5:0] avx1_awid = 0, avx1_arid = 0, avx1_bid, avx1_rid;
`AXI4_PORT(ud1, 512)
logic [5:0] ud1_awid, ud1_arid, ud1_bid, ud1_rid;
`endif

`define AXIL_CONN(bd, p) \
    .bd``_araddr(p``_araddr), .bd``_arprot(p``_arprot), .bd``_arready(p``_arready), .bd``_arvalid(p``_arvalid), \
    .bd``_awaddr(p``_awaddr), .bd``_awprot(p``_awprot), .bd``_awready(p``_awready), .bd``_awvalid(p``_awvalid), \
    .bd``_bready(p``_bready), .bd``_bresp(p``_bresp), .bd``_bvalid(p``_bvalid), \
    .bd``_rdata(p``_rdata), .bd``_rready(p``_rready), .bd``_rresp(p``_rresp), .bd``_rvalid(p``_rvalid), \
    .bd``_wdata(p``_wdata), .bd``_wready(p``_wready), .bd``_wstrb(p``_wstrb), .bd``_wvalid(p``_wvalid)
`define AXI4_CONN(bd, p) \
    .bd``_araddr(p``_araddr), .bd``_arburst(p``_arburst), .bd``_arcache(p``_arcache), .bd``_arlen(p``_arlen), \
    .bd``_arlock(p``_arlock), .bd``_arprot(p``_arprot), .bd``_arqos(p``_arqos), .bd``_arready(p``_arready), \
    .bd``_arregion(p``_arregion), .bd``_arsize(p``_arsize), .bd``_arvalid(p``_arvalid), \
    .bd``_awaddr(p``_awaddr), .bd``_awburst(p``_awburst), .bd``_awcache(p``_awcache), .bd``_awlen(p``_awlen), \
    .bd``_awlock(p``_awlock), .bd``_awprot(p``_awprot), .bd``_awqos(p``_awqos), .bd``_awready(p``_awready), \
    .bd``_awregion(p``_awregion), .bd``_awsize(p``_awsize), .bd``_awvalid(p``_awvalid), \
    .bd``_bready(p``_bready), .bd``_bresp(p``_bresp), .bd``_bvalid(p``_bvalid), \
    .bd``_rdata(p``_rdata), .bd``_rlast(p``_rlast), .bd``_rready(p``_rready), .bd``_rresp(p``_rresp), .bd``_rvalid(p``_rvalid), \
    .bd``_wdata(p``_wdata), .bd``_wlast(p``_wlast), .bd``_wready(p``_wready), .bd``_wstrb(p``_wstrb), .bd``_wvalid(p``_wvalid)
`define AXIL_SINK(p) \
    axil_sink s_``p (.aclk(aclk), .awaddr(p``_awaddr), .araddr(p``_araddr), .awprot(p``_awprot), .arprot(p``_arprot), \
        .awvalid(p``_awvalid), .arvalid(p``_arvalid), .awready(p``_awready), .arready(p``_arready), \
        .wdata(p``_wdata), .wstrb(p``_wstrb), .wvalid(p``_wvalid), .wready(p``_wready), \
        .bresp(p``_bresp), .rresp(p``_rresp), .bvalid(p``_bvalid), .rvalid(p``_rvalid), \
        .bready(p``_bready), .rready(p``_rready), .rdata(p``_rdata));
`define AXI4_SINK(p, W) \
    axi4_sink #(.DW(W)) s_``p (.aclk(aclk), .awaddr(p``_awaddr), .araddr(p``_araddr), .awlen(p``_awlen), .arlen(p``_arlen), \
        .awsize(p``_awsize), .arsize(p``_arsize), .awburst(p``_awburst), .arburst(p``_arburst), \
        .awcache(p``_awcache), .arcache(p``_arcache), .awqos(p``_awqos), .arqos(p``_arqos), \
        .awregion(p``_awregion), .arregion(p``_arregion), .awprot(p``_awprot), .arprot(p``_arprot), \
        .awlock(p``_awlock), .arlock(p``_arlock), .awvalid(p``_awvalid), .arvalid(p``_arvalid), \
        .awready(p``_awready), .arready(p``_arready), .wdata(p``_wdata), .wstrb(p``_wstrb), \
        .wlast(p``_wlast), .wvalid(p``_wvalid), .wready(p``_wready), .bresp(p``_bresp), .rresp(p``_rresp), \
        .bvalid(p``_bvalid), .rvalid(p``_rvalid), .rlast(p``_rlast), .bready(p``_bready), .rready(p``_rready), \
        .rdata(p``_rdata), .awid(p``_awid), .arid(p``_arid), .bid(p``_bid), .rid(p``_rid));

design_ctrl dut (
    .axi_main_araddr(m_araddr), .axi_main_arburst(2'b01), .axi_main_arcache(4'b0), .axi_main_arid(m_arid),
    .axi_main_arlen(m_arlen), .axi_main_arlock(1'b0), .axi_main_arprot(3'b0), .axi_main_arqos(4'b0),
    .axi_main_arregion(4'b0), .axi_main_arsize(m_arsize), .axi_main_arready(m_arready), .axi_main_arvalid(m_arvalid),
    .axi_main_awaddr(m_awaddr), .axi_main_awburst(2'b01), .axi_main_awcache(4'b0), .axi_main_awid(m_awid),
    .axi_main_awlen(m_awlen), .axi_main_awlock(1'b0), .axi_main_awprot(3'b0), .axi_main_awqos(4'b0),
    .axi_main_awregion(4'b0), .axi_main_awsize(m_awsize), .axi_main_awready(m_awready), .axi_main_awvalid(m_awvalid),
    .axi_main_bid(m_bid), .axi_main_bready(m_bready), .axi_main_bresp(m_bresp), .axi_main_bvalid(m_bvalid),
    .axi_main_rdata(m_rdata), .axi_main_rid(m_rid), .axi_main_rlast(m_rlast), .axi_main_rready(m_rready),
    .axi_main_rresp(m_rresp), .axi_main_rvalid(m_rvalid),
    .axi_main_wdata(m_wdata), .axi_main_wlast(m_wlast), .axi_main_wready(m_wready), .axi_main_wstrb(m_wstrb),
    .axi_main_wvalid(m_wvalid),
    `AXIL_CONN(axi_cnfg, cnfg),
    `AXIL_CONN(axi_ctrl_0, ctrl0),
    `AXI4_CONN(axim_ctrl_0, avx0),
    `AXI4_CONN(axim_udata_0, ud0),
    .axim_udata_0_awid(ud0_awid), .axim_udata_0_arid(ud0_arid), .axim_udata_0_bid(ud0_bid), .axim_udata_0_rid(ud0_rid),
`ifdef N2
    `AXIL_CONN(axi_ctrl_1, ctrl1),
    `AXI4_CONN(axim_ctrl_1, avx1),
    `AXI4_CONN(axim_udata_1, ud1),
    .axim_udata_1_awid(ud1_awid), .axim_udata_1_arid(ud1_arid), .axim_udata_1_bid(ud1_bid), .axim_udata_1_rid(ud1_rid),
`endif
    .xclk(xclk), .xresetn(xresetn), .aclk(aclk), .aresetn(aresetn), .nclk(nclk), .nresetn(nresetn),
    .uclk(uclk), .uresetn(uresetn), .lckresetn(lckresetn), .sys_reset(sys_reset)
);

`AXIL_SINK(cnfg)
`AXIL_SINK(ctrl0)
`AXI4_SINK(avx0, 256)
`AXI4_SINK(ud0, 512)
`ifdef N2
`AXIL_SINK(ctrl1)
`AXI4_SINK(avx1, 256)
`AXI4_SINK(ud1, 512)
`endif

int errors = 0;
bit dbg = 0;
always @(posedge aclk) if (dbg) begin
    if (cnfg_awvalid) $display("  [%0t] cnfg AW %h", $time, cnfg_awaddr);
    if (cnfg_wvalid)  $display("  [%0t] cnfg W %h strb %h", $time, cnfg_wdata, cnfg_wstrb);
    if (cnfg_bvalid)  $display("  [%0t] cnfg B ready %b", $time, cnfg_bready);
end
always @(posedge xclk) if (dbg && m_bvalid) $display("  [%0t] main B %0d id %0d", $time, m_bresp, m_bid);
`define CHECK(c, msg) if (!(c)) begin errors++; $display("FAIL [%0t] %s", $time, msg); end

function automatic logic [511:0] pat(input longint addr, input int k);
    logic [511:0] d;
    for (int l = 0; l < 8; l++) d[64*l +: 64] = {16'(l), 16'(k), 32'(addr)};
    return d;
endfunction

// One write burst on axi_main; returns bresp
task automatic wr(input longint addr, input int beats, output logic [1:0] resp,
                  input logic [2:0] size = 6, input logic [63:0] strb = '1);
    @(posedge xclk);
    m_awaddr <= addr; m_awlen <= 8'(beats - 1); m_awsize <= size; m_awid <= m_awid + 1; m_awvalid <= 1;
    do @(posedge xclk); while (!m_awready);
    m_awvalid <= 0;
    for (int k = 0; k < beats; k++) begin
        m_wdata <= pat(addr, k); m_wstrb <= (size == 3) ? (64'hFF << addr[5:0]) : strb;
        m_wlast <= (k == beats - 1); m_wvalid <= 1;
        do @(posedge xclk); while (!m_wready);
    end
    m_wvalid <= 0;
    if (dbg) $display("  [%0t] wr %h: AW+W done", $time, addr);
    while (!m_bvalid) @(posedge xclk);
    resp = m_bresp;
    `CHECK(m_bid == m_awid, $sformatf("wr %h: bid %0d, awid %0d", addr, m_bid, m_awid))
    @(posedge xclk);
endtask

// A uwin burst arrived whole at sink s: one AW with this address and length,
// then exactly its beats, wlast on the last
`define EXPECT_UD(s, addr, beats, name) \
    begin \
        `CHECK(s.aw_addr.size() == 1, $sformatf("%s: %0d AWs at %m", name, s.aw_addr.size())) \
        if (s.aw_addr.size() >= 1) begin \
            `CHECK(s.aw_addr[0] == (addr) && s.aw_len[0] == (beats) - 1, \
                   $sformatf("%s: AW addr %h len %0d", name, s.aw_addr[0], s.aw_len[0])) \
            void'(s.aw_addr.pop_front()); void'(s.aw_len.pop_front()); \
        end \
        `CHECK(s.w_data.size() == (beats), $sformatf("%s: %0d beats", name, s.w_data.size())) \
        for (int k = 0; k < (beats) && s.w_data.size() != 0; k++) begin \
            `CHECK(s.w_data.pop_front() == pat(addr, k), $sformatf("%s: beat %0d data", name, k)) \
            `CHECK(s.w_last.pop_front() == (k == (beats) - 1), $sformatf("%s: beat %0d wlast", name, k)) \
        end \
    end

task automatic expect_quiet(input string name);
    `CHECK(s_cnfg.aw_addr.size() == 0 && s_ctrl0.aw_addr.size() == 0 && s_avx0.aw_addr.size() == 0 &&
           s_ud0.aw_addr.size() == 0, $sformatf("%s: a write reached a port it should not have", name))
`ifdef N2
    `CHECK(s_ctrl1.aw_addr.size() == 0 && s_avx1.aw_addr.size() == 0 && s_ud1.aw_addr.size() == 0,
           $sformatf("%s: a write reached a port it should not have (region 1)", name))
`endif
endtask

initial begin
    logic [1:0] r;
    repeat (20) @(posedge xclk);
    sys_reset = 0;
    repeat (20) @(posedge xclk);
    xresetn = 1;
    wait (aresetn === 1'b1);
    repeat (50) @(posedge xclk);
    $display("reset done at %0t", $time);

    // uwin: a 16-beat burst at the start, and the last 4 beats of the region
    wr(UWIN, 16, r);
    `CHECK(r == 2'b00, "uwin write not OKAY")
    `EXPECT_UD(s_ud0, UWIN, 16, "uwin start")
    wr(UWIN + UWIN_SIZE - 256, 4, r);
    `CHECK(r == 2'b00, "uwin end write not OKAY")
    `EXPECT_UD(s_ud0, UWIN + UWIN_SIZE - 256, 4, "uwin end")
    expect_quiet("uwin");
    $display("ok   uwin writes");

`ifdef N2
    wr(UWIN + UWIN_SIZE, 8, r);
    `CHECK(r == 2'b00, "uwin 1 write not OKAY")
    `EXPECT_UD(s_ud1, UWIN + UWIN_SIZE, 8, "uwin 1 start")
    wr(UWIN + 2*UWIN_SIZE - 128, 2, r);
    `EXPECT_UD(s_ud1, UWIN + 2*UWIN_SIZE - 128, 2, "uwin 1 end")
    expect_quiet("uwin 1");
    $display("ok   uwin 1 writes");
`endif

    // Existing regions: an 8 B store to each, as the host CPU does
    wr(64'h0000_0040, 1, r, 3, 64'hFF);
    `CHECK(r == 2'b00 && s_cnfg.aw_addr.size() == 1 && s_cnfg.aw_addr[0][31:0] == 32'h40,
           $sformatf("shell config: resp %0d, %0d AWs", r, s_cnfg.aw_addr.size()))
    s_cnfg.aw_addr.delete(); s_cnfg.w_data.delete();
    wr(64'h0010_1008, 1, r, 3, 64'hFF);
    `CHECK(r == 2'b00 && s_ctrl0.aw_addr.size() == 1 && s_ctrl0.aw_addr[0][31:0] == 32'h0010_1008,
           $sformatf("axi_ctrl_0: resp %0d, %0d AWs", r, s_ctrl0.aw_addr.size()))
    s_ctrl0.aw_addr.delete(); s_ctrl0.w_data.delete();
    wr(64'h0100_0040, 1, r, 3, 64'hFF);
    `CHECK(r == 2'b00 && s_avx0.aw_addr.size() == 1 && s_avx0.aw_addr[0][31:0] == 32'h0100_0040,
           $sformatf("axim_ctrl_0: resp %0d, %0d AWs", r, s_avx0.aw_addr.size()))
    s_avx0.aw_addr.delete(); s_avx0.aw_len.delete(); s_avx0.w_data.delete(); s_avx0.w_last.delete();
`ifdef N2
    wr(64'h0014_0008, 1, r, 3, 64'hFF);
    `CHECK(r == 2'b00 && s_ctrl1.aw_addr.size() == 1, $sformatf("axi_ctrl_1: resp %0d", r))
    s_ctrl1.aw_addr.delete(); s_ctrl1.w_data.delete();
    wr(64'h0104_0000, 1, r, 3, 64'hFF);
    `CHECK(r == 2'b00 && s_avx1.aw_addr.size() == 1, $sformatf("axim_ctrl_1: resp %0d", r))
    s_avx1.aw_addr.delete(); s_avx1.aw_len.delete(); s_avx1.w_data.delete(); s_avx1.w_last.delete();
`endif
    expect_quiet("existing regions");
    $display("ok   existing regions");

    // Holes and past the uwin: DECERR, nothing forwarded
    wr(64'h0200_0000, 1, r);
    `CHECK(r == 2'b11, $sformatf("hole: resp %0d, expected DECERR", r))
    wr(64'h1000_0000, 4, r);
    `CHECK(r == 2'b11, $sformatf("past the uwin: resp %0d, expected DECERR", r))
    repeat (50) @(posedge aclk);
    expect_quiet("decerr");
    $display("ok   holes and past the uwin");

    // uwin read comes back from the port
    begin
        int n = 0;
        @(posedge xclk);
        m_araddr <= UWIN + 64'h40; m_arlen <= 1; m_arsize <= 6; m_arvalid <= 1;
        do @(posedge xclk); while (!m_arready);
        m_arvalid <= 0;
        while (n < 2) begin
            @(posedge xclk);
            if (m_rvalid) begin
                `CHECK(m_rdata[63:0] == {32'(n), 32'(UWIN + 64'h40)} && m_rresp == 0 && m_rlast == (n == 1),
                       $sformatf("uwin read beat %0d: %h", n, m_rdata[63:0]))
                n++;
            end
        end
        $display("ok   uwin read");
    end

    // Rate: 64 back-to-back 4-beat bursts (256 B, a PCIe write's size)
    begin
        longint t0;
        s_ud0.first_w = -1;
        fork
            begin
                for (int k = 0; k < 64; k++) begin
                    m_awaddr <= UWIN + 256*k; m_awlen <= 3; m_awsize <= 6; m_awvalid <= 1;
                    do @(posedge xclk); while (!m_awready);
                end
                m_awvalid <= 0;
            end
            begin
                for (int k = 0; k < 256; k++) begin
                    m_wdata <= pat(UWIN + 256*(k/4), k%4); m_wstrb <= '1; m_wlast <= (k%4 == 3); m_wvalid <= 1;
                    do @(posedge xclk); while (!m_wready);
                end
                m_wvalid <= 0;
            end
        join
        wait (s_ud0.w_data.size() == 256);
        repeat (50) @(posedge aclk);
        `CHECK(s_ud0.aw_addr.size() == 64, $sformatf("rate: %0d AWs", s_ud0.aw_addr.size()))
        for (int k = 0; k < 256; k++)
            `CHECK(s_ud0.w_data[k] == pat(UWIN + 256*(k/4), k%4), $sformatf("rate: beat %0d", k))
        $display("ok   rate: 256 beats reached axim_udata_0 in %0d aclk cycles",
                 s_ud0.last_w - s_ud0.first_w + 1);
    end

    // Reads in flight: the window takes reads and answers none, so the reads
    // that reach axim_udata_0 are the shell's read acceptance (a peer's
    // reads each wait a network round trip there); then all are answered
    begin
        int got = 0;
        s_ud0.n_ar = 0;
        s_ud0.r_hold = 1;
        fork
            begin
                for (int k = 0; k < 64; k++) begin
                    m_araddr <= UWIN + 256*k; m_arlen <= 3; m_arsize <= 6; m_arvalid <= 1;
                    do @(posedge xclk); while (!m_arready);
                end
                m_arvalid <= 0;
            end
        join_none
        repeat (2000) @(posedge aclk);
        `CHECK(s_ud0.n_ar == RD_IN_FLIGHT, $sformatf("reads in flight: %0d reached axim_udata_0, expected %0d",
                                                     s_ud0.n_ar, RD_IN_FLIGHT))
        $display("ok   reads in flight: %0d reached axim_udata_0 with none answered", s_ud0.n_ar);
        s_ud0.r_hold = 0;
        while (got < 256) begin
            @(posedge xclk);
            if (m_rvalid) begin
                `CHECK(m_rdata[63:0] == {32'(got % 4), 32'(UWIN + 256*(got/4))} && m_rresp == 0 && m_rlast == (got % 4 == 3),
                       $sformatf("reads in flight: beat %0d: %h", got, m_rdata[63:0]))
                got++;
            end
        end
        `CHECK(s_ud0.n_ar == 64, $sformatf("reads in flight: %0d of 64 reads reached axim_udata_0", s_ud0.n_ar))
        $display("ok   reads in flight: all 64 answered in order");
    end

    if (errors == 0) $display("TB PASS (tb_shell_ctrl_uwin, %0d regions)", NREG);
    else             $display("TB FAIL (tb_shell_ctrl_uwin): %0d errors", errors);
    $finish;
end

initial begin
    #200us;
    $display("TB FAIL (tb_shell_ctrl_uwin): timeout");
    $finish;
end

endmodule
