// Loopback DUT: encrypt bank feeding decrypt bank, wired the way the two
// nodes are in vfpga_top.svh.
//
// Node A's TX bank uses iv_dir = role and node B's RX bank uses ~role, so for
// role 0 both ends sit in IV space 0 -- which is what this wires up. The
// ciphertext between the banks is tapped out for the testbench to check
// against a software AES-GCM, so a self-consistent-but-wrong pipeline cannot
// pass.

`timescale 1ns / 1ps

module tb_bank_loopback #(
    parameter AXI_DATA_WIDTH  = 512,
    parameter KEEP_WIDTH      = AXI_DATA_WIDTH/8,
    parameter NUM_AES_ENGINES = 2
) (
    input  wire                      clk,
    input  wire                      rst,
    input  wire                      iv_dir,

    // plaintext in (to the encrypt bank)
    input  wire [AXI_DATA_WIDTH-1:0] s_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]     s_axis_tkeep,
    input  wire                      s_axis_tvalid,
    output wire                      s_axis_tready,
    input  wire                      s_axis_tlast,

    // plaintext out (from the decrypt bank)
    output wire [AXI_DATA_WIDTH-1:0] m_axis_tdata,
    output wire [KEEP_WIDTH-1:0]     m_axis_tkeep,
    output wire                      m_axis_tvalid,
    input  wire                      m_axis_tready,
    output wire                      m_axis_tlast,

    // ciphertext tap between the banks (monitor only)
    output wire [AXI_DATA_WIDTH-1:0] mid_tdata,
    output wire [KEEP_WIDTH-1:0]     mid_tkeep,
    output wire                      mid_tvalid,
    output wire                      mid_tready,
    output wire                      mid_tlast,

    output wire [2:0]                tag_ok_cnt,
    output wire                      quarantine,

    // Debug taps into decrypt engine 0 (hierarchical refs, simulation only)
    output wire [3:0]                dbg_credits,
    output wire                      dbg_pause,
    output wire                      dbg_tag_ok0,
    output wire                      dbg_eng_out_tvalid,
    output wire                      dbg_eng_out_tready,
    output wire                      dbg_eng_out_tlast,
    output wire                      dbg_is_tag,
    output wire [127:0]              dbg_computed_tag,
    output wire [127:0]              dbg_expected_tag,
    output wire [4:0]                dbg_rx_tag_wr,
    output wire [4:0]                dbg_rx_tag_rd,
    output wire                      dbg_ct_tvalid,
    output wire                      dbg_ct_tready,
    output wire                      dbg_fifo_tvalid,
    output wire                      dbg_fifo_tready,
    output wire                      dbg_coll_tvalid,
    output wire                      dbg_coll_tready
);

    assign dbg_credits        = dec.engine[0].tag_credits;
    assign dbg_pause          = dec.engine[0].pause;
    assign dbg_tag_ok0        = dec.eng_tag_ok[0];
    assign dbg_eng_out_tvalid = dec.engine[0].eng_out_tvalid;
    assign dbg_eng_out_tready = dec.engine[0].eng_out_tready;
    assign dbg_eng_out_tlast  = dec.engine[0].eng_out_tlast;
    assign dbg_is_tag         = dec.engine[0].dec.dec_module.eng_out_is_tag;
    assign dbg_computed_tag   = dec.engine[0].dec.dec_module.eng_out_tdata_le;
    assign dbg_expected_tag   = dec.engine[0].dec.dec_module.rx_tag_mem[dec.engine[0].dec.dec_module.rx_tag_rd[3:0]];
    assign dbg_rx_tag_wr      = dec.engine[0].dec.dec_module.rx_tag_wr;
    assign dbg_rx_tag_rd      = dec.engine[0].dec.dec_module.rx_tag_rd;
    assign dbg_ct_tvalid      = dec.engine[0].dec.dec_module.ct_tvalid;
    assign dbg_ct_tready      = dec.engine[0].dec.dec_module.ct_tready;
    assign dbg_fifo_tvalid    = dec.engine[0].fifo_tvalid;
    assign dbg_fifo_tready    = dec.engine[0].fifo_tready;
    assign dbg_coll_tvalid    = dec.coll_tvalid[0];
    assign dbg_coll_tready    = dec.coll_tready[0];

    wire [AXI_DATA_WIDTH-1:0] ct_tdata;
    wire [KEEP_WIDTH-1:0]     ct_tkeep;
    wire                      ct_tvalid;
    wire                      ct_tready;
    wire                      ct_tlast;

    assign mid_tdata  = ct_tdata;
    assign mid_tkeep  = ct_tkeep;
    assign mid_tvalid = ct_tvalid;
    assign mid_tready = ct_tready;
    assign mid_tlast  = ct_tlast;

    aes_gcm_bank #(
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .KEEP_WIDTH(KEEP_WIDTH),
        .NUM_AES_ENGINES(NUM_AES_ENGINES),
        .DIRECTION(0)
    ) enc (
        .clk(clk),
        .rst(rst),
        .iv_dir(iv_dir),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tkeep(s_axis_tkeep),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tready(s_axis_tready),
        .s_axis_tlast(s_axis_tlast),
        .m_axis_tdata(ct_tdata),
        .m_axis_tkeep(ct_tkeep),
        .m_axis_tvalid(ct_tvalid),
        .m_axis_tready(ct_tready),
        .m_axis_tlast(ct_tlast),
        .tag_ok_cnt(),
        .tag_quarantine()
    );

    aes_gcm_bank #(
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .KEEP_WIDTH(KEEP_WIDTH),
        .NUM_AES_ENGINES(NUM_AES_ENGINES),
        .DIRECTION(1)
    ) dec (
        .clk(clk),
        .rst(rst),
        .iv_dir(iv_dir),
        .s_axis_tdata(ct_tdata),
        .s_axis_tkeep(ct_tkeep),
        .s_axis_tvalid(ct_tvalid),
        .s_axis_tready(ct_tready),
        .s_axis_tlast(ct_tlast),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tkeep(m_axis_tkeep),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready),
        .m_axis_tlast(m_axis_tlast),
        .tag_ok_cnt(tag_ok_cnt),
        .tag_quarantine(quarantine)
    );

endmodule
