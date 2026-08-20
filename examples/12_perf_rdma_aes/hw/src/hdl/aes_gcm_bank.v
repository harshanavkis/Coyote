// A bank of NUM_AES_ENGINES streaming AES-256-GCM engines on a 512-bit
// AXI-Stream, striped round-robin over whole frames.
//
//   s_axis -> demux -> N x [ fifo 512->128 -> (strip tail) -> engine ->
//             fifo 128->512 ] -> mux -> m_axis
//
// This is corundum sec-enc-dec's jigsaw_pkt_processor.v cut down to a single
// direction (upstream fuses decrypt + txn_generator + encrypt, which only fits
// a NIC-resident pipeline). Demux and mux latch their select at frame
// boundaries and both counters advance once per frame, so order is preserved
// without a reorder buffer. Frame length is preserved either way -- the tail
// beat is a pad slot on encrypt and the received tag on decrypt.
//
// Decrypt releases a frame only once its tag verifies: one credit per verified
// frame, consumed as the frame leaves. A frame that never verifies keeps
// credits at zero and stays quarantined (upstream's policy). That makes the
// decrypt output FIFO store-and-forward, so it must hold a whole fragment.

`timescale 1ns / 1ps

module aes_gcm_bank #(
    parameter AXI_DATA_WIDTH  = 512,
    parameter KEEP_WIDTH      = AXI_DATA_WIDTH/8,
    parameter NUM_AES_ENGINES = 2,
    // 0 = encrypt, 1 = decrypt
    parameter DIRECTION       = 0,
    // Both in bytes. The output FIFO on the decrypt side is store-and-
    // forward (tag gating), so it bounds the largest decryptable frame;
    // Coyote fragments at PMTU_BYTES, so PMTU + margin is enough.
    parameter IN_FIFO_DEPTH   = 8192,
    parameter OUT_FIFO_DEPTH  = 8192,
    parameter TAG_CNT_W       = $clog2(NUM_AES_ENGINES + 1)
) (
    input  wire                      clk,
    input  wire                      rst,

    // IV-space selector, forwarded to every engine. Sampled at reset.
    input  wire                      iv_dir,

    input  wire [AXI_DATA_WIDTH-1:0] s_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]     s_axis_tkeep,
    input  wire                      s_axis_tvalid,
    output wire                      s_axis_tready,
    input  wire                      s_axis_tlast,

    output wire [AXI_DATA_WIDTH-1:0] m_axis_tdata,
    output wire [KEEP_WIDTH-1:0]     m_axis_tkeep,
    output wire                      m_axis_tvalid,
    input  wire                      m_axis_tready,
    output wire                      m_axis_tlast,

    // Status (decrypt only; tied low when DIRECTION == 0).
    // tag_ok_cnt is how many engines verified a tag THIS cycle, not a pulse:
    // with several engines running in parallel two can verify on the same
    // cycle, and an OR would silently undercount.
    output wire [TAG_CNT_W-1:0]      tag_ok_cnt,
    output wire                      tag_quarantine    // a frame is withheld
);

    localparam CL_ENG = (NUM_AES_ENGINES > 1) ? $clog2(NUM_AES_ENGINES) : 1;
    // Matches axis_demux's S_DEST_WIDTH = M_DEST_WIDTH + $clog2(M_COUNT)
    localparam DEMUX_S_DEST_W = 8 + ((NUM_AES_ENGINES > 1) ? $clog2(NUM_AES_ENGINES) : 0);

    genvar n;

    // ---------------------------------------------------------------
    // Round-robin dispatch, one engine per frame
    // ---------------------------------------------------------------
    wire [NUM_AES_ENGINES*AXI_DATA_WIDTH-1:0] disp_tdata;
    wire [NUM_AES_ENGINES*KEEP_WIDTH-1:0]     disp_tkeep;
    wire [NUM_AES_ENGINES-1:0]                disp_tvalid;
    wire [NUM_AES_ENGINES-1:0]                disp_tready;
    wire [NUM_AES_ENGINES-1:0]                disp_tlast;

    generate
    if (NUM_AES_ENGINES == 1) begin : no_dispatch

        assign disp_tdata    = s_axis_tdata;
        assign disp_tkeep    = s_axis_tkeep;
        assign disp_tvalid   = s_axis_tvalid;
        assign disp_tlast    = s_axis_tlast;
        assign s_axis_tready = disp_tready[0];

    end else begin : dispatch_rr

        reg [CL_ENG-1:0] disp_sel;

        always @(posedge clk) begin
            if (rst) begin
                disp_sel <= 0;
            end else if (s_axis_tvalid && s_axis_tready && s_axis_tlast) begin
                disp_sel <= (disp_sel == NUM_AES_ENGINES-1) ? 0 : disp_sel + 1;
            end
        end

        axis_demux #(
            .M_COUNT(NUM_AES_ENGINES),
            .DATA_WIDTH(AXI_DATA_WIDTH),
            .KEEP_ENABLE(1),
            .KEEP_WIDTH(KEEP_WIDTH),
            .ID_ENABLE(0),
            .DEST_ENABLE(0),
            .USER_ENABLE(0)
        ) dispatch (
            .clk(clk),
            .rst(rst),
            .s_axis_tdata(s_axis_tdata),
            .s_axis_tkeep(s_axis_tkeep),
            .s_axis_tvalid(s_axis_tvalid),
            .s_axis_tready(s_axis_tready),
            .s_axis_tlast(s_axis_tlast),
            .s_axis_tid(8'h0),
            .s_axis_tdest({DEMUX_S_DEST_W{1'b0}}),
            .s_axis_tuser(1'b0),
            .m_axis_tdata(disp_tdata),
            .m_axis_tkeep(disp_tkeep),
            .m_axis_tvalid(disp_tvalid),
            .m_axis_tready(disp_tready),
            .m_axis_tlast(disp_tlast),
            .m_axis_tid(),
            .m_axis_tdest(),
            .m_axis_tuser(),
            .enable(1'b1),
            .drop(1'b0),
            .select(disp_sel)
        );

    end
    endgenerate

    wire [NUM_AES_ENGINES*AXI_DATA_WIDTH-1:0] coll_tdata;
    wire [NUM_AES_ENGINES*KEEP_WIDTH-1:0]     coll_tkeep;
    wire [NUM_AES_ENGINES-1:0]                coll_tvalid;
    wire [NUM_AES_ENGINES-1:0]                coll_tready;
    wire [NUM_AES_ENGINES-1:0]                coll_tlast;

    wire [NUM_AES_ENGINES-1:0] eng_tag_ok;
    wire [NUM_AES_ENGINES-1:0] eng_paused;

    generate
        for (n = 0; n < NUM_AES_ENGINES; n = n + 1) begin : engine

            // ----- 512 -> 128 -----
            wire [127:0] fifo_tdata;
            wire [15:0]  fifo_tkeep;
            wire         fifo_tvalid;
            wire         fifo_tready;
            wire         fifo_tlast;

            axis_fifo_adapter #(
                .DEPTH(IN_FIFO_DEPTH),
                .S_DATA_WIDTH(AXI_DATA_WIDTH),
                .S_KEEP_ENABLE(1),
                .S_KEEP_WIDTH(KEEP_WIDTH),
                .M_DATA_WIDTH(128),
                .M_KEEP_ENABLE(1),
                .M_KEEP_WIDTH(16),
                .ID_ENABLE(0),
                .DEST_ENABLE(0),
                .USER_ENABLE(0)
            ) in_fifo (
                .clk(clk),
                .rst(rst),
                .s_axis_tdata(disp_tdata[n*AXI_DATA_WIDTH +: AXI_DATA_WIDTH]),
                .s_axis_tkeep(disp_tkeep[n*KEEP_WIDTH +: KEEP_WIDTH]),
                .s_axis_tvalid(disp_tvalid[n]),
                .s_axis_tready(disp_tready[n]),
                .s_axis_tlast(disp_tlast[n]),
                .s_axis_tid(8'h0),
                .s_axis_tdest(8'h0),
                .s_axis_tuser(1'b0),
                .m_axis_tdata(fifo_tdata),
                .m_axis_tkeep(fifo_tkeep),
                .m_axis_tvalid(fifo_tvalid),
                .m_axis_tready(fifo_tready),
                .m_axis_tlast(fifo_tlast),
                .m_axis_tid(),
                .m_axis_tdest(),
                .m_axis_tuser(),
                .pause_req(1'b0),
                .pause_ack(),
                .status_depth(),
                .status_depth_commit(),
                .status_overflow(),
                .status_bad_frame(),
                .status_good_frame()
            );

            // ----- AES engine -----
            wire [127:0] eng_out_tdata;
            wire [15:0]  eng_out_tkeep;
            wire         eng_out_tvalid;
            wire         eng_out_tready;
            wire         eng_out_tlast;

            if (DIRECTION == 0) begin : enc

                // Drop the 16-byte pad slot; the engine's tag beat refills it.
                wire [127:0] pt_tdata;
                wire [15:0]  pt_tkeep;
                wire         pt_tvalid;
                wire         pt_tready;
                wire         pt_tlast;

                axis_strip_tail_beat #(
                    .DATA_WIDTH(128),
                    .KEEP_WIDTH(16)
                ) pad_strip (
                    .clk(clk),
                    .rst(rst),
                    .s_axis_tdata(fifo_tdata),
                    .s_axis_tkeep(fifo_tkeep),
                    .s_axis_tvalid(fifo_tvalid),
                    .s_axis_tready(fifo_tready),
                    .s_axis_tlast(fifo_tlast),
                    .m_axis_tdata(pt_tdata),
                    .m_axis_tkeep(pt_tkeep),
                    .m_axis_tvalid(pt_tvalid),
                    .m_axis_tready(pt_tready),
                    .m_axis_tlast(pt_tlast),
                    .tail_tdata(),
                    .tail_tvalid(),
                    .tail_tready(1'b1)
                );

                aes_gcm_encryption #(
                    .IV_INIT(96'h0 + n),
                    .IV_STRIDE(NUM_AES_ENGINES)
                ) enc_module (
                    .clk(clk),
                    .rst(rst),
                    .enc_dec(1'b0),
                    .iv_dir(iv_dir),
                    .aes_in_tdata(pt_tdata),
                    .aes_in_tkeep(pt_tkeep),
                    .aes_in_tvalid(pt_tvalid),
                    .aes_in_tready(pt_tready),
                    .aes_in_tlast(pt_tlast),
                    .aes_in_tuser(1'b0),
                    .aes_out_tdata(eng_out_tdata),
                    .aes_out_tkeep(eng_out_tkeep),
                    .aes_out_tvalid(eng_out_tvalid),
                    .aes_out_tready(eng_out_tready),
                    .aes_out_tlast(eng_out_tlast),
                    .aes_out_tuser()
                );

                assign eng_tag_ok[n] = 1'b0;

            end else begin : dec

                aes_gcm_decryption_taglast #(
                    .IV_INIT(96'h0 + n),
                    .IV_STRIDE(NUM_AES_ENGINES)
                ) dec_module (
                    .clk(clk),
                    .rst(rst),
                    .iv_dir(iv_dir),
                    .aes_in_tdata(fifo_tdata),
                    .aes_in_tkeep(fifo_tkeep),
                    .aes_in_tvalid(fifo_tvalid),
                    .aes_in_tready(fifo_tready),
                    .aes_in_tlast(fifo_tlast),
                    .aes_in_tuser(1'b0),
                    .aes_out_tdata(eng_out_tdata),
                    .aes_out_tkeep(eng_out_tkeep),
                    .aes_out_tvalid(eng_out_tvalid),
                    .aes_out_tready(eng_out_tready),
                    .aes_out_tlast(eng_out_tlast),
                    .aes_out_tuser(),
                    .ghash_tag_val(eng_tag_ok[n])
                );

            end

            // ----- 128 -> 512, tag-gated on the decrypt side -----
            reg [3:0] tag_credits;
            wire frame_out = coll_tvalid[n] && coll_tready[n] && coll_tlast[n];

            always @(posedge clk) begin
                if (rst) begin
                    tag_credits <= 4'd0;
                end else begin
                    case ({eng_tag_ok[n], frame_out})
                        2'b10:   tag_credits <= tag_credits + 4'd1;
                        2'b01:   tag_credits <= tag_credits - 4'd1;
                        default: tag_credits <= tag_credits;
                    endcase
                end
            end

            wire pause = (DIRECTION == 0) ? 1'b0 : (tag_credits == 4'd0);

            // credits == 0 is also the idle state, so pause alone would report
            // a quarantine permanently. Only flag it when a frame is actually
            // buffered and being withheld.
            reg [3:0] outstanding;
            wire frame_in = eng_out_tvalid && eng_out_tready && eng_out_tlast;

            always @(posedge clk) begin
                if (rst) begin
                    outstanding <= 4'd0;
                end else begin
                    case ({frame_in, frame_out})
                        2'b10:   outstanding <= outstanding + 4'd1;
                        2'b01:   outstanding <= outstanding - 4'd1;
                        default: outstanding <= outstanding;
                    endcase
                end
            end

            assign eng_paused[n] = pause && (outstanding != 4'd0);

            axis_fifo_adapter #(
                .DEPTH(OUT_FIFO_DEPTH),
                .S_DATA_WIDTH(128),
                .S_KEEP_ENABLE(1),
                .S_KEEP_WIDTH(16),
                .M_DATA_WIDTH(AXI_DATA_WIDTH),
                .M_KEEP_ENABLE(1),
                .M_KEEP_WIDTH(KEEP_WIDTH),
                .ID_ENABLE(0),
                .DEST_ENABLE(0),
                .USER_ENABLE(0),
                .PAUSE_ENABLE(1)
            ) out_fifo (
                .clk(clk),
                .rst(rst),
                .s_axis_tdata(eng_out_tdata),
                .s_axis_tkeep(eng_out_tkeep),
                .s_axis_tvalid(eng_out_tvalid),
                .s_axis_tready(eng_out_tready),
                .s_axis_tlast(eng_out_tlast),
                .s_axis_tid(8'h0),
                .s_axis_tdest(8'h0),
                .s_axis_tuser(1'b0),
                .m_axis_tdata(coll_tdata[n*AXI_DATA_WIDTH +: AXI_DATA_WIDTH]),
                .m_axis_tkeep(coll_tkeep[n*KEEP_WIDTH +: KEEP_WIDTH]),
                .m_axis_tvalid(coll_tvalid[n]),
                .m_axis_tready(coll_tready[n]),
                .m_axis_tlast(coll_tlast[n]),
                .m_axis_tid(),
                .m_axis_tdest(),
                .m_axis_tuser(),
                .pause_req(pause),
                .pause_ack(),
                .status_depth(),
                .status_depth_commit(),
                .status_overflow(),
                .status_bad_frame(),
                .status_good_frame()
            );

        end
    endgenerate

    // ---------------------------------------------------------------
    // In-order collect
    // ---------------------------------------------------------------
    generate
    if (NUM_AES_ENGINES == 1) begin : no_collect

        assign m_axis_tdata  = coll_tdata;
        assign m_axis_tkeep  = coll_tkeep;
        assign m_axis_tvalid = coll_tvalid[0];
        assign m_axis_tlast  = coll_tlast[0];
        assign coll_tready   = m_axis_tready;

    end else begin : collect_rr

    reg [CL_ENG-1:0] coll_sel;

    // Advance on the mux's INPUT handshake, not its output. axis_mux latches
    // `select` when it internally starts a frame and its output is a further
    // register stage behind that, so for back-to-back frames it would sample
    // `select` before the previous frame's output tlast had advanced this
    // counter -- and re-select the same engine. Counting the selected port's
    // own end-of-frame matches the mux's internal detection exactly.
    wire coll_done = coll_tvalid[coll_sel] && coll_tready[coll_sel] && coll_tlast[coll_sel];

    always @(posedge clk) begin
        if (rst) begin
            coll_sel <= 0;
        end else if (coll_done) begin
            coll_sel <= (coll_sel == NUM_AES_ENGINES-1) ? 0 : coll_sel + 1;
        end
    end

    axis_mux #(
        .S_COUNT(NUM_AES_ENGINES),
        .DATA_WIDTH(AXI_DATA_WIDTH),
        .KEEP_ENABLE(1),
        .KEEP_WIDTH(KEEP_WIDTH),
        .ID_ENABLE(0),
        .DEST_ENABLE(0),
        .USER_ENABLE(0)
    ) collect (
        .clk(clk),
        .rst(rst),
        .s_axis_tdata(coll_tdata),
        .s_axis_tkeep(coll_tkeep),
        .s_axis_tvalid(coll_tvalid),
        .s_axis_tready(coll_tready),
        .s_axis_tlast(coll_tlast),
        .s_axis_tid({(NUM_AES_ENGINES*8){1'b0}}),
        .s_axis_tdest({(NUM_AES_ENGINES*8){1'b0}}),
        .s_axis_tuser({NUM_AES_ENGINES{1'b0}}),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tkeep(m_axis_tkeep),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready),
        .m_axis_tlast(m_axis_tlast),
        .m_axis_tid(),
        .m_axis_tdest(),
        .m_axis_tuser(),
        .enable(1'b1),
        .select(coll_sel)
    );

    end
    endgenerate

    integer ok_i;
    reg [TAG_CNT_W-1:0] ok_sum;
    always @* begin
        ok_sum = {TAG_CNT_W{1'b0}};
        for (ok_i = 0; ok_i < NUM_AES_ENGINES; ok_i = ok_i + 1) begin
            ok_sum = ok_sum + eng_tag_ok[ok_i];
        end
    end

    assign tag_ok_cnt     = ok_sum;
    assign tag_quarantine = |eng_paused;

endmodule
