// Streaming AES-256-GCM engine with zero per-packet pipeline flush.
//
// The AES-ECB pipe (aes_ecb_14, 56-cycle latency, 1 block/cycle) is never
// drained: exactly one counter block is issued per accepted input beat, so
// no speculative keystream ever exists and packets butt-joint seamlessly.
// The input data rides alongside in a delay FIFO and meets its keystream at
// the pipe output, where the XOR and GHASH happen. Each issued block carries
// a type tag (H / J0 / DATA) through a side FIFO so the output side knows
// what emerges.
//
// Per-packet cost: one J0 slot at the input plus one GHASH length-block
// cycle and one tag beat at the output. The 56-cycle transit is latency
// only, paid once per beat, never as a throughput bubble.
//
// IVs increment per packet: packet p of this engine uses IV_INIT +
// p*IV_STRIDE, so striped engines (engine n of N: IV_INIT=n, IV_STRIDE=N)
// jointly cover the global packet sequence 0,1,2,... without coordination.
// GCM requires unique (key, IV) pairs; encrypt and decrypt directions must
// use disjoint IV spaces (here: the runtime iv_dir input drives IV bit 95).
// LOCAL PATCH vs corundum sec-enc-dec: iv_dir was a constant in IV_INIT.
//
// Interface is in the GCM core's big-endian domain (byte 0 = bits 127:120,
// tbval contiguous from bit 15). For decryption the received tag beat must
// be stripped by the wrapper; the engine emits plaintext/ciphertext beats
// followed by one tag beat (m_is_tag, m_tlast).

module aes_gcm_stream #(
    parameter [255:0] KEY = 256'h0,
    parameter [95:0] IV_INIT = 96'h0,
    parameter [95:0] IV_STRIDE = 96'h1
) (
    input  wire clk,
    input  wire rst,

    input  wire enc_dec,   // 0 = encrypt, 1 = decrypt

    // Runtime IV-space selector: drives bit 95 of the per-packet IV.
    // GCM needs disjoint (key, IV) spaces per direction; upstream bakes
    // this into IV_INIT at elaboration, which would force a separate
    // bitstream per node role. Sampled at reset only (iv_q is loaded
    // there), so the vFPGA must hold this block in reset until software
    // has programmed the role CSR.
    input  wire iv_dir,

    input  wire [127:0] s_tdata,
    input  wire [15:0]  s_tbval,
    input  wire         s_tvalid,
    output wire         s_tready,
    input  wire         s_tlast,

    output wire [127:0] m_tdata,
    output wire [15:0]  m_tbval,
    output wire         m_tvalid,
    input  wire         m_tready,
    output wire         m_tlast,
    output wire         m_is_tag
);

    localparam [1:0] TYPE_H    = 2'd0;
    localparam [1:0] TYPE_J0   = 2'd1;
    localparam [1:0] TYPE_DATA = 2'd2;
    localparam [1:0] TYPE_DROP = 2'd3;

    localparam [1:0] IN_KEY  = 2'd0;
    localparam [1:0] IN_H    = 2'd1;
    localparam [1:0] IN_WARM = 2'd2;
    localparam [1:0] IN_RUN  = 2'd3;

    // The ECB's just-in-time key expansion (kexp stage i fires only while
    // a block enters round i as the first block's counter-change passes:
    // see rnd_stage_trg_key in aes_round) requires a continuous block
    // stream while the first post-key block transits the pipe. Issue
    // WARMUP_BLOCKS dropped dummy blocks back-to-back behind H once after
    // key load; afterwards sparse issue is fine as the expanded key is held.
    localparam WARMUP_BLOCKS = 64;

    localparam [1:0] OUT_DATA = 2'd0;
    localparam [1:0] OUT_LEN  = 2'd1;
    localparam [1:0] OUT_TAG  = 2'd2;

    // ------------------------------------------------------------------
    // AES-ECB pipeline (from the generated GCM netlist, key expansion
    // included; loaded once, never reset per packet)
    // ------------------------------------------------------------------

    reg [1:0] in_state;
    reg [3:0] key_word_val;

    wire ecb_busy;
    wire ecb_out_val;
    wire [127:0] ecb_out;

    reg ecb_in_val;
    reg [127:0] ecb_in;
    wire ecb_ack;

    aes_ecb_14 u_aes_ecb (
        .rst_i(rst),
        .clk_i(clk),
        .aes_mode_i(2'b10),
        .aes_key_word_val_i(key_word_val),
        .aes_key_word_i(KEY),
        .aes_pipe_reset_i(1'b0),
        .aes_plain_text_val_i(ecb_in_val),
        .aes_plain_text_i(ecb_in),
        .aes_cipher_text_ack_i(ecb_ack),
        .aes_cipher_text_val_o(ecb_out_val),
        .aes_cipher_text_o(ecb_out),
        .aes_ecb_busy_o(ecb_busy)
    );

    // ------------------------------------------------------------------
    // Issue side: one block into the pipe per accepted beat
    // ------------------------------------------------------------------

    reg [95:0] iv_q;
    reg [31:0] cnt_q;
    reg need_j0;

    // side FIFOs: block type (aligned with every pipe issue) and input
    // data (aligned with DATA issues only)
    localparam FIFO_AW = 7;   // 128 entries >= pipe depth + warmup backlog

    reg [1:0]   type_mem [0:(1<<FIFO_AW)-1];
    reg [FIFO_AW:0] type_wr, type_rd;
    wire [FIFO_AW:0] type_count = type_wr - type_rd;
    wire type_full = type_count >= 100;

    reg [144:0] data_mem [0:(1<<FIFO_AW)-1];  // {last, bval[15:0], data[127:0]}
    reg [FIFO_AW:0] data_wr, data_rd;
    wire [FIFO_AW:0] data_count = data_wr - data_rd;
    wire data_full = data_count >= 60;

    // J0 masks in flight (issued, tag not yet emitted)
    reg [4:0] j0_inflight;
    wire j0_room = j0_inflight < 5'd14;

    reg [6:0] warm_cnt;

    wire issue_h    = (in_state == IN_H) && !ecb_busy && !type_full;
    wire issue_warm = (in_state == IN_WARM) && !ecb_busy && !type_full;
    wire issue_j0   = (in_state == IN_RUN) && need_j0 && !ecb_busy && !type_full && j0_room;
    wire issue_data = s_tvalid && s_tready;

    assign s_tready = (in_state == IN_RUN) && !need_j0 && !ecb_busy && !type_full && !data_full;

    always @(*) begin
        key_word_val = 4'b0000;
        ecb_in_val = 1'b0;
        ecb_in = 128'h0;

        case (in_state)
            IN_KEY: begin
                key_word_val = 4'b1111;
            end
            IN_H: begin
                ecb_in_val = issue_h;
                ecb_in = 128'h0;
            end
            IN_WARM: begin
                ecb_in_val = issue_warm;
                ecb_in = 128'h0;
            end
            default: begin  // IN_RUN
                if (issue_j0) begin
                    ecb_in_val = 1'b1;
                    ecb_in = {iv_q, 32'd1};
                end else if (issue_data) begin
                    ecb_in_val = 1'b1;
                    ecb_in = {iv_q, cnt_q};
                end
            end
        endcase
    end

    wire tag_fire;

    always @(posedge clk) begin
        if (rst) begin
            in_state <= IN_KEY;
            iv_q <= {iv_dir, IV_INIT[94:0]};
            cnt_q <= 32'd2;
            need_j0 <= 1'b1;
            warm_cnt <= 7'd0;
            type_wr <= 0;
            data_wr <= 0;
            j0_inflight <= 5'd0;
        end else begin
            case (in_state)
                IN_KEY: in_state <= IN_H;
                IN_H:   if (issue_h) in_state <= IN_WARM;
                IN_WARM: begin
                    if (issue_warm) begin
                        warm_cnt <= warm_cnt + 1;
                        if (warm_cnt == WARMUP_BLOCKS-1) begin
                            in_state <= IN_RUN;
                        end
                    end
                end
                default: ;
            endcase

            if (issue_h || issue_warm || issue_j0 || issue_data) begin
                type_mem[type_wr[FIFO_AW-1:0]] <= issue_h ? TYPE_H :
                                                  (issue_warm ? TYPE_DROP :
                                                  (issue_j0 ? TYPE_J0 : TYPE_DATA));
                type_wr <= type_wr + 1;
            end

            if (issue_j0) begin
                need_j0 <= 1'b0;
                cnt_q <= 32'd2;
            end

            if (issue_data) begin
                data_mem[data_wr[FIFO_AW-1:0]] <= {s_tlast, s_tbval, s_tdata};
                data_wr <= data_wr + 1;
                cnt_q <= cnt_q + 1;
                if (s_tlast) begin
                    iv_q <= iv_q + IV_STRIDE;
                    need_j0 <= 1'b1;
                end
            end

            case ({issue_j0, tag_fire})
                2'b10: j0_inflight <= j0_inflight + 1;
                2'b01: j0_inflight <= j0_inflight - 1;
                default: ;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Output side: XOR with keystream, GHASH, tag
    // ------------------------------------------------------------------

    reg [1:0] out_state;
    reg [127:0] h_q;
    reg [127:0] y_q;
    // Supports up to 1 MiB of ciphertext per packet (GCM itself allows
    // ~64 GiB; the practical bound is the decrypt-side store-and-forward
    // FIFO, see jigsaw_pkt_processor). A packet exceeding this wraps the
    // counter and fails authentication.
    reg [20:0] ct_len_bytes;

    // small J0 store (per-packet tag masks, in packet order)
    reg [127:0] j0_mem [0:15];
    reg [4:0] j0_wr, j0_rd;

    wire [1:0] head_type = type_mem[type_rd[FIFO_AW-1:0]];
    wire [144:0] head_data = data_mem[data_rd[FIFO_AW-1:0]];
    wire head_last = head_data[144];
    wire [15:0] head_bval = head_data[143:128];
    wire [127:0] head_pdata = head_data[127:0];

    // contiguous-from-MSB byte-valid decode (same convention as the core)
    reg [4:0] bval_len;
    reg bval_val;
    reg [127:0] bval_mask;
    always @(*) begin
        case (head_bval)
            16'h8000: begin bval_len = 5'd1;  bval_val = 1'b1; bval_mask = 128'hFF000000000000000000000000000000; end
            16'hC000: begin bval_len = 5'd2;  bval_val = 1'b1; bval_mask = 128'hFFFF0000000000000000000000000000; end
            16'hE000: begin bval_len = 5'd3;  bval_val = 1'b1; bval_mask = 128'hFFFFFF00000000000000000000000000; end
            16'hF000: begin bval_len = 5'd4;  bval_val = 1'b1; bval_mask = 128'hFFFFFFFF000000000000000000000000; end
            16'hF800: begin bval_len = 5'd5;  bval_val = 1'b1; bval_mask = 128'hFFFFFFFFFF0000000000000000000000; end
            16'hFC00: begin bval_len = 5'd6;  bval_val = 1'b1; bval_mask = 128'hFFFFFFFFFFFF00000000000000000000; end
            16'hFE00: begin bval_len = 5'd7;  bval_val = 1'b1; bval_mask = 128'hFFFFFFFFFFFFFF000000000000000000; end
            16'hFF00: begin bval_len = 5'd8;  bval_val = 1'b1; bval_mask = 128'hFFFFFFFFFFFFFFFF0000000000000000; end
            16'hFF80: begin bval_len = 5'd9;  bval_val = 1'b1; bval_mask = 128'hFFFFFFFFFFFFFFFFFF00000000000000; end
            16'hFFC0: begin bval_len = 5'd10; bval_val = 1'b1; bval_mask = 128'hFFFFFFFFFFFFFFFFFFFF000000000000; end
            16'hFFE0: begin bval_len = 5'd11; bval_val = 1'b1; bval_mask = 128'hFFFFFFFFFFFFFFFFFFFFFF0000000000; end
            16'hFFF0: begin bval_len = 5'd12; bval_val = 1'b1; bval_mask = 128'hFFFFFFFFFFFFFFFFFFFFFFFF00000000; end
            16'hFFF8: begin bval_len = 5'd13; bval_val = 1'b1; bval_mask = 128'hFFFFFFFFFFFFFFFFFFFFFFFFFF000000; end
            16'hFFFC: begin bval_len = 5'd14; bval_val = 1'b1; bval_mask = 128'hFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000; end
            16'hFFFE: begin bval_len = 5'd15; bval_val = 1'b1; bval_mask = 128'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00; end
            16'hFFFF: begin bval_len = 5'd16; bval_val = 1'b1; bval_mask = 128'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF; end
            default:  begin bval_len = 5'd0;  bval_val = 1'b0; bval_mask = 128'h0; end
        endcase
    end

    wire head_is_data = ecb_out_val && (head_type == TYPE_DATA);
    wire data_fire = head_is_data && (out_state == OUT_DATA) && m_tready;
    assign tag_fire = (out_state == OUT_TAG) && m_tready;

    // pop H, J0 and warmup blocks as they emerge; DATA pops on emission
    assign ecb_ack = ecb_out_val && (
        (head_type == TYPE_H) ||
        (head_type == TYPE_J0) ||
        (head_type == TYPE_DROP) ||
        data_fire
    );

    wire [127:0] xor_masked = (head_pdata ^ ecb_out) & bval_mask;

    // GHASH: hash the ciphertext (post-XOR on encrypt, input data on
    // decrypt), then the length block; one shared multiplier
    wire [127:0] len_block = {64'h0, 40'h0, ct_len_bytes, 3'h0};
    wire [127:0] ghash_x =
        (out_state == OUT_LEN) ? len_block :
        (enc_dec ? (head_pdata & bval_mask) : xor_masked);
    wire ghash_en = (data_fire && bval_val) || (out_state == OUT_LEN);

    wire [127:0] gf_y;
    ghash_gfmul u_gfmul (
        .gf_mult_h_i(h_q),
        .gf_mult_x_i(y_q ^ ghash_x),
        .gf_mult_y_o(gf_y)
    );

    always @(posedge clk) begin
        if (rst) begin
            out_state <= OUT_DATA;
            h_q <= 128'h0;
            y_q <= 128'h0;
            ct_len_bytes <= 21'd0;
            type_rd <= 0;
            data_rd <= 0;
            j0_wr <= 5'd0;
            j0_rd <= 5'd0;
        end else begin
            if (ecb_ack) begin
                type_rd <= type_rd + 1;
                if (head_type == TYPE_H) begin
                    h_q <= ecb_out;
                end
                if (head_type == TYPE_J0) begin
                    j0_mem[j0_wr[3:0]] <= ecb_out;
                    j0_wr <= j0_wr + 1;
                end
            end

            if (ghash_en) begin
                y_q <= gf_y;
            end

            if (data_fire) begin
                data_rd <= data_rd + 1;
                ct_len_bytes <= ct_len_bytes + bval_len;
                if (head_last) begin
                    out_state <= OUT_LEN;
                end
            end

            if (out_state == OUT_LEN) begin
                out_state <= OUT_TAG;
            end

            if (tag_fire) begin
                out_state <= OUT_DATA;
                y_q <= 128'h0;
                ct_len_bytes <= 21'd0;
                j0_rd <= j0_rd + 1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Output beats: data beats, then the tag beat closing the frame
    // ------------------------------------------------------------------

    wire [127:0] tag_value = y_q ^ j0_mem[j0_rd[3:0]];

    assign m_tvalid = (out_state == OUT_TAG) ? 1'b1 : (head_is_data && (out_state == OUT_DATA));
    assign m_tdata  = (out_state == OUT_TAG) ? tag_value : xor_masked;
    assign m_tbval  = (out_state == OUT_TAG) ? 16'hFFFF : head_bval;
    assign m_tlast  = (out_state == OUT_TAG);
    assign m_is_tag = (out_state == OUT_TAG);

endmodule
