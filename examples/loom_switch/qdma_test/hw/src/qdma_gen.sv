// qdma_hw rev2: stand-in for the Coyote shell on the static's host DMA channels (N_HCHAN = 3).
// C2H (card -> host): per channel, CNT requests {paddr, len} to qdma_wr_wrapper, each followed by its packet on the
//   channel's data stream. Address of packet i = base + (i % wrap) * stride.
// H2C (host -> card): per channel, CNT read requests to qdma_rd_wrapper; the returned data is checked beat by beat.
// Data pattern (both directions, also computed by the host): 32-bit word k of packet i on channel ch =
//   seed + (ch << 24) + (i << 12) + k   (mod 2^32), little endian, word k at byte offset 4k.
import lynxTypes::*;

module qdma_gen #(
  parameter integer N_CH = 3
) (
  input  logic                     clk,
  input  logic                     rstn,

  input  logic                     c2h_go,      // pulse: start C2H on the channels in c2h_mask
  input  logic                     h2c_go,      // pulse: start H2C on the channels in h2c_mask
  input  logic                     abort,       // pulse: issue no further requests (a packet in flight completes)
  input  logic                     clr,         // pulse: clear the status counters
  input  logic [N_CH-1:0]          c2h_mask,
  input  logic [N_CH-1:0]          h2c_mask,
  input  logic [N_CH-1:0][63:0]    c2h_base, h2c_base,
  input  logic [N_CH-1:0][15:0]    c2h_len,  h2c_len,
  input  logic [N_CH-1:0][31:0]    c2h_cnt,  h2c_cnt,
  input  logic [N_CH-1:0][31:0]    c2h_stride, h2c_stride,
  input  logic [N_CH-1:0][31:0]    c2h_wrap, h2c_wrap,
  input  logic [N_CH-1:0][31:0]    c2h_seed, h2c_seed,

  // to qdma_wr_wrapper (s_dma_wr / dyn_in)
  output logic [N_CH-1:0]          wr_valid,
  input  logic [N_CH-1:0]          wr_ready,
  output dma_req_t [N_CH-1:0]      wr_req,
  output logic [N_CH-1:0]          c2h_tvalid,
  input  logic [N_CH-1:0]          c2h_tready,
  output logic [N_CH-1:0][511:0]   c2h_tdata,
  output logic [N_CH-1:0][63:0]    c2h_tkeep,
  output logic [N_CH-1:0]          c2h_tlast,

  // to qdma_rd_wrapper (s_dma_rd / dyn_out)
  output logic [N_CH-1:0]          rd_valid,
  input  logic [N_CH-1:0]          rd_ready,
  output dma_req_t [N_CH-1:0]      rd_req,
  input  logic [N_CH-1:0]          h2c_tvalid,
  output logic [N_CH-1:0]          h2c_tready,
  input  logic [N_CH-1:0][511:0]   h2c_tdata,
  input  logic [N_CH-1:0][63:0]    h2c_tkeep,
  input  logic [N_CH-1:0]          h2c_tlast,

  // per-channel status
  output logic [N_CH-1:0][63:0]    st_c2h_req, st_c2h_pkt, st_c2h_state,
  output logic [N_CH-1:0][63:0]    st_h2c_req, st_h2c_pkt, st_h2c_beat, st_h2c_bad, st_h2c_lasterr, st_h2c_tlast,
  output logic [N_CH-1:0][63:0]    st_h2c_first0, st_h2c_first1, st_h2c_state
);

  function automatic logic [63:0] keep_of(input logic [15:0] len);
    keep_of = (len[5:0] == 6'd0) ? {64{1'b1}} : ({64{1'b1}} >> (7'd64 - len[5:0]));
  endfunction

  for (genvar ch = 0; ch < N_CH; ch++) begin : g_ch
    localparam logic [31:0] CHOFF = 32'(ch) << 24;

    // ------------------------------------------------------------------ C2H source
    typedef enum logic [1:0] {C_IDLE, C_REQ, C_DATA} cst_t;
    cst_t        cst;
    logic [31:0] ci, cslot, cbase;
    logic [63:0] caddr;
    logic [15:0] clen;
    logic [10:0] cbeat, cnb;
    logic        cabort;
    logic [63:0] n_req, n_pkt;
    logic        o_wr_valid, o_tvalid, o_tlast;
    logic [63:0] o_tkeep;
    logic [511:0] o_tdata;
    dma_req_t    o_wr_req;
    wire         c_last    = (cbeat == cnb - 1'b1);
    wire         c_beat_hs = o_tvalid && c2h_tready[ch];

    always_ff @(posedge clk) begin
      if (!rstn) begin
        cst <= C_IDLE; ci <= 0; cslot <= 0; cbase <= 0; caddr <= 0; clen <= 0; cbeat <= 0; cnb <= 1; cabort <= 1'b0;
        n_req <= 0; n_pkt <= 0;
      end else begin
        if (clr) begin n_req <= 0; n_pkt <= 0; end
        if (abort) cabort <= 1'b1;
        case (cst)
          C_IDLE: if (c2h_go && c2h_mask[ch] && c2h_cnt[ch] != 0) begin
            ci <= 0; cslot <= 0; caddr <= c2h_base[ch]; clen <= c2h_len[ch];
            cnb <= 11'((c2h_len[ch] + 16'd63) >> 6); cabort <= 1'b0; cst <= C_REQ;
          end
          C_REQ: if (cabort) cst <= C_IDLE;
            else if (o_wr_valid && wr_ready[ch]) begin
              n_req <= n_req + 1'b1;
              cbeat <= 0; cbase <= c2h_seed[ch] + CHOFF + (ci << 12); cst <= C_DATA;
            end
          C_DATA: if (c_beat_hs) begin
            cbeat <= cbeat + 1'b1; cbase <= cbase + 32'd16;
            if (c_last) begin
              n_pkt <= n_pkt + 1'b1;
              ci <= ci + 1'b1;
              if (cslot + 1 >= c2h_wrap[ch]) begin cslot <= 0; caddr <= c2h_base[ch]; end
              else begin cslot <= cslot + 1'b1; caddr <= caddr + c2h_stride[ch]; end
              cst <= (ci + 1 == c2h_cnt[ch] || cabort || abort) ? C_IDLE : C_REQ;
            end
          end
          default: cst <= C_IDLE;
        endcase
      end
    end

    always_comb begin
      o_wr_valid     = (cst == C_REQ) && !cabort;
      o_wr_req       = '0;
      o_wr_req.paddr = caddr[PADDR_BITS-1:0];
      o_wr_req.len   = clen;
      o_wr_req.last  = 1'b1;
      o_tvalid       = (cst == C_DATA);
      o_tlast        = (cst == C_DATA) && c_last;
      o_tkeep        = c_last ? keep_of(clen) : {64{1'b1}};
      for (int j = 0; j < 16; j++) o_tdata[32*j +: 32] = cbase + j;
    end
    assign wr_valid[ch]     = o_wr_valid;
    assign wr_req[ch]       = o_wr_req;
    assign c2h_tvalid[ch]   = o_tvalid;
    assign c2h_tlast[ch]    = o_tlast;
    assign c2h_tkeep[ch]    = o_tkeep;
    assign c2h_tdata[ch]    = o_tdata;
    assign st_c2h_req[ch]   = n_req;
    assign st_c2h_pkt[ch]   = n_pkt;
    assign st_c2h_state[ch] = {30'd0, cst, ci};

    // ------------------------------------------------------------------ H2C request issuer
    logic        hbusy;
    logic [31:0] hi, hslot;
    logic [63:0] haddr, n_hreq;
    dma_req_t    o_rd_req;
    always_ff @(posedge clk) begin
      if (!rstn) begin
        hbusy <= 1'b0; hi <= 0; hslot <= 0; haddr <= 0; n_hreq <= 0;
      end else begin
        if (clr) n_hreq <= 0;
        if (!hbusy) begin
          if (h2c_go && h2c_mask[ch] && h2c_cnt[ch] != 0) begin hbusy <= 1'b1; hi <= 0; hslot <= 0; haddr <= h2c_base[ch]; end
        end else if (abort) begin
          hbusy <= 1'b0;
        end else if (rd_ready[ch]) begin
          n_hreq <= n_hreq + 1'b1;
          hi <= hi + 1'b1;
          if (hslot + 1 >= h2c_wrap[ch]) begin hslot <= 0; haddr <= h2c_base[ch]; end
          else begin hslot <= hslot + 1'b1; haddr <= haddr + h2c_stride[ch]; end
          if (hi + 1 == h2c_cnt[ch]) hbusy <= 1'b0;
        end
      end
    end
    always_comb begin
      o_rd_req       = '0;
      o_rd_req.paddr = haddr[PADDR_BITS-1:0];
      o_rd_req.len   = h2c_len[ch];
      o_rd_req.last  = 1'b1;
    end
    assign rd_valid[ch]     = hbusy;
    assign rd_req[ch]       = o_rd_req;
    assign st_h2c_req[ch]   = n_hreq;
    assign st_h2c_state[ch] = {31'd0, hbusy, hi};

    // ------------------------------------------------------------------ H2C checker (two stages)
    assign h2c_tready[ch] = 1'b1;
    logic [31:0]  ei, ebase;
    logic [10:0]  ebeat, enb;
    logic [15:0]  elen;
    logic         s1_v, s1_tlast, s1_elast;
    logic [511:0] s1_d, s1_exp;
    logic [63:0]  s1_keep, s1_ekeep;
    logic [31:0]  s1_i;
    logic [10:0]  s1_b;
    wire          e_last = (ebeat == enb - 1'b1);
    // stage 1: register the beat and what was expected for it
    always_ff @(posedge clk) begin
      if (!rstn) begin
        ei <= 0; ebeat <= 0; ebase <= 0; enb <= 1; elen <= 0; s1_v <= 1'b0;
      end else begin
        s1_v <= h2c_tvalid[ch];
        if (h2c_go && h2c_mask[ch]) begin
          ei <= 0; ebeat <= 0; ebase <= h2c_seed[ch] + CHOFF; elen <= h2c_len[ch];
          enb <= 11'((h2c_len[ch] + 16'd63) >> 6);
        end else if (h2c_tvalid[ch]) begin
          if (e_last) begin ei <= ei + 1'b1; ebeat <= 0; ebase <= h2c_seed[ch] + CHOFF + ((ei + 1'b1) << 12); end
          else begin ebeat <= ebeat + 1'b1; ebase <= ebase + 32'd16; end
        end
      end
      s1_d <= h2c_tdata[ch]; s1_keep <= h2c_tkeep[ch]; s1_tlast <= h2c_tlast[ch];
      s1_elast <= e_last; s1_ekeep <= e_last ? keep_of(elen) : {64{1'b1}}; s1_i <= ei; s1_b <= ebeat;
      for (int j = 0; j < 16; j++) s1_exp[32*j +: 32] <= ebase + j;
    end
    // stage 2: compare the bytes the expected keep marks, plus keep and last
    logic bad_data;
    always_comb begin
      bad_data = 1'b0;
      for (int b = 0; b < 64; b++) if (s1_ekeep[b] && (s1_d[8*b +: 8] != s1_exp[8*b +: 8])) bad_data = 1'b1;
    end
    wire bad_frame = (s1_keep != s1_ekeep) || (s1_tlast != s1_elast);
    logic        first_seen;
    logic [63:0] n_pkt_rx, n_beat_rx, n_bad, n_lasterr, n_tlast, first0, first1;
    always_ff @(posedge clk) begin
      if (!rstn || clr) begin
        n_pkt_rx <= 0; n_beat_rx <= 0; n_bad <= 0; n_lasterr <= 0; n_tlast <= 0; first0 <= 0; first1 <= 0;
        first_seen <= 1'b0;
      end else if (s1_v) begin
        n_beat_rx <= n_beat_rx + 1'b1;
        if (s1_elast) n_pkt_rx <= n_pkt_rx + 1'b1;
        if (s1_tlast) n_tlast <= n_tlast + 1'b1;
        if (bad_data) n_bad <= n_bad + 1'b1;
        if (bad_frame) n_lasterr <= n_lasterr + 1'b1;
        if ((bad_data || bad_frame) && !first_seen) begin
          first_seen <= 1'b1;
          // [63] valid, [50] keep/last wrong, [49] data wrong, [48] tlast seen, [42:32] beat, [31:0] packet
          first0 <= {1'b1, 12'd0, bad_frame, bad_data, s1_tlast, 5'd0, s1_b, s1_i};
          first1 <= {s1_exp[31:0], s1_d[31:0]};
        end
      end
    end
    assign st_h2c_pkt[ch]     = n_pkt_rx;
    assign st_h2c_beat[ch]    = n_beat_rx;
    assign st_h2c_bad[ch]     = n_bad;
    assign st_h2c_lasterr[ch] = n_lasterr;
    assign st_h2c_tlast[ch]   = n_tlast;
    assign st_h2c_first0[ch]  = first0;
    assign st_h2c_first1[ch]  = first1;
  end
endmodule
