// qdma_hw rev2: the QDMA AXI bridge master (BAR4, 256 MB).
// Outside the window (AXI address bits [27:20] != 0xFF): rev1's always-ready sink and its counter page
//   (reads: 64-bit counter index araddr[7:3], araddr[11] = clear; see rev1 axib_sink_cnt.sv for the list).
// Window 0x0FF0_0000..0x0FFF_FFFF:
//   writes: control registers ctl[0..63] = {awaddr[8:6], 64-bit lane of the beat}, byte enables honoured;
//           ctl_wr[i] pulses for one cycle when register i is written.
//   reads:  araddr[11] = 0 -> stat[araddr[10:3]] (NSTAT 64-bit status words); araddr[11] = 1 -> ctl[araddr[8:3]].
// Every read returns its 64-bit word replicated over the 512-bit beat.
module bar4_ctl #(
  parameter integer ID_W = 4, parameter integer DATA_W = 512, parameter integer ADDR_W = 64,
  parameter integer NCTL = 64, parameter integer NSTAT = 128
) (
  input  wire              clk,
  input  wire              rstn,
  input  wire [ID_W-1:0]   awid,
  input  wire [ADDR_W-1:0] awaddr,
  input  wire [7:0]        awlen,
  input  wire              awvalid,
  output wire              awready,
  input  wire [DATA_W-1:0] wdata,
  input  wire [DATA_W/8-1:0] wstrb,
  input  wire              wlast,
  input  wire              wvalid,
  output wire              wready,
  output reg  [ID_W-1:0]   bid,
  output wire [1:0]        bresp,
  output reg               bvalid,
  input  wire              bready,
  input  wire [ID_W-1:0]   arid,
  input  wire [ADDR_W-1:0] araddr,
  input  wire [7:0]        arlen,
  input  wire              arvalid,
  output wire              arready,
  output reg  [ID_W-1:0]   rid,
  output wire [DATA_W-1:0] rdata,
  output wire [1:0]        rresp,
  output reg               rlast,
  output reg               rvalid,
  input  wire              rready,

  output reg  [63:0]       ctl    [0:NCTL-1],
  output reg  [NCTL-1:0]   ctl_wr,
  input  wire [63:0]       stat   [0:NSTAT-1]
);
  localparam [63:0] MAGIC = 64'h51DA_0C0F_FEE0_0001;

  assign awready = 1'b1;
  assign wready  = 1'b1;
  assign bresp   = 2'b00;
  assign rresp   = 2'b00;

  wire aw_hs = awvalid;
  wire w_hs  = wvalid;
  wire b_hs  = bvalid & bready;
  wire ar_hs = arvalid & arready;
  wire aw_win = (awaddr[27:20] == 8'hFF);
  wire ar_win = (araddr[27:20] == 8'hFF);

  // ---- B channel: ID FIFO (bridge allows 32 writes outstanding), count of finished bursts not yet answered
  reg  [ID_W-1:0] idq [0:63];
  reg  [5:0]      idq_wp, idq_rp;
  reg  [6:0]      idq_n;
  reg  [6:0]      wl_pend;
  wire            b_fire = (!bvalid || bready) && (idq_n != 0) && (wl_pend != 0);

  always @(posedge clk) begin
    if (aw_hs) idq[idq_wp] <= awid;
  end

  always @(posedge clk) begin
    if (!rstn) begin
      idq_wp <= 0; idq_rp <= 0; idq_n <= 0; wl_pend <= 0; bvalid <= 1'b0; bid <= '0;
    end else begin
      if (aw_hs) idq_wp <= idq_wp + 1'b1;
      if (b_fire) idq_rp <= idq_rp + 1'b1;
      idq_n   <= idq_n   + (aw_hs ? 7'd1 : 7'd0) - (b_fire ? 7'd1 : 7'd0);
      wl_pend <= wl_pend + ((w_hs && wlast) ? 7'd1 : 7'd0) - (b_fire ? 7'd1 : 7'd0);
      if (b_fire) begin bvalid <= 1'b1; bid <= idq[idq_rp]; end
      else if (bready) bvalid <= 1'b0;
    end
  end

  // ---- W beat -> AW pairing for the window: FIFO of {in window, line} per AW, popped at wlast
  reg  [3:0]  awq [0:63];
  reg  [5:0]  awq_wp, awq_rp;
  reg  [6:0]  awq_n;
  reg  [2:0]  wline_off;                  // beat index inside the current burst (window bursts are 1 beat)
  wire        awq_empty = (awq_n == 0);
  wire [3:0]  w_head  = awq_empty ? {aw_win, awaddr[8:6]} : awq[awq_rp];
  wire        w_noaw  = w_hs && awq_empty && !aw_hs;   // W before its AW: not expected from the bridge
  wire        w_ctl   = w_hs && !w_noaw && w_head[3];
  wire [2:0]  w_line  = w_head[2:0] + wline_off;
  wire        awq_push = aw_hs && !(awq_empty && w_hs && wlast);   // bypass when AW and its single last W coincide
  wire        awq_pop  = w_hs && wlast && !awq_empty;
  reg  [63:0] c_wnoaw;

  always @(posedge clk) begin
    if (awq_push) awq[awq_wp] <= {aw_win, awaddr[8:6]};
  end

  integer l, b;
  always @(posedge clk) begin
    if (!rstn) begin
      awq_wp <= 0; awq_rp <= 0; awq_n <= 0; wline_off <= 0; ctl_wr <= '0; c_wnoaw <= 0;
      for (l = 0; l < NCTL; l = l + 1) ctl[l] <= 64'd0;
    end else begin
      if (awq_push) awq_wp <= awq_wp + 1'b1;
      if (awq_pop)  awq_rp <= awq_rp + 1'b1;
      awq_n <= awq_n + (awq_push ? 7'd1 : 7'd0) - (awq_pop ? 7'd1 : 7'd0);
      if (w_hs) wline_off <= wlast ? 3'd0 : wline_off + 1'b1;
      if (w_noaw) c_wnoaw <= c_wnoaw + 1'b1;
      ctl_wr <= '0;
      if (w_ctl) begin
        for (l = 0; l < 8; l = l + 1) begin
          if (|wstrb[l*8 +: 8]) ctl_wr[{w_line, l[2:0]}] <= 1'b1;
          for (b = 0; b < 8; b = b + 1)
            if (wstrb[l*8 + b]) ctl[{w_line, l[2:0]}][b*8 +: 8] <= wdata[l*64 + b*8 +: 8];
        end
      end
    end
  end

  // ---- sink counters (rev1)
  reg [63:0] c_cyc, c_aw, c_wbeat, c_wlast, c_len0, c_len3, c_lenx, c_bytes, c_tfirst, c_tlast;
  reg [63:0] c_g3, c_g4, c_g5, c_g6, c_g7, c_g8, c_g16, c_gsum, c_b, c_outmax, c_wbub, c_clr;
  reg        have_first, have_prev, inb, clr;
  reg [15:0] gcnt;
  reg [63:0] outst;

  reg [3:0]  pc8_c [0:DATA_W/64-1];
  reg [3:0]  pc8   [0:DATA_W/64-1];
  reg [7:0]  pc_sum_c, pc_sum;
  reg        pc_v, pc_sv;
  integer i, j;
  always @(*) begin
    for (i = 0; i < DATA_W/64; i = i + 1) begin
      pc8_c[i] = 4'd0;
      for (j = 0; j < 8; j = j + 1) pc8_c[i] = pc8_c[i] + wstrb[i*8 + j];
    end
    pc_sum_c = 8'd0;
    for (i = 0; i < DATA_W/64; i = i + 1) pc_sum_c = pc_sum_c + pc8[i];
  end
  always @(posedge clk) begin
    for (i = 0; i < DATA_W/64; i = i + 1) pc8[i] <= w_hs ? pc8_c[i] : 4'd0;
    pc_v   <= w_hs;
    pc_sum <= pc_sum_c;
    pc_sv  <= pc_v;
  end

  wire first_beat = w_hs && !inb;

  always @(posedge clk) begin
    if (!rstn) begin
      c_cyc <= 0; inb <= 1'b0; gcnt <= 16'hFFFF; have_first <= 1'b0; have_prev <= 1'b0; outst <= 0;
      {c_aw, c_wbeat, c_wlast, c_len0, c_len3, c_lenx, c_bytes, c_tfirst, c_tlast} <= '0;
      {c_g3, c_g4, c_g5, c_g6, c_g7, c_g8, c_g16, c_gsum, c_b, c_outmax, c_wbub, c_clr} <= '0;
    end else begin
      c_cyc <= c_cyc + 1'b1;
      if (w_hs) inb <= !wlast;
      gcnt  <= first_beat ? 16'd1 : ((gcnt == 16'hFFFF) ? gcnt : gcnt + 1'b1);
      outst <= outst + (aw_hs ? 64'd1 : 64'd0) - (b_hs ? 64'd1 : 64'd0);
      if (clr) begin
        {c_aw, c_wbeat, c_wlast, c_len0, c_len3, c_lenx, c_bytes, c_tfirst, c_tlast} <= '0;
        {c_g3, c_g4, c_g5, c_g6, c_g7, c_g8, c_g16, c_gsum, c_b, c_outmax, c_wbub} <= '0;
        c_clr <= c_clr + 1'b1;
        have_first <= 1'b0; have_prev <= 1'b0;
      end else begin
        if (aw_hs) begin
          c_aw <= c_aw + 1'b1;
          if (awlen == 8'd0)      c_len0 <= c_len0 + 1'b1;
          else if (awlen == 8'd3) c_len3 <= c_len3 + 1'b1;
          else                    c_lenx <= c_lenx + 1'b1;
        end
        if (w_hs) begin
          c_wbeat <= c_wbeat + 1'b1;
          c_tlast <= c_cyc;
          if (wlast) c_wlast <= c_wlast + 1'b1;
        end
        if (inb && !wvalid) c_wbub <= c_wbub + 1'b1;
        if (pc_sv) c_bytes <= c_bytes + pc_sum;
        if (b_hs) c_b <= c_b + 1'b1;
        if (outst > c_outmax) c_outmax <= outst;
        if (first_beat) begin
          if (!have_first) begin c_tfirst <= c_cyc; have_first <= 1'b1; end
          have_prev <= 1'b1;
          if (have_prev) begin
            if (gcnt <= 16'd3)       c_g3  <= c_g3  + 1'b1;
            else if (gcnt == 16'd4)  c_g4  <= c_g4  + 1'b1;
            else if (gcnt == 16'd5)  c_g5  <= c_g5  + 1'b1;
            else if (gcnt == 16'd6)  c_g6  <= c_g6  + 1'b1;
            else if (gcnt == 16'd7)  c_g7  <= c_g7  + 1'b1;
            else if (gcnt <= 16'd15) c_g8  <= c_g8  + 1'b1;
            else                     c_g16 <= c_g16 + 1'b1;
            if (gcnt <= 16'd15) c_gsum <= c_gsum + gcnt;
          end
        end
      end
    end
  end

  // ---- read channel: one AR at a time; the selected word is registered once, then arlen+1 beats are returned
  reg        rbusy, rsel_win, rsel_ctl, rpend;
  reg [7:0]  rleft;
  reg [7:0]  ridx;
  reg [63:0] rsink, rreg;
  assign arready = !rbusy;
  assign rdata   = {(DATA_W/64){rreg}};

  always @(*) begin
    case (ridx[4:0])
      5'd0:  rsink = MAGIC;      5'd1:  rsink = c_cyc;     5'd2:  rsink = c_aw;      5'd3:  rsink = c_wbeat;
      5'd4:  rsink = c_wlast;    5'd5:  rsink = c_len0;    5'd6:  rsink = c_len3;    5'd7:  rsink = c_lenx;
      5'd8:  rsink = c_bytes;    5'd9:  rsink = c_tfirst;  5'd10: rsink = c_tlast;   5'd11: rsink = c_g3;
      5'd12: rsink = c_g4;       5'd13: rsink = c_g5;      5'd14: rsink = c_g6;      5'd15: rsink = c_g7;
      5'd16: rsink = c_g8;       5'd17: rsink = c_g16;     5'd18: rsink = c_gsum;    5'd19: rsink = c_b;
      5'd20: rsink = c_outmax;   5'd21: rsink = c_wbub;    5'd22: rsink = c_clr;     5'd23: rsink = c_wnoaw;
      default: rsink = 64'hDEAD_BEEF_0000_0000 | ridx;
    endcase
  end

  always @(posedge clk) begin
    if (!rstn) begin
      rbusy <= 1'b0; rvalid <= 1'b0; rlast <= 1'b0; rid <= '0; rleft <= 0; ridx <= 0; clr <= 1'b0;
      rsel_win <= 1'b0; rsel_ctl <= 1'b0; rpend <= 1'b0; rreg <= 0;
    end else begin
      clr <= ar_hs && !ar_win && araddr[11];
      if (ar_hs) begin
        rbusy <= 1'b1; rpend <= 1'b1; rid <= arid; rleft <= arlen;
        rsel_win <= ar_win; rsel_ctl <= araddr[11];
        ridx <= ar_win ? (araddr[11] ? {2'd0, araddr[8:3]} : araddr[10:3]) : {3'd0, araddr[7:3]};
      end else if (rpend) begin
        rpend  <= 1'b0;
        rreg   <= !rsel_win ? rsink : (rsel_ctl ? ctl[ridx[5:0]] : ((ridx < NSTAT) ? stat[ridx] : 64'hDEAD_0000_0000_0000 | ridx));
        rvalid <= 1'b1; rlast <= (rleft == 8'd0);
      end else if (rvalid && rready) begin
        if (rlast) begin rbusy <= 1'b0; rvalid <= 1'b0; rlast <= 1'b0; end
        else begin rleft <= rleft - 1'b1; rlast <= (rleft == 8'd1); end
      end
    end
  end
endmodule
