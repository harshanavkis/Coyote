import lynxTypes::*;

/**
 * loom_read (loom_switch)
 *
 * Reads on the ingress window (uwin): the AR and R channels of axi_udata.
 * Whatever reads the window - a CPU load through the write-combining
 * mapping, a copy engine's DMA read peer-to-peer - reads the bytes the
 * window is bound to, and this switch does the rest. A read of an rdma
 * window is a get from the far export reference base + offset:
 *
 *   1. AR: take a read slot (N_SLOTS, a ring; R answers in AR order), look
 *      the address up (loom_table ub_*), and work out the 64 B lines the
 *      read's beats fall in: from the line of araddr to the line of its last
 *      beat (INCR or FIXED; a burst stays inside its 4 KiB page, so a read
 *      is at most 64 lines).
 *   2. Send a get request for those lines through loom_ingress (s_get), in
 *      its queue order: ref = the window's far reference + the offset of the
 *      first line, word = {lines [63:48], return reference [47:0]}, the
 *      return reference being this slot: {RRET_EXP, slot * 8 KiB}.
 *   3. The far loom_rx / loom_rd answer as for any get: the lines as rdma
 *      packets to return reference + offset, then the word to return
 *      reference + len (all ones if the far side rejected the request).
 *      This switch's loom_rx sees RRET_EXP and hands the lines to the slot's
 *      buffer (s_land) and the completion to this module (s_cmp): 8 KiB per
 *      slot in the reference space, so return reference + len (len <= 4 KiB)
 *      still names the slot when the word is all ones.
 *   4. R: when the head slot is complete, its beats go out, each the line
 *      its address falls in (the data on the lanes the address selects, as
 *      the AXI data bus carries it).
 *
 * A read that cannot be served - no window, a local window, past the
 * window's end, a burst wider than the bus or crossing its 4 KiB page - or
 * that the far side rejected, is answered OKAY with all ones (the value a
 * failed PCIe read returns), and counted (cnt_fail; cnt_far_err for the far
 * side's). Nothing times out: a read whose answer never comes holds its
 * slot, and the reads behind it, until a reset.
 */
module loom_read #(
    parameter integer UWIN_BITS = 27,
    parameter integer N_SLOTS   = 64,
    parameter         BUF_MEM   = "ultra"
) (
    input  logic                        aclk,
    input  logic                        aresetn,

    // The uwin's read channels (axi_udata AR / R)
    input  logic [UWIN_BITS-1:0]        s_araddr,
    input  logic [7:0]                  s_arlen,
    input  logic [2:0]                  s_arsize,
    input  logic [1:0]                  s_arburst,
    input  logic [AXI_ID_BITS-1:0]      s_arid,
    input  logic                        s_arvalid,
    output logic                        s_arready,
    output logic [AXI_DATA_BITS-1:0]    s_rdata,
    output logic [1:0]                  s_rresp,
    output logic                        s_rlast,
    output logic [AXI_ID_BITS-1:0]      s_rid,
    output logic                        s_rvalid,
    input  logic                        s_rready,

    // Table lookup by uwin address (loom_table ub_* port, two stages)
    output logic                        ub_ce1,
    output logic                        ub_ce2,
    output logic [UWIN_BITS-1:0]        ub_addr,
    input  logic                        ub_hit,
    input  logic                        ub_route,
    input  logic [PID_BITS-1:0]         ub_pid,
    input  logic [VADDR_BITS-1:0]       ub_base,
    input  logic [UWIN_BITS-1:0]        ub_ustart,
    input  logic [LEN_BITS:0]           ub_end,

    // Get requests (to loom_ingress s_get)
    output logic                        m_get_valid,
    input  logic                        m_get_ready,
    output logic [PID_BITS-1:0]         m_get_pid,
    output logic [VADDR_BITS-1:0]       m_get_ref,
    output logic [63:0]                 m_get_word,

    // Answers (from loom_rx): lines into the slot buffer, completions
    input  logic                        s_land_valid,
    input  logic [$clog2(N_SLOTS)+5:0]  s_land_line,
    input  logic [AXI_DATA_BITS-1:0]    s_land_data,
    input  logic                        s_cmp_valid,
    input  logic [$clog2(N_SLOTS)-1:0]  s_cmp_slot,
    input  logic                        s_cmp_err,

    output logic                        busy,

    // Counter pulses
    output logic                        cnt_read,      // a read taken (AR)
    output logic                        cnt_done,      // a read answered (its last R beat)
    output logic                        cnt_fail,      // a read answered with all ones
    output logic                        cnt_far_err,   // ... because the far side rejected its get
    output logic                        cnt_slot_wait, // cycles an AR waited for a free slot
    output logic                        cnt_stray      // a completion for a slot with no get out (must be 0)
);

localparam [7:0] RRET_EXP = 8'hFE;    // keep in sync with loom_rx.sv
localparam integer SLOT_W = $clog2(N_SLOTS);
localparam integer LINE_W = SLOT_W + 6;
localparam integer SK     = 4;         // R skid buffer

// ---------------------------------------------------------------------------
// Slots: [rp, wp) allocated, in AR order
// ---------------------------------------------------------------------------
logic [SLOT_W:0]        wp, rp;
wire                    s_full = (wp - rp) == (SLOT_W+1)'(N_SLOTS);
wire  [SLOT_W-1:0]      h = rp[SLOT_W-1:0];
logic [N_SLOTS-1:0]     done, err, sent;

// Per slot, what R needs
logic [AXI_ID_BITS-1:0] m_id   [N_SLOTS];
logic [7:0]             m_len  [N_SLOTS];
logic [2:0]             m_size [N_SLOTS];
logic                   m_fix  [N_SLOTS];
logic [11:0]            m_addr [N_SLOTS];

// ---------------------------------------------------------------------------
// AR: take, look up (two cycles), request
// ---------------------------------------------------------------------------
typedef enum logic [1:0] { A_IDLE, A_L1, A_L2, A_REQ } astate_t;
astate_t ast;

assign s_arready = (ast == A_IDLE) && !s_full;
wire   ar_hs     = s_arvalid && s_arready;

assign ub_addr = {s_araddr[UWIN_BITS-1:6], 6'b0};
assign ub_ce1  = (ast == A_IDLE);
assign ub_ce2  = (ast == A_L1);

// The lines the read's beats fall in: up to the line of its last beat
wire [14:0] ar_lo   = {3'b0, s_araddr[11:0]};
wire [14:0] ar_al   = ar_lo & ~((15'd1 << s_arsize) - 15'd1);
wire [14:0] ar_last = (s_arburst == 2'b00) ? ar_lo : ar_al + ({7'd0, s_arlen} << s_arsize);
wire        ar_bad  = (s_arsize > 3'd6) || (ar_last[14:12] != 3'd0);

logic [UWIN_BITS-1:0] a_line;       // the first line
logic [6:0]           a_nlines;
logic                 a_bad;
logic [SLOT_W-1:0]    a_slot;

wire [UWIN_BITS:0]  a_end = {1'b0, a_line} + {{(UWIN_BITS-12){1'b0}}, a_nlines, 6'b0};
wire                a_ok  = ub_hit && ub_route && !a_bad && ((LEN_BITS+1)'(a_end) <= ub_end);
wire [LEN_BITS-1:0] a_off = LEN_BITS'(a_line - ub_ustart);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        ast <= A_IDLE;
        wp  <= '0;
    end else case (ast)
        A_IDLE: if (ar_hs) begin
            a_line   <= {s_araddr[UWIN_BITS-1:6], 6'b0};
            a_nlines <= 7'(ar_last[11:6]) - 7'(s_araddr[11:6]) + 7'd1;
            a_bad    <= ar_bad;
            a_slot   <= wp[SLOT_W-1:0];
            m_id  [wp[SLOT_W-1:0]] <= s_arid;
            m_len [wp[SLOT_W-1:0]] <= s_arlen;
            m_size[wp[SLOT_W-1:0]] <= s_arsize;
            m_fix [wp[SLOT_W-1:0]] <= (s_arburst == 2'b00);
            m_addr[wp[SLOT_W-1:0]] <= s_araddr[11:0];
            wp  <= wp + 1'b1;
            ast <= A_L1;
        end
        A_L1: ast <= A_L2;
        A_L2: begin
            m_get_pid  <= ub_pid;
            m_get_ref  <= ub_base + VADDR_BITS'(a_off);
            m_get_word <= {9'd0, a_nlines, RRET_EXP, 40'({a_slot, 13'd0})};
            ast        <= a_ok ? A_REQ : A_IDLE;
        end
        A_REQ: if (m_get_ready) ast <= A_IDLE;
        default: ast <= A_IDLE;
    endcase
end

assign m_get_valid = (ast == A_REQ);

// ---------------------------------------------------------------------------
// Completions and the slot buffer
// ---------------------------------------------------------------------------
wire cmp_take = s_cmp_valid && sent[s_cmp_slot];
wire a_fail   = (ast == A_L2) && !a_ok;

logic r_act;
logic r_free;              // the head slot's last beat issued this cycle

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        done <= '0;
        err  <= '0;
        sent <= '0;
    end else begin
        if (m_get_valid && m_get_ready) sent[a_slot] <= 1'b1;
        if (a_fail) begin
            done[a_slot] <= 1'b1;
            err[a_slot]  <= 1'b1;
        end
        if (cmp_take) begin
            sent[s_cmp_slot] <= 1'b0;
            done[s_cmp_slot] <= 1'b1;
            err[s_cmp_slot]  <= s_cmp_err;
        end
        if (r_free) done[h] <= 1'b0;
    end
end

logic                   rd_en;
logic [LINE_W-1:0]      rd_line;
logic [AXI_DATA_BITS-1:0] buf_dout;

xpm_memory_sdpram #(
    .ADDR_WIDTH_A(LINE_W),
    .ADDR_WIDTH_B(LINE_W),
    .AUTO_SLEEP_TIME(0),
    .BYTE_WRITE_WIDTH_A(AXI_DATA_BITS),
    .CASCADE_HEIGHT(0),
    .CLOCKING_MODE("common_clock"),
    .ECC_MODE("no_ecc"),
    .MEMORY_INIT_FILE("none"),
    .MEMORY_INIT_PARAM("0"),
    .MEMORY_OPTIMIZATION("true"),
    .MEMORY_PRIMITIVE(BUF_MEM),
    .MEMORY_SIZE(N_SLOTS * 64 * AXI_DATA_BITS),
    .MESSAGE_CONTROL(0),
    .READ_DATA_WIDTH_B(AXI_DATA_BITS),
    .READ_LATENCY_B(2),
    .READ_RESET_VALUE_B("0"),
    .RST_MODE_A("SYNC"),
    .RST_MODE_B("SYNC"),
    .SIM_ASSERT_CHK(0),
    .USE_EMBEDDED_CONSTRAINT(0),
    .USE_MEM_INIT(0),
    .WAKEUP_TIME("disable_sleep"),
    .WRITE_DATA_WIDTH_A(AXI_DATA_BITS),
    .WRITE_MODE_B("read_first")
) inst_buf (
    .clka(aclk), .clkb(aclk),
    .ena(s_land_valid), .wea(s_land_valid), .addra(s_land_line), .dina(s_land_data),
    .enb(rd_en), .regceb(1'b1), .addrb(rd_line), .doutb(buf_dout),
    .rstb(1'b0), .sleep(1'b0),
    .injectsbiterra(1'b0), .injectdbiterra(1'b0), .sbiterrb(), .dbiterrb()
);

// ---------------------------------------------------------------------------
// R: the head slot's beats, in order, once it is complete. Each beat reads
// the line its address falls in (2 cycles), then waits in the skid buffer.
// ---------------------------------------------------------------------------
logic [7:0]             r_left;
logic [12:0]            r_cur;       // the beat's address in the page
logic [2:0]             r_size;
logic                   r_fix, r_err;
logic [5:0]             r_l0;        // the read's first line
logic [AXI_ID_BITS-1:0] r_id;

// Two read stages, then the skid buffer
logic                   p1_v, p2_v, p1_last, p2_last, p1_err, p2_err;
logic [AXI_ID_BITS-1:0] p1_id, p2_id;
typedef struct packed {
    logic [AXI_DATA_BITS-1:0] data;
    logic                     last;
    logic [AXI_ID_BITS-1:0]   id;
} rbeat_t;
rbeat_t sk_mem [SK];
logic [$clog2(SK):0] sk_wp, sk_rp;
wire  [$clog2(SK):0] sk_cnt = sk_wp - sk_rp;
wire  sk_empty = (sk_wp == sk_rp);

wire r_start = !r_act && (rp != wp) && done[h];
wire r_room  = ({1'b0, sk_cnt} + 2'(p1_v) + 2'(p2_v)) < ($clog2(SK)+2)'(SK);
wire r_issue = r_act && r_room;
assign r_free  = r_issue && (r_left == 8'd0);
assign rd_en   = r_issue;
assign rd_line = {h, 6'(r_cur[11:6] - r_l0)};

wire [12:0] r_step = 13'd1 << r_size;
wire [12:0] r_next = r_fix ? r_cur : ((r_cur & ~(r_step - 13'd1)) + r_step);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        r_act <= 1'b0;
        rp    <= '0;
        p1_v  <= 1'b0;
        p2_v  <= 1'b0;
    end else begin
        if (r_start) begin
            r_act  <= 1'b1;
            r_left <= m_len[h];
            r_cur  <= {1'b0, m_addr[h]};
            r_size <= m_size[h];
            r_fix  <= m_fix[h];
            r_l0   <= m_addr[h][11:6];
            r_id   <= m_id[h];
            r_err  <= err[h];
        end else if (r_issue) begin
            r_left <= r_left - 1'b1;
            r_cur  <= r_next;
            if (r_left == 8'd0) begin
                r_act <= 1'b0;
                rp    <= rp + 1'b1;
            end
        end
        p1_v <= r_issue;
        p2_v <= p1_v;
    end
end

always_ff @(posedge aclk) begin
    p1_last <= (r_left == 8'd0);
    p1_err  <= r_err;
    p1_id   <= r_id;
    p2_last <= p1_last;
    p2_err  <= p1_err;
    p2_id   <= p1_id;
end

wire r_pop = s_rvalid && s_rready;
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        sk_wp <= '0;
        sk_rp <= '0;
    end else begin
        if (p2_v) begin
            sk_mem[sk_wp[$clog2(SK)-1:0]] <= '{data: p2_err ? {AXI_DATA_BITS{1'b1}} : buf_dout,
                                               last: p2_last, id: p2_id};
            sk_wp <= sk_wp + 1'b1;
        end
        if (r_pop) sk_rp <= sk_rp + 1'b1;
    end
end

assign s_rvalid = !sk_empty;
assign s_rdata  = sk_mem[sk_rp[$clog2(SK)-1:0]].data;
assign s_rlast  = sk_mem[sk_rp[$clog2(SK)-1:0]].last;
assign s_rid    = sk_mem[sk_rp[$clog2(SK)-1:0]].id;
assign s_rresp  = 2'b00;

assign busy = (ast != A_IDLE) || r_act || p1_v || p2_v || !sk_empty || ((rp != wp) && done[h]);

assign cnt_read      = ar_hs;
assign cnt_done      = r_pop && s_rlast;
assign cnt_fail      = r_start && err[h];
assign cnt_far_err   = cmp_take && s_cmp_err;
assign cnt_slot_wait = s_arvalid && (ast == A_IDLE) && s_full;
assign cnt_stray     = s_cmp_valid && !sent[s_cmp_slot];

endmodule
