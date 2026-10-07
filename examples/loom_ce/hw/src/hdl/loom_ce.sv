import lynxTypes::*;

/**
 * loom_ce
 *
 * The emulated accelerator's copy engine. Two ways to drive it:
 *
 * SINGLE COPY (START): one copy, then an optional completion fence, as
 * before. A put (get = 0) copies from card memory (HBM) to a destination VA,
 * a get (get = 1) from a source VA into card memory. Internally a program of
 * one or two entries (COPY, then SIGNAL fence_va with the new copy count).
 *
 * PROGRAM (PROG_CMD): a list of up to N_PROG 64 B entries in card memory is
 * loaded into on-chip RAM (load), then executed in order with no host
 * involvement (run), as often as wanted. The entries are the stores an
 * accelerator kernel would issue: what to copy where, when to signal, what
 * to wait for. Entry (eight 64-bit words, little-endian):
 *   w0 [3:0]   op: 1 COPY, 2 SIGNAL, 3 WAIT, 4 REDUCE, 5 STAMP; else a no-op
 *      [4]     src_host: COPY's source on the host stream (a read of a
 *              window, as a get); else card memory
 *      [5]     dst_card: the destination (COPY, REDUCE, SIGNAL, STAMP) is
 *              card memory; else the host stream (a window, host memory)
 *      [63:32] len, bytes (COPY, REDUCE): a multiple of 64; 0 = a no-op
 *   w1  src (COPY), a (REDUCE)              all VAs in the PID's space,
 *   w2  b (REDUCE)                           64 B-aligned except SIGNAL/STAMP
 *   w3  dst (COPY, REDUCE, SIGNAL, STAMP), the polled word (WAIT)  (8 B)
 *   w4  value (SIGNAL), target (WAIT); RUN_BASE is added to both
 *   COPY    len bytes src -> dst
 *   SIGNAL  8 B value to dst, behind every earlier write: posted writes on
 *           the host stream stay in order (loom_ingress keeps it); before one
 *           into card memory the engine waits for every earlier write
 *   WAIT    poll the 8 B word at dst in card memory until it is >= target
 *           (signed difference), the arrival of a peer's signal; ABORT ends
 *           the run from here
 *   REDUCE  dst = a + b, sixteen int32 lanes a beat, a and b in card memory,
 *           in chunks of CH beats (a's chunk is held, b's streams past it)
 *   STAMP   8 B of the cycle counter (NOW), taken when the entry is reached,
 *           to dst
 *
 * Card-memory sources are read ahead: the issuer requests the next entries'
 * reads (up to MAX_RD in flight) while the executor streams the current one,
 * so short copies follow each other at the stream rate. The card stream
 * returns data in request order, so the executor takes it in program order.
 * The issuer stops at a barrier until the executor has finished it: a WAIT
 * (later reads may need what it waits for), an entry writing card memory
 * (later reads may read it; the executor finishes it only once its writes
 * completed), and a host-stream read (it would queue behind read-ahead
 * requests whose data is not taken until it completes).
 *
 * 1. sq_rd {LOCAL_READ, pid, va, len}: STRM_CARD for card memory, STRM_HOST
 *    for a window (peer-to-peer reads of the U280's uwin, loom_read)
 * 2. sq_wr {LOCAL_WRITE, pid, va, len}: STRM_HOST to a window (peer-to-peer
 *    writes into loom_ingress) or host memory, STRM_CARD into HBM
 * 3. the count, not the stream's tlast, ends a copy (a read may come back as
 *    several tlast-terminated segments)
 */
module loom_ce #(
    parameter integer N_PROG = 2048,    // program entries
    parameter integer CH     = 64,      // REDUCE chunk, beats (4 KiB)
    parameter integer MAX_RD = 8        // read-ahead requests in flight
) (
    input  logic                        aclk,
    input  logic                        aresetn,

    // From loom_ce_ctrl: the single copy
    input  logic                        start,
    input  logic [VADDR_BITS-1:0]       src_va,
    input  logic [VADDR_BITS-1:0]       dst_va,
    input  logic [LEN_BITS-1:0]         len,
    input  logic [PID_BITS-1:0]         pid,
    input  logic [VADDR_BITS-1:0]       fence_va,
    input  logic                        get,          // 1: source on the host stream, destination in card memory
    // the program
    input  logic                        prog_load,
    input  logic                        prog_run,
    input  logic [VADDR_BITS-1:0]       prog_va,
    input  logic [31:0]                 prog_n,
    input  logic [63:0]                 run_base,
    input  logic                        abort,
    output logic                        busy,
    output logic [31:0]                 copies,
    output logic [63:0]                 cycles,
    output logic [31:0]                 runs,
    output logic [31:0]                 pc,
    output logic [63:0]                 now,
    output logic                        aborted,
    output logic [63:0]                 polls,
    output logic [63:0]                 run_cycles,

    // Requests
    output req_t                        rd_req,
    output logic                        rd_valid,
    input  logic                        rd_ready,
    output req_t                        wr_req,
    output logic                        wr_valid,
    input  logic                        wr_ready,
    input  logic                        wr_done,      // cq_wr.valid

    // Card stream in (axis_card_recv)
    input  logic [AXI_DATA_BITS-1:0]    s_tdata,
    input  logic [AXI_DATA_BITS/8-1:0]  s_tkeep,
    input  logic                        s_tvalid,
    output logic                        s_tready,

    // Host stream out (axis_host_send)
    output logic [AXI_DATA_BITS-1:0]    m_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_tkeep,
    output logic                        m_tvalid,
    input  logic                        m_tready,
    output logic                        m_tlast,

    // Host stream in (axis_host_recv)
    input  logic [AXI_DATA_BITS-1:0]    s_host_tdata,
    input  logic [AXI_DATA_BITS/8-1:0]  s_host_tkeep,
    input  logic                        s_host_tvalid,
    output logic                        s_host_tready,

    // Card stream out (axis_card_send)
    output logic [AXI_DATA_BITS-1:0]    m_card_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_card_tkeep,
    output logic                        m_card_tvalid,
    input  logic                        m_card_tready,
    output logic                        m_card_tlast,

    output logic                        cnt_in_wait   // copying, and no source data presented
);

localparam logic [3:0] OP_COPY = 4'd1, OP_SIGNAL = 4'd2, OP_WAIT = 4'd3, OP_REDUCE = 4'd4, OP_STAMP = 4'd5;
localparam integer PC_BITS = $clog2(N_PROG) + 1;
localparam integer BEATS   = LEN_BITS - 6;          // beat counts
localparam integer LANES   = AXI_DATA_BITS / 32;

typedef struct packed {
    logic [3:0]            op;
    logic                  src_host;
    logic                  dst_card;
    logic [LEN_BITS-1:0]   len;
    logic [VADDR_BITS-1:0] a;
    logic [VADDR_BITS-1:0] b;
    logic [VADDR_BITS-1:0] dst;
    logic [63:0]           val;
} ent_t;

function automatic ent_t decode(input logic [AXI_DATA_BITS-1:0] w);
    ent_t e;
    e.op       = w[3:0];
    e.src_host = w[4];
    e.dst_card = w[5];
    e.len      = (w[63:32] >> LEN_BITS) != 0 ? '0 : w[32 +: LEN_BITS];   // too long: a no-op
    e.a        = w[64  +: VADDR_BITS];
    e.b        = w[128 +: VADDR_BITS];
    e.dst      = w[192 +: VADDR_BITS];
    e.val      = w[256 +: 64];
    return e;
endfunction

// An entry with data: COPY or REDUCE of nonzero length
function automatic logic moves(input ent_t e);
    return (e.op == OP_COPY || e.op == OP_REDUCE) && e.len != 0;
endfunction
// Its reads come from card memory ahead of time
function automatic logic reads_ahead(input ent_t e);
    return moves(e) && (e.op == OP_REDUCE || !e.src_host);
endfunction
// The issuer waits at it until the executor is past it
function automatic logic barrier(input ent_t e);
    return (e.op == OP_WAIT) || (moves(e) && (e.dst_card || (e.op == OP_COPY && e.src_host)));
endfunction

// ---------------------------------------------------------------------------
// Program RAM: one copy for each reader (issuer, executor), written by a load
// ---------------------------------------------------------------------------
logic [AXI_DATA_BITS-1:0] prog_i [N_PROG];
logic [AXI_DATA_BITS-1:0] prog_x [N_PROG];
logic [AXI_DATA_BITS-1:0] ram_iq, ram_xq;
logic [PC_BITS-1:0]       ipc, xpc, ld_idx;
logic                     ld_we;

always_ff @(posedge aclk) begin
    if (ld_we) begin
        prog_i[ld_idx[PC_BITS-2:0]] <= s_tdata;
        prog_x[ld_idx[PC_BITS-2:0]] <= s_tdata;
    end
    ram_iq <= prog_i[ipc[PC_BITS-2:0]];
    ram_xq <= prog_x[xpc[PC_BITS-2:0]];
end

// The single copy's entries: COPY, then the fence as a SIGNAL
logic                  single;
ent_t                  sgl [2];
logic [PC_BITS-1:0]    n_ent;
wire  ent_t            ent_i = single ? sgl[ipc[0]] : decode(ram_iq);
wire  ent_t            ent_x = single ? sgl[xpc[0]] : decode(ram_xq);

// ---------------------------------------------------------------------------
// Executor
// ---------------------------------------------------------------------------
typedef enum logic [4:0] {
    X_IDLE, X_FETCH, X_DEC, X_RD, X_WR, X_STREAM, X_RED_A, X_RED_B, X_WDONE,
    X_SIG_FL, X_SIG_REQ, X_SIG_DATA, X_WAIT_RD, X_WAIT_DATA, X_NEXT, X_DONE,
    X_LD_REQ, X_LD_DATA
} xstate_t;
xstate_t xs;

ent_t                  x;              // the entry being executed
logic [63:0]           x_val;          // its value (SIGNAL, WAIT target, STAMP)
logic [BEATS-1:0]      left;           // beats still to write
logic [$clog2(CH):0]   cleft, bi;      // REDUCE: beats in this chunk, beat index in it
logic [7:0]            wr_out;         // write requests taken, not yet completed
logic                  abort_pend;
logic [63:0]           t_run;
logic                  timing, data_posted;
logic [AXI_DATA_BITS-1:0] red_buf [CH];
logic [AXI_DATA_BITS-1:0] red_sum;

wire src_valid = x.src_host ? s_host_tvalid : s_tvalid;
wire [AXI_DATA_BITS-1:0] src_data = x.src_host ? s_host_tdata : s_tdata;
wire dst_ready = x.dst_card ? m_card_tready : m_tready;
wire last_beat = (left == 1);
wire stream_beat = (xs == X_STREAM) && src_valid && dst_ready;
wire red_a_beat  = (xs == X_RED_A) && s_tvalid;
wire red_b_beat  = (xs == X_RED_B) && s_tvalid && dst_ready;
wire wr_drained  = (wr_out == 0) || (wr_out == 1 && wr_done);
wire [63:0] wait_word = s_tdata[64*x.dst[5:3] +: 64];

always_comb
    for (int l = 0; l < LANES; l++)
        red_sum[32*l +: 32] = red_buf[bi[$clog2(CH)-1:0]][32*l +: 32] + s_tdata[32*l +: 32];

// A read-ahead request's data fully taken (one per COPY, one per REDUCE chunk half)
wire rd_taken = (stream_beat && last_beat && !x.src_host) ||
                (red_a_beat && bi == cleft - 1) || (red_b_beat && bi == cleft - 1);

wire [BEATS-1:0] len_beats = x.len[LEN_BITS-1:6];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        xs <= X_IDLE;
        copies <= 0;
        runs <= 0;
        cycles <= 0;
        run_cycles <= 0;
        polls <= 0;
        aborted <= 1'b0;
        abort_pend <= 1'b0;
        timing <= 1'b0;
        data_posted <= 1'b0;
        wr_out <= 0;
        single <= 1'b0;
        xpc <= 0;
        ld_idx <= 0;
        n_ent <= 0;
        x <= '0;
    end else begin
        wr_out <= wr_out + 8'(wr_valid && wr_ready) - 8'(wr_done);
        if (abort) abort_pend <= 1'b1;
        if (red_a_beat) red_buf[bi[$clog2(CH)-1:0]] <= s_tdata;

        case (xs)
            X_IDLE: begin
                xpc <= 0;
                if (start) begin
                    single     <= 1'b1;
                    sgl[0]     <= '{op: OP_COPY, src_host: get, dst_card: get, len: len,
                                    a: src_va, b: '0, dst: dst_va, val: '0};
                    sgl[1]     <= '{op: OP_SIGNAL, src_host: 1'b0, dst_card: 1'b0, len: '0,
                                    a: '0, b: '0, dst: fence_va, val: 64'(copies) + 64'd1};
                    n_ent      <= (fence_va != 0) ? PC_BITS'(2) : PC_BITS'(1);
                    xs         <= X_FETCH;
                end else if (prog_run) begin
                    single     <= 1'b0;
                    n_ent      <= (prog_n > N_PROG) ? PC_BITS'(N_PROG) : prog_n[PC_BITS-1:0];
                    aborted    <= 1'b0;
                    abort_pend <= 1'b0;
                    t_run      <= now;
                    xs         <= (prog_n == 0) ? X_DONE : X_FETCH;
                end else if (prog_load) begin
                    ld_idx     <= 0;
                    xs         <= (prog_n == 0) ? X_IDLE : X_LD_REQ;
                end
            end
            X_FETCH: xs <= X_DEC;                       // the RAM's read
            X_DEC: begin
                x     <= ent_x;
                x_val <= (ent_x.op == OP_STAMP) ? now : (single ? ent_x.val : ent_x.val + run_base);
                left  <= ent_x.len[LEN_BITS-1:6];
                bi    <= 0;
                cleft <= (ent_x.len[LEN_BITS-1:6] > CH) ? ($clog2(CH)+1)'(CH) : ent_x.len[6 +: $clog2(CH)+1];
                if (moves(ent_x))
                    xs <= (ent_x.op == OP_COPY && ent_x.src_host) ? X_RD : X_WR;
                else case (ent_x.op)
                    OP_SIGNAL: xs <= ent_x.dst_card ? X_SIG_FL : X_SIG_REQ;
                    OP_STAMP:  xs <= X_SIG_REQ;
                    OP_WAIT:   xs <= X_WAIT_RD;
                    default:   xs <= X_NEXT;
                endcase
            end
            X_RD: if (rd_ready) xs <= X_WR;
            X_WR: if (wr_ready) xs <= (x.op == OP_COPY) ? X_STREAM : X_RED_A;
            X_STREAM: if (stream_beat) begin
                left <= left - 1'b1;
                if (last_beat) begin
                    copies <= copies + 1;
                    xs <= x.dst_card ? X_WDONE : X_NEXT;
                end
            end
            X_RED_A: if (red_a_beat) begin
                if (bi == cleft - 1) begin
                    bi <= 0;
                    xs <= X_RED_B;
                end else bi <= bi + 1'b1;
            end
            X_RED_B: if (red_b_beat) begin
                left <= left - 1'b1;
                if (last_beat) xs <= x.dst_card ? X_WDONE : X_NEXT;
                else if (bi == cleft - 1) begin
                    bi    <= 0;
                    cleft <= (left - 1'b1 > CH) ? ($clog2(CH)+1)'(CH) : ($clog2(CH)+1)'(left - 1'b1);
                    xs    <= X_RED_A;
                end else bi <= bi + 1'b1;
            end
            // Writes into card memory complete before the next entry (this
            // cycle's completion counts)
            X_WDONE:  if (wr_drained) xs <= X_NEXT;
            X_SIG_FL: if (wr_drained) xs <= X_SIG_REQ;
            X_SIG_REQ:  if (wr_ready) xs <= X_SIG_DATA;
            X_SIG_DATA: if (dst_ready) xs <= X_NEXT;
            X_WAIT_RD:  if (rd_ready) xs <= X_WAIT_DATA;
            X_WAIT_DATA: if (s_tvalid) begin
                if ($signed(wait_word - x_val) >= 0) xs <= X_NEXT;
                else begin
                    polls <= polls + 1;
                    if (abort_pend) begin
                        aborted <= 1'b1;
                        xs <= X_DONE;
                    end else xs <= X_WAIT_RD;
                end
            end
            X_NEXT: begin
                if (xpc + 1'b1 == n_ent) xs <= X_DONE;
                else begin
                    xpc <= xpc + 1'b1;
                    xs  <= X_FETCH;
                end
            end
            X_DONE: begin
                if (!single) begin
                    runs <= runs + 1;
                    run_cycles <= now - t_run;
                end
                xs <= X_IDLE;
            end
            X_LD_REQ: if (rd_ready) xs <= X_LD_DATA;
            X_LD_DATA: if (s_tvalid) begin
                ld_idx <= ld_idx + 1'b1;
                if (ld_idx + 1'b1 == ((prog_n > N_PROG) ? PC_BITS'(N_PROG) : prog_n[PC_BITS-1:0])) xs <= X_IDLE;
            end
            default: xs <= X_IDLE;
        endcase

        // The single copy's time: START to the data write's completion,
        // which is the first write completion after the data request
        if (xs == X_IDLE && start) begin
            timing <= 1'b1;
            cycles <= 0;
            data_posted <= 1'b0;
        end else if (timing) begin
            cycles <= cycles + 1;
            if (xs == X_WR && wr_ready) data_posted <= 1'b1;
            if (data_posted && wr_done) timing <= 1'b0;
        end
    end
end

assign ld_we = (xs == X_LD_DATA) && s_tvalid;

// ---------------------------------------------------------------------------
// Issuer: the read-ahead of card-memory sources
// ---------------------------------------------------------------------------
typedef enum logic [2:0] { I_IDLE, I_FETCH, I_DEC, I_RD, I_RED, I_BAR, I_DONE } istate_t;
istate_t is;

ent_t                  y;              // the entry whose reads are being issued
logic [LEN_BITS-1:0]   i_off;          // REDUCE: offset of the chunk being requested
logic                  i_sub;          // REDUCE: 0 = a's chunk, 1 = b's
logic [LEN_BITS-1:0]   i_rest;         // REDUCE: bytes from i_off to the end
logic [LEN_BITS-1:0]   i_clen;         // REDUCE: this chunk's bytes
logic                  i_lastch;       // REDUCE: this chunk is the last
logic [$clog2(MAX_RD):0] rd_inflight;

wire x_rd   = (xs == X_LD_REQ) || (xs == X_RD) || (xs == X_WAIT_RD);
wire i_room = (rd_inflight < MAX_RD) && !x_rd;
wire i_rd   = ((is == I_RD) || (is == I_RED)) && i_room;
wire i_go   = i_rd && rd_ready;
localparam logic [LEN_BITS-1:0] CH_BYTES = LEN_BITS'(64*CH);
wire [LEN_BITS-1:0] i_rest_nx = i_rest - i_clen;
wire x_past = (xpc > ipc) || (xs == X_DONE) || (xs == X_IDLE);

// The entry's reads are issued (or it has none): wait at a barrier, else on
wire i_last = (ipc + 1'b1 == n_ent);
wire i_red_end = i_sub && i_lastch;
wire i_adv  = ((is == I_DEC) && !reads_ahead(ent_i) && !barrier(ent_i)) ||
              ((is == I_RD) && i_go && !barrier(y)) || ((is == I_RED) && i_go && i_red_end && !barrier(y)) ||
              ((is == I_BAR) && x_past);
wire i_bar  = ((is == I_DEC) && !reads_ahead(ent_i) && barrier(ent_i)) ||
              ((is == I_RD) && i_go && barrier(y)) || ((is == I_RED) && i_go && i_red_end && barrier(y));

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        is <= I_IDLE;
        ipc <= 0;
        rd_inflight <= 0;
        i_off <= 0;
        i_sub <= 1'b0;
        i_rest <= 0;
        i_clen <= 0;
        i_lastch <= 1'b0;
        y <= '0;
    end else begin
        rd_inflight <= rd_inflight + (i_go ? 1'b1 : 1'b0) - (rd_taken ? 1'b1 : 1'b0);
        case (is)
            I_IDLE: begin
                ipc <= 0;
                if (xs == X_FETCH && xpc == 0) is <= I_FETCH;     // a run (or single copy) began
            end
            I_FETCH: is <= I_DEC;
            I_DEC: begin
                y        <= ent_i;
                i_off    <= 0;
                i_sub    <= 1'b0;
                i_rest   <= ent_i.len;
                i_clen   <= (ent_i.len > CH_BYTES) ? CH_BYTES : ent_i.len;
                i_lastch <= (ent_i.len <= CH_BYTES);
                if (reads_ahead(ent_i)) is <= (ent_i.op == OP_REDUCE) ? I_RED : I_RD;
            end
            I_RED: if (i_go) begin
                i_sub <= !i_sub;
                if (i_sub) begin
                    i_off    <= i_off + i_clen;
                    i_rest   <= i_rest_nx;
                    i_clen   <= (i_rest_nx > CH_BYTES) ? CH_BYTES : i_rest_nx;
                    i_lastch <= (i_rest_nx <= CH_BYTES);
                end
            end
            default: ;
        endcase
        if (i_bar) is <= I_BAR;
        else if (i_adv) begin
            if (i_last) is <= I_DONE;
            else begin
                ipc <= ipc + 1'b1;
                is  <= I_FETCH;
            end
        end
        // The run ended (normally or aborted): back to idle
        if (xs == X_DONE) is <= I_IDLE;
    end
end

// ---------------------------------------------------------------------------
// Requests and streams
// ---------------------------------------------------------------------------
logic [AXI_DATA_BITS-1:0]   d;
logic [AXI_DATA_BITS/8-1:0] k;
logic                       v, l;

always_ff @(posedge aclk) begin
    if (!aresetn) now <= 0;
    else          now <= now + 1;
end

assign busy        = (xs != X_IDLE);
assign pc          = 32'(xpc);
assign cnt_in_wait = ((xs == X_STREAM) && !src_valid) || (((xs == X_RED_A) || (xs == X_RED_B)) && !s_tvalid);

always_comb begin
    rd_req        = '0;
    rd_req.opcode = LOCAL_READ;
    rd_req.dest   = 0;
    rd_req.pid    = pid;
    rd_req.last   = 1'b1;
    rd_req.strm   = STRM_CARD;
    if (xs == X_LD_REQ) begin
        rd_req.vaddr = prog_va;
        rd_req.len   = ((prog_n > N_PROG) ? LEN_BITS'(N_PROG) : prog_n[LEN_BITS-1:0]) << 6;
    end else if (xs == X_RD) begin
        rd_req.strm  = STRM_HOST;
        rd_req.vaddr = x.a;
        rd_req.len   = x.len;
    end else if (xs == X_WAIT_RD) begin
        rd_req.vaddr = {x.dst[VADDR_BITS-1:6], 6'b0};
        rd_req.len   = 64;
    end else if (is == I_RED) begin
        rd_req.vaddr = (i_sub ? y.b : y.a) + VADDR_BITS'(i_off);
        rd_req.len   = i_clen;
    end else begin
        rd_req.vaddr = y.a;
        rd_req.len   = y.len;
    end
    rd_valid      = x_rd || i_rd;

    wr_req        = '0;
    wr_req.opcode = LOCAL_WRITE;
    wr_req.dest   = 0;
    wr_req.pid    = pid;
    wr_req.last   = 1'b1;
    wr_req.strm   = x.dst_card ? STRM_CARD : STRM_HOST;
    wr_req.vaddr  = x.dst;
    wr_req.len    = (xs == X_SIG_REQ) ? LEN_BITS'(8) : x.len;
    wr_valid      = (xs == X_WR) || (xs == X_SIG_REQ);

    s_tready      = (xs == X_LD_DATA) || (xs == X_WAIT_DATA) || (xs == X_RED_A) ||
                    ((xs == X_STREAM) && !x.src_host && dst_ready) || ((xs == X_RED_B) && dst_ready);
    s_host_tready = (xs == X_STREAM) && x.src_host && dst_ready;

    // The beat for the destination stream
    begin
        d = src_data; k = '1; l = last_beat;
        v = ((xs == X_STREAM) && src_valid) || ((xs == X_RED_B) && s_tvalid) || (xs == X_SIG_DATA);
        if (xs == X_RED_B) d = red_sum;
        if (xs == X_SIG_DATA) begin
            d = {{(AXI_DATA_BITS-64){1'b0}}, x_val};
            k = {{(AXI_DATA_BITS/8-8){1'b0}}, 8'hFF};
            l = 1'b1;
        end
        m_tdata       = d;  m_tkeep      = k;  m_tlast      = l;  m_tvalid      = v && !x.dst_card;
        m_card_tdata  = d;  m_card_tkeep = k;  m_card_tlast = l;  m_card_tvalid = v &&  x.dst_card;
    end
end

endmodule
