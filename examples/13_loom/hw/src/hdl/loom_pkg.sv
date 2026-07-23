package loom_pkg;

import lynxTypes::*;

// ============================================================ enums
typedef enum logic [1:0] {SRC_APERTURE, SRC_DMA, SRC_RX} loom_src_t; // where a txn came from
typedef enum logic [0:0] {OP_READ, OP_WRITE}             loom_op_t;

// what the switch decides to do with a txn:
//   RT_LOCAL  - keep on this host (host DMA)
//   RT_REMOTE - push to a peer   (RDMA WRITE)
//   RT_DROP   - reject           (bad address / not permitted)
//   RT_PULL   - fetch from a peer (RDMA READ)
typedef enum logic [1:0] {RT_LOCAL, RT_REMOTE, RT_DROP, RT_PULL} loom_route_t;

// ============================================================ uniform transaction
// Every request (aperture trap or DMA descriptor) is turned into ONE of these by
// the orchestrator, so the router never has to care where it came from.
typedef struct packed {
    loom_src_t                 src;
    loom_op_t                  op;
    logic                      inline_data;   // 1 = aperture, payload rides in `data` (8 B)
    logic [AXIL_DATA_BITS-1:0] data;          // aperture write word
    logic [PID_BITS-1:0]       src_pid;       // source thread + address
    logic [VADDR_BITS-1:0]     src_vaddr;
    logic [PID_BITS-1:0]       dst_pid;       // destination thread + address
    logic [VADDR_BITS-1:0]     dst_vaddr;
    logic [LEN_BITS-1:0]       len;           // byte count
    logic [DEST_BITS-1:0]      src_strm;      // shell stream indices
    logic [DEST_BITS-1:0]      dst_strm;
    logic                      err_bounds;    // failed the orchestrator's bounds check
} loom_txn_t;

// ============================================================ CSR map (8 B stride)
// Software drives the FPGA by writing these register indices. The aperture
// window (16..31) is fixed - the SW header hardcodes AP_LO.
localparam integer N_REGS = 128;

// -- command / status --
localparam integer CMD_REG        = 0;   // bit0 = start a DMA
localparam integer STATUS_REG     = 1;   // bit0 = DMA busy
localparam integer DONE_CNT_REG   = 2;   // completed-DMA counter (SW polls this)

// -- DMA descriptor (source, destination, length) --
localparam integer SRC_PID_REG    = 3;
localparam integer SRC_ADDR_REG   = 4;
localparam integer SRC_STRM_REG   = 5;
localparam integer DST_PID_REG    = 6;
localparam integer DST_ADDR_REG   = 7;
localparam integer DST_STRM_REG   = 8;
localparam integer LEN_REG        = 9;

// -- aperture config (which peer the trapped window points at) --
localparam integer AP_PID_REG     = 10;
localparam integer AP_BASE_LO_REG = 11;
localparam integer AP_BASE_HI_REG = 12;
localparam integer AP_STRM_REG    = 13;

localparam integer COYOTE_PID_REG = 14;  // our own ctid (RDMA QPN low bits); 15 reserved

// -- aperture window: a trapped access to reg (16 + k) hits peer_base + k*8 --
localparam integer AP_LO = 16, AP_HI = 31;

// -- range table: 8 entries x 3 CSRs -> 32..55 --
//   w0 = local base vaddr
//   w1 = {valid[42], ingress[41:40], src_pid[39:34], route[33:32], dest[31:28], len[27:0]}
//   w2 = remote base vaddr (translation target)
localparam integer RANGE_BASE_REG = 32, N_RANGE = 8, RANGE_W = 3;

// -- bounds window + status/control --
localparam integer WIN_BASE_REG   = 56;
localparam integer WIN_LIMIT_REG  = 57;
localparam integer DROP_CNT_REG   = 58;  // dropped-txn counter
localparam integer ROLE_REG       = 59;  // SW-only (source/dest node); HW ignores
localparam integer CNT_CTRL_REG   = 60;  // bit0 = clear all counters

// -- cycle counters: 8 x {acc, n} -> 61..76 -- then a free-running tick --
localparam integer CNT_BASE_REG   = 61, N_CNT = 8;
localparam integer TICK_REG       = 77;

// counter indices (into the acc/n bank)
localparam integer C_TRANSLATE = 0, C_LOOKUP = 1, C_FORWARD = 2, C_QUEUE = 3;
localparam integer C_ENCAP     = 4, C_ROCE_TX = 5, C_RX_LAND = 6, C_RX_FORWARD = 7;

localparam integer AP_BYTES = 8;   // one aperture access = 8 B (one AXI-Lite word)

endpackage
