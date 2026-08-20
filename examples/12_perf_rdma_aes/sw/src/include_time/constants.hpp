/**
 * Example 12 - AES-GCM over RDMA. Constants and tag-slot bookkeeping for the
 * TIME-BASED client and server, shared so the two can never drift apart on
 * the layout or the round structure.
 */

#pragma once

#include <cstdint>
#include <cstddef>

#define DEFAULT_VFPGA_ID 0

// Time-based benchmarking (ported from the retired AES echo example's
// include_og_time): each measurement calibrates against a short run, then sizes
// the burst to fill TARGET_TIME_DEFAULT seconds, so small and large messages are
// equally well sampled. Client and server must agree on all of these.
#define N_RUNS_DEFAULT              3
#define TARGET_TIME_DEFAULT         2       // seconds per measurement
#define BATCH_SIZE_MAX              8192    // ops per round
#define MAX_BATCH_BYTES             (2 * 1024 * 1024)   // cap on batch * size
#define SYNC_INTERVAL               100     // reset counters every N rounds
#define MIN_TRANSFER_SIZE_DEFAULT   64
#define MAX_TRANSFER_SIZE_DEFAULT   (1 * 1024 * 1024)

/// Big batches at small sizes to amortise per-op submit cost, shrinking as
/// size grows. Both ends must call this with the same size.
inline unsigned int compute_batch(unsigned int size) {
    if (size == 0) { return BATCH_SIZE_MAX; }
    unsigned int by_bytes = (unsigned int) (MAX_BATCH_BYTES / size);
    unsigned int b = (by_bytes < BATCH_SIZE_MAX) ? by_bytes : BATCH_SIZE_MAX;
    return (b == 0) ? 1u : b;
}

// Must match PMTU_BYTES in the hardware build (cmake/FindCoyoteHW.cmake). The
// shell fragments every RDMA WRITE at this size and each fragment is an
// independent GCM packet.
#define RDMA_PMTU_BYTES 4096
#define GCM_TAG_BYTES   16

// Must match the CSR map in hw/src/vfpga_top.svh
enum class AesRegs : uint32_t {
    CTRL            = 0,   // [0] enable (0 = bypass), [1] role
    STATUS          = 1,   // [0] rx quarantine, [1] sticky "a tag verified"
    TAG_OK          = 2,
    TX_FRAMES       = 3,
    RX_FRAMES       = 4,
    MEAS_CTRL       = 5,   // [0] arm
    FIRST_CYCLE_LO  = 6,
    FIRST_CYCLE_HI  = 7,
    LAST_CYCLE_LO   = 8,
    LAST_CYCLE_HI   = 9
};

#define CTRL_ENABLE_BIT 0x1
#define CTRL_ROLE_BIT   0x2

#define USER_CLOCK_FREQ_HZ 250000000.0

// Tag-slot layout: hardware cannot change a fragment's byte count, so the last
// GCM_TAG_BYTES of every fragment are the tag. A transfer of `len` bytes
// therefore carries less than `len` bytes of payload.

// How many frames a transfer of `len` bytes becomes. This is a property of the
// shell's fragmentation, not of this code: dreq_rdma_parser_wr splits a WRITE
// into PMTU_BYTES fragments, and the last 16 bytes of every FRAME are the tag
// slot. The servers check this against the RX_FRAMES counter at the end of a
// run and print a MISMATCH if the shell disagrees -- the datapath is correct
// either way, only this accounting depends on the answer.
inline unsigned int fragments_for(unsigned int len) {
    return (len + RDMA_PMTU_BYTES - 1) / RDMA_PMTU_BYTES;
}

inline unsigned int payload_bytes(unsigned int len) {
    return len - GCM_TAG_BYTES * fragments_for(len);
}

/// Every fragment must be a multiple of 16 B and at least 32, so the tag lands
/// in its own 128-bit beat. Any multiple of 64 qualifies.
inline bool valid_transfer_size(unsigned int len) {
    if (len < 2 * GCM_TAG_BYTES || (len % 16) != 0) { return false; }
    const unsigned int tail = len % RDMA_PMTU_BYTES;
    return tail == 0 || tail >= 2 * GCM_TAG_BYTES;
}

inline bool is_tag_slot(unsigned int off, unsigned int len) {
    const unsigned int frag_start = (off / RDMA_PMTU_BYTES) * RDMA_PMTU_BYTES;
    unsigned int frag_end = frag_start + RDMA_PMTU_BYTES;
    if (frag_end > len) { frag_end = len; }
    return off >= frag_end - GCM_TAG_BYTES;
}

/// Fill the payload with a checkable pattern, leaving the tag slots alone
/// (hardware overwrites those).
inline void fill_payload(int *mem, unsigned int len) {
    for (unsigned int i = 0; i < len / sizeof(int); i++) {
        if (!is_tag_slot(i * sizeof(int), len)) { mem[i] = (int) i; }
    }
}

/// Number of payload words that did not survive the round trip; 0 = good.
inline unsigned int check_payload(const int *mem, unsigned int len) {
    unsigned int bad = 0;
    for (unsigned int i = 0; i < len / sizeof(int); i++) {
        if (!is_tag_slot(i * sizeof(int), len) && mem[i] != (int) i) { bad++; }
    }
    return bad;
}
