#ifndef JIGSAW_SW_FORWARDER_MESSAGES_HPP
#define JIGSAW_SW_FORWARDER_MESSAGES_HPP

/**
 * Jigsaw Software Forwarder — wire layout
 *
 * Only the on-the-wire definitions live here so the programs cannot drift;
 * all protocol logic sits inline in each program's main.cpp, mirroring the
 * structure of the proven jigsaw_baseline_rdma pair.
 *
 * Control plane exactly as in jigsaw_baseline_rdma: a single 64 B mailbox
 * at offset 0 of the QP buffer, used for both directions (the host writes a
 * request into the device's slot, the device writes the completion back
 * into the host's slot). Clear-and-wait `ready` flag, `type` distinguishes
 * request from completion. The payload region starts behind the control
 * page and mirrors the ivshmem layout one-to-one (ivshmem offset X == QP
 * buffer offset X), so guest DMA pointers translate with a single base
 * subtraction.
 */

#include <chrono>
#include <cstdint>

namespace jsfwd {

// QP buffer layout (identical on both nodes)
constexpr uint32_t CONTROL_SIZE = 0x1000;             // control page (mailbox)
constexpr uint32_t PAYLOAD_OFF  = CONTROL_SIZE;       // == ivshmem DMA_REGION_OFFSET
constexpr uint32_t BUF_BYTES    = 16U * 1024 * 1024;  // == ivshmem SHMEM_SIZE

// Message structure for the control plane
struct msg {
    uint64_t type;       // 0 = request, 1 = completion
    uint64_t op;         // OP_* below
    uint64_t addr;       // device register byte offset (MMIO ops)
    uint64_t value;      // write value / read result / setup base
    uint64_t ready;      // signaling flag: 1 means message is valid
    uint64_t padding[3]; // pad to 64 bytes
};
static_assert(sizeof(msg) == 64, "msg must be 64 B");

constexpr uint64_t MSG_REQUEST    = 0;
constexpr uint64_t MSG_COMPLETION = 1;

enum : uint64_t {
    OP_SETUP      = 1,  // value = host vaddr of the app/ivshmem buffer
    OP_MMIO_WRITE = 2,
    OP_MMIO_READ  = 3,
    OP_SYNC       = 4,  // full quiesce: both sides clearCompleted + connSync
    OP_STOP       = 5,
};

// Device register map of the jigsaw_baseline vFPGA (byte offsets in the
// guest-visible BAR; CSR index = offset >> 3, as in jigsaw_host_controller).
enum class DevReg : uint64_t {
    DMA_CMD       = 0x00,
    DMA_SRC_ADDR  = 0x08,
    DMA_DST_ADDR  = 0x10,
    DMA_H2D_LEN   = 0x18,
    DMA_STATUS    = 0x20,
    START_COMPUTE = 0x28,
    CYCLES_COMPUTE= 0x30,
    DMA_TX_LEN    = 0x38,
    DMA_D2H_LEN   = 0x40,
};
constexpr uint32_t dev_reg_index(uint64_t addr) {
    return static_cast<uint32_t>(addr >> 3);
}
constexpr uint32_t COYOTE_PID_REG = 9;

constexpr uint64_t DMA_CMD_H2D = 1;
constexpr uint64_t DMA_CMD_D2H = 3;
constexpr uint64_t STATUS_DMA_DONE_MASK    = 0x1;
constexpr uint64_t STATUS_BUNDLE_DONE_MASK = 0x3;

// Bulk transfers are sliced into 1 MiB chunks with a full MMIO sequence per
// chunk — the guest driver's convention (my_qemu_edu.c TRACE_CHUNK_BYTES).
constexpr uint64_t TRACE_CHUNK_BYTES = 1ULL << 20;

inline uint64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace jsfwd

#endif // JIGSAW_SW_FORWARDER_MESSAGES_HPP
