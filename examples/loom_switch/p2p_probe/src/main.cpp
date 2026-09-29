// G1: can a V80 vFPGA write into a U280 vFPGA's BAR, peer-to-peer?
//
// Needs, in the same host: the U280 running a Loom bitstream (driver
// coyote_driver, /dev/coyote_fpga_*) and the V80 running Coyote example 07,
// FPGA-initiated DMA (driver coyote_driver_versal, /dev/coyote_versal_fpga_*).
//
// 1. The host presets Loom's RDMA_STAGING_VA register (CSR word 16, byte 0x80)
//    to a marker and reads it back.
// 2. The U280 driver exports the first page of the U280 vFPGA's AXI-Lite user
//    control region as a dma-buf; the V80 driver imports it into the V80
//    thread's MMU at a reserved virtual address.
// 3. Example 07 on the V80 writes ONE 64 B beat to that address + 0x80. Its
//    write data is a beat counter, so the beat's first 8 B word is 1.
// 4. The host reads RDMA_STAGING_VA again: 1 means the write crossed PCIe
//    from the V80 to the U280 and reached Loom's register.
//
// The 64 B line at 0x80 is safe to overwrite: word 16 is RDMA_STAGING_VA,
// words 17-23 are read-only or unused. (The line at 0x40 holds DMA_TRIGGER.)
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

#include <coyote/cThread.hpp>

namespace {

constexpr uint32_t STAGING_WORD = 0x80 / 8;          // Loom RDMA_STAGING_VA
constexpr uint64_t MARKER = 0x5AFE'0000'C0FF'EE00ULL;
constexpr uint64_t PAGE = 4096;

// Example 07 (perf_fpga) control registers
enum Reg : uint32_t { CTRL = 0, DONE = 1, TIMER = 2, VADDR = 3, LEN = 4, PID = 5, N_REPS = 6, N_BEATS = 7 };
constexpr uint64_t START_WR = 0x2;

}  // namespace

int main() {
    coyote::cThread u280(0, getpid(), 0, nullptr, "coyote_fpga");
    coyote::cThread v80(0, getpid(), 0, nullptr, "coyote_versal_fpga");

    // 1. preset the target register
    u280.setCSR(MARKER, STAGING_WORD);
    const uint64_t before = u280.getCSR(STAGING_WORD);
    printf("U280 RDMA_STAGING_VA before: 0x%016lx\n", (unsigned long) before);
    if (before != MARKER) {
        printf("FAIL: the U280 register does not read back its preset (is a Loom bitstream loaded?)\n");
        return 1;
    }

    // 2. export the U280 control page, import it on the V80
    const int fd = u280.exportDmabuf(EXPORT_REGION_CTRL_USER, 0, PAGE);
    printf("U280 control page exported as dma-buf fd %d\n", fd);
    void *va = mmap(nullptr, PAGE, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (va == MAP_FAILED) throw std::runtime_error("mmap for a reserved address failed");
    v80.importDmabuf(fd, va);
    printf("imported into the V80 MMU at %p\n", va);

    // 3. one 64 B write from the V80 to va + 0x80
    const uint64_t target = reinterpret_cast<uint64_t>(va) + 0x80;
    v80.setCSR(target, VADDR);
    v80.setCSR(64, LEN);
    v80.setCSR(v80.getCtid(), PID);
    v80.setCSR(1, N_REPS);
    v80.setCSR(1, N_BEATS);
    v80.setCSR(START_WR, CTRL);
    const auto t0 = std::chrono::steady_clock::now();
    while (!v80.getCSR(DONE)) {
        if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(5)) {
            printf("FAIL: the V80 write never completed\n");
            return 1;
        }
    }
    usleep(1000);

    // 4. did it arrive?
    const uint64_t after = u280.getCSR(STAGING_WORD);
    printf("U280 RDMA_STAGING_VA after:  0x%016lx\n", (unsigned long) after);
    if (after == 1) {
        printf("PASS: the V80's peer-to-peer write reached the U280\n");
        return 0;
    }
    printf("FAIL: expected 0x1 (the first word of the V80's beat)%s\n",
           after == MARKER ? "; the register is unchanged, nothing arrived" : "");
    return 1;
}
