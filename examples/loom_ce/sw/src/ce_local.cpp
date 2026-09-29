/**
 * ce_local - the local half of gate G4, on one host with both cards:
 *
 *   V80 HBM --loom_ce, peer-to-peer over PCIe--> U280 uwin
 *       --loom_ingress, local route--> host buffer
 *
 *   1. the host fills a buffer and offloads it into V80 card memory (HBM)
 *   2. the U280 gets two local windows: the destination buffer at uwin 0,
 *      a fence page right after it; the U280 driver exports that part of
 *      the uwin as a dma-buf and the V80 imports it into its MMU
 *   3. loom_ce copies HBM -> the imported uwin, then writes its copy count
 *      to the fence page, behind the data
 *   4. the host waits for the count in the fence page and checks every byte
 *
 * Needs the loom_switch bitstream on the U280 (coyote_driver with
 * MMAP_UWIN / EXPORT_REGION_UWIN) and loom_ce on the V80
 * (coyote_driver_versal).
 *
 * Usage: ce_local [bytes (multiple of 64, at most 64 MiB)] [copies]
 */
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <immintrin.h>
#include <stdexcept>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"

namespace {

// loom_ce_ctrl.sv
enum CeReg : uint32_t { START = 0, SRC_VA = 1, DST_VA = 2, LEN = 3, PID = 4, FENCE_VA = 5,
                        BUSY = 8, COPIES = 9, CYCLES = 10 };

uint64_t pattern(uint64_t off, int rep) { return 0xCE00000000000000ULL ^ (off * 0x9E3779B97F4A7C15ULL) ^ rep; }

}  // namespace

int main(int argc, char **argv) {
    const uint64_t size = (argc > 1) ? strtoull(argv[1], nullptr, 0) : (1ULL << 20);
    const int reps      = (argc > 2) ? atoi(argv[2]) : 1;
    if (size == 0 || size % 4096 || size > (64ULL << 20)) {
        fprintf(stderr, "bytes must be a multiple of 4 KiB, at most 64 MiB\n");
        return 2;
    }

    coyote::cThread u280(0, getpid(), 0, nullptr, "coyote_fpga");
    coyote::cThread v80(0, getpid(), 0, nullptr, "coyote_versal_fpga");

    // U280: destination and fence buffers, one local window each
    uint64_t *dst   = static_cast<uint64_t *>(u280.getMem({coyote::CoyoteAllocType::HPF, size}));
    uint64_t *fence = static_cast<uint64_t *>(u280.getMem({coyote::CoyoteAllocType::HPF, 4096}));
    memset(dst, 0, size);
    memset(fence, 0, 4096);
    loom_switch::program_window(u280, 1, false, u280.getCtid(), dst, size, 0);
    loom_switch::program_window(u280, 2, false, u280.getCtid(), fence, 4096, size);

    // The uwin's first size + 4 KiB, exported to the V80
    const uint64_t win_len = size + 4096;
    const int fd = u280.exportDmabuf(EXPORT_REGION_UWIN, 0, win_len);
    void *uva = mmap(nullptr, win_len, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (uva == MAP_FAILED) throw std::runtime_error("mmap for a reserved address failed");
    v80.importDmabuf(fd, uva);
    printf("U280 uwin [0, %lu) imported into the V80 MMU at %p\n", (unsigned long) win_len, uva);

    // V80: the source, in host memory then offloaded to HBM at the same VA
    uint64_t *src = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, size}));

    volatile uint64_t *vfence = fence;
    int errors = 0;
    for (int r = 0; r < reps; r++) {
        for (uint64_t i = 0; i < size / 8; i++) src[i] = pattern(8 * i, r);
        v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{src, size});
        memset(dst, 0, size);

        const uint64_t before = v80.getCSR(COPIES);
        loom_switch::IngressCounters c0 = loom_switch::IngressCounters::read(u280);
        v80.setCSR(reinterpret_cast<uint64_t>(src), SRC_VA);
        v80.setCSR(reinterpret_cast<uint64_t>(uva), DST_VA);
        v80.setCSR(size, LEN);
        v80.setCSR(v80.getCtid(), PID);
        v80.setCSR(reinterpret_cast<uint64_t>(uva) + size, FENCE_VA);
        const auto t0 = std::chrono::steady_clock::now();
        v80.setCSR(1, START);

        // The fence lands behind the data
        while (*vfence != before + 1 && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5))
            _mm_pause();
        const auto t1 = std::chrono::steady_clock::now();
        loom_switch::IngressCounters c1 = loom_switch::IngressCounters::read(u280);

        uint64_t bad = 0, first = ~0ULL;
        for (uint64_t i = 0; i < size / 8; i++)
            if (dst[i] != pattern(8 * i, r)) { if (!bad) first = i; bad++; }
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        if (*vfence != before + 1) {
            printf("FAIL copy %d: fence %lu, expected %lu (copy engine busy %lu)\n", r,
                   (unsigned long) *vfence, (unsigned long) (before + 1), (unsigned long) v80.getCSR(BUSY));
            errors++;
        } else if (bad) {
            printf("FAIL copy %d: %lu of %lu words wrong, first at byte %lu (%016lx, expected %016lx)\n",
                   r, (unsigned long) bad, (unsigned long) (size / 8), (unsigned long) (8 * first),
                   (unsigned long) dst[first], (unsigned long) pattern(8 * first, r));
            errors++;
        } else {
            printf("ok   copy %d: %lu bytes, fence after %.1f us (%.2f GB/s), CE %lu cycles to the data write's completion\n",
                   r, (unsigned long) size, us, size / us / 1e3, (unsigned long) v80.getCSR(CYCLES));
        }
        printf("     ingress: %lu bursts, %lu local packets, %lu stores, %lu dropped, %lu partial words\n",
               (unsigned long) (c1.v[0] - c0.v[0]), (unsigned long) (c1.v[2] - c0.v[2]),
               (unsigned long) (c1.v[4] - c0.v[4]), (unsigned long) (c1.v[1] - c0.v[1]),
               (unsigned long) (c1.v[5] - c0.v[5]));
    }

    loom_switch::release_window(u280, 1);
    loom_switch::release_window(u280, 2);
    printf(errors ? "G4-local FAIL\n" : "G4-local PASS\n");
    return errors ? 1 : 0;
}
