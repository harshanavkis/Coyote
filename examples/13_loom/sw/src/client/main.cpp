/**
 * Loom client (source node).
 *   LOCAL  : two cThreads on one host, routed through the FPGA switch.
 *            <=8 B -> aperture (trapped AXI-Lite); >8 B -> DMA descriptor.
 *   REMOTE : same switch, range table binds the address to RDMA egress.
 * Run without --server_ip for local-only tests (no RDMA needed).
 */

#include <iostream>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <functional>
#include <boost/program_options.hpp>
#include <coyote/cThread.hpp>

#include "loom_regs.hpp"

using namespace loom;
constexpr int DEFAULT_VFPGA_ID = 0;

static double us_of(std::function<void()> fn, int iters) {
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iters; i++) fn();
    auto t1 = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::micro>(t1 - t0).count() / iters;
}

int main(int argc, char* argv[]) {
    std::string server_ip;
    uint32_t rdma_size = 4096;
    boost::program_options::options_description opts("Loom client");
    opts.add_options()
        ("server_ip,s", boost::program_options::value<std::string>(&server_ip), "server IP (omit for local-only)")
        ("size,z", boost::program_options::value<uint32_t>(&rdma_size), "RDMA transfer size");
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);
    const bool do_rdma = vm.count("server_ip") > 0;

    coyote::cThread thr_a(DEFAULT_VFPGA_ID, getpid());
    coyote::cThread thr_b(DEFAULT_VFPGA_ID, getpid());

    const uint32_t MAX = 4096;
    char* a = (char*) thr_a.getMem({coyote::CoyoteAllocType::HPF, MAX});
    char* b = (char*) thr_b.getMem({coyote::CoyoteAllocType::HPF, MAX});
    if (!a || !b) { std::cerr << "getMem failed\n"; return 1; }
    const uint64_t ca = thr_a.getCtid(), cb = thr_b.getCtid();

    bool all = true;
    auto chk = [&](const char* t, bool ok){ std::cout << (ok?"PASS":"FAIL") << ": " << t << "\n"; all &= ok; };

    thr_a.setCSR(0, ROLE_REG);          // source node
    clear_ranges(thr_a);                // empty table -> plain local routing
    clear_counters(thr_a);

    // ---------------- LOCAL ----------------
    ap_config(thr_a, cb, b, 1);
    const uint32_t sz = THRESH;         // 8 B = one AXI-Lite transaction

    for (uint32_t i=0;i<sz;i++) a[i]=(char)((i*7)&0xFF);  std::memset(b,0,sz);
    ap_write(thr_a, a, sz);
    chk("APERTURE WRITE (A->B, 8B posted)", std::memcmp(a,b,sz)==0);

    for (uint32_t i=0;i<sz;i++) b[i]=(char)((i*3)&0xFF);  std::memset(a,0,sz);
    ap_read(thr_a, a, sz);
    chk("APERTURE READ  (A<-B, 8B blocking)", std::memcmp(a,b,sz)==0);

    const uint32_t big = 4096;
    for (uint32_t i=0;i<big;i++) b[i]=(char)((i+1)&0xFF);  std::memset(a,0,big);
    dma(thr_a, cb,b,1, ca,a,0, big);
    chk("DMA READ  (A<-B)", std::memcmp(a,b,big)==0);
    for (uint32_t i=0;i<big;i++) a[i]=(char)((i*5)&0xFF);  std::memset(b,0,big);
    dma(thr_a, ca,a,0, cb,b,1, big);
    chk("DMA WRITE (A->B)", std::memcmp(a,b,big)==0);

    // ---------------- Routing table: drop path must not hang the CPU ----------------
    // Entry 0 = the peer buffer -> LOCAL. Anything else has no entry -> DROP.
    clear_counters(thr_a);
    set_range(thr_a, 0, (uint64_t)b, MAX, RT_LOCAL, 1, cb, ING_BOTH);
    uint64_t drops0 = thr_a.getCSR(DROP_CNT_REG);
    ap_config(thr_a, cb, (void*)0xdead0000ULL, 1);      // unmapped -> no range match
    uint64_t junk = 0;
    ap_read(thr_a, &junk, 8);                            // must return promptly
    bool dropped = (thr_a.getCSR(DROP_CNT_REG) > drops0);
    chk("DROP: unmapped aperture read completes (no hang)", dropped);
    std::cout << "      returned 0x" << std::hex << junk << std::dec << " (expect DEAD sentinel)\n";

    ap_config(thr_a, cb, b, 1);                          // restore

    // Buffer A needs its own entry: the crossover DMAs write INTO a, and the
    // table matches on the DESTINATION address. Without this every DMA below
    // would drop and dma()'s DONE_CNT poll would spin forever.
    set_range(thr_a, 1, (uint64_t)a, MAX, RT_LOCAL, 0, ca, ING_BOTH);
    clear_counters(thr_a);

    // ---------------- crossover ----------------
    std::cout << "\n bytes | aperture wr | aperture rd |    DMA rd   (lower is better)\n";
    for (uint32_t n : {8u, 16u, 32u, 64u, 128u}) {
        double aw = us_of([&](){ ap_write(thr_a, a, n); }, 200);
        double ar = us_of([&](){ ap_read (thr_a, a, n); }, 200);
        double dr = us_of([&](){ dma(thr_a, cb,b,1, ca,a,0, n); }, 200);
        std::cout << std::setw(6) << n << " | " << std::fixed << std::setprecision(2)
                  << std::setw(9) << aw << "us | " << std::setw(9) << ar << "us | "
                  << std::setw(8) << dr << "us"
                  << (n <= THRESH ? "   <- aperture" : "   <- DMA") << "\n";
    }
    print_counters(thr_a, "LOCAL (source side)");

    // ---------------- REMOTE ----------------
    if (do_rdma) {
        std::cout << "\n-- RDMA to " << server_ip << " --\n";
        char* r = (char*) thr_a.initRDMA(rdma_size, coyote::DEF_PORT, server_ip.c_str());
        if (!r) { std::cerr << "initRDMA failed\n"; return 1; }

        thr_a.setCSR(thr_a.getCtid(), COYOTE_PID_REG);

        // Map the local RDMA window onto the peer's window. HW translates
        // remote_base + (addr - base), so the app just posts an ordinary
        // descriptor against a local address and the switch does the rest.
        const uint64_t peer_base = (uint64_t) thr_a.getQpair()->remote.vaddr;
        set_range(thr_a, 2, (uint64_t)r, rdma_size, RT_REMOTE, 0, thr_a.getCtid(),
                  ING_BOTH, peer_base);
        std::cout << "  local  window base = 0x" << std::hex << (uint64_t)r << "\n";
        std::cout << "  remote window base = 0x" << peer_base << std::dec << "\n";
        clear_counters(thr_a);

        for (uint32_t i = 0; i < rdma_size; i++) r[i] = (char)((i * 11) & 0xFF);
        thr_a.connSync(true);

        // Whole window at offset 0: HW reads r locally, then RDMA-writes it to
        // the translated peer address (peer_base + 0).
        dma(thr_a, thr_a.getCtid(), r, 0, thr_a.getCtid(), r, 0, rdma_size);

        // Offset transfer — this is what proves the translation is real rather
        // than a fixed base. Writing r+OFF must land at peer_base+OFF, not at
        // peer_base. The server checks exactly that.
        if (rdma_size >= XLAT_OFF + XLAT_LEN) {
            for (uint32_t i = 0; i < XLAT_LEN; i++) r[XLAT_OFF + i] = (char)XLAT_MARK;
            dma(thr_a, thr_a.getCtid(), r + XLAT_OFF, 0,
                       thr_a.getCtid(), r + XLAT_OFF, 0, XLAT_LEN);
            std::cout << "  offset xfer: local 0x" << std::hex << (uint64_t)(r + XLAT_OFF)
                      << " -> remote 0x" << (peer_base + XLAT_OFF) << std::dec << "\n";
        }

        thr_a.connSync(true);
        std::cout << "PASS: RDMA WRITEs issued - verify on server\n";
        print_counters(thr_a, "REMOTE push (source side)");

        // ---- PULL: remote SOURCE, local destination ----
        // Still an ordinary DMA descriptor. The switch sees a remote source and
        // converts it into an RDMA READ, landing the response locally.
        clear_counters(thr_a);
        std::memset(a, 0, MAX);
        const uint32_t pull_len = (rdma_size < MAX ? rdma_size : MAX);
        thr_a.connSync(true);                    // server has its buffer ready
        dma(thr_a, thr_a.getCtid(), r, 0, ca, a, 0, pull_len);

        // The server's buffer holds (i*11) except the marker window; the pull
        // should reproduce that byte for byte in `a`.
        bool pull_ok = true;
        for (uint32_t i = 0; i < pull_len; i++) {
            const bool in_probe = (rdma_size >= XLAT_OFF + XLAT_LEN)
                                  && (i >= XLAT_OFF) && (i < XLAT_OFF + XLAT_LEN);
            uint8_t want = in_probe ? XLAT_MARK : (uint8_t)((i * 11) & 0xFF);
            if ((uint8_t)a[i] != want) {
                std::cout << "  pull mismatch at " << i << ": got 0x" << std::hex
                          << (int)(uint8_t)a[i] << " want 0x" << (int)want << std::dec << "\n";
                pull_ok = false; break;
            }
        }
        chk("RDMA PULL (DMA read from remote source)", pull_ok);
        print_counters(thr_a, "REMOTE pull (source side)");
    } else {
        std::cout << "\n(no --server_ip: local-only run, RDMA path not exercised)\n";
    }

    std::cout << (all ? "\nALL PASS\n" : "\nFAIL\n");
    return all ? 0 : 2;
}
