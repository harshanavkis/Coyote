/**
 * uwin_ptdma - AMD PTDMA (the host CPU's DMA engines) as Loom's copy engine,
 * host DRAM as the source: on the sending host a PTDMA engine
 * (~/loom-experiments/ptdma_bw, flag mode) copies host DRAM into window 1 of
 * the U280's user data window (uwin), then an 8 B flag into window 2, and
 * Loom lands both in the receiving host's memory.
 *
 *   rose (receiver): uwin_ptdma --server --size B [--iters N] [--port P]
 *   amy (sender):    uwin_ptdma --client <rose ip> --size B [--iters N] [--port P]
 *                               [--ko PATH] [--pci BDF] [--done FILE] [--wait-s S] [--cpu]
 *
 * server: exports a host buffer of B bytes (2 MiB pages; export 1, filled
 *   with POISON) and a 4 KiB flag (export 2, zero). A poller thread stamps
 *   each flag value 1..N when it first sees it; after each the buffer is
 *   checked against the pattern, and once more after the client says the
 *   copy is over (data missing at flag 1 but there at the end = the flag
 *   overtook the data). PASS: flags 1..N seen in order, every check clean.
 *   Flag-to-flag times on this host's clock give the arrival rate.
 * client: binds window 1 = uwin [0, B) to the server's buffer and window 2 =
 *   uwin [B, B + 4 KiB) to its flag (as uwin_pingpong), maps the uwin,
 *   prints the uwin's physical address and the insmod line for ptdma_bw
 *   (dst_phys = uwin offset 0, size = B, flag_off = B, nchan = 1), waits for
 *   FILE (default /tmp/uwin_ptdma_done, deleted at start), prints what the
 *   ingress counters saw, and tears down.
 *   --cpu: no PTDMA; the client does the same copy N times itself with CPU
 *   stores (cThread::uwinWrite, then the flag), for comparison.
 *
 * Pattern (ptdma_bw's source fill, PTBW_PATTERN): the 64-bit word at byte
 * offset o is 0x5054444d00000000 ^ o.
 *
 * Physical address of uwin offset 0: the driver maps the U280's BARs in
 * order and the third present one is the shell's (BAR_SHELL_CONFIG; the
 * XDMA's AXI bypass BAR, PCI BAR4 after the 64-bit BAR0 and BAR2); vFPGA
 * i's uwin is that BAR + VFPGA_UWIN_OFFS (0x0800_0000) + i x its share
 * (coyote_setup.c, uwin_phys_addr; cr_ctrl.tcl puts axim_udata_0 at
 * 0x0800_0000). vFPGA 0 here, so BAR4 + 0x0800_0000, read from sysfs and
 * cross-checked against the kernel's PAT list, which holds the uwin mapping.
 * A PTDMA write outside every window is accepted and dropped by
 * loom_ingress (counted); an address below the uwin hits the shell's
 * control registers instead, hence the checks.
 */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"

namespace {

constexpr uint32_t MAGIC = 0x55505444;                 // "UPTD"
constexpr uint64_t STAGING_SIZE = 1ULL << 20;
constexpr uint64_t PATTERN = 0x5054444d00000000ULL;    // ptdma_bw.c PTBW_PATTERN
constexpr uint64_t POISON  = 0xa5a5a5a5a5a5a5a5ULL;
constexpr uint64_t UWIN_OFFS  = 0x08000000;            // driver coyote_defs.h VFPGA_UWIN_OFFS
constexpr uint64_t UWIN_TOTAL = 0x08000000;            // VFPGA_UWIN_TOTAL = 2^UWIN_BITS (loom_table)
constexpr uint64_t FLAG_LEN = 4096;
using clk = std::chrono::steady_clock;

struct Hello { uint32_t magic; uint32_t dst_ctid; uint64_t staging_va, dst_ref, flag_ref, size; int64_t iters; };

bool write_full(int fd, const void *p, size_t n) {
    const char *c = static_cast<const char *>(p);
    while (n) { ssize_t k = ::write(fd, c, n); if (k <= 0) return false; c += k; n -= size_t(k); }
    return true;
}
bool read_full(int fd, void *p, size_t n) {
    char *c = static_cast<char *>(p);
    while (n) { ssize_t k = ::read(fd, c, n); if (k <= 0) return false; c += k; n -= size_t(k); }
    return true;
}
int connect_or_accept(const std::string &ip, uint16_t port) {
    int fd = -1;
    if (ip.empty()) {
        int lfd = ::socket(AF_INET, SOCK_STREAM, 0), one = 1;
        ::setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in a{};
        a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons(port);
        if (::bind(lfd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0 || ::listen(lfd, 1) < 0) return -1;
        fd = ::accept(lfd, nullptr, nullptr);
        ::close(lfd);
    } else {
        for (int i = 0; i < 300 && fd < 0; i++) {
            fd = ::socket(AF_INET, SOCK_STREAM, 0);
            sockaddr_in a{};
            a.sin_family = AF_INET; a.sin_port = htons(port);
            ::inet_pton(AF_INET, ip.c_str(), &a.sin_addr);
            if (::connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0) { ::close(fd); fd = -1; usleep(100000); }
        }
    }
    return fd;
}
double since_us(clk::time_point a, clk::time_point b) { return std::chrono::duration<double, std::micro>(b - a).count(); }

// The buffer against the pattern: wrong words, of them still POISON, the first wrong one
struct Check { uint64_t bad = 0, poison = 0, first_off = 0, first_got = 0; };
Check check(const uint64_t *dst, uint64_t size) {
    std::atomic_thread_fence(std::memory_order_acquire);
    Check c;
    for (uint64_t i = 0; i < size / 8; i++) {
        const uint64_t v = dst[i];
        if (v == (PATTERN ^ (8 * i))) continue;
        if (!c.bad++) { c.first_off = 8 * i; c.first_got = v; }
        c.poison += (v == POISON);
    }
    return c;
}
void print_check(const Check &c, uint64_t size) {
    printf("%lu of %lu words wrong (%lu still poison)", (unsigned long) c.bad, (unsigned long) (size / 8),
           (unsigned long) c.poison);
    if (c.bad)
        printf(", first at %#lx: %#lx, want %#lx", (unsigned long) c.first_off, (unsigned long) c.first_got,
               (unsigned long) (PATTERN ^ c.first_off));
}

// sysfs: BAR4 [start, end] of the PCI device, its driver
bool pci_bar4(const std::string &bdf, uint64_t &start, uint64_t &end, std::string &driver) {
    std::ifstream f("/sys/bus/pci/devices/" + bdf + "/resource");
    std::string line;
    for (int i = 0; i < 5; i++) if (!std::getline(f, line)) return false;
    unsigned long long s = 0, e = 0;
    if (sscanf(line.c_str(), "%llx %llx", &s, &e) != 2 || !s || e <= s) return false;
    start = s; end = e;
    char buf[512];
    const ssize_t n = readlink(("/sys/bus/pci/devices/" + bdf + "/driver").c_str(), buf, sizeof(buf));
    const std::string link = n > 0 ? std::string(buf, size_t(n)) : "(none)";
    driver = link.substr(link.rfind('/') + 1);
    return true;
}

// The kernel's PAT memtype list: the entry holding phys (the uwin mapping
// reserves its range there): 1 found (type in *type), 0 none, -1 unreadable
int pat_lookup(uint64_t phys, std::string &type) {
    std::ifstream f("/sys/kernel/debug/x86/pat_memtype_list");
    if (!f) return -1;
    std::string line;
    bool any = false;
    while (std::getline(f, line)) {
        unsigned long long s, e;
        char t[64];
        if (sscanf(line.c_str(), "PAT: [mem 0x%llx-0x%llx] %63s", &s, &e, t) != 3 &&   // since Linux 5.6
            sscanf(line.c_str(), "%63s @ 0x%llx-0x%llx", t, &s, &e) != 3) continue;
        any = true;
        if (s <= phys && phys < e) { type = t; return 1; }
    }
    return any ? 0 : -1;
}

}  // namespace

int main(int argc, char **argv) {
    setvbuf(stdout, nullptr, _IOLBF, 0);    // a script reads the insmod line while the client waits
    std::string ip, ko = "ptdma_bw.ko", pci = "0000:e1:00.0", done = "/tmp/uwin_ptdma_done";
    bool server = false, cpu = false;
    uint16_t port = 19860;
    uint64_t size = 1ULL << 20;
    int iters = 5, wait_s = 900, first_s = 600;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--server")                       server = true;
        else if (a == "--client" && i + 1 < argc)  ip = argv[++i];
        else if (a == "--port" && i + 1 < argc)    port = uint16_t(atoi(argv[++i]));
        else if (a == "--size" && i + 1 < argc)    size = strtoull(argv[++i], nullptr, 0);
        else if (a == "--iters" && i + 1 < argc)   iters = atoi(argv[++i]);
        else if (a == "--ko" && i + 1 < argc)      ko = argv[++i];
        else if (a == "--pci" && i + 1 < argc)     pci = argv[++i];
        else if (a == "--done" && i + 1 < argc)    done = argv[++i];
        else if (a == "--wait-s" && i + 1 < argc)  wait_s = atoi(argv[++i]);
        else if (a == "--first-s" && i + 1 < argc) first_s = atoi(argv[++i]);
        else if (a == "--cpu")                     cpu = true;
        else { server = false; ip.clear(); break; }
    }
    if (server == !ip.empty() || !size || size % 4096 || size + FLAG_LEN > UWIN_TOTAL || iters < 1) {
        printf("usage: %s --server | --client <ip>  --size B (a multiple of 4096, at most %lu) [--iters N] [--port P]\n"
               "       server: [--first-s S (wait for flag 1, default 600)]\n"
               "       client: [--ko PATH] [--pci BDF (default 0000:e1:00.0)] [--done FILE] [--wait-s S] [--cpu]\n",
               argv[0], (unsigned long) (UWIN_TOTAL - FLAG_LEN));
        return 2;
    }
    if (!server) unlink(done.c_str());

    coyote::cThread t_qp(0, getpid(), 0, nullptr, "loom_fpga");     // QP owner, rdma windows, the uwin mapping
    coyote::cThread t_data(0, getpid(), 0, nullptr, "loom_fpga");   // owns the landing buffers
    printf("%s: QP exchange on port %u ...\n", server ? "server" : "client", port);
    void *staging = server ? t_qp.initRDMA(STAGING_SIZE, port) : t_qp.initRDMA(STAGING_SIZE, port, ip.c_str());
    if (!staging) { printf("FAIL: initRDMA\n"); return 1; }

    uint64_t *dst = nullptr, *flag = nullptr;
    if (server) {
        dst  = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, size}));
        flag = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, FLAG_LEN}));
        for (uint64_t i = 0; i < size / 8; i++) dst[i] = POISON;
        memset(flag, 0, FLAG_LEN);
        loom_switch::program_export(t_qp, 1, t_data.getCtid(), dst, size);
        loom_switch::program_export(t_qp, 2, t_data.getCtid(), flag, FLAG_LEN);
    }

    const int fd = connect_or_accept(server ? std::string() : ip, uint16_t(port + 1));
    Hello mine{MAGIC, uint32_t(t_data.getCtid()), reinterpret_cast<uint64_t>(staging),
               loom_switch::export_ref(1), loom_switch::export_ref(2), size, iters}, h{};
    if (fd < 0 || !write_full(fd, &mine, sizeof(mine)) || !read_full(fd, &h, sizeof(h)) || h.magic != MAGIC ||
        h.size != size || h.iters != iters) {
        printf("FAIL: hello (--size and --iters must match on both sides)\n");
        return 1;
    }
    const auto t_ref = clk::now();
    char go = 1;

    // ---- server: wait for the flags, check the buffer -------------------
    if (server) {
        volatile uint64_t *vflag = flag;
        std::unique_ptr<std::atomic<int64_t>[]> seen(new std::atomic<int64_t>[iters + 1]);  // ns after the hello
        for (int k = 0; k <= iters; k++) seen[k].store(-1);
        std::atomic<bool> stop{false};
        std::atomic<bool> odd{false};      // the flag went down or past ITERS ...
        std::atomic<uint64_t> odd_val{0};  // ... to this
        std::atomic<int> skipped{0};       // values overwritten before the poller saw them
        std::thread poller([&] {
            uint64_t last = 0;
            while (!stop.load(std::memory_order_relaxed) && last < uint64_t(iters)) {
                const uint64_t v = *vflag;
                if (v == last) continue;
                const int64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>(clk::now() - t_ref).count();
                if (v < last || v > uint64_t(iters)) { odd_val.store(v); odd.store(true); break; }
                skipped += int(v - last - 1);
                for (uint64_t k = last + 1; k <= v; k++) seen[k].store(t);
                last = v;
            }
        });

        printf("server: %lu B export ready, waiting for flags 1..%d (flag 1 up to %d s, then 10 s each)\n",
               (unsigned long) size, iters, first_s);
        bool timeout = false, bad = false, client_end = false;
        Check first;
        std::vector<double> gbps;
        int k = 1;
        for (; k <= iters; k++) {
            // Up to first_s / 10 s, or 1 s once the client has said it is over
            const auto tw = clk::now();
            auto deadline = tw + (client_end ? std::chrono::seconds(1) : std::chrono::seconds(k == 1 ? first_s : 10));
            auto tpoll = tw;
            while (seen[k].load() < 0 && !odd.load()) {
                const auto now = clk::now();
                if (now > deadline) { timeout = true; break; }
                if (!client_end && now - tpoll > std::chrono::milliseconds(10)) {
                    tpoll = now;
                    pollfd p{fd, POLLIN, 0};
                    if (::poll(&p, 1, 0) > 0) { client_end = true; deadline = std::min(deadline, now + std::chrono::seconds(1)); }
                }
                std::this_thread::sleep_for(std::chrono::microseconds(20));
            }
            if (timeout || odd.load()) break;
            const Check c = check(dst, size);
            if (k == 1) first = c;
            bad |= c.bad != 0;
            const double t = seen[k].load() / 1e3;   // us
            printf("iter %d: flag %d at +%.3f ms", k, k, t / 1e3);
            if (k > 1) {
                const double d = t - seen[k - 1].load() / 1e3;
                if (d > 0) gbps.push_back(double(size) / d / 1e3);
                printf(", %.1f us after flag %d = %.3f GB/s", d, k - 1, d > 0 ? double(size) / d / 1e3 : 0.0);
            }
            printf("; dst ");
            print_check(c, size);
            printf("\n");
        }
        stop = true;
        poller.join();
        if (timeout) printf("FAIL: no flag %d (flag is %lu)%s\n", k, (unsigned long) *vflag,
                            client_end ? ", the client had ended" : "");
        if (odd.load()) printf("FAIL: flag went to %lu (want 1..%d in order)\n", (unsigned long) odd_val.load(), iters);
        if (skipped.load()) printf("note: %d flag values were overwritten before they were seen\n", skipped.load());
        if (!gbps.empty()) {
            std::sort(gbps.begin(), gbps.end());
            printf("arrival (this host's clock, flag to flag, %zu intervals): GB/s min %.3f median %.3f max %.3f\n",
                   gbps.size(), gbps.front(), gbps[gbps.size() / 2], gbps.back());
        }

        // The client says the copy is over (its done file, or its CPU copies)
        printf("server: waiting for the client's end\n");
        const bool ended = read_full(fd, &go, 1);
        const Check fin = check(dst, size);
        printf("final check%s: dst ", ended ? "" : " (client gone)");
        print_check(fin, size);
        printf(", flag %lu\n", (unsigned long) *vflag);
        if (first.bad && !fin.bad && seen[1].load() >= 0)
            printf("FAIL: data missing at flag 1 but there at the end: the flag overtook the data\n");
        const bool pass = !timeout && !odd.load() && !bad && !fin.bad && *vflag == uint64_t(iters);
        if (ended) write_full(fd, &go, 1);
        ::close(fd);
        loom_switch::release_export(t_qp, 1);
        loom_switch::release_export(t_qp, 2);
        t_qp.connSync(false);
        printf(pass ? "UWIN PTDMA PASS\n" : "UWIN PTDMA FAIL\n");
        return pass ? 0 : 1;
    }

    // ---- client: the windows, the uwin's address, the copy -------------
    loom_switch::csr_write(t_qp, loom_switch::RDMA_STAGING_VA, h.staging_va);
    loom_switch::program_window(t_qp, 1, true, t_qp.getCtid(), reinterpret_cast<void *>(h.dst_ref), size, 0, h.dst_ctid);
    loom_switch::program_window(t_qp, 2, true, t_qp.getCtid(), reinterpret_cast<void *>(h.flag_ref), FLAG_LEN, size, h.dst_ctid);
    void *uwin = t_qp.mapUwin(size + FLAG_LEN);

    int errors = 0;
    uint64_t bar_s = 0, bar_e = 0, phys = 0;
    std::string drv, pat;
    if (!pci_bar4(pci, bar_s, bar_e, drv)) {
        printf("FAIL: cannot read BAR4 of %s from sysfs\n", pci.c_str());
        errors++;
    } else {
        phys = bar_s + UWIN_OFFS;    // vFPGA 0
        printf("uwin: %s (driver %s) BAR4 [%#lx, %#lx], %lu MiB; uwin offset 0 = PHYS %#lx (BAR4 + %#lx); "
               "window 1 = [%#lx, %#lx), window 2 (flag) = [%#lx, %#lx); mapped at %p\n",
               pci.c_str(), drv.c_str(), (unsigned long) bar_s, (unsigned long) bar_e,
               (unsigned long) ((bar_e - bar_s + 1) >> 20), (unsigned long) phys, (unsigned long) UWIN_OFFS,
               (unsigned long) phys, (unsigned long) (phys + size), (unsigned long) (phys + size),
               (unsigned long) (phys + size + FLAG_LEN), uwin);
        if (drv != "coyote_driver") { printf("FAIL: %s is not driven by coyote_driver\n", pci.c_str()); errors++; }
        if (bar_e - bar_s + 1 < UWIN_OFFS + UWIN_TOTAL) {
            printf("FAIL: BAR4 is smaller than the uwin's end (%#lx)\n", (unsigned long) (UWIN_OFFS + UWIN_TOTAL));
            errors++;
        }
        const int p = pat_lookup(phys, pat);
        if (p == 1) printf("PAT check: the kernel's memtype list holds %#lx as %s%s\n", (unsigned long) phys,
                           pat.c_str(), pat == "write-combining" ? " (the uwin mapping)" : " (NOT write-combining)");
        else if (p == 0) { printf("FAIL: PAT check: no memtype entry holds %#lx: not the uwin mapping\n", (unsigned long) phys); errors++; }
        else printf("PAT check: /sys/kernel/debug/x86/pat_memtype_list unreadable, address from sysfs only\n");
    }

    const auto c0 = loom_switch::IngressCounters::read(t_qp);
    if (errors) {
        printf("FAIL: not printing the insmod line\n");
    } else if (!cpu) {
        printf("INSMOD: insmod %s dst_phys=%#lx size=%lu flag_off=%lu nchan=1 allow_write=1 numa=1 iters=%d\n",
               ko.c_str(), (unsigned long) phys, (unsigned long) size, (unsigned long) size, iters);
        printf("client: waiting for %s (up to %d s)\n", done.c_str(), wait_s);
        const auto tw = clk::now();
        while (access(done.c_str(), F_OK) != 0) {
            if (clk::now() - tw > std::chrono::seconds(wait_s)) { printf("FAIL: no %s after %d s\n", done.c_str(), wait_s); errors++; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        std::vector<uint64_t> src(size / 8);
        for (uint64_t i = 0; i < size / 8; i++) src[i] = PATTERN ^ (8 * i);
        std::vector<double> gbps;
        for (int k = 1; k <= iters; k++) {
            const uint64_t v = uint64_t(k);
            const auto t0 = clk::now();
            t_qp.uwinWrite(0, src.data(), size);
            t_qp.uwinWrite(size, &v, 8);
            const double us = since_us(t0, clk::now());
            gbps.push_back(double(size) / us / 1e3);
            printf("cpu iter %d: %lu B and flag %d stored in %.1f us = %.3f GB/s (stores issued, this host's clock)\n",
                   k, (unsigned long) size, k, us, gbps.back());
        }
        std::sort(gbps.begin(), gbps.end());
        printf("cpu stores: GB/s min %.3f median %.3f max %.3f\n", gbps.front(), gbps[gbps.size() / 2], gbps.back());
    }
    const auto c1 = loom_switch::IngressCounters::read(t_qp);
    printf("ingress counters moved:\n");
    loom_switch::print_ingress_delta(c0, c1);

    if (!write_full(fd, &go, 1) || !read_full(fd, &go, 1)) printf("(final barrier lost)\n");
    ::close(fd);
    t_qp.unmapUwin();
    loom_switch::release_window(t_qp, 1);
    loom_switch::release_window(t_qp, 2);
    t_qp.connSync(true);
    printf(errors ? "UWIN PTDMA CLIENT FAIL\n" : "UWIN PTDMA CLIENT DONE\n");
    return errors ? 1 : 0;
}
