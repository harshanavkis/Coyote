/**
 * Loom test — two mechanisms, size-selected (like a GPU copy engine).
 *   DMA  (large): post descriptor {src,dst,len}, poll done. host<->host copy.
 *   APERTURE (small): setCSR/getCSR to an aperture window -> HW traps each and
 *     performs it against the PEER thread's memory (8 B/access).
 *       write = posted sq_wr (fast) ; read = blocking sq_rd (slow round trip).
 * 2 cThreads A(stream0)/B(stream1). READ = A<-B ; WRITE = A->B.
 */

#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <functional>
#include <coyote/cThread.hpp>

constexpr int DEFAULT_VFPGA_ID = 0;
constexpr uint32_t THRESH = 128;   // <THRESH bytes -> aperture, else DMA

enum LoomReg : uint32_t {
    CMD_REG=0, STATUS_REG=1, DONE_CNT_REG=2,
    SRC_PID_REG=3, SRC_ADDR_REG=4, SRC_STRM_REG=5,
    DST_PID_REG=6, DST_ADDR_REG=7, DST_STRM_REG=8, LEN_REG=9,
    AP_PID_REG=10, AP_BASE_LO_REG=11, AP_BASE_HI_REG=12, AP_STRM_REG=13,
    AP_LO=16,   // aperture window base reg (16 regs = 128 B)
};

static coyote::cThread* g_ct;

// ---- DMA copy engine (large) ----
static void dma(uint64_t s_pid, void* s_addr, uint64_t s_strm,
                uint64_t d_pid, void* d_addr, uint64_t d_strm, uint32_t len) {
    uint64_t before = g_ct->getCSR(DONE_CNT_REG);
    g_ct->setCSR(s_pid, SRC_PID_REG); g_ct->setCSR((uint64_t)s_addr, SRC_ADDR_REG); g_ct->setCSR(s_strm, SRC_STRM_REG);
    g_ct->setCSR(d_pid, DST_PID_REG); g_ct->setCSR((uint64_t)d_addr, DST_ADDR_REG); g_ct->setCSR(d_strm, DST_STRM_REG);
    g_ct->setCSR(len, LEN_REG);
    g_ct->setCSR(0x1, CMD_REG);
    while (g_ct->getCSR(DONE_CNT_REG) == before) {}
}

// ---- Aperture (small): point the window at a peer buffer, then word accesses ----
static void ap_config(uint64_t peer_ctid, void* peer_base, uint64_t peer_strm) {
    g_ct->setCSR(peer_ctid,                 AP_PID_REG);
    g_ct->setCSR((uint64_t)peer_base & 0xFFFFFFFF, AP_BASE_LO_REG);
    g_ct->setCSR((uint64_t)peer_base >> 32,        AP_BASE_HI_REG);
    g_ct->setCSR(peer_strm,                 AP_STRM_REG);
}
// write `size` bytes (<=128, mult of 8) into peer memory via trapped writes (posted)
static void ap_write(void* src, uint32_t size) {
    uint64_t* w = (uint64_t*) src;
    for (uint32_t k = 0; k < size/8; k++) g_ct->setCSR(w[k], AP_LO + k);
}
// read `size` bytes from peer memory via trapped reads (blocking round trips)
static void ap_read(void* dst, uint32_t size) {
    uint64_t* w = (uint64_t*) dst;
    for (uint32_t k = 0; k < size/8; k++) w[k] = g_ct->getCSR(AP_LO + k);
}

static double us_of(std::function<void()> fn, int iters) {
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iters; i++) fn();
    auto t1 = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::micro>(t1 - t0).count() / iters;
}

int main(int argc, char* argv[]) {
    coyote::cThread thr_a(DEFAULT_VFPGA_ID, getpid());
    coyote::cThread thr_b(DEFAULT_VFPGA_ID, getpid());
    g_ct = &thr_a;

    const uint32_t MAX = 4096;
    char* a = (char*) thr_a.getMem({coyote::CoyoteAllocType::HPF, MAX});
    char* b = (char*) thr_b.getMem({coyote::CoyoteAllocType::HPF, MAX});
    if (!a || !b) { std::cerr << "getMem failed\n"; return 1; }
    const uint64_t ca = thr_a.getCtid(), cb = thr_b.getCtid();

    bool all = true;
    auto chk = [&](const char* t, bool ok){ std::cout << (ok?"PASS":"FAIL") << ": " << t << "\n"; all &= ok; };

    // ---- APERTURE (small) — peer = B ----
    ap_config(cb, b, 1);
    const uint32_t sz = 64;   // <=128, mult of 8

    // WRITE A->B: put A's bytes into B via trapped posted writes
    for (uint32_t i=0;i<sz;i++) a[i]=(char)((i*7)&0xFF);  std::memset(b,0,sz);
    ap_write(a, sz);
    chk("APERTURE WRITE (A->B, posted)", std::memcmp(a,b,sz)==0);

    // READ A<-B: pull B's bytes into A via trapped blocking reads
    for (uint32_t i=0;i<sz;i++) b[i]=(char)((i*3)&0xFF);  std::memset(a,0,sz);
    ap_read(a, sz);
    chk("APERTURE READ  (A<-B, blocking)", std::memcmp(a,b,sz)==0);

    // ---- DMA (large) ----
    const uint32_t big = 4096;
    for (uint32_t i=0;i<big;i++) b[i]=(char)((i+1)&0xFF);  std::memset(a,0,big);
    dma(cb,b,1, ca,a,0, big);    // READ A<-B
    chk("DMA READ  (A<-B)", std::memcmp(a,b,big)==0);
    for (uint32_t i=0;i<big;i++) a[i]=(char)((i*5)&0xFF);  std::memset(b,0,big);
    dma(ca,a,0, cb,b,1, big);    // WRITE A->B
    chk("DMA WRITE (A->B)", std::memcmp(a,b,big)==0);

    // ---- latency: aperture write vs read (reads should be much slower) ----
    ap_config(cb, b, 1);
    double w_us = us_of([&](){ ap_write(a, sz); }, 200) / (sz/8);   // per 8B word
    double r_us = us_of([&](){ ap_read(a, sz); }, 200) / (sz/8);
    std::cout << "aperture per-8B: write " << std::fixed << std::setprecision(3) << w_us
              << " us | read " << r_us << " us  (read/write = " << r_us/w_us << "x)\n";

    std::cout << (all ? "\nALL PASS\n" : "\nFAIL\n");
    return all ? 0 : 2;
}
