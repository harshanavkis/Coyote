#pragma once
/**
 * Programs for loom_ce (hw/src/hdl/loom_ce.sv): a list of 64 B entries the
 * copy engine loads from card memory and runs with no host involvement.
 *
 *   ce_prog::Program p(v80);
 *   p.add(ce_prog::copy(src, dst, len));          // HBM -> dst (host stream)
 *   p.add(ce_prog::signal(flag, 1));              // behind the copy
 *   p.load();                                     // into the engine
 *   p.run(base);                                  // SIGNAL values, WAIT targets + base
 *
 * VAs are the cThread's: card memory is a buffer offloaded to the V80
 * (LOCAL_OFFLOAD), a host-stream destination a window (imported dma-buf) or
 * host memory. COPY/REDUCE lengths are multiples of 64, their VAs 64 B
 * aligned; SIGNAL, STAMP and WAIT words 8 B aligned.
 */
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>
#include <immintrin.h>

#include <coyote/cThread.hpp>

namespace ce_prog {

// loom_ce_ctrl.sv
enum Reg : uint32_t { PID = 4, BUSY = 8, COPIES = 9, PROG_VA = 11, PROG_N = 12, PROG_CMD = 13, RUN_BASE = 14,
                      ABORT = 15, RUNS = 16, PC = 17, NOW = 18, ABORTED = 19, POLLS = 20, RUN_CYC = 21 };
enum Op : uint64_t { COPY = 1, SIGNAL = 2, WAIT = 3, REDUCE = 4, STAMP = 5 };
constexpr uint64_t SRC_HOST = 1ULL << 4, DST_CARD = 1ULL << 5;
constexpr uint32_t N_PROG   = 2048;       // loom_ce N_PROG
constexpr double   CLK_NS   = 4.0;        // the V80 user clock, 250 MHz

struct Entry { uint64_t w[8]; };

inline uint64_t va(const void *p) { return reinterpret_cast<uint64_t>(p); }
inline Entry make(uint64_t op, uint64_t flags, uint64_t len, uint64_t a, uint64_t b, uint64_t dst, uint64_t val) {
    Entry e{};
    e.w[0] = op | flags | (len << 32);
    e.w[1] = a; e.w[2] = b; e.w[3] = dst; e.w[4] = val;
    return e;
}

// len bytes src -> dst; a source on the host stream (src_host) is a window read (a get)
inline Entry copy(const void *src, void *dst, uint64_t len, bool dst_card = false, bool src_host = false) {
    return make(COPY, (src_host ? SRC_HOST : 0) | (dst_card ? DST_CARD : 0), len, va(src), 0, va(dst), 0);
}
// 8 B value (+ RUN_BASE) to dst, behind every earlier write
inline Entry signal(void *dst, uint64_t val, bool dst_card = false) {
    return make(SIGNAL, dst_card ? DST_CARD : 0, 0, 0, 0, va(dst), val);
}
// until the 8 B word in card memory is >= target (+ RUN_BASE)
inline Entry wait(const void *word, uint64_t target) { return make(WAIT, 0, 0, 0, 0, va(word), target); }
// dst = a + b, int32 lanes; a and b in card memory
inline Entry reduce(const void *a, const void *b, void *dst, uint64_t len, bool dst_card = false) {
    return make(REDUCE, dst_card ? DST_CARD : 0, len, va(a), va(b), va(dst), 0);
}
// 8 B cycle counter (NOW) to dst when the entry is reached
inline Entry stamp(void *dst, bool dst_card = false) { return make(STAMP, dst_card ? DST_CARD : 0, 0, 0, 0, va(dst), 0); }

class Program {
public:
    explicit Program(coyote::cThread &t) : t_(t) {
        buf_ = static_cast<Entry *>(t_.getMem({coyote::CoyoteAllocType::HPF, N_PROG * sizeof(Entry)}));
        if (!buf_) throw std::runtime_error("ce_prog: no memory for the program");
    }
    void clear() { e_.clear(); }
    void add(const Entry &e) { e_.push_back(e); }
    size_t size() const { return e_.size(); }

    // Into card memory, then into the engine
    void load() {
        if (e_.empty() || e_.size() > N_PROG) throw std::runtime_error("ce_prog: program of " + std::to_string(e_.size()) + " entries");
        std::memcpy(buf_, e_.data(), e_.size() * sizeof(Entry));
        const uint64_t bytes = (e_.size() * sizeof(Entry) + 4095) & ~4095ULL;
        t_.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{buf_, bytes});
        t_.setCSR(t_.getCtid(), PID);
        t_.setCSR(va(buf_), PROG_VA);
        t_.setCSR(e_.size(), PROG_N);
        t_.setCSR(1, PROG_CMD);
        if (!idle(std::chrono::milliseconds(1000))) throw std::runtime_error("ce_prog: load did not finish");
    }

    // Start a run; SIGNAL values and WAIT targets get base added
    void start(uint64_t base = 0) {
        t_.setCSR(base, RUN_BASE);
        t_.setCSR(2, PROG_CMD);
    }
    // Wait for the run's end; on timeout ABORT it (ends a WAIT) and return false
    bool finish(std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
        if (idle(timeout)) return t_.getCSR(ABORTED) == 0;
        t_.setCSR(1, ABORT);
        idle(std::chrono::milliseconds(1000));
        return false;
    }
    bool run(uint64_t base = 0, std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
        start(base);
        return finish(timeout);
    }
    uint64_t run_cycles() { return t_.getCSR(RUN_CYC); }

private:
    bool idle(std::chrono::milliseconds timeout) {
        const auto t0 = std::chrono::steady_clock::now();
        while (t_.getCSR(BUSY)) {
            if (std::chrono::steady_clock::now() - t0 > timeout) return false;
            _mm_pause();
        }
        return true;
    }
    coyote::cThread   &t_;
    Entry             *buf_;
    std::vector<Entry> e_;
};

}  // namespace ce_prog
