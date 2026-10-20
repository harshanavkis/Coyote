#pragma once

/**
 * ptq - user side of ptdma_q (examples/loom_switch/ptdma/ptdma_q.c): one AMD PTDMA engine as a copy engine.
 * Copies between registered regions (host buffers, this host's U280 uwin) are submitted in batches with one
 * ioctl each; completion is the engine's head register, read from a read-only mapping of its queue page.
 * Descriptors of one queue run in ring order.
 * enable_user() (2026-10-19, hosts whose IOMMU translates for the engine: amy, rose): from then on submit() writes the
 * descriptors into the mapped ring and the tail register itself - no syscall per copy.
 */
#include <immintrin.h>
#include <map>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include "../../ptdma/ptdma_q.h"

namespace ptq {

class Queue {
public:
    explicit Queue(const std::string &dev) {
        fd_ = ::open(dev.c_str(), O_RDWR);
        if (fd_ < 0) throw std::runtime_error("ptq: open " + dev + ": " + strerror(errno));
        void *p = ::mmap(nullptr, 4096, PROT_READ, MAP_SHARED, fd_, 0);
        if (p == MAP_FAILED) { ::close(fd_); throw std::runtime_error("ptq: mmap of the queue page failed"); }
        regs_ = static_cast<volatile uint32_t *>(p);
        ptq_info i = info();
        ring_lo_ = i.ring_lo; qlen_ = i.qlen; seq_ = i.submitted;
    }
    ~Queue() {
        ::munmap(const_cast<uint32_t *>(regs_), 4096);
        if (ring_) ::munmap(ring_, ring_bytes_);
        ::close(fd_);
    }
    Queue(const Queue &) = delete;
    Queue &operator=(const Queue &) = delete;

    uint32_t reg_host(const void *va, uint64_t len, uint32_t *nseg = nullptr) {
        ptq_reg_host a{reinterpret_cast<uint64_t>(va), len, 0, 0};
        if (::ioctl(fd_, PTQ_IOC_REG_HOST, &a)) throw std::runtime_error(std::string("ptq: REG_HOST: ") + strerror(errno));
        if (nseg) *nseg = a.nseg;
        std::vector<ptq_seg> &v = segs_[a.handle];     // the DMA addresses, for user mode
        v.resize(a.nseg);
        ptq_segs q{reinterpret_cast<uint64_t>(v.data()), a.handle, a.nseg, 0, 0};
        if (::ioctl(fd_, PTQ_IOC_SEGS, &q)) throw std::runtime_error(std::string("ptq: SEGS: ") + strerror(errno));
        return a.handle;
    }
    // this host's U280 uwin (the module's uwin_bdf); offset 0 = uwin offset 0
    uint32_t reg_uwin(uint64_t *len = nullptr, uint64_t *phys = nullptr) {
        ptq_reg_uwin a{};
        if (::ioctl(fd_, PTQ_IOC_REG_UWIN, &a)) throw std::runtime_error(std::string("ptq: REG_UWIN: ") + strerror(errno));
        if (len) *len = a.len;
        if (phys) *phys = a.phys;
        segs_[a.handle] = {ptq_seg{0, a.iova, a.len}};
        return a.handle;
    }
    void unreg(uint32_t h) { ::ioctl(fd_, PTQ_IOC_UNREG, h); segs_.erase(h); }

    // user mode: the module refuses it (EPERM) where no IOMMU translates for the engine (clara)
    bool enable_user() {
        if (::ioctl(fd_, PTQ_IOC_USER)) return false;
        ::munmap(const_cast<uint32_t *>(regs_), 4096);
        void *p = ::mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (p == MAP_FAILED) throw std::runtime_error("ptq: mmap of the queue page (writable) failed");
        regs_ = static_cast<volatile uint32_t *>(p);
        const ptq_info i = info();
        ring_bytes_ = (uint64_t(i.qlen) * i.desc_size + 4095) & ~4095ULL;
        void *r = ::mmap(nullptr, ring_bytes_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 4096);
        if (r == MAP_FAILED) throw std::runtime_error("ptq: mmap of the ring failed");
        ring_ = static_cast<uint32_t *>(r);
        qcontrol_ = i.qcontrol;
        ring_lo_ = i.ring_lo;
        qlen_ = i.qlen;
        seq_ = done_cache_ = 0;      // the module reset the ring
        user_ = true;
        return true;
    }
    bool user() const { return user_; }

    // n <= PTQ_MAX_BATCH copies, in order; returns the sequence number the last of them completes at
    uint64_t submit(const ptq_copy *c, uint32_t n) {
        if (user_) return submit_user(c, n);
        ptq_submit s{reinterpret_cast<uint64_t>(c), n, 0, 0};
        if (::ioctl(fd_, PTQ_IOC_SUBMIT, &s)) throw std::runtime_error(std::string("ptq: SUBMIT: ") + strerror(errno));
        return seq_ = s.seq;
    }
    uint64_t copy(uint32_t src, uint64_t src_off, uint32_t dst, uint64_t dst_off, uint64_t len) {
        ptq_copy c{src, dst, src_off, dst_off, len};
        return submit(&c, 1);
    }
    // descriptors finished (valid while at most qlen - 1 are outstanding, which the module enforces)
    uint64_t completed() const {
        const uint32_t head = (regs_[PTQ_HEAD_LO_OFF / 4] - ring_lo_) / 32, tail = uint32_t(seq_ % qlen_);
        return seq_ - ((tail - head) & (qlen_ - 1));
    }
    bool done(uint64_t s) const { return completed() >= s; }
    ptq_info info() const {
        ptq_info i{};
        if (::ioctl(fd_, PTQ_IOC_INFO, &i)) throw std::runtime_error(std::string("ptq: INFO: ") + strerror(errno));
        return i;
    }
    uint64_t seq() const { return seq_; }

private:
    // the DMA address of off in region h and how many bytes stay contiguous from there
    uint64_t dma(uint32_t h, uint64_t off, uint64_t *run) const {
        const std::vector<ptq_seg> &v = segs_.at(h);
        size_t lo = 0, hi = v.size();
        while (hi - lo > 1) { const size_t m = (lo + hi) / 2; if (v[m].off <= off) lo = m; else hi = m; }
        *run = v[lo].off + v[lo].len - off;
        return v[lo].dma + (off - v[lo].off);
    }
    uint64_t submit_user(const ptq_copy *c, uint32_t n) {
        for (uint32_t i = 0; i < n; i++) {
            uint64_t so = c[i].src_off, to = c[i].dst_off, left = c[i].len;
            while (left) {
                uint64_t sr, tr;
                const uint64_t sd = dma(c[i].src, so, &sr), td = dma(c[i].dst, to, &tr);
                const uint64_t len = std::min(std::min(left, std::min(sr, tr)), uint64_t(PTQ_MAX_DESC));
                while (seq_ - done_cache_ >= qlen_ - 1) done_cache_ = completed();   // ring full: the head register
                uint32_t *d = ring_ + (seq_ % qlen_) * 8;
                d[0] = PTQ_DESC_DW0; d[1] = uint32_t(len);
                d[2] = uint32_t(sd); d[3] = uint32_t(sd >> 32) & 0xffff;
                d[4] = uint32_t(td); d[5] = uint32_t(td >> 32) & 0xffff;
                d[6] = d[7] = 0;
                seq_++;
                so += len; to += len; left -= len;
            }
        }
        asm volatile("sfence" ::: "memory");    // descriptors in memory (and not reordered by the compiler) before the tail
        regs_[PTQ_TAIL_LO_OFF / 4] = ring_lo_ + uint32_t(seq_ % qlen_) * 32;
        regs_[PTQ_CONTROL_OFF / 4] = qcontrol_ | PTQ_CMD_Q_RUN;
        return seq_;
    }

    int fd_ = -1;
    volatile uint32_t *regs_ = nullptr;
    uint32_t *ring_ = nullptr;
    uint64_t ring_bytes_ = 0;
    uint32_t ring_lo_ = 0, qlen_ = 0, qcontrol_ = 0;
    uint64_t seq_ = 0, done_cache_ = 0;
    bool user_ = false;
    std::map<uint32_t, std::vector<ptq_seg>> segs_;
};

}  // namespace ptq
