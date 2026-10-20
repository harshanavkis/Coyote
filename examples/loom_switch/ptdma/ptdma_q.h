/* SPDX-License-Identifier: GPL-2.0-only */
/* ptdma_q's user interface (ptdma_q.c); shared by the module and user space */
#ifndef PTDMA_Q_H
#define PTDMA_Q_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define PTQ_MAX_REGIONS	16
#define PTQ_MAX_BATCH	64		/* copies per submit */
#define PTQ_MAX_DESC	(4ULL << 20)	/* bytes per descriptor (a longer copy becomes several) */

struct ptq_reg_host {	/* in: va (64 B aligned), len; out: handle, nseg (DMA-contiguous pieces) */
	__u64 va, len;
	__u32 handle, nseg;
};

struct ptq_reg_uwin {	/* out: handle, len, phys of the U280 uwin (module param uwin_bdf), its DMA address (iova) */
	__u64 len, phys, iova;
	__u32 handle, pad;
};

struct ptq_seg {	/* [off, off + len) of a region is DMA-contiguous at dma */
	__u64 off, dma, len;
};

struct ptq_segs {	/* in: handle, max entries at user address segs; out: n */
	__u64 segs;
	__u32 handle, max, n, pad;
};

struct ptq_copy {	/* len bytes from region src at src_off to region dst at dst_off */
	__u32 src, dst;
	__u64 src_off, dst_off, len;
};

struct ptq_submit {	/* in: n copies at user address copies; out: seq = descriptors submitted so far */
	__u64 copies;
	__u32 n, pad;
	__u64 seq;
};

struct ptq_info {	/* the ring for polling the head register (mmap page 0) and, in user mode, writing it */
	__u64 submitted, completed;
	__u32 ring_lo, qlen, desc_size, status, int_status, qcontrol, user_ok, pad;
};

#define PTQ_IOC_REG_HOST	_IOWR('Q', 1, struct ptq_reg_host)
#define PTQ_IOC_REG_UWIN	_IOWR('Q', 2, struct ptq_reg_uwin)
#define PTQ_IOC_UNREG		_IO('Q', 3)
#define PTQ_IOC_SUBMIT		_IOWR('Q', 4, struct ptq_submit)
#define PTQ_IOC_INFO		_IOR('Q', 5, struct ptq_info)
#define PTQ_IOC_SEGS		_IOWR('Q', 6, struct ptq_segs)
/* user mode (2026-10-19): this open writes descriptors into the ring (mmap page 1) and the tail register (page 0,
 * writable) itself; SUBMIT is refused from then on. Only where the IOMMU translates for the engine (amy, rose): a bad
 * descriptor then faults instead of writing anywhere. */
#define PTQ_IOC_USER		_IO('Q', 7)

/* registers in the mmapped queue page (ptdma.h CMD_Q_*_BASE) */
#define PTQ_CONTROL_OFF	0x0000
#define PTQ_TAIL_LO_OFF	0x0004
#define PTQ_HEAD_LO_OFF	0x0008
#define PTQ_CMD_Q_RUN	0x1
#define PTQ_DESC_DW0	0x500012	/* passthru engine 5, eom, ioc */

#endif
