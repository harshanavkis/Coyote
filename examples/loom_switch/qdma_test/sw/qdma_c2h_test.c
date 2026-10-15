// SPDX-License-Identifier: GPL-2.0
/*
 * qdma_c2h_test: every QDMA path the U280 soft-QDMA static (Coyote 7cb6023d) uses, on the qdma_hw rev2 test image
 * (10ee:9f3f; IP = the static's QDMA config with pfch_cache_depth 64; C2H/H2C logic = the static's qdma_wr_wrapper /
 * qdma_rd_wrapper / qdma_reg_slice, driven by a shell stand-in; control + counters in BAR4's top 1 MB).
 *
 * Setup = Coyote driver/src/platform/pci_qdma.c enable_queues()/enable_queue() (copied; waits bounded): mask, FMAP
 * (qid 0..511), H2C queues 1..64, C2H buffer size (0xAB0) = PAGE_SIZE, C2H queues 256..287 with prefetch context and a
 * prefetch tag each (all 32 must be granted for their own qid), tags delivered to the wrapper's tag table as the
 * driver does (static_slave register 17 encoding).
 * Tests: C2H singles 64 B / 4 KiB on each of the 24 wrapper queues (qid + tag on the wire checked), C2H back-to-back
 * (3 channels), H2C singles + back-to-back (data checked in the FPGA), C2H / H2C / concurrent throughput.
 * Every wait is bounded. If a transfer gets stuck the module stops testing, keeps its DMA buffers (never freed while
 * the QDMA may still write them) and does not touch the contexts again (no hang on rmmod).
 *
 *   insmod qdma_c2h_test.ko [tput_cnt=65536]   -> results in dmesg ("qdma_c2h_test:"), lines "RESULT <test> PASS|FAIL"
 */
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/ktime.h>
#include <linux/io.h>
#include <linux/slab.h>

#define DRV "qdma_c2h_test"

static unsigned int tput_cnt = 65536;   /* 4 KiB packets per channel in the throughput runs */
module_param(tput_cnt, uint, 0444);
static unsigned int run_tests = 1;
module_param(run_tests, uint, 0444);

/* ---- QDMA registers (Coyote driver/include/coyote_defs.h) */
#define QDMA_N_QUEUES               512
#define QDMA_RD_QUEUE_START_IDX     1
#define QDMA_WR_QUEUE_START_IDX     (QDMA_N_QUEUES / 2)
#define QDMA_N_ACTIVE_QUEUES        64
#define QDMA_N_C2H_QUEUES           32          /* U+ (7cb6023d) */
#define QDMA_CTX_CLR                0
#define QDMA_CTX_WR                 1
#define QDMA_CTX_RD                 2
#define QDMA_CTX_INV                3
#define QDMA_CTX_CMD_REG            0x844
#define QDMA_CTX_N_DATA_REGS        8
#define QDMA_CTX_DATA_REG_START     0x804
#define QDMA_CTX_MASK_REG_START     0x824
#define QDMA_CXT_MASK_DEF_VAL       0xFFFFFFFF
#define QDMA_CTX_BUSY_VAL_DEAULT    0x00000000
#define QDMA_CTX_SEL_SHIFT          1
#define QDMA_CTX_SEL_MASK           0x0000000F
#define QDMA_CTX_OP_SHIFT           5
#define QDMA_CTX_OP_MASK            0x00000003
#define QDMA_CTX_QID_SHIFT          7
#define QDMA_CTX_QID_MASK           0x00001FFF
#define QDMA_CTXT_SELC_DEC_SW_C2H   0x0
#define QDMA_CTXT_SELC_DEC_SW_H2C   0x1
#define QDMA_CTXT_SELC_DEC_HW_C2H   0x2
#define QDMA_CTXT_SELC_DEC_HW_H2C   0x3
#define QDMA_CTXT_SELC_DEC_CR_C2H   0x4
#define QDMA_CTXT_SELC_DEC_CR_H2C   0x5
#define QDMA_CTXT_SELC_WRB          0x6
#define QDMA_CTXT_SELC_PFTCH        0x7
#define QDMA_CTXT_SELC_TIMER        0xb
#define QDMA_CTXT_SELC_FMAP         0xc
#define QDMA_C2H_PFCH_BYP_QID_REG   0x1408
#define QDMA_C2H_PFCH_BYP_TAG_REG   0x140c
#define DMA_MIN_SLEEP_CMD           10

#define BAR_DMA     2               /* the static's BAR_DMA_CONFIG (BAR0 AXI-Lite, BAR2 DMA, BAR4 bridge) */
#define BAR_BRIDGE  4
#define WIN_OFF     0x0FF00000UL     /* BAR4 control/status window */
#define WIN_LEN     0x1000UL

/* ---- test image registers (qdma_hw rev2 qdma_hw2_core.sv) */
#define CTL_CMD     0               /* bit0 C2H go, bit1 H2C go, bit2 clear counters, bit3 abort */
#define CTL_PFCH    1
#define CTL_C2HMASK 2
#define CTL_H2CMASK 3
#define CTL_C2H(ch) (8 + 8 * (ch))  /* +0 base +1 len +2 count +3 stride +4 wrap +5 seed */
#define CTL_H2C(ch) (32 + 8 * (ch))
#define ST_MAGIC 0
#define ST_C2H_CMD 2
#define ST_C2H_BEAT 3
#define ST_C2H_TLAST 4
#define ST_C2H_BYTES 5
#define ST_STS 6
#define ST_STS_DROP 7
#define ST_STS_ERR 8
#define ST_STS_CMP 9
#define ST_STS_LAST 10
#define ST_STS_LASTV 11
#define ST_DMAWR 12
#define ST_C2H_CMD_STALL 13
#define ST_C2H_STALL 14
#define ST_T_C2H_CMD0 15
#define ST_T_C2H_BEAT_L 16
#define ST_T_C2H_STS_L 17
#define ST_T_DMAWR_L 18
#define ST_H2C_CMD 19
#define ST_H2C_BEAT 20
#define ST_H2C_TLAST 21
#define ST_H2C_BYTES 22
#define ST_H2C_ERR 23
#define ST_H2C_ZB 24
#define ST_QSTS 25
#define ST_QSTS_OP1 26
#define ST_QSTS_ERR 27
#define ST_QSTS_LASTV 28
#define ST_H2C_CMD_STALL 29
#define ST_T_H2C_CMD0 30
#define ST_T_H2C_BEAT_L 31
#define ST_T_QSTS_L 32
#define ST_TM 33
#define ST_TM_ERR 34
#define ST_TM_LASTV 35
#define ST_C2H_CMD_LASTV 36
#define ST_C2H_LENERR 37
#define ST_CH(ch, k) (64 + 16 * (ch) + (k))
#define CH_C2H_REQ 0
#define CH_C2H_PKT 1
#define CH_WR_DONE 2
#define CH_C2H_STATE 3
#define CH_H2C_REQ 4
#define CH_H2C_PKT 5
#define CH_H2C_BEAT 6
#define CH_H2C_BAD 7
#define CH_H2C_LASTERR 8
#define CH_RD_DONE 9
#define CH_H2C_FIRST0 10
#define CH_H2C_FIRST1 11
#define CH_H2C_TLAST 12
#define CH_H2C_STATE 13

#define NCH      3
#define SLOTS    256
#define SLOT_SZ  4096
#define BUF_SZ   (SLOTS * SLOT_SZ)
#define CLK_MHZ  250

struct tdev {
	struct pci_dev *pdev;
	void __iomem *dma;          /* BAR2 */
	void __iomem *win;          /* BAR4 + WIN_OFF */
	void __iomem *sink;         /* BAR4 + 0 (rev1 bridge sink counters) */
	void *c2h_buf[NCH], *h2c_buf[NCH];
	dma_addr_t c2h_dma[NCH], h2c_dma[NCH];
	int tag[QDMA_N_QUEUES];
	int n_c2h_q, n_h2c_q;
	bool ctx_stuck;             /* a context command never finished */
	bool dma_stuck;             /* a transfer never completed: keep buffers, leave contexts alone */
	int fails;
};

static struct tdev *gt;

/* ================================================================== pci_qdma.c (copied; waits bounded) */
static int wait_until_busy_cleared(struct tdev *t)
{
	int i, busy;

	for (i = 0; i < 10000; i++) {        /* 10000 x 10 us = 100 ms; pci_qdma.c waits forever */
		usleep_range(DMA_MIN_SLEEP_CMD, DMA_MIN_SLEEP_CMD);
		busy = ioread32(t->dma + QDMA_CTX_CMD_REG) & 0x1;
		if (!busy)
			return 0;
	}
	t->ctx_stuck = true;
	pr_err(DRV ": context command 0x%08x busy for 100 ms\n", ioread32(t->dma + QDMA_CTX_CMD_REG));
	return -ETIMEDOUT;
}

static int ctx_cmd(struct tdev *t, int32_t qid, int32_t sel, int32_t op)
{
	int32_t reg_val = QDMA_CTX_BUSY_VAL_DEAULT |
			  ((sel & QDMA_CTX_SEL_MASK) << QDMA_CTX_SEL_SHIFT) |
			  ((op & QDMA_CTX_OP_MASK) << QDMA_CTX_OP_SHIFT) |
			  ((qid & QDMA_CTX_QID_MASK) << QDMA_CTX_QID_SHIFT);
	iowrite32(reg_val, t->dma + QDMA_CTX_CMD_REG);
	wmb();
	return wait_until_busy_cleared(t);
}

static void win_wr(struct tdev *t, int idx, u64 v) { writeq(v, t->win + 8 * idx); }
static u64 st(struct tdev *t, int idx) { return readq(t->win + 8 * idx); }

static int enable_queue(struct tdev *t, int32_t qid, bool c2h)
{
	int32_t reg_val, i;

	if (ctx_cmd(t, qid, QDMA_CTXT_SELC_DEC_SW_C2H, QDMA_CTX_CLR) || ctx_cmd(t, qid, QDMA_CTXT_SELC_DEC_SW_H2C, QDMA_CTX_CLR) ||
	    ctx_cmd(t, qid, QDMA_CTXT_SELC_DEC_HW_C2H, QDMA_CTX_CLR) || ctx_cmd(t, qid, QDMA_CTXT_SELC_DEC_HW_H2C, QDMA_CTX_CLR) ||
	    ctx_cmd(t, qid, QDMA_CTXT_SELC_DEC_CR_C2H, QDMA_CTX_CLR) || ctx_cmd(t, qid, QDMA_CTXT_SELC_DEC_CR_H2C, QDMA_CTX_CLR))
		return -1;

	for (i = 0; i < QDMA_CTX_N_DATA_REGS; i++) {
		iowrite32(QDMA_CXT_MASK_DEF_VAL, t->dma + QDMA_CTX_MASK_REG_START + i * 4);
		wmb();
	}
	usleep_range(DMA_MIN_SLEEP_CMD, DMA_MIN_SLEEP_CMD);

	/* SW context: fnc 0, irq_arm 0; qen (32) + bypass (50), streaming */
	iowrite32(0, t->dma + QDMA_CTX_DATA_REG_START);
	wmb();
	iowrite32(0x00040001, t->dma + QDMA_CTX_DATA_REG_START + 4);
	wmb();
	for (i = 2; i < QDMA_CTX_N_DATA_REGS; i++) {
		iowrite32(0, t->dma + QDMA_CTX_DATA_REG_START + i * 4);
		wmb();
	}
	if (ctx_cmd(t, qid, c2h ? QDMA_CTXT_SELC_DEC_SW_C2H : QDMA_CTXT_SELC_DEC_SW_H2C, QDMA_CTX_WR))
		return -1;

	/* HW context idl_stp_b (bit 41) */
	if (ctx_cmd(t, qid, c2h ? QDMA_CTXT_SELC_DEC_HW_C2H : QDMA_CTXT_SELC_DEC_HW_H2C, QDMA_CTX_RD))
		return -1;
	reg_val = ioread32(t->dma + QDMA_CTX_DATA_REG_START + 4);
	if (!((reg_val >> 9) & 0x1)) {
		pr_err(DRV ": failed to enable software context for qid %d\n", qid);
		return -1;
	}

	if (ctx_cmd(t, qid, QDMA_CTXT_SELC_PFTCH, QDMA_CTX_CLR) || ctx_cmd(t, qid, QDMA_CTXT_SELC_WRB, QDMA_CTX_CLR))
		return -1;

	if (c2h) {
		int32_t pfch_tag_reg, pfch_qid, pfch_tag;

		/* prefetch context: bypass (0), pfch_en 0, valid (45) */
		iowrite32(0x1, t->dma + QDMA_CTX_DATA_REG_START);
		wmb();
		iowrite32(0x2000, t->dma + QDMA_CTX_DATA_REG_START + 4);
		wmb();
		if (ctx_cmd(t, qid, QDMA_CTXT_SELC_PFTCH, QDMA_CTX_WR))
			return -1;
		reg_val = QDMA_CTX_BUSY_VAL_DEAULT |
			  ((QDMA_CTXT_SELC_PFTCH & QDMA_CTX_SEL_MASK) << QDMA_CTX_SEL_SHIFT) |
			  ((QDMA_CTX_RD & QDMA_CTX_OP_MASK) << QDMA_CTX_OP_SHIFT) |
			  ((qid & QDMA_CTX_QID_MASK) << QDMA_CTX_QID_SHIFT);
		iowrite32(reg_val, t->dma + QDMA_CTX_CMD_REG);
		wmb();
		usleep_range(DMA_MIN_SLEEP_CMD, DMA_MIN_SLEEP_CMD);
		reg_val = ioread32(t->dma + QDMA_CTX_DATA_REG_START + 4);
		if (!((reg_val >> 13) & 0x1)) {
			pr_err(DRV ": C2H prefetch context could not be set valid, qid %d\n", qid);
			return -1;
		}

		/* prefetch tag: qid -> 0x1408, tag <- 0x140C; must be granted for this qid (7cb6023d) */
		iowrite32(qid, t->dma + QDMA_C2H_PFCH_BYP_QID_REG);
		wmb();
		usleep_range(DMA_MIN_SLEEP_CMD, DMA_MIN_SLEEP_CMD);
		pfch_tag_reg = ioread32(t->dma + QDMA_C2H_PFCH_BYP_TAG_REG);
		if (((pfch_tag_reg >> 8) & 0xFFF) != qid) {
			pr_err(DRV ": C2H prefetch tag for qid %d not granted (0x%08x)\n", qid, pfch_tag_reg);
			return -1;
		}
		pfch_qid = ((pfch_tag_reg >> 8) & 0xFFF) - QDMA_WR_QUEUE_START_IDX;
		pfch_tag = pfch_tag_reg & 0x7F;
		t->tag[qid] = pfch_tag;
		/* to the wrapper's tag table, as the driver writes stat_cnfg->qdma_pfch_tag (static_slave reg 17) */
		win_wr(t, CTL_PFCH, (1ULL << 31) | ((u64)pfch_qid << 8) | pfch_tag);
		wmb();
		usleep_range(DMA_MIN_SLEEP_CMD, DMA_MIN_SLEEP_CMD);
		win_wr(t, CTL_PFCH, 0);
	}

	/* completion context: valid (124), dir_c2h (146) for C2H */
	iowrite32(0, t->dma + QDMA_CTX_DATA_REG_START);
	iowrite32(0, t->dma + QDMA_CTX_DATA_REG_START + 4);
	iowrite32(0, t->dma + QDMA_CTX_DATA_REG_START + 8);
	iowrite32(0x10000000, t->dma + QDMA_CTX_DATA_REG_START + 12);
	iowrite32(c2h ? 0x40000 : 0, t->dma + QDMA_CTX_DATA_REG_START + 16);
	for (i = 5; i < QDMA_CTX_N_DATA_REGS; i++)
		iowrite32(0, t->dma + QDMA_CTX_DATA_REG_START + i * 4);
	wmb();
	if (ctx_cmd(t, qid, QDMA_CTXT_SELC_WRB, QDMA_CTX_WR))
		return -1;
	return 0;
}

static int enable_queues(struct tdev *t)
{
	int32_t qid, i;

	for (i = 0; i < QDMA_CTX_N_DATA_REGS; i++)
		iowrite32(QDMA_CXT_MASK_DEF_VAL, t->dma + QDMA_CTX_MASK_REG_START + i * 4);
	wmb();
	usleep_range(DMA_MIN_SLEEP_CMD, DMA_MIN_SLEEP_CMD);
	for (i = 0; i < QDMA_CTX_N_DATA_REGS; i++)
		iowrite32(i == 1 ? QDMA_N_QUEUES - 1 : 0, t->dma + QDMA_CTX_DATA_REG_START + i * 4);
	wmb();
	if (ctx_cmd(t, 0, QDMA_CTXT_SELC_FMAP, QDMA_CTX_WR))
		return -1;

	for (qid = QDMA_RD_QUEUE_START_IDX; qid < QDMA_RD_QUEUE_START_IDX + QDMA_N_ACTIVE_QUEUES; qid++) {
		if (enable_queue(t, qid, false))
			return -1;
		t->n_h2c_q++;
	}
	iowrite32(PAGE_SIZE, t->dma + 0xab0);
	wmb();
	usleep_range(DMA_MIN_SLEEP_CMD, DMA_MIN_SLEEP_CMD);
	for (qid = QDMA_WR_QUEUE_START_IDX; qid < QDMA_WR_QUEUE_START_IDX + QDMA_N_C2H_QUEUES; qid++) {
		if (enable_queue(t, qid, true))
			return -1;
		t->n_c2h_q++;
	}
	return 0;
}

static void disable_queues(struct tdev *t)
{
	int i;

	for (i = 0; i < t->n_c2h_q && !t->ctx_stuck; i++) {
		int qid = QDMA_WR_QUEUE_START_IDX + i;
		ctx_cmd(t, qid, QDMA_CTXT_SELC_PFTCH, QDMA_CTX_INV);
		ctx_cmd(t, qid, QDMA_CTXT_SELC_WRB, QDMA_CTX_INV);
		ctx_cmd(t, qid, QDMA_CTXT_SELC_DEC_SW_C2H, QDMA_CTX_INV);
		ctx_cmd(t, qid, QDMA_CTXT_SELC_TIMER, QDMA_CTX_INV);
	}
	for (i = 0; i < t->n_h2c_q && !t->ctx_stuck; i++) {
		int qid = QDMA_RD_QUEUE_START_IDX + i;
		ctx_cmd(t, qid, QDMA_CTXT_SELC_WRB, QDMA_CTX_INV);
		ctx_cmd(t, qid, QDMA_CTXT_SELC_DEC_SW_H2C, QDMA_CTX_INV);
	}
}

/* ================================================================== diagnostics */
static const u32 err_regs[] = { 0x248, 0x254, 0x264, 0x314, 0xF4, 0xFC, 0xAF0, 0xAF8, 0xE00 };
static const char *const err_names[] = { "GLBL_ERR_STAT", "GLBL_DSC_ERR_STS", "GLBL_TRQ_ERR_STS", "FAB_THR_LOG",
					 "RAM_SBE_STS_A", "RAM_DBE_STS_A", "C2H_ERR_STAT", "C2H_FATAL_ERR_STAT", "H2C_ERR_STAT" };

static bool dump_err_regs(struct tdev *t, const char *when)
{
	bool any = false;
	int i;

	for (i = 0; i < ARRAY_SIZE(err_regs); i++) {
		u32 v = ioread32(t->dma + err_regs[i]);
		if (v && err_regs[i] != 0x314) {
			pr_info(DRV ": [%s] %s (0x%x) = 0x%08x\n", when, err_names[i], err_regs[i], v);
			any = true;
		}
	}
	return any;
}

static void dump_c2h_stat_regs(struct tdev *t)
{
	u32 o;

	for (o = 0xA80; o <= 0xB4C; o += 16)
		pr_info(DRV ": QDMA 0x%03x: %08x %08x %08x %08x\n", o, ioread32(t->dma + o), ioread32(t->dma + o + 4),
			ioread32(t->dma + o + 8), ioread32(t->dma + o + 12));
	for (o = 0xE00; o <= 0xE2C; o += 16)
		pr_info(DRV ": QDMA 0x%03x: %08x %08x %08x %08x\n", o, ioread32(t->dma + o), ioread32(t->dma + o + 4),
			ioread32(t->dma + o + 8), ioread32(t->dma + o + 12));
}

static void dump_stat(struct tdev *t)
{
	int i, ch;

	for (i = 0; i < 40; i += 4)
		pr_info(DRV ": stat[%2d..%2d] %llx %llx %llx %llx\n", i, i + 3, st(t, i), st(t, i + 1), st(t, i + 2), st(t, i + 3));
	for (ch = 0; ch < NCH; ch++)
		for (i = 0; i < 14; i += 7)
			pr_info(DRV ": ch%d [%2d..%2d] %llx %llx %llx %llx %llx %llx %llx\n", ch, i, i + 6,
				st(t, ST_CH(ch, i)), st(t, ST_CH(ch, i + 1)), st(t, ST_CH(ch, i + 2)), st(t, ST_CH(ch, i + 3)),
				st(t, ST_CH(ch, i + 4)), st(t, ST_CH(ch, i + 5)), st(t, ST_CH(ch, i + 6)));
	dump_err_regs(t, "stat dump");
	dump_c2h_stat_regs(t);
}

/* ================================================================== data pattern (= qdma_gen.sv) */
static inline u32 pword(u32 seed, int ch, u32 i, u32 k)
{
	return seed + ((u32)ch << 24) + (i << 12) + k;
}

static void fill_pattern(void *buf, int ch, u32 i, u32 len, u32 seed)
{
	u8 *p = buf;
	u32 j;

	for (j = 0; j < len; j += 4) {
		u32 w = pword(seed, ch, i, j / 4);
		u32 n = min_t(u32, 4, len - j);
		memcpy(p + j, &w, n);
	}
}

/* bytes [0, len) must hold packet i's pattern, bytes [len, SLOT_SZ) the 0xA5 sentinel */
static long check_slot(const void *buf, int ch, u32 i, u32 len, u32 seed, long *first)
{
	const u8 *p = buf;
	long bad = 0;
	u32 j;

	for (j = 0; j < SLOT_SZ; j++) {
		u8 exp;
		if (j < len) {
			u32 w = pword(seed, ch, i, j / 4);
			exp = (w >> (8 * (j % 4))) & 0xFF;
		} else {
			exp = 0xA5;
		}
		if (p[j] != exp) {
			if (!bad && first)
				*first = j;
			bad++;
		}
	}
	return bad;
}

/* ================================================================== runs */
struct runres {
	u64 ns, bytes, cyc;
	bool timeout;
};

static void cfg_ch(struct tdev *t, bool h2c, int ch, u32 len, u32 cnt, u32 seed)
{
	int b = h2c ? CTL_H2C(ch) : CTL_C2H(ch);

	win_wr(t, b + 0, h2c ? t->h2c_dma[ch] : t->c2h_dma[ch]);
	win_wr(t, b + 1, len);
	win_wr(t, b + 2, cnt);
	win_wr(t, b + 3, SLOT_SZ);
	win_wr(t, b + 4, min_t(u32, cnt, SLOTS));
	win_wr(t, b + 5, seed);
}

/* go: bit0 C2H, bit1 H2C. Waits until every masked channel's wrapper reports cnt completions. */
static void run(struct tdev *t, u32 go, u32 c2h_mask, u32 h2c_mask, u32 len, u32 cnt, u32 c2h_seed, u32 h2c_seed,
		unsigned int timeout_ms, struct runres *r)
{
	ktime_t t0, deadline;
	int ch;

	memset(r, 0, sizeof(*r));
	win_wr(t, CTL_CMD, 0x4);                       /* clear counters */
	win_wr(t, CTL_C2HMASK, c2h_mask);
	win_wr(t, CTL_H2CMASK, h2c_mask);
	for (ch = 0; ch < NCH; ch++) {
		if (c2h_mask & (1 << ch))
			cfg_ch(t, false, ch, len, cnt, c2h_seed + ch);
		if (h2c_mask & (1 << ch))
			cfg_ch(t, true, ch, len, cnt, h2c_seed + ch);
	}
	(void)st(t, ST_MAGIC);                         /* flush the posted writes */
	t0 = ktime_get();
	deadline = ktime_add_ms(t0, timeout_ms);
	win_wr(t, CTL_CMD, go);
	for (;;) {
		bool done = true;

		for (ch = 0; ch < NCH; ch++) {
			if ((c2h_mask & (1 << ch)) && st(t, ST_CH(ch, CH_WR_DONE)) < cnt)
				done = false;
			if ((h2c_mask & (1 << ch)) && st(t, ST_CH(ch, CH_RD_DONE)) < cnt)
				done = false;
		}
		if (done)
			break;
		if (ktime_after(ktime_get(), deadline)) {
			r->timeout = true;
			break;
		}
		usleep_range(20, 50);
	}
	r->ns = ktime_to_ns(ktime_sub(ktime_get(), t0));
	if (r->timeout) {
		t->dma_stuck = true;
		win_wr(t, CTL_CMD, 0x8);                   /* abort: no further requests */
		pr_err(DRV ": TIMEOUT after %u ms (go %u, c2h mask %u, h2c mask %u, len %u, cnt %u)\n",
		       timeout_ms, go, c2h_mask, h2c_mask, len, cnt);
		dump_stat(t);
	}
}

static void result(struct tdev *t, const char *name, bool pass, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	pr_info(DRV ": RESULT %-28s %s %pV\n", name, pass ? "PASS" : "FAIL", &vaf);
	va_end(args);
	if (!pass)
		t->fails++;
}

/* C2H run with counters + host data check; per channel the last packet written to each slot is checked */
static bool c2h_run(struct tdev *t, const char *name, u32 mask, u32 len, u32 cnt, u32 seed, struct runres *r)
{
	u64 total, drop, err, sts, lenerr, cmd;
	long bad = 0, first = -1;
	int ch, nch = hweight32(mask);
	u32 s, wrap = min_t(u32, cnt, SLOTS);
	bool pass;

	for (ch = 0; ch < NCH; ch++)
		if (mask & (1 << ch))
			memset(t->c2h_buf[ch], 0xA5, BUF_SZ);
	wmb();
	run(t, 0x1, mask, 0, len, cnt, seed, 0, 10000, r);
	total = (u64)nch * cnt;
	cmd = st(t, ST_C2H_CMD); sts = st(t, ST_STS); drop = st(t, ST_STS_DROP); err = st(t, ST_STS_ERR);
	lenerr = st(t, ST_C2H_LENERR);
	r->bytes = st(t, ST_C2H_BYTES);
	r->cyc = st(t, ST_T_C2H_STS_L) - st(t, ST_T_C2H_CMD0);
	rmb();
	if (!r->timeout) {
		for (ch = 0; ch < NCH; ch++) {
			if (!(mask & (1 << ch)))
				continue;
			for (s = 0; s < wrap; s++) {
				u32 i = s + wrap * ((cnt - 1 - s) / wrap);   /* last packet written to slot s */
				long f = -1, b = check_slot((u8 *)t->c2h_buf[ch] + (size_t)s * SLOT_SZ, ch, i, len, seed + ch, &f);
				if (b && first < 0) {
					first = f;
					pr_info(DRV ": %s ch%d slot %u (packet %u): %ld bytes wrong, first at %ld\n", name, ch, s, i, b, f);
				}
				bad += b;
			}
		}
	}
	pass = !r->timeout && cmd == total && sts == total && drop == 0 && err == 0 && lenerr == 0 && bad == 0 &&
	       r->bytes == total * len;
	result(t, name, pass, "desc %llu/%llu status %llu drop %llu err %llu lenerr %llu bytes %llu wrong-bytes %ld dmawr %llu",
	       cmd, total, sts, drop, err, lenerr, r->bytes, bad, st(t, ST_DMAWR));
	if (!pass && !r->timeout)
		dump_stat(t);
	return pass;
}

static bool h2c_run(struct tdev *t, const char *name, u32 mask, u32 len, u32 cnt, u32 seed, bool check, struct runres *r)
{
	u64 total, cmd, qerr, herr;
	u64 pkts = 0, bad = 0, lasterr = 0;
	int ch, nch = hweight32(mask);
	u32 s, wrap = min_t(u32, cnt, SLOTS);
	bool pass;

	for (ch = 0; ch < NCH; ch++) {
		if (!(mask & (1 << ch)))
			continue;
		memset(t->h2c_buf[ch], 0xA5, BUF_SZ);
		for (s = 0; s < wrap; s++)
			fill_pattern((u8 *)t->h2c_buf[ch] + (size_t)s * SLOT_SZ, ch, s, len, seed + ch);
	}
	wmb();
	run(t, 0x2, 0, mask, len, cnt, 0, seed, 10000, r);
	total = (u64)nch * cnt;
	cmd = st(t, ST_H2C_CMD); qerr = st(t, ST_QSTS_ERR); herr = st(t, ST_H2C_ERR);
	r->bytes = st(t, ST_H2C_BYTES);
	r->cyc = st(t, ST_T_H2C_BEAT_L) - st(t, ST_T_H2C_CMD0);
	for (ch = 0; ch < NCH; ch++) {
		if (!(mask & (1 << ch)))
			continue;
		pkts += st(t, ST_CH(ch, CH_H2C_PKT));
		bad += st(t, ST_CH(ch, CH_H2C_BAD));
		lasterr += st(t, ST_CH(ch, CH_H2C_LASTERR));
		if (check && st(t, ST_CH(ch, CH_H2C_FIRST0)) >> 63)
			pr_info(DRV ": %s ch%d first mismatch %llx (exp|got word0 %llx)\n", name, ch,
				st(t, ST_CH(ch, CH_H2C_FIRST0)), st(t, ST_CH(ch, CH_H2C_FIRST1)));
	}
	pass = !r->timeout && cmd == total && pkts == total && qerr == 0 && herr == 0 && r->bytes == total * len &&
	       (!check || (bad == 0 && lasterr == 0));
	result(t, name, pass, "desc %llu/%llu pkts %llu bad-beats %llu%s keep/last-err %llu qsts-err %llu tuser-err %llu bytes %llu",
	       cmd, total, pkts, bad, check ? "" : " (data not checked: wrapped)", lasterr, qerr, herr, r->bytes);
	if (!pass && !r->timeout)
		dump_stat(t);
	return pass;
}

static void rate(struct tdev *t, const char *name, struct runres *r)
{
	/* FPGA counters: bytes over the cycles between the first descriptor and the last status/beat (250 MHz) */
	u64 fpga_mbs = r->cyc ? div64_u64(r->bytes * CLK_MHZ, r->cyc) : 0;
	u64 host_mbs = r->ns ? div64_u64(r->bytes * 1000, r->ns) : 0;

	pr_info(DRV ": RATE %-28s %llu bytes, FPGA %llu cycles = %llu.%02llu GB/s, host %llu ns = %llu.%02llu GB/s (measured)\n",
		name, r->bytes, r->cyc, fpga_mbs / 1000, (fpga_mbs % 1000) / 10, r->ns, host_mbs / 1000, (host_mbs % 1000) / 10);
}

static void run_all(struct tdev *t)
{
	struct runres r;
	char nm[48];
	int ch, k, n, granted = 0, dup = 0;
	int qid, q2;

	/* --- setup gate: all 32 tags granted, distinct */
	for (qid = QDMA_WR_QUEUE_START_IDX; qid < QDMA_WR_QUEUE_START_IDX + t->n_c2h_q; qid++) {
		if (t->tag[qid] >= 0)
			granted++;
		for (q2 = QDMA_WR_QUEUE_START_IDX; q2 < qid; q2++)
			if (t->tag[q2] == t->tag[qid])
				dup++;
	}
	pr_info(DRV ": tags 256..263: %d %d %d %d %d %d %d %d | 272..279: %d %d %d %d %d %d %d %d | 287: %d\n",
		t->tag[256], t->tag[257], t->tag[258], t->tag[259], t->tag[260], t->tag[261], t->tag[262], t->tag[263],
		t->tag[272], t->tag[273], t->tag[274], t->tag[275], t->tag[276], t->tag[277], t->tag[278], t->tag[279], t->tag[287]);
	result(t, "setup_queues_tags", granted == QDMA_N_C2H_QUEUES && dup == 0 && t->n_h2c_q == QDMA_N_ACTIVE_QUEUES,
	       "h2c queues %d c2h queues %d tags granted %d duplicates %d", t->n_h2c_q, t->n_c2h_q, granted, dup);
	if (granted != QDMA_N_C2H_QUEUES)
		return;
	dump_err_regs(t, "after setup");

	/* --- C2H singles: 8 consecutive packets per channel = each of its 8 queues once; qid + tag checked */
	for (n = 0; n < 2 && !t->dma_stuck; n++) {
		u32 len = n ? 4096 : 64;
		int ok = 0, tagbad = 0;
		u32 seen = 0;

		for (ch = 0; ch < NCH && !t->dma_stuck; ch++) {
			for (k = 0; k < 8 && !t->dma_stuck; k++) {
				u64 lv;
				int q, tg;

				snprintf(nm, sizeof(nm), "c2h_single_%u_ch%d_%d", len, ch, k);
				if (c2h_run(t, nm, 1 << ch, len, 1, 0x11000000 + (n << 20) + (k << 8), &r))
					ok++;
				lv = st(t, ST_C2H_CMD_LASTV);
				q = lv & 0x7FF;
				tg = (lv >> 16) & 0x7F;
				if (q >= 256 && q < 280) {
					seen |= 1u << (q - 256);
					if (tg != t->tag[q]) {
						tagbad++;
						pr_info(DRV ": qid %d sent tag %d, QDMA granted %d\n", q, tg, t->tag[q]);
					}
				}
			}
		}
		snprintf(nm, sizeof(nm), "c2h_singles_%u_all24", len);
		result(t, nm, ok == 24 && seen == 0xFFFFFF && tagbad == 0, "passed %d/24, queues seen 0x%06x, tag mismatches %d",
		       ok, seen, tagbad);
	}
	if (t->dma_stuck)
		return;

	/* --- C2H back-to-back, 3 channels at once */
	c2h_run(t, "c2h_b2b_64_3x4096", 0x7, 64, 4096, 0x22000000, &r);
	if (!t->dma_stuck) c2h_run(t, "c2h_b2b_4096_3x4096", 0x7, 4096, 4096, 0x23000000, &r);
	if (!t->dma_stuck) c2h_run(t, "c2h_b2b_100_3x1000", 0x7, 100, 1000, 0x24000000, &r);
	if (t->dma_stuck)
		return;

	/* --- C2H throughput (4 KiB = the C2H buffer size) */
	if (c2h_run(t, "c2h_tput_4096_3ch", 0x7, 4096, tput_cnt, 0x41000000, &r)) rate(t, "c2h_tput_4096_3ch", &r);
	if (!t->dma_stuck && c2h_run(t, "c2h_tput_4096_1ch", 0x1, 4096, tput_cnt, 0x42000000, &r)) rate(t, "c2h_tput_4096_1ch", &r);
	if (!t->dma_stuck && c2h_run(t, "c2h_tput_256_3ch", 0x7, 256, tput_cnt, 0x43000000, &r)) rate(t, "c2h_tput_256_3ch", &r);
	if (!t->dma_stuck && c2h_run(t, "c2h_tput_64_3ch", 0x7, 64, tput_cnt, 0x45000000, &r)) rate(t, "c2h_tput_64_3ch", &r);
	if (!t->dma_stuck && c2h_run(t, "c2h_tput_64_1ch", 0x1, 64, tput_cnt, 0x46000000, &r)) rate(t, "c2h_tput_64_1ch", &r);
	if (t->dma_stuck)
		return;

	/* --- H2C singles and back-to-back (FPGA checks the data; <= 256 packets so no slot is reused) */
	for (ch = 0; ch < NCH && !t->dma_stuck; ch++) {
		snprintf(nm, sizeof(nm), "h2c_single_64_ch%d", ch);
		h2c_run(t, nm, 1 << ch, 64, 1, 0x31000000, true, &r);
		snprintf(nm, sizeof(nm), "h2c_single_4096_ch%d", ch);
		if (!t->dma_stuck) h2c_run(t, nm, 1 << ch, 4096, 1, 0x32000000, true, &r);
	}
	if (!t->dma_stuck) h2c_run(t, "h2c_b2b_64_3x256", 0x7, 64, 256, 0x33000000, true, &r);
	if (!t->dma_stuck) h2c_run(t, "h2c_b2b_4096_3x256", 0x7, 4096, 256, 0x34000000, true, &r);
	if (!t->dma_stuck) h2c_run(t, "h2c_b2b_100_3x256", 0x7, 100, 256, 0x35000000, true, &r);
	if (t->dma_stuck)
		return;

	/* --- H2C throughput (data not checked: slots reused) */
	if (h2c_run(t, "h2c_tput_4096_3ch", 0x7, 4096, tput_cnt, 0x44000000, false, &r)) rate(t, "h2c_tput_4096_3ch", &r);
	if (t->dma_stuck)
		return;

	/* --- concurrent C2H + H2C (the static runs both at once) */
	{
		u64 c2h_by, h2c_by, c2h_cyc, h2c_cyc, drop, err, qerr;
		bool pass;

		for (ch = 0; ch < NCH; ch++)
			memset(t->c2h_buf[ch], 0xA5, BUF_SZ);
		run(t, 0x3, 0x7, 0x7, 4096, tput_cnt, 0x51000000, 0x52000000, 20000, &r);
		c2h_by = st(t, ST_C2H_BYTES); h2c_by = st(t, ST_H2C_BYTES);
		c2h_cyc = st(t, ST_T_C2H_STS_L) - st(t, ST_T_C2H_CMD0);
		h2c_cyc = st(t, ST_T_H2C_BEAT_L) - st(t, ST_T_H2C_CMD0);
		drop = st(t, ST_STS_DROP); err = st(t, ST_STS_ERR); qerr = st(t, ST_QSTS_ERR);
		pass = !r.timeout && c2h_by == 3ULL * tput_cnt * 4096 && h2c_by == 3ULL * tput_cnt * 4096 && !drop && !err && !qerr;
		result(t, "c2h_h2c_concurrent_4096_3ch", pass, "c2h bytes %llu h2c bytes %llu drop %llu err %llu qsts-err %llu",
		       c2h_by, h2c_by, drop, err, qerr);
		if (pass) {
			r.bytes = c2h_by; r.cyc = c2h_cyc; rate(t, "concurrent_c2h_part", &r);
			r.bytes = h2c_by; r.cyc = h2c_cyc; rate(t, "concurrent_h2c_part", &r);
		}
	}
}

/* ================================================================== probe/remove */
static int qdma_test_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct tdev *t;
	int ch, ret;
	u64 magic;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;
	t->pdev = pdev;
	memset(t->tag, 0xff, sizeof(t->tag));
	ret = pci_enable_device(pdev);
	if (ret)
		goto err_free;
	ret = pci_request_regions(pdev, DRV);
	if (ret)
		goto err_disable;
	pci_set_master(pdev);
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret)
		goto err_regions;
	t->dma = pci_iomap(pdev, BAR_DMA, 0);
	t->win = pci_iomap_range(pdev, BAR_BRIDGE, WIN_OFF, WIN_LEN);
	t->sink = pci_iomap_range(pdev, BAR_BRIDGE, 0, WIN_LEN);
	if (!t->dma || !t->win || !t->sink) {
		ret = -ENOMEM;
		goto err_unmap;
	}
	for (ch = 0; ch < NCH; ch++) {
		t->c2h_buf[ch] = dma_alloc_coherent(&pdev->dev, BUF_SZ, &t->c2h_dma[ch], GFP_KERNEL);
		t->h2c_buf[ch] = dma_alloc_coherent(&pdev->dev, BUF_SZ, &t->h2c_dma[ch], GFP_KERNEL);
		if (!t->c2h_buf[ch] || !t->h2c_buf[ch]) {
			ret = -ENOMEM;
			goto err_bufs;
		}
	}
	pci_set_drvdata(pdev, t);
	gt = t;

	magic = st(t, ST_MAGIC);
	pr_info(DRV ": %s BAR2 DMA config, BAR4 window magic %llx (expect 51da0c0ffee00002), sink magic %llx, "
		"QDMA id 0x%08x, cycle %llu\n", pci_name(pdev), magic, readq(t->sink), ioread32(t->dma + 0x0), st(t, 1));
	if (magic != 0x51DA0C0FFEE00002ULL) {
		pr_err(DRV ": not the rev2 test image, not touching the QDMA\n");
		return 0;
	}
	dump_err_regs(t, "before setup");
	if (enable_queues(t)) {
		pr_err(DRV ": queue setup failed (h2c %d c2h %d)\n", t->n_h2c_q, t->n_c2h_q);
		result(t, "setup_queues_tags", false, "h2c queues %d c2h queues %d", t->n_h2c_q, t->n_c2h_q);
		dump_err_regs(t, "setup failed");
		return 0;
	}
	if (run_tests)
		run_all(t);
	dump_err_regs(t, "end");
	pr_info(DRV ": DONE fails %d%s\n", t->fails, t->dma_stuck ? " (a transfer is STUCK: buffers kept, contexts left alone)" : "");
	return 0;

err_bufs:
	for (ch = 0; ch < NCH; ch++) {
		if (t->c2h_buf[ch])
			dma_free_coherent(&pdev->dev, BUF_SZ, t->c2h_buf[ch], t->c2h_dma[ch]);
		if (t->h2c_buf[ch])
			dma_free_coherent(&pdev->dev, BUF_SZ, t->h2c_buf[ch], t->h2c_dma[ch]);
	}
err_unmap:
	if (t->dma) pci_iounmap(pdev, t->dma);
	if (t->win) pci_iounmap(pdev, t->win);
	if (t->sink) pci_iounmap(pdev, t->sink);
err_regions:
	pci_release_regions(pdev);
err_disable:
	pci_disable_device(pdev);
err_free:
	kfree(t);
	return ret;
}

static void qdma_test_remove(struct pci_dev *pdev)
{
	struct tdev *t = pci_get_drvdata(pdev);
	int ch;

	if (!t)
		return;
	if (!t->dma_stuck && !t->ctx_stuck) {
		disable_queues(t);
		for (ch = 0; ch < NCH; ch++) {
			dma_free_coherent(&pdev->dev, BUF_SZ, t->c2h_buf[ch], t->c2h_dma[ch]);
			dma_free_coherent(&pdev->dev, BUF_SZ, t->h2c_buf[ch], t->h2c_dma[ch]);
		}
	} else {
		pr_warn(DRV ": stuck transfer/context: DMA buffers deliberately leaked, contexts not invalidated\n");
	}
	pci_iounmap(pdev, t->dma);
	pci_iounmap(pdev, t->win);
	pci_iounmap(pdev, t->sink);
	pci_release_regions(pdev);
	if (!t->dma_stuck)
		pci_disable_device(pdev);
	kfree(t);
	gt = NULL;
}

static const struct pci_device_id qdma_test_ids[] = {
	{ PCI_DEVICE(0x10ee, 0x9f3f) },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, qdma_test_ids);

static struct pci_driver qdma_test_driver = {
	.name = DRV,
	.id_table = qdma_test_ids,
	.probe = qdma_test_probe,
	.remove = qdma_test_remove,
};
module_pci_driver(qdma_test_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("QDMA C2H/H2C path test on the qdma_hw rev2 test image");
