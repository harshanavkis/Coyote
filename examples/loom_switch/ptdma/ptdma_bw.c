// SPDX-License-Identifier: GPL-2.0
/*
 * ptdma_bw: copy bandwidth of AMD PTDMA engines (drivers/dma/amd/ptdma,
 * DMA_MEMCPY) between host DRAM and host DRAM, or between host DRAM and an
 * MMIO range such as a PCIe BAR.
 *
 * The test runs once, in module init; init then fails with -EAGAIN on
 * purpose so that nothing stays loaded. Results go to the kernel log.
 *
 * Per iteration, `size` bytes are cut into descriptors of at most `chunk`
 * bytes (never across a host piece of up to 4 MiB), dealt round-robin over
 * the channels, issued, and waited for. The iteration's time runs from the
 * first prep to the last completion callback.
 *
 * Source pattern: a host DRAM source is always filled so that the 64-bit
 * word at byte offset o of the copy is PTBW_PATTERN ^ o
 * (0x5054444d00000000 ^ o); a receiver of the copy checks that.
 *
 * Flag mode (flag_off != 0, needs dst_phys and nchan=1): after an
 * iteration's data descriptors, one more 8 B descriptor on the same channel
 * copies flag_val + iteration + 1 (iteration 0-based) from a host page to
 * dst_phys + flag_off, and the iteration's time runs to that descriptor's
 * completion. One channel is one requester, whose posted writes PCIe keeps
 * in order, so the flag lands behind the data (several channels would be
 * several requesters, with no order between them). Written for Loom: the
 * data and the flag go to two windows of the U280's user data window and
 * land on the far host, where the receiver waits for the flag.
 *
 * tag: printed in the first line of the run ("start <tag>: ..."), so a
 * script finds this run's lines in a full kernel log by grep.
 *
 * What this relies on in the ptdma driver (v7.2.2):
 *  - one channel per PTDMA PCI function; the dma_device is DMA_PRIVATE, so
 *    channels come from dma_request_channel() with a filter;
 *  - the driver keeps ONE descriptor in the hardware and starts the next one
 *    from the completion interrupt, and it enables that interrupt only if
 *    the last prep asked for it: every descriptor carries DMA_PREP_INTERRUPT;
 *  - completion callbacks run in hard-irq context;
 *  - hardware errors are only logged ("PTDMA error: ..."), the descriptor
 *    still completes: check dmesg, and the dst check below.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/string.h>

#define PTBW_MAX_CHAN	16
#define PTBW_PATTERN	0x5054444d00000000ULL	/* "PTDM", XORed with the byte offset */
#define PTBW_POISON	0xa5a5a5a5a5a5a5a5ULL	/* dst before every iteration */

static unsigned long size = SZ_64M;
module_param(size, ulong, 0444);
MODULE_PARM_DESC(size, "bytes copied per iteration, multiple of 64 (default 64 MiB, max 4 GiB)");

static unsigned int chunk = SZ_1M;
module_param(chunk, uint, 0444);
MODULE_PARM_DESC(chunk, "bytes per descriptor, multiple of 64 (default 1 MiB; host pieces cap it at 4 MiB)");

static unsigned int iters = 3;
module_param(iters, uint, 0444);
MODULE_PARM_DESC(iters, "iterations (default 3)");

static int numa = 1;
module_param(numa, int, 0444);
MODULE_PARM_DESC(numa, "use PTDMA devices on this NUMA node, -1 = any; host buffers go there too (default 1)");

static unsigned int nchan = 1;
module_param(nchan, uint, 0444);
MODULE_PARM_DESC(nchan, "channels (= PTDMA devices) to use, at most 16 (default 1)");

static char *devs;
module_param(devs, charp, 0444);
MODULE_PARM_DESC(devs, "comma-separated PTDMA PCI names to use, in order (overrides numa and nchan)");

static unsigned long long src_phys;
module_param(src_phys, ullong, 0444);
MODULE_PARM_DESC(src_phys, "source: 0 = host DRAM buffer (default), else MMIO physical address (e.g. a BAR)");

static unsigned long long dst_phys;
module_param(dst_phys, ullong, 0444);
MODULE_PARM_DESC(dst_phys, "destination: 0 = host DRAM buffer (default), else MMIO physical address");

static bool allow_write;
module_param(allow_write, bool, 0444);
MODULE_PARM_DESC(allow_write, "allow dst_phys != 0 (writing MMIO can hang a card; default 0)");

static unsigned int timeout_ms = 10000;
module_param(timeout_ms, uint, 0444);
MODULE_PARM_DESC(timeout_ms, "per-iteration completion timeout (default 10000)");

static unsigned long long flag_off;
module_param(flag_off, ullong, 0444);
MODULE_PARM_DESC(flag_off, "flag mode: after each iteration's data, write the 8 B flag at dst_phys + flag_off on the same channel (needs dst_phys, nchan=1, flag_off >= size, 8 B aligned; default 0 = no flag)");

static unsigned long long flag_val;
module_param(flag_val, ullong, 0444);
MODULE_PARM_DESC(flag_val, "flag value base: iteration i (0-based) writes flag_val + i + 1 (default 0)");

static char *tag;
module_param(tag, charp, 0444);
MODULE_PARM_DESC(tag, "a word printed in the run's first log line (\"start <tag>:\"), to find the run in dmesg");

/* One end of the copy: host DRAM in pieces, or an MMIO range */
struct ptbw_side {
	const char *name;
	u64 phys;			/* 0: host DRAM */
	u64 len;			/* MMIO: bytes mapped (size, and the flag for dst) */
	enum dma_data_direction dir;
	unsigned int order, n;		/* host: n pieces of PAGE_SIZE << order */
	struct page **pg;
	dma_addr_t *dma;		/* host: [piece * nch + chan]; MMIO: [chan] */
};

struct ptbw_chan {
	struct dma_chan *ch;
	struct device *dev;		/* the device that does the DMA: maps go here */
	u64 bytes;			/* this iteration */
	ktime_t t_done;			/* its last completion this iteration */
};

static struct ptbw_chan chans[PTBW_MAX_CHAN];
static unsigned int nch;
static struct ptbw_side src = { .name = "src", .dir = DMA_TO_DEVICE };
static struct ptbw_side dst = { .name = "dst", .dir = DMA_FROM_DEVICE };
static atomic_t pending;
static DECLARE_COMPLETION(done);

/* Flag mode: the host page the flag descriptor copies from (chans[0]'s map) */
static struct page *flag_pg;
static dma_addr_t flag_dma = DMA_MAPPING_ERROR;
static ktime_t t_flag;			/* the flag descriptor's completion */

static inline u64 piece_size(const struct ptbw_side *s)
{
	return (u64)PAGE_SIZE << s->order;
}

/* bytes per ns, x1000 (GB/s with three decimals, GB = 1e9 B) */
static inline u64 gbps(u64 bytes, s64 ns)
{
	return ns > 0 ? div64_u64(bytes * 1000, ns) : 0;
}

/* ---- channels ---------------------------------------------------------- */

struct ptbw_pick {
	const char *name;		/* a PCI name from devs, or NULL */
	size_t len;
	bool distinct;			/* refuse devices already picked */
};

static bool ptbw_filter(struct dma_chan *ch, void *arg)
{
	const struct ptbw_pick *p = arg;
	struct device *dev = ch->device->dev;
	unsigned int i;

	if (!dev->driver || strcmp(dev->driver->name, "ptdma"))
		return false;
	if (p->name)
		return strlen(dev_name(dev)) == p->len && !strncmp(dev_name(dev), p->name, p->len);
	if (numa >= 0 && dev_to_node(dev) != numa)
		return false;
	for (i = 0; p->distinct && i < nch; i++)
		if (chans[i].ch->device == ch->device)
			return false;
	return true;
}

static int ptbw_get_chans(void)
{
	const char *s = devs && *devs ? devs : NULL;
	struct ptbw_pick p = {};
	dma_cap_mask_t mask;
	struct dma_chan *ch;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	while (nch < PTBW_MAX_CHAN) {
		if (s) {
			s += strspn(s, ", ");
			if (!*s)
				break;
			p.name = s;
			p.len = strcspn(s, ", ");
			s += p.len;
		} else if (nch == nchan) {
			break;
		}
		/* a different device per channel first, then any free channel */
		p.distinct = true;
		ch = dma_request_channel(mask, ptbw_filter, &p);
		if (!ch && !p.name) {
			p.distinct = false;
			ch = dma_request_channel(mask, ptbw_filter, &p);
		}
		if (!ch) {
			if (!p.name)
				break;
			pr_err("no free ptdma channel on %.*s\n", (int)p.len, p.name);
			return -ENODEV;
		}
		chans[nch].ch = ch;
		chans[nch].dev = dmaengine_get_dma_device(ch);
		pr_info("chan %u: %s on %s, numa node %d\n", nch, dma_chan_name(ch),
			dev_name(ch->device->dev), dev_to_node(ch->device->dev));
		nch++;
	}
	if (!nch) {
		pr_err("no free ptdma channel (numa=%d); is ptdma loaded, are they in use?\n", numa);
		return -ENODEV;
	}
	if (!s && nch < nchan)
		pr_warn("only %u of the %u channels asked for are free\n", nch, nchan);
	return 0;
}

/* ---- buffers and mappings ---------------------------------------------- */

static void side_free(struct ptbw_side *s)
{
	unsigned int i;

	for (i = 0; s->pg && i < s->n; i++)
		if (s->pg[i])
			__free_pages(s->pg[i], s->order);
	kvfree(s->pg);
	s->pg = NULL;
	s->n = 0;
}

/* `size` bytes of host DRAM on node, in the largest pieces it gives (<= 4 MiB) */
static int side_alloc(struct ptbw_side *s, int node)
{
	gfp_t gfp = GFP_KERNEL | __GFP_NOWARN | (node == NUMA_NO_NODE ? 0 : __GFP_THISNODE);
	unsigned int i;
	int order;

	for (order = MAX_PAGE_ORDER; order >= 0; order--) {
		s->order = order;
		s->n = DIV_ROUND_UP_ULL(size, piece_size(s));
		s->pg = kvcalloc(s->n, sizeof(*s->pg), GFP_KERNEL);
		if (!s->pg)
			return -ENOMEM;
		for (i = 0; i < s->n; i++) {
			s->pg[i] = alloc_pages_node(node, gfp | (order ? __GFP_NORETRY : 0), order);
			if (!s->pg[i])
				break;
		}
		if (i == s->n)
			return 0;
		side_free(s);
	}
	pr_err("%s: cannot allocate %lu B on node %d\n", s->name, size, node);
	return -ENOMEM;
}

/* Map the side for every channel's device (each PTDMA has its own IOMMU group) */
static int side_map(struct ptbw_side *s)
{
	unsigned int i, n = (s->phys ? 1 : s->n) * nch;
	struct device *dev;
	dma_addr_t a;

	s->dma = kvcalloc(n, sizeof(*s->dma), GFP_KERNEL);
	if (!s->dma)
		return -ENOMEM;
	for (i = 0; i < n; i++)
		s->dma[i] = DMA_MAPPING_ERROR;
	for (i = 0; i < n; i++) {
		dev = chans[i % nch].dev;
		if (s->phys)
			a = dma_map_resource(dev, s->phys, s->len, s->dir, 0);
		else
			a = dma_map_page(dev, s->pg[i / nch], 0, piece_size(s), s->dir);
		if (dma_mapping_error(dev, a)) {
			pr_err("%s: mapping %s for %s failed\n", s->name,
			       s->phys ? "MMIO" : "a host piece", dev_name(dev));
			return -EIO;
		}
		s->dma[i] = a;
	}
	return 0;
}

static void side_unmap(struct ptbw_side *s)
{
	unsigned int i, n = (s->phys ? 1 : s->n) * nch;
	struct device *dev;

	for (i = 0; s->dma && i < n; i++) {
		if (s->dma[i] == DMA_MAPPING_ERROR)
			continue;
		dev = chans[i % nch].dev;
		if (s->phys)
			dma_unmap_resource(dev, s->dma[i], s->len, s->dir, 0);
		else
			dma_unmap_page(dev, s->dma[i], piece_size(s), s->dir);
	}
	kvfree(s->dma);
	s->dma = NULL;
}

/* Host side only: hand the pieces to the devices, or back to the CPU */
static void side_sync(struct ptbw_side *s, bool for_cpu)
{
	unsigned int i;

	for (i = 0; i < s->n * nch; i++) {
		if (for_cpu)
			dma_sync_single_for_cpu(chans[i % nch].dev, s->dma[i], piece_size(s), s->dir);
		else
			dma_sync_single_for_device(chans[i % nch].dev, s->dma[i], piece_size(s), s->dir);
	}
}

/* Flag mode: one host page on node, mapped for chans[0] (the only channel) */
static int flag_alloc(int node)
{
	flag_pg = alloc_pages_node(node, GFP_KERNEL | __GFP_ZERO, 0);
	if (!flag_pg) {
		pr_err("flag: cannot allocate a page on node %d\n", node);
		return -ENOMEM;
	}
	flag_dma = dma_map_page(chans[0].dev, flag_pg, 0, PAGE_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(chans[0].dev, flag_dma)) {
		flag_dma = DMA_MAPPING_ERROR;
		pr_err("flag: mapping its page for %s failed\n", dev_name(chans[0].dev));
		return -EIO;
	}
	return 0;
}

static void flag_free(void)
{
	if (flag_dma != DMA_MAPPING_ERROR)
		dma_unmap_page(chans[0].dev, flag_dma, PAGE_SIZE, DMA_TO_DEVICE);
	flag_dma = DMA_MAPPING_ERROR;
	if (flag_pg)
		__free_page(flag_pg);
	flag_pg = NULL;
}

/* DMA address of byte off for channel c; *room = bytes that follow it contiguously */
static dma_addr_t side_addr(const struct ptbw_side *s, u64 off, unsigned int c, u64 *room)
{
	u64 in = off & (piece_size(s) - 1);

	if (s->phys) {
		*room = size - off;
		return s->dma[c] + off;
	}
	*room = piece_size(s) - in;
	return s->dma[(off >> (PAGE_SHIFT + s->order)) * nch + c] + in;
}

static void src_fill(void)
{
	u64 j, words = piece_size(&src) / 8;
	unsigned int i;

	for (i = 0; i < src.n; i++) {
		u64 *p = page_address(src.pg[i]), base = (u64)i * piece_size(&src);

		for (j = 0; j < words; j++)
			p[j] = PTBW_PATTERN ^ (base + j * 8);
	}
}

static void dst_poison(void)
{
	unsigned int i;

	for (i = 0; i < dst.n; i++)
		memset(page_address(dst.pg[i]), 0xa5, piece_size(&dst));
}

/*
 * dst after an iteration. Host src: every word against the pattern. MMIO src:
 * count words the copy left at the poison, and words that read back all-ones
 * (what failed MMIO reads return).
 */
static int dst_check(unsigned int it)
{
	u64 j, nw, base, v, bad = 0, first = 0, got = 0, poison = 0, ones = 0;
	unsigned int i;

	for (i = 0; i < dst.n; i++) {
		const u64 *p = page_address(dst.pg[i]);

		base = (u64)i * piece_size(&dst);
		nw = min_t(u64, piece_size(&dst), size - base) / 8;
		for (j = 0; j < nw; j++) {
			v = p[j];
			if (src.phys) {
				poison += v == PTBW_POISON;
				ones += v == ~0ULL;
			} else if (v != (PTBW_PATTERN ^ (base + j * 8)) && !bad++) {
				first = base + j * 8;
				got = v;
			}
		}
	}
	if (src.phys) {
		pr_info("iter %u: dst: %llu of %lu words still poison, %llu all-ones%s\n", it,
			poison, size / 8, ones, poison || ones ? " (CHECK dmesg for PTDMA errors)" : "");
		return 0;
	}
	if (!bad) {
		pr_info("iter %u: dst verify OK (%lu B)\n", it, size);
		return 0;
	}
	pr_err("iter %u: dst verify FAILED: %llu of %lu words wrong, first at %#llx: %#llx, expected %#llx\n",
	       it, bad, size / 8, first, got, PTBW_PATTERN ^ first);
	return -EIO;
}

/* ---- the copy ------------------------------------------------------------ */

static void ptbw_done(void *arg)		/* hard-irq context */
{
	struct ptbw_chan *c = arg;

	c->t_done = ktime_get();
	if (atomic_dec_and_test(&pending))
		complete(&done);
}

static void ptbw_flag_done(void *arg)	/* hard-irq context */
{
	t_flag = ktime_get();
	if (atomic_dec_and_test(&pending))
		complete(&done);
}

/* One iteration; *ns = first prep to last completion */
static int ptbw_iter(unsigned int it, s64 *ns)
{
	struct dma_async_tx_descriptor *tx;
	unsigned int c, k = 0;
	u64 off, n, room, g;
	ktime_t t0, t_sub, t_end;
	dma_addr_t s, d;
	char line[256];
	int pos, ret = 0;

	reinit_completion(&done);
	atomic_set(&pending, 1);		/* bias, dropped once all are submitted */
	for (c = 0; c < nch; c++) {
		chans[c].bytes = 0;
		chans[c].t_done = 0;
	}
	t_flag = 0;
	if (flag_off) {			/* the last iteration's flag descriptor has completed */
		*(u64 *)page_address(flag_pg) = flag_val + it + 1;
		dma_sync_single_for_device(chans[0].dev, flag_dma, PAGE_SIZE, DMA_TO_DEVICE);
	}

	t0 = ktime_get();
	for (off = 0; off < size; off += n, k++) {
		c = k % nch;
		n = min_t(u64, chunk, size - off);
		s = side_addr(&src, off, c, &room);
		n = min(n, room);
		d = side_addr(&dst, off, c, &room);
		n = min(n, room);
		tx = dmaengine_prep_dma_memcpy(chans[c].ch, d, s, n, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!tx) {
			pr_err("iter %u: prep of %llu B at offset %#llx failed\n", it, n, off);
			ret = -ENOMEM;
			break;
		}
		tx->callback = ptbw_done;
		tx->callback_param = &chans[c];
		atomic_inc(&pending);
		if (dma_submit_error(dmaengine_submit(tx))) {
			atomic_dec(&pending);
			pr_err("iter %u: submit at offset %#llx failed\n", it, off);
			ret = -EIO;
			break;
		}
		chans[c].bytes += n;
	}
	/* Flag mode: the flag on the data's channel, behind its last write */
	if (!ret && flag_off) {
		tx = dmaengine_prep_dma_memcpy(chans[0].ch, dst.dma[0] + flag_off, flag_dma, 8,
					       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!tx) {
			pr_err("iter %u: prep of the flag failed\n", it);
			ret = -ENOMEM;
		} else {
			tx->callback = ptbw_flag_done;
			tx->callback_param = NULL;
			atomic_inc(&pending);
			if (dma_submit_error(dmaengine_submit(tx))) {
				atomic_dec(&pending);
				pr_err("iter %u: submit of the flag failed\n", it);
				ret = -EIO;
			}
		}
	}
	for (c = 0; c < nch; c++)
		dma_async_issue_pending(chans[c].ch);
	t_sub = ktime_get();
	if (atomic_dec_and_test(&pending))
		complete(&done);

	if (!wait_for_completion_timeout(&done, msecs_to_jiffies(timeout_ms))) {
		pr_err("iter %u: %d descriptors still pending after %u ms\n",
		       it, atomic_read(&pending), timeout_ms);
		return -ETIMEDOUT;
	}
	if (ret)
		return ret;

	t_end = t0;
	pos = scnprintf(line, sizeof(line), "per-chan GB/s:");
	for (c = 0; c < nch; c++) {
		if (!chans[c].bytes) {
			pos += scnprintf(line + pos, sizeof(line) - pos, " -");
			continue;
		}
		if (ktime_after(chans[c].t_done, t_end))
			t_end = chans[c].t_done;
		g = gbps(chans[c].bytes, ktime_to_ns(ktime_sub(chans[c].t_done, t0)));
		pos += scnprintf(line + pos, sizeof(line) - pos, " %llu.%03llu", g / 1000, g % 1000);
	}
	if (flag_off) {			/* the iteration ends at the flag's completion */
		pos += scnprintf(line + pos, sizeof(line) - pos, "; flag %llu done %lld ns after the data",
				 flag_val + it + 1, ktime_to_ns(ktime_sub(t_flag, t_end)));
		if (ktime_after(t_flag, t_end))
			t_end = t_flag;
	}
	*ns = ktime_to_ns(ktime_sub(t_end, t0));
	g = gbps(size, *ns);
	pr_info("iter %u: %lu B in %lld ns = %llu.%03llu GB/s (%u desc, submit %lld us); %s\n",
		it, size, *ns, g / 1000, g % 1000, k, ktime_us_delta(t_sub, t0), line);
	return 0;
}

static void ptbw_cleanup(void)
{
	unsigned int c;

	side_unmap(&src);			/* needs chans[].dev */
	side_unmap(&dst);
	flag_free();
	side_free(&src);
	side_free(&dst);
	for (c = 0; c < nch; c++)
		dma_release_channel(chans[c].ch);
	nch = 0;
}

static void side_desc(const struct ptbw_side *s, char *buf, size_t len, int node)
{
	if (s->phys)
		scnprintf(buf, len, "MMIO %#llx", s->phys);
	else
		scnprintf(buf, len, "host DRAM (node %d, %llu KiB pieces)", node, piece_size(s) >> 10);
}

static int __init ptbw_init(void)
{
	u64 g, tot_ns = 0, lo = U64_MAX, hi = 0;
	char sd[64], dd[64], names[PTBW_MAX_CHAN * 24];
	unsigned int it, c;
	int node, pos = 0, bad = 0, ret;
	u64 dst_len = flag_off ? max_t(u64, size, flag_off + 8) : size;
	s64 ns;

	pr_info("start%s%s: size %lu chunk %u iters %u nchan %u numa %d src_phys %#llx dst_phys %#llx flag_off %#llx flag_val %llu\n",
		tag ? " " : "", tag ?: "", size, chunk, iters, nchan, numa, src_phys, dst_phys,
		flag_off, flag_val);
	if (!iters || !size || size % 64 || size > SZ_4G || !chunk || chunk % 64 ||
	    chunk > SZ_1G || nchan < 1 || nchan > PTBW_MAX_CHAN || src_phys % 64 || dst_phys % 64) {
		pr_err("bad parameters (size/chunk multiples of 64, size <= 4 GiB, chunk <= 1 GiB, 1 <= nchan <= 16, phys 64 B aligned)\n");
		return -EINVAL;
	}
	if (flag_off && (!dst_phys || flag_off % 8 || flag_off < size || flag_off > SZ_4G)) {
		pr_err("flag_off needs dst_phys, and must be 8 B aligned, >= size (past the data) and <= 4 GiB\n");
		return -EINVAL;
	}
	if (dst_phys && !allow_write) {
		pr_err("refusing to write MMIO at dst_phys=%#llx without allow_write=1\n", dst_phys);
		return -EPERM;
	}
	if (src_phys && dst_phys && src_phys < dst_phys + dst_len && dst_phys < src_phys + size) {
		pr_err("src and dst MMIO ranges overlap\n");
		return -EINVAL;
	}

	ret = ptbw_get_chans();
	if (ret)
		return ret;
	node = numa >= 0 ? numa : dev_to_node(chans[0].dev);
	if (flag_off && nch != 1) {
		pr_err("flag_off needs exactly one channel (nchan=1): only one requester's posted writes stay in order\n");
		ret = -EINVAL;
		goto out;
	}

	src.phys = src_phys;
	src.len = size;
	dst.phys = dst_phys;
	dst.len = dst_len;
	if (flag_off) {
		ret = flag_alloc(node);
		if (ret)
			goto out;
	}
	if (!src.phys) {
		ret = side_alloc(&src, node);
		if (ret)
			goto out;
		src_fill();
	}
	if (!dst.phys) {
		ret = side_alloc(&dst, node);
		if (ret)
			goto out;
	}
	ret = side_map(&src);
	if (!ret)
		ret = side_map(&dst);
	if (ret)
		goto out;

	side_desc(&src, sd, sizeof(sd), node);
	side_desc(&dst, dd, sizeof(dd), node);
	pr_info("%s -> %s: %lu B per iteration, chunk %u B, %u iterations, %u channels%s\n",
		sd, dd, size, chunk, iters, nch, flag_off ? ", then the 8 B flag on the same channel" : "");
	if (flag_off)
		pr_info("flag: %#llx (dst_phys + %#llx) <- %llu .. %llu, one per iteration\n",
			dst_phys + flag_off, flag_off, flag_val + 1, flag_val + iters);

	for (it = 0; it < iters; it++) {
		if (!dst.phys) {
			dst_poison();
			side_sync(&dst, false);
		}
		ret = ptbw_iter(it, &ns);
		if (ret == -ETIMEDOUT) {
			/*
			 * Descriptors may still be in the hardware, and the driver's
			 * terminate does not stop the one it is running: stay loaded
			 * and keep every buffer, mapping and channel until rmmod.
			 */
			pr_err("DMA did not finish: STAYING LOADED, holding all buffers and channels; check dmesg, then rmmod ptdma_bw (or reboot)\n");
			return 0;
		}
		if (ret)
			goto out;
		if (!dst.phys) {
			side_sync(&dst, true);
			if (dst_check(it))
				bad = -EIO;
		}
		tot_ns += ns;
		g = gbps(size, ns);
		lo = min(lo, g);
		hi = max(hi, g);
	}

	for (c = 0; c < nch; c++)
		pos += scnprintf(names + pos, sizeof(names) - pos, "%s%s", c ? "," : "",
				 dev_name(chans[c].ch->device->dev));
	g = gbps((u64)size * iters, tot_ns);
	pr_info("SUMMARY %s -> %s, size %lu, chunk %u, %u chan [%s] numa %d, %u iters: GB/s min %llu.%03llu max %llu.%03llu aggregate %llu.%03llu%s\n",
		sd, dd, size, chunk, nch, names, numa, iters, lo / 1000, lo % 1000, hi / 1000,
		hi % 1000, g / 1000, g % 1000,
		flag_off ? ", timed to the flag's completion" :
		src.phys || dst.phys ? "" : bad ? ", VERIFY FAILED" : ", verify OK");
	ret = bad ?: -EAGAIN;
out:
	ptbw_cleanup();
	if (ret == -EAGAIN)
		pr_info("done; init returns -EAGAIN on purpose so the module does not stay loaded\n");
	return ret;
}

/* Only reached after a timeout left the module loaded */
static void __exit ptbw_exit(void)
{
	unsigned int c;

	for (c = 0; c < nch; c++)
		dmaengine_terminate_sync(chans[c].ch);
	ptbw_cleanup();
}

module_init(ptbw_init);
module_exit(ptbw_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AMD PTDMA copy bandwidth test (runs in init, then fails with -EAGAIN)");
