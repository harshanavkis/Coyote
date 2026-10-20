// SPDX-License-Identifier: GPL-2.0-only
/*
 * ptdma_q - one AMD PTDMA engine as a low-latency copy engine for user space.
 *
 * The stock ptdma driver (drivers/dma/amd/ptdma) hands the hardware ONE descriptor at a time and starts the next
 * from the completion interrupt: ~5.5 us per descriptor (ptdma_bw, measured 2026-10-12/15). The engine itself walks
 * a ring of 32 B descriptors until its head reaches the tail. ptdma_q takes one PTDMA PCI function from the stock
 * driver (driver_override), keeps its queue running with interrupts off, and lets a process
 *   - register host buffers (pinned, mapped for the engine) and the uwin of a U280 (Loom's user data window),
 *   - submit a batch of copies with one ioctl: each is checked against the registered regions, split where the
 *     DMA mapping is not contiguous, written into the ring, and the tail is written once,
 *   - see completions without a syscall: the queue's register page is mapped READ-ONLY, and the head register
 *     says which descriptor the engine works on (PTQ_INFO gives the ring's base and length).
 *   - or, in USER MODE (PTQ_IOC_USER, 2026-10-19), write the descriptors into the ring and the tail register itself,
 *     no syscall per copy: the ring (mmap page 1) and the register page (page 0) are mapped writable, PTQ_IOC_SEGS and
 *     PTQ_REG_UWIN give the DMA addresses. Allowed only when the engine's IOMMU domain translates (amy, rose): a bad
 *     descriptor then faults in the IOMMU. Without one (clara) user space could DMA anywhere, so only SUBMIT exists.
 * Descriptors of one queue are processed in ring order; whether one descriptor's writes can be overtaken by the
 * next one's (a flag behind data) is for the hardware tests to show (ptq_bench --order).
 * User space never writes descriptors or registers: a bad offset cannot make the engine write outside the
 * registered regions (clara runs without an IOMMU).
 *
 * Register layout and init sequence from drivers/dma/amd/ptdma/ptdma{.h,-dev.c} (pt_core_init,
 * pt_core_execute_cmd).
 *
 * Claim / release a function (the scripts do this):
 *   echo ptdma_q > /sys/bus/pci/devices/0000:e8:00.2/driver_override
 *   echo 0000:e8:00.2 > /sys/bus/pci/drivers/ptdma/unbind; echo 0000:e8:00.2 > /sys/bus/pci/drivers_probe
 *   -> /dev/ptq_0000_e8_00_2
 *   back: echo > .../driver_override; echo 0000:e8:00.2 > /sys/bus/pci/drivers/ptdma_q/unbind; ... drivers_probe
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#include "ptdma_q.h"

/* ptdma.h */
#define CMD_QUEUE_PRIO_OFFSET		0x00
#define CMD_REQID_CONFIG_OFFSET		0x04
#define CMD_TIMEOUT_OFFSET		0x08
#define CMD_CONFIG_OFFSET		0x1120
#define CMD_CLK_GATE_CTL_OFFSET		0x6004
#define CMD_Q_STATUS_INCR		0x1000
#define Q_CONTROL			0x0000
#define Q_TAIL_LO			0x0004
#define Q_HEAD_LO			0x0008
#define Q_INT_ENABLE			0x000C
#define Q_INT_STATUS			0x0010
#define Q_STATUS			0x0100
#define Q_INT_STATUS_RD			0x0104
#define CMD_Q_RUN			BIT(0)
#define CMD_Q_SIZE			GENMASK(7, 3)
#define CMD_QUEUE_PRIO			GENMASK(2, 1)
#define CMD_CONFIG_VHB_EN		BIT(0)
#define CMD_CLK_GATE_CONFIG		(BIT(0) | BIT(12))
#define SUPPORTED_INTERRUPTS		(BIT(0) | BIT(1))
#define CMD_DESC_DW0_VAL		0x500012	/* passthru engine 5, eom, ioc (as the stock driver) */
#define PTQ_BAR				2

#define U280_UWIN_OFFS			0x8000000ULL	/* coyote_defs.h VFPGA_UWIN_OFFS / VFPGA_UWIN_TOTAL */
#define U280_UWIN_LEN			0x8000000ULL

static unsigned int qlen = 128;
module_param(qlen, uint, 0444);
MODULE_PARM_DESC(qlen, "descriptors in the ring, power of two, 32..8192 (default 128)");

static char *uwin_bdf;
module_param(uwin_bdf, charp, 0444);
MODULE_PARM_DESC(uwin_bdf, "PCI name of this host's U280 (e.g. 0000:e1:00.0): its uwin is the only MMIO a copy may touch");

struct ptq_desc {
	u32 dw0, length, src_lo, dw3, dst_lo, dw5, rsvd1, rsvd2;
};


struct ptq_region {
	bool used, mmio;
	u64 len;
	unsigned int nseg;
	struct ptq_seg *seg;
	/* host: */
	struct page **pages;
	unsigned long npages;
	struct sg_table sgt;
	/* mmio: */
	phys_addr_t phys;
	dma_addr_t dma;
};

struct ptq_dev {
	struct pci_dev *pdev;
	void __iomem *regs, *q;
	phys_addr_t q_phys;			/* the queue's register page */
	struct ptq_desc *ring;
	dma_addr_t ring_dma;
	u32 qcontrol;
	u64 submitted;				/* descriptors written so far */
	u64 done_cache;				/* completions last read from the head register */
	bool wedged;				/* a DMA did not finish: keep every mapping */
	bool user_ok;				/* the IOMMU translates for this function */
	bool user;				/* this open drives the ring itself */
	struct mutex lock;
	bool open;
	struct ptq_region reg[PTQ_MAX_REGIONS];
	struct ptq_copy batch[PTQ_MAX_BATCH];	/* the submit being checked (under lock) */
	char name[32];
	struct miscdevice misc;
};

static inline u32 ring_lo(struct ptq_dev *d) { return lower_32_bits(d->ring_dma); }

/* descriptors the engine has finished, from its head register */
static u64 ptq_completed(struct ptq_dev *d)
{
	u32 head = (ioread32(d->q + Q_HEAD_LO) - ring_lo(d)) / sizeof(struct ptq_desc);
	u32 tail = d->submitted % qlen;

	return d->submitted - ((tail - head) & (qlen - 1));
}

/* the engine has nothing left: head == tail (both modes; in user mode only the registers know) */
static bool ptq_idle(struct ptq_dev *d, unsigned int timeout_ms)
{
	unsigned long end = jiffies + msecs_to_jiffies(timeout_ms);

	while (ioread32(d->q + Q_HEAD_LO) != ioread32(d->q + Q_TAIL_LO)) {
		if (time_after(jiffies, end))
			return false;
		usleep_range(10, 50);
	}
	return true;
}

static void ptq_region_free(struct ptq_dev *d, struct ptq_region *r)
{
	struct device *dev = &d->pdev->dev;

	if (!r->used)
		return;
	if (r->mmio) {
		dma_unmap_resource(dev, r->dma, r->len, DMA_BIDIRECTIONAL, 0);
	} else {
		dma_unmap_sgtable(dev, &r->sgt, DMA_BIDIRECTIONAL, 0);
		sg_free_table(&r->sgt);
		unpin_user_pages_dirty_lock(r->pages, r->npages, true);
		kvfree(r->pages);
	}
	kfree(r->seg);
	memset(r, 0, sizeof(*r));
}

static int ptq_free_slot(struct ptq_dev *d)
{
	int i;

	for (i = 0; i < PTQ_MAX_REGIONS; i++)
		if (!d->reg[i].used)
			return i;
	return -ENOSPC;
}

static int ptq_reg_host(struct ptq_dev *d, struct ptq_reg_host __user *arg)
{
	struct device *dev = &d->pdev->dev;
	struct ptq_reg_host a;
	struct ptq_region *r;
	struct scatterlist *sg;
	unsigned long first;
	unsigned int i;
	u64 off = 0;
	long got;
	int h, ret;

	if (copy_from_user(&a, arg, sizeof(a)))
		return -EFAULT;
	if (!a.len || a.len > (16ULL << 30) || (a.va & 63))
		return -EINVAL;
	h = ptq_free_slot(d);
	if (h < 0)
		return h;
	r = &d->reg[h];
	first = a.va >> PAGE_SHIFT;
	r->npages = ((a.va + a.len - 1) >> PAGE_SHIFT) - first + 1;
	r->pages = kvcalloc(r->npages, sizeof(*r->pages), GFP_KERNEL);
	if (!r->pages)
		return -ENOMEM;
	got = pin_user_pages_fast(a.va & PAGE_MASK, r->npages, FOLL_WRITE | FOLL_LONGTERM, r->pages);
	if (got != r->npages) {
		if (got > 0)
			unpin_user_pages(r->pages, got);
		kvfree(r->pages);
		memset(r, 0, sizeof(*r));
		return got < 0 ? got : -EFAULT;
	}
	ret = sg_alloc_table_from_pages(&r->sgt, r->pages, r->npages, offset_in_page(a.va), a.len, GFP_KERNEL);
	if (!ret)
		ret = dma_map_sgtable(dev, &r->sgt, DMA_BIDIRECTIONAL, 0);
	if (ret) {
		sg_free_table(&r->sgt);
		unpin_user_pages(r->pages, r->npages);
		kvfree(r->pages);
		memset(r, 0, sizeof(*r));
		return ret;
	}
	r->used = true;
	r->len = a.len;
	r->seg = kcalloc(r->sgt.nents, sizeof(*r->seg), GFP_KERNEL);
	if (!r->seg) {
		ptq_region_free(d, r);
		return -ENOMEM;
	}
	for_each_sgtable_dma_sg(&r->sgt, sg, i) {
		r->seg[r->nseg++] = (struct ptq_seg){ off, sg_dma_address(sg), sg_dma_len(sg) };
		off += sg_dma_len(sg);
	}
	if (off != a.len) {
		dev_err(dev, "mapping covers %llu of %llu bytes\n", off, a.len);
		ptq_region_free(d, r);
		return -EIO;
	}
	a.handle = h;
	a.nseg = r->nseg;
	return copy_to_user(arg, &a, sizeof(a)) ? -EFAULT : 0;
}

/* the U280's shell BAR = its third memory BAR (as coyote_driver: BAR_SHELL_CONFIG), the uwin above 128 MiB */
static int ptq_reg_uwin(struct ptq_dev *d, struct ptq_reg_uwin __user *arg)
{
	struct device *dev = &d->pdev->dev;
	struct pci_dev *u = NULL;
	struct ptq_reg_uwin a = {};
	struct ptq_region *r;
	unsigned int bus, slot, fn, dom;
	int i, n = 0, h;

	if (!uwin_bdf || sscanf(uwin_bdf, "%x:%x:%x.%x", &dom, &bus, &slot, &fn) != 4)
		return -ENODEV;
	u = pci_get_domain_bus_and_slot(dom, bus, PCI_DEVFN(slot, fn));
	if (!u)
		return -ENODEV;
	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		if (!(pci_resource_flags(u, i) & IORESOURCE_MEM) || !pci_resource_len(u, i))
			continue;
		if (++n == 3)
			break;
	}
	if (n != 3 || pci_resource_len(u, i) < U280_UWIN_OFFS + U280_UWIN_LEN) {
		pci_dev_put(u);
		return -ENODEV;
	}
	h = ptq_free_slot(d);
	if (h < 0) {
		pci_dev_put(u);
		return h;
	}
	r = &d->reg[h];
	r->phys = pci_resource_start(u, i) + U280_UWIN_OFFS;
	pci_dev_put(u);
	r->dma = dma_map_resource(dev, r->phys, U280_UWIN_LEN, DMA_BIDIRECTIONAL, 0);
	if (dma_mapping_error(dev, r->dma))
		return -EIO;
	r->seg = kzalloc(sizeof(*r->seg), GFP_KERNEL);
	if (!r->seg) {
		dma_unmap_resource(dev, r->dma, U280_UWIN_LEN, DMA_BIDIRECTIONAL, 0);
		return -ENOMEM;
	}
	r->used = r->mmio = true;
	r->len = U280_UWIN_LEN;
	r->nseg = 1;
	r->seg[0] = (struct ptq_seg){ 0, r->dma, U280_UWIN_LEN };
	a.handle = h;
	a.len = r->len;
	a.phys = r->phys;
	a.iova = r->dma;
	return copy_to_user(arg, &a, sizeof(a)) ? -EFAULT : 0;
}

static struct ptq_region *ptq_get(struct ptq_dev *d, u32 h, u64 off, u64 len)
{
	struct ptq_region *r;

	if (h >= PTQ_MAX_REGIONS || !d->reg[h].used)
		return NULL;
	r = &d->reg[h];
	return (len && off < r->len && len <= r->len - off) ? r : NULL;
}

/* the DMA address of off and how many bytes stay contiguous from there */
static dma_addr_t ptq_dma(struct ptq_region *r, u64 off, u64 *run)
{
	unsigned int lo = 0, hi = r->nseg;

	while (hi - lo > 1) {
		unsigned int mid = (lo + hi) / 2;

		if (r->seg[mid].off <= off)
			lo = mid;
		else
			hi = mid;
	}
	*run = r->seg[lo].off + r->seg[lo].len - off;
	return r->seg[lo].dma + (off - r->seg[lo].off);
}

static int ptq_put_desc(struct ptq_dev *d, dma_addr_t s, dma_addr_t t, u32 len)
{
	struct ptq_desc *q;
	unsigned long end = jiffies + msecs_to_jiffies(1000);

	/* ring full? the head register (an MMIO read, ~us) only when the cached count says so (2026-10-19: reading it
	 * before every descriptor cost two reads per collective step) */
	while (d->submitted - d->done_cache >= qlen - 1) {
		d->done_cache = ptq_completed(d);
		if (d->submitted - d->done_cache < qlen - 1)
			break;
		if (time_after(jiffies, end))
			return -ETIMEDOUT;
		cpu_relax();
	}
	q = &d->ring[d->submitted % qlen];
	q->dw0 = CMD_DESC_DW0_VAL;
	q->length = len;
	q->src_lo = lower_32_bits(s);
	q->dw3 = upper_32_bits(s) & 0xffff;		/* src_mem 0 = system memory */
	q->dst_lo = lower_32_bits(t);
	q->dw5 = upper_32_bits(t) & 0xffff;
	q->rsvd1 = q->rsvd2 = 0;
	d->submitted++;
	return 0;
}

static void ptq_kick(struct ptq_dev *d)
{
	wmb();		/* descriptors in memory before the engine sees the new tail */
	iowrite32(ring_lo(d) + (d->submitted % qlen) * sizeof(struct ptq_desc), d->q + Q_TAIL_LO);
	iowrite32(d->qcontrol | CMD_Q_RUN, d->q + Q_CONTROL);
}

static int ptq_segs(struct ptq_dev *d, struct ptq_segs __user *arg)
{
	struct ptq_segs a;
	struct ptq_region *r;

	if (copy_from_user(&a, arg, sizeof(a)))
		return -EFAULT;
	if (a.handle >= PTQ_MAX_REGIONS || !d->reg[a.handle].used)
		return -EINVAL;
	r = &d->reg[a.handle];
	a.n = r->nseg;
	if (copy_to_user(u64_to_user_ptr(a.segs), r->seg, min(a.max, r->nseg) * sizeof(*r->seg)))
		return -EFAULT;
	return copy_to_user(arg, &a, sizeof(a)) ? -EFAULT : 0;
}

/* back to a fresh queue: head = tail = ring start, nothing submitted (after every open, both modes) */
static void ptq_reset_ring(struct ptq_dev *d)
{
	iowrite32(d->qcontrol, d->q + Q_CONTROL);
	iowrite32(ring_lo(d), d->q + Q_TAIL_LO);
	iowrite32(ring_lo(d), d->q + Q_HEAD_LO);
	d->submitted = d->done_cache = 0;
}

static int ptq_submit(struct ptq_dev *d, struct ptq_submit __user *arg)
{
	struct ptq_copy *c = d->batch;
	struct ptq_submit a;
	unsigned int i;
	int ret = 0;

	if (d->wedged)
		return -EIO;
	if (d->user)
		return -EBUSY;
	if (copy_from_user(&a, arg, sizeof(a)))
		return -EFAULT;
	if (!a.n || a.n > PTQ_MAX_BATCH)
		return -EINVAL;
	if (copy_from_user(c, u64_to_user_ptr(a.copies), a.n * sizeof(*c)))
		return -EFAULT;
	for (i = 0; i < a.n; i++)		/* check everything before anything goes to the engine */
		if (!ptq_get(d, c[i].src, c[i].src_off, c[i].len) || !ptq_get(d, c[i].dst, c[i].dst_off, c[i].len))
			return -EINVAL;
	for (i = 0; i < a.n && !ret; i++) {
		struct ptq_region *s = &d->reg[c[i].src], *t = &d->reg[c[i].dst];
		u64 so = c[i].src_off, to = c[i].dst_off, left = c[i].len;

		while (left && !ret) {
			u64 srun, trun, n;
			dma_addr_t sd = ptq_dma(s, so, &srun), td = ptq_dma(t, to, &trun);

			n = min3(left, srun, trun);
			n = min_t(u64, n, PTQ_MAX_DESC);
			ret = ptq_put_desc(d, sd, td, n);
			so += n; to += n; left -= n;
		}
	}
	ptq_kick(d);		/* whatever was written, even after an error */
	a.seq = d->submitted;
	if (ret)
		return ret;
	return copy_to_user(arg, &a, sizeof(a)) ? -EFAULT : 0;
}

static long ptq_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct ptq_dev *d = f->private_data;
	void __user *p = (void __user *)arg;
	long ret;

	mutex_lock(&d->lock);
	switch (cmd) {
	case PTQ_IOC_REG_HOST:
		ret = ptq_reg_host(d, p);
		break;
	case PTQ_IOC_REG_UWIN:
		ret = ptq_reg_uwin(d, p);
		break;
	case PTQ_IOC_UNREG:
		ret = -EINVAL;
		if (arg < PTQ_MAX_REGIONS && d->reg[arg].used) {
			ret = -EBUSY;
			if (ptq_idle(d, 1000)) {
				ptq_region_free(d, &d->reg[arg]);
				ret = 0;
			}
		}
		break;
	case PTQ_IOC_SUBMIT:
		ret = ptq_submit(d, p);
		break;
	case PTQ_IOC_SEGS:
		ret = ptq_segs(d, p);
		break;
	case PTQ_IOC_USER:
		ret = -EPERM;
		if (d->user_ok && !d->wedged) {
			ret = -EBUSY;
			if (ptq_idle(d, 1000)) {
				ptq_reset_ring(d);
				d->user = true;
				ret = 0;
			}
		}
		break;
	case PTQ_IOC_INFO: {
		struct ptq_info i = {
			.ring_lo = ring_lo(d), .qlen = qlen, .desc_size = sizeof(struct ptq_desc),
			.submitted = d->submitted, .completed = ptq_completed(d),
			.status = ioread32(d->q + Q_STATUS), .int_status = ioread32(d->q + Q_INT_STATUS),
			.qcontrol = d->qcontrol, .user_ok = d->user_ok,
		};
		ret = copy_to_user(p, &i, sizeof(i)) ? -EFAULT : 0;
		break;
	}
	default:
		ret = -ENOTTY;
	}
	mutex_unlock(&d->lock);
	return ret;
}

/* page 0: the queue's register page (read-only, writable in user mode); page 1: the ring (user mode) */
static int ptq_mmap(struct file *f, struct vm_area_struct *vma)
{
	struct ptq_dev *d = f->private_data;
	const size_t ring = PAGE_ALIGN(qlen * sizeof(struct ptq_desc));

	if (vma->vm_pgoff == 1) {
		if (!d->user || vma->vm_end - vma->vm_start != ring)
			return -EINVAL;
		vma->vm_pgoff = 0;
		return dma_mmap_coherent(&d->pdev->dev, vma, d->ring, d->ring_dma, ring);
	}
	if (vma->vm_pgoff || vma->vm_end - vma->vm_start != PAGE_SIZE || ((vma->vm_flags & VM_WRITE) && !d->user))
		return -EINVAL;
	if (!d->user)
		vm_flags_clear(vma, VM_MAYWRITE);
	vm_flags_set(vma, VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return io_remap_pfn_range(vma, vma->vm_start, d->q_phys >> PAGE_SHIFT, PAGE_SIZE, vma->vm_page_prot);
}

static int ptq_open(struct inode *ino, struct file *f)
{
	struct ptq_dev *d = container_of(f->private_data, struct ptq_dev, misc);

	mutex_lock(&d->lock);
	if (d->open) {
		mutex_unlock(&d->lock);
		return -EBUSY;
	}
	d->open = true;
	f->private_data = d;
	mutex_unlock(&d->lock);
	return 0;
}

static void ptq_drop_all(struct ptq_dev *d)
{
	int i;

	if (!ptq_idle(d, 2000)) {
		/* the engine may still write: keep every buffer pinned and mapped (leaked) */
		dev_err(&d->pdev->dev, "queue not idle (%llu of %llu done): mappings kept\n",
			ptq_completed(d), d->submitted);
		d->wedged = true;
		return;
	}
	for (i = 0; i < PTQ_MAX_REGIONS; i++)
		ptq_region_free(d, &d->reg[i]);
}

static int ptq_release(struct inode *ino, struct file *f)
{
	struct ptq_dev *d = f->private_data;

	mutex_lock(&d->lock);
	ptq_drop_all(d);
	if (!d->wedged)
		ptq_reset_ring(d);
	d->user = false;
	d->open = false;
	mutex_unlock(&d->lock);
	return 0;
}

static const struct file_operations ptq_fops = {
	.owner = THIS_MODULE,
	.open = ptq_open,
	.release = ptq_release,
	.unlocked_ioctl = ptq_ioctl,
	.mmap = ptq_mmap,
};

static int ptq_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct ptq_dev *d;
	int ret;

	if (qlen < 32 || qlen > 8192 || !is_power_of_2(qlen))
		return -EINVAL;
	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->pdev = pdev;
	mutex_init(&d->lock);
	ret = pci_enable_device(pdev);
	if (ret)
		return ret;
	ret = pci_request_region(pdev, PTQ_BAR, "ptdma_q");
	if (ret)
		goto e_disable;
	d->regs = pci_iomap(pdev, PTQ_BAR, 0);
	if (!d->regs) {
		ret = -ENOMEM;
		goto e_region;
	}
	pci_set_master(pdev);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(48));
	if (ret)
		goto e_unmap;
	dma_set_max_seg_size(dev, UINT_MAX);
	d->ring = dma_alloc_coherent(dev, PAGE_ALIGN(qlen * sizeof(struct ptq_desc)), &d->ring_dma, GFP_KERNEL);
	if (!d->ring) {
		ret = -ENOMEM;
		goto e_unmap;
	}

	/* pt_core_init, without the interrupt */
	iowrite32(CMD_CONFIG_VHB_EN, d->regs + CMD_CONFIG_OFFSET);
	iowrite32(CMD_QUEUE_PRIO, d->regs + CMD_QUEUE_PRIO_OFFSET);
	iowrite32(0, d->regs + CMD_TIMEOUT_OFFSET);
	iowrite32(CMD_CLK_GATE_CONFIG, d->regs + CMD_CLK_GATE_CTL_OFFSET);
	iowrite32(0, d->regs + CMD_REQID_CONFIG_OFFSET);
	d->q = d->regs + CMD_Q_STATUS_INCR;
	d->q_phys = pci_resource_start(pdev, PTQ_BAR) + CMD_Q_STATUS_INCR;
	iowrite32(0, d->q + Q_INT_ENABLE);
	iowrite32(0, d->q + Q_CONTROL);
	ioread32(d->q + Q_INT_STATUS_RD);
	ioread32(d->q + Q_STATUS);
	iowrite32(SUPPORTED_INTERRUPTS, d->q + Q_INT_STATUS);
	d->qcontrol = FIELD_PREP(CMD_Q_SIZE, (ffs(qlen) - 2) & 0x1f) | (upper_32_bits(d->ring_dma) << 16);
	iowrite32(ring_lo(d), d->q + Q_TAIL_LO);
	iowrite32(ring_lo(d), d->q + Q_HEAD_LO);
	iowrite32(d->qcontrol, d->q + Q_CONTROL);

	{
		struct iommu_domain *dom = iommu_get_domain_for_dev(dev);

		d->user_ok = dom && (dom->type & __IOMMU_DOMAIN_PAGING);
	}
	snprintf(d->name, sizeof(d->name), "ptq_%04x_%02x_%02x_%x", pci_domain_nr(pdev->bus), pdev->bus->number,
		 PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));
	d->misc = (struct miscdevice){ .minor = MISC_DYNAMIC_MINOR, .name = d->name, .fops = &ptq_fops, .mode = 0666 };
	ret = misc_register(&d->misc);
	if (ret)
		goto e_ring;
	pci_set_drvdata(pdev, d);
	dev_info(dev, "/dev/%s: ring %u x 32 B at dma %pad, numa node %d, user mode %s\n", d->name, qlen, &d->ring_dma,
		 dev_to_node(dev), d->user_ok ? "allowed (IOMMU translates)" : "refused (no translating IOMMU)");
	return 0;

e_ring:
	dma_free_coherent(dev, PAGE_ALIGN(qlen * sizeof(struct ptq_desc)), d->ring, d->ring_dma);
e_unmap:
	pci_iounmap(pdev, d->regs);
e_region:
	pci_release_region(pdev, PTQ_BAR);
e_disable:
	pci_disable_device(pdev);
	return ret;
}

static void ptq_remove(struct pci_dev *pdev)
{
	struct ptq_dev *d = pci_get_drvdata(pdev);

	misc_deregister(&d->misc);
	mutex_lock(&d->lock);
	ptq_drop_all(d);
	mutex_unlock(&d->lock);
	iowrite32(0, d->q + Q_CONTROL);		/* stop the queue */
	if (!d->wedged)
		dma_free_coherent(&pdev->dev, PAGE_ALIGN(qlen * sizeof(struct ptq_desc)), d->ring, d->ring_dma);
	pci_iounmap(pdev, d->regs);
	pci_release_region(pdev, PTQ_BAR);
	pci_clear_master(pdev);
	pci_disable_device(pdev);
}

/* no ids: a function is given to ptdma_q only through driver_override */
static struct pci_driver ptq_driver = {
	.name = "ptdma_q",
	.id_table = NULL,
	.probe = ptq_probe,
	.remove = ptq_remove,
};
module_pci_driver(ptq_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AMD PTDMA queue for user space: batched submit, polled completion");
