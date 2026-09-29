/*
 * Copyright (c) 2025,  Systems Group, ETH Zurich
 * All rights reserved.
 *
 * This file is part of the Coyote device driver for Linux.
 * Coyote can be found at: https://github.com/fpgasystems/Coyote
 *
 * This source code is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING". If not found, a copy of the GNU General Public
 * License can be found <https://www.gnu.org/licenses/>.
 */

#include "vfpga_export.h"

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/fcntl.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)

/// What an exported dma-buf refers to: a physical BAR range of this card
struct bar_dmabuf {
    phys_addr_t phys;
    size_t size;
};

static int bar_dmabuf_attach(struct dma_buf *buf, struct dma_buf_attachment *attach) {
    // A BAR is MMIO: only importers that can take peer-to-peer addresses
    if (!attach->peer2peer) {
        pr_warn("coyote export: importer does not allow peer-to-peer\n");
        return -EOPNOTSUPP;
    }
    return 0;
}

static struct sg_table *bar_dmabuf_map(struct dma_buf_attachment *attach, enum dma_data_direction dir) {
    struct bar_dmabuf *priv = attach->dmabuf->priv;
    struct sg_table *sgt;
    dma_addr_t addr;

    sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
    if (!sgt)
        return ERR_PTR(-ENOMEM);
    if (sg_alloc_table(sgt, 1, GFP_KERNEL)) {
        kfree(sgt);
        return ERR_PTR(-ENOMEM);
    }

    // The bus address the IMPORTING device must use to reach this BAR range
    addr = dma_map_resource(attach->dev, priv->phys, priv->size, dir, DMA_ATTR_SKIP_CPU_SYNC);
    if (dma_mapping_error(attach->dev, addr)) {
        pr_warn("coyote export: dma_map_resource failed for %pa, %zu bytes\n", &priv->phys, priv->size);
        sg_free_table(sgt);
        kfree(sgt);
        return ERR_PTR(-EIO);
    }

    // No struct page behind MMIO; importers use only the DMA address/length
    sg_set_page(sgt->sgl, NULL, priv->size, 0);
    sg_dma_address(sgt->sgl) = addr;
    sg_dma_len(sgt->sgl) = priv->size;
    return sgt;
}

static void bar_dmabuf_unmap(struct dma_buf_attachment *attach, struct sg_table *sgt, enum dma_data_direction dir) {
    struct bar_dmabuf *priv = attach->dmabuf->priv;

    dma_unmap_resource(attach->dev, sg_dma_address(sgt->sgl), priv->size, dir, DMA_ATTR_SKIP_CPU_SYNC);
    sg_free_table(sgt);
    kfree(sgt);
}

static void bar_dmabuf_release(struct dma_buf *buf) {
    kfree(buf->priv);
}

static const struct dma_buf_ops bar_dmabuf_ops = {
    .attach = bar_dmabuf_attach,
    .map_dma_buf = bar_dmabuf_map,
    .unmap_dma_buf = bar_dmabuf_unmap,
    .release = bar_dmabuf_release,
};

int vfpga_export_dmabuf(struct vfpga_dev *device, uint32_t region, uint64_t offset, uint64_t size) {
    DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
    struct bar_dmabuf *priv;
    struct dma_buf *buf;
    phys_addr_t base;
    uint64_t region_size;
    int fd;

    switch (region) {
        case EXPORT_REGION_CTRL_USER:
            base = device->vfpga_cnfg_phys_addr + VFPGA_CTRL_USER_OFFS;
            region_size = VFPGA_CTRL_USER_SIZE;
            break;
        case EXPORT_REGION_UWIN:
            base = device->uwin_phys_addr;
            region_size = device->uwin_size;
            break;
        default:
            pr_warn("coyote export: unknown region %u\n", region);
            return -EINVAL;
    }

    if (!size || (offset & ~PAGE_MASK) || (size & ~PAGE_MASK) || offset + size > region_size) {
        pr_warn("coyote export: offset %llu / size %llu not page aligned or outside the %llu byte region\n",
                offset, size, region_size);
        return -EINVAL;
    }

    priv = kzalloc(sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;
    priv->phys = base + offset;
    priv->size = size;

    exp_info.ops = &bar_dmabuf_ops;
    exp_info.size = size;
    exp_info.flags = O_RDWR | O_CLOEXEC;
    exp_info.priv = priv;

    buf = dma_buf_export(&exp_info);
    if (IS_ERR(buf)) {
        kfree(priv);
        return PTR_ERR(buf);
    }

    fd = dma_buf_fd(buf, O_CLOEXEC);
    if (fd < 0) {
        dma_buf_put(buf);   // releases priv through bar_dmabuf_release
        return fd;
    }

    dbg_info("exported vFPGA %d region %u, phys %pa, %llu bytes as dma-buf fd %d\n",
             device->id, region, &priv->phys, size, fd);
    return fd;
}

#else

int vfpga_export_dmabuf(struct vfpga_dev *device, uint32_t region, uint64_t offset, uint64_t size) {
    pr_warn("coyote export: dma-buf export needs Linux >= 6.2\n");
    return -EOPNOTSUPP;
}

#endif
