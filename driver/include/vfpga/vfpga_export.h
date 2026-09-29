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

#ifndef _VFPGA_EXPORT_H_
#define _VFPGA_EXPORT_H_

#include "vfpga_hw.h"
#include "coyote_defs.h"

/**
 * @brief Exports a region of this card's BAR as a dma-buf, so that another
 * device (e.g. a second FPGA) can import it and write to it peer-to-peer.
 *
 * The exporter hands out no struct pages: an importer's attach maps the BAR
 * range for the importing device with dma_map_resource() and receives a
 * single-entry sg_table holding that bus address. Only importers that
 * declare allow_peer2peer are accepted.
 *
 * @param device vFPGA whose region is exported
 * @param region EXPORT_REGION_* (which BAR region)
 * @param offset byte offset into the region, page aligned
 * @param size   bytes to export, page aligned, within the region
 * @return a dma-buf file descriptor (>= 0), or a negative error
 */
int vfpga_export_dmabuf(struct vfpga_dev *device, uint32_t region, uint64_t offset, uint64_t size);

#endif // _VFPGA_EXPORT_H_
