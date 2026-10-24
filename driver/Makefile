###############################################################################
# Copyright (c) 2025,  Systems Group, ETH Zurich
# All rights reserved.
# 
# This file is part of the Coyote device driver for Linux.
# Coyote can be found at: https://github.com/fpgasystems/Coyote
#
# This source code is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# The full GNU General Public License is included in this distribution in
# the file called "COPYING". If not found, a copy of the GNU General Public  
# Licence can be found <https://www.gnu.org/licenses/>.
###############################################################################

# Target platform: ultrascale_plus for u55c, u250, u280 or versal for v80
TARGET_PLATFORM ?= ultrascale_plus

# Target output name: a Versal build is a separate module (coyote_driver_versal)
# so that both can be loaded in one host
# Loom builds are named loom_coyote[_versal] (owner 2026-10-22): another user's stock coyote_driver on the same host must not
# unload, shadow or collide with ours (module, PCI driver, device and sysfs names all differ; coyote_defs.h)
ifeq ($(TARGET_PLATFORM), versal)
MODNAME := loom_coyote_versal
else
MODNAME := loom_coyote
endif
obj-m := $(MODNAME).o

# Pre-requisites that need to be compiled for the Coyote driver
$(MODNAME)-objs := src/coyote_driver.o 
$(MODNAME)-objs += src/coyote_setup.o 
$(MODNAME)-objs += src/coyote_sysfs.o 

# LOOM LOCAL CHANGE: DMA=qdma builds the UltraScale+ driver for the soft-QDMA U280 static (STATIC_DMA=qdma,
# device 913F) from pci_qdma.c, into build_qdma/. DMA_XDMA / DMA_QDMA select the DMA layer, PLATFORM_* the card.
ifeq ($(TARGET_PLATFORM), versal)
$(MODNAME)-objs += src/platform/pci_qdma.o
ccflags-y += -DPLATFORM_VERSAL -DDMA_QDMA
else ifeq ($(TARGET_PLATFORM), ultrascale_plus)
ifeq ($(DMA), qdma)
$(MODNAME)-objs += src/platform/pci_qdma.o
ccflags-y += -DPLATFORM_ULTRASCALE_PLUS -DDMA_QDMA
else
$(MODNAME)-objs += src/platform/pci_xdma.o
ccflags-y += -DPLATFORM_ULTRASCALE_PLUS -DDMA_XDMA
endif
else
$(error "Unsupported TARGET_PLATFORM specified. Supported options are: ultrascale_plus, versal")
endif
$(MODNAME)-objs += src/platform/pci_util.o

$(MODNAME)-objs += src/reconfig/reconfig_hw.o 
$(MODNAME)-objs += src/reconfig/reconfig_isr.o
$(MODNAME)-objs += src/reconfig/reconfig_mem.o
$(MODNAME)-objs += src/reconfig/reconfig_ops.o

$(MODNAME)-objs += src/vfpga/vfpga_gup.o 
$(MODNAME)-objs += src/vfpga/vfpga_export.o
$(MODNAME)-objs += src/vfpga/vfpga_hw.o 
$(MODNAME)-objs += src/vfpga/vfpga_isr.o 
$(MODNAME)-objs += src/vfpga/vfpga_ops.o 
$(MODNAME)-objs += src/vfpga/vfpga_uisr.o 

# Location of the Linux kernel header files, we will be building within the context of the kernel's Makefile (the flag -C sets the context)
# This gives us access to all the Linux kernel functionality, typically implemented in <linux/*.h> and <asm/*.h>
#KERNELDIR ?= /nix/store/83wasx6wh3idjgkdjmk7b1w2w1v620gb-linux-6.9-dev/lib/modules/6.9.0-rc7/build
KERNELDIR ?= /home/harshanavkis/doctor-cluster-config/result-dev/lib/modules/$(shell uname -r)/build

ifeq ($(TARGET_PLATFORM), versal)
BUILD_DIR := $(shell pwd)/build_versal
else ifeq ($(DMA), qdma)
BUILD_DIR := $(shell pwd)/build_qdma
else
BUILD_DIR := $(shell pwd)/build
endif
ccflags-y += -std=gnu11 -Wno-declaration-after-statement -I$(src)/include -I$(src)/include/reconfig -I$(src)/include/vfpga -I$(src)/include/platform

# To avoid polluting the source files with a bunch of build (*.o, *.mod etc.) files, the source files are temporarily copied to the build folder for compilation
all:
	mkdir -p $(BUILD_DIR)
	cp Makefile $(BUILD_DIR)
	cp -r src/ $(BUILD_DIR)
	cp -r include/ $(BUILD_DIR)
	$(MAKE) -C $(KERNELDIR) M=$(BUILD_DIR) modules
	rm -rf $(BUILD_DIR)/src $(BUILD_DIR)/include $(BUILD_DIR)/Makefile

clean:
	rm -rf build build_versal build_qdma
