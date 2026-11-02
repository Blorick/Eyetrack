#########################################################################
# Copyright (c) 2023, Anlogic Inc. and Contributors. All rights reserved.

# SPDX-License-Identifier: BSD-3-Clause
#########################################################################

# gcc riscv option: https://gcc.gnu.org/onlinedocs/gcc/RISC-V-Options.html

#########################################################################
# chip define
CORE            := riscv
CHIP_ARCH       := rv32imafc_zba_zbb_zbc_zbs
ARCH_ABI        := ilp32f


#########################################################################
# cflags define
AL_CFLAGS     += -march=$(CHIP_ARCH) -mabi=$(ARCH_ABI) -mcmodel=medlow $(GC_CFLAGS) -fno-common
LDFLAGS       += -march=$(CHIP_ARCH) -mabi=$(ARCH_ABI) -mcmodel=medlow $(GC_CFLAGS) -fno-common

# define nuclei arch
ARCH_EXT        := ext-nuclei

#########################################################################
# ldflags define

NEWLIB = nano

ifeq ($(NEWLIB),nano)
NEWLIB_LDFLAGS = --specs=nano.specs
endif


#########################################################################
# PH1P35 linker script comparison (DOWNLOAD mode reference)
#
# | DOWNLOAD    | Linker Script               | Boot/Load Source | .text execute (VMA) | rodata/data/bss/heap/stack | Typical scenario |
# |-------------|-----------------------------|------------------|---------------------|----------------------------|------------------|
# | ddr         | gcc_ph1p35_ddr.ld           | DDR (debug load) | DDR                 | DDR                        | Full DDR run, large memory footprint; requires DDR ready |
# | flash_ddr   | gcc_ph1p35_flash_ddr.ld     | Flash (LMA)      | DDR                 | DDR (config in DLM)        | Boot from flash, copy hot runtime sections to DDR |
# | ilm_dlm     | gcc_ph1p35_ilm_dlm.ld       | Flash (LMA)      | ILM                 | DLM                        | On-chip low-latency run (code in ILM, data in DLM) |
# | tcm         | gcc_ph1p35_tcm.ld           | TCM              | TCM                 | TCM                        | All-in-TCM execution, simple single-region mapping |
# | xip_dlm     | gcc_ph1p35_xip_dlm.ld       | Flash XIP        | Flash (XIP)         | DLM                        | Execute code in place from flash; mutable data in DLM |
# | xip_ram     | gcc_ph1p35_xip_ram.ld       | Flash XIP        | Flash (XIP)         | DLM (+ .dlm_section)       | XIP + extra DLM placed code sections to update flash |
#
# Notes:
# 1) LMA/VMA behavior:
#    - Modes with "Flash (LMA)" place load image in flash and relocate at startup.
#    - Modes with "Flash (XIP)" keep .text in flash (no text relocation), while writable
#      sections are still placed in DLM.
# 2) Stack/heap defaults in linker scripts:
#    - 1K: ddr, tcm, xip_dlm
#    - 2K: flash_ddr, ilm_dlm, xip_ram
# 3) ph1p35.inc CFLAGS mapping:
#    - ilm_dlm/flash_ddr enable VECTOR_TABLE_REMAPPED.
#    - ddr/flash_ddr/xip_dlm/xip_ram enable I/D cache.
#    - xip_ram additionally enables WR_FLASH_XIP and USE_DLM_SECTION.
#########################################################################

ifeq ($(AL_CHIP),ph1p35)
CFLAGS += -DSIMPLE_PRINTF
ifeq ($(DOWNLOAD),ilm_dlm)
CFLAGS += -DVECTOR_TABLE_REMAPPED -DENABLE_DCACHE
endif
ifeq ($(DOWNLOAD),xip_dlm)
CFLAGS += -DENABLE_ICACHE -DENABLE_DCACHE
endif
ifeq ($(DOWNLOAD),xip_ram)
CFLAGS += -DENABLE_ICACHE -DWR_FLASH_XIP -DENABLE_DCACHE
CFLAGS += -DUSE_DLM_SECTION
endif
ifeq ($(DOWNLOAD),ddr)
CFLAGS += -DENABLE_ICACHE -DENABLE_DCACHE
endif
ifeq ($(DOWNLOAD),flash_ddr)
CFLAGS += -DVECTOR_TABLE_REMAPPED
CFLAGS += -DENABLE_ICACHE -DENABLE_DCACHE
endif
endif


#########################################################################
# lib prebuild define
LIB_PREBUILD_DIR  ?= $(AL_SDK_ROOT)/prebuild/$(CHIP)


#########################################################################
# define drivers support list

ps_driver_list = dma_n gpio_n i2c_n misc_n qspi_n qspixip_n usart_n
pl_driver_list =

# vim: syntax=make