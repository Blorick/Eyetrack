/*
 * Copyright (c) 2023, Anlogic Inc. and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "al_core.h"
#include "al_rv_sysregs.h"
#include "al_arch_sysops.h"

#define RISCV_XLEN 32

typedef enum
{
    RISCV_TRIGGER_NONE = 0,               // 当前 tselect 位置没有触发器
    RISCV_TRIGGER_SIFIVE_ADDR = 1,        // SiFive 的旧版地址匹配触发器
    RISCV_TRIGGER_ADDR_DATA_MATCH = 2,    // 地址/数据匹配触发器
    RISCV_TRIGGER_INSTRUCTION_COUNT = 3,  // 指令计数触发器
    RISCV_TRIGGER_INTERRUPT = 4,          // 中断触发器
    RISCV_TRIGGER_EXCEPTION = 5,          // 异常触发器
    RISCV_TRIGGER_RESERVED = 15           // 存在但不可用的触发器
} AL_RISCV_TRGIERR_TYPE_ENUM;

typedef enum
{
    RISCV_TDATA1_LOAD = 0,
    RISCV_TDATA1_STORE = 1,
    RISCV_TDATA1_EXCUTE = 2,
    RISCV_TDATA1_USER_MODE = 3,
    RISCV_TDATA1_SUPERVISOR_MODE = 4,
    RISCV_TDATA1_MACHINE_MODE = 6,
    RISCV_TDATA1_MATCH = 7,
    RISCV_TDATA1_CHAIN = 11,
    RISCV_TDATA1_ACTION = 12,
    RISCV_TDATA1_SIZELO = 16,
    RISCV_TDATA1_TIMING = 18,
    RISCV_TDATA1_SELECT = 19,
    RISCV_TDATA1_HIT = 20,
    RISCV_TDATA1_MASKMAX = 21,
    RISCV_TDATA1_DMODE = 27,
    RISCV_TDATA1_TYPE = 28,
} AL_RISCV_TRGIERR_TDATA1_ENUM;

void Al_riscv_set_instruction_breakpoint(unsigned int address);
void Al_riscv_set_data_watchpoint(unsigned int address);
