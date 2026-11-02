#include "al_rv32_trigger.h"

void Al_riscv_set_instruction_breakpoint(unsigned int address)
{
    unsigned int Tdata1 = 0;
    ARCH_SYSREG_WRITE(CSR_TSELECT, 0);
    printf("TDATA1 is %lx\r\n", ARCH_SYSREG_READ(CSR_TDATA1));
    Tdata1 = 2 << RISCV_TDATA1_TYPE | 0 << RISCV_TDATA1_MATCH | 1 << RISCV_TDATA1_MACHINE_MODE | 1 << RISCV_TDATA1_EXCUTE;
    ARCH_SYSREG_WRITE(CSR_TDATA1, Tdata1);
    printf("TDATA1 is %lx\r\n", ARCH_SYSREG_READ(CSR_TDATA1));
    ARCH_SYSREG_WRITE(CSR_TDATA2, address);
    printf("TDATA2 breakpoint address is %lx\r\n", ARCH_SYSREG_READ(CSR_TDATA2));
}

void Al_riscv_set_data_watchpoint(unsigned int address)
{
    unsigned int Tdata1 = 0;
    ARCH_SYSREG_WRITE(CSR_TSELECT, 0);
    printf("TDATA1 is %x\r\n", ARCH_SYSREG_READ(CSR_TDATA1));
    Tdata1 = 2 << RISCV_TDATA1_TYPE | 1 << RISCV_TDATA1_TIMING | 0 << RISCV_TDATA1_MATCH | 1 << RISCV_TDATA1_MACHINE_MODE | 1 << RISCV_TDATA1_STORE;
    ARCH_SYSREG_WRITE(CSR_TDATA1, Tdata1);
    printf("Tdata1 is %x\r\n", Tdata1);
    printf("TDATA1 is %lx\r\n", ARCH_SYSREG_READ(CSR_TDATA1));
    ARCH_SYSREG_WRITE(CSR_TDATA2, address);
    printf("TDATA2 watchpoint address is %lx\r\n", ARCH_SYSREG_READ(CSR_TDATA2));
}