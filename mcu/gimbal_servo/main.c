//============================================================================
//  main.c  --  云台总线舵机驱动（gimbal_servo）
//
//  数据流：FPGA 的 gimbal_ctrl 算好 pan/tilt 角（0..1000）→ 经 AHB 寄存器桥
//  （ahb_sram_regs 的 reg[8..11]）→ 本程序轮询读取 → 角度转脉宽
//  （pulse = 500 + 2*angle，与 servo_pwm.v 完全一致）→ 组总线帧 → UART1 发出。
//
//  追踪算法留在 FPGA，本程序不做任何控制决策，因此行为与 v1.15 100% 一致。
//============================================================================
#include <stdint.h>
#include "config.h"
#include "servo_bus.h"
#include "servo_uart.h"

// fabric 寄存器读取：AHB 外部总线直接映射，volatile 防优化
#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

//------------------------------------------------------------------------------
// 粗延时（忙等）。用于 20ms 量级的舵机周期，精度不敏感，避免依赖 BSP 定时器。
// 循环体每轮约 4~8 拍，故除以 4 后按核心频率估算 1ms。
//------------------------------------------------------------------------------
static void delay_ms(uint32_t ms)
{
    volatile uint32_t n = ms * (CORE_CLOCK_HZ / 1000UL / 4UL);
    while (n--) { }
}

int main(void)
{
    char frame[32];
    int  n;

    servo_uart_init();

    // 上电先把两轴归到 1500us 中位（较慢速度），与 K210 例程 init 一致
    n = 0;
    n += servo_bus_move(frame + n, SERVO_PAN_ID,  1500, SERVO_INIT_TIME_MS);
    n += servo_bus_move(frame + n, SERVO_TILT_ID, 1500, SERVO_INIT_TIME_MS);
    servo_uart_send(frame, n);
    delay_ms(1000);

    while (1) {
        // 读 FPGA 算好的角度（每帧更新一次）
        uint32_t pan    = REG32(FABRIC_REG_BASE + REG_PAN_OFFSET)    & 0xFFFFu;
        uint32_t tilt   = REG32(FABRIC_REG_BASE + REG_TILT_OFFSET)   & 0xFFFFu;
        uint32_t status = REG32(FABRIC_REG_BASE + REG_STATUS_OFFSET);

        // 角度 -> 脉宽：pulse_us = 500 + 2*angle
        uint16_t pan_us  = (uint16_t)(SERVO_PULSE_MIN + 2u * pan);
        uint16_t tilt_us = (uint16_t)(SERVO_PULSE_MIN + 2u * tilt);

        // 组帧发送
        n = servo_bus_build_frame(frame, pan_us, tilt_us);
        servo_uart_send(frame, n);

        // 调试打印（走 UART0，可选；注释掉以减小开销）
        // AL_LOG(AL_LOG_LEVEL_NOTICE, "pan=%u tilt=%u lock=%u\r\n",
        //        (unsigned)pan_us, (unsigned)tilt_us,
        //        (unsigned)((status >> 1) & 1u));

        (void)status;
        delay_ms(SERVO_UPDATE_PERIOD_MS);
    }

    return 0;
}
