//============================================================================
//  servo_uart.c  --  UART1 物理层适配（松甲总线舵机单线串口）
//
//  已按本工程 BSP 实测的 usart 驱动重写（driver/ps_driver/usart_n/）：
//    - 头文件：al_usart_dev.h（不是 al_usart.h）
//    - 初始化：AlUsart_Dev_Init(&dev, 1, &init)，DevId=1 即 USART1
//    - 发送：  AlUsart_Dev_SendDataPolling(&dev, data, size)，阻塞轮询
//  USART1 基址 0x11012000（见 al_hwcfg.h）。本设计只发位置指令，仅用 TX。
//
//  关键点：BSP 生成的 soc_plat.c 里 board_init() 只开了 USART0 的时钟/复位，
//  没开 USART1（我们的 HPF 是 UART1 使能、UART0 关）。所以这里必须自己开
//  USART1 时钟 + 释放复位，否则 USART1 无时钟、发不出数据。
//============================================================================
#include "servo_uart.h"
#include "config.h"

#include "al_hwcfg.h"       // MISC_CTRL_BASE_ADDR 等基地址
#include "al_misc_ll.h"     // AlMisc_ll_SetClkEn / AlMisc_ll_SetReset
#include "al_usart_dev.h"   // AlUsart_Dev_Init / AlUsart_Dev_SendDataPolling

static AL_USART_DevStruct s_usart1;

void servo_uart_init(void)
{
    AL_USART_InitStruct init;

    // 开 USART1 时钟 + 复位再释放（仿 soc_plat.c 对 USART0 的写法）
    AlMisc_ll_SetClkEn(AL_MISC_USART1, 1);
    AlMisc_ll_SetReset(AL_MISC_USART1, 0);
    AlMisc_ll_SetReset(AL_MISC_USART1, 1);

    init.BaudRate        = SERVO_BAUD;            // 115200
    init.WordLength      = AL_USART_CHAR_8_BITS;  // 8N1
    init.StopBits        = AL_USART_STOP_1_BIT;
    init.Parity          = AL_USART_NO_PARITY;
    init.TxCtl           = AL_TRUE;               // 使能发送
    init.RxCtl           = AL_FALSE;              // 只发不读，关接收
    init.HwFlowCtl.Cts   = AL_FALSE;
    init.HwFlowCtl.Rts   = AL_FALSE;
    init.TxFifoWaterMark = 0x8;
    init.RxFifoWaterMark = 0x8;
    init.RxIdleTimeout   = 0xffff;
    init.RxWmTimeout     = 0xffff;
    init.SeparateEn      = AL_TRUE;               // TX/RX 分开引脚（非单线复用）
    init.DmaEn           = AL_FALSE;
    init.RxDmaEn         = AL_FALSE;

    AlUsart_Dev_Init(&s_usart1, 1, &init);        // DevId 1 = USART1
}

void servo_uart_send(const char *buf, int len)
{
    if (len <= 0) {
        return;
    }
    AlUsart_Dev_SendDataPolling(&s_usart1, (AL_U8 *)buf, (AL_U32)len);
}
