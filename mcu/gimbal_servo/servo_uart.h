//============================================================================
//  servo_uart.h  --  UART1 物理层（驱动松甲总线舵机单线串口）
//============================================================================
#ifndef SERVO_UART_H
#define SERVO_UART_H

#include <stdint.h>

void servo_uart_init(void);                    // 初始化 UART1 @ SERVO_BAUD
void servo_uart_send(const char *buf, int len); // 阻塞发送 len 字节

#endif
