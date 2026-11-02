//============================================================================
//  servo_bus.c  --  松甲总线舵机协议组帧
//
//  帧格式（取自厂商 K210 云台例程 platform_color_tracing.py 与 STM32 源码）：
//     #<ID3>P<脉宽4>T<时间4>!
//   脉宽单位 us（500..2500，1500=中位），时间单位 ms（0=全速），ID 3 位十进制。
//   两轴连发：#000P1500T0000!#001P1500T0000!
//============================================================================
#include "servo_bus.h"
#include "config.h"

// 零填充 4 位十进制（0..9999），写到 out，不写 '\0'
static void put4(char *out, uint16_t v)
{
    out[0] = (char)('0' + (v / 1000) % 10);
    out[1] = (char)('0' + (v / 100)  % 10);
    out[2] = (char)('0' + (v / 10)   % 10);
    out[3] = (char)('0' + (v % 10));
}

// 零填充 3 位十进制 ID（0..999）
static void put3(char *out, uint16_t id)
{
    out[0] = (char)('0' + (id / 100) % 10);
    out[1] = (char)('0' + (id / 10)  % 10);
    out[2] = (char)('0' + (id % 10));
}

int servo_bus_move(char *buf, uint16_t id, uint16_t pulse_us, uint16_t time_ms)
{
    // 双保险钳位（FPGA 侧 gimbal_ctrl 已把角度限在 120..880）
    if (pulse_us < SERVO_PULSE_MIN) pulse_us = SERVO_PULSE_MIN;
    if (pulse_us > SERVO_PULSE_MAX) pulse_us = SERVO_PULSE_MAX;

    char *p = buf;
    *p++ = '#';
    put3(p, id);        p += 3;
    *p++ = 'P';
    put4(p, pulse_us);  p += 4;
    *p++ = 'T';
    put4(p, time_ms);   p += 4;
    *p++ = '!';
    *p   = '\0';
    return (int)(p - buf);          // 15 字节
}

int servo_bus_build_frame(char *buf, uint16_t pan_us, uint16_t tilt_us)
{
    int n = 0;
    n += servo_bus_move(buf + n, SERVO_PAN_ID,  pan_us,  SERVO_MOVE_TIME_MS);
    n += servo_bus_move(buf + n, SERVO_TILT_ID, tilt_us, SERVO_MOVE_TIME_MS);
    return n;                        // 30 字节
}
