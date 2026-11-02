//============================================================================
//  servo_bus.h  --  松甲总线舵机协议组帧
//============================================================================
#ifndef SERVO_BUS_H
#define SERVO_BUS_H

#include <stdint.h>

// 生成一条总线舵机位置指令：#<ID3>P<脉宽4>T<时间4>!
// 返回写入 buf 的字节数（不含结尾 '\0'）。buf 至少 16 字节（帧 15 字节 + 1）。
//   id       舵机 ID（0..254）
//   pulse_us 脉宽（微秒，500..2500，1500=中位）
//   time_ms  移动时间（毫秒，0=全速）
int servo_bus_move(char *buf, uint16_t id, uint16_t pulse_us, uint16_t time_ms);

// 生成云台两轴一整帧：#<PAN_ID>P<pan>T<t>!#<TILT_ID>P<tilt>T<t>!
// 返回帧长（不含 '\0'）。buf 至少 32 字节（两帧 30 字节 + '\0'）。
int servo_bus_build_frame(char *buf, uint16_t pan_us, uint16_t tilt_us);

#endif
