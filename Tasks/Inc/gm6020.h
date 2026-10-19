#ifndef GM6020_H
#define GM6020_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/*
 * DJI GM6020 电压控制驱动 (C 语言)
 *
 * 控制帧:
 *   0x1FF: ID 1~4, 每电机 2 字节 int16, 大端
 *   0x2FF: ID 5~7
 *   电压范围 -25000 ~ +25000
 *
 * 反馈帧:
 *   0x204 + ID (ID=1 -> 0x205)
 *   data[0..1] 机械角度 0~8191
 *   data[2..3] 转速 rpm (int16)
 *   data[4..5] 转矩电流 (int16)
 *   data[6]    温度 degC
 *   data[7]    保留
 *
 * 说明:
 *   - 驱动只使用 CAN1, 过滤器接受 0x200~0x20F, 软件再检查 ID。
 *   - 角度支持多圈累计, 第一次收到反馈时把当前位置作为 0 点。
 *   - 所有对外状态通过临界区快照读取, 避免和 CAN 中断竞争。
 */

typedef struct
{
    float    angle_rad;       /* 累计机械角度, 第一次反馈时清零 */
    float    speed_rad_s;     /* 转速, rad/s */
    float    current_a;       /* 转矩电流近似值, A */
    float    temperature_c;   /* 温度, degC */
    int16_t  raw_speed_rpm;   /* 原始转速 */
    int16_t  raw_current;     /* 原始转矩电流 */
    uint16_t raw_angle;       /* 原始机械角度 0~8191 */
    uint32_t timestamp_ms;    /* 收到该帧时的 1 kHz 控制节拍 */
    uint8_t  online;          /* 至少收到过一帧 */
} GM6020_State;

/* 初始化 CAN 过滤器、启动 CAN1、开启 RX FIFO0 中断。返回 0 表示失败。 */
uint8_t GM6020_Init(CAN_HandleTypeDef *hcan, uint8_t motor_id);

/* 在 HAL_CAN_RxFifo0MsgPendingCallback 中调用。now_ms 为控制节拍。 */
void GM6020_OnCanRx(CAN_HandleTypeDef *hcan, uint32_t now_ms);

/* 读取一份一致的状态快照。返回 1 表示已收到过反馈。 */
uint8_t GM6020_GetState(GM6020_State *out);

/* 发送电压给定, 自动限幅到 -25000~25000。返回 1 表示已写入邮箱。 */
uint8_t GM6020_SendVoltage(int16_t voltage);

uint32_t GM6020_GetRxCount(void);
uint32_t GM6020_GetTxCount(void);
uint32_t GM6020_GetTxFailCount(void);

#endif /* GM6020_H */
