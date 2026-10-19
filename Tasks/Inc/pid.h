#ifndef PID_H
#define PID_H

#include <stdint.h>

/*
 * 位置式 PID:
 *   u(k) = Kp*e(k) + I(k) + Kd*d(-y)/dt
 *
 * 特点:
 *   - 微分作用在测量值上, 给定阶跃不会产生微分冲击
 *   - 微分项一阶低通滤波
 *   - 输出限幅 + 条件积分抗饱和
 *   - 切换模式时 PID_Reset() 同时重置积分和微分历史
 */

typedef struct
{
    float kp;
    float ki;
    float kd;

    float out_max;
    float out_min;

    float integral;
    float prev_meas;
    float d_state;

    float deriv_tau_s;
    uint8_t initialized;
} PID;

void  PID_Init(PID *pid, float kp, float ki, float kd,
               float out_max, float out_min, float deriv_tau_s);
void  PID_SetGains(PID *pid, float kp, float ki, float kd);
void  PID_SetOutputLimit(PID *pid, float out_max, float out_min);
void  PID_Reset(PID *pid, float meas);
float PID_Calc(PID *pid, float ref, float meas, float dt_s);

#endif /* PID_H */
