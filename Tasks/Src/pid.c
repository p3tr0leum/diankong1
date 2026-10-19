#include "pid.h"

#include <stddef.h>

void PID_Init(PID *pid, float kp, float ki, float kd,
              float out_max, float out_min, float deriv_tau_s)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_max = out_max;
    pid->out_min = out_min;
    pid->integral = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_state = 0.0f;
    pid->deriv_tau_s = deriv_tau_s;
    pid->initialized = 0U;
}

void PID_SetGains(PID *pid, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetOutputLimit(PID *pid, float out_max, float out_min)
{
    if (pid == NULL)
    {
        return;
    }
    pid->out_max = out_max;
    pid->out_min = out_min;
}

void PID_Reset(PID *pid, float meas)
{
    if (pid == NULL)
    {
        return;
    }
    pid->integral = 0.0f;
    pid->prev_meas = meas;
    pid->d_state = 0.0f;
    pid->initialized = 1U;
}

float PID_Calc(PID *pid, float ref, float meas, float dt_s)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    if (dt_s <= 0.0f)
    {
        dt_s = 0.001f;
    }
    if (pid->initialized == 0U)
    {
        PID_Reset(pid, meas);
    }

    const float err = ref - meas;

    /* 微分作用在测量值上: d(-y)/dt */
    const float d_meas = (meas - pid->prev_meas) / dt_s;
    float alpha = 1.0f;
    if (pid->deriv_tau_s > 0.0f)
    {
        alpha = dt_s / (pid->deriv_tau_s + dt_s);
    }
    pid->d_state += alpha * (-d_meas - pid->d_state);

    const float p_term = pid->kp * err;
    const float d_term = pid->kd * pid->d_state;

    float i_new = pid->integral + pid->ki * err * dt_s;
    float out = p_term + i_new + d_term;

    /* 条件积分: 输出饱和且误差继续把输出推向饱和方向时, 撤销本拍积分。 */
    if (out > pid->out_max)
    {
        out = pid->out_max;
        if (err > 0.0f)
        {
            i_new = pid->integral;
        }
    }
    else if (out < pid->out_min)
    {
        out = pid->out_min;
        if (err < 0.0f)
        {
            i_new = pid->integral;
        }
    }

    pid->integral = i_new;
    pid->prev_meas = meas;
    return out;
}
