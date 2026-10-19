#ifndef SINE_PROFILE_H
#define SINE_PROFILE_H

/*
 * 正弦给定发生器:
 *   y(t)  = center + A * env(t) * sin(omega*t)
 *   y'(t) = A * (env'(t)*sin(omega*t) + env(t)*omega*cos(omega*t))
 *
 * env(t) 是 smoothstep 启动包络, 在 ramp_time 内从 0 平滑到 1,
 * 用来避免上电/切模式时给定和速度前馈突变。
 */

typedef struct
{
    float center;
    float amplitude;
    float omega;
    float ramp_time_s;
    float t_s;
} SineProfile;

void SineProfile_Init(SineProfile *p, float amplitude, float freq_hz,
                      float ramp_time_s);
void SineProfile_Start(SineProfile *p, float center);
void SineProfile_Update(SineProfile *p, float dt_s);
void SineProfile_Eval(const SineProfile *p, float *value, float *derivative);

#endif /* SINE_PROFILE_H */
