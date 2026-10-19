#include "sine_profile.h"

#include <math.h>
#include <stddef.h>

#include "app_config.h"

void SineProfile_Init(SineProfile *p, float amplitude, float freq_hz,
                      float ramp_time_s)
{
    if (p == NULL)
    {
        return;
    }
    p->center = 0.0f;
    p->amplitude = amplitude;
    p->omega = APP_TWO_PI * freq_hz;
    p->ramp_time_s = (ramp_time_s > 0.0f) ? ramp_time_s : 0.001f;
    p->t_s = 0.0f;
}

void SineProfile_Start(SineProfile *p, float center)
{
    if (p == NULL)
    {
        return;
    }
    p->center = center;
    p->t_s = 0.0f;
}

void SineProfile_Update(SineProfile *p, float dt_s)
{
    if (p == NULL)
    {
        return;
    }
    p->t_s += dt_s;
}

void SineProfile_Eval(const SineProfile *p, float *value, float *derivative)
{
    if (p == NULL)
    {
        if (value != NULL)
        {
            *value = 0.0f;
        }
        if (derivative != NULL)
        {
            *derivative = 0.0f;
        }
        return;
    }

    const float t = p->t_s;
    const float wt = p->omega * t;

    float env = 1.0f;
    float denv = 0.0f;

    if (t < p->ramp_time_s)
    {
        const float r = t / p->ramp_time_s;
        /* smoothstep: 3r^2 - 2r^3 */
        env = r * r * (3.0f - 2.0f * r);
        denv = 6.0f * r * (1.0f - r) / p->ramp_time_s;
    }

    const float s = sinf(wt);
    const float c = cosf(wt);

    if (value != NULL)
    {
        *value = p->center + p->amplitude * env * s;
    }
    if (derivative != NULL)
    {
        *derivative = p->amplitude * (denv * s + env * p->omega * c);
    }
}
