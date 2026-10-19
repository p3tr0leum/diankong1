#include "app_control.h"

#include <math.h>
#include <stddef.h>

#include "app_config.h"
#include "can.h"
#include "gm6020.h"
#include "iwdg.h"
#include "main.h"
#include "pid.h"
#include "sine_profile.h"
#include "tim.h"

/* -------------------------------------------------------------------------- */
/* Ozone 可观察变量                                                           */
/* -------------------------------------------------------------------------- */
volatile uint32_t g_control_tick = 0U;
volatile uint32_t g_fault_flags = 0U;
volatile uint32_t g_boot_reset_cause = 0U;
volatile uint32_t g_main_loop_counter = 0U;
volatile uint32_t g_wdg_feed_count = 0U;

volatile uint32_t g_dbg_can_rx_count = 0U;
volatile uint32_t g_dbg_can_tx_count = 0U;
volatile uint32_t g_dbg_can_tx_fail = 0U;

/* 0 = 停止, 1 = 位置(串级), 2 = 速度(固定正弦)。 */
volatile uint8_t g_dbg_mode = 0U;
/* 期望模式, 可在 Ozone 里直接改写: 0 停止, 1 位置, 2 速度。 */
volatile uint8_t g_cmd_mode = 1U;
volatile uint8_t g_wdg_test_inhibit = 0U;

volatile float g_dbg_ref = 0.0f;
volatile float g_dbg_meas = 0.0f;
volatile float g_dbg_error = 0.0f;
volatile float g_dbg_angle = 0.0f;
volatile float g_dbg_speed = 0.0f;
volatile float g_dbg_speed_ref = 0.0f;
volatile float g_dbg_speed_error = 0.0f;
volatile float g_dbg_temp = 0.0f;
volatile int16_t g_dbg_voltage = 0;

/* -------------------------------------------------------------------------- */
/* 内部状态                                                                   */
/* -------------------------------------------------------------------------- */
static PID s_pos_pid;
static PID s_speed_pid;

static SineProfile s_pos_profile;
static SineProfile s_speed_profile;

static AppMode s_mode = APP_MODE_STOP;
static float s_voltage_cmd = 0.0f;

static uint32_t s_critical_since_ms = 0U;
static uint32_t s_overcurrent_since_ms = 0U;
static uint32_t s_overtemp_latched = 0U;
static uint32_t s_overcurrent_latched = 0U;

static uint32_t s_last_main_loop_counter = 0U;
static uint32_t s_main_loop_check_ms = 0U;

/* -------------------------------------------------------------------------- */
/* 小工具                                                                     */
/* -------------------------------------------------------------------------- */
static uint32_t app_read_reset_cause(void)
{
    uint32_t cause = 0U;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
    {
        cause |= APP_RESET_IWDG;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
    {
        cause |= APP_RESET_WWDG;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
    {
        cause |= APP_RESET_SOFT;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
    {
        cause |= APP_RESET_PIN;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
    {
        cause |= APP_RESET_POR;
    }

    __HAL_RCC_CLEAR_RESET_FLAGS();
    return cause;
}

/* 把 Ozone 里写的模式值限制到合法范围。 */
static AppMode app_clamp_mode(uint8_t raw)
{
    if (raw == (uint8_t)APP_MODE_POSITION)
    {
        return APP_MODE_POSITION;
    }
    if (raw == (uint8_t)APP_MODE_SPEED)
    {
        return APP_MODE_SPEED;
    }
    return APP_MODE_STOP;
}

/* 切换模式: 清两个环的历史, 重建对应曲线, 避免给定跳变。 */
static void app_enter_mode(AppMode mode, const GM6020_State *st)
{
    PID_Reset(&s_pos_pid, st->angle_rad);
    PID_Reset(&s_speed_pid, st->speed_rad_s);
    s_voltage_cmd = 0.0f;

    if ((mode != APP_MODE_STOP) && (st->online == 0U))
    {
        mode = APP_MODE_STOP;
    }

    if (mode == APP_MODE_POSITION)
    {
        /* 以当前角度为中心, 位置给定从当前角度平滑开始。 */
        SineProfile_Start(&s_pos_profile, st->angle_rad);
    }
    else if (mode == APP_MODE_SPEED)
    {
        /* 速度给定从 0 开始, 1.5 s 包络平滑建立。 */
        SineProfile_Start(&s_speed_profile, 0.0f);
    }

    s_mode = mode;
}

static void app_update_mode(const GM6020_State *st)
{
    const AppMode desired = app_clamp_mode(g_cmd_mode);
    if (desired != s_mode)
    {
        app_enter_mode(desired, st);
    }
}

static float app_limit_voltage(float v)
{
    if (v > APP_OUTPUT_LIMIT)
    {
        return APP_OUTPUT_LIMIT;
    }
    if (v < -APP_OUTPUT_LIMIT)
    {
        return -APP_OUTPUT_LIMIT;
    }
    return v;
}

static float app_slew_voltage(float target, float dt_s)
{
    const float max_delta = APP_OUTPUT_SLEW_PER_S * dt_s;
    float delta = target - s_voltage_cmd;

    if (delta > max_delta)
    {
        delta = max_delta;
    }
    else if (delta < -max_delta)
    {
        delta = -max_delta;
    }

    s_voltage_cmd += delta;
    return app_limit_voltage(s_voltage_cmd);
}

static void app_watchdog_service(uint32_t now_ms, uint8_t critical)
{
#if (APP_WATCHDOG_ENABLE != 0U)
    if (critical != 0U)
    {
        if (s_critical_since_ms == 0U)
        {
            s_critical_since_ms = now_ms;
        }
    }
    else
    {
        s_critical_since_ms = 0U;
    }

    if (g_wdg_test_inhibit != 0U)
    {
        return;
    }

    /*
     * 正常时每 1 ms 喂狗。
     * 严重故障时先给 APP_WDG_GRACE_MS 的恢复窗口, 之后停止喂狗。
     * IWDG 超时约 0.8 s, 所以故障后约 1.1 s 复位。
     */
    if ((critical == 0U) ||
        ((s_critical_since_ms != 0U) &&
         ((now_ms - s_critical_since_ms) < APP_WDG_GRACE_MS)))
    {
        if (HAL_IWDG_Refresh(&hiwdg) == HAL_OK)
        {
            g_wdg_feed_count++;
        }
    }
#else
    (void)now_ms;
    (void)critical;
#endif
}

/* -------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* -------------------------------------------------------------------------- */
void AppControl_Init(void)
{
    g_boot_reset_cause = app_read_reset_cause();

#if (APP_IWDG_FREEZE_ON_DEBUG != 0U)
    /* 调试器暂停内核时冻结 IWDG; 运行态看门狗仍然计数。 */
    __HAL_DBGMCU_FREEZE_IWDG();
#endif

    (void)GM6020_Init(&hcan1, (uint8_t)APP_MOTOR_ID);

    PID_Init(&s_pos_pid,
             APP_POS_KP, APP_POS_KI, APP_POS_KD,
             APP_POS_SPEED_LIMIT_RAD_S, -APP_POS_SPEED_LIMIT_RAD_S,
             APP_PID_DERIV_TAU_S);
    PID_Init(&s_speed_pid,
             APP_SPEED_KP, APP_SPEED_KI, APP_SPEED_KD,
             APP_OUTPUT_LIMIT, -APP_OUTPUT_LIMIT,
             APP_PID_DERIV_TAU_S);

    SineProfile_Init(&s_pos_profile,
                     APP_POS_AMPLITUDE_RAD,
                     APP_POS_FREQ_HZ,
                     APP_SINE_RAMP_TIME_S);
    SineProfile_Init(&s_speed_profile,
                     APP_SPEED_AMPLITUDE_RAD_S,
                     APP_SPEED_FREQ_HZ,
                     APP_SINE_RAMP_TIME_S);

    s_mode = APP_MODE_STOP;
    s_voltage_cmd = 0.0f;
    s_critical_since_ms = 0U;
    s_overcurrent_since_ms = 0U;
    s_overtemp_latched = 0U;
    s_overcurrent_latched = 0U;
    s_last_main_loop_counter = g_main_loop_counter;
    s_main_loop_check_ms = 0U;

    /* 初始化期间先喂一次, 防止外设初始化耗时导致复位。 */
    (void)HAL_IWDG_Refresh(&hiwdg);

    /* 1 kHz 控制节拍: TIM6, 84 MHz / 84 / 1000 = 1 kHz。 */
    (void)HAL_TIM_Base_Start_IT(&htim6);
}

void AppControl_MainLoop(void)
{
    g_main_loop_counter++;
}

/* -------------------------------------------------------------------------- */
/* 1 kHz 控制任务                                                             */
/* -------------------------------------------------------------------------- */
void AppControl_1kHz(void)
{
    g_control_tick++;
    const uint32_t now_ms = g_control_tick;

    GM6020_State st;
    (void)GM6020_GetState(&st);

    uint32_t faults = 0U;

    /* 1) 反馈超时 */
    if ((st.online == 0U) ||
        ((now_ms - st.timestamp_ms) > APP_FEEDBACK_TIMEOUT_MS))
    {
        faults |= APP_FAULT_FEEDBACK_TIMEOUT;
    }

    /* 2) 温度保护: 超过上限锁存, 降到恢复温度以下自动解除 */
    if (st.online != 0U)
    {
        if (st.temperature_c >= APP_TEMP_LIMIT_C)
        {
            s_overtemp_latched = 1U;
        }
        else if (st.temperature_c <= APP_TEMP_RECOVER_C)
        {
            s_overtemp_latched = 0U;
        }
    }
    if (s_overtemp_latched != 0U)
    {
        faults |= APP_FAULT_OVERTEMP;
    }

    /* 3) 转矩电流保护: 连续超过 APP_CURRENT_LIMIT_MS 才锁存 */
    if (st.online != 0U)
    {
        const int32_t current = (st.raw_current < 0)
                                    ? -(int32_t)st.raw_current
                                    : (int32_t)st.raw_current;
        if (current >= APP_CURRENT_LIMIT_RAW)
        {
            if (s_overcurrent_since_ms == 0U)
            {
                s_overcurrent_since_ms = now_ms;
            }
            else if ((now_ms - s_overcurrent_since_ms) >= APP_CURRENT_LIMIT_MS)
            {
                s_overcurrent_latched = 1U;
            }
        }
        else
        {
            s_overcurrent_since_ms = 0U;
        }
    }
    if (s_overcurrent_latched != 0U)
    {
        faults |= APP_FAULT_OVERCURRENT;
    }

    /* 4) 主循环心跳: 每 100 ms 检查一次 */
    if ((now_ms - s_main_loop_check_ms) >= APP_MAIN_LOOP_TIMEOUT_MS)
    {
        if (g_main_loop_counter == s_last_main_loop_counter)
        {
            faults |= APP_FAULT_MAIN_LOOP;
        }
        s_last_main_loop_counter = g_main_loop_counter;
        s_main_loop_check_ms = now_ms;
    }

    g_fault_flags = faults;

    /* 有故障时立即停止, 输出 0, 不允许继续跟踪曲线。 */
    if (faults != 0U)
    {
        if (s_mode != APP_MODE_STOP)
        {
            app_enter_mode(APP_MODE_STOP, &st);
        }
        s_voltage_cmd = 0.0f;
        g_dbg_voltage = 0;
        (void)GM6020_SendVoltage(0);
        g_dbg_mode = 0U;
        g_dbg_angle = st.angle_rad;
        g_dbg_speed = st.speed_rad_s;
        g_dbg_speed_ref = 0.0f;
        g_dbg_speed_error = 0.0f;
        g_dbg_temp = st.temperature_c;
        g_dbg_ref = 0.0f;
        g_dbg_meas = 0.0f;
        g_dbg_error = 0.0f;
        g_dbg_can_rx_count = GM6020_GetRxCount();
        g_dbg_can_tx_count = GM6020_GetTxCount();
        g_dbg_can_tx_fail = GM6020_GetTxFailCount();
        app_watchdog_service(now_ms, (uint8_t)((faults & (APP_FAULT_FEEDBACK_TIMEOUT |
                                                          APP_FAULT_MAIN_LOOP)) != 0U));
        return;
    }

    app_update_mode(&st);

    float speed_ref = 0.0f;
    float ref = 0.0f;
    float meas = 0.0f;
    float voltage = 0.0f;

    if (s_mode == APP_MODE_POSITION)
    {
        float pos_ref;
        float pos_vel_ref;
        SineProfile_Update(&s_pos_profile, APP_CONTROL_DT_S);
        SineProfile_Eval(&s_pos_profile, &pos_ref, &pos_vel_ref);

        /* 外环: 位置 -> 速度给定, 再叠加曲线微分前馈。 */
        speed_ref = PID_Calc(&s_pos_pid, pos_ref, st.angle_rad,
                             APP_CONTROL_DT_S);
        speed_ref += APP_POS_VEL_FF * pos_vel_ref;

        ref = pos_ref;
        meas = st.angle_rad;
    }
    else if (s_mode == APP_MODE_SPEED)
    {
        float speed_acc_ref;
        SineProfile_Update(&s_speed_profile, APP_CONTROL_DT_S);
        SineProfile_Eval(&s_speed_profile, &speed_ref, &speed_acc_ref);

        ref = speed_ref;
        meas = st.speed_rad_s;
    }
    else
    {
        voltage = 0.0f;
        s_voltage_cmd = 0.0f;
    }

    if (s_mode != APP_MODE_STOP)
    {
        /* 限幅只作用于位置环输出; 速度模式的给定由曲线自身限幅。 */
        if (s_mode == APP_MODE_POSITION)
        {
            if (speed_ref > APP_POS_SPEED_LIMIT_RAD_S)
            {
                speed_ref = APP_POS_SPEED_LIMIT_RAD_S;
            }
            else if (speed_ref < -APP_POS_SPEED_LIMIT_RAD_S)
            {
                speed_ref = -APP_POS_SPEED_LIMIT_RAD_S;
            }
        }

        /* 内环: 速度 -> 电压给定; 反电动势前馈 + PID 修正。 */
        voltage = APP_SPEED_FF * speed_ref +
                  PID_Calc(&s_speed_pid, speed_ref, st.speed_rad_s,
                           APP_CONTROL_DT_S);
    }

    voltage = app_limit_voltage(voltage);
    voltage = app_slew_voltage(voltage, APP_CONTROL_DT_S);

    const int16_t output = (int16_t)voltage;
    (void)GM6020_SendVoltage(output);

    /* 调试变量 */
    g_dbg_mode = (uint8_t)s_mode;
    g_dbg_ref = ref;
    g_dbg_meas = meas;
    g_dbg_error = ref - meas;
    g_dbg_angle = st.angle_rad;
    g_dbg_speed = st.speed_rad_s;
    g_dbg_speed_ref = speed_ref;
    g_dbg_speed_error = speed_ref - st.speed_rad_s;
    g_dbg_temp = st.temperature_c;
    g_dbg_voltage = output;
    g_dbg_can_rx_count = GM6020_GetRxCount();
    g_dbg_can_tx_count = GM6020_GetTxCount();
    g_dbg_can_tx_fail = GM6020_GetTxFailCount();

    app_watchdog_service(now_ms, 0U);
}

/* -------------------------------------------------------------------------- */
/* HAL 回调                                                                   */
/* -------------------------------------------------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        AppControl_1kHz();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        GM6020_OnCanRx(hcan, g_control_tick);
    }
}
