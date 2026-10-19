#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * 所有可调参数集中在本文件。
 *
 * 任务:
 *   1) 串级控制: 位置外环输出速度给定, 速度内环输出电压; 上电即运行,
 *      没有模式切换, 位置和速度可以在 Ozone 里同时观察。
 *   2) IWDG 看门狗: CubeMX 中默认使能, 控制任务健康时才喂狗
 *
 * 注意:
 *   - 第一次上电请把电机空载固定好, 先确认转向和 CAN 反馈正常。
 *   - 如果出现高频抖动, 优先降低 APP_POS_KP / APP_SPEED_KP。
 */

#include <stdint.h>

#define APP_PI                         3.14159265358979323846f
#define APP_TWO_PI                     6.28318530717958647692f

/* ---------------- 控制周期 ---------------- */
#define APP_CONTROL_HZ                 1000U
#define APP_CONTROL_DT_S               0.001f

/* ---------------- CAN / 电机 ---------------- */
/* GM6020 拨码 ID, 1~7。默认 1, 对应反馈 0x205。 */
#define APP_MOTOR_ID                   1U
/* 本工程固定使用 C 板 CAN1: PD0=CAN1_RX, PD1=CAN1_TX, 1 Mbps。 */
#define APP_CAN_FILTER_BANK            0U
#define APP_FEEDBACK_TIMEOUT_MS        20U

/* ---------------- 位置正弦曲线 ---------------- */
/* 幅值 ±90 deg, 频率 0.2 Hz (5 s 一个周期)。 */
#define APP_POS_AMPLITUDE_RAD          (0.5f * APP_PI)
#define APP_POS_FREQ_HZ                0.20f
/* 上电/故障恢复时用 1.5 s 平滑建立振幅, 避免给定突变。 */
#define APP_SINE_RAMP_TIME_S           1.5f

/* ---------------- 速度正弦曲线 (速度模式用) ---------------- */
/* 手动切到速度模式时使用: 幅值 ±1 rev/s = ±2π rad/s, 0.25 Hz。 */
#define APP_SPEED_AMPLITUDE_RAD_S      (APP_TWO_PI)
#define APP_SPEED_FREQ_HZ              0.25f

/* 位置模式的串级: 位置环输出速度给定 + 位置曲线微分前馈。 */

/* ---------------- PID 参数 (串级) ---------------- */
/*
 * 控制结构: 位置环 P -> 速度给定(rad/s) -> 速度环 PID -> 电压给定
 *
 * 位置环:
 *   输入误差单位 rad, 输出单位 rad/s
 *   内环已经提供阻尼, 外环一般不需要 Kd
 * 速度环:
 *   输入误差单位 rad/s, 输出单位电压给定 (-25000~25000)
 * 两个环的微分都作用在测量值上, 避免给定突变造成微分冲击。
 */
#define APP_POS_KP                     8.0f
#define APP_POS_KI                     0.0f
#define APP_POS_KD                     0.0f
/* 位置曲线速度前馈: 直接叠加到速度给定, 1.0 = 完全前馈, 0 = 不用前馈。 */
#define APP_POS_VEL_FF                 1.0f
/* 位置环输出(速度给定)限幅, rad/s。6 rad/s 约 0.95 rev/s。 */
#define APP_POS_SPEED_LIMIT_RAD_S      6.0f

/* 速度环输出单位: 电压给定; 速度误差单位: rad/s
 * 起步值故意偏保守: 先保证内环不振荡, 再逐步加 Kp, 最后加一点 Ki。 */
#define APP_SPEED_KP                   300.0f
#define APP_SPEED_KI                   60.0f
#define APP_SPEED_KD                   0.0f
/*
 * 反电动势前馈: 电压给定 / (rad/s), 约等于电机的反电动势常数 Ke。
 * 只用 P 时, 要产生抵消反电动势的电压就必须先有速度误差, 表现为峰值跟不上;
 * 加上前馈后 P 只负责修正误差, 因此可以用较小的 Kp 而不失稳。
 * 24 V 对应 25000; 空载约 320 rpm (33.5 rad/s) 时约 750。
 * 峰值跟不上就加大, 峰值过冲就减小。
 */
#define APP_SPEED_FF                   800.0f

/* PID 微分低通滤波时间常数, 0 表示不滤波。 */
#define APP_PID_DERIV_TAU_S            0.008f

/* ---------------- 输出保护 ---------------- */
/* 电压给定限幅, 调试时不要一上来给满 25000。 */
#define APP_OUTPUT_LIMIT               12000.0f
/* 电压变化率限幅: 250000/25000 = 10 ms 走完全量程。 */
#define APP_OUTPUT_SLEW_PER_S          250000.0f

/* GM6020 反馈温度保护 (degC) */
#define APP_TEMP_LIMIT_C               75.0f
#define APP_TEMP_RECOVER_C             60.0f
/* 原始转矩电流保护: 反馈为 int16, 约 ±16384 对应额定范围附近。 */
#define APP_CURRENT_LIMIT_RAW          15000
#define APP_CURRENT_LIMIT_MS           100U

/* ---------------- 运行方式 ---------------- */
/*
 * 用 Ozone 写全局变量 g_cmd_mode 切换:
 *   0 = 停止 (输出 0)
 *   1 = 位置模式: 位置外环 + 速度内环串级 (默认)
 *   2 = 速度模式: 只跑速度环, 跟踪 APP_SPEED_AMPLITUDE_RAD_S / APP_SPEED_FREQ_HZ
 *       这条固定速度正弦, 用来单独整定速度 PID。
 * 切换时程序会复位两个 PID 并按新模式重建曲线。
 * 故障或反馈丢失时强制停止 (g_dbg_mode = 0)。
 */

/* ---------------- 看门狗 ---------------- */
/*
 * IWDG 在 CubeMX (.ioc) 中已使能:
 *   Prescaler = 64, Reload = 400
 *   LSI 约 32 kHz -> 超时约 (400+1)*64/32000 = 0.802 s
 *
 * 只有控制任务健康时才喂狗。反馈丢失超过 APP_WDG_GRACE_MS 后停止喂狗,
 * 约 0.8 s 后 MCU 复位。g_wdg_test_inhibit 置 1 可立即停止喂狗用于验证。
 */
#define APP_WATCHDOG_ENABLE            1U
#define APP_WDG_GRACE_MS               300U
/* Ozone 暂停内核时冻结 IWDG, 方便调试; 运行态看门狗仍然有效。 */
#define APP_IWDG_FREEZE_ON_DEBUG       1U
/*
 * 注意: 若把 APP_WATCHDOG_ENABLE 改成 0, 必须同时在 CubeMX 里关闭 IWDG,
 * 否则 IWDG 仍会启动, 而代码不再喂狗, MCU 会持续复位。
 */

/* ---------------- 主循环心跳 ---------------- */
/* 主循环超过该时间没有运行, 视为严重故障。 */
#define APP_MAIN_LOOP_TIMEOUT_MS       100U

#endif /* APP_CONFIG_H */
