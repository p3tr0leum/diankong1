#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>

/* 控制模式 (g_cmd_mode) */
typedef enum
{
    APP_MODE_STOP = 0,
    APP_MODE_POSITION = 1,   /* 位置外环 + 速度内环(串级) */
    APP_MODE_SPEED = 2       /* 只跑速度环, 跟踪固定速度正弦 */
} AppMode;

/* 故障位 */
#define APP_FAULT_FEEDBACK_TIMEOUT   (1UL << 0)
#define APP_FAULT_OVERTEMP           (1UL << 1)
#define APP_FAULT_OVERCURRENT        (1UL << 2)
#define APP_FAULT_MAIN_LOOP          (1UL << 3)

/* 复位原因位 (g_boot_reset_cause) */
#define APP_RESET_IWDG               (1UL << 0)
#define APP_RESET_WWDG               (1UL << 1)
#define APP_RESET_SOFT               (1UL << 2)
#define APP_RESET_PIN                (1UL << 3)
#define APP_RESET_POR                (1UL << 4)

/* 上电初始化: 在 main.c 所有 MX_xxx_Init() 之后调用一次。 */
void AppControl_Init(void);

/* 主循环心跳, 在 while(1) 中反复调用。 */
void AppControl_MainLoop(void);

/* 1 kHz 控制任务, 由 TIM6 更新中断调用。 */
void AppControl_1kHz(void);

/* 给 Ozone / 调试器观察的全局变量。 */
extern volatile uint32_t g_control_tick;
extern volatile uint32_t g_fault_flags;
extern volatile uint32_t g_boot_reset_cause;
extern volatile uint32_t g_main_loop_counter;
extern volatile uint32_t g_wdg_feed_count;
extern volatile uint32_t g_dbg_can_rx_count;
extern volatile uint32_t g_dbg_can_tx_count;
extern volatile uint32_t g_dbg_can_tx_fail;
extern volatile uint8_t  g_dbg_mode;
extern volatile uint8_t  g_cmd_mode;
extern volatile uint8_t  g_wdg_test_inhibit;
extern volatile float    g_dbg_ref;
extern volatile float    g_dbg_meas;
extern volatile float    g_dbg_error;
extern volatile float    g_dbg_angle;
extern volatile float    g_dbg_speed;
extern volatile float    g_dbg_speed_ref;
extern volatile float    g_dbg_speed_error;
extern volatile float    g_dbg_temp;
extern volatile int16_t  g_dbg_voltage;

#endif /* APP_CONTROL_H */
