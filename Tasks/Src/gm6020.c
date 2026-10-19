#include "gm6020.h"

#include <stddef.h>

#include "app_config.h"

#define GM6020_ANGLE_PER_REV   8192U
#define GM6020_ANGLE_TO_RAD    (APP_TWO_PI / (float)GM6020_ANGLE_PER_REV)
#define GM6020_RPM_TO_RAD_S    (APP_TWO_PI / 60.0f)
#define GM6020_RAW_TO_AMP      (3.0f / 16384.0f)

static CAN_HandleTypeDef *s_hcan = NULL;
static uint8_t s_motor_id = 0U;

/* 接收中断写, 控制中断读; 用 PRIMASK 临界区保护多字段快照。 */
static volatile GM6020_State s_state;
static int32_t s_full_turns = 0;
static uint16_t s_last_raw_angle = 0U;
static float s_zero_offset_rad = 0.0f;
static uint8_t s_zeroed = 0U;

static volatile uint32_t s_rx_count = 0U;
static volatile uint32_t s_tx_count = 0U;
static volatile uint32_t s_tx_fail_count = 0U;

static void gm6020_enter_critical(uint32_t *primask)
{
    *primask = __get_PRIMASK();
    __disable_irq();
}

static void gm6020_exit_critical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

uint8_t GM6020_Init(CAN_HandleTypeDef *hcan, uint8_t motor_id)
{
    if ((hcan == NULL) || (hcan->Instance != CAN1))
    {
        return 0U;
    }
    if ((motor_id < 1U) || (motor_id > 7U))
    {
        return 0U;
    }

    s_hcan = hcan;
    s_motor_id = motor_id;

    /* 接受 0x200~0x20F, 软件再判断 0x205~0x20B 和具体 ID。 */
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = APP_CAN_FILTER_BANK;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (uint16_t)(0x200U << 5);
    filter.FilterMaskIdHigh = (uint16_t)(0x7F0U << 5);
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(s_hcan, &filter) != HAL_OK)
    {
        return 0U;
    }
    if (HAL_CAN_Start(s_hcan) != HAL_OK)
    {
        return 0U;
    }
    if (HAL_CAN_ActivateNotification(s_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

void GM6020_OnCanRx(CAN_HandleTypeDef *hcan, uint32_t now_ms)
{
    if ((s_hcan == NULL) || (hcan != s_hcan))
    {
        return;
    }

    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(s_hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(s_hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }
        if ((header.IDE != CAN_ID_STD) || (header.RTR != CAN_RTR_DATA) ||
            (header.DLC != 8U))
        {
            continue;
        }
        if ((header.StdId < 0x205U) || (header.StdId > 0x20BU))
        {
            continue;
        }
        if ((uint8_t)(header.StdId - 0x204U) != s_motor_id)
        {
            continue;
        }

        const uint16_t raw_angle =
            (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
        const int16_t raw_speed =
            (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
        const int16_t raw_current =
            (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
        const uint8_t raw_temp = data[6];

        uint32_t primask;
        gm6020_enter_critical(&primask);

        if (s_zeroed != 0U)
        {
            const int32_t diff = (int32_t)raw_angle - (int32_t)s_last_raw_angle;
            if (diff > (int32_t)(GM6020_ANGLE_PER_REV / 2U))
            {
                /* 0 -> 8191: 反向跨零, 累计圈数 -1 */
                s_full_turns--;
            }
            else if (diff < -(int32_t)(GM6020_ANGLE_PER_REV / 2U))
            {
                /* 8191 -> 0: 正向跨零, 累计圈数 +1 */
                s_full_turns++;
            }
        }

        const float raw_rad = (float)raw_angle * GM6020_ANGLE_TO_RAD;
        float angle_rad = (float)s_full_turns * APP_TWO_PI + raw_rad;

        if (s_zeroed == 0U)
        {
            s_zero_offset_rad = angle_rad;
            s_zeroed = 1U;
        }
        angle_rad -= s_zero_offset_rad;

        s_last_raw_angle = raw_angle;
        s_state.angle_rad = angle_rad;
        s_state.speed_rad_s = (float)raw_speed * GM6020_RPM_TO_RAD_S;
        s_state.current_a = (float)raw_current * GM6020_RAW_TO_AMP;
        s_state.temperature_c = (float)raw_temp;
        s_state.raw_speed_rpm = raw_speed;
        s_state.raw_current = raw_current;
        s_state.raw_angle = raw_angle;
        s_state.timestamp_ms = now_ms;
        s_state.online = 1U;
        s_rx_count++;

        gm6020_exit_critical(primask);
    }
}

uint8_t GM6020_GetState(GM6020_State *out)
{
    if (out == NULL)
    {
        return 0U;
    }

    uint32_t primask;
    gm6020_enter_critical(&primask);
    out->angle_rad = s_state.angle_rad;
    out->speed_rad_s = s_state.speed_rad_s;
    out->current_a = s_state.current_a;
    out->temperature_c = s_state.temperature_c;
    out->raw_speed_rpm = s_state.raw_speed_rpm;
    out->raw_current = s_state.raw_current;
    out->raw_angle = s_state.raw_angle;
    out->timestamp_ms = s_state.timestamp_ms;
    out->online = s_state.online;
    gm6020_exit_critical(primask);

    return out->online;
}

uint8_t GM6020_SendVoltage(int16_t voltage)
{
    if (s_hcan == NULL)
    {
        return 0U;
    }

    if (voltage > 25000)
    {
        voltage = 25000;
    }
    else if (voltage < -25000)
    {
        voltage = -25000;
    }

    uint8_t data[8] = {0};
    const uint8_t index = (s_motor_id <= 4U)
                              ? (uint8_t)((s_motor_id - 1U) * 2U)
                              : (uint8_t)((s_motor_id - 5U) * 2U);
    data[index] = (uint8_t)(((uint16_t)voltage >> 8) & 0xFFU);
    data[index + 1U] = (uint8_t)((uint16_t)voltage & 0xFFU);

    CAN_TxHeaderTypeDef header = {0};
    header.StdId = (s_motor_id <= 4U) ? 0x1FFU : 0x2FFU;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0U;
    if (HAL_CAN_AddTxMessage(s_hcan, &header, data, &mailbox) != HAL_OK)
    {
        /*
         * 总线异常/无 ACK 时, 自动重传可能把 3 个邮箱一直占住。
         * 这里主动撤销挂起帧, 下个 1 ms 再发新给定, 避免输出长时间卡在旧值。
         */
        (void)HAL_CAN_AbortTxRequest(s_hcan,
                                     CAN_TX_MAILBOX0 |
                                     CAN_TX_MAILBOX1 |
                                     CAN_TX_MAILBOX2);
        s_tx_fail_count++;
        return 0U;
    }

    s_tx_count++;
    return 1U;
}

uint32_t GM6020_GetRxCount(void)
{
    return s_rx_count;
}

uint32_t GM6020_GetTxCount(void)
{
    return s_tx_count;
}

uint32_t GM6020_GetTxFailCount(void)
{
    return s_tx_fail_count;
}
