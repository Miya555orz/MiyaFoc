#include "foc_can.h"

#include "Open_Loop.h"
#include "PID.h"
#include "Paremeter.h"
#include "can.h"
#include "usart.h"
#include <string.h>

#define FOC_TWO_PI 6.2831853071795864769f

typedef struct {
    uint32_t prescaler;
    uint32_t sjw;
    uint32_t bs1;
    uint32_t bs2;
} FocCanTimingProfile_t;

typedef struct {
    volatile uint8_t pending;
    volatile uint8_t mode;
    volatile float value;
    volatile uint32_t last_rx_ms;
} FocCanPendingCommand_t;

static FocCanPendingCommand_t pending_command;
static FocCanStatus_t status;
static uint32_t last_feedback_ms;
static uint32_t boot_test_count;
static uint8_t active_timing_profile;
static uint8_t active_loopback_mode;

static const FocCanTimingProfile_t timing_profiles[] = {
    {2U, CAN_SJW_4TQ, CAN_BS1_12TQ, CAN_BS2_5TQ}, /* 36 MHz CAN clock: 1 Mbps, sample 72.2%, H7-like test */
    {2U, CAN_SJW_4TQ, CAN_BS1_13TQ, CAN_BS2_4TQ}, /* 36 MHz CAN clock: 1 Mbps, sample 77.8%, H7-like test */
    {2U, CAN_SJW_2TQ, CAN_BS1_15TQ, CAN_BS2_2TQ}, /* 36 MHz CAN clock: 1 Mbps, sample 88.9% */
    {3U, CAN_SJW_1TQ, CAN_BS1_9TQ,  CAN_BS2_2TQ}, /* 36 MHz CAN clock: 1 Mbps, sample 83.3% */
    {3U, CAN_SJW_1TQ, CAN_BS1_8TQ,  CAN_BS2_3TQ}, /* 36 MHz CAN clock: 1 Mbps, sample 75.0% */
    {4U, CAN_SJW_1TQ, CAN_BS1_6TQ,  CAN_BS2_2TQ}, /* 36 MHz CAN clock: 1 Mbps, sample 77.8% */
    {6U, CAN_SJW_1TQ, CAN_BS1_3TQ,  CAN_BS2_2TQ}, /* 36 MHz CAN clock: 1 Mbps, sample 66.7% */
    {4U, CAN_SJW_2TQ, CAN_BS1_15TQ, CAN_BS2_2TQ}, /* 72 MHz actual CAN clock fallback */
    {6U, CAN_SJW_1TQ, CAN_BS1_9TQ,  CAN_BS2_2TQ}, /* 72 MHz actual CAN clock fallback */
    {8U, CAN_SJW_1TQ, CAN_BS1_6TQ,  CAN_BS2_2TQ}, /* 72 MHz actual CAN clock fallback */
};

static uint8_t checksum8(const uint8_t *data, uint8_t length)
{
    uint8_t sum = 0U;

    for (uint8_t i = 0U; i < length; ++i) {
        sum = (uint8_t)(sum + data[i]);
    }
    return (uint8_t)(~sum);
}

static uint8_t float_is_valid(float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    return ((bits & 0x7F800000U) != 0x7F800000U) ? 1U : 0U;
}

static float clampf(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static void reset_pid(PID_Handle *pid)
{
    pid->Current_Error = 0.0f;
    pid->Last_Error = 0.0f;
    pid->Error_Sum = 0.0f;
    pid->Integral = 0.0f;
    pid->Differential = 0.0f;
    pid->OutPut = 0.0f;
}

static void stop_motor(void)
{
    CMD.CMD_Type = CMD_NONE;
    CMD.Target = 0.0f;
    reset_pid(&PID_Speed);
    reset_pid(&PID_Torque_d);
    reset_pid(&PID_Torque_q);
    reset_pid(&PID_Position);
    PWM_Set_Compare(0.0f, 0.0f, 0.0f);
}

static HAL_StatusTypeDef apply_timing_profile(uint8_t profile)
{
    const FocCanTimingProfile_t *timing;

    if (profile >= (uint8_t)(sizeof(timing_profiles) / sizeof(timing_profiles[0]))) {
        return HAL_ERROR;
    }

    timing = &timing_profiles[profile];
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = timing->prescaler;
    hcan.Init.Mode = (active_loopback_mode != 0U) ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = timing->sjw;
    hcan.Init.TimeSeg1 = timing->bs1;
    hcan.Init.TimeSeg2 = timing->bs2;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = ENABLE;

    active_timing_profile = profile;
    return HAL_CAN_Init(&hcan);
}

static void abort_pending_tx(void)
{
    if (HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) == HAL_OK) {
        status.tx_abort_count++;
    }
}

static void apply_command(uint8_t mode, float value)
{
    status.last_mode = mode;

    switch ((FocCanMode_t)mode) {
    case FOC_CAN_MODE_CURRENT:
        CMD.CMD_Type = CMD_TORQUE;
        CMD.Target = value;
        break;

    case FOC_CAN_MODE_SPEED:
        CMD.CMD_Type = CMD_SPEED;
        /* Keep CAN speed command in the same unit as the original UART SPEED:x command. */
        CMD.Target = clampf(value, FOC_CAN_MAX_SPEED_CMD);
        break;

    case FOC_CAN_MODE_POSITION:
        CMD.CMD_Type = CMD_POSITION;
        CMD.Target = value;
        break;

    case FOC_CAN_MODE_STOP:
    default:
        stop_motor();
        break;
    }
}

HAL_StatusTypeDef FocCan_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef result;

    memset(&pending_command, 0, sizeof(pending_command));
    memset(&status, 0, sizeof(status));
    status.timing_profile = active_timing_profile;
    status.loopback_mode = active_loopback_mode;
    stop_motor();

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    result = HAL_CAN_ConfigFilter(&hcan, &filter);
    if (result != HAL_OK) {
        return result;
    }
    result = HAL_CAN_Start(&hcan);
    if (result != HAL_OK) {
        return result;
    }
    return HAL_CAN_ActivateNotification(&hcan,
                                        CAN_IT_RX_FIFO0_MSG_PENDING |
                                        CAN_IT_BUSOFF |
                                        CAN_IT_ERROR);
}

void FocCan_Process(void)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8];
    uint32_t mailbox;
    uint32_t now = HAL_GetTick();
#if FOC_CAN_BOOT_TEST_ENABLE
    uint32_t value;
#else
    float speed_rps;
    float position_rad;
#endif

#if FOC_CAN_BOOT_TEST_ENABLE
    if ((now - last_feedback_ms) < FOC_CAN_BOOT_TEST_PERIOD_MS) {
        return;
    }
    last_feedback_ms = now;

    value = boot_test_count++;
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
    data[4] = (uint8_t)(0xC1U);
    data[5] = 0x55U;
    data[6] = 0xAAU;
    data[7] = (uint8_t)(data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6]);

    header.StdId = FOC_CAN_BOOT_TEST_ID;
#else
    if ((now - last_feedback_ms) < FOC_CAN_FEEDBACK_PERIOD_MS) {
        return;
    }
    last_feedback_ms = now;

    speed_rps = Filter_Speed / FOC_TWO_PI;
    position_rad = Motor_Position_Rad;
    memcpy(&data[0], &speed_rps, sizeof(speed_rps));
    memcpy(&data[4], &position_rad, sizeof(position_rad));

    header.StdId = FOC_CAN_FEEDBACK_ID;
#endif
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;

    status.last_tx_id = header.StdId;
    status.last_tx_free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    status.last_hal_error = hcan.ErrorCode;

    if (status.last_tx_free_level == 0U) {
        abort_pending_tx();
        status.tx_error_count++;
        status.last_hal_error = hcan.ErrorCode;
        return;
    }

    if (HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox) != HAL_OK) {
        status.tx_error_count++;
        status.last_hal_error = hcan.ErrorCode;
        return;
    }
    status.last_hal_error = hcan.ErrorCode;
    status.tx_count++;
}

void FocCan_ControlStep(void)
{
    uint32_t now = HAL_GetTick();

    if (pending_command.pending != 0U) {
        uint8_t mode = pending_command.mode;
        float value = pending_command.value;

        pending_command.pending = 0U;
        status.owner = FOC_CONTROL_OWNER_CAN;
        apply_command(mode, value);
    }

    if (FOC_CAN_COMMAND_TIMEOUT_MS > 0U &&
        status.owner == FOC_CONTROL_OWNER_CAN &&
        (now - pending_command.last_rx_ms) > FOC_CAN_COMMAND_TIMEOUT_MS) {
        stop_motor();
        status.owner = FOC_CONTROL_OWNER_NONE;
        status.online = 0U;
        status.timeout_count++;
    }
}

void FocCan_NotifySerialCommand(void)
{
    pending_command.pending = 0U;
    status.owner = FOC_CONTROL_OWNER_SERIAL;
}

void FocCan_Stop(void)
{
    pending_command.pending = 0U;
    status.owner = FOC_CONTROL_OWNER_NONE;
    status.online = 0U;
    stop_motor();
}

const FocCanStatus_t *FocCan_GetStatus(void)
{
    return &status;
}

HAL_StatusTypeDef FocCan_SetTimingProfile(uint8_t profile)
{
    HAL_StatusTypeDef result;

    stop_motor();
    (void)HAL_CAN_Stop(&hcan);
    (void)HAL_CAN_DeInit(&hcan);

    result = apply_timing_profile(profile);
    if (result != HAL_OK) {
        return result;
    }
    result = FocCan_Init();
    if (result == HAL_OK) {
        last_feedback_ms = 0U;
        boot_test_count = 0U;
        status.timing_profile = profile;
        status.loopback_mode = active_loopback_mode;
    }
    return result;
}

HAL_StatusTypeDef FocCan_SetLoopback(uint8_t enable)
{
    active_loopback_mode = (enable != 0U) ? 1U : 0U;
    return FocCan_SetTimingProfile(active_timing_profile);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_handle)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    float value;

    if (hcan_handle->Instance != CAN1 ||
        HAL_CAN_GetRxMessage(hcan_handle, CAN_RX_FIFO0, &header, data) != HAL_OK) {
        status.rx_error_count++;
        return;
    }

    status.last_rx_id = header.StdId;
    status.rx_count++;

    if (header.IDE == CAN_ID_STD &&
        header.RTR == CAN_RTR_DATA &&
        header.StdId == FOC_CAN_BOOT_TEST_ID) {
        status.loopback_count++;
        return;
    }

    if (header.IDE != CAN_ID_STD ||
        header.RTR != CAN_RTR_DATA ||
        header.StdId != FOC_CAN_COMMAND_ID ||
        header.DLC != 8U ||
        data[1] != FOC_CAN_NODE_ID ||
        data[0] > FOC_CAN_MODE_POSITION ||
        checksum8(data, 7U) != data[7]) {
        status.rx_error_count++;
        return;
    }

    memcpy(&value, &data[2], sizeof(value));
    if (float_is_valid(value) == 0U) {
        status.rx_error_count++;
        return;
    }

    pending_command.mode = data[0];
    pending_command.value = value;
    pending_command.last_rx_ms = HAL_GetTick();
    pending_command.pending = 1U;
    status.last_rx_ms = pending_command.last_rx_ms;
    status.online = 1U;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan_handle)
{
    if (hcan_handle->Instance == CAN1) {
        status.rx_error_count++;
        status.last_hal_error = hcan_handle->ErrorCode;
    }
}
