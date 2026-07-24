#include "foc_can.h"

#include "Open_Loop.h"
#include "PID.h"
#include "Paremeter.h"
#include "can.h"
#include "usart.h"
#include <string.h>

#define FOC_TWO_PI 6.2831853071795864769f

typedef struct {
    volatile uint8_t pending;
    volatile uint8_t mode;
    volatile float value;
    volatile uint32_t last_rx_ms;
} FocCanPendingCommand_t;

static FocCanPendingCommand_t pending_command;
static FocCanStatus_t status;
static uint32_t last_feedback_ms;

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

static void apply_command(uint8_t mode, float value)
{
    status.last_mode = mode;

    switch ((FocCanMode_t)mode) {
    case FOC_CAN_MODE_CURRENT:
        CMD.CMD_Type = CMD_TORQUE;
        CMD.Target = clampf(value, MAX_CURRENT) / MAX_CURRENT;
        break;

    case FOC_CAN_MODE_SPEED:
        CMD.CMD_Type = CMD_SPEED;
        CMD.Target = clampf(value, FOC_CAN_MAX_SPEED_RPS) * FOC_TWO_PI;
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
    stop_motor();

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (uint16_t)(FOC_CAN_COMMAND_ID << 5U);
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5U);
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
    float speed_rps;
    float position_rad;

    if ((now - last_feedback_ms) < FOC_CAN_FEEDBACK_PERIOD_MS) {
        return;
    }
    last_feedback_ms = now;

    speed_rps = Filter_Speed / FOC_TWO_PI;
    position_rad = Motor_Position_Rad;
    memcpy(&data[0], &speed_rps, sizeof(speed_rps));
    memcpy(&data[4], &position_rad, sizeof(position_rad));

    header.StdId = FOC_CAN_FEEDBACK_ID;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U ||
        HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox) != HAL_OK) {
        status.tx_error_count++;
        return;
    }
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

    if (status.owner == FOC_CONTROL_OWNER_CAN &&
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

    if (header.IDE != CAN_ID_STD ||
        header.RTR != CAN_RTR_DATA ||
        header.StdId != FOC_CAN_COMMAND_ID ||
        header.DLC != 8U ||
        data[1] != FOC_CAN_NODE_ID ||
        data[0] > FOC_CAN_MODE_SPEED ||
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
    status.rx_count++;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan_handle)
{
    if (hcan_handle->Instance == CAN1) {
        status.rx_error_count++;
    }
}
