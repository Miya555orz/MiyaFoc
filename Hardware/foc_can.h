#ifndef FOC_CAN_H
#define FOC_CAN_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define FOC_CAN_NODE_ID                1U
#define FOC_CAN_COMMAND_BASE_ID        0x210U
#define FOC_CAN_FEEDBACK_BASE_ID       0x290U
#define FOC_CAN_COMMAND_ID             (FOC_CAN_COMMAND_BASE_ID + FOC_CAN_NODE_ID)
#define FOC_CAN_FEEDBACK_ID            (FOC_CAN_FEEDBACK_BASE_ID + FOC_CAN_NODE_ID)
#define FOC_CAN_COMMAND_TIMEOUT_MS     100U
#define FOC_CAN_FEEDBACK_PERIOD_MS     10U
#define FOC_CAN_MAX_SPEED_RPS          120.0f

typedef enum {
    FOC_CAN_MODE_STOP = 0,
    FOC_CAN_MODE_CURRENT = 1,
    FOC_CAN_MODE_SPEED = 2,
    FOC_CAN_MODE_POSITION = 3
} FocCanMode_t;

typedef enum {
    FOC_CONTROL_OWNER_NONE = 0,
    FOC_CONTROL_OWNER_SERIAL,
    FOC_CONTROL_OWNER_CAN
} FocControlOwner_t;

typedef struct {
    uint8_t online;
    uint8_t last_mode;
    FocControlOwner_t owner;
    uint32_t last_rx_ms;
    uint32_t rx_count;
    uint32_t rx_error_count;
    uint32_t tx_count;
    uint32_t tx_error_count;
    uint32_t timeout_count;
} FocCanStatus_t;

HAL_StatusTypeDef FocCan_Init(void);
void FocCan_Process(void);
void FocCan_ControlStep(void);
void FocCan_NotifySerialCommand(void);
void FocCan_Stop(void);
const FocCanStatus_t *FocCan_GetStatus(void);

#endif
