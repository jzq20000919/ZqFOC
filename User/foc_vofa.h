#ifndef FOC_VOFA_H
#define FOC_VOFA_H

#include "stm32g4xx_hal.h"
#include "foc_encoder.h"
#include "foc_loop_cur.h"
#include "foc_loop_spd.h"
#include "foc_loop_pos.h"

typedef enum
{
    FOC_CONTROL_SPEED = 0,
    FOC_CONTROL_POSITION = 1
} FOC_CONTROL_MODE;

HAL_StatusTypeDef FOC_VOFA_Init(UART_HandleTypeDef *uart,
                                FOC_ENCODER_HandleTypeDef *encoder,
                                FOC_LOOP_CUR_HandleTypeDef *loop_cur,
                                FOC_LOOP_SPD_HandleTypeDef *loop_spd,
                                FOC_LOOP_POS_HandleTypeDef *loop_pos);

FOC_CONTROL_MODE FOC_VOFA_GetControlMode(void);
void FOC_VOFA_RequestTelemetry(void);
void FOC_VOFA_ProcessTx(void);
void FOC_VOFA_ProcessRx(void);
void FOC_VOFA_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size);
void FOC_VOFA_TxCompleteCallback(UART_HandleTypeDef *uart);

#endif // FOC_VOFA_H
