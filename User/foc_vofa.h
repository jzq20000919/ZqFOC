/**
 * @file    foc_vofa.h
 * @brief   VOFA+遥测与命令接口
 */
#ifndef FOC_VOFA_H
#define FOC_VOFA_H

#include "stm32g4xx_hal.h"
#include "foc_encoder.h"
#include "foc_loop_cur.h"
#include "foc_loop_spd.h"
#include "foc_loop_pos.h"

typedef enum
{
    FOC_CONTROL_SPEED = 0,   // 速度控制模式
    FOC_CONTROL_POSITION = 1 // 位置控制模式
} FOC_CONTROL_MODE;

/**
 * @brief  初始化VOFA+通信并启动UART单字节中断接收
 * @param  uart UART句柄
 * @param  encoder 编码器句柄
 * @param  loop_cur 电流环句柄
 * @param  loop_spd 速度环句柄
 * @param  loop_pos 位置环句柄
 * @return HAL_OK 接收启动成功
 * @return HAL_ERROR 接收启动失败
 * @note   返回值来自HAL_UART_Receive_IT，也可能返回HAL_BUSY。
 */
HAL_StatusTypeDef FOC_VOFA_Init(UART_HandleTypeDef *uart,
                                FOC_ENCODER_HandleTypeDef *encoder,
                                FOC_LOOP_CUR_HandleTypeDef *loop_cur,
                                FOC_LOOP_SPD_HandleTypeDef *loop_spd,
                                FOC_LOOP_POS_HandleTypeDef *loop_pos);

/**
 * @brief  获取当前控制模式
 * @return 当前速度或位置控制模式
 */
FOC_CONTROL_MODE FOC_VOFA_GetControlMode(void);
/**
 * @brief  标记需要发送一帧遥测数据
 * @note   可由电流环中断或停机时的主循环调用；打包和DMA发送均在主循环中进行。
 */
void FOC_VOFA_RequestTelemetry(void);
/**
 * @brief  在主循环中打包并通过UART DMA发送JustFloat遥测帧
 * @note   DMA忙时跳过当前帧。
 */
void FOC_VOFA_ProcessTx(void);
/**
 * @brief  在主循环中解析并执行已接收的控制命令
 */
void FOC_VOFA_ProcessRx(void);

#endif // FOC_VOFA_H
