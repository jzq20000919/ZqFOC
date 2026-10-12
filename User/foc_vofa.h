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
 * @brief  初始化VOFA+通信并启动512字节UART循环DMA接收
 * @param  uart UART句柄
 * @param  encoder 编码器句柄
 * @param  loop_cur 电流环句柄
 * @param  loop_spd 速度环句柄
 * @param  loop_pos 位置环句柄
 * @return HAL_OK 接收启动成功
 * @return HAL_ERROR 接收启动失败
 * @note   USART必须已有循环RX DMA；返回值来自HAL_UART_Receive_DMA，也可能返回HAL_BUSY。
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
/** @brief 由主循环选择速度/位置模式；CAN与串口复用原有切换逻辑，相同模式不清除目标。 */
void FOC_VOFA_SetControlMode(FOC_CONTROL_MODE mode);
/**
 * @brief  标记需要发送一帧遥测数据
 * @note   可由电流环中断或停机时的主循环调用；打包和DMA发送均在主循环中进行。
 */
void FOC_VOFA_RequestTelemetry(void);
/**
 * @brief  在主循环中打包并通过UART DMA发送JustFloat遥测帧
 * @note   最快每1ms发送一次最新值，DMA忙时跳过当前帧；十个通道与44字节帧格式不变。
 */
void FOC_VOFA_ProcessTx(void);
/**
 * @brief  在主循环中读取循环DMA，遇LF执行完整ASCII命令，每轮最多处理128字节
 * @note   支持半条命令和连续多条命令；超长、覆盖或接收错误时丢弃到LF，不执行残缺命令。
 */
void FOC_VOFA_ProcessRx(void);

#endif // FOC_VOFA_H
