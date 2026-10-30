/**
 * @file    foc_comm_usart.h
 * @brief   STM32G4 与 PyQt 上位机共用的 USART 通信接口。
 *
 * 调用顺序：外设/FOC 初始化 -> Init -> 主循环反复 ProcessRx/ProcessTx。
 * ADC 中断只调用 RequestTelemetry；GetControlMode 可在控制中断中读取。
 * USART2：2,000,000 baud，8N1，RX 循环 DMA，TX 普通 DMA。
 * RX：ASCII 命令，以 LF 结尾，也接受 CRLF；TX：10 个 float32 + JustFloat 帧尾。
 * 本协议没有 CRC、命令回执和运行状态字段；发送成功不代表对端已执行。
 */
#ifndef FOC_COMM_USART_H
#define FOC_COMM_USART_H

#include "stm32g4xx_hal.h"
#include "foc_encoder.h"
#include "foc_loop_cur.h"
#include "foc_loop_spd.h"
#include "foc_loop_pos.h"

/* CAN、串口、按键及控制中断使用同一个控制模式。 */
typedef enum
{
    FOC_CONTROL_SPEED = 0,    /**< 速度环接收 rpm 目标。 */
    FOC_CONTROL_POSITION = 1  /**< 位置环根据单圈角度生成速度目标。 */
} FOC_CONTROL_MODE;

/**
 * @brief  保存 FOC 对象并启动接收；外设和 FOC 对象初始化后，在主循环调用一次。
 * @param  uart 已配置 RX 循环 DMA 和 TX 普通 DMA 的 UART 句柄。
 * @param  encoder 编码器对象；对象的生命周期须覆盖整个通信过程。
 * @param  loop_cur 电流环对象。
 * @param  loop_spd 速度环对象。
 * @param  loop_pos 位置环对象。
 * @return HAL_OK 成功；HAL_ERROR 参数/配置错误；其他值来自 HAL 启动接收。
 * @note   成功初始化后再次调用返回 HAL_BUSY，防止覆盖正在使用的 DMA 缓冲。
 */
HAL_StatusTypeDef FOC_COMM_USART_Init(UART_HandleTypeDef *uart,
                                     FOC_ENCODER_HandleTypeDef *encoder,
                                     FOC_LOOP_CUR_HandleTypeDef *loop_cur,
                                     FOC_LOOP_SPD_HandleTypeDef *loop_spd,
                                     FOC_LOOP_POS_HandleTypeDef *loop_pos);

/** @brief 读取当前模式；可在主循环或控制中断中调用，无副作用。 */
FOC_CONTROL_MODE FOC_COMM_USART_GetControlMode(void);

/**
 * @brief 在主循环切换模式；无效模式、未初始化或重复选择当前模式时不处理。
 * @param mode 速度模式清零速度目标；位置模式先以当前位置作为目标。
 * @note  切换时清零速度/位置 PI 积分，短临界区结束后恢复原来的中断状态。
 */
void FOC_COMM_USART_SetControlMode(FOC_CONTROL_MODE mode);

/**
 * @brief 请求发送最新遥测；可在 ADC 中断调用，只置一个标志，耗时固定。
 * @note  多个请求合并为一次，实际发送由主循环 ProcessTx 完成。
 */
void FOC_COMM_USART_RequestTelemetry(void);

/**
 * @brief 主循环调用：每轮最多处理 128 字节，收到完整 LF 命令才执行。
 * @note  支持拆包/粘包；超长、非法字节、DMA 覆盖或接收错误后丢弃到 LF。
 *        错误恢复也在此执行；中断不解析命令、不启停电机。
 */
void FOC_COMM_USART_ProcessRx(void);

/**
 * @brief 主循环调用：取最新数据快照，最多每 1 ms 启动一次 44 字节 DMA 发送。
 * @note  DMA 忙时跳过，不排队、不等待；发送完成前不修改发送缓冲区。
 *        未请求时不发送，当前工程仅在电机运行时请求遥测。
 */
void FOC_COMM_USART_ProcessTx(void);

#endif /* FOC_COMM_USART_H */
