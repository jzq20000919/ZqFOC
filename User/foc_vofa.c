/**
 * @file    foc_vofa.c
 * @brief   VOFA+遥测发送与命令处理
 */
#include "foc_vofa.h"
#include "motor_param.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define VOFA_TX_FRAME_SIZE   36U // 8个float及JustFloat帧尾，总字节数
#define VOFA_COMMAND_SIZE    32U // 单条ASCII命令缓冲区长度，字节
static UART_HandleTypeDef *vofa_uart;
static FOC_ENCODER_HandleTypeDef *vofa_encoder;
static FOC_LOOP_CUR_HandleTypeDef *vofa_loop_cur;
static FOC_LOOP_SPD_HandleTypeDef *vofa_loop_spd;
static FOC_LOOP_POS_HandleTypeDef *vofa_loop_pos;
static volatile FOC_CONTROL_MODE control_mode = FOC_CONTROL_SPEED;
static volatile uint8_t telemetry_pending;
static volatile uint8_t tx_busy;
static uint8_t tx_buffer[VOFA_TX_FRAME_SIZE];
static uint8_t rx_byte;
static char command_buffer[VOFA_COMMAND_SIZE];
static volatile uint8_t command_ready = 0U;
static uint8_t command_length;

/**
 * @brief  切换到速度模式并清除旧目标与PI积分
 */
static void FOC_VOFA_SwitchToSpeed(void)
{
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    /* 切到速度模式时目标置零，避免旧位置目标引起突跳。 */
    FOC_LOOP_SPD_SetSpeedRef(vofa_loop_spd, 0.0f);
    FOC_PI_Reset(&vofa_loop_spd->pi_spd);
    FOC_PI_Reset(&vofa_loop_pos->pi_pos);
    control_mode = FOC_CONTROL_SPEED;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief  以当前位置为目标切换到位置模式
 */
static void FOC_VOFA_SwitchToPosition(void)
{
    if (control_mode == FOC_CONTROL_POSITION)
    {
        return;
    }

    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    /* 先读取当前位置，再将它作为位置目标。 */
    FOC_ENCODER_UpdateAngle(vofa_encoder);
    FOC_LOOP_POS_SetPositionRef(vofa_loop_pos, vofa_encoder->angle_m);
    FOC_LOOP_SPD_SetSpeedRef(vofa_loop_spd, 0.0f);
    FOC_PI_Reset(&vofa_loop_pos->pi_pos);
    FOC_PI_Reset(&vofa_loop_spd->pi_spd);
    control_mode = FOC_CONTROL_POSITION;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief  解析一条已完成的ASCII控制命令
 * @param  command 以空字符结尾的命令字符串
 */
static void FOC_VOFA_ExecuteCommand(const char *command)
{
    float value;
    char *end;

    if (strcmp(command, "MS") == 0)
    {
        FOC_VOFA_SwitchToSpeed();
    }
    else if (strcmp(command, "MP") == 0)
    {
        FOC_VOFA_SwitchToPosition();
    }
    else if (strcmp(command, "STOP") == 0)
    {
        FOC_VOFA_SwitchToSpeed();
    }
    else if ((command[0] == 'S') || (command[0] == 'P'))
    {
        value = strtof(&command[1], &end);
        if ((end == &command[1]) || (*end != '\0') || !isfinite(value))
        {
            return;
        }

        if (command[0] == 'P')
        {
            /* 位置模块按单圈角度工作，先取余以免超大输入反复减 2π。 */
            value = fmodf(value, FOC_TWO_PI);
        }

        NVIC_DisableIRQ(TIM6_DAC_IRQn);
        if (command[0] == 'S')
        {
            if (value > MOTOR_MAX_SPEED_RPM) value = MOTOR_MAX_SPEED_RPM;
            if (value < -MOTOR_MAX_SPEED_RPM) value = -MOTOR_MAX_SPEED_RPM;
            FOC_LOOP_SPD_SetSpeedRef(vofa_loop_spd, value);
        }
        else
        {
            FOC_LOOP_POS_SetPositionRef(vofa_loop_pos, value);
        }
        NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
}

HAL_StatusTypeDef FOC_VOFA_Init(UART_HandleTypeDef *uart,
                                FOC_ENCODER_HandleTypeDef *encoder,
                                FOC_LOOP_CUR_HandleTypeDef *loop_cur,
                                FOC_LOOP_SPD_HandleTypeDef *loop_spd,
                                FOC_LOOP_POS_HandleTypeDef *loop_pos)
{
    vofa_uart = uart;
    vofa_encoder = encoder;
    vofa_loop_cur = loop_cur;
    vofa_loop_spd = loop_spd;
    vofa_loop_pos = loop_pos;
    control_mode = FOC_CONTROL_SPEED;
    telemetry_pending = 0U;
    tx_busy = 0U;
    command_ready = 0U;
    command_length = 0U;

    return HAL_UART_Receive_IT(vofa_uart, &rx_byte, 1U);
}

FOC_CONTROL_MODE FOC_VOFA_GetControlMode(void)
{
    return control_mode;
}

void FOC_VOFA_RequestTelemetry(void)
{
    telemetry_pending = 1U;
}

void FOC_VOFA_ProcessTx(void)
{
    // 电流环中断只置请求标志，帧打包与DMA发送留在主循环。
    if (telemetry_pending == 0U)
    {
        return;
    }
    telemetry_pending = 0U;
    if (tx_busy != 0U)
    {
        return; // DMA忙时丢弃本帧，避免等待影响FOC。
    }

    // 先读取8个状态值，再按JustFloat顺序打包。
    const volatile FOC_LOOP_SPD_HandleTypeDef *spd = vofa_loop_spd;
    const volatile FOC_LOOP_CUR_HandleTypeDef *cur = vofa_loop_cur;
    const float values[8] = {
        spd->speed_ref, spd->speed_fbk, cur->id_ref, cur->i_d,
        cur->iq_ref, cur->i_q, cur->v_d, cur->v_q
    };
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        memcpy(&tx_buffer[i * 4U], &values[i], sizeof(float));
    }
    // JustFloat帧尾：00 00 80 7F。
    tx_buffer[32] = 0x00U;
    tx_buffer[33] = 0x00U;
    tx_buffer[34] = 0x80U;
    tx_buffer[35] = 0x7FU;

    if (HAL_UART_Transmit_DMA(vofa_uart, tx_buffer, VOFA_TX_FRAME_SIZE) == HAL_OK)
    {
        tx_busy = 1U;
    }
}

void FOC_VOFA_ProcessRx(void)
{
    if (command_ready == 0U)
    {
        return;
    }

    /* 接收中断在命令执行期间忽略新字符，避免改写当前命令。 */
    FOC_VOFA_ExecuteCommand(command_buffer);
    command_length = 0U;
    command_ready = 0U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    // UART中断只收集字符，完整命令交给主循环解析。
    if (uart != vofa_uart)
    {
        return;
    }

    if (command_ready == 0U)
    {
        if (rx_byte == '\n')
        {
            if ((command_length > 0U) && (command_length < VOFA_COMMAND_SIZE))
            {
                command_buffer[command_length] = '\0';
                command_ready = 1U;
            }
            else
            {
                command_length = 0U;
            }
        }
        else if (rx_byte != '\r')
        {
            if (command_length < VOFA_COMMAND_SIZE - 1U)
            {
                command_buffer[command_length++] = (char)rx_byte;
            }
            else
            {
                /* 超长命令用长度 32 标记，遇到换行时整条丢弃。 */
                command_length = VOFA_COMMAND_SIZE;
            }
        }
    }

    /* 每次只接收一个字节，处理完立即重新启动接收中断。 */
    HAL_UART_Receive_IT(vofa_uart, &rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == vofa_uart)
    {
        /* 串口溢出会结束 HAL 接收，丢弃半条命令后重新接收。 */
        if (command_ready == 0U)
        {
            command_length = 0U;
        }
        HAL_UART_Receive_IT(vofa_uart, &rx_byte, 1U);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == vofa_uart)
    {
        // 释放发送标志，允许主循环提交下一帧DMA。
        tx_busy = 0U;
    }
}
