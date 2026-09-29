#include "foc_vofa.h"
#include "motor_param.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define VOFA_TX_FLOAT_COUNT  8U
#define VOFA_TX_FRAME_SIZE   (VOFA_TX_FLOAT_COUNT * sizeof(float) + 4U)
#define VOFA_RX_DMA_SIZE     64U
#define VOFA_RX_QUEUE_SIZE   256U
#define VOFA_COMMAND_SIZE    32U

_Static_assert(sizeof(float) == 4U, "JustFloat requires 32-bit float");

static UART_HandleTypeDef *vofa_uart;
static FOC_ENCODER_HandleTypeDef *vofa_encoder;
static FOC_LOOP_CUR_HandleTypeDef *vofa_loop_cur;
static FOC_LOOP_SPD_HandleTypeDef *vofa_loop_spd;
static FOC_LOOP_POS_HandleTypeDef *vofa_loop_pos;

static volatile FOC_CONTROL_MODE control_mode = FOC_CONTROL_SPEED;
static volatile uint8_t telemetry_pending;
static volatile uint8_t tx_busy;
static uint8_t tx_buffer[VOFA_TX_FRAME_SIZE];

static uint8_t rx_dma_buffer[VOFA_RX_DMA_SIZE];
static uint16_t rx_dma_pos;
static volatile uint8_t rx_queue[VOFA_RX_QUEUE_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint8_t rx_overflow;
static char command_buffer[VOFA_COMMAND_SIZE];
static uint8_t command_length;
static uint8_t discard_line;

static void FOC_VOFA_QueueRxByte(uint8_t value)
{
    uint16_t next = (uint16_t)((rx_head + 1U) % VOFA_RX_QUEUE_SIZE);
    if (next == rx_tail)
    {
        rx_overflow = 1U;
        return;
    }
    rx_queue[rx_head] = value;
    rx_head = next;
}

static uint32_t FOC_VOFA_MaskTim6(void)
{
    uint32_t was_enabled = NVIC_GetEnableIRQ(TIM6_DAC_IRQn);
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    return was_enabled;
}

static void FOC_VOFA_RestoreTim6(uint32_t was_enabled)
{
    if (was_enabled != 0U)
    {
        NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
}

static void FOC_VOFA_SwitchToSpeed(void)
{
    if (control_mode == FOC_CONTROL_SPEED)
    {
        return;
    }

    uint32_t tim6_enabled = FOC_VOFA_MaskTim6();
    /* A zero target and cleared outer-loop integrals avoid a stale position command. */
    FOC_LOOP_SPD_SetSpeedRef(vofa_loop_spd, 0.0f);
    FOC_PI_Reset(&vofa_loop_spd->pi_spd);
    FOC_PI_Reset(&vofa_loop_pos->pi_pos);
    control_mode = FOC_CONTROL_SPEED;
    FOC_VOFA_RestoreTim6(tim6_enabled);
}

static void FOC_VOFA_SwitchToPosition(void)
{
    if (control_mode == FOC_CONTROL_POSITION)
    {
        return;
    }

    FOC_ENCODER_UpdateAngle(vofa_encoder);
    float current_position = vofa_encoder->angle_m;
    uint32_t tim6_enabled = FOC_VOFA_MaskTim6();
    /* Hold the measured position before enabling the position loop. */
    FOC_LOOP_POS_SetPositionRef(vofa_loop_pos, current_position);
    FOC_LOOP_SPD_SetSpeedRef(vofa_loop_spd, 0.0f);
    FOC_PI_Reset(&vofa_loop_pos->pi_pos);
    FOC_PI_Reset(&vofa_loop_spd->pi_spd);
    control_mode = FOC_CONTROL_POSITION;
    FOC_VOFA_RestoreTim6(tim6_enabled);
}

static uint8_t FOC_VOFA_ParseFloat(const char *text, float *value)
{
    char *end;
    float parsed;

    if (*text == '\0')
    {
        return 0U;
    }
    parsed = strtof(text, &end);
    if ((end == text) || (*end != '\0') || !isfinite(parsed))
    {
        return 0U;
    }
    *value = parsed;
    return 1U;
}

static void FOC_VOFA_ExecuteCommand(const char *command)
{
    float value;
    uint32_t tim6_enabled;

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
        tim6_enabled = FOC_VOFA_MaskTim6();
        FOC_LOOP_SPD_SetSpeedRef(vofa_loop_spd, 0.0f);
        FOC_PI_Reset(&vofa_loop_spd->pi_spd);
        FOC_VOFA_RestoreTim6(tim6_enabled);
    }
    else if ((command[0] == 'S') && FOC_VOFA_ParseFloat(&command[1], &value))
    {
        if (value > MOTOR_MAX_SPEED_RPM) value = MOTOR_MAX_SPEED_RPM;
        if (value < -MOTOR_MAX_SPEED_RPM) value = -MOTOR_MAX_SPEED_RPM;
        tim6_enabled = FOC_VOFA_MaskTim6();
        FOC_LOOP_SPD_SetSpeedRef(vofa_loop_spd, value);
        FOC_VOFA_RestoreTim6(tim6_enabled);
    }
    else if ((command[0] == 'P') && FOC_VOFA_ParseFloat(&command[1], &value))
    {
        tim6_enabled = FOC_VOFA_MaskTim6();
        FOC_LOOP_POS_SetPositionRef(vofa_loop_pos, value);
        FOC_VOFA_RestoreTim6(tim6_enabled);
    }
}

HAL_StatusTypeDef FOC_VOFA_Init(UART_HandleTypeDef *uart,
                                FOC_ENCODER_HandleTypeDef *encoder,
                                FOC_LOOP_CUR_HandleTypeDef *loop_cur,
                                FOC_LOOP_SPD_HandleTypeDef *loop_spd,
                                FOC_LOOP_POS_HandleTypeDef *loop_pos)
{
    HAL_StatusTypeDef status;

    if ((uart == NULL) || (uart->hdmarx == NULL) || (uart->hdmatx == NULL) ||
        (encoder == NULL) || (loop_cur == NULL) || (loop_spd == NULL) || (loop_pos == NULL))
    {
        return HAL_ERROR;
    }

    vofa_uart = uart;
    vofa_encoder = encoder;
    vofa_loop_cur = loop_cur;
    vofa_loop_spd = loop_spd;
    vofa_loop_pos = loop_pos;
    control_mode = FOC_CONTROL_SPEED;
    telemetry_pending = 0U;
    tx_busy = 0U;
    rx_dma_pos = 0U;
    rx_head = 0U;
    rx_tail = 0U;
    rx_overflow = 0U;
    command_length = 0U;
    discard_line = 0U;

    status = HAL_UARTEx_ReceiveToIdle_DMA(uart, rx_dma_buffer, VOFA_RX_DMA_SIZE);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(uart->hdmarx, DMA_IT_HT);
    }
    return status;
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
    if (__atomic_exchange_n(&telemetry_pending, 0U, __ATOMIC_ACQUIRE) == 0U)
    {
        return;
    }
    if ((tx_busy != 0U) || (vofa_uart == NULL))
    {
        return; /* Drop this snapshot while the previous DMA frame is in flight. */
    }

    const volatile FOC_LOOP_SPD_HandleTypeDef *spd = vofa_loop_spd;
    const volatile FOC_LOOP_CUR_HandleTypeDef *cur = vofa_loop_cur;
    const float speed_ref = spd->speed_ref;
    const float speed_fbk = spd->speed_fbk;
    const float id_ref = cur->id_ref;
    const float i_d = cur->i_d;
    const float iq_ref = cur->iq_ref;
    const float i_q = cur->i_q;
    const float v_d = cur->v_d;
    const float v_q = cur->v_q;

    memcpy(&tx_buffer[0], &speed_ref, sizeof(float));
    memcpy(&tx_buffer[4], &speed_fbk, sizeof(float));
    memcpy(&tx_buffer[8], &id_ref, sizeof(float));
    memcpy(&tx_buffer[12], &i_d, sizeof(float));
    memcpy(&tx_buffer[16], &iq_ref, sizeof(float));
    memcpy(&tx_buffer[20], &i_q, sizeof(float));
    memcpy(&tx_buffer[24], &v_d, sizeof(float));
    memcpy(&tx_buffer[28], &v_q, sizeof(float));
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
    if (rx_overflow != 0U)
    {
        rx_overflow = 0U;
        rx_tail = rx_head;
        command_length = 0U;
        discard_line = 1U;
    }

    for (uint16_t processed = 0U; processed < VOFA_RX_QUEUE_SIZE; ++processed)
    {
        uint16_t tail = rx_tail;
        if (tail == rx_head)
        {
            break;
        }

        char value = (char)rx_queue[tail];
        rx_tail = (uint16_t)((tail + 1U) % VOFA_RX_QUEUE_SIZE);
        if (value == '\r')
        {
            continue;
        }
        if (value == '\n')
        {
            if ((discard_line == 0U) && (command_length != 0U))
            {
                command_buffer[command_length] = '\0';
                FOC_VOFA_ExecuteCommand(command_buffer);
            }
            command_length = 0U;
            discard_line = 0U;
        }
        else if (discard_line == 0U)
        {
            if (command_length < (VOFA_COMMAND_SIZE - 1U))
            {
                command_buffer[command_length++] = value;
            }
            else
            {
                command_length = 0U;
                discard_line = 1U;
            }
        }
    }
}

void FOC_VOFA_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if ((uart != vofa_uart) || (size > VOFA_RX_DMA_SIZE))
    {
        return;
    }

    uint16_t pos = rx_dma_pos;
    if (pos == VOFA_RX_DMA_SIZE)
    {
        pos = 0U;
    }

    if (HAL_UARTEx_GetRxEventType(uart) == HAL_UART_RXEVENT_TC)
    {
        /* TC proves a wrap, even if DMA has already written bytes into the next lap. */
        while (pos < VOFA_RX_DMA_SIZE)
        {
            FOC_VOFA_QueueRxByte(rx_dma_buffer[pos++]);
        }
        pos = 0U;
        if (size == VOFA_RX_DMA_SIZE)
        {
            rx_dma_pos = size;
            return;
        }
    }
    else if (size < pos)
    {
        while (pos < VOFA_RX_DMA_SIZE)
        {
            FOC_VOFA_QueueRxByte(rx_dma_buffer[pos++]);
        }
        pos = 0U;
    }
    while (pos < size)
    {
        FOC_VOFA_QueueRxByte(rx_dma_buffer[pos++]);
    }
    rx_dma_pos = size;
    /* Circular ReceiveToIdle DMA stays active after IDLE and transfer-complete events. */
}

void FOC_VOFA_TxCompleteCallback(UART_HandleTypeDef *uart)
{
    if (uart == vofa_uart)
    {
        tx_busy = 0U;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    FOC_VOFA_RxEventCallback(uart, size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    FOC_VOFA_TxCompleteCallback(uart);
}
