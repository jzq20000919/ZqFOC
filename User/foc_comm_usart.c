/**
 * @file    foc_comm_usart.c
 * @brief   USART 接收、命令执行和遥测发送；与 ZqFOC_PyQt6/zqfoc_pc.py 配套。
 *
 * 数据流：RX DMA -> 字节缓冲 -> LF 分行 -> 严格校验 -> 已有 FOC 接口。
 *         ADC 置发送请求 -> 主循环取快照 -> TX DMA -> 完成回调释放缓冲。
 *
 * 命令：MS / MP / START / STOP / S<rpm> / A<度> / P<弧度>，每条以 \n 结束。
 * A 自动切到位置模式；P 是保留的旧接口，只改位置目标，不自动切模式。
 * START 复用现有对齐流程；对齐期间设置的目标仍可能被启动流程清零。
 *
 * DMA 在后台收发，中断只记状态。解析、打包和启停均由主循环执行。
 * 不使用堆内存、阻塞串口发送或逐字节接收中断。
 */
#include "foc_comm_usart.h"
#include "foc_manager.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* 一、协议和容量：改帧格式时必须同时修改 PC 端。 */
#define TX_CHANNELS       10U
#define TX_FRAME_SIZE     44U
#define TX_PERIOD_MS      1U
#define RX_DMA_SIZE       512U
#define RX_BUDGET         128U  /* 每轮处理上限，避免连续输入占满主循环。 */
#define COMMAND_SIZE      32U  /* 包括字符串末尾的 '\0'。 */
_Static_assert(sizeof(float) == 4U, "The protocol requires 32-bit float");

/* 二、共享对象和状态：本模块只支持一个 UART、一台电机。 */
static UART_HandleTypeDef *comm_uart;
static FOC_ENCODER_HandleTypeDef *comm_encoder;
static FOC_LOOP_CUR_HandleTypeDef *comm_cur;
static FOC_LOOP_SPD_HandleTypeDef *comm_spd;
static FOC_LOOP_POS_HandleTypeDef *comm_pos;
static volatile FOC_CONTROL_MODE control_mode = FOC_CONTROL_SPEED;

static uint8_t tx_buffer[TX_FRAME_SIZE]; /* 静态存储，DMA 完成前一直有效。 */
static volatile uint8_t tx_busy;
static volatile uint8_t telemetry_pending;
static uint32_t tx_last_tick;

static volatile uint8_t rx_dma_buffer[RX_DMA_SIZE]; /* DMA 写，主循环读。 */
static volatile uint32_t rx_wrap_count; /* RX DMA 每写满一圈，由回调加一。 */
static uint32_t rx_read_count;          /* 主循环累计消费的字节数。 */
static volatile uint8_t rx_restart_pending;
static uint32_t rx_overrun_count;       /* 可在调试器观察的缓冲覆盖次数。 */
static char command_buffer[COMMAND_SIZE];
static uint8_t command_length;
static uint8_t command_discard;         /* 当前行损坏，直到 LF 都不执行。 */
static uint8_t command_cr;              /* CR 只允许紧挨 LF，避免拼接坏命令。 */

/* 三、DMA 接收：用累计写入量区分“没有数据”和“恰好写满一圈”。 */
/**
 * @brief 在初始化或接收已停止后启动循环 DMA，并重置读写计数。
 * @return HAL_UART_Receive_DMA 的状态；失败由调用者处理或稍后重试。
 * @note 主循环调用；关掉不需要的半满中断，满缓冲中断仍用于计圈。
 */
static HAL_StatusTypeDef StartRxDMA(void)
{
    rx_wrap_count = 0U;
    rx_read_count = 0U;
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(
        comm_uart, (uint8_t *)rx_dma_buffer, RX_DMA_SIZE);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(comm_uart->hdmarx, DMA_IT_HT);
    }
    return status;
}

/**
 * @brief 读取 DMA 累计写入字节数，短临界区内只读计数和寄存器。
 * @return uint32 累计字节数，允许自然回绕，差值也用无符号运算。
 * @note DMA 完成但回调尚未执行时，临时补算一圈，不能只比较读写下标。
 *       正常运行须及时服务 DMA 中断：2 Mbps 连续满速输入下每圈约 2.56 ms。
 */
static uint32_t GetRxCount(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t wraps = rx_wrap_count;
    uint32_t remaining = __HAL_DMA_GET_COUNTER(comm_uart->hdmarx);
    if (__HAL_DMA_GET_FLAG(comm_uart->hdmarx,
                          __HAL_DMA_GET_TC_FLAG_INDEX(comm_uart->hdmarx)) != 0U)
    {
        wraps++;
        remaining = __HAL_DMA_GET_COUNTER(comm_uart->hdmarx);
        if (remaining == 0U)
        {
            remaining = RX_DMA_SIZE; /* 重装载瞬间，不能重复计算同一圈。 */
        }
    }
    __set_PRIMASK(primask);
    return wraps * RX_DMA_SIZE + RX_DMA_SIZE - remaining;
}

/* 四、命令：传输层只负责完整性，执行层复用已有 FOC 功能。 */
/**
 * @brief 把十进制文本解析为有限浮点数；允许符号、小数和科学计数法。
 * @param text 非空、以 '\0' 结束的数值字符串，不包含命令字母。
 * @param value 成功时写入解析结果。
 * @return 1 合法；0 空值、空格、十六进制、尾随字符、上溢或下溢。
 */
static uint8_t ParseValue(const char *text, float *value)
{
    if (*text == '\0') return 0U;
    for (const char *p = text; *p != '\0'; ++p)
    {
        if ((*p < '0' || *p > '9') && *p != '+' && *p != '-' &&
            *p != '.' && *p != 'e' && *p != 'E') return 0U;
    }
    char *end;
    errno = 0;
    float parsed = strtof(text, &end);
    if (end == text || *end != '\0' || errno == ERANGE || !isfinite(parsed))
    {
        return 0U;
    }
    *value = parsed;
    return 1U;
}

/**
 * @brief 执行一条完整且无非法字节的命令；仅由主循环调用。
 * @param command 不含 CR/LF、以 '\0' 结束的命令；未知命令直接忽略。
 * @note S 按 MOTOR_MAX_SPEED_RPM 限幅；A/P 先取单圈余数，避免巨大角度慢循环。
 *       没有 ACK；调用 START 不代表电机已经完成对齐并进入闭环。
 */
static void ExecuteCommand(const char *command)
{
    if (strcmp(command, "MS") == 0)
    {
        FOC_COMM_USART_SetControlMode(FOC_CONTROL_SPEED);
    }
    else if (strcmp(command, "MP") == 0)
    {
        FOC_COMM_USART_SetControlMode(FOC_CONTROL_POSITION);
    }
    else if (strcmp(command, "START") == 0)
    {
        FOC_Motor_Start();
    }
    else if (strcmp(command, "STOP") == 0)
    {
        FOC_Motor_Stop();
    }
    else if (command[0] == 'S' || command[0] == 'A' || command[0] == 'P')
    {
        float value;
        if (ParseValue(command + 1, &value) == 0U) return;
        if (command[0] == 'S')
        {
            if (value > MOTOR_MAX_SPEED_RPM) value = MOTOR_MAX_SPEED_RPM;
            if (value < -MOTOR_MAX_SPEED_RPM) value = -MOTOR_MAX_SPEED_RPM;
        }
        else if (command[0] == 'A')
        {
            value = fmodf(value, 360.0f) * FOC_PI / 180.0f;
            FOC_COMM_USART_SetControlMode(FOC_CONTROL_POSITION);
        }
        else
        {
            value = fmodf(value, FOC_TWO_PI);
        }

        /* 只保护目标写入；解析、取余等运算都在临界区外完成。 */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (command[0] == 'S') FOC_LOOP_SPD_SetSpeedRef(comm_spd, value);
        else FOC_LOOP_POS_SetPositionRef(comm_pos, value);
        __set_PRIMASK(primask);
    }
}

/**
 * @brief 消费一个 RX 字节；只在 LF 时执行，保存未结束的半条命令。
 * @param byte DMA 缓冲读出的字节。
 * @note LF 后重置行状态；超长、NUL、非 ASCII 或行内 CR 后的字符使整行作废。
 */
static void ConsumeByte(uint8_t byte)
{
    if (byte == '\n')
    {
        if (command_discard == 0U && command_length > 0U)
        {
            command_buffer[command_length] = '\0';
            ExecuteCommand(command_buffer);
        }
        command_length = 0U;
        command_discard = 0U;
        command_cr = 0U;
        return;
    }
    if (command_discard != 0U) return;
    if (command_cr != 0U)
    {
        command_discard = 1U; /* CR 后只允许 LF，不能把 ST\rOP 当成 STOP。 */
    }
    else if (byte == '\r')
    {
        command_cr = 1U;
    }
    else if (byte < 0x20U || byte > 0x7EU || command_length >= COMMAND_SIZE - 1U)
    {
        command_discard = 1U;
    }
    else
    {
        command_buffer[command_length++] = (char)byte;
    }
}

/* 五、公开接口：主循环负责实际收发，中断仅请求或通知完成。 */
/** @brief 校验所有对象和 DMA 配置，建立初始状态并启动接收；参数见头文件。 */
HAL_StatusTypeDef FOC_COMM_USART_Init(UART_HandleTypeDef *uart,
                                     FOC_ENCODER_HandleTypeDef *encoder,
                                     FOC_LOOP_CUR_HandleTypeDef *loop_cur,
                                     FOC_LOOP_SPD_HandleTypeDef *loop_spd,
                                     FOC_LOOP_POS_HandleTypeDef *loop_pos)
{
    if (comm_uart != NULL) return HAL_BUSY;
    if (uart == NULL || encoder == NULL || loop_cur == NULL ||
        loop_spd == NULL || loop_pos == NULL || uart->hdmarx == NULL ||
        uart->hdmatx == NULL || uart->hdmarx->Init.Mode != DMA_CIRCULAR ||
        uart->hdmatx->Init.Mode != DMA_NORMAL) return HAL_ERROR;

    comm_uart = uart;
    comm_encoder = encoder;
    comm_cur = loop_cur;
    comm_spd = loop_spd;
    comm_pos = loop_pos;
    control_mode = FOC_CONTROL_SPEED;
    tx_busy = 0U;
    telemetry_pending = 0U;
    tx_last_tick = HAL_GetTick() - TX_PERIOD_MS;
    rx_restart_pending = 0U;
    rx_overrun_count = 0U;
    command_length = 0U;
    command_discard = 0U;
    command_cr = 0U;

    HAL_StatusTypeDef status = StartRxDMA();
    if (status != HAL_OK) comm_uart = NULL;
    return status;
}

/** @brief 读取全工程共用的模式；读取不会重置目标或积分。 */
FOC_CONTROL_MODE FOC_COMM_USART_GetControlMode(void)
{
    return control_mode;
}

/** @brief 在主循环切换模式，复位相关目标/积分；重复选择当前模式不清零目标。 */
void FOC_COMM_USART_SetControlMode(FOC_CONTROL_MODE mode)
{
    if (comm_uart == NULL || mode == control_mode ||
        (mode != FOC_CONTROL_SPEED && mode != FOC_CONTROL_POSITION)) return;

    /* 与控制中断互斥，结束时恢复原中断状态，不强行打开原本关闭的中断。
     * 这里只读取编码器计数并更新少量对象，不等待外设、不进行串口收发。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (mode == FOC_CONTROL_POSITION)
    {
        FOC_ENCODER_UpdateAngle(comm_encoder);
        FOC_LOOP_POS_SetPositionRef(comm_pos, comm_encoder->angle_m);
    }
    FOC_LOOP_SPD_SetSpeedRef(comm_spd, 0.0f);
    FOC_PI_Reset(&comm_spd->pi_spd);
    FOC_PI_Reset(&comm_pos->pi_pos);
    control_mode = mode;
    __set_PRIMASK(primask);
}

/** @brief ADC 中断只提交“有更新”标志，不在中断里打包或调用 HAL 发送。 */
void FOC_COMM_USART_RequestTelemetry(void)
{
    telemetry_pending = 1U;
}

/** @brief 主循环消费 RX DMA 字节并执行完整命令，异常时先恢复接收再重新同步。 */
void FOC_COMM_USART_ProcessRx(void)
{
    if (comm_uart == NULL) return;
    if (rx_restart_pending != 0U)
    {
        /* 只中止接收，不影响可能仍在进行的 TX DMA；失败留待下一轮处理。 */
        if (HAL_UART_AbortReceive(comm_uart) != HAL_OK) return;
        command_discard = 1U;
        rx_restart_pending = 0U;
        if (StartRxDMA() != HAL_OK) rx_restart_pending = 1U;
        return;
    }

    for (uint32_t i = 0U; i < RX_BUDGET; ++i)
    {
        if (rx_restart_pending != 0U) return;
        uint32_t received = GetRxCount();
        if ((uint32_t)(received - rx_read_count) > RX_DMA_SIZE)
        {
            rx_overrun_count++;
            rx_read_count = received;
            command_discard = 1U; /* 旧数据被覆盖，不能执行其中残存的半条命令。 */
            return;
        }
        if (received == rx_read_count) break;
        uint8_t byte = rx_dma_buffer[rx_read_count % RX_DMA_SIZE];
        if (rx_restart_pending != 0U) return;
        received = GetRxCount();
        if ((uint32_t)(received - rx_read_count) > RX_DMA_SIZE)
        {
            rx_overrun_count++;
            rx_read_count = received;
            command_discard = 1U; /* 读字节期间被高优先级中断打断，也要检查覆盖。 */
            return;
        }
        rx_read_count++;
        ConsumeByte(byte);
    }
}

/** @brief 主循环把最新十个值打包为 JustFloat 帧，并非阻塞提交 TX DMA。 */
void FOC_COMM_USART_ProcessTx(void)
{
    if (comm_uart == NULL || telemetry_pending == 0U) return;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - tx_last_tick) < TX_PERIOD_MS) return;
    telemetry_pending = 0U; /* 遥测只关心最新值；忙时丢帧，不累积旧帧。 */
    if (tx_busy != 0U) return;

    float values[TX_CHANNELS];
    const volatile FOC_LOOP_SPD_HandleTypeDef *spd = comm_spd;
    const volatile FOC_LOOP_CUR_HandleTypeDef *cur = comm_cur;
    uint32_t primask = __get_PRIMASK();
    __disable_irq(); /* 只复制快照；防止同一帧混入控制中断更新到一半的数据。 */
    values[0] = spd->speed_ref;               /* CH0：目标转速，rpm。 */
    values[1] = spd->speed_fbk;               /* CH1：反馈转速，rpm。 */
    values[2] = cur->id_ref;                  /* CH2：d 轴目标电流，A。 */
    values[3] = cur->i_d;                     /* CH3：d 轴反馈电流，A。 */
    values[4] = cur->iq_ref;                  /* CH4：q 轴目标电流，A。 */
    values[5] = cur->i_q;                     /* CH5：q 轴反馈电流，A。 */
    values[6] = cur->v_d;                     /* CH6：d 轴电压，V。 */
    values[7] = cur->v_q;                     /* CH7：q 轴电压，V。 */
    values[8] = ((volatile FOC_LOOP_POS_HandleTypeDef *)comm_pos)->position_ref;
    values[9] = ((volatile FOC_ENCODER_HandleTypeDef *)comm_encoder)->angle_m;
    __set_PRIMASK(primask);
    values[8] *= 180.0f / FOC_PI;             /* CH8：目标单圈机械角，度。 */
    values[9] *= 180.0f / FOC_PI;             /* CH9：实际单圈机械角，度。 */

    /* STM32G4 的 float32 为小端 IEEE754，与 PC 的 struct.unpack("<10f") 对应。
     * memcpy 避免类型别名和对齐问题；DMA 持有的是静态 tx_buffer。 */
    memcpy(tx_buffer, values, sizeof(values));
    tx_buffer[40] = 0x00U;
    tx_buffer[41] = 0x00U;
    tx_buffer[42] = 0x80U;
    tx_buffer[43] = 0x7FU;
    tx_busy = 1U; /* 必须在提交前置忙，防止完成回调先于此处清忙。 */
    if (HAL_UART_Transmit_DMA(comm_uart, tx_buffer, TX_FRAME_SIZE) == HAL_OK)
    {
        tx_last_tick = now;
    }
    else
    {
        tx_busy = 0U; /* 提交失败，DMA 未持有本帧；下一次请求可以重试。 */
    }
}

/* 六、HAL 回调：HAL 全局回调只能有一份，全部检查 UART 归属。 */
/** @brief RX DMA 满一圈回调：只累计圈数，循环 DMA 无需重启。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (comm_uart != NULL && uart == comm_uart) rx_wrap_count++;
}

/** @brief TX 完成回调：HAL 已结束本帧发送，释放静态缓冲供主循环复用。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (comm_uart != NULL && uart == comm_uart) tx_busy = 0U;
}

/**
 * @brief UART/DMA 错误回调：接收终止则请求恢复；发送终止则释放发送缓冲。
 * @param uart HAL 报错的 UART；其他 UART 不处理。
 * @note 不在中断中 abort/restart 或等待。损坏的当前命令作废到下一个 LF。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (comm_uart == NULL || uart != comm_uart) return;
    if (uart->RxState == HAL_UART_STATE_READY) rx_restart_pending = 1U;
    if (uart->gState == HAL_UART_STATE_READY) tx_busy = 0U;
}
