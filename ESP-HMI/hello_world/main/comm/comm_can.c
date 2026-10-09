#include "comm_can.h"
#include <stdbool.h>   // 提供bool类型
#include "esp_twai.h"          // TWAI通用驱动接口
#include "esp_twai_onchip.h"   // ESP32片上TWAI控制器接口
#include "freertos/FreeRTOS.h"   // FreeRTOS基础接口
#include "freertos/queue.h"      // FreeRTOS队列接口
#include "esp_attr.h"
#define CAN_TX_GPIO GPIO_NUM_5   // TWAI发送引脚
#define CAN_RX_GPIO GPIO_NUM_6   // TWAI接收引脚
static twai_node_handle_t can_node = NULL;   // 保存TWAI控制器句柄
static QueueHandle_t can_rx_queue = NULL;   // 创建队列，使用队列保存接收到的CAN报文
static uint8_t can_tx_data[CAN_FRAME_DLC] = {0};   // 8字节发送缓冲区
static bool can_tx_pending = false;               // 上一帧是否尚未完成发送
static twai_frame_t can_tx_frame = {
    .header.id = CAN_ID_CMD,         // CAN ID为0x100
    .header.dlc = CAN_FRAME_DLC,     // 数据长度为8字节
    .buffer = can_tx_data,           // 指向发送缓冲区,这里保存的是can_tx_data数组的首地址
    .buffer_len = CAN_FRAME_DLC      // 缓冲区容量
};


/* ==================== CAN控制命令打包 ==================== */
void comm_can_pack_command(CAN_Command_t cmd, uint16_t param, uint8_t data[CAN_FRAME_DLC])
{
    for (uint8_t i = 0; i < CAN_FRAME_DLC; i++)
    {
        data[i] = 0;   // 清空8字节数据区
    }

    data[CAN_CMD_INDEX] = (uint8_t)cmd;                 // Byte0：命令编号
    data[CAN_PARAM_L_INDEX] = (uint8_t)(param & 0xFFU); // Byte1：参数低8位,0xFFU是为了取出低8位
    data[CAN_PARAM_H_INDEX] = (uint8_t)(param >> 8);    // Byte2：参数高8位
}
/* ==================== CAN控制命令发送 ==================== */
esp_err_t comm_can_send_command(CAN_Command_t cmd, uint16_t param)
{
    if (can_node == NULL) return ESP_ERR_INVALID_STATE;   // 检查CAN是否初始化
    if (can_tx_pending)
    {
        esp_err_t ret = twai_node_transmit_wait_all_done(can_node, 10);   // 等待上一帧发送完成
        if (ret != ESP_OK) return ret;                                    // 等待失败则返回
        can_tx_pending = false;                                           // 允许重用缓冲区
    }
    comm_can_pack_command(cmd, param, can_tx_data);   // 打包控制命令
    esp_err_t ret = twai_node_transmit(can_node, &can_tx_frame, 0);   // 提交CAN发送请求
    if (ret == ESP_OK) can_tx_pending = true;                        // 记录尚未确认发送完成
    return ret;   // 返回发送请求提交结果
}

/* ==================== CAN接收回调 ==================== */
static bool IRAM_ATTR can_rx_callback(const twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    (void)edata;       // 暂时不使用事件数据
    (void)user_ctx;    // 暂时不使用用户参数
    BaseType_t task_woken = pdFALSE;   // 记录是否需要唤醒其他任务
    CAN_RxMessage_t message = {0};     // 创建一条接收消息
    twai_frame_t rx_frame = {
        .buffer = message.data,        // 将rx报文的数据部分指针指向我们创建的message结构体的数组成员
        .buffer_len = sizeof(message.data)   // 接收缓冲区大小
    };
    if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK)//调用接收函数，将接收到的CAN报文保存到rx_frame的buffer指针指向的message.data数组中
    {
        message.id = rx_frame.header.id;       // 保存CAN报文ID
        message.d_length = rx_frame.header.dlc;     // 保存数据长度

        if (can_rx_queue != NULL)
        {
            xQueueSendFromISR(can_rx_queue, &message, &task_woken);   // 将消息复制到队列
        }
    }

    return task_woken == pdTRUE;   // 告诉系统是否需要切换任务
}
//也就是说twai_frame_t的buffer是一个指针变量，将其指向了我们创建的缓冲区数组。然后调用twai_node_receive_from_isr将接收到的数据传入rx_frame的buffer指针指向的message.data数组中。再然后将报文id和数据长度赋值给message的id和长度成员。先让 rx_frame.buffer 指向 message.data，接收函数通过这个指针写入数据，并填充 rx_frame.header；我们再把 ID 和长度复制给 message，最后把完整的 message 放入队列。


/* ==================== CAN控制器初始化 ==================== */
esp_err_t comm_can_init(void)
{
    if (can_node != NULL) return ESP_OK;   // 避免重复初始化

    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = CAN_TX_GPIO,                  // TWAI发送引脚
        .io_cfg.rx = CAN_RX_GPIO,                  // TWAI接收引脚
        .io_cfg.quanta_clk_out = GPIO_NUM_NC,      // 不使用时钟输出引脚
        .io_cfg.bus_off_indicator = GPIO_NUM_NC,   // 不使用Bus-Off指示引脚
        .bit_timing.bitrate = 500000,              // CAN波特率500kbit/s
        .tx_queue_depth = 5,                       // 发送队列最多存放5帧
        .fail_retry_cnt = 3                        // 发送失败时最多重试3次
    };
    esp_err_t ret = twai_new_node_onchip(&node_config, &can_node);   // 创建TWAI节点
    if (ret != ESP_OK) return ret;                                  // 创建失败则返回
    can_rx_queue = xQueueCreate(16, sizeof(CAN_RxMessage_t));   // 创建接收队列，最多保存16帧
    if (can_rx_queue == NULL)
    {
        twai_node_delete(can_node);   // 创建接收队列失败则释放资源
        can_node = NULL;              // 清空句柄
        return ESP_ERR_NO_MEM;       // 返回内存不足错误码
    }
    twai_event_callbacks_t callbacks = {
    .on_rx_done = can_rx_callback   // 收到CAN报文时执行回调
    };
    ret = twai_node_register_event_callback(can_node, &callbacks);   // 注册CAN接收回调
    if (ret != ESP_OK)
    {
        vQueueDelete(can_rx_queue);   // 删除接收队列
        can_rx_queue = NULL;          // 清空接收队列句柄
        twai_node_delete(can_node);     // 启动失败则释放资源
        can_node = NULL;                // 清空句柄
        return ret;                     // 返回错误码
    }
    ret = twai_node_enable(can_node);   // 启动TWAI控制器
    if (ret != ESP_OK)
    {
        vQueueDelete(can_rx_queue);   // 删除接收队列
        can_rx_queue = NULL;          // 清空接收队列句柄
        twai_node_delete(can_node);     // 启动失败则释放资源
        can_node = NULL;                // 清空句柄
        return ret;                     // 返回错误码
    }
    return ESP_OK;   // 初始化成功
}//1.配置CAN结构体。2.创建TWAI节点和接收队列

