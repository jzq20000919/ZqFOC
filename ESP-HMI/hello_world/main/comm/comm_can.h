#ifndef COMM_CAN_H
#define COMM_CAN_H

#include <stdint.h>
#include "can_protocol.h"
#include "esp_err.h"   // ESP-IDF错误码定义
#include "stdbool.h"

esp_err_t comm_can_init(void);   // 初始化CAN控制器
/* 将控制命令及参数打包成8字节CAN数据 */
void comm_can_pack_command(CAN_Command_t cmd, uint16_t param, uint8_t data[CAN_FRAME_DLC]);
/** @brief 提交CAN控制命令；ESP_OK仅表示驱动接受请求，不代表总线发送成功。仅从同一个LVGL任务调用，不支持多个任务同时发送。 */
esp_err_t comm_can_send_command(CAN_Command_t cmd, uint16_t param);   // 发送CAN控制命令

typedef struct
{
    uint32_t id;   // 接收CAN报文ID
    uint8_t d_length;   // 接收CAN报文数据长度
    uint8_t data[CAN_FRAME_DLC];   // 接收CAN报文数据
} CAN_RxMessage_t;//can发来一条CAN报文，我们就可以将其ID,数据长度和8字节内容放入结构体

/* ==================== 电机状态信息 ==================== */
typedef struct
{
    CAN_State_t state;           // 电机运行状态
    CAN_Mode_t mode;             // 当前控制模式
} CAN_Status_t;//存放解析之后的结果。
/* ==================== 电机速度信息 ==================== */
typedef struct
{
    int16_t actual_speed_rpm;   // 电机实际转速
    int16_t target_speed_rpm;   // 电机目标转速
} CAN_Speed_t;
bool comm_can_receive(CAN_RxMessage_t *message);   // 从接收队列中取出一条报文
bool comm_can_parse_status(const CAN_RxMessage_t *message, CAN_Status_t *status);   // 解析电机状态报文
bool comm_can_parse_current(const CAN_RxMessage_t *message, CAN_Current_t *current);   // 解析电流报文
bool comm_can_parse_speed(const CAN_RxMessage_t *message, CAN_Speed_t *speed);   // 解析速度报文
#endif /* COMM_CAN_H */
