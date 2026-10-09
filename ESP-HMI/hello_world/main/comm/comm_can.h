#ifndef COMM_CAN_H
#define COMM_CAN_H

#include <stdint.h>
#include "can_protocol.h"
#include "esp_err.h"   // ESP-IDF错误码定义
#include "stdbool.h"

esp_err_t comm_can_init(void);   // 初始化CAN控制器
/* 将控制命令及参数打包成8字节CAN数据 */
void comm_can_pack_command(CAN_Command_t cmd, uint16_t param, uint8_t data[CAN_FRAME_DLC]);
esp_err_t comm_can_send_command(CAN_Command_t cmd, uint16_t param);   // 发送CAN控制命令

typedef struct
{
    uint32_t id;   // 接收CAN报文ID
    uint8_t d_length;   // 接收CAN报文数据长度
    uint8_t data[CAN_FRAME_DLC];   // 接收CAN报文数据
} CAN_RxMessage_t;//can发来一条CAN报文，我们就可以将其ID,数据长度和8字节内容放入结构体

bool comm_can_receive(CAN_RxMessage_t *message);   // 从接收队列中取出一条报文

#endif /* COMM_CAN_H */