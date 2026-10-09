#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H
#include <stdint.h>   // 提供标准整数类型
/* ==================== CAN通信参数 ==================== */
#define CAN_FRAME_DLC       8U       // 每帧数据长度为8字节
/* ==================== CAN报文ID ==================== */
#define CAN_ID_CMD          0x100U   // ESP32 → STM32：电机控制命令
#define CAN_ID_STATUS       0x180U   // STM32 → ESP32：电机状态和速度
#define CAN_ID_POSITION     0x181U   // STM32 → ESP32：电机位置
#define CAN_ID_CURRENT      0x182U   // STM32 → ESP32：电机电流
/* ==================== CAN命令类型 ==================== */

/* ==================== CAN控制命令 ==================== */
typedef enum
{
    CAN_CMD_SET_MODE     = 0x01,   // 设置控制模式
    CAN_CMD_SET_SPEED    = 0x02,   // 设置目标速度
    CAN_CMD_SET_POSITION = 0x03,   // 设置目标位置
    CAN_CMD_START        = 0x04,   // 启动电机
    CAN_CMD_STOP         = 0x05,   // 停止电机
    CAN_CMD_CLEAR_FAULT  = 0x06    // 清除故障
} CAN_Command_t;
/* ==================== 电机运行状态 ==================== */
typedef enum
{
    CAN_STATE_STOPPED       = 0,   // 电机停止
    CAN_STATE_ALIGNING      = 1,   // 电机对齐中
    CAN_STATE_ALIGN_RELEASE = 2,   // 对齐释放阶段
    CAN_STATE_RUNNING       = 3    // 电机运行中
} CAN_State_t;
/* ==================== 电机控制模式 ==================== */
typedef enum
{
    CAN_MODE_SPEED    = 0,   // 速度控制模式
    CAN_MODE_POSITION = 1    // 位置控制模式
} CAN_Mode_t;
/* ==================== 控制帧字节位置 ==================== */
#define CAN_CMD_INDEX       0U   // Byte0：控制命令编号
#define CAN_PARAM_L_INDEX   1U   // Byte1：参数低8位
#define CAN_PARAM_H_INDEX   2U   // Byte2：参数高8位
/* ==================== 参数单位 ==================== */
#define CAN_SPEED_UNIT_RPM  1U     // 速度参数单位：1 rpm
#define CAN_POSITION_SCALE  100U   // 位置参数：1°对应100个计数
#endif /* CAN_PROTOCOL_H */
