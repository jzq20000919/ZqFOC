/**
 * @file foc_comm_usart.h
 * @brief USART2 Modbus RTU基础接口；IOC配置2 Mbps、8N2和循环RX DMA。
 *
 * 03读寄存器，06写模式/指令，10写一个float。设备地址为1。
 * 所有地址从0开始；寄存器高字节在前；float高16位寄存器在前。
 *
 * 0000      电机状态：0 STOP、1 ALIGN、2速度运行、3位置运行，只读。
 * 0001      控制模式：0速度、1位置，只允许STOP中切换。
 * 0002      操作指令：1 START、2 STOP、3 GOZERO，只写。
 * 0010-0011 目标转速，float，rpm。
 * 0012-0013 目标位置，float，度。
 * 0100-0101 最新完整批次的起始时间，uint32，ms，高16位在前。
 * 0102-0165 按时间顺序排列的5组采样，每组10个float，采样间隔1ms：
 *           转速目标/反馈、Id目标/反馈、Iq目标/反馈、Vd/Vq、位置目标/反馈。
 *           单位依次为rpm、A、V、度。读取0100起的102个寄存器可取得一批。
 *
 * 无请求不发送；重复时间戳表示同一批，跳过批次表示电脑读取落后。
 * 仅实现约定地址及功能；无效请求丢弃，不做重试或自动恢复。
 * 写指令回复表示已收到指令，实际运行状态通过0000读取。
 */
#ifndef FOC_COMM_USART_H
#define FOC_COMM_USART_H

#include "stm32g4xx_hal.h"

// 保留CAN和按键共用的模式接口，不增加另一套模式变量。
typedef enum
{
    FOC_CONTROL_SPEED = 0,
    FOC_CONTROL_POSITION = 1
} FOC_CONTROL_MODE;

/**
 * @brief  启动USART2循环DMA接收并允许采样，外设和FOC初始化后调用一次。
 * @return 无。
 */
void Modbus_StartReceive(void);

/**
 * @brief  在主循环中处理完整Modbus请求，执行指令并回复，不阻塞等待。
 * @return 无。
 */
void Modbus_Task(void);

/**
 * @brief  由SysTick每1ms调用，记录一组数据，每5组组成一个批次。
 * @return 无。
 */
void Modbus_Sample(void);

/**
 * @brief  读取当前控制模式。
 * @return FOC_CONTROL_SPEED或FOC_CONTROL_POSITION。
 */
FOC_CONTROL_MODE FOC_COMM_USART_GetControlMode(void);

/**
 * @brief  在STOPPED状态选择控制模式，保留目标值。
 * @param[in] mode FOC_CONTROL_SPEED为速度模式，FOC_CONTROL_POSITION为位置模式。
 * @return 无。
 */
void FOC_COMM_USART_SetControlMode(FOC_CONTROL_MODE mode);

#endif
