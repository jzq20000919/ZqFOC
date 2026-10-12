/** @file Ui_position.h
 *  @brief LVGL单圈位置页面；目标与反馈使用0.01度单位，范围0～359.99度。
 */
#ifndef UI_POSITION_H
#define UI_POSITION_H

#include "lvgl.h"
#include "comm_can.h"   // 复用CAN解析结果，不创建第二条接收队列

/** @brief 在LVGL锁内创建位置页面；由main先于速度页面调用，不自动启动电机。 */
void UI_Position_Create(lv_display_t *display);
/** @brief 在LVGL上下文显示位置页面，只改变显示，不发送控制命令。 */
void UI_Position_Show(void);
/** @brief 由共用的50ms定时器更新CAN在线状态，未收到有效反馈时保持离线。 */
void UI_Position_SetCanOnline(bool online);
/** @brief 由共用定时器更新电机真实运行状态；status不可为空。 */
void UI_Position_UpdateStatus(const CAN_Status_t *status);
/** @brief 由共用定时器更新Iq、Id的目标和实际电流，单位A。 */
void UI_Position_UpdateCurrent(const CAN_Current_t *current);
/** @brief 由共用定时器更新实际角度及STM32目标角度，上方滑块选择值保持不变。 */
void UI_Position_UpdatePosition(const CAN_Position_t *position);

#endif /* UI_POSITION_H */

