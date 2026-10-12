/** @file Ui_speed.h
 *  @brief LVGL速度页面及两页共用的CAN接收定时器接口。
 */
#ifndef UI_SPEED_H
#define UI_SPEED_H

#include "lvgl.h"

/** @brief 在LVGL锁内创建速度页面和唯一的50ms CAN接收定时器；先创建位置页面。 */
void UI_Speed_Create(lv_display_t *display);
/** @brief 在LVGL上下文显示速度页面，只切换页面，不发送电机控制命令。 */
void UI_Speed_Show(void);

#endif /* UI_SPEED_H */
