#ifndef BSP_TOUCH_H
#define BSP_TOUCH_H

#include "esp_err.h"
#include "lvgl.h"   // LVGL显示器和输入设备类型

/** @brief 复用XL9555的I2C总线，复位并注册CHSC5432设备；先调用BSP_XL9555_Init()。 */
esp_err_t BSP_touch_init(void);
/** @brief 将触摸设备注册到指定LVGL显示器；先初始化触摸和显示器，调用时持有LVGL锁。 */
esp_err_t BSP_touch_register_lvgl(lv_display_t *display);

#endif /* BSP_TOUCH_H */
