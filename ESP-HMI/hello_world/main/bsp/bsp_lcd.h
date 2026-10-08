#ifndef BSP_LCD_H
#define BSP_LCD_H

#include "lvgl.h"

/**
 * 初始化 LCD 和 LVGL，返回显示器句柄。
 */
lv_display_t *bsp_lcd_init(void);

#endif
