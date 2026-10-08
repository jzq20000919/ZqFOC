#include "esp_err.h"
#include "esp_lvgl_port.h"

#include "bsp_board.h"
#include "bsp_lcd.h"
#include "motor_ui.h"
#include "comm_can.h"

void app_main(void)
{
    ESP_ERROR_CHECK(bsp_board_init());                   // 初始化I2C和XL9555

    lv_display_t *display = bsp_lcd_init();              // 初始化LCD和LVGL

    ESP_ERROR_CHECK(comm_can_init());                    // 初始化CAN控制器

    lvgl_port_lock(0);                                   // 获取LVGL锁
    motor_ui_create(display);                            // 创建UI
    lvgl_port_unlock();                                  // 释放LVGL锁

    ESP_ERROR_CHECK(bsp_board_set_lcd_backlight(true));  // 打开LCD背光
}