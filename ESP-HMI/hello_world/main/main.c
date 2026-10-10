#include "esp_err.h"
#include "esp_lvgl_port.h"

#include "bsp_XL9555.h"
#include "bsp_lcd.h"
#include "motor_ui.h"
#include "comm_can.h"
#include "bsp_touch.h"
#include "esp_log.h"
void app_main(void)
{
    ESP_ERROR_CHECK(BSP_XL9555_Init());                   // 初始化I2C和XL9555
    esp_err_t touch_ret = BSP_touch_probe();   // 检测触摸芯片
    ESP_LOGI("TOUCH", "Probe result: %s", esp_err_to_name(touch_ret));   // 输出检测结果
    lv_display_t *display = bsp_lcd_init();              // 初始化LCD和LVGL
    ESP_ERROR_CHECK(comm_can_init());                    // 初始化CAN控制器
    lvgl_port_lock(0);   // 获取LVGL锁
    motor_ui_create(display);   // 创建速度控制页面
    if (touch_ret == ESP_OK)
    {
        touch_ret = BSP_touch_register_lvgl(display);   // 注册触摸输入设备
    }
    lvgl_port_unlock();   // 释放LVGL锁
    if (touch_ret != ESP_OK)
    {
        ESP_LOGE("TOUCH", "Touch initialization failed: %s", esp_err_to_name(touch_ret));
    }
    ESP_ERROR_CHECK(BSP_XL9555_SetBacklight(true));  // 打开LCD背光
}
