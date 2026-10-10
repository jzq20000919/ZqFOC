#include "bsp_touch.h"
#include "bsp_XL9555.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"   // ESP-IDF日志接口
#include <stdbool.h>
#include <stdint.h>
#define TOUCH_I2C_ADDR 0x2E   // CHSC5432的I2C地址
static i2c_master_dev_handle_t touch_dev = NULL;   // 保存触摸设备句柄

/* ==================== 触摸芯片初始化 ==================== */
esp_err_t BSP_touch_init(void)
{
    if (touch_dev != NULL) return ESP_OK;   // 避免重复初始化
    i2c_master_bus_handle_t bus = BSP_XL9555_GetI2CBus();   // 获取共享I2C总线
    if (bus == NULL) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = BSP_XL9555_SetTouchReset(true);   // 拉低RST，复位触摸芯片
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(20));   // 保持复位20ms
    ret = BSP_XL9555_SetTouchReset(false);   // 拉高RST，释放复位
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(80));   // 等待触摸芯片启动
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,   // 使用7位I2C地址
        .device_address = TOUCH_I2C_ADDR,        // CHSC5432地址0x2E
        .scl_speed_hz = 400000                   // I2C频率400kHz
    };
    return i2c_master_bus_add_device(bus, &dev_config, &touch_dev);   // 注册触摸设备
}