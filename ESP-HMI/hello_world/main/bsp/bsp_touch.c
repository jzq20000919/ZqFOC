#include "bsp_touch.h"
#include "bsp_XL9555.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"   // ESP-IDF日志接口
#include <stdbool.h>
#include <stdint.h>
#define TOUCH_I2C_ADDR 0x2E   // CHSC5432的I2C地址
#define TOUCH_ID_ADDR 0x20000080UL   // CHSC5432芯片ID寄存器
static i2c_master_dev_handle_t touch_dev = NULL;   // 保存触摸设备句柄
static bool touch_addr_little_endian = true;        // 记录寄存器地址字节序
/* ==================== 触摸芯片检测 ==================== */
/* ==================== 触摸芯片初始化与检测 ==================== */
esp_err_t BSP_touch_probe(void)
{
    i2c_master_bus_handle_t bus = BSP_XL9555_GetI2CBus();   // 获取共享I2C总线
    if (bus == NULL) return ESP_ERR_INVALID_STATE;          // 检查总线是否初始化

    esp_err_t ret = BSP_XL9555_SetTouchReset(true);   // 拉低RST，复位触摸芯片
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(20));   // 保持复位20ms

    ret = BSP_XL9555_SetTouchReset(false);   // 拉高RST，释放复位
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(80));   // 等待触摸芯片启动

    ret = i2c_master_probe(bus, TOUCH_I2C_ADDR, 1000);   // 检测I2C地址0x2E
    if (ret != ESP_OK) return ret;

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,   // 7位I2C地址
        .device_address = TOUCH_I2C_ADDR,        // CHSC5432地址
        .scl_speed_hz = 400000                   // I2C频率400kHz
    };

    ret = i2c_master_bus_add_device(bus, &dev_config, &touch_dev);   // 注册触摸设备
    if (ret != ESP_OK) return ret;

    uint8_t reg_addr[4] = {0x80, 0x00, 0x00, 0x20};   // ID寄存器地址0x20000080，小端格式
    uint8_t chip_id[4] = {0};                       // 保存读取的芯片ID

    ret = i2c_master_transmit_receive(touch_dev, reg_addr, 4, chip_id, 4, 1000);   // 读取4字节芯片ID
    if (ret != ESP_OK) return ret;

    bool all_zero = true;
    bool all_ff = true;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (chip_id[i] != 0x00) all_zero = false;   // 判断是否全部为0
        if (chip_id[i] != 0xFF) all_ff = false;     // 判断是否全部为FF
    }

    if (all_zero || all_ff) return ESP_ERR_INVALID_RESPONSE;   // 排除明显无效的ID

    ESP_LOGI("TOUCH", "CHSC5432 ID: %02X %02X %02X %02X",
             chip_id[0], chip_id[1], chip_id[2], chip_id[3]);   // 输出芯片ID

    return ESP_OK;   // 初始化与检测成功
}