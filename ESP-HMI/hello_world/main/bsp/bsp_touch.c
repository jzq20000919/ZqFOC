#include "bsp_touch.h"
#include "bsp_XL9555.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"   // ESP-IDF日志接口
#include <stdbool.h>
#include <stdint.h>
#define TOUCH_I2C_ADDR 0x2E   // CHSC5432的I2C地址
#define TOUCH_EVENT_SIZE 28U   // 每次读取完整的28字节触摸事件
#define TOUCH_H_RES      320U   // 横屏触摸坐标宽度
#define TOUCH_V_RES      240U   // 横屏触摸坐标高度
static i2c_master_dev_handle_t touch_dev = NULL;   // 保存触摸设备句柄
static lv_point_t touch_last_point = {0};   // 保存上一次有效触摸坐标
static lv_indev_t *touch_indev = NULL;   // LVGL触摸输入设备句柄
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
/* ==================== 读取触摸坐标 ==================== */
static bool BSP_touch_ReadPoint(uint16_t *x, uint16_t *y)
{
    if (touch_dev == NULL || x == NULL || y == NULL) return false;   // 检查参数
    uint8_t reg_addr[4] = {0x2C, 0x00, 0x00, 0x20};   // 事件寄存器0x2000002C，小端格式
    uint8_t data[TOUCH_EVENT_SIZE] = {0};            // 接收触摸事件数据
    esp_err_t ret = i2c_master_transmit_receive(touch_dev, reg_addr, 4, data, sizeof(data), 20);   // 读取触摸数据
    if (ret != ESP_OK) return false;
    uint8_t touch_count = data[1] & 0x0F;   // 获取触摸点数量
    if (touch_count == 0 || touch_count > 5) return false;
    uint16_t raw_x = ((uint16_t)(data[5] >> 4) << 8) | data[3];     // 解析X坐标
    uint16_t raw_y = ((uint16_t)(data[5] & 0x0F) << 8) | data[2];   // 解析Y坐标
    if (raw_x >= TOUCH_H_RES || raw_y >= TOUCH_V_RES) return false;   // 检查坐标范围
    *x = raw_x;                         // X坐标
    *y = TOUCH_V_RES - 1U - raw_y;       // Y坐标翻转
    return true;   // 成功读取有效触摸坐标
}

/* ==================== LVGL触摸读取回调 ==================== */
static void BSP_touch_ReadCallback(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;   // 暂时不使用输入设备参数
    uint16_t x = 0;
    uint16_t y = 0;
    if (BSP_touch_ReadPoint(&x, &y))
    {
        touch_last_point.x = x;                  // 保存X坐标
        touch_last_point.y = y;                  // 保存Y坐标
        data->point = touch_last_point;          // 将触摸坐标传给LVGL
        data->state = LV_INDEV_STATE_PRESSED;    // 告诉LVGL手指已按下
    }
    else
    {
        data->point = touch_last_point;          // 保留上一次有效坐标
        data->state = LV_INDEV_STATE_RELEASED;   // 告诉LVGL手指已释放
    }
}

/* ==================== 注册LVGL触摸输入设备 ==================== */
esp_err_t BSP_touch_register_lvgl(lv_display_t *display)
{
    if (display == NULL) return ESP_ERR_INVALID_ARG;   // 检查显示器
    if (touch_dev == NULL) return ESP_ERR_INVALID_STATE;   // 检查触摸芯片是否初始化
    if (touch_indev != NULL) return ESP_OK;   // 避免重复注册
    touch_indev = lv_indev_create();   // 创建LVGL输入设备
    if (touch_indev == NULL) return ESP_ERR_NO_MEM;
    lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);   // 设置为触摸指针设备
    lv_indev_set_read_cb(touch_indev, BSP_touch_ReadCallback);   // 注册触摸读取回调
    lv_indev_set_display(touch_indev, display);   // 关联LCD显示器
    lv_timer_t *timer = lv_indev_get_read_timer(touch_indev);   // 获取输入设备读取定时器
    if (timer != NULL)
    {
        lv_timer_set_period(timer, 10);   // 每10ms读取一次触摸数据
    }
    return ESP_OK;
}
