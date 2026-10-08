#include "bsp_XL9555.h"
#include "driver/i2c_master.h"
#include <stdint.h>   // 提供 uint8_t 类型
#define BOARD_I2C_PORT        I2C_NUM_0/* 使用 ESP32-S3 的 I2C0 控制器 */
#define BOARD_I2C_SDA         GPIO_NUM_48/* 板载 I2C 数据线 SDA */
#define BOARD_I2C_SCL         GPIO_NUM_45/* 板载 I2C 时钟线 SCL */
#define BOARD_I2C_FREQ_HZ     400000/* I2C 通信频率：400 kHz */
/* ==================== XL9555 参数 ==================== */
#define XL9555_I2C_ADDR       0x20/* XL9555 在 I2C 总线上的设备地址 */
#define XL9555_OUTPUT_PORT0   0x02/* XL9555 输出寄存器 Port0 */
#define XL9555_CONFIG_PORT0   0x06/* XL9555 配置寄存器 Port0 */
#define XL9555_LCD_BL_MASK    (1U << 7)/* LCD 背光连接在 XL9555 的 P0.7 */

static i2c_master_bus_handle_t i2c_bus = NULL;//i2c句柄
static i2c_master_dev_handle_t xl9555_dev = NULL;//XL9555 设备句柄

//i2c总线初始化函数
static esp_err_t BSP_I2C_Init(void)
{
    const i2c_master_bus_config_t i2c_bus_config ={ 
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = BOARD_I2C_PORT,
        .scl_io_num = BOARD_I2C_SCL,
        .sda_io_num = BOARD_I2C_SDA,
        .glitch_ignore_cnt =7,
        .flags.enable_internal_pullup = true
    };
    return i2c_new_master_bus(
        &i2c_bus_config,
        &i2c_bus
    );
}
/* ============================================================
 * 将 XL9555 加入 I2C 总线
 * ============================================================ */
static esp_err_t BSP_XL9555_I2C_LINK(void)
{
    const i2c_device_config_t device_config =
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,       // 使用7位I2C地址
        .device_address = XL9555_I2C_ADDR,           // XL9555地址为0x20
        .scl_speed_hz = BOARD_I2C_FREQ_HZ            // 通信速度400kHz
    };

    return i2c_master_bus_add_device(
        i2c_bus,                                     // XL9555挂在哪条I2C总线上
        &device_config,                              // XL9555的通信参数
        &xl9555_dev                                  // 返回XL9555设备句柄
    );
}

//写寄存器函数
static esp_err_t XL9555_WriteRegister(uint8_t reg, uint8_t data)
{
    uint8_t tx_buffer[2] =
    {
        reg,    // 第1个字节：寄存器地址
        data    // 第2个字节：要写入的数据
    };

    return i2c_master_transmit(
        xl9555_dev,            // 要通信的设备：XL9555
        tx_buffer,             // 要发送的数据
        sizeof(tx_buffer),     // 一共发送2字节
        1000                   // 超时时间1000 ms
    );
}

//读寄存器函数
static esp_err_t XL9555_ReadRegister(uint8_t reg, uint8_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;   // 防止传入空指针

    return i2c_master_transmit_receive(
        xl9555_dev,           // 要通信的设备：XL9555
        &reg,                  // 先发送要读取的寄存器地址
        sizeof(reg),           // 发送1字节寄存器地址
        data,                  // 读取的数据保存到这里
        1,                     // 读取1字节
        1000                   // 超时时间1000 ms
    );
}

/* 将 XL9555 的 P0.7 配置为输出，用于控制 LCD 背光 */
static esp_err_t BSP_XL9555_ConfigBacklightPin(void)
{
    uint8_t config_value = 0;
    esp_err_t ret = XL9555_ReadRegister(XL9555_CONFIG_PORT0, &config_value);   // 读取Port0方向配置
    if (ret != ESP_OK) return ret;                                            // 读取失败则直接返回
    config_value &= ~XL9555_LCD_BL_MASK;                                      // 将bit7清0，P0.7设置为输出
    return XL9555_WriteRegister(XL9555_CONFIG_PORT0, config_value);            // 写回配置寄存器
}

/* 控制 LCD 背光开关 */
esp_err_t BSP_XL9555_SetBacklight(bool enabled)
{
    uint8_t output_value = 0;
    esp_err_t ret = XL9555_ReadRegister(XL9555_OUTPUT_PORT0, &output_value);   // 读取当前Port0输出状态
    if (ret != ESP_OK) return ret;                                             // 读取失败则直接返回
    if (enabled)
    {
        output_value |= XL9555_LCD_BL_MASK;                                    // bit7置1，打开背光
    }
    else
    {
        output_value &= ~XL9555_LCD_BL_MASK;                                   // bit7清0，关闭背光
    }
    return XL9555_WriteRegister(XL9555_OUTPUT_PORT0, output_value);             // 将新状态写回XL9555
}

/* 初始化板级I2C和XL9555 */
esp_err_t BSP_XL9555_Init(void)
{
    esp_err_t ret;

    ret = BSP_I2C_Init();                     // 创建ESP32-S3的I2C总线
    if (ret != ESP_OK) return ret;                // 创建失败则直接返回错误
    ret = BSP_XL9555_I2C_LINK();                 // 将XL9555加入I2C总线
    if (ret != ESP_OK) return ret;                // 添加失败则直接返回错误
    ret = BSP_XL9555_ConfigBacklightPin();         // 将XL9555的P0.7配置为输出
    if (ret != ESP_OK) return ret;                // 配置失败则直接返回错误

    return ESP_OK;                                // 全部初始化成功
}