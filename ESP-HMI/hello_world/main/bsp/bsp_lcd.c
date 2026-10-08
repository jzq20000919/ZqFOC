#include "bsp_lcd.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lvgl_port.h"

#define LCD_H_RES 320
#define LCD_V_RES 240

#define LCD_GPIO_CS GPIO_NUM_1
#define LCD_GPIO_DC GPIO_NUM_2
#define LCD_GPIO_RD GPIO_NUM_41
#define LCD_GPIO_WR GPIO_NUM_42
#define LCD_GPIO_D0 GPIO_NUM_40
#define LCD_GPIO_D1 GPIO_NUM_39
#define LCD_GPIO_D2 GPIO_NUM_38
#define LCD_GPIO_D3 GPIO_NUM_12
#define LCD_GPIO_D4 GPIO_NUM_11
#define LCD_GPIO_D5 GPIO_NUM_10
#define LCD_GPIO_D6 GPIO_NUM_9
#define LCD_GPIO_D7 GPIO_NUM_46

#define LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define LCD_DRAW_BUF_LINES 40

static esp_lcd_i80_bus_handle_t lcd_bus = NULL;
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static lv_display_t *lvgl_display = NULL;

/**
 * 配置 RD 引脚，保持高电平以禁用 LCD 读取。
 */
static void BSP_LCD_InitRD(void)
{
    const gpio_config_t rd_config =
    {
        .pin_bit_mask = 1ULL << LCD_GPIO_RD,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&rd_config));
    ESP_ERROR_CHECK(gpio_set_level(LCD_GPIO_RD, 1)); // RD 空闲为高电平
}

/**
 * 创建 8 位 I80 总线，配置引脚及 DMA 传输大小。
 */
static void BSP_LCD_InitBus(void)
{
    const esp_lcd_i80_bus_config_t bus_config =
    {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = LCD_GPIO_DC,
        .wr_gpio_num = LCD_GPIO_WR,
        .data_gpio_nums =
        {
            LCD_GPIO_D0, LCD_GPIO_D1, LCD_GPIO_D2, LCD_GPIO_D3,
            LCD_GPIO_D4, LCD_GPIO_D5, LCD_GPIO_D6, LCD_GPIO_D7
        },
        .bus_width = 8,
        .max_transfer_bytes = LCD_H_RES * LCD_DRAW_BUF_LINES * sizeof(uint16_t), // 40 行 RGB565 数据
        .dma_burst_size = 64
    };

    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &lcd_bus));
}

/**
 * 创建 LCD Panel IO，配置片选、时钟和命令/数据电平。
 */
static void BSP_LCD_InitPanelIO(void)
{
    const esp_lcd_panel_io_i80_config_t io_config =
    {
        .cs_gpio_num = LCD_GPIO_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 2,
        .dc_levels =
        {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1
        },
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags =
        {
            .swap_color_bytes = true // 在 IO 层交换 RGB565 字节
        }
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(lcd_bus, &io_config, &lcd_io));
}

/**
 * 初始化 ST7789 面板，设置 RGB565 色彩及 320×240 横屏方向。
 */
static void BSP_LCD_InitPanel(void)
{
    const esp_lcd_panel_dev_config_t panel_config =
    {
        .reset_gpio_num = GPIO_NUM_NC, // 使用软件复位
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_panel, true)); // 保留面板反色设置
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(lcd_panel, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_panel, true)); // 横屏显示
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true));
}

/**
 * 启动 LVGL 后台任务，将 LCD 注册为 LVGL 显示器。
 */
static lv_display_t *BSP_LCD_InitLVGL(void)
{
    lvgl_port_cfg_t lvgl_config = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_config.task_priority = 8;
    lvgl_config.task_max_sleep_ms = 5;
    lvgl_config.timer_period_ms = 2;
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_config));

    const lvgl_port_display_cfg_t display_config =
    {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = LCD_H_RES * LCD_DRAW_BUF_LINES, // 单位为像素
        .double_buffer = false,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation =
        {
            .swap_xy = true,
            .mirror_x = true,
            .mirror_y = false
        },
        .flags =
        {
            .buff_dma = true,
            .full_refresh = false, // 使用局部刷新
            .swap_bytes = false, // 字节交换已由 IO 层完成
            .sw_rotate = false
        }
    };

    return lvgl_port_add_disp(&display_config);
}

/**
 * 按顺序初始化 RD、I80、ST7789 和 LVGL，返回显示器句柄。
 * 重复调用时返回已创建的显示器。
 */
lv_display_t *bsp_lcd_init(void)
{
    if (lvgl_display != NULL)
    {
        return lvgl_display;
    }

    BSP_LCD_InitRD();
    BSP_LCD_InitBus();
    BSP_LCD_InitPanelIO();
    BSP_LCD_InitPanel();
    lvgl_display = BSP_LCD_InitLVGL();

    return lvgl_display;
}
