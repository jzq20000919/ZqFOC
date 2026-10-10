idf_component_register(
    SRCS "main.c" "bsp/bsp_lcd.c" "ui/motor_ui.c" "bsp/bsp_XL9555.c"
    "bsp/bsp_touch.c"
    "comm/comm_can.c"
    INCLUDE_DIRS "." "bsp" "ui" "comm"
    PRIV_REQUIRES esp_lcd esp_driver_gpio esp_driver_i2c esp_lvgl_port lvgl esp_driver_twai
)
