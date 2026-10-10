#ifndef BSP_XL9555_H
#define BSP_XL9555_H

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"   // I2C总线句柄定义
i2c_master_bus_handle_t BSP_XL9555_GetI2CBus(void);   // 获取共享I2C总线
esp_err_t BSP_XL9555_Init(void);
esp_err_t BSP_XL9555_SetBacklight(bool enabled);
esp_err_t BSP_XL9555_SetTouchReset(bool asserted);   // 控制触摸芯片复位
#endif /* BSP_XL9555_H */