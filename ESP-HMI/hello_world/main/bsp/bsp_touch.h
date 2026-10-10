#ifndef BOARD_TOUCH_H
#define BOARD_TOUCH_H

#include "esp_err.h"

esp_err_t BSP_touch_probe(void);   // 复位并检测触摸芯片

#endif