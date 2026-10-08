#ifndef BSP_XL9555_H
#define BSP_XL9555_H

#include <stdbool.h>
#include "esp_err.h"

esp_err_t BSP_XL9555_Init(void);
esp_err_t BSP_XL9555_SetBacklight(bool enabled);

#endif /* BSP_XL9555_H */