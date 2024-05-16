#pragma once
#include "esp_err.h"
#define I2C_MASTER_NUM              0

extern esp_err_t gxhtc3_read_id(void);
extern esp_err_t gxhtc3_get_tah(void);
