#pragma once
#include <stdint.h>
#include "driver/i2c.h"

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   GPIO_NUM_6
#define I2C_MASTER_SCL_IO   GPIO_NUM_7
#define I2C_MASTER_FREQ_HZ  400000
#define OLED_ADDR           0x3C

void i2c_master_init(void);
void oled_init(void);
void oled_clear(void);
void oled_set_cursor(uint8_t x, uint8_t y);
void oled_write_str(const char *str);
