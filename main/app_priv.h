#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

void app_driver_init(void);
esp_err_t app_driver_set_gpio(const char *name, bool state);
