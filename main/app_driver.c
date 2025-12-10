#include <sdkconfig.h>
#include <string.h>
#include <esp_log.h>

#include <app_reset.h>
#include "app_priv.h"

#define OUTPUT_FAN_SWITCH 8

//To be used by voice_cb and write_cb to turn on/off the fan
esp_err_t app_driver_set_gpio(const char *name, bool state)
{
    if (strcmp(name, "Fan") == 0) {
        gpio_set_level(OUTPUT_FAN_SWITCH, state);
    } 
    else {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void app_driver_init()
{
    //Configure the fan switch with internal pull up
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 1,
    };
    uint64_t pin_mask = ((uint64_t)1 << OUTPUT_FAN_SWITCH );
    io_conf.pin_bit_mask = pin_mask;

    gpio_config(&io_conf);
    gpio_set_level(OUTPUT_FAN_SWITCH, false); //Turn off the fan first
}
