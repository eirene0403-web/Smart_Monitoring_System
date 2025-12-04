#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>

#include <app_network.h>
#include <app_insights.h>

#include "app_priv.h"

#include "driver/gpio.h"
#include "ssd1306.h" //as a library of SSD1306 OLDE, taken from Practical Task 1

static const char *TAG = "app_main";

/* Callback to handle commands received from the RainMaker cloud */
static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
            const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }

    const char *device_name= esp_rmaker_device_get_name(device);
    const char *param_name= esp_rmaker_param_get_name(param);

    //Only handle the standart "Power" command
    if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME)==0){
        ESP_LOGI(TAG, "Received value = %s for %s - %s", val.val.b ? "true":"false", device_name, param_name);

        //Pass the device name to your driver (because the param is now always "Power")
        if(app_driver_set_gpio(device_name, val.val.b)==ESP_OK){
            esp_rmaker_param_update(param, val);
        }
    }
    return ESP_OK;
}

void app_main()
{
    /* Initialize Application specific hardware drivers and
     * set initial state.
     */
    app_driver_init();

    //Initialize i2c and OLED
    // i2c_master_init();
    // oled_init();
    // oled_clear(); //Clear the screen before displaying anything

    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    /* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_node_init()
     */
    app_network_init();
    
    /* Initialize the ESP RainMaker Agent.
     * Note that this should be called after app_network_init() but before app_network_start()
     * */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "GPIO-Device");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    // /* Create a device and add the relevant parameters to it */
    // esp_rmaker_device_t *gpio_device = esp_rmaker_device_create("GPIO-Device", NULL, NULL);
    // esp_rmaker_device_add_cb(gpio_device, write_cb, NULL);

    // esp_rmaker_param_t *red_param = esp_rmaker_param_create("Red", NULL, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    // esp_rmaker_param_add_ui_type(red_param, ESP_RMAKER_UI_TOGGLE);
    // esp_rmaker_device_add_param(gpio_device, red_param);

    // esp_rmaker_param_t *green_param = esp_rmaker_param_create("Green", NULL, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    // esp_rmaker_param_add_ui_type(green_param, ESP_RMAKER_UI_TOGGLE);
    // esp_rmaker_device_add_param(gpio_device, green_param);

    // esp_rmaker_param_t *blue_param = esp_rmaker_param_create("Blue", NULL, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    // esp_rmaker_param_add_ui_type(blue_param, ESP_RMAKER_UI_TOGGLE);
    // esp_rmaker_device_add_param(gpio_device, blue_param);

    // esp_rmaker_node_add_device(node, gpio_device);

    // -----------------------------------------------------------
    // DEVICE 1: RED
    // We create a standard SWITCH device named "Red"
    // -----------------------------------------------------------
    esp_rmaker_device_t *red_device = esp_rmaker_device_create("Red", ESP_RMAKER_DEVICE_SWITCH, NULL);
    esp_rmaker_device_add_cb(red_device, write_cb, NULL);
    
    // Add the Standard POWER parameter (Required for Alexa)
    esp_rmaker_param_t *red_power = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
    esp_rmaker_device_add_param(red_device, red_power);
    esp_rmaker_device_assign_primary_param(red_device, red_power);
    
    esp_rmaker_node_add_device(node, red_device);

    // -----------------------------------------------------------
    // DEVICE 2: GREEN
    // -----------------------------------------------------------
    esp_rmaker_device_t *green_device = esp_rmaker_device_create("Green", ESP_RMAKER_DEVICE_SWITCH, NULL);
    esp_rmaker_device_add_cb(green_device, write_cb, NULL);
    
    esp_rmaker_param_t *green_power = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
    esp_rmaker_device_add_param(green_device, green_power);
    esp_rmaker_device_assign_primary_param(green_device, green_power);
    
    esp_rmaker_node_add_device(node, green_device);

    // -----------------------------------------------------------
    // DEVICE 3: BLUE
    // -----------------------------------------------------------
    esp_rmaker_device_t *blue_device = esp_rmaker_device_create("Blue", ESP_RMAKER_DEVICE_SWITCH, NULL);
    esp_rmaker_device_add_cb(blue_device, write_cb, NULL);
    
    esp_rmaker_param_t *blue_power = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
    esp_rmaker_device_add_param(blue_device, blue_power);
    esp_rmaker_device_assign_primary_param(blue_device, blue_power);
    
    esp_rmaker_node_add_device(node, blue_device);

    // //Create MCP9700 device and add the relevant parameters to it
    // esp_rmaker_device_t *temperature_device = esp_rmaker_device_create("Temperature Device", NULL, NULL);

    /* Enable OTA */
    esp_rmaker_ota_enable_default();

    /* Enable Insights. Requires CONFIG_ESP_INSIGHTS_ENABLED=y */
    app_insights_enable();

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();

    /* Start the Wi-Fi.
     * If the node is provisioned, it will start connection attempts,
     * else, it will start Wi-Fi provisioning. The function will return
     * after a connection has been successfully established
     */
    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }
}
