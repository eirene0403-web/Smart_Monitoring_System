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
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define MCP9700_OFFSET         500.0f          // 500 mV a 0°C
#define MCP9700_TC             10.0f           // 10 mV/°C

adc_oneshot_unit_handle_t adc_handle = NULL;        
adc_cali_handle_t cali_handle = NULL;
bool calibrated = false;
int voltage_mv;
float temperature = 0.0f; //global variable

// ESP RainMaker parameter handles
esp_rmaker_param_t *temperature_param;
esp_rmaker_param_t *fan_switch;

static const char *TAG = "app_main";

//ADC calibration
static void adc_calibration(void)
{
    adc_cali_scheme_ver_t scheme_mask;
    esp_err_t ret = adc_cali_check_scheme(&scheme_mask);

    if (ret == ESP_OK) 
    { 
        if (scheme_mask & ADC_CALI_SCHEME_VER_CURVE_FITTING)
        {   
            adc_cali_curve_fitting_config_t cali_cfg = {
                .unit_id = ADC_UNIT_1,
                .chan = ADC_CHANNEL_2,
                .atten = ADC_ATTEN_DB_12,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };

            if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle) == ESP_OK)
                calibrated = true;             
        }       
    } 
}

//Initialize ADC
static esp_err_t adc_oneshot_init(void)
{
    // Configuration of ADC
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_cfg, &adc_handle);
    if (ret != ESP_OK)
        return ret;     
    
    // Configuration of ADC Channel
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12
    };
    ret = adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_2, &chan_cfg);
    
    if (ret != ESP_OK)
        adc_oneshot_del_unit(adc_handle);    

    return ret;
}

//Read voltage from ADC and convert to temperature
float read_temperature(){
    int adc_raw;
  
    esp_err_t ret = adc_oneshot_read(adc_handle, ADC_CHANNEL_2, &adc_raw);
    if (ret == ESP_OK)
    {
       if(calibrated)
       {
            if (adc_cali_raw_to_voltage(cali_handle, adc_raw, &voltage_mv) == ESP_OK)
            {
                float temp = (voltage_mv - MCP9700_OFFSET) / MCP9700_TC;  //convert V to temperature
                return temp;
            }
            
       }
    }
    return 0;
}

void temperature_reading_task(void* pvParameters){
    while(1){
        float temp=read_temperature();

        temperature=temp;
        ESP_LOGI(TAG, "Temperature: %.2f°C", temperature);

        // Update the ESP RainMaker device parameter
        esp_rmaker_param_update_and_report(temperature_param, esp_rmaker_float(temperature));

        vTaskDelay(pdMS_TO_TICKS(10000)); // Update every 10 seconds
    }

}


/* Callback to handle commands received from the RainMaker cloud */
static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
            const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    // if (app_driver_set_gpio(esp_rmaker_param_get_name(param), val.val.b) == ESP_OK) {
    //     esp_rmaker_param_update(param, val);
    // }

    // Handle devices by their name (e.g., "Red", "Fan")
    const char *device_name = esp_rmaker_device_get_name(device);
    if (strcmp(device_name, "Red") == 0) {
        // Handle the "Red" device (e.g., set GPIO)
        if (app_driver_set_gpio(device_name, val.val.b) == ESP_OK) {
            esp_rmaker_param_update(param, val); // Update RainMaker parameter for Red
        }
    } else if (strcmp(device_name, "Fan") == 0) {
        // Handle the "Fan" device (e.g., set GPIO)
        if (app_driver_set_gpio(device_name, val.val.b) == ESP_OK) {
            esp_rmaker_param_update(param, val); // Update RainMaker parameter for Fan
        }
    } else {
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* Callback to handle commands received from the RainMaker cloud */
static esp_err_t voice_control_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
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

    //Initialize ADC and temperature sensor
    adc_oneshot_init();
    adc_calibration();

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

    // // -----------------------------------------------------------
    // // DEVICE 2: GREEN
    // // -----------------------------------------------------------
    // esp_rmaker_device_t *green_device = esp_rmaker_device_create("Green", ESP_RMAKER_DEVICE_SWITCH, NULL);
    // esp_rmaker_device_add_cb(green_device, write_cb, NULL);
    
    // esp_rmaker_param_t *green_power = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
    // esp_rmaker_device_add_param(green_device, green_power);
    // esp_rmaker_device_assign_primary_param(green_device, green_power);
    
    // esp_rmaker_node_add_device(node, green_device);

    // // -----------------------------------------------------------
    // // DEVICE 3: BLUE
    // // -----------------------------------------------------------
    // esp_rmaker_device_t *blue_device = esp_rmaker_device_create("Blue", ESP_RMAKER_DEVICE_SWITCH, NULL);
    // esp_rmaker_device_add_cb(blue_device, write_cb, NULL);
    
    // esp_rmaker_param_t *blue_power = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
    // esp_rmaker_device_add_param(blue_device, blue_power);
    // esp_rmaker_device_assign_primary_param(blue_device, blue_power);
    
    // esp_rmaker_node_add_device(node, blue_device);

    //Create MCP9700 device and add the relevant parameters to it
    esp_rmaker_device_t *temperature_device = esp_rmaker_device_create("Temperature", ESP_RMAKER_DEVICE_TEMP_SENSOR, NULL);
    esp_rmaker_device_add_cb(temperature_device, write_cb, NULL);

    temperature_param = esp_rmaker_param_create("Temperature", NULL, esp_rmaker_float(0), PROP_FLAG_READ | PROP_FLAG_PERSIST);
    esp_rmaker_device_add_param(temperature_device, temperature_param);
    esp_rmaker_node_add_device(node, temperature_device);

    //Fan switch
    esp_rmaker_device_t *fan_device = esp_rmaker_device_create("Fan", ESP_RMAKER_DEVICE_FAN, NULL);
    esp_rmaker_device_add_cb(fan_device, voice_control_cb, NULL); //voice control to on/off fan
    
    // Add the Standard POWER parameter (Required for Alexa)
    fan_switch = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
    esp_rmaker_device_add_param(fan_device, fan_switch);
    esp_rmaker_device_assign_primary_param(fan_device, fan_switch);
    
    esp_rmaker_node_add_device(node, fan_device);

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

    //Create Task
    xTaskCreate(temperature_reading_task, "temperature_reading", 4096, NULL, 2, NULL); //High prio, to run first
}
