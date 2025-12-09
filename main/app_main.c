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
#include "freertos/event_groups.h"
// #include "freertos/semaphore.h"
#include "freertos/queue.h"

#define MCP9700_OFFSET         500.0f          // 500 mV a 0°C
#define MCP9700_TC             10.0f           // 10 mV/°C
#define ADC_SAMPLES 1000  // Number of samples to average
#define TEMPERATURE_THRESHOLD 35 //deg C
#define BUTTON_ACTIVE_LEVEL 0 //when button pressed = 0
#define OUTPUT_FAN_SWITCH 8
#define OUTPUT_GPIO_RED   4
#define DEBOUNCE_DELAY pdMS_TO_TICKS(100)

#define HOT_TEMP_EVENT (1<<0)
#define COMFORT_TEMP_EVENT (1<<1)

adc_oneshot_unit_handle_t adc_handle = NULL;        
adc_cali_handle_t cali_handle = NULL;
bool calibrated = false;
int voltage_mv;
float temperature = 0.0f; //global variable

// Global handles for semaphores and queues
SemaphoreHandle_t fanSemaphore;  // To control fan state
QueueHandle_t temperatureQueue;  // To queue temperature values
QueueHandle_t notification_queue;  // To queue temperature values
EventGroupHandle_t temp_event_group;
TaskHandle_t fanControlTaskHandle; // Task handle for fan control

// ESP RainMaker parameter handles
esp_rmaker_param_t *temperature_param;
esp_rmaker_param_t *temp_state_param;
esp_rmaker_param_t *temp_alert_param;
esp_rmaker_param_t *fan_switch;

static const char *TAG = "app_main";

bool manual_mode = false;

// Queue message structure
typedef struct {
    char message[64];
} notification_msg_t;

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
    long total = 0;

    esp_err_t ret;

    // Read multiple samples and average them
    for (int i = 0; i < ADC_SAMPLES; i++) {
        ret = adc_oneshot_read(adc_handle, ADC_CHANNEL_2, &adc_raw);
        if (ret == ESP_OK) {
            total += adc_raw;
        }
    }

    int avg_adc = total / ADC_SAMPLES;  // Calculate the average ADC value
    int voltage_mv;
  
    // esp_err_t ret = adc_oneshot_read(adc_handle, ADC_CHANNEL_2, &adc_raw);
    if (ret == ESP_OK)
    {
       if(calibrated)
       {
            if (adc_cali_raw_to_voltage(cali_handle, avg_adc, &voltage_mv) == ESP_OK)
            {
                float temp = (voltage_mv - MCP9700_OFFSET) / MCP9700_TC;  //convert V to temperature
                return temp;
            }
            
       }
    }
    return 0;
}

// Send unified push + UI notification via RainMaker
void send_unified_notification(const char *message) {
    esp_rmaker_param_update_and_report(temp_alert_param, esp_rmaker_str(message));
    
    esp_err_t err = esp_rmaker_raise_alert(message);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Notification sent and alert updated: %s", message);
    } else {
        ESP_LOGE(TAG, "Failed to send notification! Error: %d", err);
    }
}

// Notification handler task (listens for messages on queue)
void notification_task(void *arg) {
    notification_msg_t received_msg;

    while (1) {
        if (xQueueReceive(notification_queue, &received_msg, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Processing notification: %s", received_msg.message);
            send_unified_notification(received_msg.message);
        }
    }
}

void temperature_reading_task(void* pvParameters){
    while(1){
        float temp=read_temperature();

        temperature=temp;
        ESP_LOGI(TAG, "Temperature: %.2f°C", temperature);

        // Update the ESP RainMaker device parameter
        esp_rmaker_param_update_and_report(temperature_param, esp_rmaker_float(temperature));
    
        // Determine temperature state and act accordingly
        notification_msg_t msg;
        const char* status;
        
        if (temperature > TEMPERATURE_THRESHOLD) {
            status = "Hot";
            snprintf(msg.message, sizeof(msg.message), "ALERT: It's Hot! Turn on the fan");
            xEventGroupSetBits(temp_event_group, HOT_TEMP_EVENT);

            if (fanControlTaskHandle != NULL) {
                xTaskNotifyGive(fanControlTaskHandle); // Notify watering task
            }
        } 
        else {
            status = "OK";
            snprintf(msg.message, sizeof(msg.message), "It's comfortable. Turn off the fan");
            xEventGroupSetBits(temp_event_group, COMFORT_TEMP_EVENT);
            if (fanControlTaskHandle != NULL) {
                xTaskNotifyGive(fanControlTaskHandle); // Notify watering task
            }
        }

        esp_rmaker_param_update_and_report(temp_state_param, esp_rmaker_str(status));

        if (strcmp(status, "OK") == 0) {
            esp_rmaker_param_update_and_report(temp_alert_param, esp_rmaker_str(msg.message));
            ESP_LOGI(TAG, "Updated alert (UI only): %s", msg.message);
        } else {
            xQueueSend(notification_queue, &msg, portMAX_DELAY); // Send notification
        }

        vTaskDelay(pdMS_TO_TICKS(10000)); // Update every 2 seconds
    }

}

void fan_control_task(void* pvParameters) {
    while(1) {
        // Wait for the temperature value from the queue
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for signal
        if(xSemaphoreTake(fanSemaphore, portMAX_DELAY)){
            if(manual_mode == false){
                //Wait for HOT or COMFORT TEMP EVENT bit to be set in the event group
                EventBits_t bits=xEventGroupWaitBits(temp_event_group, HOT_TEMP_EVENT | COMFORT_TEMP_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);
                if (bits & HOT_TEMP_EVENT) {
                    // Turn on the fan if temperature > threshold
                    ESP_LOGI(TAG, "Temperature exceeds %.2f°C, turning on fan", TEMPERATURE_THRESHOLD);
                    gpio_set_level(OUTPUT_FAN_SWITCH, 1); // Turn on fan
                }
                else if(bits & COMFORT_TEMP_EVENT){
                    // Turn off the fan if temperature <= threshold
                    ESP_LOGI(TAG, "Temperature is normal, turning off fan");
                    gpio_set_level(OUTPUT_FAN_SWITCH, 0); // Turn off fan
                }
                else{
                    //Do nothing, no event bits set
                    ESP_LOGI(TAG, "Do nothing, no event bits set");
                }
            }
            xSemaphoreGive(fanSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Debounce delay
    }   
}

/* Callback to handle commands received from the RainMaker cloud */
static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
            const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }

    // Handle devices by their name (e.g., "Red", "Fan")
    const char *device_name = esp_rmaker_device_get_name(device);
    if (strcmp(device_name, "Fan") == 0) {
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

    const char *device_name=esp_rmaker_device_get_name(device);
    const char *param_name=esp_rmaker_param_get_name(param);

    //Only handle the standart "Power" command
    if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME)==0){
        manual_mode=true;
        if (xSemaphoreTake(fanSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_LOGI(TAG, "Received value = %s for %s - %s", val.val.b ? "true":"false", device_name, param_name);

            //Pass the device name to your driver (because the param is now always "Power")
            if(app_driver_set_gpio(device_name, val.val.b)==ESP_OK){
                esp_rmaker_param_update(param, val);
            
                // Prepare and send notification message
                notification_msg_t msg;
                snprintf(msg.message, sizeof(msg.message), "Fan is turned on manually");
                xQueueSend(notification_queue, &msg, portMAX_DELAY);
            }
            xSemaphoreGive(fanSemaphore);
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
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "SmartDevice");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    //Create MCP9700 device and add the relevant parameters to it
    esp_rmaker_device_t *temperature_device = esp_rmaker_device_create("Temperature", ESP_RMAKER_DEVICE_TEMP_SENSOR, NULL);
    esp_rmaker_device_add_cb(temperature_device, write_cb, NULL);

    temperature_param = esp_rmaker_param_create("Temperature", NULL, esp_rmaker_float(0), PROP_FLAG_READ | PROP_FLAG_PERSIST);
    temp_state_param = esp_rmaker_param_create("Temperature Status", NULL, esp_rmaker_str("OK"), PROP_FLAG_READ | PROP_FLAG_PERSIST);
    temp_alert_param = esp_rmaker_param_create("Alert", NULL, esp_rmaker_str("No Alerts"), PROP_FLAG_READ | PROP_FLAG_PERSIST);

    esp_rmaker_device_add_param(temperature_device, temperature_param);
    esp_rmaker_device_add_param(temperature_device, temp_state_param);
    esp_rmaker_device_add_param(temperature_device, temp_alert_param);

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

    // Create semaphores and queues
    fanSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(fanSemaphore);
    notification_queue = xQueueCreate(5, sizeof(notification_msg_t));
    temp_event_group = xEventGroupCreate();

    //Create Task
    xTaskCreate(temperature_reading_task, "temperature_reading", 4096, NULL, 2, NULL); //High prio, to run first
    xTaskCreate(fan_control_task, "fan control task", 4096, NULL, 2, &fanControlTaskHandle); //High prio, to run first
    xTaskCreate(notification_task, "notification_task", 4096, NULL, 2, NULL);
}
