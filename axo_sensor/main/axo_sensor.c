#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

#include "ds18b20.h"
#include "onewire_bus.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"

static const char *TAG = "AXO_SENSOR";

// --- HARDWARE CONFIGURATION ---
#define GPIO_DS18B20 (GPIO_NUM_4)

// --- MQTT CONFIGURATION ---
#define TOPIC_STATUS "sed/G09/axo_cool/status"
#define TOPIC_TEMP "sed/G09/axo_cool/temp"

typedef struct
{
    float temp;
    bool valid;
} sensor_data_t;

// --- GLOBAL VARIABLES ---
QueueHandle_t sensor_data_queue;
esp_mqtt_client_handle_t mqtt_client;
bool is_mqtt_connected = false;

// --- FUNCTION PROTOTYPES ---
void sensor_task(void *pvParameters);
void mqtt_publish_task(void *pvParameters);
void wifi_init_sta(void);
static esp_mqtt_client_handle_t mqtt_app_start(void);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

void app_main(void)
{
    ESP_LOGI(TAG, "Initiating AXO_SENSOR");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(2000));
    mqtt_client = mqtt_app_start();

    sensor_data_queue = xQueueCreate(5, sizeof(sensor_data_t));

    // Configuración del bus OneWire
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = GPIO_DS18B20};
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10};
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    // Tareas
    xTaskCreate(sensor_task, "sensor_task", 4096, (void *)bus, 5, NULL);
    xTaskCreate(mqtt_publish_task, "mqtt_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "AXO_SENSOR system initialized.");
}

void sensor_task(void *pvParameters)
{
    onewire_bus_handle_t bus = (onewire_bus_handle_t)pvParameters;
    ds18b20_device_handle_t sensor = NULL;
    ds18b20_config_t ds_config = {};

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_device;
    onewire_new_device_iter(bus, &iter);

    if (onewire_device_iter_get_next(iter, &next_device) == ESP_OK)
    {
        ESP_ERROR_CHECK(ds18b20_new_device_from_enumeration(&next_device, &ds_config, &sensor));
        ESP_LOGI(TAG, "Sensor found and initialized");
    }
    onewire_del_device_iter(iter);

    sensor_data_t data_read;

    while (1)
    {
        if (sensor && ds18b20_trigger_temperature_conversion(sensor) == ESP_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(800));

            if (ds18b20_get_temperature(sensor, &data_read.temp) == ESP_OK)
            {
                data_read.valid = true;
                ESP_LOGI(TAG, "Temperature read: %.2f°C", data_read.temp);
            }
            else
            {
                data_read.valid = false;
                ESP_LOGE(TAG, "Failed to read temperature");
            }
        }
        else
        {
            data_read.valid = false;
            ESP_LOGE(TAG, "Sensor not detected or failed to trigger conversion");
        }

        xQueueSend(sensor_data_queue, &data_read, pdMS_TO_TICKS(10));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void mqtt_publish_task(void *pvParameters)
{
    sensor_data_t received_data;
    char payload[16];

    while (1)
    {
        if (xQueueReceive(sensor_data_queue, &received_data, portMAX_DELAY))
        {
            if (received_data.valid && is_mqtt_connected)
            {
                snprintf(payload, sizeof(payload), "%.2f", received_data.temp);
                esp_mqtt_client_publish(mqtt_client, TOPIC_TEMP, payload, 0, 1, 0);
                ESP_LOGI(TAG, "MQTT publish: %s", payload);
            }
        }
    }
}

void wifi_init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();
}

static esp_mqtt_client_handle_t mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .session.last_will = {
            .topic = TOPIC_STATUS,
            .msg = "Offline",
            .qos = 1,
            .retain = 1,
        }};
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    return client;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        is_mqtt_connected = true;
        esp_mqtt_client_publish(mqtt_client, TOPIC_STATUS, "Online", 0, 1, 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        is_mqtt_connected = false;
        break;
    default:
        break;
    }
}