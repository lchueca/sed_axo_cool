#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

#include "ds18b20.h"
#include "onewire_bus.h"

static const char *TAG = "AXO_SENSOR";

#define GPIO_DS18B20 (GPIO_NUM_4)
#define TEMP_UMBRAL_PELIGRO 25.0
#define TEMP_UMBRAL_IDEAL 21.0

typedef struct
{
    float temp;
    bool valid;
} sensor_data_t;

QueueHandle_t sensor_data_queue;

void sensor_task(void *pvParameters);
void logic_task(void *pvParameters);

void app_main(void)
{

    // Cola para comunicación entre tareas
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
    xTaskCreate(logic_task, "logic_task", 4096, NULL, 5, NULL);
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
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void logic_task(void *pvParameters)
{
    sensor_data_t received_data;
    float last_printed = 0;

    while (1)
    {
        if (xQueueReceive(sensor_data_queue, &received_data, portMAX_DELAY))
        {

            if (!received_data.valid)
            {
                ESP_LOGE(TAG, "Received invalid temperature data");
                continue;
            }

            if (fabsf(received_data.temp - last_printed) >= 0.5 || received_data.temp >= TEMP_UMBRAL_PELIGRO)
            {
                ESP_LOGI(TAG, "Temperature: %.2f °C", received_data.temp);
                last_printed = received_data.temp;

                if (received_data.temp >= TEMP_UMBRAL_PELIGRO)
                {
                    ESP_LOGW(TAG, "Danger! Temperature exceeds %.2f °C", TEMP_UMBRAL_PELIGRO);
                }
            }
        }
    }
}