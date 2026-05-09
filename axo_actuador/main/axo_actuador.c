#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"

static const char *TAG = "AXO_ACTUADOR";

// --- GLOBAL VARIABLES -- -
static QueueHandle_t fan_speed_queue;

// --- HARDWARE CONFIGURATION ---
#define FAN_PWM_GPIO 18
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // Resolución 0 a 1023
#define FAN_FREQ_HZ 25000               // 25kHz estándar ventiladores PC

// --- MQTT CONFIGURATION ---
#define TOPIC_STATUS "sed/G09/status"    // Para el LWT
#define TOPIC_FAN "sed/G09/actuador/fan" // Para recibir órdenes

// --- FUNCTION PROTOTYPES ---
void init_fan_pwm(void);
void set_fan_speed(uint8_t percentage);
void actuator_task(void *pvParameters);
void wifi_init_sta(void);
static esp_mqtt_client_handle_t mqtt_app_start(void);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

void app_main(void)
{
    ESP_LOGI(TAG, "Initiating AXO_ACTUADOR");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    fan_speed_queue = xQueueCreate(10, sizeof(int));
    if (fan_speed_queue != NULL)
    {
        xTaskCreate(actuator_task, "ACTUATOR_TASK", 4096, NULL, 5, NULL);
    }

    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(5000));
    mqtt_app_start();

    ESP_LOGI(TAG, "Sistema AXO_ACTUADOR iniciado completamente");
}

void init_fan_pwm(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = FAN_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = FAN_PWM_GPIO,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void set_fan_speed(uint8_t percentage)
{
    if (percentage > 100)
        percentage = 100;
    uint32_t duty = (percentage * 1023) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

void actuator_task(void *pvParameters)
{
    int received_speed = 0;
    ESP_LOGI(TAG, "Actuator task started, waiting for fan speed commands...");

    init_fan_pwm();

    while (1)
    {
        if (xQueueReceive(fan_speed_queue, &received_speed, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Dato extraído de la cola: Ajustando a %d%%", received_speed);
            set_fan_speed((uint8_t)received_speed);
        }
    }
}

void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
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
        },
        .session.keepalive = 10,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    return client;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to Broker");
        esp_mqtt_client_publish(client, TOPIC_STATUS, "Online", 0, 1, 1);
        esp_mqtt_client_subscribe(client, TOPIC_FAN, 0);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Data received: %.*s", event->data_len, event->data);
        int speed = atoi(event->data);
        xQueueSend(fan_speed_queue, &speed, 0);
        break;
    default:
        break;
    }
}