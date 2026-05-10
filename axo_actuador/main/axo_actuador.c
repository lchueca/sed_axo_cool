#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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
static QueueHandle_t temp_data_queue;
static esp_mqtt_client_handle_t mqtt_client;
static bool is_mqtt_connected = false;
static int red_led_blink_state = 0;

// --- HARDWARE CONFIGURATION ---
#define FAN_PWM_GPIO 18
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // Resolución 0 a 1023
#define FAN_FREQ_HZ 25000               // 25kHz estándar ventiladores PC
#define LED_RED 25
#define LED_YELLOW 26
#define LED_GREEN 27

// --- MQTT CONFIGURATION ---
#define TOPIC_STATUS "sed/G09/axo_cool/status"
#define TOPIC_SENSOR_DATA "sed/G09/axo_cool/temp"
#define TOPIC_FEEDBACK "sed/G09/axo_cool/feedback"

// --- TEMPERATURE THRESHOLDS ---
#define TEMP_UMBRAL_PELIGRO 25.0
#define MARGEN 0.5
#define TEMP_UMBRAL_IDEAL 23.0

// --- FUNCTION PROTOTYPES ---
void init_fan_pwm(void);
void set_fan_speed(uint8_t percentage);
void actuator_task(void *pvParameters);
void wifi_init_sta(void);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static esp_mqtt_client_handle_t mqtt_app_start(void);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void init_leds(void);
void set_led_state(int r, int y, int g);

void app_main(void)
{
    ESP_LOGI(TAG, "Initiating AXO_ACTUADOR...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    temp_data_queue = xQueueCreate(10, sizeof(float));
    if (temp_data_queue != NULL)
    {
        xTaskCreate(actuator_task, "ACTUATOR_TASK", 4096, NULL, 5, NULL);
    }

    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(5000));
    mqtt_client = mqtt_app_start();

    ESP_LOGI(TAG, "AXO_ACTUADOR system initialized.");
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
    float current_temp = 0.0;
    int missing_data_count = 0;
    init_fan_pwm();
    init_leds();

    while (1)
    {
        if (xQueueReceive(temp_data_queue, &current_temp, pdMS_TO_TICKS(500)))
        {
            int power = 0;
            missing_data_count = 0;

            // 1. Estado REPOSO
            if (current_temp < (TEMP_UMBRAL_IDEAL - MARGEN))
            {
                power = 0;
                set_led_state(0, 0, 1);
                ESP_LOGI(TAG, "Status: REST (%.2f C). Fan OFF.", current_temp);
            }
            // 2. Estado ENFRIANDO
            else if (current_temp >= (TEMP_UMBRAL_IDEAL - MARGEN) && current_temp <= TEMP_UMBRAL_PELIGRO)
            {
                power = 20 + (int)((current_temp - (TEMP_UMBRAL_IDEAL - MARGEN)) * 24);
                set_led_state(0, 1, 0);
                ESP_LOGI(TAG, "Status: COOLING (%.2f C). Power: %d%%.", current_temp, power);
            }
            // 3. Estado ALERTA
            else if (current_temp > TEMP_UMBRAL_PELIGRO)
            {
                power = 100;
                set_led_state(1, 0, 0);
                ESP_LOGW(TAG, "¡Status: DANGER! Axolot in danger (%.2f C). Power: 100%%.", current_temp);
            }
            set_fan_speed(power);

            if (is_mqtt_connected)
            {
                char feedback[80];
                snprintf(feedback, sizeof(feedback), "{\"temp\": %.2f, \"pw\": %d, \"status\": \"OK\"}", current_temp, power);
                esp_mqtt_client_publish(mqtt_client, TOPIC_FEEDBACK, feedback, 0, 1, 0);
            }
        }
        else
        {
            missing_data_count++;
            if (missing_data_count >= 10)
            {

                set_fan_speed(100);
                red_led_blink_state = !red_led_blink_state;
                set_led_state(red_led_blink_state, 0, 0);

                if (missing_data_count % 20 == 0)
                {
                    ESP_LOGE(TAG, "¡TIMEOUT! No data from sensor. Fan at 100%% for safety.");
                    if (is_mqtt_connected)
                    {
                        esp_mqtt_client_publish(mqtt_client, TOPIC_FEEDBACK, "{\"status\": \"SENSOR_LOST\"}", 0, 1, 0);
                    }
                }
            }
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

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "Lost connection. Retrying to connect to Wi-Fi...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP Obtained: " IPSTR, IP2STR(&event->ip_info.ip));
    }
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
        is_mqtt_connected = true;
        esp_mqtt_client_publish(client, TOPIC_STATUS, "Online", 0, 1, 1);
        esp_mqtt_client_subscribe(client, TOPIC_SENSOR_DATA, 0);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Data received: %.*s", event->data_len, event->data);
        char bf[16] = {0};
        int len = (event->data_len < sizeof(bf) - 1) ? event->data_len : sizeof(bf) - 1;
        memcpy(bf, event->data, len);

        float speed = atof(bf);
        xQueueSend(temp_data_queue, &speed, 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected. Attempting to reconnect...");
        is_mqtt_connected = false;
        break;
    default:
        break;
    }
}

void init_leds(void)
{
    gpio_reset_pin(LED_RED);
    gpio_reset_pin(LED_YELLOW);
    gpio_reset_pin(LED_GREEN);
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_YELLOW, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
}

void set_led_state(int r, int y, int g)
{
    gpio_set_level(LED_RED, r);
    gpio_set_level(LED_YELLOW, y);
    gpio_set_level(LED_GREEN, g);
}