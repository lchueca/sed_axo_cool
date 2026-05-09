#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "AXO_ACTUADOR";

// --- CONFIGURACIÓN DE HARDWARE ---
#define FAN_PWM_GPIO 18
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // Resolución 0 a 1023
#define FAN_FREQ_HZ 25000               // 25kHz estándar ventiladores PC

void init_fan_pwm(void);
void set_fan_speed(uint8_t percentage);
void actuator_task(void *pvParameters);

void app_main(void)
{
    ESP_LOGI(TAG, "Initiating AXO_ACTUADOR");
    xTaskCreate(actuator_task, "ACTUATOR_TASK", 4096, NULL, 5, NULL);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
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
    ESP_LOGI(TAG, "Initiating fan control task...");

    init_fan_pwm();

    while (1)
    {
        ESP_LOGI(TAG, "Work charge: 25%%");
        set_fan_speed(25);
        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI(TAG, "Work charge: 75%%");
        set_fan_speed(75);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}