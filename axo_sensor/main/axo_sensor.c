/**
 * @file axo_sensor_main.c
 * @author Laura Chueca Bronte / Grupo 09
 * @brief Nodo de Telemetría Térmica AXO-COOL.
 * Implementa lectura asíncrona de bus OneWire, gestión de eventos MQTT
 * y sistema de actualización remota (OTA) con capacidad de Rollback.
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

// --- FreeRTOS Kernel ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// --- ESP-IDF System & Drivers ---
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"

// --- Periféricos (OneWire / DS18B20) ---
#include "ds18b20.h"
#include "onewire_bus.h"

// --- Comunicaciones y OTA (Mender) ---
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "esp_ota_ops.h"
#include "mender-client.h"
#include "mender-flash.h"

static const char *TAG = "AXO_SENSOR";

// =====================================================================
// --- CONFIGURACIÓN GLOBAL Y ESTRUCTURAS ---
// =====================================================================

typedef struct
{
    float temp;
    bool valid;
} sensor_data_t;

#define VERSION "1.0.1"
#define GPIO_DS18B20 (GPIO_NUM_4)

// --- GLOBAL HANDLERS ---
QueueHandle_t sensor_data_queue;
esp_mqtt_client_handle_t mqtt_client;
bool is_mqtt_connected = false;

// --- TOPICS MQTT ---
#define TOPIC_STATUS "sed/G09/axo_cool/status"
#define TOPIC_TEMP "sed/G09/axo_cool/temp"

// --- PROTOTIPOS DE FUNCIONES ---
void sensor_task(void *pvParameters);
esp_err_t reinitialize_onewire_bus(onewire_bus_handle_t *bus_handle);
esp_err_t recover_ds18b20_sensor(onewire_bus_handle_t bus, ds18b20_device_handle_t *sensor);
void mqtt_publish_task(void *pvParameters);
void wifi_init_sta(void);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static esp_mqtt_client_handle_t mqtt_app_start(void);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
bool perform_health_check();
void check_and_commit_ota();

// =====================================================================
// --- INFRAESTRUCTURA MENDER (OTA) ---
// =====================================================================

mender_client_config_t mender_config;
mender_client_callbacks_t mender_callbacks;
mender_keystore_t mender_identity[2];
char mender_mac_address[18];

// --- CALLBACKS MENDER ---
static mender_err_t mender_network_connect_cb(void) { return MENDER_OK; }
static mender_err_t mender_network_release_cb(void) { return MENDER_OK; }
static mender_err_t mender_auth_failure_cb(void) { return MENDER_OK; }
static mender_err_t mender_deployment_status_cb(mender_deployment_status_t status, char *desc)
{
    ESP_LOGI("MENDER", "Estado del despliegue: %s", desc ? desc : "Desconocido");
    return MENDER_OK;
}
static mender_err_t mender_auth_success_cb(void)
{
    return MENDER_OK;
}
static mender_err_t mender_restart_cb(void)
{
    ESP_LOGI("MENDER", "Reiniciando por petición de OTA...");
    esp_restart();
    return MENDER_OK;
}

// =====================================================================
// --- APLICACIÓN PRINCIPAL (app_main) ---
// =====================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "--- Iniciando AXO_SENSOR ---");
    const esp_partition_t *running = esp_ota_get_running_partition();
    printf("Versión de Firmware: %s\n", VERSION);
    ESP_LOGI(TAG, "Ejecutando desde partición: %s", running->label);

    // Inicialización NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicialización Wi-Fi y self-test OTA
    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(10000));
    check_and_commit_ota();

    // Configuración Mender
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(mender_mac_address, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    mender_identity[0].name = "mac";
    mender_identity[0].value = mender_mac_address;
    mender_identity[1].name = NULL;

    mender_config.identity = mender_identity;
    mender_config.artifact_name = VERSION;
    mender_config.device_type = "esp32";
    mender_config.host = CONFIG_MENDER_SERVER_HOST; // URL del portátil
    mender_config.authentication_poll_interval = 60;
    mender_config.update_poll_interval = 60;

    mender_callbacks.network_connect = mender_network_connect_cb;
    mender_callbacks.authentication_success = mender_auth_success_cb;
    mender_callbacks.deployment_status = mender_deployment_status_cb;
    mender_callbacks.restart = mender_restart_cb;

    if (mender_client_init(&mender_config, &mender_callbacks) == MENDER_OK)
    {
        mender_client_activate();
    }

    // Lanzamiento de tareas
    mqtt_client = mqtt_app_start();
    sensor_data_queue = xQueueCreate(5, sizeof(sensor_data_t));

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(mqtt_publish_task, "mqtt_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "AXO_SENSOR inicializado correctamente.");
}

// =====================================================================
// --- LÓGICA DE SENSOR Y CONTROL ---
// =====================================================================

/**
 * @brief Gestiona el ciclo de vida del bus OneWire y la captura térmica.
 * Implementa recuperación automática ante desconexión física del sensor.
 */
void sensor_task(void *pvParameters)
{
    onewire_bus_handle_t bus = NULL;
    ds18b20_device_handle_t sensor = NULL;
    sensor_data_t data_read;

    while (1)
    {
        // Reinicialización del bus si se detecta desconexión del sensor
        if (bus == NULL)
        {
            if (reinitialize_onewire_bus(&bus) != ESP_OK)
            {
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
        }

        // Si no hay sensor, buscamos
        if (sensor == NULL)
        {
            if (recover_ds18b20_sensor(bus, &sensor) == ESP_OK)
            {
                ESP_LOGI(TAG, "Sensor DS18B20 recuperado.");
            }
        }

        // Intento de lectura
        if (sensor != NULL)
        {
            if (ds18b20_trigger_temperature_conversion(sensor) == ESP_OK)
            {
                vTaskDelay(pdMS_TO_TICKS(800));
                if (ds18b20_get_temperature(sensor, &data_read.temp) == ESP_OK)
                {
                    data_read.valid = true;
                    ESP_LOGI(TAG, "Leyendo: %.2f C", data_read.temp);
                    xQueueSend(sensor_data_queue, &data_read, 0);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
            }
            // SI FALLA ALGO
            ESP_LOGE(TAG, "Fallo en la lectura. Liberando recursos...");
            ds18b20_del_device(sensor);
            sensor = NULL;
            onewire_bus_del(bus);
            bus = NULL;

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            if (bus)
            {
                onewire_bus_del(bus);
                bus = NULL;
            }
            data_read.valid = false;
            xQueueSend(sensor_data_queue, &data_read, 0);
            ESP_LOGW(TAG, "Buscando el sensor...");
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}

/**
 * @brief  Realiza el reset físico y lógico del bus OneWire.
 * @note   Utiliza gpio_reset_pin para asegurar que no hay estados residuales
 * que bloqueen la comunicación con la sonda.
 * @param[out] bus_handle Dirección del manejador a inicializar.
 * @return esp_err_t ESP_OK si el bus RMT se instanció correctamente.
 */
esp_err_t reinitialize_onewire_bus(onewire_bus_handle_t *bus_handle)
{
    onewire_bus_config_t bus_config = {.bus_gpio_num = GPIO_DS18B20};
    onewire_bus_rmt_config_t rmt_config = {.max_rx_bytes = 10};

    ESP_LOGW(TAG, "Reinicializando bus OneWire...");

    gpio_reset_pin(GPIO_DS18B20);
    vTaskDelay(pdMS_TO_TICKS(100));

    if (onewire_new_bus_rmt(&bus_config, &rmt_config, bus_handle) != ESP_OK)
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief  Escanea el bus y vincula el driver específico del sensor térmico.
 * @details Implementa la lógica de enumeración necesaria para el protocolo OneWire.
 * @param[in]  bus    Manejador del bus activo.
 * @param[out] sensor Puntero donde se almacenará la instancia del driver DS18B20.
 * @return esp_err_t ESP_OK si se identifica y vincula el hardware.
 */
esp_err_t recover_ds18b20_sensor(onewire_bus_handle_t bus, ds18b20_device_handle_t *sensor)
{
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_device;
    ds18b20_config_t ds_config = {}; // Configuración por defecto

    // Creamos el iterador para buscar dispositivos en el bus
    onewire_new_device_iter(bus, &iter);

    if (onewire_device_iter_get_next(iter, &next_device) == ESP_OK)
    {
        // Intentamos instanciar el driver DS18B20 sobre el dispositivo encontrado
        if (ds18b20_new_device_from_enumeration(&next_device, &ds_config, sensor) == ESP_OK)
        {
            onewire_del_device_iter(iter);
            return ESP_OK;
        }
    }

    onewire_del_device_iter(iter);
    return ESP_FAIL;
}

/**
 * @brief Tarea que publica los datos del sensor en MQTT.
 */
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
                ESP_LOGI(TAG, "MQTT Publicación [TEMP]: %s", payload);
            }
        }
    }
}

// =====================================================================
// --- INFRAESTRUCTURA DE RED Y RESILIENCIA ---
// =====================================================================

void wifi_init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "Conexión perdida. Intentando reconectar a Wi-Fi...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP obtenida: " IPSTR, IP2STR(&event->ip_info.ip));
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
        ESP_LOGI(TAG, "Conectado al Broker");
        is_mqtt_connected = true;
        esp_mqtt_client_publish(mqtt_client, TOPIC_STATUS, "Online", 0, 1, 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        is_mqtt_connected = false;
        ESP_LOGW(TAG, "Desconectado del Broker MQTT.");
        break;
    default:
        break;
    }
}

bool perform_health_check()
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
    {
        ESP_LOGI("HEALTH", "Conectado al AP con RSSI: %d", ap_info.rssi);
        return true;
    }
    ESP_LOGE("HEALTH", "Fallo de conexión Wi-Fi.");
    return false;
}

void check_and_commit_ota()
{
    esp_ota_img_states_t ota_state;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            ESP_LOGI("OTA", "Imagen en periodo de prueba. Iniciando auto-diagnóstico...");
            if (perform_health_check())
            {
                ESP_LOGI("OTA", "Salud verificada. Marcando firmware como VÁLIDO.");
                esp_ota_mark_app_valid_cancel_rollback(); //
            }
            else
            {
                ESP_LOGE("OTA", "Error en diagnóstico. Forzando Rollback...");
                esp_ota_mark_app_invalid_rollback_and_reboot(); //
            }
        }
    }
}