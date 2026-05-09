idf_component_register(SRCS "axo_sensor.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES nvs_flash esp_wifi esp_event esp_netif mqtt driver onewire_bus ds18b20)
