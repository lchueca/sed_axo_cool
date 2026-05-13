# AXO-COOL: Gestión Térmica Distribuida [G09]

## 1. Resumen
Sistema IoT distribuido para la monitorización y control climático de acuarios de ajolotes. El sistema garantiza que la temperatura no exceda los **25°C** mediante una arquitectura de sensores y actuadores comunicados por MQTT.

## 2. Especificaciones de Hardware
| Componente | Función | Pin (GPIO) | Protocolo / Señal |
| :--- | :--- | :--- | :--- |
| **ESP32 v1** | Nodo Sensor | - | Publisher (WiFi/MQTT) |
| **DS18B20** | Sonda Térmica | **GPIO 4** | OneWire |
| **ESP32 v4** | Nodo Actuador | - | Subscriber (WiFi/MQTT) |
| **Ventilador NZXT** | Enfriamiento | **GPIO 18** | PWM (25kHz) |
| **LED Rojo** | Alerta Crítica | **GPIO 25** | Digital Out |
| **LED Amarillo** | Enfriando | **GPIO 26** | Digital Out |
| **LED Verde** | Estado Ideal | **GPIO 27** | Digital Out |

### Detalles de Electrónica y Protección
* **Resistencias de Pull-up:** 
    * **4.7kΩ** en la línea de datos del DS18B20 (GPIO 4) para permitir la comunicación OneWire.
    * **1kΩ** en la línea PWM (GPIO 18) para asegurar un nivel lógico estable durante el arranque del SoC.
* **Resistencias de Protección LED:** 
    * Cada LED (Rojo, Amarillo y Verde) cuenta con una resistencia limitadora de **220Ω** en serie para proteger los puertos GPIO de sobrecorrientes.

## 3. Instrucciones de Compilación y Ejecución
El proyecto utiliza configuraciones base en el archivo `sdkconfig.defaults`. 


### 1. Preparación de Componentes Externos
> IMPORTANTE: Antes de compilar, es obligatorio descargar el cliente de Mender, ya que no forma parte del núcleo de ESP-IDF. Desde la raíz del proyecto, ejecuta:

``` sh
# 1. Crear el directorio de componentes externos en cada nodo
mkdir -p external/mender-mcu-client

# 2. Clonar el repositorio del cliente Mender (Versión específica)
git clone --branch 0.12.3 --recursive https://github.com/joelguittet/mender-mcu-client.git external/mender-mcu-client/
```

### 2. Configuración de Credenciales
Para configurar los datos de red sin modificar el código fuente, crea un archivo llamado `sdkconfig.local` en la raíz de la carpeta de cada nodo con la siguiente estructura:

```text
CONFIG_ESP_WIFI_SSID="Nombre_Tu_Wifi"
CONFIG_ESP_WIFI_PASSWORD="Password_Tu_Wifi"

# Broker para entorno de desarrollo o laboratorio
CONFIG_BROKER_URL="mqtt://broker.emqx.io"

# Configuración de Mender
CONFIG_MENDER_SERVER_HOST="https://IP_DE_TU_PORTATIL"
```

También puedes hacerlo desde `idf.py menuconfig` en `Example Configuration` 

### 3. Compilación y Flasheo: 
Dado que los nodos residen en directorios independientes, sitúate en la carpeta del firmware correspondiente y ejecuta:
``` sh
# Sustituye X por el número de puerto serie (ej: USB0)
idf.py -p /dev/ttyUSBX flash monitor
```

### 4. Generación de Artefactos OTA
Para desplegar una nueva versión una vez que el dispositivo esté aceptado en el panel de Mender:

1. Incrementa el #define VERSION en el código.
2. Compila con idf.py build.
3. Genera el artefacto .mender para subir a la web:
``` sh
mender-artifact write rootfs-image \
  --device-type esp32 \
  --artifact-name 1.0.1 \
  --file build/p4.1.bin \
  --output-path update_v1.0.1.mender
```

## 4. Resiliencia y Supervivencia (Fail-safe)
El sistema se pone en modo Alerta ante cualquier fallo de comunicación:
* **Lógica de Seguridad (No Data):** Si el actuador no recibe lecturas del sensor durante 5 segundos (`missing_data_count >= 10`), el sistema entra en modo de emergencia: ventilador al **100% de potencia** y parpadeo de **LED Rojo**.
* **Last Will and Testament (LWT):** El sistema registra un mensaje de "testamento" en el Broker. Si el nodo pierde la conexión, el Broker publica automáticamente el estado **"Offline"**, permitiendo que el Dashboard detecte la caída del hardware.


## 5. Dashboard (Node-RED)
Interfaz visual para la supervisión técnica y el diagnóstico de salud del sistema en tiempo real:
* **Telemetría:** Gauges de temperatura y potencia.
* **Estado de Salud:** Indicadores dinámicos basados en la carga del JSON y el LWT: 
    * `OK`: Funcionamiento normal.
    * `SENSOR_LOST`: El nodo actuador está online pero no recibe datos del sensor.
    * `OFFLINE`: El nodo actuador ha perdido la conexión con el Broker.
**Histórico:** Gráfica de evolución térmica (Chart) para analizar la inercia térmica del acuario.
* **Alertas:** Lógica `Switch` independiente para temperaturas críticas > 25°C, futura implementación de alertas por Telegram.

## 6. Vídeo Demostrativo
**[Link vídeo demostrativo](https://drive.google.com/file/d/1OkcP9L0U24dNBReip2bYV8LNu5w_a5Fa/view?usp=sharing)**