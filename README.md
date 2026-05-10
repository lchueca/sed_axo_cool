# AXO-COOL: Gestión Térmica Distribuida para Ajolotes

## 1. Descripción del Proyecto
**AXO-COOL** es un sistema IoT diseñado para la monitorización y regulación térmica de acuarios de ajolotes (*Ambystoma mexicanum*). Dado que esta especie es extremadamente sensible al calor (requiere < 21°C y entra en estado crítico a los 25°C), el sistema utiliza una arquitectura distribuida para enfriar el agua mediante evaporación forzada de forma automática.

## 2. Especificaciones de Hardware (Fase 1)

El sistema consta de dos nodos ESP32 con configuraciones específicas de hardware:

### Nodo Sensor (ESP32 v1)
* **Sensor DS18B20:** Sonda digital sumergible de precisión.
* **Resistencia de Pull-up (4.7kΩ):** Conectada entre VCC y la línea de datos (GPIO 4).
    * **Justificación:** El protocolo OneWire es de tipo "open-drain", lo que significa que requiere una resistencia de pull-up para mantener la línea en estado lógico alto y permitir la comunicación bidireccional. Sin ella, el sensor no es detectable.

### Nodo Actuador (ESP32 v4)
* **Ventilador NZXT (4 pines):** Ventilador de PC con control PWM integrado.
    * **Simplificación de diseño:** Al utilizar un ventilador de 4 pines, el control de potencia (MOSFET) y la protección contra corrientes inversas (diodo) ya están integrados en la electrónica interna del ventilador. Esto permite el control directo desde la ESP32 (GPIO 18).
* **Resistencia de Pull-up (1kΩ) en PWM:**
    * **Justificación:** Garantiza un nivel lógico definido en la línea de control durante el arranque del microcontrolador. Esto evita ruidos eléctricos o arranques erráticos del ventilador antes de que el firmware tome el control del pin.

## 3. Arquitectura de Software y Conectividad (Fase 2)

### Robustez y Supervivencia
Para garantizar la seguridad del animal, el software implementa capas de protección ante fallos:
* **Gestión de Reintentos:** Tanto el sensor como el actuador monitorizan el estado del Wi-Fi. Si la conexión se pierde, ejecutan una lógica de reconexión infinita sin bloquear las tareas críticas.
* **Comunicación MQTT:** Uso del broker `broker.emqx.io` con mensajes de **Last Will (LWT)** para detectar la caída de los nodos en tiempo real.
* **Tratamiento Seguro de Datos:** El actuador procesa los mensajes de red mediante buffers temporales para asegurar una conversión de `string` a `float` estable, evitando reinicios por memoria sucia.

### MQTT Topics
| Topic | Descripción |
| :--- | :--- |
| `sed/G09/axo_cool/temp` | Telemetría de temperatura en tiempo real. |
| `sed/G09/axo_cool/status` | Estado de salud de los nodos (Online/Offline). |

## 4. Instrucciones de Compilación y Ejecución


## 5. Vídeo Demostrativo
Acceso a la demostración visual del sistema, donde se observa la respuesta del hardware ante cambios térmicos y la estabilidad de la red distribuida:

**[ENLACE AL VÍDEO (YouTube/Drive)]**

---
**Desarrollado por:** Laura Chueca Bronte [G09]  
**Asignatura:** Sistemas Empotrados Distribuidos (SED)