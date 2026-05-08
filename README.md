# STM32F070RE-e-interfaz-de-usuario-en-Visual-Studio.-
El entorno consiste en una placa STM32 y un módulo para bus CAN  conectado por SPI.  Además, se hará uso de los  siguientes periféricos: un servo motor, un motor paso a paso (con su driver de potencia), un display I2C, un  sensor de temperatura LM35, un sensor de distancia ToF y un módulo CAN basado en el chip MCP2515 de  SiLabs. 
# Sistema de Control Industrial: STM32 + FreeRTOS + HMI en C# 🚀

![C](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white)
![C#](https://img.shields.io/badge/C%23-239120?style=for-the-badge&logo=c-sharp&logoColor=white)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-22314E?style=for-the-badge&logo=freertos&logoColor=white)
![STM32](https://img.shields.io/badge/STM32-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white)

Este repositorio contiene el código fuente de un proyecto universitario enfocado en la **Informática y Comunicaciones Industriales**. Consiste en un sistema embebido basado en el microcontrolador **STM32F070RE** gobernado por **FreeRTOS**, el cual se comunica vía puerto serie con una Interfaz Hombre-Máquina (HMI) desarrollada en **Windows Forms (C#)**.

## 🎯 Objetivos del Proyecto

- Programación en C para microcontroladores usando un Sistema Operativo en Tiempo Real (**FreeRTOS**).
- Gestión concurrente mediante Tareas, Colas (*Queues*) y Semáforos.
- Manejo de periféricos e interrupciones: `Timers`, `ADC`, `UART`, `SPI`, e `I2C`.
- Diseño de interfaces de usuario (GUI) para la monitorización y control remoto.
- Implementación del bus de comunicación industrial **CAN Bus** para la lectura remota entre múltiples placas.

## 🛠️ Hardware Utilizado

El entorno de trabajo se basa en una placa entrenadora con los siguientes componentes:
*   **Microcontrolador:** STM32 Nucleo (STM32F070RE).
*   **Actuadores:**
    *   Servomotor.
    *   Motor paso a paso (con driver de potencia).
*   **Sensores:**
    *   Sensor de temperatura LM35 (Analógico - ADC).
    *   Sensor de distancia ToF (Time of Flight - Timers/Input Capture).
*   **Visualización Local:** Pantalla OLED/LCD I2C (Controlador SSD1306).
*   **Comunicaciones:** Módulo CAN Bus basado en el chip MCP2515 (Vía SPI).

---

## 🖥️ Arquitectura del Software (STM32 / FreeRTOS)

El firmware se ha diseñado utilizando múltiples hilos de ejecución (Tasks) para asegurar que el sistema no se bloquee y pueda atender a los sensores, actuadores y comunicaciones simultáneamente:

*   `recepcion_pserie_task`: Lee tramas UART usando una cola de mensajes gestionada por interrupciones (ISR).
*   `transmision_pserie_task`: Envía la telemetría al PC protegida por semáforos.
*   `motor_pp_task`: Controla la secuencia de fases del motor paso a paso.
*   `lectura_temp_task` / `transmit_temp_task`: Lee el ADC del LM35 y calcula la temperatura.
*   `servo_task`: Genera la señal PWM requerida y ejecuta barridos programados.
*   `time_of_fly_task`: Mide el ancho de pulso (Echo) mediante Input Capture del Timer.
*   `I2C_pantalla_task`: Actualiza la pantalla local con el estado del sistema.
*   `CAN_TX_Task` / `CAN_RX_Task`: Gestiona la transmisión y recepción de datos con otras placas industriales.

---

## 💻 Interfaz Gráfica (HMI)

La interfaz desarrollada en Visual Studio permite al usuario controlar el sistema completo de forma intuitiva:

![Interfaz HMI](Assets/trabajo_rfm.exe)

### Funcionalidades de la HMI:
1.  **Monitorización de Sensores:** Visualización en tiempo real de temperatura (ºC) y distancia (cm).
2.  **Control del Motor Paso a Paso:** Ajuste de velocidad (RPM) mediante un potenciómetro virtual (Knob) y cambio de sentido de giro.
3.  **Control del Servomotor:** Control de posición estática (0º - 90º) o modo automático (barrido de apertura/cierre con ajustes de velocidad y tiempos de espera).
4.  **Monitorización CAN Bus:** Lectura del estado del bus (Normal, Error Pasivo, Bus Off), contadores TEC/REC y datos remotos (telemetría de la placa de otro compañero).
5.  **Parada de Emergencia:** Botón "Cancelar Todo" que detiene todos los comandos activos, apaga motores y limpia pantallas.

---

## 📡 Protocolo de Comunicación (UART)

Para la comunicación entre el PC y el STM32 se ha diseñado un protocolo basado en tramas de **4 bytes**. 

**Estructura de la trama:** `[ CMD | DATO_0 | DATO_1 | EOF ]`
*El byte `EOF` (End of Frame) siempre es `0xE0`.*

### Ejemplos de Comandos (PC -> STM32)
| Acción | CMD | DATO 0 | DATO 1 | EOF |
| :--- | :---: | :---: | :---: | :---: |
| Modificar parpadeo LED | `0x4C` | Freq MSB | Freq LSB | `0xE0` |
| Girar motor P-P (Velocidad) | `0x4D` | Vel (RPM) | `0xFF` | `0xE0` |
| Posicionar Servo | `0x53` | Pos (0-90) | `0xFF` | `0xE0` |
| Barrido automático Servo | `0x41` | Espera (s)| Vel (º/s)| `0xE0` |
| Iniciar envío de Distancia | `0x60` | `0xFF` | `0xFF` | `0xE0` |
| Cancelar comandos | `0x43` | `0xFF` | `0xFF` | `0xE0` |

*(El repositorio incluye el código para manejar todas las tramas de ida y vuelta, incluyendo enteros y partes decimales).*

---

## 🚀 Cómo ejecutar el proyecto

1. **Firmware STM32:**
   * Abre el proyecto en **STM32CubeIDE**.
   * Compila el proyecto (`Project -> Build Project`).
   * Carga el firmware a la placa STM32 Nucleo mediante ST-Link.

2. **Aplicación PC (HMI):**
   * Abre la solución en **Visual Studio** (preferiblemente 2019 o superior).
   * Restaura los paquetes NuGet si es necesario (se utiliza `LBSoft.IndustrialCtrls` para los Knobs y Gauges).
   * Compila y ejecuta la aplicación.
   * Selecciona el puerto COM correspondiente a la placa Nucleo y pulsa "OPEN".
