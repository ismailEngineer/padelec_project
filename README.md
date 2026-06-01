# PADELEC PROJECT – TECHNICAL DOCUMENTATION

## 1. System Overview

The project is a motor-control and communication platform based on:

- STM32F446RE (main motor controller)
- ESP32 (wireless communication layer)
- Python / PyQt5 GUI (desktop supervision)
- Arduino Serial Bridge (optional USB/UART bridge)

Architecture:

GUI <-> Serial/UART <-> STM32F446RE <-> ESP32 Bridge <-> ESP-NOW Remote

Additional BLE firmware is provided on ESP32 for mobile application integration.

---

# 2. STM32F446RE Firmware

## MCU

- Device: STM32F446RET6
- Package: LQFP64
- Framework: STM32Cube HAL
- IDE: STM32CubeIDE
- Firmware Package: STM32Cube FW_F4 V1.28.3

## Peripheral Summary

| Peripheral | Function |
|------------|----------|
| ADC1 | Current/Voltage acquisition |
| DMA2 Stream0 | ADC transfer |
| TIM3 | Hall sensor interface |
| TIM4 | PWM outputs (low-side) |
| TIM8 | PWM outputs (high-side) |
| USART1 | BMS communication |
| USART3 | ESP32 communication |
| UART5 | Debug communication |
| GPIO | LEDs and control signals |

---

## Clock Configuration

Configured in .ioc:

- Clock source: HSI
- PLL frequency configured: 168 MHz
- USB clock: 48 MHz

Current generated firmware starts from HSI.

---

## GPIO Mapping

### Analog Inputs

| Pin | Label | Function |
|------|--------|----------|
| PA0 | IA | Phase current A |
| PA1 | IB | Phase current B |
| PA2 | IC | Phase current C |
| PA3 | Voltage | Bus/Battery voltage |

### LEDs

| Pin | Label |
|------|-------|
| PA4 | RED_LED |
| PA5 | GREEN_LED |
| PC4 | YELLOW_LED |

### Hall Sensors

| Pin | Label |
|------|-------|
| PA6 | HALL_SENSOR_A |
| PB5 | HALL_SENSOR_C |
| PB0 | HALL_SENSOR_B |

### MOSFET PWM Outputs

High-side:

| Pin | Function |
|------|----------|
| PC6 | MOSFET_1_1 |
| PC7 | MOSFET_2_1 |
| PC8 | MOSFET_3_1 |

Low-side:

| Pin | Function |
|------|----------|
| PB6 | MOSFET_1_2 |
| PB7 | MOSFET_2_2 |
| PB8 | MOSFET_3_2 |

### Communication

#### USART1 – BMS

| Pin | Signal |
|------|--------|
| PA9 | TX_COM_BMS |
| PA10 | RX_COM_BMS |

#### USART3 – ESP32

| Pin | Signal |
|------|--------|
| PC10 | TX_COM_ESP |
| PC5 | RX_COM_ESP |

Baudrate: 115200

#### UART5 – Debug

| Pin | Signal |
|------|--------|
| PC12 | TX_COM_DEBUG |
| PD2 | RX_COM_DEBUG |

---

# ADC1 Configuration

Purpose:
- Phase current measurement
- Voltage measurement

Configuration:

- Resolution: 12 bits
- Scan mode: Enabled
- Number of conversions: 2
- Sampling time: 56 cycles
- Trigger: Software
- DMA support configured

DMA:

- DMA2 Stream0
- Circular mode
- Peripheral → Memory
- High priority

Configured channels:

| ADC Channel | Pin |
|-------------|-----|
| ADC_CHANNEL_0 | PA0 |
| ADC_CHANNEL_1 | PA1 |

Future extension:
PA2 and PA3 are already allocated for additional current and voltage measurements.

---

# TIM3 – Hall Sensor Interface

Purpose:
Rotor position detection using Hall sensors.

Pins:

- CH1 → PA6
- CH2 → PB5
- CH3 → PB0

Mode:

- XOR Hall Sensor Interface

Applications:

- Rotor position tracking
- Electrical angle estimation
- Commutation control
- Speed estimation

---

# TIM4 – PWM Generator

Purpose:
Three PWM outputs for MOSFET driving.

Channels:

| Channel | Pin |
|----------|-----|
| CH1 | PB6 |
| CH2 | PB7 |
| CH3 | PB8 |

Configuration:

- PWM Mode
- Active High
- Period = 1000

---

# TIM8 – PWM Generator

Purpose:
Complementary motor PWM outputs.

Channels:

| Channel | Pin |
|----------|-----|
| CH1 | PC6 |
| CH2 | PC7 |
| CH3 | PC8 |

Configuration:

- PWM Mode
- Active High
- Period = 1000
- Break disabled

---

# PWM Control

The firmware starts:

- TIM4 CH1/2/3
- TIM8 CH1/2/3

Function available:

set_pwm(channel, percent)

Expected command:

PWM;<channel>;<duty>

Example:

PWM;1;75

Meaning:

Channel 1 = 75% duty cycle

---

# UART Command Protocol

STM32 receives ASCII commands.

Format:

COMMAND;TARGET;VALUE

Examples:

PWM;1;50
PWM;2;80
LED;1;1
ADC;0;0

Responses:

PWM:<value>
ADC:<adc0>,<adc1>

---

# Interrupts

Enabled:

- USART1 IRQ
- USART3 IRQ
- ADC IRQ
- DMA2_Stream0 IRQ

USART3 interrupt is used for ESP32 communication.

---

# 3. ESP32 Receiver (ESP-NOW Bridge)

Location:

esp32/ESP32Receiver

Purpose:

Bridge between ESP-NOW wireless commands and STM32 UART.

Communication chain:

Remote Controller
→ ESP-NOW
→ ESP32
→ UART
→ STM32

UART:

- RXD2 = GPIO16
- TXD2 = GPIO17
- Baudrate = 115200

Message format:

PWM;1;70
LED;1;1
ADC;0;0

The ESP32 forwards messages directly to STM32.

---

# ESP-NOW Callback

Function:

onReceive()

Responsibilities:

1. Receive wireless frame
2. Decode ASCII payload
3. Extract:
   - type
   - target
   - value
4. Send UART frame to STM32

---

# 4. ESP32 BLE Firmware

Location:

esp32/ElectricRollerEsp32

Purpose:

Bluetooth Low Energy server intended for a mobile application.

Device Name:

ElectricSkate-ESP32

---

## BLE Service

Service UUID:

00000000-0000-0000-0000-000000000000

Characteristics:

### Command Characteristic

UUID:

00000000-0000-0000-0000-000000000001

Properties:

- READ
- WRITE

### Telemetry Characteristic

UUID:

00000000-0000-0000-0000-000000000002

Properties:

- NOTIFY

---

## Supported Commands

HELLO

Response:

HELLO_FROM_ESP32

LOCK

Response:

{"locked":true}

UNLOCK

Response:

{"locked":false}

SET_DRIVE_MODE:ECO

SET_DRIVE_MODE:NORMAL

SET_DRIVE_MODE:SPORT

---

## Telemetry JSON

Example:

{
 "speed":12.3,
 "totalMileage":42.300,
 "lastTripDistance":5.100,
 "batteryLevel":87,
 "locked":false,
 "drivingMode":"NORMAL"
}

Notification period:

500 ms

---

# 5. Desktop GUI

Location:

GUI/

Technology:

- Python
- PyQt5
- pyserial

Main file:

interface.py

---

## Features

### PWM Control

3 sliders:

- PWM1
- PWM2
- PWM3

Each slider sends:

PWM;channel;value

---

### ADC Acquisition

GUI periodically sends:

ADC;0;0

Expected STM32 response:

ADC:value1,value2

Displayed on LCD widgets:

- IA
- IB
- IC

---

### Serial Communication

Class:

SerialThread

Responsibilities:

- Open serial port
- Send PWM commands
- Read responses
- Update GUI

Default baudrate:

115200

---

### Available Widgets

- PWM sliders
- Current displays
- Voltage display
- Velocity display
- Serial port selector

---

# 6. Arduino Serial Bridge

Location:

arduino_serial_pc/

Purpose:

USB ↔ UART bridge

Connections:

| Arduino | STM32 |
|----------|--------|
| D8 | STM32 TX |
| D9 | STM32 RX |

SoftwareSerial used.

Data flow:

Python GUI
↔ USB
↔ Arduino
↔ UART
↔ STM32

---

# 7. Recommended Future Improvements

## Motor Control

- Six-step commutation
- FOC implementation
- Current loop PI regulator
- Speed loop PI regulator

## Safety

- Over-current protection
- Over-voltage protection
- Under-voltage protection
- Hall sensor fault detection

## Communication

- CAN Bus support
- BLE telemetry from STM32
- BMS protocol integration

## GUI

- Real-time plots
- Data logging
- Fault dashboard
- Firmware update support

---

Author: Ismail Hamrouni
