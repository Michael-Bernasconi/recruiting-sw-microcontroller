# recruiting-sw-microcontroller
This project implements a real-time signal acquisition and event-handling system on the Nucleo C031C6 microcontroller. The firmware manages analog and digital inputs, organizes execution flow using a Finite State Machine (FSM), and streams data in real time via UART for visualization.

## 🔰 Project Objectives
The firmware demonstrates:
- Real-time acquisition of an analog signal using ADC1 (polling).
- Handling of asynchronous digital events via hardware interrupt.
- Clear and modular code organization using STM32 HAL libraries.
- Generation of three signal modes: RAW, NOISE (raw + random noise), and Moving Average (average over 150 samples).
- Visualization of digital interrupt events.
- Playback of an acoustic signal during continuous analog acquisition.

## ⚡ Hardware Setup
| Component | Function |
|----------|----------|
| Nucleo C031C6 | Main microcontroller board |
| Potentiometer | Provides analog input to ADC1 |
| USER Button | Used to change FSM state in polling mode |
| External Button | Triggers a digital interrupt event |
| Green LED | Indicates current system state |
| UART2 | Serial data output at 115200 baud |

## 👷‍♂️ System Architecture
The core of the firmware is a Finite State Machine that controls system behavior. The main states are:

- STATE_INIT: System configuration.
- STATE_WAIT_REQUEST: Waiting for the user to start acquisition.
- STATE_LISTENING: Analog signal acquisition, processing and UART transmission.
- STATE_PAUSE: Temporary acquisition pause with LED blinking.
- STATE_WARNING: Analog signal exceeded threshold for a prolonged time.
- STATE_ERROR: Critical threshold exceeded, system resets.
Transitions occur via USER button press or analog signal threshold checks.

## 📤 Data Output Example
Data is streamed to a serial monitor in real time:
- Mode=RAW | ADC=1082 -> 871 mV 
- Mode=MA | ADC=1062 -> 855 mV  
- Mode=NOISE | ADC=1106 -> 891 mV  
Warnings and error messages are also printed, while the green LED reflects state changes.

## 📊 Data Visualization
Data can be visualized using:
- Arduino Serial Plotter
- Python live plotting scripts
- Any serial monitoring tool with graph support
