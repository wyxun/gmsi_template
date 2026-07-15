# AT32F407 CNC Pin Mapping Specification

This document details the mapping between the standard Arduino CNC Shield pin nomenclature and the physical ports of the AT32F407 MCU, used for stepper motor control, spindle logic, limits, and UART interface.

## Pin Mapping Table

| Arduino PIN | CNC Shield Function | AT32F407 Pin | Default Alt Function | Signal Type | Direction |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **D0** | RX | **PA3** | USART2_RX | Digital | Input |
| **D1** | TX | **PA2** | USART2_TX | Digital | Output |
| **D2** | X-STEP | **PA10** | GPIO | Digital | Output |
| **D3** | Y-STEP | **PB3** | TMR2_CH2 / GPIO | Digital | Output |
| **D4** | Z-STEP | **PB5** | GPIO | Digital | Output |
| **D5** | X-DIR | **PB4** | TMR3_CH1 / GPIO | Digital | Output |
| **D6** | Y-DIR | **PB10** | TMR2_CH3 / GPIO | Digital | Output |
| **D7** | Z-DIR | **PA8** | GPIO | Digital | Output |
| **D8** | EN (Stepper Enable) | **PA9** | GPIO | Digital | Output |
| **D9** | X-EndStop (Limit) | **PC7** | TMR3_CH2 / GPIO | Digital | Input (Pull-up) |
| **D10** | Y-EndStop (Limit) | **PA15 / PB6** | TMR4_CH1 / GPIO | Digital | Input (Pull-up) |
| **D11** | Z-EndStop (Limit) | **PA7** | TMR3_CH2 / GPIO | Digital | Input (Pull-up) |
| **D12** | SpinEnable (Spindle) | **PA6** | TMR3_CH1 / GPIO | Digital | Output / PWM |
| **D13** | SpinDir (Spindle Dir) | **PA5** | GPIO | Digital | Output |
| **GND** | Ground | **GND** | Ground Reference | Power | Reference |
| **AREF** | Analog Reference | **VREF+** | VREF+ Input/Output | Analog | Reference |

---

## Configuration Guidelines

### 1. Stepper Motor Control (D2 ~ D8)
* **EN Pin (PA9)**: Must be configured as a standard GPIO output. Stepper drivers typically expect a **LOW** level to enable the outputs (holding torque / running).
* **STEP Pins (PA10, PB3, PB5)**: Set as high-speed push-pull outputs. Driven by the hardware timer interrupts at microsecond resolution.
* **DIR Pins (PB4, PB10, PA8)**: Set as push-pull outputs. Must be updated *before* firing a step pulse (adhering to driver setup timing).

### 2. Limit Inputs (D9 ~ D11)
* Internal pull-ups must be configured in `GPIO_Init` for PC7, PA15/PB6, and PA7 to prevent floating triggers when limit switches are normally open (NO) or unconnected.
* Debouncing can be handled in software or by grblHAL's core limits module.

### 3. Spindle Control (D12 ~ D13)
* **SpinEnable (PA6)**: If simple ON/OFF control is configured, set as GPIO output. If variable spindle speed control is desired, configure PA6 for PWM output utilizing the AT32 internal timer (TMR3_CH1).
