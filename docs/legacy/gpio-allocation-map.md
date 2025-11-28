# ESP32-S3 GPIO Allocation Map

## Overview
This document defines the GPIO pin allocation for the yarobot_control_unit ESP32-S3 motor controller supporting 6 axes (4 servo drives, 2 steppers).

## Pin Allocation Table

### RMT-Controlled Servo Drives (Axes 0-3)

| Signal Type | Axis 0 | Axis 1 | Axis 2 | Axis 3 | Description |
|------------|--------|--------|--------|--------|-------------|
| Step/Pulse | GPIO1 | GPIO2 | GPIO4 | GPIO5 | RMT channels for precise pulse generation |
| Direction | GPIO6 | GPIO7 | GPIO8 | GPIO9 | Direct GPIO control |
| Enable | Exp1-P0 | Exp1-P1 | Exp1-P2 | Exp1-P3 | Via I2C Expander 1 |
| Position Complete | GPIO14 | GPIO15 | GPIO16 | GPIO17 | Input from P100S drives |
| Z-signal | GPIO18 | GPIO19 | GPIO20 | GPIO21 | Index pulse for absolute position |

### MCPWM-Controlled Steppers (Axes 4-5)

| Signal Type | Axis 4 | Axis 5 | Description |
|------------|---------|---------|-------------|
| Step/Pulse | GPIO47 | GPIO48 | MCPWM units 0 and 1 |
| Direction | GPIO38 | GPIO39 | Direct GPIO control |
| Enable | Exp1-P4 | Exp1-P5 | Via I2C Expander 1 |

### System Interfaces

| Interface | Pins | Description |
|-----------|------|-------------|
| I2C Bus | SDA: GPIO40, SCL: GPIO41 | For OLED display and I/O expanders |
| Expander 1 INT | GPIO10 | Interrupt from I2C Expander 1 (end switches) |
| Expander 2 INT | GPIO11 | Interrupt from I2C Expander 2 (end switches, buttons) |
| E-Stop Input | GPIO12 | Direct GPIO interrupt for emergency stop button |
| UART Debug | TX: GPIO43, RX: GPIO44 | Serial debug interface |
| USB Native | GPIO19, GPIO20 | If using native USB instead of UART |

### I2C Expander Allocation (CJMCU-2317)

#### Expander 1 (Address 0x20) - Mixed I/O
- Pins 0-5: Motor Enable outputs (axes 0-5)
- Pins 6-11: Brake control outputs (axes 0-5)
- Pins 12-15: Reserved outputs
- INT_A connected to GPIO10

#### Expander 2 (Address 0x21) - Inputs
- Pins 0-11: End switches (2 per axis, MIN/MAX)
- Pin 12: Emergency stop button
- Pin 13: User button 1
- Pin 14: User button 2
- Pin 15: Reserved input
- INT_B connected to GPIO11

## Design Rationale

1. **RMT Channels (0-3)**: Used for servo drives requiring precise pulse timing and complex motion profiles
2. **MCPWM Units**: Used for simpler stepper control with built-in PWM generation
3. **Direct GPIO Control**: All motor control signals on native GPIOs for reliability
4. **I2C Expansion**: Non-critical I/O moved to expanders to conserve GPIOs

## Constraints

- Total GPIO usage: 28 of ~30 available GPIOs
- Avoided strapping pins: GPIO0, GPIO3, GPIO45, GPIO46
- Avoided flash pins: GPIO26-32
- USB pins (GPIO19-20) can be reconfigured if not using native USB