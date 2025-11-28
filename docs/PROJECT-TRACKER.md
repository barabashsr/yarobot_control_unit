# YaRobot Control Unit - Project Tracker

## Overview
This document tracks all project aspects, design decisions, and implementation topics for the YaRobot Control Unit ESP32-S3 based motor controller.

## Architecture Documents

### ✅ Completed Documentation
| Document | Description | Status |
|----------|-------------|---------|
| [README.md](../README.md) | Project overview and motor mapping | Updated - C on MCPWM, D on LEDC |
| [channel-letter-mapping.md](channel-letter-mapping.md) | X,Y,Z,A,B,C,D axis definitions | Updated - ROS2 compatibility added |
| [gpio-allocation-7motors.md](gpio-allocation-7motors.md) | GPIO pin assignments | Updated - C/D pins separated |
| [unified-cd-stepper-control.md](unified-cd-stepper-control.md) | C/D axis control (now independent) | Rewritten - Independent control |
| [command-interface.md](command-interface.md) | Serial command protocol with events | Updated - Event system added |
| [real-time-control-architecture.md](real-time-control-architecture.md) | Universal control principles | New - ROS2 compatible |
| [brake-control-system.md](brake-control-system.md) | Brake strategies and TPIC6B595N control | Updated - Shift register control |
| [schematics-design.md](schematics-design.md) | Hardware schematics with TPIC6B595N | New - 24V logic support |
| [servo-feedback-signals.md](servo-feedback-signals.md) | Z-Signal & InPos processing for servos | New - Core feedback concepts |
| [configuration-commands.md](configuration-commands.md) | CLEAR, RATIO, CALZ, LIMCFG, POLINV commands | New - Runtime configuration |
| [discrete-axis-e.md](discrete-axis-e.md) | Axis E linear drive control | New - Two-state actuator |
| [yaml-configuration-system.md](yaml-configuration-system.md) | YAML-based configuration with USB transfer | New - Primary config method |

| [oled-display-tech.md](oled-display-tech.md) | OLED debug display (128x64 SSD1306) | New - Tech info display |

| [event-queue-implementation.md](event-queue-implementation.md) | FreeRTOS event queue system | New - Inter-task communication |

| [homing-sequences.md](homing-sequences.md) | Homing state machine with Z-signal support | New - Safety-first approach |

### 📝 Documentation Needed
| Topic | Description | Priority |
|-------|-------------|----------|
| Error Recovery Procedures | I2C failures, position loss | Medium |

## Hardware Architecture

### Motor Control (Finalized - Updated)
- **X** - Railway X axis: RMT CH0 + DMA (servo)
- **Y** - Gripper Y axis: MCPWM0 Timer0 + PCNT0 (servo)
- **Z** - Selector Z axis: RMT CH1 + DMA (servo)
- **A** - Picker Z axis: RMT CH2 + DMA (servo)
- **B** - Picker Y axis: RMT CH3 + DMA (servo)
- **C** - Picker jaw: MCPWM0 Timer1 + PCNT1 (stepper with floating switch for object width measurement)
- **D** - Picker retractor: LEDC (stepper with calculated position)
- **E** - Linear drive: I2C expander outputs (two-state actuator with constant speed)

### I/O Architecture (In Progress)
```
I2C Bus (GPIO8=SDA, GPIO18=SCL):
├── 0x20: MCP23017 #1 - Limit Switches
│   ├── GPA0-1: X axis MIN/MAX
│   ├── GPA2-3: Y axis MIN/MAX
│   ├── GPA4-5: Z axis MIN/MAX
│   ├── GPA6-7: A axis MIN/MAX
│   ├── GPB0-1: B axis MIN/MAX
│   ├── GPB2-3: C axis MIN/MAX (GPB3 = floating switch)
│   └── GPB4-5: D axis MIN/MAX
│
├── 0x21: MCP23017 #2 - General Purpose I/O
│   ├── GPA0-7: 8 digital inputs
│   └── GPB0-7: 8 digital outputs
│
├── 0x22: MCP23017 #3 - Servo Feedback
│   ├── GPA0-4: Position complete inputs (X,Y,Z,A,B)
│   ├── GPA5-7: Reserved
│   ├── GPB0-7: Reserved/Future Use
│
└── 0x3C: SSD1306 OLED Display (128x64)
```

### Shift Register Allocation  
```
TPIC6B595N Chain (24 bits for 24V logic):
├── Register #1 (Motor Control):
│   ├── Bit 0: X_DIR (24V)
│   ├── Bit 1: X_EN (24V)
│   ├── Bit 2: Y_DIR (24V)
│   ├── Bit 3: Y_EN (24V)
│   ├── Bit 4: Z_DIR (24V)
│   ├── Bit 5: Z_EN (24V)
│   ├── Bit 6: A_DIR (24V)
│   └── Bit 7: A_EN (24V)
├── Register #2 (Motor Control):
│   ├── Bit 0: B_DIR (24V)
│   ├── Bit 1: B_EN (24V)
│   ├── Bit 2: C_DIR (24V)
│   ├── Bit 3: C_EN (24V)
│   ├── Bit 4: D_DIR (24V)
│   ├── Bit 5: D_EN (24V)
│   └── Bit 6-7: Spare
└── Register #3 (Brake Control):
    ├── Bit 0: X_BRAKE_RELAY
    ├── Bit 1: Y_BRAKE_RELAY
    ├── Bit 2: Z_BRAKE_RELAY
    ├── Bit 3: A_BRAKE_RELAY
    ├── Bit 4: B_BRAKE_RELAY
    └── Bit 5-7: Spare relays

Critical: VDD must be ≥24V for 24V logic levels!
```

## Software Architecture Topics

### ✅ Decided
1. **Communication**: Commands, Events, Errors architecture
2. **Testing Phases**: Serial → Mock ROS2 → Real ROS2
3. **Task Priorities**: Safety(24), Motion(25), USB(20/19)
4. **C Axis Measurement**: Event-driven on floating switch trigger (not command-based)
5. **D Axis**: Discrete positions with calculated state
6. **Brake Control**: TPIC6B595N shift registers instead of I2C
7. **24V Logic**: TPIC6B595N for motor driver EN/DIR pins
8. **Universal Interface**: All axes provide uniform position/velocity interfaces

### 🔄 In Discussion
1. **Configuration Commands**:
   - `CLEAR <axis>` - Clear position without moving
   - `RATIO <axis> <value>` - Set digital ratio/scaling
   - `CALZ <axis>` - Calibrate using Z-signal
   
2. **Z-Signal Handling**:
   - Hardware interrupt on GPIO
   - Latch position on rising edge
   - Use for absolute position calibration
   
3. **Position Complete Processing**:
   - Via MCP23017 expander
   - Generate events when servo reaches target
   - Use for motion sequencing

4. **Brake Control System**:
   - Multiple strategies: BRAKE_ON_DISABLE, BRAKE_ON_ESTOP, BRAKE_ON_IDLE, BRAKE_NEVER
   - Relay modules with optocoupler isolation
   - Fail-safe design (brake engaged on power loss)
   - Controlled via TPIC6B595N register #3

### 📋 To Be Discussed
1. **Homing Sequences**: Order, speeds, safety
2. **Error Recovery**: I2C failure modes, position loss
3. **Motion Profiles**: Trapezoidal vs S-curve implementation
4. **Coordinated Motion**: Multi-axis synchronization
5. **Power Management**: Brake behavior on power loss
6. **Thermal Management**: Motor driver temperature monitoring

## OLED Display Concept

### Normal Operation Display
```
X:125.5→150.0 M
Y: 50.0       
Z: 75.3 E2    
A: 90.0       
B:-45.0       
C: 30.0 W12.5 
D:EXT         
I:OK E:0 24.1°
```

Legend:
- Position → Target (if moving)
- M = Moving, E1/E2 = Limit switches
- W12.5 = Object width (C axis)
- I:OK = I2C status, E:0 = Error count

### Error Display Mode
```
!E-STOP ACTIVE!
─────────────
Last Error:
I2C TIMEOUT
Device: 0x20
─────────────
X:DISABLED
[RST] to clear
```

## Implementation Priorities

### Sprint 1: Core Infrastructure
1. FreeRTOS task architecture
2. I2C MCP23017 drivers
3. Basic command parser
4. Event queue system

### Sprint 2: Motion Control
1. RMT+DMA for X,Z,A,B
2. MCPWM+PCNT for Y,C
3. LEDC for D
4. Limit switch handling

### Sprint 3: Servo Features
1. Z-signal processing
2. Position complete handling
3. Brake control
4. OLED display

### Sprint 4: Advanced Features
1. Configuration commands
2. Calibration sequences
3. Error recovery
4. Performance optimization

## Testing Requirements

### Hardware Tests
- All 14 limit switches interrupt handling
- MCP23017 interrupt aggregation
- OLED update impact on timing
- I2C bus recovery mechanisms
- Shift register reliability
- Z-signal edge detection
- Position complete timing

### Software Tests
- Event queue overflow handling
- Command parsing edge cases
- Multi-axis simultaneous motion
- Error propagation
- Configuration persistence
- Memory leak detection

## Open Questions

1. **Servo Drivers**: Which servo drivers are being used? (affects position complete and Z-signal interface)
2. **Brake Specifications**: Voltage, current, engage/release times?
3. **Temperature Sensing**: Add thermistors for motor temperature?
4. **User Buttons**: How many needed on general purpose inputs?
5. **Status LEDs**: How many and what states to indicate?
6. **Mechanical Limits**: Soft limits vs hard limits configuration?

## References

### ESP32-S3 Resources
- [ESP-IDF RMT Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/rmt.html)
- [MCPWM Servo Control](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/mcpwm.html)
- [PCNT Pulse Counter](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/pcnt.html)

### Component Datasheets
- MCP23017 16-bit I/O Expander
- TPIC6B595N Power Shift Register (150mA, 50V max)
- SSD1306 OLED Controller
- 5V/24V Relay Modules with Optocoupler

## Recent Updates

### 2024 Hardware Architecture Changes
- Changed C axis from LEDC to MCPWM+PCNT for hardware position counting
- Changed D axis from MCPWM to LEDC for simplified discrete control
- Added floating switch on C axis for automatic object width measurement
- Switched from 74HC595 to TPIC6B595N for 24V logic level support
- Implemented relay-based brake control with fail-safe design
- Optimized GPIO usage: Internal MCPWM→PCNT routing for Y/C axes saves 2 pins

### Key Design Decisions
- C axis is for picking/grasping, width measurement is automatic side effect
- Object width measurement is event-driven, not command-driven
- VDD voltage on TPIC6B595N must be ≥ load voltage to prevent damage
- Universal real-time control architecture with ROS2 compatibility
- All axes provide uniform interfaces regardless of underlying hardware
- Internal MCPWM→PCNT routing eliminates external wiring for Y/C position counting

---
*This document is the central tracking point for all project decisions and pending topics. Last major update: Hardware architecture revision for independent C/D control.*