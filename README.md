# yarobot_control_unit

## Latest Approach: 7 Motors on ESP32-S3

ESP32-S3 based motor control unit for YaRobot, controlling 7 motors (5 servos + 2 steppers) with advanced peripherals:
- **5 Servo drives** (X, Y, Z, A, B) with position feedback and index pulses
- **2 Stepper motors** (C, D) with discrete position control
- **Shift registers** for direction and enable signals (saves GPIO pins)
- **I2C expanders** for limit switches and general I/O
- **USB CDC** command interface with human-readable text commands

## Architecture

### Motor Channel Mapping
- **X** - Servo on RMT Channel 0 + DMA (railway X axis)
- **Y** - Servo on MCPWM + PCNT (gripper Y axis, dedicated peripheral)
- **Z** - Servo on RMT Channel 1 + DMA (selector Z axis)
- **A** - Servo on RMT Channel 2 + DMA (picker Z axis)
- **B** - Servo on RMT Channel 3 + DMA (picker Y axis)
- **C** - Stepper on MCPWM + PCNT (picker jaw with floating switch for object measurement)
- **D** - Stepper on LEDC (picker retractor, discrete positions)

### Peripheral Assignment
- **4 RMT channels** with DMA for complex servo motion profiles (X, Z, A, B)
- **MCPWM + PCNT** for Y servo (dedicated) and C/D steppers (shared)
- **2x 74HC595 shift registers** via SPI for 14 direction/enable signals
- **2x I2C expanders** for limit switches and auxiliary I/O
- **Direct GPIO** for critical signals (E-stop, step pulses, position feedback)

### Key Features
- Hardware pulse generation with position feedback
- Z-signal (index pulse) support for absolute positioning
- Independent C/D motor control (C on MCPWM, D on LEDC)
- Trapezoidal motion profiles (S-curve planned for future)
- Real-time safety monitoring with hardware E-stop
- Simple OLED display for motor tuning (position/target/status)
- ROS2 control framework compatibility (position/velocity interfaces)
- Floating switch on C axis for object width measurement

## Hardware Connections

### Motor Control Signals
- Step pulses: Direct GPIO outputs
- Direction/Enable: Via shift registers (SPI)
- Position complete: Direct GPIO inputs (servos only)
- Z-signal: Direct GPIO inputs (servos only)

### I2C Expanders
- Address 0x20: 14 limit switches (7 motors × 2 limits)
- Address 0x21: 16 general purpose I/O

### OLED Display
- I2C interface (address 0x3C typical)
- 128x64 pixels showing motor positions and status

## Software Architecture

### FreeRTOS Tasks
1. **Safety Monitor** (Priority 24) - E-stop, limits, faults
2. **Motion Control** (Priority 25) - RMT/MCPWM pulse generation
3. **USB Communication** (Priority 20/19) - Command processing
4. **Command Executor** (Priority 15) - Motion planning
5. **I2C Monitor** (Priority 10) - Peripheral health checks
6. **Display Update** (Priority 5) - OLED status display

### Command Interface
Text-based commands via USB CDC (ROS2 compatible design):
```
MOVE X 100.0    # Move X to 100mm absolute
MOVR Y -10.0    # Move Y -10mm relative  
VEL Z 0.05      # Set Z velocity to 0.05m/s
HOME A          # Home A axis
STAT ALL        # Get all axes status
ENA XY          # Enable X and Y motors
DIS CD          # Disable C and D motors
MEAS C          # Get object width from C axis floating switch
```

## Documentation

See `/docs` folder for detailed documentation:
- [GPIO Allocation](docs/gpio-allocation-7motors.md) - Pin assignments for 7 motors
- [Shift Register Control](docs/shift-register-control.md) - DIR/EN signal management
- [Unified C/D Control](docs/unified-cd-stepper-control.md) - Shared peripheral system
- [Channel Mapping](docs/channel-letter-mapping.md) - Motor letter designations
- [Command Interface](docs/command-interface.md) - Complete command reference
- [Safety System](docs/safety-interlocks-and-fault-detection.md) - Safety architecture
- [FreeRTOS Architecture](docs/freertos-task-architecture.md) - Task priorities and communication
- [NVS Organization](docs/nvs-organization.md) - Configuration storage

## Building and Flashing

### Prerequisites
- ESP-IDF v5.0 or later
- ESP32-S3 DevKit with minimum 8MB flash

### Build
```bash
idf.py build
```

### Flash
```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

## Status
🚧 **Under Development** - Architecture defined, implementation in progress

### Development Phases
1. **Phase 1** (Current): Serial command interface development and testing
2. **Phase 2**: Mock ROS2 interface validation
3. **Phase 3**: Full ROS2 hardware interface integration

### Future Development (Post-MVP)
- Ethernet interface
- Modbus RTU communication with drivers
- Encoder feedback for closed-loop control
- S-curve motion profiles
- G-code compatibility layer

---

## Legacy Approach (Reference)
ESP32-S3 based servo drive and stepper motor control unit with discrete inputs and outputs, OLED screen for status display/ ESP-IDF framework, RTM DMA peripherals dynamically assigned to the GPIOs which control active drive (4 simultaneously, up to 10 total), I2C port expander for EN and DIR pins, as well as for discrete I/O if needed, potentially RS-485 MODBUS RTU communication for reading status from servo and stepper driver units.