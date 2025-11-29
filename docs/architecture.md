# Architecture

## Executive Summary

The YaRobot Control Unit architecture is designed around a real-time FreeRTOS-based firmware running on ESP32-S3, providing a unified command interface for controlling 8 heterogeneous motor axes. The architecture prioritizes deterministic timing, safety-first design, and clean abstraction layers that hide motor-type complexity from the host interface.

Key architectural principles:
- **Dual-core separation**: Core 0 handles communication and safety, Core 1 handles motion control
- **Hardware-first timing**: RMT+DMA, MCPWM, and LEDC peripherals for jitter-free pulse generation
- **Fail-safe design**: Brakes engage on power loss, E-stop bypasses software entirely
- **Unified abstraction**: All motor types expose identical command interface

## Project Initialization

**First implementation step:**

```bash
# Create ESP-IDF project
idf.py create-project yarobot_control_unit
cd yarobot_control_unit

# Set target to ESP32-S3
idf.py set-target esp32s3

# Add ESP-IDF component dependencies
idf.py add-dependency "espressif/mcp23017^0.1.1"

# Configure for N16R8 variant (16MB Flash, 8MB PSRAM)
idf.py menuconfig
```

**Key sdkconfig settings to configure:**
- `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y`
- `CONFIG_SPIRAM_MODE_OCT=y` (Octal PSRAM)
- `CONFIG_FREERTOS_HZ=1000` (1ms tick for timing precision)
- `CONFIG_ESP_CONSOLE_USB_CDC=y` (USB CDC for serial)
- `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`

**ESP-IDF provides these decisions:**
- FreeRTOS (IDF SMP variant for dual-core)
- CMake-based build system
- Core peripheral drivers (RMT, MCPWM, LEDC, PCNT, SPI, I2C)
- USB stack with CDC support

## Decision Summary

| Category | Decision | Version | Affects FRs | Rationale |
| -------- | -------- | ------- | ----------- | --------- |
| Platform | ESP32-S3-DevKitC-1 N16R8 | - | All | 16MB Flash, 8MB PSRAM, dual-core, native USB |
| Framework | ESP-IDF | v5.4 | All | FreeRTOS SMP, mature peripheral drivers |
| Component Structure | Layered Architecture | - | All | Clear HAL/Driver/Control separation, testable |
| Motor Abstraction | Hybrid (Inheritance + Composition) | - | FR1-10 | Injectable pulse generators, mockable for tests |
| Shift Registers | 3x TPIC6B595N (24-bit chain) | - | FR3-5, FR14 | Native 24V, fail-safe brakes |
| I2C Expanders | 3x MCP23017 (on I2C0) | espressif/mcp23017 v0.1.1 | FR11, FR35-37 | Limit switches, general I/O |
| OLED Display | SSD1306 128x64 (on I2C1) | esp_lcd (ESP-IDF native) | FR47-47c | Dedicated bus isolates from critical I/O |


## Project Structure

```
yarobot_control_unit/
├── CMakeLists.txt                 # Top-level project CMake
├── sdkconfig                      # ESP-IDF configuration
├── sdkconfig.defaults             # Default config values
├── partitions.csv                 # Flash partition table
│
├── main/                          # Application entry point
│   ├── CMakeLists.txt
│   ├── main.cpp                   # app_main() entry
│   └── Kconfig.projbuild          # Project-specific menu options
│
├── components/
│   ├── hal/                       # Hardware Abstraction Layer
│   │   ├── gpio_hal/
│   │   ├── i2c_hal/
│   │   └── spi_hal/
│   │
│   ├── drivers/                   # Device drivers
│   │   ├── mcp23017/              # I2C GPIO expander (espressif/mcp23017 component)
│   │   ├── tpic6b595/             # Shift register chain (custom SPI driver)
│   │   └── oled/                  # OLED display (esp_lcd_panel_ssd1306 from ESP-IDF)
│   │
│   ├── pulse_gen/                 # Pulse generation interfaces
│   │   ├── include/
│   │   │   └── pulse_generator.h  # IPulseGenerator interface
│   │   ├── rmt_pulse_gen.cpp      # RMT+DMA implementation
│   │   ├── mcpwm_pulse_gen.cpp    # MCPWM implementation
│   │   └── ledc_pulse_gen.cpp     # LEDC implementation
│   │
│   ├── position/                  # Position tracking
│   │   ├── include/
│   │   │   └── position_tracker.h # IPositionTracker interface
│   │   ├── pcnt_tracker.cpp       # Hardware counter
│   │   └── time_tracker.cpp       # Time-based calculation
│   │
│   ├── motor/                     # Motor abstraction
│   │   ├── include/
│   │   │   ├── motor_base.h
│   │   │   ├── servo_motor.h
│   │   │   ├── stepper_motor.h
│   │   │   └── discrete_axis.h
│   │   └── src/
│   │
│   ├── control/                   # Control logic
│   │   ├── motion_controller/     # Coordinates all axes
│   │   ├── safety_monitor/        # E-stop, limits, faults
│   │   └── command_executor/      # Command processing
│   │
│   ├── interface/                 # External interfaces
│   │   ├── usb_cdc/               # USB serial interface
│   │   └── command_parser/        # Text command parsing
│   │
│   ├── config/                    # Configuration management
│   │   ├── nvs_manager/           # NVS read/write
│   │   └── yaml_parser/           # YAML config handling
│   │
│   └── events/                    # Event system
│       └── event_manager/         # Publish/subscribe events
│
├── test/                          # Unit tests (host-based)
│   └── components/
│
└── docs/                          # Documentation
    ├── architecture.md
    └── planning_phase/
```

## FR Category to Architecture Mapping

| FR Category | Architecture Component | Notes |
|-------------|----------------------|-------|
| Motor Control (FR1-10) | `components/motor/`, `components/pulse_gen/` | 8 axes via IMotor interface |
| Safety (FR11-18) | `components/control/safety_monitor/` | Core 0, highest priority |
| Communication (FR19-26) | `components/interface/usb_cdc/`, `command_parser/` | USB CDC, text protocol |
| Configuration (FR27-34) | `components/config/nvs_manager/`, `yaml_parser/` | NVS + serial YAML transfer |
| I/O (FR35-42) | `components/drivers/mcp23017/` | 3x I2C expanders |
| Monitoring (FR43-50) | `components/control/`, `components/drivers/oled/` | OLED + events |
| Modes (FR51-55) | `components/control/command_executor/` | State machine |
| Errors (FR56-61) | `components/control/safety_monitor/`, `events/` | Centralized error manager |

## Technology Stack Details

### Core Technologies

| Component | Technology | Version | Purpose |
|-----------|------------|---------|---------|
| MCU | ESP32-S3-DevKitC-1 N16R8 | - | 16MB Flash, 8MB PSRAM, 240MHz dual-core |
| Framework | ESP-IDF | 5.4 | FreeRTOS SMP, drivers, build system |
| RTOS | FreeRTOS | IDF bundled | Task management, queues, mutexes |
| USB | TinyUSB (IDF component) | - | USB CDC serial interface |
| NVS | ESP-IDF NVS | - | Persistent configuration storage |
| Build | CMake + idf.py | - | Component-based build |

### Peripheral Usage

| Peripheral | Assignment | Notes |
|------------|------------|-------|
| RMT | 4 channels (X,Z,A,B) | DMA-driven pulse generation |
| MCPWM | 2 timers (Y,C) | PWM + internal PCNT routing |
| LEDC | 1 channel (D) | Discrete stepper pulses |
| PCNT | 2 units (Y,C) | Hardware pulse counting |
| SPI2 | Shift registers | 24-bit DIR/EN/Brake chain |
| I2C0 | MCP23017 x3 | Limit switches, I/O, feedback |
| I2C1 | SSD1306 OLED | Status display (isolated bus) |

### Integration Points

**Host Interface (USB CDC):**
- Text commands at 115200+ baud
- Async event notifications
- YAML configuration transfer

**External Hardware:**
- STEP/DIR/EN to servo/stepper drivers (24V logic via shift registers)
- 14 limit switches via I2C expanders
- 5 brake relays via shift registers
- E-stop via direct GPIO

**Future Integration:**
- ROS2 hardware_interface (post-MVP)
- Python/C++ host libraries

## Implementation Patterns

These patterns ensure consistent implementation across all components:

### Motor Control Pattern

```cpp
// All motor types implement this interface
class IMotor {
public:
    virtual esp_err_t moveAbsolute(float position, float velocity) = 0;
    virtual esp_err_t moveRelative(float delta, float velocity) = 0;
    virtual esp_err_t stop() = 0;
    virtual esp_err_t stopImmediate() = 0;
    virtual float getPosition() const = 0;
    virtual bool isMoving() const = 0;
    virtual void enable(bool en) = 0;
};

// Composition: Motor uses injected pulse generator and position tracker
class ServoMotor : public IMotor {
private:
    std::unique_ptr<IPulseGenerator> pulse_gen_;
    std::unique_ptr<IPositionTracker> tracker_;
    ShiftRegisterController* shift_reg_;
    uint8_t axis_id_;
    // ...
};
```

### Command Handler Pattern

```cpp
// Each command type has a dedicated handler
typedef esp_err_t (*CommandHandler)(const ParsedCommand* cmd, Response* resp);

static const CommandEntry command_table[] = {
    { CMD_MOVE,  handle_move,  STATE_READY | STATE_MOVING },
    { CMD_STOP,  handle_stop,  STATE_ANY },
    { CMD_EN,    handle_enable, STATE_READY },
    // ...
};

// Dispatcher validates state before calling handler
esp_err_t dispatch_command(const ParsedCommand* cmd, Response* resp) {
    for (const auto& entry : command_table) {
        if (strcmp(cmd->verb, entry.verb) == 0) {
            if (!(current_state & entry.allowed_states)) {
                return ERR_MODE_BLOCKED;
            }
            return entry.handler(cmd, resp);
        }
    }
    return ERR_INVALID_COMMAND;
}
```

### Event Publication Pattern

```cpp
// Central event manager for decoupled communication
typedef struct {
    EventType type;
    uint8_t axis;
    union {
        float position;
        float width;
        uint8_t error_code;
    } data;
    int64_t timestamp;
} Event;

// Publishers (motion, safety, I/O)
EventManager::publish(Event{ .type = EVT_MOTION_COMPLETE, .axis = 2, .data.position = 100.0 });

// Subscribers (USB TX, OLED, logging)
EventManager::subscribe(EVT_MOTION_COMPLETE, [](const Event& e) {
    send_event_response("DONE %c %.2f", 'X' + e.axis, e.data.position);
});
```

## Consistency Rules

### Naming Conventions

**Files:**
```
components/motor/src/servo_motor.cpp    # snake_case for files
components/motor/include/servo_motor.h  # Header matches source
```

**Types and Classes:**
```cpp
class ServoMotor;              // PascalCase for classes
typedef struct axis_config_t;  // snake_case_t for C structs
enum class BrakeStrategy;      // PascalCase for enums
```

**Functions and Variables:**
```cpp
void handleMotionComplete();                   // camelCase for functions
float current_position;                        // snake_case for variables
static const int MAX_AXES = LIMIT_NUM_AXES;    // UPPER_CASE for constants
```

**GPIO and Config Defines:**
```cpp
// Defined in config_gpio.h - see gpio-assignment.md for actual pin numbers
#define GPIO_X_STEP         ...        // GPIO_ prefix for pin names
#define TIMING_DIR_SETUP_US ...        // TIMING_ prefix, value in µs
#define ERR_POSITION_LIMIT  "E005"     // ERR_ prefix for error codes
```

**Axis Naming:**
- Physical: X, Y, Z, A, B, C, D, E (letters)
- Index: 0-7 (array indices)
- Alias: User-defined strings (e.g., "RAILWAY", "GRIPPER")

### Code Organization

**Component Structure:**
```
components/motor/
├── CMakeLists.txt           # Component build config
├── Kconfig                  # Optional menuconfig
├── include/
│   ├── motor.h              # Public API only
│   └── motor_types.h        # Shared type definitions
└── src/
    ├── motor_base.cpp       # Base class implementation
    ├── servo_motor.cpp      # Servo-specific
    ├── stepper_motor.cpp    # Stepper-specific
    └── motor_private.h      # Internal declarations
```

**Header Guards:**
```cpp
#ifndef COMPONENTS_MOTOR_INCLUDE_MOTOR_H_
#define COMPONENTS_MOTOR_INCLUDE_MOTOR_H_
// ...
#endif  // COMPONENTS_MOTOR_INCLUDE_MOTOR_H_
```

### Error Handling

**Return Codes:**
```cpp
// Use ESP-IDF error codes for internal APIs
esp_err_t move_to_position(uint8_t axis, float pos) {
    if (axis >= LIMIT_NUM_AXES) return ESP_ERR_INVALID_ARG;
    if (!is_enabled(axis)) return ESP_ERR_INVALID_STATE;
    if (pos < limits[axis].min) return ESP_ERR_INVALID_SIZE;
    // ...
    return ESP_OK;
}
```

**Error Response Format:**
```
ERROR <code> <message>
ERROR E005 Position limit exceeded
ERROR E006 Emergency stop active
```

**Error Propagation:**
```cpp
// Check and propagate errors
esp_err_t err = move_to_position(axis, target);
if (err != ESP_OK) {
    send_error_response(map_esp_err_to_code(err));
    return err;
}
```

### Logging Strategy

**Log Levels (ESP-IDF standard):**
```cpp
ESP_LOGE(TAG, "Critical error: %s", msg);     // Always shown
ESP_LOGW(TAG, "Warning: axis %d limit", axis); // Operational issues
ESP_LOGI(TAG, "Motion complete: %.2f", pos);   // Normal events
ESP_LOGD(TAG, "Pulse count: %d", count);       // Development
ESP_LOGV(TAG, "ISR entry");                    // Trace (verbose)
```

**Tag Naming:**
```cpp
static const char* TAG = "motor";     // Component name
static const char* TAG = "safety";    // Functional area
static const char* TAG = "usb_cdc";   // Specific module
```

**Runtime Log Control:**
```cpp
// LOG command adjusts level
// LOG NONE | ERROR | WARN | INFO | DEBUG
esp_log_level_set("*", ESP_LOG_INFO);        // Default
esp_log_level_set("motor", ESP_LOG_DEBUG);   // Specific component
```

**Performance Logging:**
```cpp
// Use ESP_EARLY_LOG* for ISR-safe logging (limited)
// Never log from ISR in production - use queued events
void IRAM_ATTR some_isr(void* arg) {
    // DON'T: ESP_LOGI(TAG, "ISR triggered");
    // DO: Queue event for task logging
    xQueueSendFromISR(log_queue, &event, &woken);
}
```

## Compile-Time Configuration

All hardware assignments, timing constants, buffer sizes, and system parameters are defined in header files. **No magic numbers in source code.**

### MANDATORY: Header-Only Configuration Requirement

> **⚠️ ARCHITECTURE CONSTRAINT - APPLIES TO ALL EPICS AND STORIES**
>
> **Every configurable value MUST be defined in a header file.** This is non-negotiable.
>
> - **GPIO pins**: Use `GPIO_X_STEP`, never `GPIO_NUM_2`
> - **Timing values**: Use `TIMING_DIR_SETUP_US`, never `20`
> - **Buffer sizes**: Use `LIMIT_CMD_MAX_LENGTH`, never `256`
> - **Command strings**: Use `CMD_MOVE`, never `"MOVE"`
> - **Error codes**: Use `ERR_INVALID_AXIS`, never `"E002"`
> - **YAML keys**: Use `YAML_KEY_VELOCITY`, never `"velocity"`
> - **I2C addresses**: Use `I2C_ADDR_MCP23017_0`, never `0x20`
>
> **Validation Rule**: Code review MUST reject any:
> - Literal GPIO numbers in driver code
> - Hardcoded timing values
> - Magic numbers for buffer sizes or limits
> - String literals for commands, errors, or YAML keys
>
> **This requirement MUST be included in every epic and story context file.**

### Core Concepts

**1. Single Source of Truth**
Each configuration category lives in exactly one header file. GPIO pins are defined only in `config_gpio.h`, never scattered across driver files. When you need to change a pin, you change it in one place.

**2. Separation of Concerns**
Configuration files are organized by domain:
- **Hardware mapping** (`config_gpio.h`, `config_i2c.h`) — What physical pin connects to what
- **Peripheral assignment** (`config_peripherals.h`) — Which ESP32 peripheral handles which function
- **Behavior tuning** (`config_timing.h`, `config_limits.h`) — How the system behaves
- **Protocol definition** (`config_commands.h`) — The external interface
- **Default values** (`config_defaults.h`) — Initial runtime parameters

**3. Compile-Time vs Runtime Configuration**
| Compile-Time (headers) | Runtime (NVS/YAML) |
|------------------------|-------------------|
| GPIO assignments | Axis units, limits |
| Peripheral channels | Velocity, acceleration |
| Buffer sizes | Backlash compensation |
| Command syntax | Axis aliases |
| Error codes | System behavior |
| Feature flags | User preferences |

Compile-time = hardware/firmware structure (requires rebuild).
Runtime = operational parameters (changeable via commands).

**4. Self-Documenting Code**
Configuration defines serve as inline documentation:
```c
// Instead of hardcoded pin numbers:
gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);  // What is this pin for?

// Always use named constants from config headers:
gpio_set_direction(GPIO_X_STEP, GPIO_MODE_OUTPUT);  // Clear: X-axis step output
```

**5. Build-Time Validation**
Grouped defines enable compile-time checks:
```c
// config_limits.h
#define LIMIT_NUM_AXES          8
#define LIMIT_NUM_SERVOS        5
#define LIMIT_NUM_STEPPERS      2
#define LIMIT_NUM_DISCRETE      1

// In code:
static_assert(LIMIT_NUM_SERVOS + LIMIT_NUM_STEPPERS + LIMIT_NUM_DISCRETE == LIMIT_NUM_AXES,
              "Axis count mismatch!");
```

**6. Easy Hardware Variants**
To create a variant with different GPIO assignments:
1. Copy `config_gpio.h` to `config_gpio_variant.h`
2. Modify pin assignments
3. Include variant header in `config.h`
4. Rebuild

No source code changes needed — only configuration.

**7. Feature Flags**
Optional features compile in/out cleanly:
```c
#define FEATURE_OLED_DISPLAY    0   // Set to 1 to enable

// In code:
#if FEATURE_OLED_DISPLAY
    initOledDisplay();
    xTaskCreate(displayUpdateTask, "display", STACK_DISPLAY_TASK, ...);
#endif
```

### Configuration Header Structure (Complete Tree)

```
components/config/include/
│
├── config.h                          # Master include
│   ├── FIRMWARE_NAME                 # "YAROBOT_CONTROL_UNIT"
│   ├── FIRMWARE_VERSION_*            # Major, Minor, Patch, String
│   └── FEATURE_*                     # Feature flags (OLED, Z_SIGNAL, STREAMING)
│
├── config_gpio.h                     # GPIO pin assignments (grouped by AXIS)
│   │
│   ├── Servo Axes (STEP + Z_SIGNAL paired on same board row)
│   │   ├── X Axis (row 4)
│   │   │   ├── GPIO_X_STEP           # J1-4 (RMT CH0)
│   │   │   └── GPIO_X_Z_SIGNAL       # J3-4 (index pulse)
│   │   ├── Y Axis (row 5)
│   │   │   ├── GPIO_Y_STEP           # J1-5 (MCPWM T0)
│   │   │   └── GPIO_Y_Z_SIGNAL       # J3-5 (index pulse)
│   │   ├── Z Axis (row 6)
│   │   │   ├── GPIO_Z_STEP           # J1-6 (RMT CH1)
│   │   │   └── GPIO_Z_Z_SIGNAL       # J3-6 (index pulse)
│   │   ├── A Axis (row 7)
│   │   │   ├── GPIO_A_STEP           # J1-7 (RMT CH2)
│   │   │   └── GPIO_A_Z_SIGNAL       # J3-7 (index pulse)
│   │   └── B Axis (row 8)
│   │       ├── GPIO_B_STEP           # J1-8 (RMT CH3)
│   │       └── GPIO_B_Z_SIGNAL       # J3-8 (index pulse)
│   │
│   ├── Stepper Axes (no Z-signal)
│   │   ├── GPIO_C_STEP               # J1-9 (MCPWM T1)
│   │   └── GPIO_D_STEP               # J1-10 (LEDC CH0)
│   │
│   ├── E Axis                        # Discrete - DIR/EN via shift register (bits 21-22)
│   │
│   ├── InPos Signals                 # NOT GPIO! Via MCP23017 #2 (0x22)
│   │
│   ├── Shift Register (SPI2) - J1 pins 15-18
│   │   ├── GPIO_SR_OE                # Output enable (active-low)
│   │   ├── GPIO_SR_CS                # Latch
│   │   ├── GPIO_SR_MOSI              # Data
│   │   └── GPIO_SR_SCLK              # Clock
│   │
│   ├── I2C Bus 0 (MCP23017s) - J1 pins 11-14
│   │   ├── GPIO_I2C_SCL              # Clock (400kHz)
│   │   ├── GPIO_I2C_SDA              # Data
│   │   ├── GPIO_I2C_INT0             # MCP #0 interrupt (limits)
│   │   └── GPIO_I2C_INT1             # MCP #1 interrupt (GP I/O)
│   │
│   ├── Safety
│   │   └── GPIO_E_STOP               # Emergency stop input (J1-19)
│   │
│   ├── OLED I2C Bus 1 (isolated)
│   │   ├── GPIO_OLED_SDA             # J1-20
│   │   └── GPIO_OLED_SCL             # J3-18
│   │
│   └── USB (Fixed - do not change)
│       ├── GPIO_USB_DN               # Native USB D-
│       └── GPIO_USB_DP               # Native USB D+
│
├── config_peripherals.h              # ESP32 peripheral assignments
│   ├── RMT Channels
│   │   ├── RMT_CHANNEL_X             # 0
│   │   ├── RMT_CHANNEL_Z             # 1
│   │   ├── RMT_CHANNEL_A             # 2
│   │   └── RMT_CHANNEL_B             # 3
│   │
│   ├── MCPWM
│   │   ├── MCPWM_GROUP_ID            # 0
│   │   ├── MCPWM_TIMER_Y             # 0
│   │   └── MCPWM_TIMER_C             # 1
│   │
│   ├── PCNT (Pulse Counter)
│   │   ├── PCNT_UNIT_Y               # 0
│   │   └── PCNT_UNIT_C               # 1
│   │
│   ├── LEDC
│   │   ├── LEDC_TIMER                # LEDC_TIMER_0
│   │   ├── LEDC_CHANNEL_D            # LEDC_CHANNEL_0
│   │   └── LEDC_MODE                 # LEDC_LOW_SPEED_MODE
│   │
│   └── SPI
│       └── SPI_HOST_SR               # SPI2_HOST
│
├── config_timing.h                   # Timing constants (all in ms/µs)
│   ├── Motion Timing
│   │   ├── TIMING_DIR_SETUP_US       # 20    - Direction setup
│   │   ├── TIMING_ENABLE_DELAY_US    # 50    - Enable to motion
│   │   ├── TIMING_BRAKE_ENGAGE_MS    # 50    - Brake engage time
│   │   └── TIMING_BRAKE_RELEASE_MS   # 30    - Brake release time
│   │
│   ├── Communication Timing
│   │   ├── TIMING_CMD_RESPONSE_MS    # 10    - Max response time
│   │   └── TIMING_USB_RX_TIMEOUT_MS  # 100   - USB receive timeout
│   │
│   ├── I2C Timing
│   │   ├── TIMING_I2C_POLL_MS        # 5     - Polling interval
│   │   ├── TIMING_I2C_TIMEOUT_MS     # 50    - Transaction timeout
│   │   └── TIMING_I2C_RETRY_COUNT    # 3     - Retry attempts
│   │
│   ├── Safety Timing
│   │   ├── TIMING_ESTOP_DEBOUNCE_MS  # 5     - E-stop debounce
│   │   ├── TIMING_LIMIT_DEBOUNCE_MS  # 10    - Limit switch debounce
│   │   ├── TIMING_IDLE_TIMEOUT_S     # 300   - Motor idle timeout
│   │   └── TIMING_SAFETY_POLL_MS     # 10    - Safety task poll interval
│   │
│   └── Task Periods
│       ├── PERIOD_I2C_MONITOR_MS     # 100   - I2C health check
│       ├── PERIOD_DISPLAY_UPDATE_MS  # 100   - OLED update
│       └── PERIOD_IDLE_CHECK_MS      # 1000  - Idle timeout check
│
├── config_limits.h                   # Buffer sizes, queue depths
│   ├── Buffer Sizes
│   │   ├── LIMIT_CMD_MAX_LENGTH      # 256   - Command line max
│   │   ├── LIMIT_RESPONSE_MAX_LENGTH # 256   - Response max
│   │   ├── LIMIT_YAML_BUFFER_SIZE    # 8192  - YAML buffer (8KB)
│   │   └── LIMIT_ALIAS_MAX_LENGTH    # 16    - Axis alias max
│   │
│   ├── Queue Depths
│   │   ├── LIMIT_COMMAND_QUEUE_DEPTH # 32
│   │   ├── LIMIT_RESPONSE_QUEUE_DEPTH# 32
│   │   ├── LIMIT_SAFETY_QUEUE_DEPTH  # 64
│   │   └── LIMIT_EVENT_QUEUE_DEPTH   # 32
│   │
│   ├── Axis Limits
│   │   ├── LIMIT_NUM_AXES            # 8     - Total axes
│   │   ├── LIMIT_NUM_SERVOS          # 5     - Servo count
│   │   ├── LIMIT_NUM_STEPPERS        # 2     - Stepper count
│   │   ├── LIMIT_NUM_DISCRETE        # 1     - Discrete count
│   │   └── LIMIT_CONFIG_SLOTS        # 10    - NVS slots
│   │
│   ├── Motion Limits
│   │   ├── LIMIT_MAX_PULSE_FREQ_HZ   # 100000 (100kHz)
│   │   ├── LIMIT_MIN_PULSE_FREQ_HZ   # 1
│   │   └── LIMIT_MOTION_QUEUE_DEPTH  # 4
│   │
│   └── Stack Sizes (words)
│       ├── STACK_SAFETY_TASK         # 4096
│       ├── STACK_USB_RX_TASK         # 2048
│       ├── STACK_USB_TX_TASK         # 2048
│       ├── STACK_CMD_EXECUTOR_TASK   # 8192
│       ├── STACK_I2C_MONITOR_TASK    # 2048
│       ├── STACK_MOTION_TASK         # 2048
│       ├── STACK_DISPLAY_TASK        # 4096
│       └── STACK_IDLE_MONITOR_TASK   # 2048
│
├── config_commands.h                 # Protocol strings & error codes
│   ├── Command Strings
│   │   ├── CMD_MOVE, CMD_MOVR, CMD_VEL, CMD_STOP
│   │   ├── CMD_POS, CMD_STAT, CMD_INFO
│   │   ├── CMD_EN, CMD_BRAKE, CMD_HOME, CMD_ZERO, CMD_CALB
│   │   ├── CMD_GETWIDTH, CMD_SETU, CMD_SETL, CMD_SETV, CMD_SETB, CMD_ALIAS
│   │   ├── CMD_DIN, CMD_DOUT, CMD_MODE
│   │   ├── CMD_SAVE, CMD_LOAD, CMD_RST, CMD_ECHO
│   │   ├── CMD_TEST, CMD_LOG, CMD_DIAG, CMD_STREAM
│   │   └── CMD_CFGSTART, CMD_CFGDATA, CMD_CFGEND, CMD_CFGEXPORT
│   │
│   ├── Response Prefixes
│   │   ├── RESP_OK                   # "OK"
│   │   ├── RESP_ERROR                # "ERROR"
│   │   ├── RESP_EVENT                # "EVENT"
│   │   ├── RESP_YAML                 # "YAML:"
│   │   └── RESP_STREAM               # "STRM"
│   │
│   ├── Error Codes
│   │   ├── ERR_INVALID_COMMAND       # "E001"
│   │   ├── ERR_INVALID_AXIS          # "E002"
│   │   ├── ERR_INVALID_PARAMETER     # "E003"
│   │   ├── ERR_AXIS_NOT_ENABLED      # "E004"
│   │   ├── ERR_POSITION_LIMIT        # "E005"
│   │   ├── ERR_EMERGENCY_STOP        # "E006"
│   │   ├── ERR_CALIBRATION_REQUIRED  # "E007"
│   │   ├── ERR_MOTOR_FAULT           # "E008"
│   │   ├── ERR_COMMUNICATION         # "E009"
│   │   ├── ERR_CONFIGURATION         # "E010"
│   │   ├── ERR_EVENT_OVERFLOW        # "E011"
│   │   ├── ERR_MODE_BLOCKED          # "E012"
│   │   └── ERR_MOTION_ACTIVE         # "E013"
│   │
│   └── Error Messages
│       └── MSG_* (corresponding message strings)
│
├── config_i2c.h                      # I2C addresses & pin mappings
│   ├── Bus Configuration
│   │   ├── I2C_PORT                  # I2C_NUM_0
│   │   └── I2C_FREQ_HZ               # 400000 (400kHz)
│   │
│   ├── Device Addresses
│   │   ├── I2C_ADDR_MCP23017_0       # 0x20 - Limit switches
│   │   ├── I2C_ADDR_MCP23017_1       # 0x21 - GP I/O + E axis
│   │   ├── I2C_ADDR_MCP23017_2       # 0x22 - Servo feedback (optional)
│   │   └── I2C_ADDR_SSD1306          # 0x3C - OLED display
│   │
│   ├── MCP23017 #0 Pin Mappings (Limit Switches)
│   │   ├── MCP0_X_LIMIT_MIN/MAX      # 0,1   - X axis
│   │   ├── MCP0_Y_LIMIT_MIN/MAX      # 2,3   - Y axis
│   │   ├── MCP0_Z_LIMIT_MIN/MAX      # 4,5   - Z axis
│   │   ├── MCP0_A_LIMIT_MIN/MAX      # 6,7   - A axis
│   │   ├── MCP0_B_LIMIT_MIN/MAX      # 8,9   - B axis
│   │   ├── MCP0_C_LIMIT_MIN/MAX      # 10,11 - C axis (floating switch)
│   │   ├── MCP0_D_LIMIT_MIN/MAX      # 12,13 - D axis
│   │   └── MCP0_E_LIMIT_MIN/MAX      # 14,15 - E axis
│   │
│   └── MCP23017 #1 Pin Mappings (General I/O)
│       ├── MCP1_GP_IN_0..7           # 0-7   - General inputs
│       └── MCP1_GP_OUT_0..7          # 8-15  - General outputs
│
├── config_defaults.h                 # Default runtime parameters
│   ├── Generic Defaults
│   │   ├── DEFAULT_UNITS_PER_PULSE   # 0.001f
│   │   ├── DEFAULT_LIMIT_MIN         # -1000.0f
│   │   ├── DEFAULT_LIMIT_MAX         # 1000.0f
│   │   ├── DEFAULT_MAX_VELOCITY      # 100.0f
│   │   ├── DEFAULT_MAX_ACCELERATION  # 500.0f
│   │   ├── DEFAULT_BACKLASH          # 0.0f
│   │   └── DEFAULT_HOME_OFFSET       # 0.0f
│   │
│   └── E-Axis Specific (discrete)
│       ├── E_AXIS_UNITS_PER_PULSE    # 1.0f
│       ├── E_AXIS_MAX_VELOCITY       # 1.0f
│       ├── E_AXIS_LIMIT_MIN          # 0.0f
│       └── E_AXIS_LIMIT_MAX          # 1.0f
│
├── config_oled.h                     # OLED display configuration
│   ├── OLED I2C Bus (dedicated, separate from main I2C)
│   │   ├── I2C_OLED_PORT             # I2C_NUM_1
│   │   ├── I2C_OLED_FREQ_HZ          # 400000 (400kHz)
│   │   ├── GPIO_OLED_SDA             # (defined in config_gpio.h)
│   │   └── GPIO_OLED_SCL             # (defined in config_gpio.h)
│   │
│   └── Display Parameters
│       ├── OLED_ADDRESS              # 0x3C
│       ├── OLED_WIDTH                # 128
│       ├── OLED_HEIGHT               # 64
│       └── OLED_UPDATE_HZ            # 2 (2Hz update rate)
│
├── config_sr.h                       # Shift register bit positions
│   ├── Per-Axis Bit Positions (3 bits per axis)
│   │   ├── SR_X_DIR, SR_X_EN, SR_X_BRAKE     # Bits 0-2
│   │   ├── SR_Y_DIR, SR_Y_EN, SR_Y_BRAKE     # Bits 3-5
│   │   ├── SR_Z_DIR, SR_Z_EN, SR_Z_BRAKE     # Bits 6-8
│   │   ├── SR_A_DIR, SR_A_EN, SR_A_BRAKE     # Bits 9-11
│   │   ├── SR_B_DIR, SR_B_EN, SR_B_BRAKE     # Bits 12-14
│   │   ├── SR_C_DIR, SR_C_EN, SR_C_BRAKE     # Bits 15-17
│   │   ├── SR_D_DIR, SR_D_EN, SR_D_BRAKE     # Bits 18-20
│   │   └── SR_E_DIR, SR_E_EN, SR_SPARE       # Bits 21-23
│   │
│   ├── Helper Macros
│   │   ├── SR_DIR_BIT(axis)          # Get DIR bit for axis
│   │   ├── SR_EN_BIT(axis)           # Get EN bit for axis
│   │   ├── SR_BRAKE_BIT(axis)        # Get BRAKE bit for axis
│   │   ├── SR_SET_BIT(data, bit)     # Set bit in data
│   │   └── SR_CLR_BIT(data, bit)     # Clear bit in data
│   │
│   └── Fail-Safe Defaults
│       └── SR_SAFE_STATE             # 0x000000 (all brakes engaged)
│
└── config_yaml_schema.h              # YAML configuration schema & validation
    ├── Schema Version
    │   └── YAML_SCHEMA_VERSION       # 1
    │
    ├── Root Keys
    │   ├── YAML_KEY_VERSION          # "version"
    │   ├── YAML_KEY_AXES             # "axes"
    │   └── YAML_KEY_SYSTEM           # "system"
    │
    ├── Axis Names (section keys)
    │   └── YAML_AXIS_X..E            # "X", "Y", "Z", "A", "B", "C", "D", "E"
    │
    ├── Per-Axis Parameter Keys
    │   ├── YAML_KEY_ALIAS            # "alias"
    │   ├── YAML_KEY_UNITS_PER_PULSE  # "units_per_pulse"
    │   ├── YAML_KEY_LIMITS           # "limits"
    │   ├── YAML_KEY_VELOCITY         # "velocity"
    │   ├── YAML_KEY_ACCELERATION     # "acceleration"
    │   ├── YAML_KEY_BACKLASH         # "backlash"
    │   └── YAML_KEY_HOME_OFFSET      # "home_offset"
    │
    ├── System Parameter Keys
    │   ├── YAML_KEY_DEBOUNCE_MS      # "debounce_ms"
    │   ├── YAML_KEY_IDLE_TIMEOUT_S   # "idle_timeout_s"
    │   └── YAML_KEY_LOG_LEVEL        # "log_level"
    │
    ├── NVS Key Mappings
    │   ├── NVS_KEY_PREFIX_AXIS       # "ax" (ax0_, ax1_, etc.)
    │   ├── NVS_KEY_SUFFIX_*          # _units, _min, _max, _vel, etc.
    │   └── NVS_KEY_SYS_*             # sys_version, sys_slot, sys_debounce
    │
    └── Validation Macros
        ├── YAML_MAX_VELOCITY(units)  # Computed from LIMIT_MAX_PULSE_FREQ_HZ
        ├── YAML_MIN_VELOCITY         # Computed from LIMIT_MIN_PULSE_FREQ_HZ
        └── YAML_VALIDATE_*           # Runtime validation helpers
```

### config_gpio.h — Pin Assignments

> **IMPORTANT:** For complete GPIO assignment details including physical board layout,
> shift register bit mapping, and MCP23017 pin assignments, see:
> **[GPIO Assignment Document](./gpio-assignment.md)**

```c
#ifndef CONFIG_GPIO_H
#define CONFIG_GPIO_H

// ============================================================================
// MOTOR STEP PULSE OUTPUTS - GROUPED BY AXIS
// ============================================================================
// All STEP outputs on LEFT side (J1), paired with Z_SIGNAL on RIGHT side (J3)
// Same row number = same physical position on board for clean cabling

// Servo axes (X, Y, Z, A, B) - STEP on left, Z_SIGNAL on right (same row)
#define GPIO_X_STEP         GPIO_NUM_4      // X-axis servo (RMT CH0) - J1-4 (row 4)
#define GPIO_Y_STEP         GPIO_NUM_5      // Y-axis servo (MCPWM T0) - J1-5 (row 5)
#define GPIO_Z_STEP         GPIO_NUM_6      // Z-axis servo (RMT CH1) - J1-6 (row 6)
#define GPIO_A_STEP         GPIO_NUM_7      // A-axis servo (RMT CH2) - J1-7 (row 7)
#define GPIO_B_STEP         GPIO_NUM_15     // B-axis servo (RMT CH3) - J1-8 (row 8)

// Stepper axes (C, D) - no Z-signal
#define GPIO_C_STEP         GPIO_NUM_16     // C-axis stepper (MCPWM T1) - J1-9
#define GPIO_D_STEP         GPIO_NUM_17     // D-axis stepper (LEDC CH0) - J1-10

// E-axis: DIR/EN via shift register (bits 21-22), no STEP GPIO needed (discrete actuator)

// ============================================================================
// SERVO Z-SIGNAL (INDEX) INPUTS - PAIRED WITH STEP OUTPUTS
// ============================================================================
// All Z_SIGNAL inputs on RIGHT side (J3), same row as corresponding STEP on left
// Hardware interrupts required for precise encoder index capture

#define GPIO_X_Z_SIGNAL     GPIO_NUM_1      // X servo index pulse - J3-4 (row 4)
#define GPIO_Y_Z_SIGNAL     GPIO_NUM_2      // Y servo index pulse - J3-5 (row 5)
#define GPIO_Z_Z_SIGNAL     GPIO_NUM_42     // Z servo index pulse - J3-6 (row 6)
#define GPIO_A_Z_SIGNAL     GPIO_NUM_41     // A servo index pulse - J3-7 (row 7)
#define GPIO_B_Z_SIGNAL     GPIO_NUM_40     // B servo index pulse - J3-8 (row 8)

// ============================================================================
// INPOS SIGNALS - VIA I2C EXPANDER (NOT GPIO!)
// ============================================================================
// InPos (Position Complete) signals are read via MCP23017 #2 (0x22)
// They don't require fast interrupt response - I2C polling is sufficient
// See config_i2c.h for MCP23017 #2 pin assignments

// ============================================================================
// SHIFT REGISTER CONTROL (SPI2/HSPI)
// ============================================================================
// Grouped on left side (J1 pins 15-18)
#define GPIO_SR_OE          GPIO_NUM_9      // Output enable (active-low) - J1-15
#define GPIO_SR_CS          GPIO_NUM_10     // Shift register latch - J1-16
#define GPIO_SR_MOSI        GPIO_NUM_11     // Shift register data - J1-17
#define GPIO_SR_SCLK        GPIO_NUM_12     // Shift register clock - J1-18

// ============================================================================
// I2C BUS 0 (MCP23017 Expanders)
// ============================================================================
// Grouped on left side (J1 pins 11-14)
#define GPIO_I2C_SCL        GPIO_NUM_18     // I2C clock - J1-11
#define GPIO_I2C_SDA        GPIO_NUM_8      // I2C data - J1-12
#define GPIO_I2C_INT0       GPIO_NUM_3      // MCP23017 #0 interrupt - J1-13
#define GPIO_I2C_INT1       GPIO_NUM_46     // MCP23017 #1 interrupt - J1-14

// ============================================================================
// SAFETY SIGNALS
// ============================================================================
#define GPIO_E_STOP         GPIO_NUM_13     // Emergency stop input - J1-19

// ============================================================================
// OLED I2C BUS (Dedicated bus, isolated from main I2C)
// ============================================================================
#define GPIO_OLED_SDA       GPIO_NUM_14     // OLED I2C data (I2C_NUM_1) - J1-20
#define GPIO_OLED_SCL       GPIO_NUM_21     // OLED I2C clock (I2C_NUM_1) - J3-18

// ============================================================================
// USB (Fixed by hardware - DO NOT CHANGE)
// ============================================================================
#define GPIO_USB_DN         GPIO_NUM_19     // USB D- - J3-20
#define GPIO_USB_DP         GPIO_NUM_20     // USB D+ - J3-19

#endif // CONFIG_GPIO_H
```

### config_peripherals.h — Peripheral Assignments

```c
#ifndef CONFIG_PERIPHERALS_H
#define CONFIG_PERIPHERALS_H

// ============================================================================
// RMT CHANNEL ASSIGNMENTS
// ============================================================================
#define RMT_CHANNEL_X       0               // X-axis RMT channel
#define RMT_CHANNEL_Z       1               // Z-axis RMT channel
#define RMT_CHANNEL_A       2               // A-axis RMT channel
#define RMT_CHANNEL_B       3               // B-axis RMT channel

// ============================================================================
// MCPWM ASSIGNMENTS
// ============================================================================
#define MCPWM_GROUP_ID      0               // MCPWM group
#define MCPWM_TIMER_Y       0               // Y-axis timer
#define MCPWM_TIMER_C       1               // C-axis timer

// ============================================================================
// PCNT (Pulse Counter) ASSIGNMENTS
// ============================================================================
#define PCNT_UNIT_Y         0               // Y-axis pulse counter
#define PCNT_UNIT_C         1               // C-axis pulse counter

// ============================================================================
// LEDC ASSIGNMENTS
// ============================================================================
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_CHANNEL_D      LEDC_CHANNEL_0  // D-axis LEDC channel
#define LEDC_MODE           LEDC_LOW_SPEED_MODE

// ============================================================================
// SPI ASSIGNMENTS
// ============================================================================
#define SPI_HOST_SR         SPI2_HOST       // Shift register SPI host

#endif // CONFIG_PERIPHERALS_H
```

### config_timing.h — Timing Constants

```c
#ifndef CONFIG_TIMING_H
#define CONFIG_TIMING_H

// ============================================================================
// MOTION TIMING
// ============================================================================
#define TIMING_DIR_SETUP_US         20      // Direction setup time (µs)
#define TIMING_ENABLE_DELAY_US      50      // Enable to motion delay (µs)
#define TIMING_BRAKE_ENGAGE_MS      50      // Brake engagement time (ms)
#define TIMING_BRAKE_RELEASE_MS     30      // Brake release time (ms)

// ============================================================================
// COMMUNICATION TIMING
// ============================================================================
#define TIMING_CMD_RESPONSE_MS      10      // Max command response time (ms)
#define TIMING_USB_RX_TIMEOUT_MS    100     // USB receive timeout (ms)

// ============================================================================
// I2C TIMING
// ============================================================================
#define TIMING_I2C_POLL_MS          5       // I2C polling interval (ms)
#define TIMING_I2C_TIMEOUT_MS       50      // I2C transaction timeout (ms)
#define TIMING_I2C_RETRY_COUNT      3       // I2C retry attempts

// ============================================================================
// SAFETY TIMING
// ============================================================================
#define TIMING_ESTOP_DEBOUNCE_MS    5       // E-stop debounce time (ms)
#define TIMING_LIMIT_DEBOUNCE_MS    10      // Limit switch debounce (ms)
#define TIMING_IDLE_TIMEOUT_S       300     // Motor idle timeout (seconds, 0=disabled)
#define TIMING_SAFETY_POLL_MS       10      // Safety task poll interval (ms)

// ============================================================================
// TASK PERIODS
// ============================================================================
#define PERIOD_I2C_MONITOR_MS       100     // I2C health check period (ms)
#define PERIOD_DISPLAY_UPDATE_MS    100     // OLED update period (ms)
#define PERIOD_IDLE_CHECK_MS        1000    // Idle timeout check period (ms)

#endif // CONFIG_TIMING_H
```

### config_limits.h — Buffer Sizes and Limits

```c
#ifndef CONFIG_LIMITS_H
#define CONFIG_LIMITS_H

// ============================================================================
// BUFFER SIZES
// ============================================================================
#define LIMIT_CMD_MAX_LENGTH        256     // Max command line length
#define LIMIT_RESPONSE_MAX_LENGTH   256     // Max response line length
#define LIMIT_YAML_BUFFER_SIZE      8192    // YAML receive buffer (8KB)
#define LIMIT_ALIAS_MAX_LENGTH      16      // Max axis alias length

// ============================================================================
// QUEUE DEPTHS
// ============================================================================
#define LIMIT_COMMAND_QUEUE_DEPTH   32      // Command queue size
#define LIMIT_RESPONSE_QUEUE_DEPTH  32      // Response queue size
#define LIMIT_SAFETY_QUEUE_DEPTH    64      // Safety event queue size
#define LIMIT_EVENT_QUEUE_DEPTH     32      // Event queue size

// ============================================================================
// AXIS LIMITS
// ============================================================================
#define LIMIT_NUM_AXES              8       // Total number of axes (X-E)
#define LIMIT_NUM_SERVOS            5       // Servo axes (X,Y,Z,A,B)
#define LIMIT_NUM_STEPPERS          2       // Stepper axes (C,D)
#define LIMIT_NUM_DISCRETE          1       // Discrete axes (E)
#define LIMIT_CONFIG_SLOTS          10      // NVS configuration slots (0-9)

// ============================================================================
// MOTION LIMITS
// ============================================================================
#define LIMIT_MAX_PULSE_FREQ_HZ     100000  // Maximum pulse frequency (100kHz)
#define LIMIT_MIN_PULSE_FREQ_HZ     1       // Minimum pulse frequency (1Hz)
#define LIMIT_MOTION_QUEUE_DEPTH    4       // Motion commands per axis

// ============================================================================
// STACK SIZES (words, not bytes)
// ============================================================================
#define STACK_SAFETY_TASK           4096
#define STACK_USB_RX_TASK           2048
#define STACK_USB_TX_TASK           2048
#define STACK_CMD_EXECUTOR_TASK     8192
#define STACK_I2C_MONITOR_TASK      2048
#define STACK_MOTION_TASK           2048
#define STACK_DISPLAY_TASK          4096
#define STACK_IDLE_MONITOR_TASK     2048

#endif // CONFIG_LIMITS_H
```

### config_commands.h — Command Strings and Error Codes

```c
#ifndef CONFIG_COMMANDS_H
#define CONFIG_COMMANDS_H

// ============================================================================
// COMMAND STRINGS
// ============================================================================
#define CMD_MOVE            "MOVE"
#define CMD_MOVR            "MOVR"
#define CMD_VEL             "VEL"
#define CMD_STOP            "STOP"
#define CMD_POS             "POS"
#define CMD_STAT            "STAT"
#define CMD_INFO            "INFO"
#define CMD_EN              "EN"
#define CMD_BRAKE           "BRAKE"
#define CMD_HOME            "HOME"
#define CMD_ZERO            "ZERO"
#define CMD_CALB            "CALB"
#define CMD_GETWIDTH        "GETWIDTH"
#define CMD_SETU            "SETU"
#define CMD_SETL            "SETL"
#define CMD_SETV            "SETV"
#define CMD_SETB            "SETB"
#define CMD_ALIAS           "ALIAS"
#define CMD_DIN             "DIN"
#define CMD_DOUT            "DOUT"
#define CMD_MODE            "MODE"
#define CMD_SAVE            "SAVE"
#define CMD_LOAD            "LOAD"
#define CMD_RST             "RST"
#define CMD_ECHO            "ECHO"
#define CMD_TEST            "TEST"
#define CMD_LOG             "LOG"
#define CMD_DIAG            "DIAG"
#define CMD_STREAM          "STREAM"
#define CMD_CFGSTART        "CFGSTART"
#define CMD_CFGDATA         "CFGDATA"
#define CMD_CFGEND          "CFGEND"
#define CMD_CFGEXPORT       "CFGEXPORT"

// ============================================================================
// RESPONSE PREFIXES
// ============================================================================
#define RESP_OK             "OK"
#define RESP_ERROR          "ERROR"
#define RESP_EVENT          "EVENT"
#define RESP_YAML           "YAML:"
#define RESP_STREAM         "STRM"

// ============================================================================
// ERROR CODES
// ============================================================================
#define ERR_INVALID_COMMAND         "E001"
#define ERR_INVALID_AXIS            "E002"
#define ERR_INVALID_PARAMETER       "E003"
#define ERR_AXIS_NOT_ENABLED        "E004"
#define ERR_POSITION_LIMIT          "E005"
#define ERR_EMERGENCY_STOP          "E006"
#define ERR_CALIBRATION_REQUIRED    "E007"
#define ERR_MOTOR_FAULT             "E008"
#define ERR_COMMUNICATION           "E009"
#define ERR_CONFIGURATION           "E010"
#define ERR_EVENT_OVERFLOW          "E011"
#define ERR_MODE_BLOCKED            "E012"
#define ERR_MOTION_ACTIVE           "E013"

// ============================================================================
// ERROR MESSAGES
// ============================================================================
#define MSG_INVALID_COMMAND         "Invalid command"
#define MSG_INVALID_AXIS            "Invalid axis"
#define MSG_INVALID_PARAMETER       "Invalid parameter"
#define MSG_AXIS_NOT_ENABLED        "Axis not enabled"
#define MSG_POSITION_LIMIT          "Position limit exceeded"
#define MSG_EMERGENCY_STOP          "Emergency stop active"
#define MSG_CALIBRATION_REQUIRED    "Calibration required"
#define MSG_MOTOR_FAULT             "Motor fault"
#define MSG_COMMUNICATION           "Communication error"
#define MSG_CONFIGURATION           "Configuration error"
#define MSG_EVENT_OVERFLOW          "Event buffer overflow"
#define MSG_MODE_BLOCKED            "Command blocked in current mode"
#define MSG_MOTION_ACTIVE           "Motion active - stop first"

#endif // CONFIG_COMMANDS_H
```

### config_i2c.h — I2C Device Addresses

```c
#ifndef CONFIG_I2C_H
#define CONFIG_I2C_H

// ============================================================================
// I2C BUS CONFIGURATION
// ============================================================================
#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         400000          // 400kHz Fast Mode

// ============================================================================
// MCP23017 I/O EXPANDER ADDRESSES
// ============================================================================
#define I2C_ADDR_MCP23017_0 0x20            // Limit switches (14 switches)
#define I2C_ADDR_MCP23017_1 0x21            // GP I/O + E axis control
#define I2C_ADDR_MCP23017_2 0x22            // Servo feedback signals (optional)

// ============================================================================
// SSD1306 OLED DISPLAY (on dedicated I2C_NUM_1 bus)
// OLED is isolated from main I2C to prevent display issues affecting I/O
// ============================================================================
// Note: OLED I2C configuration is in config_oled.h (separate bus)

// ============================================================================
// MCP23017 PIN MAPPINGS - EXPANDER 0 (Limit Switches)
// ============================================================================
#define MCP0_X_LIMIT_MIN    0               // GPA0
#define MCP0_X_LIMIT_MAX    1               // GPA1
#define MCP0_Y_LIMIT_MIN    2               // GPA2
#define MCP0_Y_LIMIT_MAX    3               // GPA3
#define MCP0_Z_LIMIT_MIN    4               // GPA4
#define MCP0_Z_LIMIT_MAX    5               // GPA5
#define MCP0_A_LIMIT_MIN    6               // GPA6
#define MCP0_A_LIMIT_MAX    7               // GPA7
#define MCP0_B_LIMIT_MIN    8               // GPB0
#define MCP0_B_LIMIT_MAX    9               // GPB1
#define MCP0_C_LIMIT_MIN    10              // GPB2
#define MCP0_C_LIMIT_MAX    11              // GPB3 (floating switch)
#define MCP0_D_LIMIT_MIN    12              // GPB4
#define MCP0_D_LIMIT_MAX    13              // GPB5
#define MCP0_E_LIMIT_MIN    14              // GPB6
#define MCP0_E_LIMIT_MAX    15              // GPB7

// ============================================================================
// MCP23017 PIN MAPPINGS - EXPANDER 1 (GP I/O + E axis)
// ============================================================================
#define MCP1_GP_IN_0        0               // GPA0 - General input
#define MCP1_GP_IN_1        1               // GPA1 - General input
#define MCP1_GP_IN_2        2               // GPA2 - General input
#define MCP1_GP_IN_3        3               // GPA3 - General input
#define MCP1_GP_IN_4        4               // GPA4 - General input
#define MCP1_GP_IN_5        5               // GPA5 - General input
#define MCP1_GP_IN_6        6               // GPA6 - General input
#define MCP1_GP_IN_7        7               // GPA7 - General input
#define MCP1_GP_OUT_0       8               // GPB0 - General output
#define MCP1_GP_OUT_1       9               // GPB1 - General output
#define MCP1_GP_OUT_2       10              // GPB2 - General output
#define MCP1_GP_OUT_3       11              // GPB3 - General output
#define MCP1_GP_OUT_4       12              // GPB4 - General output
#define MCP1_GP_OUT_5       13              // GPB5 - General output
#define MCP1_GP_OUT_6       14              // GPB6 - General output
#define MCP1_GP_OUT_7       15              // GPB7 - General output

// Note: E axis DIR/EN now controlled via shift register (bits 21-22)
// for unified control approach across all axes

#endif // CONFIG_I2C_H
```

### config_sr.h — Shift Register Bit Positions

```c
#ifndef CONFIG_SR_H
#define CONFIG_SR_H

// ============================================================================
// SHIFT REGISTER BIT POSITIONS
// ============================================================================
// Organization: 3 bits per axis [DIR, EN, BRAKE]
// Chain: MOSI → SR0 → SR1 → SR2 (3x TPIC6B595N)

// X-axis (Servo)
#define SR_X_DIR            0       // SR0.Q0 - Direction
#define SR_X_EN             1       // SR0.Q1 - Enable
#define SR_X_BRAKE          2       // SR0.Q2 - Brake release

// Y-axis (Servo)
#define SR_Y_DIR            3       // SR0.Q3
#define SR_Y_EN             4       // SR0.Q4
#define SR_Y_BRAKE          5       // SR0.Q5

// Z-axis (Servo)
#define SR_Z_DIR            6       // SR0.Q6
#define SR_Z_EN             7       // SR0.Q7
#define SR_Z_BRAKE          8       // SR1.Q0

// A-axis (Servo)
#define SR_A_DIR            9       // SR1.Q1
#define SR_A_EN             10      // SR1.Q2
#define SR_A_BRAKE          11      // SR1.Q3

// B-axis (Servo)
#define SR_B_DIR            12      // SR1.Q4
#define SR_B_EN             13      // SR1.Q5
#define SR_B_BRAKE          14      // SR1.Q6

// C-axis (Stepper - no physical brake)
#define SR_C_DIR            15      // SR1.Q7
#define SR_C_EN             16      // SR2.Q0
#define SR_C_BRAKE          17      // SR2.Q1 (not connected)

// D-axis (Stepper - no physical brake)
#define SR_D_DIR            18      // SR2.Q2
#define SR_D_EN             19      // SR2.Q3
#define SR_D_BRAKE          20      // SR2.Q4 (not connected)

// E-axis (Discrete)
#define SR_E_DIR            21      // SR2.Q5
#define SR_E_EN             22      // SR2.Q6
#define SR_SPARE            23      // SR2.Q7

// ============================================================================
// HELPER MACROS
// ============================================================================
#define SR_DIR_BIT(axis)    (SR_X_DIR + ((axis) * 3))
#define SR_EN_BIT(axis)     (SR_X_EN + ((axis) * 3))
#define SR_BRAKE_BIT(axis)  (SR_X_BRAKE + ((axis) * 3))

// Bit manipulation
#define SR_SET_BIT(data, bit)   ((data) | (1UL << (bit)))
#define SR_CLR_BIT(data, bit)   ((data) & ~(1UL << (bit)))
#define SR_GET_BIT(data, bit)   (((data) >> (bit)) & 1)

// ============================================================================
// FAIL-SAFE DEFAULTS
// ============================================================================
// On power-up or reset, all outputs LOW = all brakes engaged
#define SR_SAFE_STATE       0x000000    // All bits 0 = fail-safe

#endif // CONFIG_SR_H
```

### config_defaults.h — Default Axis Parameters

```c
#ifndef CONFIG_DEFAULTS_H
#define CONFIG_DEFAULTS_H

// ============================================================================
// DEFAULT AXIS CONFIGURATION
// These values are used when no NVS configuration exists
// ============================================================================

// Units per pulse (default: 0.001 = 1000 pulses/mm or 1000 pulses/degree)
#define DEFAULT_UNITS_PER_PULSE     0.001f

// Position limits (default: ±1000 units)
#define DEFAULT_LIMIT_MIN           -1000.0f
#define DEFAULT_LIMIT_MAX           1000.0f

// Velocity (default: 100 units/sec)
#define DEFAULT_MAX_VELOCITY        100.0f

// Acceleration (default: 500 units/sec²)
#define DEFAULT_MAX_ACCELERATION    500.0f

// Backlash compensation (default: none)
#define DEFAULT_BACKLASH            0.0f

// Home offset (default: none)
#define DEFAULT_HOME_OFFSET         0.0f

// ============================================================================
// AXIS-SPECIFIC OVERRIDES (use these instead of defaults where different)
// ============================================================================

// E-axis (discrete) - fixed speed, binary positions
#define E_AXIS_UNITS_PER_PULSE      1.0f    // 1 unit = 1 "step" (time-based)
#define E_AXIS_MAX_VELOCITY         1.0f    // Fixed speed
#define E_AXIS_LIMIT_MIN            0.0f
#define E_AXIS_LIMIT_MAX            1.0f    // 0=retracted, 1=extended

#endif // CONFIG_DEFAULTS_H
```

### config_oled.h — OLED Display Configuration

```c
#ifndef CONFIG_OLED_H
#define CONFIG_OLED_H

// ============================================================================
// OLED I2C BUS CONFIGURATION (Dedicated bus, isolated from main I2C)
// ============================================================================
#define I2C_OLED_PORT       I2C_NUM_1       // Dedicated I2C bus for OLED
#define I2C_OLED_FREQ_HZ    400000          // 400kHz Fast Mode

// GPIO pins defined in config_gpio.h:
// GPIO_OLED_SDA (J1-20)
// GPIO_OLED_SCL (J3-18)

// ============================================================================
// DISPLAY PARAMETERS
// ============================================================================
#define OLED_ADDRESS        0x3C            // SSD1306 I2C address
#define OLED_WIDTH          128             // Display width in pixels
#define OLED_HEIGHT         64              // Display height in pixels
#define OLED_UPDATE_HZ      2               // Update rate (2Hz = 500ms)

// ============================================================================
// DISPLAY LAYOUT
// ============================================================================
#define OLED_FONT_WIDTH     6               // Font character width
#define OLED_FONT_HEIGHT    8               // Font character height
#define OLED_CHARS_PER_LINE 21              // Characters per line (128/6)
#define OLED_NUM_LINES      7               // Total lines (64/8, all usable)

// Line allocations
#define OLED_AXIS_LINES     4               // Lines 0-3 for axis status
#define OLED_SEPARATOR_LINE 4               // Line 4 for separator
#define OLED_EVENT_LINES    2               // Lines 5-6 for events/errors

// ============================================================================
// TIMING
// ============================================================================
#define OLED_EVENT_DISPLAY_MS   2000        // Event display duration (2 sec)

#endif // CONFIG_OLED_H
```

### config_yaml_schema.h — YAML Configuration Schema

```c
#ifndef CONFIG_YAML_SCHEMA_H
#define CONFIG_YAML_SCHEMA_H

#include "config_limits.h"
#include "config_defaults.h"

// ============================================================================
// YAML SCHEMA VERSION
// Increment when schema structure changes (not just values)
// ============================================================================
#define YAML_SCHEMA_VERSION         1

// ============================================================================
// ROOT KEYS
// ============================================================================
#define YAML_KEY_VERSION            "version"
#define YAML_KEY_AXES               "axes"
#define YAML_KEY_SYSTEM             "system"

// ============================================================================
// AXIS NAMES (used as YAML section keys under "axes:")
// ============================================================================
#define YAML_AXIS_X                 "X"
#define YAML_AXIS_Y                 "Y"
#define YAML_AXIS_Z                 "Z"
#define YAML_AXIS_A                 "A"
#define YAML_AXIS_B                 "B"
#define YAML_AXIS_C                 "C"
#define YAML_AXIS_D                 "D"
#define YAML_AXIS_E                 "E"

// Axis name array for iteration
static const char* const YAML_AXIS_NAMES[LIMIT_NUM_AXES] = {
    YAML_AXIS_X, YAML_AXIS_Y, YAML_AXIS_Z, YAML_AXIS_A,
    YAML_AXIS_B, YAML_AXIS_C, YAML_AXIS_D, YAML_AXIS_E
};

// ============================================================================
// PER-AXIS PARAMETER KEYS
// ============================================================================
#define YAML_KEY_ALIAS              "alias"
#define YAML_KEY_UNITS_PER_PULSE    "units_per_pulse"
#define YAML_KEY_LIMITS             "limits"
#define YAML_KEY_VELOCITY           "velocity"
#define YAML_KEY_ACCELERATION       "acceleration"
#define YAML_KEY_BACKLASH           "backlash"
#define YAML_KEY_HOME_OFFSET        "home_offset"

// ============================================================================
// SYSTEM PARAMETER KEYS
// ============================================================================
#define YAML_KEY_DEBOUNCE_MS        "debounce_ms"
#define YAML_KEY_IDLE_TIMEOUT_S     "idle_timeout_s"
#define YAML_KEY_LOG_LEVEL          "log_level"

// ============================================================================
// LOG LEVEL VALUES (must match ESP-IDF esp_log_level_t)
// ============================================================================
#define YAML_VAL_LOG_NONE           "NONE"
#define YAML_VAL_LOG_ERROR          "ERROR"
#define YAML_VAL_LOG_WARN           "WARN"
#define YAML_VAL_LOG_INFO           "INFO"
#define YAML_VAL_LOG_DEBUG          "DEBUG"

// ============================================================================
// NVS KEY MAPPINGS
// Maps YAML structure to flat NVS key-value storage
// ============================================================================
#define NVS_NAMESPACE               "yarobot"
#define NVS_KEY_PREFIX_AXIS         "ax"       // ax0_, ax1_, etc.
#define NVS_KEY_SUFFIX_UNITS        "_units"
#define NVS_KEY_SUFFIX_MIN          "_min"
#define NVS_KEY_SUFFIX_MAX          "_max"
#define NVS_KEY_SUFFIX_VEL          "_vel"
#define NVS_KEY_SUFFIX_ACC          "_acc"
#define NVS_KEY_SUFFIX_BACKLASH     "_blash"
#define NVS_KEY_SUFFIX_ALIAS        "_alias"
#define NVS_KEY_SUFFIX_HOME_OFF     "_home_off"

// System NVS keys
#define NVS_KEY_SYS_VERSION         "sys_version"
#define NVS_KEY_SYS_SLOT            "sys_slot"
#define NVS_KEY_SYS_DEBOUNCE        "sys_debounce"

// ============================================================================
// VALIDATION MACROS
// Compute limits from compile-time constants for runtime validation
// ============================================================================

// Maximum velocity in user units, given units_per_pulse
// velocity_units_per_sec = pulse_freq_hz * units_per_pulse
#define YAML_MAX_VELOCITY(units_per_pulse) \
    ((float)LIMIT_MAX_PULSE_FREQ_HZ * (units_per_pulse))

// Minimum velocity (must generate at least 1 pulse/sec)
#define YAML_MIN_VELOCITY(units_per_pulse) \
    ((float)LIMIT_MIN_PULSE_FREQ_HZ * (units_per_pulse))

// Validate velocity against computed limits
#define YAML_VALIDATE_VELOCITY(vel, units_per_pulse) \
    ((vel) >= YAML_MIN_VELOCITY(units_per_pulse) && \
     (vel) <= YAML_MAX_VELOCITY(units_per_pulse))

// Validate alias length
#define YAML_VALIDATE_ALIAS_LEN(alias) \
    (strlen(alias) <= LIMIT_ALIAS_MAX_LENGTH)

// Validate axis index
#define YAML_VALIDATE_AXIS(idx) \
    ((idx) >= 0 && (idx) < LIMIT_NUM_AXES)

// Validate position limits (min < max)
#define YAML_VALIDATE_LIMITS(min, max) \
    ((min) < (max))

// ============================================================================
// HELPER: Build NVS key string
// Usage: char key[16]; NVS_AXIS_KEY(key, 0, NVS_KEY_SUFFIX_UNITS); // "ax0_units"
// ============================================================================
#define NVS_AXIS_KEY(buf, axis_idx, suffix) \
    snprintf((buf), sizeof(buf), "%s%d%s", NVS_KEY_PREFIX_AXIS, (axis_idx), (suffix))

#endif // CONFIG_YAML_SCHEMA_H
```

### config.h — Master Include

```c
#ifndef CONFIG_H
#define CONFIG_H

// Master configuration header - includes all configuration files

#include "config_gpio.h"
#include "config_peripherals.h"
#include "config_timing.h"
#include "config_limits.h"
#include "config_commands.h"
#include "config_i2c.h"
#include "config_sr.h"              // Shift register bit positions
#include "config_defaults.h"
#include "config_oled.h"
#include "config_yaml_schema.h"

// ============================================================================
// VERSION INFORMATION
// ============================================================================
#define FIRMWARE_NAME           "YAROBOT_CONTROL_UNIT"
#define FIRMWARE_VERSION_MAJOR  1
#define FIRMWARE_VERSION_MINOR  0
#define FIRMWARE_VERSION_PATCH  0
#define FIRMWARE_VERSION_STRING "1.0.0"

// ============================================================================
// FEATURE FLAGS
// ============================================================================
#define FEATURE_OLED_DISPLAY    1           // MVP: OLED display enabled
#define FEATURE_Z_SIGNAL        1           // Set to 1 to enable Z-signal homing
#define FEATURE_STREAMING       1           // Set to 1 to enable position streaming

#endif // CONFIG_H
```

**Usage in Code:**

```cpp
// Example: Using configuration in motor driver
#include "config.h"

void initMotorX() {
    // GPIO comes from config
    gpio_set_direction(GPIO_X_STEP, GPIO_MODE_OUTPUT);

    // Timing comes from config
    esp_rom_delay_us(TIMING_DIR_SETUP_US);

    // Limits from config
    if (queue_depth > LIMIT_MOTION_QUEUE_DEPTH) {
        return ESP_ERR_NO_MEM;
    }
}

// Example: Using command strings
if (strcmp(cmd, CMD_MOVE) == 0) {
    // Handle MOVE command
} else if (strcmp(cmd, CMD_STOP) == 0) {
    // Handle STOP command
}

// Example: Error response
sendResponse(RESP_ERROR " " ERR_AXIS_NOT_ENABLED " " MSG_AXIS_NOT_ENABLED);
// Output: "ERROR E004 Axis not enabled"

// Example: Using YAML schema keys in parser
#include "config_yaml_schema.h"

esp_err_t parse_axis_config(yaml_node_t* axis_node, uint8_t axis_idx) {
    float velocity = yaml_get_float(axis_node, YAML_KEY_VELOCITY);
    float units = yaml_get_float(axis_node, YAML_KEY_UNITS_PER_PULSE);

    // Validate against compile-time limits
    if (!YAML_VALIDATE_VELOCITY(velocity, units)) {
        ESP_LOGE(TAG, "Velocity %.2f exceeds max %.2f for axis %d",
                 velocity, YAML_MAX_VELOCITY(units), axis_idx);
        return ESP_ERR_INVALID_ARG;
    }

    // Build NVS key and store
    char nvs_key[16];
    NVS_AXIS_KEY(nvs_key, axis_idx, NVS_KEY_SUFFIX_VEL);
    nvs_set_float(nvs_handle, nvs_key, velocity);

    return ESP_OK;
}
```

## Data Architecture

### NVS (Non-Volatile Storage) Organization

ESP32-S3 NVS is used for persistent configuration storage.

**Namespace:** `yarobot`

```
Per-Axis Keys (X=0 to E=7):
├── ax{n}_units      (float)   Units per pulse (e.g., 0.001 mm/pulse)
├── ax{n}_min        (float)   Position limit minimum
├── ax{n}_max        (float)   Position limit maximum
├── ax{n}_vel        (float)   Max velocity (units/sec)
├── ax{n}_acc        (float)   Max acceleration (units/sec²)
├── ax{n}_blash      (float)   Backlash compensation
├── ax{n}_alias      (str:16)  Human-readable alias
└── ax{n}_home_off   (float)   Home position offset

System Keys:
├── sys_version      (u32)     Configuration version number
├── sys_slot         (u8)      Active configuration slot (0-9)
└── sys_debounce     (u16)     Input debounce time (ms)

Configuration Slots (0-9):
└── slot{n}_blob     (blob)    Serialized configuration snapshot
```

### YAML Configuration Schema

Configuration transferred via serial using CFGSTART/CFGDATA/CFGEND commands.

```yaml
# yarobot_config.yaml
version: 1

axes:
  X:
    alias: "RAILWAY"
    units_per_pulse: 0.001    # mm per pulse
    limits: [-500.0, 500.0]   # [min, max] in units
    velocity: 200.0           # max velocity (units/sec)
    acceleration: 1000.0      # max accel (units/sec²)
    backlash: 0.05            # backlash compensation
    home_offset: 0.0          # offset after homing

  Y:
    alias: "GRIPPER"
    units_per_pulse: 0.01     # degrees per pulse
    limits: [0.0, 360.0]
    velocity: 720.0
    acceleration: 3600.0
    backlash: 0.0
    home_offset: 0.0

  # ... Z, A, B, C, D, E axes follow same schema

system:
  debounce_ms: 10             # input debounce time
  idle_timeout_s: 300         # motor idle timeout (0 = disabled)
  log_level: "INFO"           # NONE, ERROR, WARN, INFO, DEBUG
```

### Data Flow

```
Host (Python/C++)
     │
     ├─ CFGSTART ──────────► Firmware enters CONFIG mode
     ├─ CFGDATA line1 ─────► Buffer accumulates YAML
     ├─ CFGDATA line2 ─────►
     ├─ ...
     ├─ CFGEND APPLY ──────► Parse YAML, validate, apply to runtime
     │                       └─► SAVE ──► Persist to NVS
     │
     └─ CFGEXPORT ─────────► Firmware outputs current config as YAML
```

## Safety Architecture

### System State Machine

```
                    ┌─────────────────┐
                    │   POWER_ON      │
                    └────────┬────────┘
                             │ Init complete
                    ┌────────v────────┐
                    │     IDLE        │◄──────────────────┐
                    │ (Motors enabled │                   │
                    │  Brakes engaged)│                   │
                    └────────┬────────┘                   │
                             │ First command              │ RST command
                    ┌────────v────────┐                   │ (E-stop released)
              ┌─────┤    READY        ├───────┐           │
              │     │ (Brakes can be  │       │           │
              │     │  released)      │       │           │
              │     └────────┬────────┘       │           │
              │              │ Motion cmd     │           │
              │     ┌────────v────────┐       │           │
              │     │    MOVING       │       │           │
              │     │ (Active motion) │       │           │
              │     └────────┬────────┘       │           │
              │              │ Limit/Fault    │           │
              │     ┌────────v────────┐       │           │
              │     │     ERROR       │───────┤           │
              │     │ (Axis stopped)  │       │           │
              │     └─────────────────┘       │           │
              │                               │           │
              │     ┌─────────────────┐       │           │
              └────►│  ESTOP_ACTIVE   │◄──────┘           │
                    │ (All stopped,   ├───────────────────┘
                    │  Brakes engaged)│
                    └─────────────────┘
                          │
                    E-stop held │
                          v
                    ┌─────────────────┐
                    │ CRITICAL_FAULT  │
                    │ (Requires power │
                    │  cycle)         │
                    └─────────────────┘
```

### E-Stop Handling (Highest Priority)

E-Stop uses direct GPIO interrupt for fastest response:

```c
// ISR context - executes in <5µs
static void IRAM_ATTR emergency_stop_isr(void* arg) {
    // 1. Stop all pulse generation immediately via direct register access
    RMT.conf_ch[RMT_CHANNEL_X].conf1.tx_start = 0;
    RMT.conf_ch[RMT_CHANNEL_Z].conf1.tx_start = 0;
    RMT.conf_ch[RMT_CHANNEL_A].conf1.tx_start = 0;
    RMT.conf_ch[RMT_CHANNEL_B].conf1.tx_start = 0;
    MCPWM0.timer[MCPWM_TIMER_Y].mode.start = 0;
    MCPWM0.timer[MCPWM_TIMER_C].mode.start = 0;

    // 2. Force STEP outputs low
    GPIO.out_w1tc = STEP_OUTPUT_MASK;

    // 3. Notify safety task (deferred brake handling)
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(safety_task_handle, NOTIFY_ESTOP, eSetBits, &woken);
    portYIELD_FROM_ISR(woken);
}
```

### Brake Control Strategies

Per-axis configurable brake behavior:

| Strategy | When Engaged | Use Case |
|----------|--------------|----------|
| `BRAKE_ON_DISABLE` | Motor disabled | Default - normal operation |
| `BRAKE_ON_ESTOP` | E-stop only | High-speed horizontal axes |
| `BRAKE_ON_IDLE` | After idle timeout | Vertical axes, power saving |
| `BRAKE_NEVER` | Manual only | Testing/maintenance |

```c
// Brake control via shift register (active-low, fail-safe)
// Bit 0 = engaged (spring applied), Bit 1 = released (power applied)
typedef struct {
    BrakeStrategy strategy;
    uint32_t idle_threshold_ms;  // For BRAKE_ON_IDLE
    bool is_engaged;
    uint32_t idle_timer;
} brake_state_t;

brake_state_t brake_states[LIMIT_NUM_SERVOS];  // X,Y,Z,A,B only (servos)
```

### Limit Switch Handling

14 limit switches on MCP23017 expanders with interrupt-driven detection:

```c
// Limit switch response (task context, <5ms latency)
void process_limit_switch(uint8_t axis, bool min_triggered, bool max_triggered) {
    float velocity = get_current_velocity(axis);

    // Only stop if moving toward triggered limit
    if ((min_triggered && velocity < 0) || (max_triggered && velocity > 0)) {
        stop_axis_immediate(axis);
        set_axis_error(axis, ERR_LIMIT_SWITCH);

        // Still allow motion away from limit
        axis_state[axis].can_move_positive = !max_triggered;
        axis_state[axis].can_move_negative = !min_triggered;
    }
}
```

### Error Categories

| Category | Examples | Response | Recovery |
|----------|----------|----------|----------|
| **Critical** | E-stop, I2C lost, runaway | Stop all, engage brakes | Power cycle or RST |
| **Axis** | Limit hit, driver fault | Stop axis only | CLR command |
| **Warning** | USB timeout, retry success | Log, continue | Auto-clear |

### Safety Task (Core 0, Priority 24 - Highest)

```c
void safety_monitor_task(void* param) {
    while (1) {
        uint32_t notify;

        // Wait for safety events or periodic check
        if (xTaskNotifyWait(0, ULONG_MAX, &notify, pdMS_TO_TICKS(TIMING_SAFETY_POLL_MS))) {
            if (notify & NOTIFY_ESTOP) handle_estop();
            if (notify & NOTIFY_LIMIT)  process_limit_switches();
        }

        // Periodic checks at TIMING_SAFETY_POLL_MS interval
        monitor_estop_gpio();
        check_driver_faults();
        update_brake_timers();
        check_motion_timeouts();
    }
}
```

### Power Loss Protection

Fail-safe design ensures brakes engage on power loss:

1. **Shift register outputs go LOW** on power loss (TPIC6B595N)
2. **Brake logic is active-low** (0 = engaged, 1 = released)
3. **Spring-applied brakes** mechanically engage without power
4. **RTC memory stores positions** before brownout (if detected)

```c
// Brownout detection (if available)
void power_monitor_isr(void) {
    // Quick save to RTC memory
    for (int i = 0; i < LIMIT_NUM_AXES; i++) {
        rtc_position[i] = current_position[i];
    }
    rtc_power_loss_flag = true;
}
```

## API Contracts

### Command Protocol

All commands are newline-terminated ASCII text:

```
<VERB> [<AXIS>] [<PARAM1>] [<PARAM2>] ...
```

**Response Format:**
```
OK [<data>]                    # Success
ERROR <code> <message>         # Error
EVENT <type> <axis> [<data>]   # Async notification
```

**Command Reference:** See `docs/planning_phase/command-interface.md` for complete specification.

### Internal APIs

**Motor Interface:**
```cpp
esp_err_t motor_move_absolute(uint8_t axis, float position, float velocity);
esp_err_t motor_move_relative(uint8_t axis, float delta, float velocity);
esp_err_t motor_stop(uint8_t axis);
esp_err_t motor_enable(uint8_t axis, bool enable);
float motor_get_position(uint8_t axis);
bool motor_is_moving(uint8_t axis);
```

**Shift Register Interface:**
```cpp
void sr_set_direction(uint8_t axis, bool forward);
void sr_set_enable(uint8_t axis, bool enable);
void sr_engage_brake(uint8_t axis);
void sr_release_brake(uint8_t axis);
void sr_emergency_disable_all(void);  // ISR-safe
```

## Security Architecture

**Scope:** Internal tool, no network connectivity, physical access required.

**Measures:**
- No authentication (USB physical access = full control)
- Input validation prevents buffer overflows and invalid parameters
- NaN/Infinity checks on all float inputs
- Axis bounds checking before array access
- No code execution from USB - command parsing only

**Not Applicable:**
- Network security (no network)
- Encryption (local USB only)
- User roles (single operator)

## Performance Considerations

### Critical Timing Requirements

| Requirement | Target | Strategy |
|-------------|--------|----------|
| E-stop response | <5µs | Direct ISR register access |
| Limit switch response | <5ms | I2C interrupt + polling |
| Command latency | <10ms | Queue-based processing |
| Pulse accuracy | ±1% | Hardware peripherals (RMT/MCPWM) |
| Motion update rate | 1000Hz | FreeRTOS tick at 1ms |

### Memory Budget

| Region | Size | Usage |
|--------|------|-------|
| Flash | 16MB | Firmware (~1MB), NVS (64KB), OTA partition |
| PSRAM | 8MB | Available for buffers, future expansion |
| SRAM | 512KB | Task stacks, queues, runtime data |

### Task Stack Allocation

See `config_limits.h` for stack sizes. Total ~32KB for all tasks.

## Deployment Architecture

### Flash Partition Table

```
# Name,   Type, SubType, Offset,   Size,  Flags
nvs,      data, nvs,     0x9000,   0x6000
phy_init, data, phy,     0xf000,   0x1000
factory,  app,  factory, 0x10000,  0x300000
```

### Firmware Update Process

```bash
# Build
idf.py build

# Flash via USB
idf.py -p /dev/ttyACM0 flash

# Monitor
idf.py -p /dev/ttyACM0 monitor
```

### Configuration Deployment

```bash
# Export current config
echo "CFGEXPORT" > /dev/ttyACM0 && cat /dev/ttyACM0 > config_backup.yaml

# Deploy new config
echo "CFGSTART" > /dev/ttyACM0
cat new_config.yaml | while read line; do echo "CFGDATA $line" > /dev/ttyACM0; done
echo "CFGEND APPLY" > /dev/ttyACM0
echo "SAVE 0" > /dev/ttyACM0
```

## Development Environment

### Prerequisites

- ESP-IDF v5.4
- Python 3.8+ (for idf.py, esptool)
- CMake 3.16+
- USB-C cable for flashing and debugging

### Setup Commands

```bash
# Install ESP-IDF (if not already)
mkdir -p ~/esp
cd ~/esp
git clone -b v5.4 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
source export.sh

# Clone project
cd ~/projects
git clone <repository_url> yarobot_control_unit
cd yarobot_control_unit

# Configure
idf.py set-target esp32s3
idf.py menuconfig  # Verify Flash size, PSRAM, USB CDC settings

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/ttyACM0 flash monitor
```

### Debugging

```bash
# OpenOCD for JTAG debugging
idf.py openocd

# GDB attachment
idf.py gdb

# Core dump analysis (if enabled)
idf.py coredump-info
```

## Architecture Decision Records (ADRs)

### ADR-001: ESP32-S3 Platform Selection
**Context:** Need MCU with multiple PWM channels, USB CDC, sufficient GPIO.
**Decision:** ESP32-S3-DevKitC-1 N16R8 (16MB Flash, 8MB PSRAM).
**Rationale:** Native USB (no FTDI), 4 RMT channels, 2 MCPWM groups, ample GPIO, FreeRTOS SMP.

### ADR-002: Layered Component Architecture
**Context:** Need modular design for testability and maintainability.
**Decision:** HAL → Drivers → Control → Interface layering.
**Rationale:** Clear dependencies, unit testable, follows ESP-IDF conventions.

### ADR-003: TPIC6B595N for Shift Registers
**Context:** Need to control DIR/EN/Brake signals at 24V logic levels.
**Decision:** 3x TPIC6B595N (24-bit chain) instead of 74HC595.
**Rationale:** Native 24V open-drain outputs, no level shifters, fail-safe brakes.

### ADR-004: Separate I2C Bus for OLED
**Context:** OLED display sharing I2C with critical limit switches is risky.
**Decision:** OLED on I2C_NUM_1 (GPIO_OLED_SDA/GPIO_OLED_SCL), MCP23017s on I2C_NUM_0 (GPIO_I2C_SDA/GPIO_I2C_SCL).
**Rationale:** Isolation prevents display issues from affecting safety-critical I/O.

### ADR-005: Text Command Protocol
**Context:** Need simple, debuggable host interface.
**Decision:** Human-readable text commands over USB CDC.
**Rationale:** Works with any serial terminal, easy debugging, no special tooling needed.

---

_Generated by BMAD Decision Architecture Workflow v1.0_
_Date: 2025-11-29_
_For: Sergey_
