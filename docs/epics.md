# yarobot_control_unit - Epic Breakdown

**Author:** Sergey
**Date:** 2025-11-29
**Project Level:** Medium Complexity
**Target Scale:** Internal Engineering Tool

---

## Overview

This document provides the complete epic and story breakdown for yarobot_control_unit, decomposing the requirements from the [PRD](./prd.md) into implementable stories.

**Living Document Notice:** This is the initial version created from PRD and Architecture documents.

### Epic Summary

| Epic | Title | Stories | FRs Covered |
|------|-------|---------|-------------|
| 1 | Foundation & Infrastructure | 7 | Enables ALL (infrastructure) |
| 2 | Communication & Command Interface | 7 | FR19-26, FR51-55 |
| 3 | Motor Control Core | 11 | FR1-10, FR43-44, FR48 |
| 4 | Safety & I/O Systems | 14 | FR11-18, FR33, FR35-42, FR45, FR49, FR56-64 |
| 5 | Configuration & Status Display | 12 | FR27-32, FR34, FR46-47c, FR50 |
| 6 | Advanced Position Feedback | 6 | FR65-70 (Z-signal, InPos, position sync) |

**Total: 57 Stories covering 70 Functional Requirements**

---

## Functional Requirements Inventory

**Motor Control Capabilities (FR1-FR10):**
- FR1: System can control up to 8 independent motor axes simultaneously (X,Y,Z,A,B,C,D,E)
- FR2: System can generate STEP pulses up to 80-100 kHz for high-speed motion
- FR3: System can control 5 servo motors through STEP/DIR interfaces with position feedback
- FR4: System can control 2 stepper motors with hardware position counting
- FR5: System can control 1 discrete actuator (E axis) with time-based position calculation
- FR6: Each axis can execute absolute position moves (MOVE command)
- FR7: Each axis can execute relative position moves (MOVR command)
- FR8: Each axis can execute continuous jog movements
- FR9: System can stop any axis motion immediately on command
- FR10: System can enable/disable individual axes independently

**Safety and Limit Management (FR11-FR18):**
- FR11: System can monitor 14 limit switches (2 per axis) in real-time
- FR12: System can execute hardware E-stop to disable all motors immediately
- FR13: System can automatically stop axis motion when limit switches activate
- FR14: System can control motor brakes with configurable strategies
- FR15: System can engage brakes automatically on power loss (fail-safe)
- FR16: System can detect and report position loss conditions
- FR17: System can prevent motion commands that would exceed configured limits
- FR18: Users can configure limit switch polarity and behavior

**Communication and Commands (FR19-FR26):**
- FR19: System accepts commands via USB CDC serial interface
- FR20: System responds to commands within 10ms
- FR21: System accepts human-readable text commands (MOVE, MOVR, VEL, STOP, etc.)
- FR22: System accepts status query commands (POS, STAT, INFO, DIAG)
- FR23: System provides command acknowledgment and error responses
- FR24: System generates asynchronous event notifications
- FR25: System reports comprehensive status for all axes on demand
- FR26: System maintains command history for debugging

**Configuration and Calibration (FR27-FR34):**
- FR27: System can load configuration from YAML files via USB
- FR28: System can export current configuration to YAML format
- FR29: Users can adjust motion parameters at runtime (speed, acceleration, limits)
- FR30: System persists calibration data across power cycles
- FR31: Users can clear axis positions without physical movement
- FR32: Users can set custom scaling ratios per axis
- FR33: System can execute homing sequences for each axis
- FR34: Users can name axes with custom aliases for clarity

**I/O and Peripheral Control (FR35-FR42):**
- FR35: System can read 4 general-purpose digital inputs (hardware design constraint)
- FR36: System can control 8 general-purpose digital outputs
- FR37: System can process servo feedback signals (InPos, alarms)
- FR38: System can detect floating switch activation on C axis (picker jaw)
- FR39: System can measure object width using C axis floating switch and report via event
- FR40: System can query last measured object width from C axis
- FR41: System provides configurable input debouncing
- FR42: Users can read/write I/O using pin names or aliases

**Status and Monitoring (FR43-FR50):**
- FR43: System reports real-time position for all axes
- FR44: System reports motion status (idle, moving, error) per axis
- FR45: System tracks and reports cumulative error counts
- FR46: System monitors I2C communication health
- FR47: System displays axis positions and movement status on OLED screen
- FR47a: System displays E-stop status with highest priority on OLED
- FR47b: System displays errors and events on OLED for 2 seconds
- FR47c: OLED operates on dedicated I2C bus isolated from limit switch I/O
- FR48: System generates events for motion completion
- FR49: System generates events for errors and faults
- FR50: Users can query individual axis status or all axes

**Operational Modes (FR51-FR55):**
- FR51: System supports normal operation mode for motion control
- FR52: System supports configuration mode for YAML updates
- FR53: System can switch between operational modes on command
- FR54: System indicates current mode via status responses
- FR55: System prevents motion commands in configuration mode

**Error Handling (FR56-FR61):**
- FR56: System detects and reports I2C communication failures
- FR57: System provides detailed error messages with context
- FR58: System can recover from transient communication errors
- FR59: System logs errors for troubleshooting
- FR60: Users can clear error conditions after resolution
- FR61: System prevents unsafe operations when in error state

**Driver Alarm Management (FR62-FR64):**
- FR62: System can monitor driver alarm signals (ALARM_INPUT) for axes X-D in real-time
- FR63: System can attempt to clear driver alarms via ALARM_CLEAR outputs (CLR command)
- FR64: System prevents motion commands on axes with active driver alarms

**Advanced Position Feedback (FR65-FR70):**
- FR65: System can read Z-signal (encoder index pulse) from servo axes (X, Y, Z, A, B)
- FR66: System can detect position drift via Z-signal comparison during motion
- FR67: System can apply deferred Z-signal corrections when axis becomes IDLE
- FR68: System provides configurable Z-signal fallback during homing (auto/confirm/fail)
- FR69: System can report Z-signal synchronization events and drift alarms
- FR70: System can use InPos signals to confirm motion completion and detect following errors

---

## FR Coverage Map

| Epic | FRs Covered |
|------|-------------|
| **Epic 1: Foundation** | Enables ALL (infrastructure) |
| **Epic 2: Communication** | FR19-26, FR51-55 (13 FRs) |
| **Epic 3: Motor Control** | FR1-10, FR43-44, FR48 (13 FRs) |
| **Epic 4: Safety & I/O** | FR11-18, FR33, FR35-42, FR45, FR49, FR56-64 (28 FRs) |
| **Epic 5: Config & Display** | FR27-32, FR34, FR46-47c, FR50 (13 FRs) |
| **Epic 6: Position Feedback** | FR65-70 (6 FRs) |

---

## Epic 1: Foundation & Infrastructure

**Goal:** Establish project structure, build system, and core abstractions that enable all subsequent development.

**User Value:** After this epic, developers can build, flash, and run a basic firmware that boots correctly on the ESP32-S3, with all configuration headers and component structure in place. Hardware peripherals are verified working.

**FR Coverage:** Infrastructure enabling all FRs (no direct FR implementation)

---

### Story 1.1: ESP-IDF Project Initialization

**As a** developer,
**I want** an ESP-IDF project properly configured for ESP32-S3 N16R8,
**So that** I can build and flash firmware to the target hardware.

**Acceptance Criteria:**

**Given** a fresh clone of the repository
**When** I run `cd firmware && idf.py build`
**Then** the project compiles without errors

**And** the target is set to esp32s3
**And** sdkconfig contains:
- `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y`
- `CONFIG_SPIRAM_MODE_OCT=y`
- `CONFIG_FREERTOS_HZ=1000`
- `CONFIG_ESP_CONSOLE_USB_CDC=y`
- `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`

**And** `idf.py flash` successfully programs the device
**And** `idf.py monitor` shows boot messages

**Prerequisites:** None (first story)

**Technical Notes:**
- Use `idf.py create-project` or manual setup
- Create `sdkconfig.defaults` for reproducible builds
- Create `partitions.csv` per architecture spec
- Add `idf_component.yml` for managed dependencies (espressif/mcp23017)

---

### Story 1.2: Component Directory Structure

**As a** developer,
**I want** the component folder structure established per architecture spec,
**So that** all subsequent code has a defined home and dependencies are clear.

**Acceptance Criteria:**

**Given** the project is initialized
**When** I examine the `firmware/components/` directory
**Then** the following structure exists:
```
firmware/components/
├── hal/
│   ├── gpio_hal/
│   ├── i2c_hal/
│   └── spi_hal/
├── drivers/
│   ├── tpic6b595/
│   └── oled/
├── pulse_gen/
├── position/
├── motor/
├── control/
│   ├── motion_controller/
│   ├── safety_monitor/
│   └── command_executor/
├── interface/
│   ├── usb_cdc/
│   └── command_parser/
├── config/
│   ├── nvs_manager/
│   └── yaml_parser/
└── events/
    └── event_manager/
```

**And** each component has a `CMakeLists.txt` file
**And** each component has an `include/` directory for public headers
**And** the project builds successfully with empty components

**Prerequisites:** Story 1.1

**Technical Notes:**
- Create stub `CMakeLists.txt` with proper `idf_component_register()`
- Use `REQUIRES` and `PRIV_REQUIRES` for dependency declaration
- No implementation code yet - just structure

---

### Story 1.3: Configuration Header Framework

**As a** developer,
**I want** all configuration headers created with compile-time constants,
**So that** no magic numbers appear in code and hardware assignments are centralized.

**Acceptance Criteria:**

**Given** the component structure exists
**When** I include `config.h` in any source file
**Then** I have access to all configuration constants

**And** the following headers exist in `firmware/components/config/include/`:
- `config.h` (master include with FIRMWARE_VERSION, FEATURE_FLAGS)
- `config_gpio.h` (all GPIO assignments per architecture)
- `config_peripherals.h` (RMT, MCPWM, PCNT, LEDC, SPI assignments)
- `config_timing.h` (all timing constants in ms/µs)
- `config_limits.h` (buffer sizes, queue depths, axis counts, stack sizes)
- `config_commands.h` (CMD_* command strings, RESP_* response prefixes, ERR_* error codes)
- `config_i2c.h` (I2C addresses, MCP23017 pin mappings)
- `config_sr.h` (shift register bit positions, helper macros)
- `config_defaults.h` (default axis parameters)
- `config_oled.h` (OLED display configuration)
- `config_yaml_schema.h` (YAML keys, NVS mappings, validation macros)

**And** all values match the architecture document exactly
**And** `static_assert(LIMIT_NUM_SERVOS + LIMIT_NUM_STEPPERS + LIMIT_NUM_DISCRETE == LIMIT_NUM_AXES)` compiles

**Prerequisites:** Story 1.2

**Technical Notes:**
- Copy values directly from architecture document sections
- Use proper header guards (`#ifndef CONFIG_GPIO_H`)
- Include comments explaining each constant's purpose
- This is the "no magic numbers" mandate implementation
- All command verbs defined as CMD_MOVE, CMD_MOVR, CMD_VEL, CMD_STOP, etc.

---

### Story 1.4: HAL Layer Stubs

**As a** developer,
**I want** Hardware Abstraction Layer interfaces defined,
**So that** drivers can be implemented against stable abstractions and unit tested.

**Acceptance Criteria:**

**Given** configuration headers are complete
**When** I examine the HAL components
**Then** each HAL has a public header with function declarations:

**gpio_hal.h:**
```c
esp_err_t gpio_hal_init(void);
esp_err_t gpio_hal_set_direction(gpio_num_t pin, gpio_mode_t mode);
esp_err_t gpio_hal_set_level(gpio_num_t pin, uint32_t level);
int gpio_hal_get_level(gpio_num_t pin);
esp_err_t gpio_hal_set_interrupt(gpio_num_t pin, gpio_int_type_t type, gpio_isr_t handler, void* arg);
```

**i2c_hal.h:**
```c
esp_err_t i2c_hal_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq_hz);
esp_err_t i2c_hal_write(i2c_port_t port, uint8_t addr, const uint8_t* data, size_t len);
esp_err_t i2c_hal_read(i2c_port_t port, uint8_t addr, uint8_t* data, size_t len);
esp_err_t i2c_hal_write_read(i2c_port_t port, uint8_t addr, const uint8_t* wr, size_t wr_len, uint8_t* rd, size_t rd_len);
```

**spi_hal.h:**
```c
esp_err_t spi_hal_init(spi_host_device_t host, gpio_num_t mosi, gpio_num_t sclk, gpio_num_t cs);
esp_err_t spi_hal_transfer(spi_host_device_t host, const uint8_t* tx, uint8_t* rx, size_t len);
```

**And** stub implementations return `ESP_OK` or placeholder values
**And** the project compiles successfully

**Prerequisites:** Story 1.3

**Technical Notes:**
- These wrap ESP-IDF driver APIs
- Stubs enable compilation; real implementation comes in later epics
- HAL enables future mocking for unit tests

---

### Story 1.5: FreeRTOS Task Framework

**As a** developer,
**I want** the FreeRTOS task structure defined with proper core affinity and priorities,
**So that** real-time requirements are met from the start.

**Acceptance Criteria:**

**Given** HAL stubs are in place
**When** the firmware boots
**Then** the following tasks are created:

| Task | Core | Priority | Stack | Purpose |
|------|------|----------|-------|---------|
| safety_monitor | 0 | 24 | STACK_SAFETY_TASK | E-stop, limits, faults |
| usb_rx | 0 | 10 | STACK_USB_RX_TASK | USB receive |
| usb_tx | 0 | 10 | STACK_USB_TX_TASK | USB transmit |
| cmd_executor | 0 | 12 | STACK_CMD_EXECUTOR_TASK | Command processing |
| i2c_monitor | 0 | 8 | STACK_I2C_MONITOR_TASK | I2C health check |
| motion_X..E | 1 | 15 | STACK_MOTION_TASK | Per-axis motion (8 tasks) |
| display | 1 | 5 | STACK_DISPLAY_TASK | OLED update |
| idle_monitor | 0 | 4 | STACK_IDLE_MONITOR_TASK | Idle timeout |

**And** tasks are created with `xTaskCreatePinnedToCore()`
**And** stack sizes use constants from `config_limits.h`
**And** each task has a minimal loop that yields (`vTaskDelay`)
**And** `idf.py monitor` shows all tasks running (visible via logging)

**Prerequisites:** Story 1.4

**Technical Notes:**
- Core 0: Communication, safety, coordination
- Core 1: Motion control (time-critical)
- Use task notifications for inter-task communication
- Create queue handles as globals for later use

**Implementation Learnings (Story 1-5):**
- **Flash port:** Auto-detect works (`idf.py flash` without `-p`)
- **Monitor port:** `/dev/cu.usbmodem1201` (USB CDC)
- **Axis naming:** Use explicit array `{"X", "Y", "Z", "A", "B", "C", "D", "E"}` — arithmetic `'X' + axis` fails for axes A-E

---

### Story 1.6: Hardware Peripheral Verification

**As a** developer,
**I want** to verify all hardware peripherals are communicating correctly,
**So that** I can confidently build drivers knowing the hardware foundation works.

**Acceptance Criteria:**

**Given** the FreeRTOS task framework is running
**When** I execute hardware diagnostic tests
**Then** the following verifications pass:

**I2C Bus 0 (MCP23017 Expanders):**
- **Given** I2C0 is initialized at I2C_FREQ_HZ on GPIO_I2C_SDA/GPIO_I2C_SCL
- **When** I scan the bus
- **Then** devices respond at I2C_ADDR_MCP23017_0 (0x20), I2C_ADDR_MCP23017_1 (0x21)
- **And** read/write to MCP23017 registers succeeds without timeout

**I2C Bus 1 (OLED - Isolated):**
- **Given** I2C1 is initialized at I2C_OLED_FREQ_HZ on GPIO_OLED_SDA/GPIO_OLED_SCL
- **When** I scan the bus
- **Then** device responds at OLED_ADDRESS (0x3C)
- **And** OLED displays test pattern or "BOOT OK" message
- **And** I2C1 operations do NOT affect I2C0 (bus isolation verified)

**Shift Register Chain (SPI2):**
- **Given** SPI2 is initialized with GPIO_SR_MOSI, GPIO_SR_SCLK, GPIO_SR_CS, GPIO_SR_OE
- **When** I write test patterns (0xAAAAAA, 0x555555, 0x000000, 0xFFFFFF)
- **Then** patterns shift through all 24 bits correctly
- **And** GPIO_SR_OE controls output enable (outputs disabled when HIGH)
- **And** latch (GPIO_SR_CS) captures data on rising edge

**GPIO Direct Pins:**
- **Given** STEP output pins are configured (GPIO_X_STEP through GPIO_D_STEP)
- **When** I toggle each pin
- **Then** oscilloscope/logic analyzer confirms signal at correct GPIO
- **And** E-stop input (GPIO_E_STOP) reads correct logic level

**CMD_TEST Command (Basic):**
- **Given** USB CDC is functional (from task framework)
- **When** user sends `TEST I2C` command (CMD_TEST)
- **Then** system responds with I2C scan results
- **When** user sends `TEST SR` command
- **Then** system responds with shift register test results
- **When** user sends `TEST GPIO` command
- **Then** system responds with GPIO pin states

**Prerequisites:** Story 1.5

**Technical Notes:**
- Implement minimal CMD_TEST command handler in cmd_executor
- Use ESP_LOGI for detailed diagnostic output during tests
- Create `firmware/components/drivers/test_utils/` for reusable test functions
- This validates hardware before building full driver implementations
- If any test fails, stop and debug hardware/wiring before proceeding
- CMD_DIAG command can be implemented here as alias to CMD_TEST

---

### Story 1.7: Build Verification & Documentation

**As a** developer,
**I want** the build system verified with CI-ready scripts,
**So that** the foundation is solid before building on it.

**Acceptance Criteria:**

**Given** all previous stories are complete
**When** I run the build verification
**Then** the following all pass:

- `idf.py build` completes without warnings (treat warnings as errors)
- `idf.py size` shows memory usage within budget
- `idf.py flash` programs the device successfully
- Device boots and shows task creation in serial output
- All configuration constants are accessible from `main.cpp`

**And** README.md documents:
- Prerequisites (ESP-IDF version, Python, CMake)
- Build commands
- Flash commands
- Basic troubleshooting

**Prerequisites:** Story 1.6

**Technical Notes:**
- Add `-Werror` to CMakeLists.txt for strict compilation
- Verify PSRAM is detected on boot
- Verify USB CDC enumeration works
- This story validates Epic 1 is complete

---

## Epic 2: Communication & Command Interface

**Goal:** User can send text commands via USB and receive responses - the primary interaction point.

**User Value:** After this epic, users can connect via any serial terminal, send commands, and receive properly formatted responses. The system responds to basic queries and has the command dispatch architecture ready for motor control commands.

**FR Coverage:** FR19, FR20, FR21, FR22, FR23, FR24, FR25, FR26, FR51, FR52, FR53, FR54, FR55

---

### Story 2.1: USB CDC Serial Interface

**As a** user,
**I want** to connect to the controller via USB serial,
**So that** I can send commands and receive responses from any terminal program.

**Acceptance Criteria:**

**Given** the device is powered via USB-C
**When** I connect from a host computer
**Then** the device enumerates as a USB CDC ACM device

**And** I can open a serial connection (115200 baud default)
**And** characters I type are received by the firmware
**And** characters sent by firmware appear in my terminal
**And** connection works with: minicom, screen, PuTTY, Arduino Serial Monitor, Python pyserial

**Given** USB cable is disconnected during operation
**When** cable is reconnected
**Then** device re-enumerates without requiring power cycle
**And** serial communication resumes normally (NFR8)

**Prerequisites:** Epic 1 complete

**Technical Notes:**
- Use ESP-IDF TinyUSB component with CDC class
- Implement in `firmware/components/interface/usb_cdc/`
- Create FreeRTOS queues for RX and TX (LIMIT_COMMAND_QUEUE_DEPTH, LIMIT_RESPONSE_QUEUE_DEPTH)
- usb_rx_task reads from USB, pushes to rx_queue
- usb_tx_task reads from tx_queue, writes to USB
- Handle USB suspend/resume events gracefully
- Line-based input (commands terminated by \n or \r\n)

---

### Story 2.2: Command Parser

**As a** developer,
**I want** incoming text parsed into structured command objects,
**So that** command handlers receive validated, typed parameters.

**Acceptance Criteria:**

**Given** a text line is received from USB
**When** the parser processes it
**Then** a ParsedCommand structure is populated:

```c
typedef struct {
    char verb[16];           // Command verb (CMD_MOVE, CMD_STOP, etc.)
    char axis;               // Axis letter or '\0' if none
    float params[4];         // Numeric parameters
    uint8_t param_count;     // Number of parameters parsed
    char str_param[32];      // String parameter (for CMD_ALIAS, etc.)
    bool has_str_param;
} ParsedCommand;
```

**And** the following parsing rules apply:
- Command verb is case-insensitive (move = MOVE = Move)
- Axis letters are case-insensitive (x = X)
- Whitespace separates tokens
- Empty lines are ignored
- Lines starting with # are comments (ignored)

**And** parsing examples:
| Input | verb | axis | params | param_count |
|-------|------|------|--------|-------------|
| `MOVE X 100` | CMD_MOVE | 'X' | [100.0] | 1 |
| `MOVR Y -50.5 200` | CMD_MOVR | 'Y' | [-50.5, 200.0] | 2 |
| `STOP` | CMD_STOP | '\0' | [] | 0 |
| `STOP Z` | CMD_STOP | 'Z' | [] | 0 |
| `ALIAS X RAILWAY` | CMD_ALIAS | 'X' | [] | 0, str_param="RAILWAY" |
| `EN X 1` | CMD_EN | 'X' | [1.0] | 1 |

**And** invalid input returns parse error (not crash)

**Prerequisites:** Story 2.1

**Technical Notes:**
- Implement in `firmware/components/interface/command_parser/`
- Use `strtok_r` for thread-safe tokenization
- Use `strtof` for float parsing with error checking
- Validate axis against valid set (X,Y,Z,A,B,C,D,E or alias lookup)
- Buffer overflow protection (use LIMIT_CMD_MAX_LENGTH)
- Return ESP_ERR_INVALID_ARG for malformed commands
- Compare parsed verb against CMD_* constants from config_commands.h

---

### Story 2.3: Response Formatter

**As a** user,
**I want** consistent, parseable response formats,
**So that** I can programmatically process controller output.

**Acceptance Criteria:**

**Given** command processing completes
**When** a response is generated
**Then** it follows one of these formats:

**Success Response:**
```
OK [data]
```
Examples using RESP_OK from config_commands.h:
- `OK` (simple acknowledgment)
- `OK X 123.456` (position query)
- `OK READY` (mode query)

**Error Response:**
```
ERROR <code> <message>
```
Examples using RESP_ERROR, ERR_*, MSG_* from config_commands.h:
- `ERROR E001 Invalid command` (ERR_INVALID_COMMAND)
- `ERROR E002 Invalid axis` (ERR_INVALID_AXIS)
- `ERROR E004 Axis not enabled` (ERR_AXIS_NOT_ENABLED)

**Event Notification (async):**
```
EVENT <type> <axis> [data]
```
Examples using RESP_EVENT from config_commands.h:
- `EVENT DONE X 100.000` (motion complete)
- `EVENT LIMIT Y MIN` (limit switch triggered)
- `EVENT ESTOP ACTIVE` (emergency stop)

**And** all responses are terminated with `\r\n`
**And** response codes use constants from `config_commands.h`
**And** responses are sent within TIMING_CMD_RESPONSE_MS (10ms) for simple commands

**Prerequisites:** Story 2.1

**Technical Notes:**
- Implement in `firmware/components/interface/command_parser/` (or separate response_formatter)
- Use `snprintf` with LIMIT_RESPONSE_MAX_LENGTH buffer
- Thread-safe: multiple tasks may generate events
- Events go to same TX queue as responses
- All string literals use constants: RESP_OK, RESP_ERROR, RESP_EVENT, ERR_*, MSG_*

---

### Story 2.4: Command Dispatcher & Executor

**As a** developer,
**I want** commands routed to appropriate handlers based on verb,
**So that** command processing is modular and extensible.

**Acceptance Criteria:**

**Given** a ParsedCommand is ready
**When** the dispatcher processes it
**Then** it looks up the handler in the command table:

```c
typedef esp_err_t (*CommandHandler)(const ParsedCommand* cmd, char* response, size_t resp_len);

typedef struct {
    const char* verb;        // CMD_* constant from config_commands.h
    CommandHandler handler;
    uint32_t allowed_states; // Bitmask of valid system states
} CommandEntry;

static const CommandEntry command_table[] = {
    { CMD_ECHO,  handle_echo,  STATE_ANY },
    { CMD_INFO,  handle_info,  STATE_ANY },
    { CMD_STAT,  handle_stat,  STATE_ANY },
    { CMD_MODE,  handle_mode,  STATE_ANY },
    // ... more commands added in later epics
};
```

**And** state validation occurs before handler invocation:
- If command not allowed in current state → respond with RESP_ERROR, ERR_MODE_BLOCKED, MSG_MODE_BLOCKED

**And** unknown commands return RESP_ERROR ERR_INVALID_COMMAND MSG_INVALID_COMMAND

**And** cmd_executor_task:
1. Reads from command queue
2. Parses command
3. Validates state
4. Dispatches to handler
5. Sends response to TX queue

**Prerequisites:** Story 2.2, Story 2.3

**Technical Notes:**
- Implement in `firmware/components/control/command_executor/`
- Command table uses CMD_* constants, never literal strings
- Handler functions are stateless (receive all context as params)
- Use mutex if handlers access shared state
- Log commands at DEBUG level for troubleshooting (FR26: command history)

---

### Story 2.5: Basic Query Commands (CMD_INFO, CMD_STAT, CMD_ECHO)

**As a** user,
**I want** to query system information and test communication,
**So that** I can verify the controller is working and check its status.

**Acceptance Criteria:**

**CMD_ECHO Command:**
**Given** I send `ECHO hello world`
**When** command is processed
**Then** response is `OK hello world`

**Given** I send `ECHO` (no arguments)
**When** command is processed
**Then** response is `OK`

**CMD_INFO Command:**
**Given** I send `INFO`
**When** command is processed
**Then** response includes:
```
OK YAROBOT_CONTROL_UNIT 1.0.0
```
(FIRMWARE_NAME and FIRMWARE_VERSION_STRING from config.h)

**CMD_STAT Command (System Status):**
**Given** I send `STAT`
**When** command is processed
**Then** response includes:
```
OK MODE:READY ESTOP:0 AXES:8 UPTIME:12345
```
- MODE: current system mode (IDLE, READY, CONFIG, ESTOP)
- ESTOP: 0=inactive, 1=active
- AXES: LIMIT_NUM_AXES (8)
- UPTIME: milliseconds since boot

**Given** I send `STAT X`
**When** command is processed
**Then** response includes axis-specific status:
```
OK X POS:0.000 EN:0 MOV:0 ERR:0 LIM:00
```
- POS: current position
- EN: enabled (0/1)
- MOV: moving (0/1)
- ERR: error state (0/1)
- LIM: limit switches (bit0=min, bit1=max)

**Prerequisites:** Story 2.4

**Technical Notes:**
- CMD_ECHO is essential for communication testing
- CMD_INFO provides version for compatibility checking
- CMD_STAT provides comprehensive status (FR22, FR25, FR44, FR50)
- Axis status returns placeholder values until motor control implemented
- Use `esp_timer_get_time()` for uptime

---

### Story 2.6: Mode Management (CMD_MODE Command)

**As a** user,
**I want** to switch between operational modes,
**So that** I can safely configure the system or perform normal operations.

**Acceptance Criteria:**

**System Modes:**
| Mode | Description | Allowed Commands |
|------|-------------|------------------|
| IDLE | Initial state after boot | All except motion |
| READY | Normal operation | All commands |
| CONFIG | Configuration mode | Config commands only, no motion |
| ESTOP | Emergency stop active | Status queries only, CMD_RST to exit |
| ERROR | Axis error state | Status, CMD_RST to exit |

**CMD_MODE Query:**
**Given** I send `MODE`
**When** command is processed
**Then** response is `OK <current_mode>` (e.g., `OK READY`)

**CMD_MODE Set:**
**Given** system is in READY mode
**When** I send `MODE CONFIG`
**Then** response is `OK CONFIG`
**And** system enters CONFIG mode
**And** motion commands are blocked (FR55)
**And** CMD_CFGSTART/CMD_CFGDATA/CMD_CFGEND commands are allowed

**Given** system is in CONFIG mode
**When** I send `MODE READY`
**Then** response is `OK READY`
**And** system returns to READY mode

**Given** system is in ESTOP mode
**When** I send `MODE READY`
**Then** response is `ERROR E006 Emergency stop active` (ERR_EMERGENCY_STOP)
**And** mode does not change (must use CMD_RST after E-stop released)

**State Persistence:**
**And** current mode is tracked in global system state
**And** mode transitions generate EVENT notifications:
```
EVENT MODE READY
EVENT MODE CONFIG
```

**Prerequisites:** Story 2.5

**Technical Notes:**
- Implement state machine per architecture Safety Architecture section
- Mode stored in atomic variable for ISR-safe access
- E-stop ISR can force mode to ESTOP
- FR51-55 implemented here
- CMD_RST command (reset from ESTOP) implemented in Epic 4 with safety

---

### Story 2.7: Event System Foundation

**As a** developer,
**I want** an event publish/subscribe system,
**So that** subsystems can communicate asynchronously without tight coupling.

**Acceptance Criteria:**

**Given** the event manager is initialized
**When** a publisher calls `event_publish()`
**Then** all registered subscribers receive the event

**Event Structure:**
```c
typedef enum {
    EVT_MOTION_COMPLETE,
    EVT_MOTION_ERROR,
    EVT_LIMIT_TRIGGERED,
    EVT_ESTOP_CHANGED,
    EVT_MODE_CHANGED,
    EVT_ERROR,
    EVT_WIDTH_MEASURED,
    // ... extensible
} EventType;

typedef struct {
    EventType type;
    uint8_t axis;           // 0-7 or 0xFF for system-wide
    union {
        float position;
        float width;
        uint8_t error_code;
        uint8_t limit_state;
        bool estop_active;
    } data;
    int64_t timestamp;      // esp_timer_get_time()
} Event;
```

**API:**
```c
esp_err_t event_manager_init(void);
esp_err_t event_subscribe(EventType type, EventCallback callback, void* ctx);
esp_err_t event_unsubscribe(EventType type, EventCallback callback);
esp_err_t event_publish(const Event* event);  // Thread-safe, ISR-safe variant available
```

**USB Event Subscriber:**
**And** a default subscriber formats events for USB output using RESP_EVENT:
```
EVENT DONE X 100.000
EVENT LIMIT Y MIN
EVENT ESTOP ACTIVE
```

**And** event queue depth is LIMIT_EVENT_QUEUE_DEPTH (32)
**And** queue overflow generates error (ERR_EVENT_OVERFLOW) logged, oldest dropped

**Prerequisites:** Story 2.4

**Technical Notes:**
- Implement in `firmware/components/events/event_manager/`
- Use FreeRTOS queue for thread safety
- ISR-safe publish uses `xQueueSendFromISR`
- Callback registration uses linked list or static array
- This enables FR24 (async event notifications), FR48, FR49
- Events are foundation for motion completion, errors, limit switches

---

## Epic 3: Motor Control Core

**Goal:** User can move motors to positions using CMD_MOVE/CMD_MOVR/CMD_STOP commands - core product value.

**User Value:** After this epic, users can control all 8 motor axes through unified text commands. Servos, steppers, and discrete actuators all respond to the same MOVE/MOVR/STOP interface, abstracting hardware complexity.

**FR Coverage:** FR1, FR2, FR3, FR4, FR5, FR6, FR7, FR8, FR9, FR10, FR43, FR44, FR48

---

### Story 3.1: Shift Register Driver (DIR/EN Control)

**As a** developer,
**I want** the shift register chain operational,
**So that** I can control direction, enable, and brake signals for all axes.

**Acceptance Criteria:**

**Given** SPI2 is initialized per Story 1.6 hardware verification
**When** I call shift register API functions
**Then** the 40-bit chain (5x TPIC6B595N) is updated correctly

**API:**
```c
esp_err_t sr_init(void);
esp_err_t sr_set_direction(uint8_t axis, bool forward);  // SR_X_DIR..SR_E_DIR
esp_err_t sr_set_enable(uint8_t axis, bool enable);      // SR_X_EN..SR_E_EN
esp_err_t sr_set_brake(uint8_t axis, bool release);      // SR_X_BRAKE..SR_E_BRAKE (servos only)
esp_err_t sr_set_alarm_clear(uint8_t axis, bool active); // SR_X_ALARM_CLR..SR_D_ALARM_CLR
esp_err_t sr_set_gp_output(uint8_t pin, bool level);     // SR_GP_OUT_0..SR_GP_OUT_7 (SR4)
esp_err_t sr_update(void);                                // Latch current state to outputs
void sr_emergency_disable_all(void);                      // ISR-safe, all outputs LOW
uint64_t sr_get_state(void);                              // Read current 40-bit state
```

**Bit Position Verification:**
**Given** I call `sr_set_direction(0, true)` (X axis forward)
**When** I read the shift register state
**Then** bit SR_X_DIR (0) is set

**Given** I call `sr_set_enable(2, true)` (Z axis enable)
**When** I latch the data
**Then** bit SR_Z_EN (7) drives output HIGH (active)

**Fail-Safe Behavior:**
**Given** `sr_emergency_disable_all()` is called
**When** outputs are latched
**Then** all 40 bits are 0 (SR_SAFE_STATE)
**And** all brakes engage (active-low logic)
**And** all motors disabled

**And** GPIO_SR_OE (output enable) is LOW during normal operation
**And** GPIO_SR_OE goes HIGH during emergency (outputs tristated)

**Prerequisites:** Epic 1 complete, Epic 2 complete

**Technical Notes:**
- Implement in `firmware/components/drivers/tpic6b595/`
- Use SPI DMA for efficient 40-bit transfers (5 bytes)
- Maintain shadow register in RAM for read-back (uint64_t)
- Bit positions from `config_sr.h`: SR0-SR3 for motor control (4 bits/axis), SR4 for GP outputs
- Thread-safe with mutex (multiple axes may update simultaneously)
- ISR-safe emergency function uses direct register access

---

### Story 3.2: RMT Pulse Generator (X, Z, A, B Axes)

**As a** developer,
**I want** RMT-based pulse generation for servo axes,
**So that** I can generate precise STEP pulses up to 100kHz with DMA.

**Acceptance Criteria:**

**Given** RMT channels RMT_CHANNEL_X, RMT_CHANNEL_Z, RMT_CHANNEL_A, RMT_CHANNEL_B are configured
**When** I request pulse generation
**Then** STEP pulses are generated on GPIO_X_STEP, GPIO_Z_STEP, GPIO_A_STEP, GPIO_B_STEP

**IPulseGenerator Interface:**
```c
class IPulseGenerator {
public:
    virtual esp_err_t init() = 0;
    virtual esp_err_t start(uint32_t frequency_hz, uint32_t pulse_count) = 0;
    virtual esp_err_t stop() = 0;
    virtual esp_err_t setFrequency(uint32_t frequency_hz) = 0;
    virtual bool isRunning() const = 0;
    virtual uint32_t getPulseCount() const = 0;
    virtual ~IPulseGenerator() = default;
};
```

**RMT Implementation:**
**Given** I call `start(50000, 10000)` (50kHz, 10000 pulses)
**When** RMT generates pulses
**Then** exactly 10000 pulses are output at 50kHz (±1%)
**And** pulse width is 50% duty cycle (configurable)
**And** generation stops automatically after pulse_count reached
**And** callback fires on completion

**Frequency Range:**
**And** frequencies from LIMIT_MIN_PULSE_FREQ_HZ (1) to LIMIT_MAX_PULSE_FREQ_HZ (100000) are supported
**And** invalid frequencies return ESP_ERR_INVALID_ARG

**Direction Setup:**
**Given** direction change is needed before motion
**When** pulse generation starts
**Then** TIMING_DIR_SETUP_US (20µs) delay occurs after DIR signal change before first STEP pulse

**Prerequisites:** Story 3.1

**Technical Notes:**
- Implement in `firmware/components/pulse_gen/rmt_pulse_gen.cpp`
- Use RMT TX with DMA (esp_rmt_new_tx_channel)
- Use RMT encoder for pulse pattern generation
- Loop mode for continuous, finite mode for counted pulses
- End-of-transmission callback for completion event
- FR2: 80-100kHz pulse generation

---

### Story 3.3: MCPWM Pulse Generator with PCNT (Y, C Axes)

**As a** developer,
**I want** MCPWM-based pulse generation for Y and C axes with hardware pulse counting,
**So that** I can generate STEP pulses and track position via PCNT.

**Acceptance Criteria:**

**Given** MCPWM timers MCPWM_TIMER_Y and MCPWM_TIMER_C are configured
**When** I request pulse generation
**Then** STEP pulses are generated on GPIO_Y_STEP and GPIO_C_STEP

**MCPWM + PCNT Internal Routing:**
**Given** ESP-IDF v5.x `io_loop_back` flag is enabled
**When** MCPWM output and PCNT input are configured on same GPIO
**Then** internal GPIO matrix routes MCPWM output to PCNT input
**And** no external loopback wire is required
**And** GPIO is configured as GPIO_MODE_INPUT_OUTPUT

**MCPWM Implementation (implements IPulseGenerator):**
**Given** I call `start(25000, 5000)` on Y axis
**When** MCPWM generates pulses
**Then** pulses are output at 25kHz
**And** PCNT_UNIT_Y counts pulses internally for position tracking
**And** generation stops after 5000 pulses (via PCNT limit callback)

**PCNT Integration:**
**Given** MCPWM is generating pulses
**When** I query pulse count
**Then** PCNT_UNIT_Y or PCNT_UNIT_C returns accurate count
**And** count direction reflects forward/reverse motion

**C-Axis Stepper Specifics:**
**Given** C axis is a stepper (not servo)
**When** motion completes
**Then** position is calculated from pulse count (no external feedback)

**Prerequisites:** Story 3.1

**Technical Notes:**
- Implement in `firmware/components/pulse_gen/mcpwm_pulse_gen.cpp`
- ESP-IDF v5.x: Use `io_loop_back` in PCNT and MCPWM config for internal routing
- Reference: https://esp32.com/viewtopic.php?t=30817
- PCNT high/low limits trigger stop and callback
- Y axis: servo with PCNT backup counting
- C axis: stepper relying on PCNT for position

---

### Story 3.4: LEDC Pulse Generator (D Axis)

**As a** developer,
**I want** LEDC-based pulse generation for D axis stepper,
**So that** the discrete stepper can generate motion pulses.

**Acceptance Criteria:**

**Given** LEDC_TIMER and LEDC_CHANNEL_D are configured
**When** I request pulse generation
**Then** STEP pulses are generated on GPIO_D_STEP

**LEDC Implementation (implements IPulseGenerator):**
**Given** I call `start(10000, 2000)` on D axis
**When** LEDC generates pulses
**Then** pulses are output at 10kHz
**And** software counter tracks pulse count
**And** generation stops after 2000 pulses (via timer interrupt)

**Software Position Tracking:**
**Given** no hardware PCNT for D axis
**When** motion runs
**Then** software counter increments per pulse (timer-based)
**And** position accuracy is maintained within tolerance

**Prerequisites:** Story 3.1

**Technical Notes:**
- Implement in `firmware/components/pulse_gen/ledc_pulse_gen.cpp`
- LEDC provides frequency generation
- Use high-resolution timer for pulse counting
- Less precise than RMT/MCPWM but sufficient for D axis stepper
- Could alternatively use GPIO + timer for manual pulse generation

---

### Story 3.5: Position Tracker Interface

**As a** developer,
**I want** position tracking abstractions,
**So that** each motor type can track position appropriately.

**Acceptance Criteria:**

**IPositionTracker Interface:**
```c
class IPositionTracker {
public:
    virtual esp_err_t init() = 0;
    virtual esp_err_t reset(int64_t position = 0) = 0;
    virtual int64_t getPosition() const = 0;          // In pulses
    virtual void setDirection(bool forward) = 0;
    virtual ~IPositionTracker() = default;
};
```

**PCNT Tracker (Y, C axes):**
**Given** PCNT unit is configured
**When** pulses are generated
**Then** hardware counter tracks position
**And** `getPosition()` returns accurate pulse count
**And** overflow handling extends range beyond 16-bit PCNT limit

**Software Tracker (X, Z, A, B, D axes):**
**Given** RMT or LEDC generates pulses
**When** motion runs
**Then** software counter tracks commanded pulses
**And** position updated on motion completion callback

**Time-Based Tracker (E axis):**
**Given** E axis is discrete actuator
**When** motion command issued
**Then** position is calculated from elapsed time and known speed
**And** position is 0.0 (retracted) or 1.0 (extended)

**Prerequisites:** Story 3.2, Story 3.3, Story 3.4

**Technical Notes:**
- Implement in `firmware/components/position/`
- PCNT tracker wraps ESP-IDF PCNT driver
- Software tracker uses atomic counters updated from callbacks
- E axis: binary state, time-based position interpolation
- All positions stored as int64_t pulses, converted to user units via config

---

### Story 3.6: Motor Base Class & Servo Motor

**As a** developer,
**I want** the motor abstraction layer with servo motor implementation,
**So that** servo axes (X, Y, Z, A, B) can execute motion commands.

**Acceptance Criteria:**

**IMotor Interface:**
```c
class IMotor {
public:
    virtual esp_err_t init() = 0;
    virtual esp_err_t moveAbsolute(float position, float velocity) = 0;
    virtual esp_err_t moveRelative(float delta, float velocity) = 0;
    virtual esp_err_t stop() = 0;
    virtual esp_err_t stopImmediate() = 0;
    virtual float getPosition() const = 0;            // In user units
    virtual bool isMoving() const = 0;
    virtual bool isEnabled() const = 0;
    virtual esp_err_t enable(bool en) = 0;
    virtual AxisState getState() const = 0;
    virtual ~IMotor() = default;
};
```

**ServoMotor Implementation:**
**Given** a ServoMotor is constructed with:
- IPulseGenerator (RMT or MCPWM)
- IPositionTracker
- ShiftRegisterController reference
- Axis configuration (pulses_per_rev, units_per_rev, limits, velocity from config_defaults.h)

**When** I call `moveAbsolute(100.0, 50.0)`
**Then:**
1. Target position validated against limits (DEFAULT_LIMIT_MIN, DEFAULT_LIMIT_MAX or configured)
2. Direction calculated and set via shift register
3. Derived pulses_per_unit = pulses_per_rev / units_per_rev
4. Pulse count calculated: `(target - current) * pulses_per_unit`
5. Velocity converted to frequency: `velocity * pulses_per_unit`
6. Pulse generator started
7. State set to MOVING
8. On completion: EVT_MOTION_COMPLETE published

**Enable/Disable:**
**Given** I call `enable(true)`
**Then** shift register EN bit is set
**And** TIMING_ENABLE_DELAY_US elapses before motion allowed

**Given** I call `enable(false)`
**Then** any active motion stops
**And** shift register EN bit is cleared
**And** brake engages (per brake strategy - implemented in Epic 4)

**Prerequisites:** Story 3.1, Story 3.2, Story 3.3, Story 3.5

**Technical Notes:**
- Implement in `firmware/components/motor/`
- Composition: motor HAS-A pulse generator and position tracker
- Servo motors: X (RMT), Y (MCPWM), Z (RMT), A (RMT), B (RMT)
- Thread-safe state access
- FR3: 5 servo motors with STEP/DIR

---

### Story 3.7: Stepper Motor Implementation

**As a** developer,
**I want** stepper motor implementation for C and D axes,
**So that** stepper axes execute motion with hardware position counting.

**Acceptance Criteria:**

**StepperMotor Implementation:**
**Given** a StepperMotor is constructed with:
- IPulseGenerator (MCPWM for C, LEDC for D)
- IPositionTracker (PCNT for C, software for D)
- ShiftRegisterController reference
- Axis configuration

**When** I call `moveAbsolute(50.0, 25.0)` on C axis
**Then:**
1. Motion executes same as servo
2. Position tracked via PCNT_UNIT_C hardware counter
3. No external position feedback expected

**When** I call `moveAbsolute(500, 100)` on D axis
**Then:**
1. Motion executes with LEDC pulses
2. Position tracked via software counter
3. Completion based on commanded pulse count

**Stepper-Specific:**
**And** no brake control for steppers (holding torque sufficient)
**And** position maintained by pulse counting only
**And** power loss = position lost (requires homing)

**Prerequisites:** Story 3.6

**Technical Notes:**
- StepperMotor may inherit from common MotorBase with ServoMotor
- C axis: MCPWM + PCNT (most accurate)
- D axis: LEDC + software count (adequate for discrete positioning)
- FR4: 2 stepper motors with hardware position counting

---

### Story 3.8: Discrete Axis Implementation (E Axis)

**As a** developer,
**I want** discrete actuator implementation for E axis,
**So that** simple on/off actuators can be controlled through unified interface.

**Acceptance Criteria:**

**DiscreteAxis Implementation:**
**Given** E axis is configured as discrete actuator
**When** I call `moveAbsolute(1.0, 1.0)` (extend)
**Then:**
1. Direction set via shift register (SR_E_DIR)
2. Enable activated via shift register (SR_E_EN)
3. Time-based position calculation starts
4. After travel time, position = 1.0
5. EVT_MOTION_COMPLETE published

**When** I call `moveAbsolute(0.0, 1.0)` (retract)
**Then:**
1. Direction reversed
2. Position transitions from 1.0 to 0.0
3. Motion complete when position = 0.0

**Binary Positions:**
**And** E axis only supports positions E_AXIS_LIMIT_MIN (0.0) and E_AXIS_LIMIT_MAX (1.0)
**And** intermediate positions are interpolated during motion
**And** velocity is fixed (E_AXIS_MAX_VELOCITY, actuator speed not controllable)

**No Pulse Generation:**
**And** E axis does NOT generate STEP pulses
**And** Control is via DIR/EN shift register bits only (SR_E_DIR, SR_E_EN)
**And** Position based on elapsed time and known travel duration

**Prerequisites:** Story 3.6

**Technical Notes:**
- Implement in `firmware/components/motor/discrete_axis.cpp`
- E axis: solenoid, pneumatic cylinder, or similar
- Travel time configured in YAML (e.g., 500ms extend, 500ms retract)
- FR5: 1 discrete actuator with time-based position

---

### Story 3.9: Motion Controller & CMD_MOVE/CMD_MOVR Commands

**As a** user,
**I want** to control motors with CMD_MOVE and CMD_MOVR commands,
**So that** I can position axes through simple text commands.

**Acceptance Criteria:**

**CMD_MOVE Command (Absolute Position):**
**Given** X axis is enabled
**When** I send `MOVE X 100`
**Then** X axis moves to position 100.0 (user units)
**And** response is RESP_OK
**And** on completion: `EVENT DONE X 100.000`

**Given** I send `MOVE X 200 50`
**When** command is processed
**Then** X axis moves to 200.0 at velocity 50.0 units/sec

**CMD_MOVR Command (Relative Position):**
**Given** X axis is at position 100.0
**When** I send `MOVR X 25`
**Then** X axis moves to position 125.0
**And** response is RESP_OK
**And** on completion: `EVENT DONE X 125.000`

**Given** I send `MOVR Y -10 30`
**When** command is processed
**Then** Y axis moves -10 units from current position at 30 units/sec

**Multi-Axis:**
**Given** I send `MOVE X 100` then `MOVE Y 50`
**When** commands execute
**Then** both axes move simultaneously (independent)
**And** each generates its own completion event

**Error Cases:**
**Given** X axis is disabled
**When** I send `MOVE X 100`
**Then** response is RESP_ERROR ERR_AXIS_NOT_ENABLED MSG_AXIS_NOT_ENABLED

**Given** target exceeds limits
**When** I send `MOVE X 9999`
**Then** response is RESP_ERROR ERR_POSITION_LIMIT MSG_POSITION_LIMIT

**Prerequisites:** Story 3.6, Story 3.7, Story 3.8, Epic 2 complete

**Technical Notes:**
- Implement handlers in `firmware/components/control/command_executor/`
- MotionController holds array of IMotor* for all LIMIT_NUM_AXES (8) axes
- Default velocity from config if not specified in command (DEFAULT_MAX_VELOCITY)
- FR6, FR7: absolute and relative moves
- FR1: 8 independent axes

---

### Story 3.10: CMD_VEL, CMD_STOP, CMD_EN, CMD_POS Commands

**As a** user,
**I want** velocity jogging, stop, enable, and position query commands,
**So that** I have complete manual control over motor axes.

**Acceptance Criteria:**

**CMD_VEL Command (Continuous Jog):**
**Given** X axis is enabled
**When** I send `VEL X 50`
**Then** X axis moves continuously at 50 units/sec (positive direction)
**And** motion continues until CMD_STOP or limit reached
**And** response is RESP_OK

**Given** I send `VEL Y -25`
**When** command executes
**Then** Y axis jogs in negative direction at 25 units/sec

**CMD_STOP Command:**
**Given** X axis is moving
**When** I send `STOP X`
**Then** X axis decelerates and stops
**And** response is RESP_OK
**And** `EVENT DONE X <final_position>` generated

**Given** multiple axes are moving
**When** I send `STOP` (no axis specified)
**Then** all axes stop
**And** response is RESP_OK

**CMD_EN Command (Enable/Disable):**
**Given** X axis is disabled
**When** I send `EN X 1`
**Then** X axis is enabled
**And** response is RESP_OK

**Given** X axis is enabled and moving
**When** I send `EN X 0`
**Then** motion stops immediately
**And** axis is disabled
**And** response is RESP_OK

**CMD_POS Command (Position Query):**
**Given** X axis is at position 123.456
**When** I send `POS X`
**Then** response is `OK X 123.456`

**Given** I send `POS` (no axis)
**When** command executes
**Then** response is `OK X:0.000 Y:0.000 Z:0.000 A:0.000 B:0.000 C:0.000 D:0.000 E:0.000`

**Prerequisites:** Story 3.9

**Technical Notes:**
- CMD_VEL uses very large pulse count (effectively infinite until CMD_STOP)
- CMD_STOP can use deceleration ramp (post-MVP: S-curve)
- CMD_EN integrates with shift register driver
- CMD_POS converts pulse count to user units
- FR8: jog movements, FR9: immediate stop, FR10: enable/disable, FR43: position reporting

---

### Story 3.11: Motion Completion Events

**As a** user,
**I want** notification when motion completes,
**So that** I can sequence operations without polling.

**Acceptance Criteria:**

**Given** a CMD_MOVE command is executing
**When** motion completes successfully
**Then** event is published: `EVENT DONE <axis> <position>`

**Given** motion is stopped by CMD_STOP command
**When** axis comes to rest
**Then** event is published: `EVENT DONE <axis> <position>`

**Given** motion is stopped by limit switch (Epic 4)
**When** axis stops
**Then** event is published: `EVENT LIMIT <axis> <MIN|MAX>`
**And** followed by: `EVENT DONE <axis> <position>`

**Given** motion encounters error
**When** error occurs
**Then** event is published: `EVENT ERROR <axis> <code>`

**Event Timing:**
**And** events are published within TIMING_CMD_RESPONSE_MS (10ms) of condition occurring
**And** events are delivered in order (no reordering)
**And** subscriber (USB TX) formats and sends to host using RESP_EVENT

**Prerequisites:** Story 3.9, Story 2.7 (Event System)

**Technical Notes:**
- Motion task publishes events via event_manager
- USB TX task subscribes and formats for output
- FR48: motion completion events
- FR44: motion status reporting
- Foundation for autonomous sequencing by host software

---

## Epic 4: Safety & I/O Systems

**Goal:** Implement safety monitoring, limit switches, E-stop, brake control, driver alarm handling, general-purpose I/O, and homing sequences for safe operation.

**User Value:** After this epic, the system monitors all safety conditions in real-time, stops motion when limits are reached, responds to emergency stop immediately, controls brakes appropriately, detects and clears driver alarms, provides general-purpose I/O access, and can home all axes to known positions.

**FR Coverage:** FR11, FR12, FR13, FR14, FR15, FR16, FR17, FR18, FR33, FR35, FR36, FR37, FR38, FR39, FR40, FR41, FR42, FR45, FR49, FR56, FR57, FR58, FR59, FR60, FR61, FR62, FR63, FR64

---

### Story 4.1: MCP23017 I/O Expander Driver

**As a** developer,
**I want** MCP23017 I2C expanders operational,
**So that** I can read limit switches and control additional I/O.

**Acceptance Criteria:**

**Given** I2C0 is initialized (verified in Story 1.6)
**When** I call MCP23017 driver functions
**Then** both expanders respond correctly (inputs only - all outputs via shift registers)

**API:**
```c
esp_err_t mcp23017_init(uint8_t addr);
esp_err_t mcp23017_set_direction(uint8_t addr, uint8_t port, uint8_t direction);  // 0=output, 1=input
esp_err_t mcp23017_set_pullup(uint8_t addr, uint8_t port, uint8_t pullup);
esp_err_t mcp23017_read_port(uint8_t addr, uint8_t port, uint8_t* value);
esp_err_t mcp23017_read_pin(uint8_t addr, uint8_t port, uint8_t pin, bool* value);
esp_err_t mcp23017_set_interrupt(uint8_t addr, uint8_t port, uint8_t mask, mcp23017_isr_t handler);
```

**Expander Configuration (from config_i2c.h) - INPUTS ONLY:**
| Address | Port A Function | Port B Function |
|---------|-----------------|-----------------|
| I2C_ADDR_MCP23017_0 (0x20) | Limit switches X,Y,Z,A (8 pins) | Limit switches B,C,D,E (8 pins) |
| I2C_ADDR_MCP23017_1 (0x21) | ALARM_INPUT X-D (7 pins) + 1 spare | InPos X,Y,Z,A,B (5 pins) + 3 spare inputs |

> **Note:** All outputs (DIR, EN, BRAKE, ALARM_CLEAR, GP_OUT) are handled via 5x TPIC6B595N shift registers for fast SPI updates.

**Interrupt Configuration (4 interrupt lines for 2 MCPs):**
| GPIO | MCP | Port | Function | Interrupt Use |
|------|-----|------|----------|---------------|
| GPIO_MCP0_INTA | #0 | A | X-A limit switches | Yes - safety critical |
| GPIO_MCP0_INTB | #0 | B | B-E limit switches | Yes - safety critical |
| GPIO_MCP1_INTA | #1 | A | ALARM_INPUT signals | Yes - driver faults |
| GPIO_MCP1_INTB | #1 | B | InPos signals | Optional - polling OK |

**Given** each MCP23017 has separate INTA/INTB lines routed to ESP32 GPIOs
**When** an input changes state on a monitored port
**Then** corresponding interrupt fires on ESP32-S3
**And** ISR identifies source MCP and port from GPIO that triggered

**Prerequisites:** Epic 1 complete, Epic 3 complete

**Technical Notes:**
- Implement in `firmware/components/drivers/mcp23017/`
- Use espressif/mcp23017 component from ESP Component Registry
- Configure interrupt-on-change for limit switch inputs (MCP0)
- Configure interrupt-on-change for ALARM_INPUT (MCP1 Port A)
- InPos signals (MCP1 Port B) can use polling - not time-critical
- Polling fallback at TIMING_I2C_POLL_MS (5ms) if interrupt missed
- I2C mutex required for thread-safe access
- FR35, FR37 foundation (FR36 GP outputs via shift registers)

---

### Story 4.2: Limit Switch Monitoring

**As a** user,
**I want** limit switches monitored continuously,
**So that** motion stops before mechanical damage occurs.

**Acceptance Criteria:**

**Given** 14 limit switches are connected (2 per 7 axes, E axis has none)
**When** I query limit switch status
**Then** each switch state is reported correctly

**Limit Switch Mapping (from config_i2c.h) - All on MCP23017 #0 (0x20):**
| Axis | Min Switch | Max Switch |
|------|------------|------------|
| X | MCP0_GPA0 | MCP0_GPA1 |
| Y | MCP0_GPA2 | MCP0_GPA3 |
| Z | MCP0_GPA4 | MCP0_GPA5 |
| A | MCP0_GPA6 | MCP0_GPA7 |
| B | MCP0_GPB0 | MCP0_GPB1 |
| C | MCP0_GPB2 | MCP0_GPB3 (floating switch) |
| D | MCP0_GPB4 | MCP0_GPB5 |
| E | MCP0_GPB6 | MCP0_GPB7 |

**Polling/Interrupt Behavior:**
**Given** i2c_monitor_task is running at TIMING_I2C_POLL_MS (5ms) interval
**When** limit switch state changes
**Then** safety_monitor is notified within TIMING_I2C_POLL_MS + TIMING_SAFETY_RESPONSE_MS

**Interrupt Mode (Primary - Safety Critical):**
**Given** MCP23017 #0 interrupts are configured:
- GPIO_MCP0_INTA for Port A (X,Y,Z,A limits)
- GPIO_MCP0_INTB for Port B (B,C,D,E limits)
**When** any limit switch activates
**Then** corresponding GPIO triggers ESP32 ISR
**And** ISR signals safety_monitor_task via task notification (NOTIFY_LIMIT)
**And** safety task reads MCP0 to identify which switch triggered

**Polarity Configuration (FR18):**
**Given** limit switch polarity may be NO or NC
**When** configuration is loaded
**Then** each switch's polarity is read from config (DEFAULT_LIMIT_POLARITY)
**And** readings are inverted if NC (normally closed)

**CMD_LIM Command:**
**Given** I send `LIM` (CMD_LIM)
**When** command executes
**Then** response shows all limit states:
```
OK X:00 Y:00 Z:01 A:00 B:00 C:10 D:00 E:--
```
(bit0=min, bit1=max, --=no limits)

**Given** I send `LIM X`
**Then** response is `OK X MIN:0 MAX:0` (or MIN:1 if triggered)

**Prerequisites:** Story 4.1

**Technical Notes:**
- Implement in `firmware/components/control/safety_monitor/`
- Debouncing handled in software (TIMING_DEBOUNCE_MS)
- FR11: 14 limit switches monitored
- FR18: configurable polarity
- Limit state cached in RAM, updated on poll/interrupt

---

### Story 4.3: Limit Switch Motion Stop

**As a** user,
**I want** motion to stop automatically when limit switches activate,
**So that** axes don't crash into mechanical stops.

**Acceptance Criteria:**

**Given** X axis is moving in positive direction
**When** X max limit switch activates
**Then** X axis motion stops immediately
**And** event published: `EVENT LIMIT X MAX`
**And** event published: `EVENT DONE X <position>`
**And** further positive motion commands rejected with RESP_ERROR ERR_LIMIT_ACTIVE

**Given** X axis max limit is active
**When** I send `MOVE X -50` (moving away from limit)
**Then** motion is allowed (moving in safe direction)
**And** response is RESP_OK

**Given** X axis is jogging (CMD_VEL) toward max limit
**When** max limit activates
**Then** jog stops immediately
**And** events published as above

**Velocity Jog Near Limits:**
**Given** I send `VEL Y 100` (positive direction)
**When** Y approaches max limit
**Then** motion continues until limit activates
**Then** motion stops immediately

**Soft Limits (FR17):**
**Given** soft limits are configured (DEFAULT_LIMIT_MIN, DEFAULT_LIMIT_MAX)
**When** I send `MOVE X 9999` (beyond soft limit)
**Then** response is RESP_ERROR ERR_POSITION_LIMIT MSG_POSITION_LIMIT
**And** motion is not started

**Prerequisites:** Story 4.2, Story 3.9

**Technical Notes:**
- safety_monitor_task has highest priority (24) for immediate response
- Uses task notification from limit monitoring to motion tasks
- Each motion task checks limit state before and during motion
- Immediate stop: pulse generator stop() called
- FR13: automatic stop on limit
- FR17: soft limit prevention

---

### Story 4.4: Emergency Stop System

**As a** user,
**I want** E-stop to immediately disable all motors,
**So that** I can quickly stop all motion in an emergency.

**Acceptance Criteria:**

**Given** E-stop button is pressed (GPIO_E_STOP goes LOW/active)
**When** safety_monitor detects state change
**Then** within TIMING_ESTOP_RESPONSE_MS (1ms):
1. All motor enables are cleared via shift register emergency function
2. All brakes engage (fail-safe)
3. GPIO_SR_OE is set HIGH (outputs tristated for extra safety)
4. System mode set to ESTOP
5. Event published: `EVENT ESTOP ACTIVE`
6. All motion commands rejected with RESP_ERROR ERR_EMERGENCY_STOP

**Given** E-stop is released (GPIO_E_STOP goes HIGH/inactive)
**When** user sends CMD_RST command
**Then** response is RESP_OK
**And** system mode transitions to IDLE
**And** event published: `EVENT ESTOP INACTIVE`
**And** motion commands allowed again (after re-enabling axes with CMD_EN)

**CMD_RST Command (Reset from E-stop/Error):**
**Given** system is in ESTOP mode and E-stop button is released
**When** I send `RST`
**Then** system clears E-stop state
**And** response is RESP_OK

**Given** system is in ESTOP mode and E-stop button is still pressed
**When** I send `RST`
**Then** response is RESP_ERROR ERR_ESTOP_ACTIVE MSG_ESTOP_ACTIVE
**And** state remains ESTOP

**Hardware Interrupt:**
**Given** GPIO_E_STOP is configured with interrupt on falling edge
**When** E-stop activates
**Then** ISR executes `sr_emergency_disable_all()` immediately
**And** ISR notifies safety_monitor_task for event generation

**Prerequisites:** Story 3.1 (shift register), Story 2.6 (mode management)

**Technical Notes:**
- E-stop ISR must be minimal: direct register writes only
- Use `sr_emergency_disable_all()` which is ISR-safe
- FR12: hardware E-stop
- E-stop input active-low (typical for safety circuits)
- Consider hardware watchdog for additional safety

---

### Story 4.5: Brake Control System

**As a** developer,
**I want** configurable brake control strategies,
**So that** vertical axes don't fall when motors are disabled.

**Acceptance Criteria:**

**Given** servo axes (X, Y, Z, A, B) have brake outputs (SR_X_BRAKE..SR_B_BRAKE)
**When** brake strategy is configured
**Then** brakes engage/release according to strategy

**Brake Strategies (from config_defaults.h):**
| Strategy | Behavior |
|----------|----------|
| BRAKE_ON_DISABLE | Engage when axis disabled (most common) |
| BRAKE_ON_ESTOP | Engage only on E-stop |
| BRAKE_ON_IDLE | Engage when axis idle (after TIMING_IDLE_TIMEOUT_MS) |
| BRAKE_MANUAL | Only controlled by CMD_BRAKE command |

**Brake Logic (Active-Low on Hardware):**
**Given** brake strategy is BRAKE_ON_DISABLE
**When** I disable X axis with `EN X 0`
**Then** brake engages (shift register bit SR_X_BRAKE = 0)
**And** TIMING_BRAKE_ENGAGE_MS delay before motor drive removed

**Given** brake strategy is BRAKE_ON_DISABLE
**When** I enable X axis with `EN X 1`
**Then** motor drive enabled
**And** TIMING_BRAKE_RELEASE_MS delay
**Then** brake releases (shift register bit SR_X_BRAKE = 1)

**Power Loss Fail-Safe (FR15):**
**Given** shift register outputs are active-low for brakes
**When** power is lost (no SPI clocking)
**Then** all brake outputs float LOW
**And** all brakes engage mechanically (fail-safe)

**CMD_BRAKE Command (Manual Override):**
**Given** brake strategy is BRAKE_MANUAL
**When** I send `BRAKE X 1` (CMD_BRAKE)
**Then** X axis brake engages
**And** response is RESP_OK

**When** I send `BRAKE X 0`
**Then** X axis brake releases
**And** response is RESP_OK

**Prerequisites:** Story 3.1

**Technical Notes:**
- Implement in `firmware/components/control/safety_monitor/`
- Brake timing critical: engage before disable, release after enable
- Per-axis strategy stored in configuration
- FR14: configurable brake strategies
- FR15: fail-safe engagement
- Only servo axes have brakes (5 total)

---

### Story 4.6: Position Loss Detection

**As a** user,
**I want** to know if axis position may be lost,
**So that** I can re-home before continuing operation.

**Acceptance Criteria:**

**Given** servo axis has position feedback (servos X, Y, Z, A, B via external encoders on drives)
**When** actual position diverges from commanded position
**Then** event published: `EVENT POSLOS <axis>`
**And** axis state set to ERROR

**Position Loss Conditions:**
1. **Power cycle**: All positions marked as "not homed" on boot
2. **E-stop**: Positions flagged as potentially lost (servos may have coasted)
3. **Motion watchdog timeout**: Motion expected but no progress detected

**CMD_POSOK Command (Position Acknowledge):**
**Given** axis X is in POSLOS (position loss) state
**When** I send `POSOK X` (CMD_POSOK)
**Then** position loss flag is cleared
**And** axis state returns to IDLE
**And** response is RESP_OK
**Note:** User takes responsibility for position accuracy

**Given** I send `POSOK` (no axis)
**Then** all position loss flags cleared

**Stepper Position Loss:**
**Given** stepper axes (C, D) have no external feedback
**When** power is cycled
**Then** stepper positions marked as "unknown" (requires homing)
**And** `STAT C` shows HOMED:0

**Prerequisites:** Story 4.1, Story 3.6

**Technical Notes:**
- Basic position loss detection based on system state changes
- Steppers: position always "lost" on power cycle
- FR16: position loss detection
- **Advanced position feedback (Z-signal sync, InPos confirmation) is in Epic 6**

---

### Story 4.7: General-Purpose Digital I/O

**As a** user,
**I want** to read inputs and control outputs,
**So that** I can interface with external sensors and actuators.

**Acceptance Criteria:**

**Digital Inputs (4 spare pins on MCP23017 #1):**
| Pin | MCP | Port.Pin | Config Define |
|-----|-----|----------|---------------|
| DIN0 | #1 | GPA7 | MCP1_GP_IN_0 |
| DIN1 | #1 | GPB5 | MCP1_GP_IN_1 |
| DIN2 | #1 | GPB6 | MCP1_GP_IN_2 |
| DIN3 | #1 | GPB7 | MCP1_GP_IN_3 |

**Given** I send `DIN` (CMD_DIN)
**When** command executes
**Then** response is `OK 0b1010` (4-bit binary) or `OK 10` (decimal)

**Given** I send `DIN 2` (read input 2)
**Then** response is `OK DIN2 1` (or 0)

**Given** input aliases are configured (e.g., "SENSOR1" = DIN1)
**When** I send `DIN SENSOR1`
**Then** response is `OK SENSOR1 1`

**Digital Outputs (8 pins on shift register SR4):**
| Pin | SR | Bit | Config Define |
|-----|-----|-----|---------------|
| DOUT0-7 | SR4 | Q0-Q7 | SR_GP_OUT_0..SR_GP_OUT_7 (bits 32-39) |

**Given** I send `DOUT 5 1` (CMD_DOUT)
**When** command executes
**Then** output 5 is set HIGH via shift register (SR_GP_OUT_5, bit 37)
**And** response is RESP_OK

**Given** I send `DOUT` (query all)
**Then** response is `OK 0b00100000` (showing bit 5 set)

**Given** output alias "LIGHT" = DOUT7
**When** I send `DOUT LIGHT 1`
**Then** output 7 is set HIGH via shift register (SR_GP_OUT_7, bit 39)

**Input Debouncing (FR41):**
**Given** TIMING_DEBOUNCE_MS is configured (default 10ms)
**When** input changes state
**Then** state is only updated after stable for debounce period
**And** rapid toggling is filtered out

**Input Event Notifications:**
**Given** input event mode is enabled for DIN1
**When** DIN1 changes from 0 to 1
**Then** event published: `EVENT DIN DIN1 1`

**Prerequisites:** Story 4.1, Story 3.1 (shift register driver)

**Technical Notes:**
- Implement commands in `firmware/components/control/command_executor/`
- FR35: 4 digital inputs (spare pins on MCP23017 #1)
- FR36: 8 digital outputs via shift register SR4 (bits 32-39)
- FR41: debouncing
- FR42: pin names/aliases
- Digital inputs share MCP1 with alarm/InPos signals - coordinate I2C access
- Outputs via SPI shift register for fast updates

---

### Story 4.8: Servo Feedback Processing (Basic InPos)

**As a** user,
**I want** servo InPos feedback signals read and reported,
**So that** I can query servo position-in-range status.

**Acceptance Criteria:**

**InPos Signal Reading:**
**Given** servo driver X reports position reached (InPos_X active on MCP23017 #1 Port B, pin GPB0)
**When** I query servo status
**Then** InPos state is reported

**CMD_SERVOSTAT Command:**
**Given** I send `SERVOSTAT` (CMD_SERVOSTAT)
**When** command executes
**Then** response shows servo InPos status:
```
OK X:INPOS:1 Y:INPOS:1 Z:INPOS:0 A:INPOS:1 B:INPOS:1
```

**Prerequisites:** Story 4.1, Story 3.6

**Technical Notes:**
- InPos signals on MCP23017 #1 (0x21) Port B (5 pins: GPB0-GPB4 for servo axes X,Y,Z,A,B)
- FR37: servo feedback processing (InPos portion - basic read)
- InPos is read-only status in this story (no timeout/error handling)
- Alarm handling is separate in Story 4.14 (uses MCP23017 #1 Port A)
- **Advanced InPos processing (motion confirmation, timeout errors) is in Epic 6 Story 6.5**

---

### Story 4.9: C-Axis Floating Switch (Object Width Measurement)

**As a** user,
**I want** to measure object width using C axis floating switch,
**So that** the picker jaw can detect object size.

**Acceptance Criteria:**

**Given** C axis (picker jaw) has floating switch connected
**When** C axis closes and contacts object
**Then** floating switch activates
**And** motion stops
**And** width calculated from position
**And** event published: `EVENT WIDTH C <measured_width>`

**Width Measurement Sequence:**
1. User sends `CLOSE C` or `MOVE C 0` (close direction)
2. C axis moves toward closed position
3. Floating switch activates when object contacted
4. Motion stops (not a limit switch - separate input)
5. Width = C_AXIS_MAX_OPEN - current_position
6. Width stored for query

**Floating Switch Input:**
**Given** floating switch is connected to C_LIMIT_MAX input (MCP0 GPB3)
**When** switch activates during closing motion
**Then** GPIO_MCP0_INTB triggers interrupt (same as B-E limit switches)
**And** safety task identifies floating switch vs hard limit switch
**And** position recorded
**And** event generated (FR39)

**CMD_WIDTH Command (FR40):**
**Given** width measurement has been taken
**When** I send `WIDTH C` (CMD_WIDTH)
**Then** response is `OK C 45.5` (last measured width in mm)

**Given** no measurement has been taken
**When** I send `WIDTH C`
**Then** response is `OK C -1.0` (no measurement)

**Prerequisites:** Story 4.2, Story 3.7

**Technical Notes:**
- Floating switch uses C_LIMIT_MAX input (MCP0 GPB3) - shares interrupt with B-E limits
- Floating switch is different from hard limit switches - doesn't set error state
- Only C axis has this feature (picker jaw)
- FR38: detect floating switch
- FR39: measure and report width via event
- FR40: query last measured width
- Width = jaw_open_position - contact_position

---

### Story 4.10: Error Tracking & Recovery

**As a** user,
**I want** errors tracked and recoverable,
**So that** I can diagnose issues and resume operation.

**Acceptance Criteria:**

**Error Counting (FR45):**
**Given** errors occur during operation
**When** I send `ERRCNT` (CMD_ERRCNT)
**Then** response shows error counts:
```
OK I2C:3 TIMEOUT:1 LIMIT:5 ESTOP:2 POSLOS:0
```

**Given** I send `ERRCNT X`
**Then** response shows axis-specific errors:
```
OK X LIMIT:2 POSLOS:0 TIMEOUT:1
```

**Error Event Generation (FR49):**
**Given** I2C communication fails
**When** failure detected
**Then** event published: `EVENT ERROR SYSTEM E020` (ERR_I2C_FAILURE)

**Given** axis motion fails
**When** failure detected
**Then** event published: `EVENT ERROR <axis> <code>`

**CMD_CLRERR Command (FR60):**
**Given** axis X is in error state
**When** I send `CLRERR X` (CMD_CLRERR)
**Then** X axis error state is cleared
**And** axis state returns to IDLE
**And** response is RESP_OK

**Given** I send `CLRERR` (no axis)
**Then** all axis errors cleared
**And** system error state cleared

**Unsafe Operation Prevention (FR61):**
**Given** axis X is in ERROR state
**When** I send `MOVE X 100`
**Then** response is RESP_ERROR ERR_AXIS_ERROR MSG_CLEAR_ERROR_FIRST
**And** motion not started

**Prerequisites:** Story 4.4, Story 4.6

**Technical Notes:**
- Error counts stored in RAM, persist until power cycle
- Error counts per category and per axis
- FR45: cumulative error counts
- FR49: error/fault events
- FR60: clear error conditions
- FR61: prevent unsafe operations

---

### Story 4.11: I2C Communication Health

**As a** user,
**I want** I2C health monitored,
**So that** communication failures are detected and handled.

**Acceptance Criteria:**

**Health Monitoring (FR46):**
**Given** i2c_monitor_task is running
**When** I2C transaction fails (timeout, NACK)
**Then** failure count incremented
**And** if count > I2C_FAILURE_THRESHOLD: event published `EVENT ERROR SYSTEM E020`

**Given** I2C transactions succeed after failures
**When** success count > I2C_RECOVERY_THRESHOLD
**Then** health status restored
**And** event published: `EVENT I2C RECOVERED`

**CMD_I2C Command:**
**Given** I send `I2C` (CMD_I2C)
**When** command executes
**Then** response shows I2C health:
```
OK I2C0:OK(1234) I2C1:OK(5678)
```
(numbers are transaction counts since boot)

**Given** I send `I2C SCAN`
**Then** response lists all detected devices:
```
OK 0x20:MCP23017 0x21:MCP23017 0x3C:OLED
```

**Error Recovery (FR58):**
**Given** I2C bus is stuck (SDA held low)
**When** recovery triggered
**Then** clock pulses sent to unstick bus
**And** bus re-initialized
**And** operation retried

**Error Detection (FR56):**
**Given** expected device not responding
**When** multiple retries fail
**Then** device marked as offline
**And** event published: `EVENT ERROR I2C <addr>`
**And** dependent functionality degraded gracefully

**Prerequisites:** Story 4.1

**Technical Notes:**
- Implement in `firmware/components/control/safety_monitor/` or dedicated i2c_health component
- FR46: I2C health monitoring
- FR56: detect I2C failures
- FR58: transient error recovery
- Use ESP-IDF i2c_master_bus_reset() for stuck bus recovery

---

### Story 4.12: Error Logging

**As a** developer,
**I want** errors logged for troubleshooting,
**So that** I can diagnose issues after they occur.

**Acceptance Criteria:**

**Error Log Storage (FR59):**
**Given** error occurs
**When** logged
**Then** entry includes:
- Timestamp (ms since boot)
- Error code
- Axis (if applicable)
- Context data (position, command, etc.)

**Given** log storage is full (LIMIT_ERROR_LOG_ENTRIES)
**When** new error occurs
**Then** oldest entry is overwritten (circular buffer)

**CMD_LOG Command:**
**Given** I send `LOG` (CMD_LOG)
**When** command executes
**Then** response shows recent errors:
```
OK LOG 3 ENTRIES
12345 E004 X AXIS_NOT_ENABLED
23456 E013 Y LIMIT_ACTIVE
34567 E020 SYSTEM I2C_FAILURE
```

**Given** I send `LOG CLEAR`
**Then** error log is cleared
**And** response is RESP_OK

**Detailed Error Messages (FR57):**
**Given** error response is generated
**When** response is formatted
**Then** message includes code and description:
```
ERROR E004 Axis not enabled: X
ERROR E013 Limit switch active: Y MAX
ERROR E020 I2C communication failure: 0x20
```

**Prerequisites:** Story 4.10

**Technical Notes:**
- Implement in `firmware/components/events/` or separate logging component
- Store in RAM (not NVS - too many writes)
- Circular buffer with configurable depth
- FR57: detailed error messages
- FR59: error logging
- Consider outputting to ESP_LOG for serial debug

---

### Story 4.13: Homing Sequences (Limit-Based)

**As a** user,
**I want** to home axes to known positions using limit switches,
**So that** I can establish accurate position references.

**Acceptance Criteria:**

**CMD_HOME Command (FR33):**
**Given** X axis is enabled
**When** I send `HOME X` (CMD_HOME)
**Then** X axis executes basic homing sequence:
1. Move toward home limit at homing speed (DEFAULT_HOMING_VELOCITY)
2. Stop when limit switch activates
3. Event: `EVENT HOMING X LIMIT_HIT POS:<pos>`
4. Back off limit by homing_backoff distance
5. Event: `EVENT HOMING X BACKOFF POS:<pos>`
6. Move slowly toward limit again (homing_velocity_slow)
7. Stop at limit activation
8. Set position to 0 (or configured home offset)
9. Response: RESP_OK
10. Event: `EVENT HOMING X COMPLETE POS:0.000`
**And** axis position is now referenced (HOMED:1)

**Given** I send `HOME` (no axis)
**Then** all axes home in sequence (one at a time)
**And** order defined by HOMING_SEQUENCE_ORDER from config

**Homing State Events:**
```
EVENT HOMING X SEEK_LIMIT POS:0.150
EVENT HOMING X LIMIT_HIT POS:-0.002
EVENT HOMING X BACKOFF POS:-0.002
EVENT HOMING X COMPLETE POS:0.000
```

**Homing Direction:**
**Given** axis home direction is configurable
**When** homing.direction = "min" (config)
**Then** axis homes toward min limit
**When** homing.direction = "max"
**Then** axis homes toward max limit

**Homing Abort:**
**Given** homing is in progress
**When** I send `STOP` or E-stop activates
**Then** homing aborts
**And** event: `EVENT HOMING <axis> FAILED ABORTED`
**And** axis position remains "not homed"

**Backoff Collision Detection:**
**Given** homing is in BACKOFF phase
**When** opposite limit is hit during backoff
**Then** homing fails with `EVENT HOMING <axis> FAILED BACKOFF_COLLISION`

**Home Status Query:**
**Given** I send `STAT X`
**Then** response includes homing status:
```
OK X POS:0.000 EN:1 MOV:0 ERR:0 LIM:00 HOMED:1
```

**Prerequisites:** Story 4.2, Story 4.3, Story 3.9

**Technical Notes:**
- Implement in `firmware/components/control/motion_controller/` or dedicated homing component
- Two-stage homing: fast approach, slow final (limit-only)
- Uses limit switches as home sensors
- FR33: homing sequences (basic limit-based)
- Stepper axes especially need homing (no absolute position)
- E axis (discrete) doesn't need homing (NONE mode switches)
- **Z-signal homing (SEEK_ZSIGNAL phase) is in Epic 6 Story 6.3**

---

### Story 4.14: Driver Alarm Monitoring & Clearance

**As a** user,
**I want** driver alarms detected and clearable,
**So that** I know when motor drivers have faults and can attempt to recover without power cycling.

**Acceptance Criteria:**

**Alarm Detection (FR62):**
**Given** motor driver X enters alarm state (overcurrent, overheat, position error, etc.)
**When** ALARM_INPUT_X signal goes active on MCP23017_1 Port A
**Then** event published: `EVENT ALARM X`
**And** X axis motion stopped immediately
**And** X axis state set to ALARM
**And** OLED shows "ALARM X" with high priority

**Given** multiple drivers have alarms
**When** alarms are active on X and Z
**Then** events published: `EVENT ALARM X`, `EVENT ALARM Z`
**And** both axes stopped and set to ALARM state

**Alarm Status Query:**
**Given** I send `STAT X`
**When** X has active alarm
**Then** response includes alarm status:
```
OK X POS:123.456 EN:1 MOV:0 ERR:0 LIM:00 HOMED:1 ALM:1
```

**Given** I send `STAT`
**Then** response shows alarm status for all axes:
```
OK X:ALM:0 Y:ALM:1 Z:ALM:0 A:ALM:0 B:ALM:0 C:ALM:0 D:ALM:0
```

**Motion Blocking (FR64):**
**Given** axis X has active alarm (ALM:1)
**When** I send `MOVE X 100`
**Then** response is `ERROR E014 Driver alarm active`
**And** motion not started

**Given** axis X alarm is cleared (ALM:0)
**When** I send `MOVE X 100`
**Then** motion executes normally

**Alarm Clearance (FR63):**
**Given** axis X has active alarm
**When** I send `CLR X` (CMD_CLR)
**Then** ALARM_CLEAR_X output pulses via shift register (SR_X_ALARM_CLR, bit 3)
**And** system waits TIMING_ALARM_CLEAR_PULSE_MS (100ms default)
**And** ALARM_CLEAR_X output deasserts
**And** system checks ALARM_INPUT_X on MCP23017 #1 after TIMING_ALARM_CHECK_DELAY_MS (50ms)

**Given** alarm clears successfully (ALARM_INPUT_X inactive)
**Then** response is `OK`
**And** event published: `EVENT ALARMCLR X`
**And** X axis state returns to IDLE
**And** motion commands allowed again

**Given** alarm persists after clear attempt
**Then** response is `ERROR E015 Alarm clear failed`
**And** axis remains in ALARM state
**And** user should check physical cause (motor, driver, wiring)

**CLR ALL Command:**
**Given** multiple axes have alarms
**When** I send `CLR ALL`
**Then** clear attempted on all axes with active alarms
**And** results reported:
```
OK X:CLEARED Y:FAILED Z:CLEARED
```

**Prerequisites:** Story 4.1 (MCP23017 driver), Story 4.10 (error tracking)

**Technical Notes:**
- ALARM_INPUT signals on MCP23017 #1 (0x21) Port A pins GPA0-GPA6 for axes X-D
- ALARM_CLEAR outputs on shift register bits SR_X_ALARM_CLR through SR_D_ALARM_CLR (bits 3, 7, 11, 15, 19, 23, 27)
- Interrupt on GPIO_MCP1_INTA for alarm detection (see architecture)
- FR62: monitor ALARM_INPUT signals
- FR63: CLR command pulses ALARM_CLEAR via shift register
- FR64: block motion when alarm active
- E axis (discrete actuator) has no alarm signals
- Clear pulse polarity/duration may need per-driver configuration
- Alarm states tracked in axis_state[].alarm_active
- ISR handler queues notification to safety task (process_alarm_inputs)

---

## Epic 5: Configuration & Status Display

**Goal:** Implement YAML configuration loading/export, runtime parameter adjustment, NVS persistence, axis aliasing, and OLED status display.

**User Value:** After this epic, users can configure the system via YAML files, adjust parameters at runtime without recompilation, persist calibration data, use custom axis names, and see real-time status on the OLED display.

**FR Coverage:** FR27, FR28, FR29, FR30, FR31, FR32, FR34, FR46, FR47, FR47a, FR47b, FR47c, FR50

---

### Story 5.1: YAML Parser Integration

**As a** developer,
**I want** YAML parsing capability,
**So that** configuration files can be loaded and processed.

**Acceptance Criteria:**

**Given** YAML text is provided
**When** parser processes it
**Then** configuration values are extracted correctly

**Parser API:**
```c
esp_err_t yaml_parse_string(const char* yaml_text, YamlConfig* config);
esp_err_t yaml_parse_section(const char* yaml_text, const char* section, YamlSection* out);
esp_err_t yaml_get_int(const YamlSection* section, const char* key, int32_t* value);
esp_err_t yaml_get_float(const YamlSection* section, const char* key, float* value);
esp_err_t yaml_get_string(const YamlSection* section, const char* key, char* value, size_t max_len);
esp_err_t yaml_get_bool(const YamlSection* section, const char* key, bool* value);
```

**YAML Schema (from config_yaml_schema.h):**
```yaml
system:
  name: "MyRobot"
  version: 1

axes:
  X:
    alias: "RAILWAY"
    type: linear                    # linear (meters) or rotary (radians)
    pulses_per_rev: 10000           # Driver PA14 setting (pulses per motor revolution)
    units_per_rev: 0.005            # Physical travel per rev: 5mm ball screw = 0.005 m/rev
    # Derived: pulses_per_unit = pulses_per_rev / units_per_rev = 2,000,000 pulses/meter
    max_velocity: 0.200             # 200mm/s in m/s (SI units)
    max_acceleration: 1.0           # 1 m/s² (SI units)
    limits: [-0.500, 0.500]         # ±500mm in meters (SI units)
    home_direction: -1
    home_velocity: 0.010            # 10mm/s in m/s
    brake_strategy: "on_disable"
    z_signal:
      enabled: true                 # Enable Z-signal position sync
      drift_threshold: 100          # Alarm if drift > 100 pulses
  # ... Y, Z, A, B, C, D, E

io:
  inputs:
    DIN0: "SENSOR1"
    DIN1: "DOOR_OPEN"
  outputs:
    DOUT0: "LIGHT"
    DOUT7: "ALARM"
```

**Validation:**
**Given** YAML contains invalid values (out of range, wrong type)
**When** validation runs
**Then** specific error returned with key name and expected type
**And** configuration not applied (atomic: all or nothing)

**Prerequisites:** Epic 1 complete

**Technical Notes:**
- Implement in `firmware/components/config/yaml_parser/`
- Consider using minimal YAML parser (no full libyaml to save flash)
- Or implement simple line-by-line parser for flat/simple structures
- Schema validation against config_yaml_schema.h macros
- FR27, FR28 foundation

---

### Story 5.2: Configuration Upload (CMD_CFGSTART/CMD_CFGDATA/CMD_CFGEND)

**As a** user,
**I want** to upload configuration via USB,
**So that** I can configure the system without recompilation.

**Acceptance Criteria:**

**Configuration Upload Sequence:**
**Given** I want to upload new configuration
**When** I execute the upload sequence
**Then** configuration is applied:

1. Send `CFGSTART` (CMD_CFGSTART)
   - Response: RESP_OK
   - System enters CONFIG mode
   - Motion commands blocked (FR55)

2. Send `CFGDATA <base64_chunk>` (CMD_CFGDATA) - repeat for all chunks
   - Response: RESP_OK after each chunk
   - Data accumulated in RAM buffer
   - Maximum chunk size: LIMIT_CFGDATA_CHUNK_SIZE (256 bytes)

3. Send `CFGEND` (CMD_CFGEND)
   - Response: RESP_OK if valid, RESP_ERROR if parse/validation fails
   - YAML parsed and validated
   - If valid: configuration applied, persisted to NVS
   - System returns to READY mode
   - Event: `EVENT CONFIG LOADED`

**Alternative: Direct YAML (Small Configs):**
**Given** configuration is small (< LIMIT_CMD_MAX_LENGTH)
**When** I send `CFG <yaml_text>` (single line format)
**Then** configuration is parsed and applied immediately

**Partial Configuration:**
**Given** YAML only contains some axes
**When** configuration is applied
**Then** only specified axes are updated
**And** other axes retain existing settings

**Prerequisites:** Story 5.1, Story 2.6 (mode management)

**Technical Notes:**
- Implement in `firmware/components/config/`
- Base64 encoding handles binary-safe transfer
- RAM buffer sized to LIMIT_CONFIG_MAX_SIZE (4KB typical)
- Atomic application: parse/validate before applying
- FR27: load configuration via USB

---

### Story 5.3: Configuration Export (CMD_CFGEXPORT)

**As a** user,
**I want** to download current configuration,
**So that** I can backup, modify, or transfer settings.

**Acceptance Criteria:**

**CMD_CFGEXPORT Command (FR28):**
**Given** I send `CFGEXPORT` (CMD_CFGEXPORT)
**When** command executes
**Then** current configuration is output as YAML:
```
OK CONFIG BEGIN
system:
  name: "MyRobot"
  version: 1
axes:
  X:
    alias: "RAILWAY"
    type: linear
    pulses_per_rev: 10000
    units_per_rev: 0.005
    ...
OK CONFIG END
```

**Given** I send `CFGEXPORT X`
**Then** only X axis configuration is output

**Given** I send `CFGEXPORT SYSTEM`
**Then** only system section is output

**Chunked Output:**
**Given** configuration is large
**When** exported
**Then** output is chunked with continuation markers
**And** host can reconstruct complete YAML

**Prerequisites:** Story 5.1

**Technical Notes:**
- Generate YAML from current runtime configuration
- Format matches expected import format (round-trip compatible)
- FR28: export configuration to YAML
- Useful for backup before changes
- Essential for YAML editing workflow

---

### Story 5.4: Runtime Parameter Adjustment (CMD_SET)

**As a** user,
**I want** to adjust motion parameters at runtime,
**So that** I can tune the system without uploading full configuration.

**Acceptance Criteria:**

**CMD_SET Command (FR29):**
**Given** I send `SET X MAXVEL 150.0` (CMD_SET)
**When** command executes
**Then** X axis max velocity is changed to 150.0
**And** response is RESP_OK
**And** change takes effect immediately

**Settable Parameters:**
| Parameter | Key | Example |
|-----------|-----|---------|
| Max velocity | MAXVEL | `SET X MAXVEL 100` |
| Acceleration | ACCEL | `SET Y ACCEL 500` |
| Soft limit min | LIMMIN | `SET Z LIMMIN -100` |
| Soft limit max | LIMMAX | `SET Z LIMMAX 500` |
| Units per pulse | RATIO | `SET A RATIO 0.01` |
| Jerk (post-MVP) | JERK | `SET B JERK 1000` |

**Query Current Value:**
**Given** I send `SET X MAXVEL` (no value)
**Then** response is `OK X MAXVEL 100.0` (current value)

**Given** I send `SET X` (no parameter)
**Then** response lists all settable parameters for X axis

**Validation:**
**Given** I send `SET X MAXVEL -50` (negative velocity)
**When** validation runs
**Then** response is RESP_ERROR ERR_INVALID_PARAM MSG_VALUE_OUT_OF_RANGE
**And** parameter not changed

**Prerequisites:** Story 3.6 (motor control needs params)

**Technical Notes:**
- Parameters stored in AxisConfig structure
- Changes immediate but NOT auto-persisted (use CMD_SAVE)
- FR29: runtime adjustable parameters
- Provides fast tuning without full config cycle

---

### Story 5.5: NVS Configuration Persistence (CMD_SAVE, CMD_LOAD)

**As a** user,
**I want** configuration persisted across power cycles,
**So that** I don't have to reconfigure after every restart.

**Acceptance Criteria:**

**CMD_SAVE Command (FR30):**
**Given** configuration has been modified
**When** I send `SAVE` (CMD_SAVE)
**Then** current configuration is written to NVS
**And** response is RESP_OK

**Given** I send `SAVE X` (single axis)
**Then** only X axis configuration is saved

**CMD_LOAD Command:**
**Given** configuration exists in NVS
**When** I send `LOAD` (CMD_LOAD)
**Then** configuration is loaded from NVS
**And** response is RESP_OK
**And** current settings replaced

**Auto-Load on Boot:**
**Given** system boots
**When** NVS contains valid configuration
**Then** configuration is automatically loaded
**And** boot log shows "Configuration loaded from NVS"

**Given** NVS is empty or corrupted
**When** system boots
**Then** default configuration (from config_defaults.h) is used
**And** boot log shows "Using default configuration"

**Factory Reset:**
**Given** I send `LOAD DEFAULTS` (CMD_LOAD DEFAULTS)
**Then** default configuration is loaded
**And** NVS configuration is NOT erased (use CMD_NVSERASE for that)

**CMD_NVSERASE Command:**
**Given** I send `NVSERASE` (CMD_NVSERASE)
**Then** all NVS configuration is erased
**And** response is RESP_OK
**And** next boot will use defaults

**Prerequisites:** Story 5.1

**Technical Notes:**
- Implement in `firmware/components/config/nvs_manager/`
- Use ESP-IDF NVS API with namespace "yarobot"
- Store serialized config structure or key-value pairs
- NVS wear leveling handled by ESP-IDF
- FR30: persistent calibration data

---

### Story 5.6: Position Clear (CMD_CLR)

**As a** user,
**I want** to clear axis positions without moving,
**So that** I can set current position as reference.

**Acceptance Criteria:**

**CMD_CLR Command (FR31):**
**Given** X axis is at position 123.456
**When** I send `CLR X` (CMD_CLR)
**Then** X axis position is set to 0.000
**And** response is RESP_OK
**And** no physical motion occurs

**Given** I send `CLR X 50.0`
**Then** X axis position is set to 50.0
**And** response is RESP_OK
**And** no physical motion occurs

**Given** I send `CLR` (no axis)
**Then** all axis positions cleared to 0
**And** response is RESP_OK

**Use Case:**
**Given** user manually positions axis to known location
**When** user sends `CLR X 100.0`
**Then** current physical position becomes 100.0 in software
**And** subsequent moves are relative to this reference

**Prerequisites:** Story 3.5 (position tracker)

**Technical Notes:**
- Modifies position tracker offset, not physical position
- FR31: clear positions without movement
- Also known as "teach" or "set origin" in industrial terminology
- Does not affect homed status (still requires HOME after power cycle)

---

### Story 5.7: Scaling Ratio Configuration (CMD_SCALE)

**As a** user,
**I want** to set custom scaling ratios per axis,
**So that** positions are in meaningful SI units (meters, radians).

**Acceptance Criteria:**

**CMD_SCALE Command (FR32):**
**Given** I send `SCALE X PPR 10000` (pulses per revolution)
**When** command executes
**Then** X axis pulses_per_rev is set to 10000
**And** response is RESP_OK

**Given** I send `SCALE X UPR 0.005` (units per revolution - meters for linear, radians for rotary)
**When** command executes
**Then** X axis units_per_rev is set to 0.005 (5mm ball screw lead)
**And** derived pulses_per_unit = 10000 / 0.005 = 2,000,000 pulses/meter
**And** response is RESP_OK

**Given** I send `SCALE X` (query)
**Then** response is `OK X PPR:10000 UPR:0.005 PPU:2000000`

**Given** I send `SCALE` (query all)
**Then** response shows all axes scaling:
```
OK X:PPR:10000,UPR:0.005 Y:PPR:10000,UPR:6.283 Z:PPR:10000,UPR:0.010 ...
```

**Unit Verification:**
**Given** X has PPR=10000, UPR=0.005 (pulses_per_unit = 2,000,000)
**When** I send `MOVE X 0.150` (move to 150mm in meters)
**Then** motor moves 300,000 pulses
**And** position reports 0.150 (meters)

**Prerequisites:** Story 3.6

**Technical Notes:**
- Two fundamental parameters: pulses_per_rev (driver setting) and units_per_rev (mechanical lead/ratio)
- Derived: pulses_per_unit = pulses_per_rev / units_per_rev
- All external values in SI units: meters (linear) or radians (rotary)
- Affects: MOVE, MOVR, VEL, POS, limits
- Velocity scaled: SI units/sec → pulses/sec
- FR32: custom scaling ratios (now using intuitive rev-based parameters)

---

### Story 5.8: Axis Aliasing (CMD_ALIAS)

**As a** user,
**I want** custom axis names,
**So that** commands are more readable for my application.

**Acceptance Criteria:**

**CMD_ALIAS Command (FR34):**
**Given** I send `ALIAS X RAILWAY` (CMD_ALIAS)
**When** command executes
**Then** X axis can be addressed as "RAILWAY"
**And** response is RESP_OK

**Given** alias "RAILWAY" is set for X
**When** I send `MOVE RAILWAY 100`
**Then** X axis moves to 100
**And** response is RESP_OK

**Given** I send `POS RAILWAY`
**Then** response is `OK RAILWAY 100.000`

**Query Aliases:**
**Given** I send `ALIAS` (CMD_ALIAS with no params)
**Then** response lists all aliases:
```
OK X:RAILWAY Y:SHUTTLE Z:LIFT A:ROTATE B:TILT C:GRIPPER D:FEED E:ACTUATOR
```

**Clear Alias:**
**Given** I send `ALIAS X -`
**Then** X axis alias is cleared
**And** must use "X" to address axis

**Validation:**
**Given** I send `ALIAS RAILWAY X` (wrong order)
**Then** response is RESP_ERROR ERR_INVALID_PARAM (must be axis first, then alias)

**Given** I send `ALIAS X GRIPPER` but GRIPPER already assigned to C
**Then** response is RESP_ERROR ERR_ALIAS_EXISTS

**Prerequisites:** Story 2.2 (command parser needs alias lookup)

**Technical Notes:**
- Aliases stored in AxisConfig
- Alias lookup in command parser before axis validation
- Maximum alias length: LIMIT_ALIAS_MAX_LENGTH (16 chars)
- FR34: custom axis aliases
- Both axis letter and alias work simultaneously

---

### Story 5.9: OLED Display Driver

**As a** developer,
**I want** the OLED display operational,
**So that** status information can be shown to users.

**Acceptance Criteria:**

**Given** I2C1 is initialized (verified in Story 1.6)
**When** OLED driver initializes
**Then** SSD1306 128x64 display shows content

**Display API:**
```c
esp_err_t oled_init(void);
esp_err_t oled_clear(void);
esp_err_t oled_draw_text(uint8_t x, uint8_t y, const char* text, oled_font_t font);
esp_err_t oled_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
esp_err_t oled_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool filled);
esp_err_t oled_set_contrast(uint8_t contrast);
esp_err_t oled_display_on(bool on);
esp_err_t oled_update(void);  // Flush buffer to display
```

**Font Support:**
- FONT_SMALL: 5x7 pixels (8 lines of ~21 chars)
- FONT_MEDIUM: 8x8 pixels (8 lines of 16 chars)
- FONT_LARGE: 16x16 pixels (4 lines of 8 chars)

**Buffer Management:**
**Given** display operations are called
**When** `oled_update()` is called
**Then** framebuffer is transferred to display via I2C1
**And** double-buffering prevents tearing

**I2C Bus Isolation (FR47c):**
**Given** I2C1 is dedicated to OLED
**When** OLED communication occurs
**Then** it does NOT affect I2C0 (MCP23017s)
**And** limit switch monitoring continues uninterrupted

**Prerequisites:** Story 1.6 (I2C1 verified)

**Technical Notes:**
- Implement in `firmware/components/drivers/oled/`
- Use SSD1306 driver from ESP Component Registry or custom
- I2C1 at lower speed okay (OLED not time-critical)
- FR47c: dedicated I2C bus isolation
- Consider using esp_lvgl_port for more features (post-MVP)

---

### Story 5.10: Status Display Task

**As a** user,
**I want** to see real-time status on the OLED,
**So that** I can monitor operation without a computer.

**Acceptance Criteria:**

**Normal Status Display (FR47):**
**Given** system is in normal operation
**When** display updates (TIMING_DISPLAY_UPDATE_MS = 100ms)
**Then** screen shows:
```
┌────────────────────────┐
│ YAROBOT v1.0.0   READY │
│ X: 123.4  Y:  56.7     │
│ Z:  89.0  A:   0.0     │
│ B:   0.0  C:  12.3     │
│ D: 100.0  E:   1.0     │
│                        │
│ [X=] [Y ] [Z=] [A ]    │
│ [B ] [C ] [D ] [E ]    │
└────────────────────────┘
```
- Axis positions in user units
- Movement indicators: [X=] = idle at position, [X>] = moving positive, [X<] = moving negative

**E-Stop Display (FR47a - Highest Priority):**
**Given** E-stop is active
**When** display updates
**Then** screen shows prominently:
```
┌────────────────────────┐
│     !! E-STOP !!       │
│                        │
│   ALL MOTORS DISABLED  │
│                        │
│  Release E-stop and    │
│  send RST command      │
│                        │
│                        │
└────────────────────────┘
```
**And** this takes priority over all other displays

**Error/Event Display (FR47b):**
**Given** error or event occurs
**When** display updates
**Then** message shown for TIMING_DISPLAY_MESSAGE_MS (2000ms):
```
┌────────────────────────┐
│ EVENT: DONE X 100.0    │
│                        │
│ ... normal status ...  │
└────────────────────────┘
```
**And** after timeout, returns to normal status

**Priority Order:**
1. E-stop (always visible when active)
2. Errors (displayed for 2 seconds)
3. Events (displayed for 2 seconds)
4. Normal status (default)

**Prerequisites:** Story 5.9

**Technical Notes:**
- display_task runs on Core 1 at low priority (5)
- Uses event subscription to receive status updates
- FR47: axis positions and movement status
- FR47a: E-stop highest priority
- FR47b: errors/events for 2 seconds
- Scrolling list for multiple events (or most recent only)

---

### Story 5.11: Extended Status Query (CMD_STAT, CMD_DIAG, CMD_INFO)

**As a** user,
**I want** comprehensive status queries,
**So that** I can check system state and diagnose issues.

**Acceptance Criteria:**

**CMD_STAT All Axes (FR50):**
**Given** I send `STAT` (CMD_STAT)
**When** command executes
**Then** response shows all axes:
```
OK
X POS:123.4 EN:1 MOV:0 ERR:0 LIM:00 HOMED:1
Y POS:56.7 EN:1 MOV:1 ERR:0 LIM:00 HOMED:1
Z POS:89.0 EN:0 MOV:0 ERR:0 LIM:01 HOMED:0
A POS:0.0 EN:1 MOV:0 ERR:0 LIM:00 HOMED:1
B POS:0.0 EN:1 MOV:0 ERR:0 LIM:00 HOMED:1
C POS:12.3 EN:1 MOV:0 ERR:0 LIM:10 HOMED:0
D POS:100.0 EN:1 MOV:0 ERR:1 LIM:00 HOMED:0
E POS:1.0 EN:1 MOV:0 ERR:0 LIM:-- HOMED:-
```

**CMD_DIAG System Diagnostics (FR46):**
**Given** I send `DIAG` (CMD_DIAG)
**When** command executes
**Then** response shows system health:
```
OK DIAG
UPTIME:123456 ms
MODE:READY
ESTOP:0
I2C0:OK(1234) I2C1:OK(567)
SPI:OK(890)
MEM:FREE 45000 PSRAM 7800000
TASKS:14 RUNNING
ERRCNT:I2C:0 LIMIT:2 POSLOS:0
```

**CMD_INFO Extended (FR50):**
**Given** I send `INFO FULL`
**When** command executes
**Then** response includes detailed info:
```
OK INFO
NAME:YAROBOT_CONTROL_UNIT
VERSION:1.0.0
BUILD:2024-01-15 14:30
IDF:5.1.2
CHIP:ESP32-S3
FLASH:16MB
PSRAM:8MB
MAC:AA:BB:CC:DD:EE:FF
```

**Prerequisites:** Story 2.5, Story 4.11

**Technical Notes:**
- CMD_STAT with axis shows single axis (Story 2.5)
- CMD_STAT without axis shows all (this story)
- CMD_DIAG shows system-level diagnostics
- FR50: query individual or all axes
- FR46: I2C communication health

---

### Story 5.12: Configuration Validation Commands

**As a** developer,
**I want** configuration validation commands,
**So that** I can verify settings are correct before operation.

**Acceptance Criteria:**

**CMD_CFGVALID Command:**
**Given** I send `CFGVALID` (CMD_CFGVALID)
**When** command executes
**Then** response indicates configuration validity:
```
OK CONFIG VALID
```
or
```
ERROR E030 CONFIG INVALID: X.max_velocity exceeds limit
```

**Validation Checks:**
- All required parameters present
- Values within valid ranges (per config_yaml_schema.h)
- No conflicting settings (e.g., limit_min > limit_max)
- GPIO assignments don't conflict
- I2C addresses match expected devices

**CMD_CFGDIFF Command:**
**Given** I send `CFGDIFF` (CMD_CFGDIFF)
**When** command executes
**Then** response shows differences from defaults:
```
OK CFGDIFF
X.max_velocity: 50.0 (default: 100.0)
Y.alias: "SHUTTLE" (default: none)
C.limit_max: 200.0 (default: 1000.0)
```

**Prerequisites:** Story 5.1, Story 5.5

**Technical Notes:**
- Validation runs same checks as config load
- CMD_CFGVALID useful before CMD_SAVE
- CMD_CFGDIFF helps understand customizations
- Part of configuration debugging toolkit

---

## Epic 6: Advanced Position Feedback

**Goal:** Implement Z-signal synchronization, enhanced InPos processing, and precision homing for servo axes.

**User Value:** After this epic, servo axes achieve sub-pulse position accuracy through Z-signal synchronization, motion completion is confirmed via InPos signals with timeout detection, and homing uses encoder index pulses for maximum repeatability.

**FR Coverage:** FR65-70

**Prerequisites:** Epic 3 (motor control), Epic 4 (basic homing, InPos reading)

---

### Story 6.1: Z-Signal GPIO Configuration

**As a** developer,
**I want** Z-signal GPIO inputs configured with interrupts,
**So that** encoder index pulses can be captured for position synchronization.

**Acceptance Criteria:**

**GPIO Configuration:**
**Given** Z-signal inputs are defined in config_gpio.h
**When** system initializes
**Then** GPIO_X_Z_SIGNAL through GPIO_B_Z_SIGNAL are configured as inputs
**And** interrupt on rising edge enabled
**And** ISR handlers registered for each Z-signal

**Z-Signal Pin Assignments (from architecture):**
| Axis | GPIO | Board Location |
|------|------|----------------|
| X | GPIO_X_Z_SIGNAL | J3-4 (row 4) |
| Y | GPIO_Y_Z_SIGNAL | J3-5 (row 5) |
| Z | GPIO_Z_Z_SIGNAL | J3-6 (row 6) |
| A | GPIO_A_Z_SIGNAL | J3-7 (row 7) |
| B | GPIO_B_Z_SIGNAL | J3-8 (row 8) |

**ISR Registration:**
**Given** Z-signal GPIO is configured
**When** rising edge detected on GPIO_X_Z_SIGNAL
**Then** ISR `onZSignal()` is called for X axis
**And** ISR executes in < 5µs (IRAM_ATTR)

**Prerequisites:** Story 1.3 (config headers), Story 3.6 (servo motor class)

**Technical Notes:**
- Z-signals are direct GPIO (not via I2C expander) for low latency
- All 5 servo axes have Z-signal capability
- Steppers (C, D) and discrete (E) do not have Z-signals
- FR65: Read Z-signal from servo axes

---

### Story 6.2: Z-Signal Drift Detection

**As a** user,
**I want** position drift detected via Z-signal comparison,
**So that** I'm alerted when mechanical slip or missed steps occur.

**Acceptance Criteria:**

**Z-Signal Counting:**
**Given** axis X has been homed with Z-signal
**When** axis moves and crosses Z-signal position
**Then** `z_signal_count_` increments
**And** expected pulse count calculated: `expected = z_signal_count * pulses_per_rev`

**Drift Detection:**
**Given** axis X Z-signal triggers during motion
**When** actual `pulse_count_` differs from `expected`
**Then** drift calculated: `drift = actual - expected`
**And** event published: `EVENT ZSYNC X DETECTED DRIFT:<drift>`

**Deferred Correction (Architecture Constraint):**
**Given** drift is detected during motion
**When** drift is non-zero
**Then** correction is NOT applied immediately (could cause trajectory discontinuity)
**And** drift accumulated in `z_pending_correction_`
**And** correction applied when motion completes (axis IDLE)

**Drift Threshold Alarm:**
**Given** `z_signal.drift_threshold: 100` in config
**When** detected drift exceeds threshold
**Then** event published: `EVENT ALARM X ERR_ZSYNC_DRIFT`
**And** alarm state set (does NOT stop motion - informational)

**Prerequisites:** Story 6.1, Story 3.6

**Technical Notes:**
- Implement `onZSignal()` ISR callback per architecture
- Drift detection is per-revolution accuracy check
- FR66: Detect position drift via Z-signal
- FR69: Report Z-signal events and drift alarms

---

### Story 6.3: Z-Signal Homing (SEEK_ZSIGNAL Phase)

**As a** user,
**I want** homing to use Z-signal for maximum repeatability,
**So that** position reference is accurate to within one encoder count.

**Acceptance Criteria:**

**Extended Homing Sequence (Servo Axes):**
**Given** axis X has `z_signal.enabled: true` in config
**When** I send `HOME X`
**Then** homing sequence includes Z-signal phase:
1. SEEK_LIMIT: Move toward limit at homing velocity
2. LIMIT_HIT: Stop on limit switch
3. BACKOFF: Back off by `homing.backoff` distance
4. **SEEK_ZSIGNAL: Move slowly, wait for Z-signal**
5. SET_HOME: Reset `pulse_count_=0`, `z_signal_count_=0`
6. COMPLETE: Fire `EVENT HOMING X COMPLETE POS:0.000`

**Homing Events:**
```
EVENT HOMING X SEEK_LIMIT POS:0.150
EVENT HOMING X LIMIT_HIT POS:-0.002
EVENT HOMING X BACKOFF POS:0.010
EVENT HOMING X SEEK_ZSIGNAL POS:0.003
EVENT HOMING X COMPLETE POS:0.000
```

**Z-Signal Timeout & Fallback (FR68):**
**Given** `z_signal.fallback: auto` in config (default)
**When** Z-signal not detected within one motor revolution × 3 retries
**Then** fallback to limit-only homing
**And** event: `EVENT HOMING_DEGRADED X`
**And** homing completes with reduced accuracy

**Given** `z_signal.fallback: confirm` in config
**When** Z-signal not found after 3 retries
**Then** homing pauses
**And** event: `EVENT HOMING_PAUSED X`
**And** user must send `HOME X SWITCH` to continue with limit-only

**Given** `z_signal.fallback: fail` in config
**When** Z-signal not found after 3 retries
**Then** homing fails
**And** event: `EVENT HOMING X FAILED ZSIGNAL_TIMEOUT`
**And** axis remains UNHOMED

**Z-Signal Ignored During Backoff:**
**Given** axis is in BACKOFF phase
**When** Z-signal triggers
**Then** Z-signal is ignored (only processed in SEEK_ZSIGNAL phase)

**Prerequisites:** Story 4.13 (basic homing), Story 6.1, Story 6.2

**Technical Notes:**
- Extends Story 4.13 homing with SEEK_ZSIGNAL phase
- Only servo axes (X, Y, Z, A, B) have Z-signal homing
- Steppers use limit-only homing (Story 4.13)
- FR68: Configurable Z-signal fallback

---

### Story 6.4: Deferred Z-Signal Correction

**As a** user,
**I want** accumulated Z-signal drift corrections applied safely,
**So that** position accuracy is maintained without mid-motion jumps.

**Acceptance Criteria:**

**Correction Applied on Motion Complete:**
**Given** `z_pending_correction_` is non-zero
**When** motion completes (axis transitions to IDLE)
**Then** `applyDeferredZCorrection()` is called
**And** `pulse_count_` adjusted by accumulated correction
**And** `current_position_` recalculated from corrected pulse count
**And** event: `EVENT ZSYNC X CORRECTED DRIFT:<correction>`
**And** `z_pending_correction_` reset to 0

**Correction Not Applied During Motion:**
**Given** axis is in MOVING state
**When** Z-signal drift is detected
**Then** drift is accumulated only
**And** no position modification occurs
**And** motion trajectory continues unaffected

**Accumulated Drift Tracking:**
**Given** multiple Z-signals occur during long move
**When** each Z-signal detects drift
**Then** all drifts accumulated in `z_pending_correction_`
**And** total drift reported at motion complete

**CMD_ZSYNC Query:**
**Given** I send `ZSYNC X` (query Z-signal status)
**When** command executes
**Then** response shows:
```
OK X ZSYNC:1 COUNT:15 DRIFT:3 PENDING:0 TOTAL_CORRECTED:47
```
- ZSYNC: Z-signal enabled (1) or disabled (0)
- COUNT: Z-signals seen since home
- DRIFT: Last detected drift (pulses)
- PENDING: Correction awaiting application
- TOTAL_CORRECTED: Cumulative corrections applied

**Prerequisites:** Story 6.2, Story 3.11 (motion completion events)

**Technical Notes:**
- `applyDeferredZCorrection()` called from motion task context (not ISR)
- FR67: Apply deferred Z-signal corrections
- Architecture requires deferred correction to prevent trajectory discontinuities

---

### Story 6.5: InPos Motion Confirmation

**As a** user,
**I want** InPos signals to confirm motion completion,
**So that** I know the servo drive reports position reached.

**Acceptance Criteria:**

**InPos Confirmation After Motion:**
**Given** axis X completes commanded move
**When** InPos_X signal goes active on MCP23017 #1 Port B
**Then** motion confirmed complete
**And** event: `EVENT MOTION_CONFIRMED X`

**InPos Timeout Error:**
**Given** axis X commanded motion completes (pulses generated)
**When** InPos_X does not activate within TIMING_INPOS_TIMEOUT_MS (default 500ms)
**Then** event: `EVENT ERROR X E010` (ERR_INPOS_TIMEOUT)
**And** axis state set to ERROR
**And** indicates servo following error or mechanical issue

**InPos During Motion Monitoring:**
**Given** axis X is moving (continuous motion or position move)
**When** InPos_X goes inactive during motion
**Then** this is expected (servo is seeking position)
**And** no error generated

**Given** axis X is in IDLE state
**When** InPos_X goes inactive unexpectedly
**Then** event: `EVENT WARNING X INPOS_LOST`
**And** indicates possible external disturbance

**Enhanced STAT Response:**
**Given** I send `STAT X`
**Then** response includes InPos confirmation:
```
OK X POS:100.000 EN:1 MOV:0 ERR:0 LIM:00 HOMED:1 INPOS:1 CONFIRMED:1
```
- INPOS: Current InPos signal state
- CONFIRMED: Last motion was InPos-confirmed

**Prerequisites:** Story 4.8 (basic InPos reading), Story 3.11

**Technical Notes:**
- InPos signals on MCP23017 #1 (0x21) Port B (GPB0-GPB4)
- Interrupt on GPIO_MCP1_INTB for InPos changes
- FR70: Use InPos for motion confirmation and following error detection
- This extends basic InPos reading from Story 4.8 with timeout/confirmation logic

---

### Story 6.6: Position Synchronization Events

**As a** user,
**I want** comprehensive position synchronization events,
**So that** the host can monitor position accuracy in real-time.

**Acceptance Criteria:**

**Z-Signal Events:**
```
EVENT ZSYNC X DETECTED DRIFT:5        # Z-signal triggered, drift detected
EVENT ZSYNC X CORRECTED DRIFT:5       # Correction applied
EVENT ALARM X ERR_ZSYNC_DRIFT         # Drift exceeded threshold
EVENT HOMING_DEGRADED X               # Z-signal homing fell back to limit-only
```

**InPos Events:**
```
EVENT MOTION_CONFIRMED X              # InPos confirmed motion complete
EVENT ERROR X E010                    # InPos timeout (ERR_INPOS_TIMEOUT)
EVENT WARNING X INPOS_LOST            # InPos went inactive while idle
```

**Event Subscription:**
**Given** event system from Story 2.7
**When** Z-signal or InPos event occurs
**Then** event is published via EventManager
**And** host receives event via USB CDC

**Event Filtering:**
**Given** I send `EVTFILT ZSYNC 0` (disable Z-sync events)
**When** Z-signal events occur
**Then** events are not sent to host (still processed internally)

**Given** I send `EVTFILT ZSYNC 1` (enable Z-sync events)
**Then** Z-signal events are sent to host

**Prerequisites:** Story 2.7, Story 6.2, Story 6.5

**Technical Notes:**
- Events use EVENT_* constants from config_commands.h
- Host can filter verbose events if not needed
- FR69: Report Z-signal synchronization events

---

## FR Coverage Matrix

| FR | Description | Epic | Story | Status |
|----|-------------|------|-------|--------|
| **Motor Control Capabilities** |
| FR1 | Control 8 independent motor axes | Epic 3 | 3.9 | Planned |
| FR2 | Generate STEP pulses up to 80-100 kHz | Epic 3 | 3.2, 3.3, 3.4 | Planned |
| FR3 | Control 5 servo motors (STEP/DIR) | Epic 3 | 3.6 | Planned |
| FR4 | Control 2 stepper motors with position counting | Epic 3 | 3.7 | Planned |
| FR5 | Control 1 discrete actuator (E axis) | Epic 3 | 3.8 | Planned |
| FR6 | Absolute position moves (CMD_MOVE) | Epic 3 | 3.9 | Planned |
| FR7 | Relative position moves (CMD_MOVR) | Epic 3 | 3.9 | Planned |
| FR8 | Continuous jog movements (CMD_VEL) | Epic 3 | 3.10 | Planned |
| FR9 | Stop axis motion immediately (CMD_STOP) | Epic 3 | 3.10 | Planned |
| FR10 | Enable/disable individual axes (CMD_EN) | Epic 3 | 3.10 | Planned |
| **Safety and Limit Management** |
| FR11 | Monitor 14 limit switches | Epic 4 | 4.2 | Planned |
| FR12 | Hardware E-stop | Epic 4 | 4.4 | Planned |
| FR13 | Automatic stop on limit switch | Epic 4 | 4.3 | Planned |
| FR14 | Configurable brake strategies | Epic 4 | 4.5 | Planned |
| FR15 | Fail-safe brake engagement on power loss | Epic 4 | 4.5 | Planned |
| FR16 | Position loss detection | Epic 4 | 4.6 | Planned |
| FR17 | Prevent motion beyond configured limits | Epic 4 | 4.3 | Planned |
| FR18 | Configure limit switch polarity | Epic 4 | 4.2 | Planned |
| **Communication and Commands** |
| FR19 | USB CDC serial interface | Epic 2 | 2.1 | Planned |
| FR20 | Response within 10ms | Epic 2 | 2.3 | Planned |
| FR21 | Human-readable text commands | Epic 2 | 2.2 | Planned |
| FR22 | Status query commands | Epic 2 | 2.5 | Planned |
| FR23 | Command acknowledgment/error responses | Epic 2 | 2.3 | Planned |
| FR24 | Asynchronous event notifications | Epic 2 | 2.7 | Planned |
| FR25 | Comprehensive status reporting | Epic 2 | 2.5 | Planned |
| FR26 | Command history for debugging | Epic 2 | 2.4 | Planned |
| **Configuration and Calibration** |
| FR27 | Load configuration from YAML via USB | Epic 5 | 5.2 | Planned |
| FR28 | Export configuration to YAML | Epic 5 | 5.3 | Planned |
| FR29 | Runtime adjustable parameters | Epic 5 | 5.4 | Planned |
| FR30 | Persist calibration data across power cycles | Epic 5 | 5.5 | Planned |
| FR31 | Clear axis positions without movement | Epic 5 | 5.6 | Planned |
| FR32 | Custom scaling ratios per axis | Epic 5 | 5.7 | Planned |
| FR33 | Homing sequences | Epic 4 | 4.13 | Planned |
| FR34 | Custom axis aliases | Epic 5 | 5.8 | Planned |
| **I/O and Peripheral Control** |
| FR35 | Read 4 general-purpose digital inputs | Epic 4 | 4.7 | Planned |
| FR36 | Control 8 general-purpose digital outputs | Epic 4 | 4.7 | Planned |
| FR37 | Process servo feedback signals | Epic 4 | 4.8 | Planned |
| FR38 | Detect C axis floating switch | Epic 4 | 4.9 | Planned |
| FR39 | Measure object width and report via event | Epic 4 | 4.9 | Planned |
| FR40 | Query last measured object width | Epic 4 | 4.9 | Planned |
| FR41 | Configurable input debouncing | Epic 4 | 4.7 | Planned |
| FR42 | Read/write I/O using pin names/aliases | Epic 4 | 4.7 | Planned |
| **Status and Monitoring** |
| FR43 | Report real-time position | Epic 3 | 3.10 | Planned |
| FR44 | Report motion status per axis | Epic 3 | 3.11 | Planned |
| FR45 | Track cumulative error counts | Epic 4 | 4.10 | Planned |
| FR46 | Monitor I2C communication health | Epic 4, Epic 5 | 4.11, 5.11 | Planned |
| FR47 | Display positions on OLED | Epic 5 | 5.10 | Planned |
| FR47a | Display E-stop with highest priority | Epic 5 | 5.10 | Planned |
| FR47b | Display errors/events for 2 seconds | Epic 5 | 5.10 | Planned |
| FR47c | OLED on dedicated I2C bus | Epic 5 | 5.9 | Planned |
| FR48 | Events for motion completion | Epic 3 | 3.11 | Planned |
| FR49 | Events for errors and faults | Epic 4 | 4.10 | Planned |
| FR50 | Query individual axis or all axes status | Epic 5 | 5.11 | Planned |
| **Operational Modes** |
| FR51 | Normal operation mode | Epic 2 | 2.6 | Planned |
| FR52 | Configuration mode | Epic 2 | 2.6 | Planned |
| FR53 | Switch between modes | Epic 2 | 2.6 | Planned |
| FR54 | Indicate current mode | Epic 2 | 2.6 | Planned |
| FR55 | Prevent motion in configuration mode | Epic 2 | 2.6 | Planned |
| **Error Handling** |
| FR56 | Detect I2C communication failures | Epic 4 | 4.11 | Planned |
| FR57 | Detailed error messages | Epic 4 | 4.12 | Planned |
| FR58 | Recover from transient errors | Epic 4 | 4.11 | Planned |
| FR59 | Log errors for troubleshooting | Epic 4 | 4.12 | Planned |
| FR60 | Clear error conditions | Epic 4 | 4.10 | Planned |
| FR61 | Prevent unsafe operations in error state | Epic 4 | 4.10 | Planned |
| FR62 | Monitor driver alarm signals (ALARM_INPUT) | Epic 4 | 4.14 | Planned |
| FR63 | Clear driver alarms (CLR command) | Epic 4 | 4.14 | Planned |
| FR64 | Block motion on axes with active alarms | Epic 4 | 4.14 | Planned |
| **Advanced Position Feedback** |
| FR65 | Read Z-signal from servo axes | Epic 6 | 6.1 | Planned |
| FR66 | Detect position drift via Z-signal | Epic 6 | 6.2 | Planned |
| FR67 | Apply deferred Z-signal corrections | Epic 6 | 6.4 | Planned |
| FR68 | Configurable Z-signal fallback during homing | Epic 6 | 6.3 | Planned |
| FR69 | Report Z-signal sync events and drift alarms | Epic 6 | 6.2, 6.6 | Planned |
| FR70 | InPos motion confirmation and following error detection | Epic 6 | 6.5 | Planned |

**Coverage Summary:**
- Total FRs: 70 (including FR47a, FR47b, FR47c as sub-requirements)
- All FRs mapped to stories: ✓
- All FRs have implementation plan: ✓

---

## Summary

### Epic Breakdown Complete

This document decomposes the YaRobot Control Unit PRD into **6 epics** containing **57 user stories** that fully cover all **70 functional requirements**.

### Epic Execution Order

Epics are designed to be executed sequentially with clear dependencies:

```
Epic 1: Foundation & Infrastructure (7 stories)
    ↓ Enables all subsequent development
Epic 2: Communication & Command Interface (7 stories)
    ↓ Provides command framework
Epic 3: Motor Control Core (11 stories)
    ↓ Core product functionality
Epic 4: Safety & I/O Systems (14 stories)
    ↓ Safety and I/O integration
Epic 5: Configuration & Status Display (12 stories)
    ↓ Configuration and user interface
Epic 6: Advanced Position Feedback (6 stories)
    └ Z-signal sync, InPos confirmation, precision homing
```

### Story Dependencies

Stories within each epic generally depend on earlier stories in the same epic. Key cross-epic dependencies:

- **Epic 3** depends on Epic 1 (hardware init), Epic 2 (command dispatch)
- **Epic 4** depends on Epic 3 (motor control for limit-triggered stops)
- **Epic 5** depends on Epic 2 (command framework), Epic 4 (status data)
- **Epic 6** depends on Epic 3 (motor control), Epic 4 (basic homing, InPos reading)

### Implementation Recommendation

1. **Phase 1 (Foundation):** Complete Epic 1 to establish build system and hardware verification
2. **Phase 2 (Core):** Complete Epic 2 and Epic 3 in parallel tracks (communication + motor control)
3. **Phase 3 (Safety):** Complete Epic 4 to add safety systems
4. **Phase 4 (Polish):** Complete Epic 5 for configuration and display
5. **Phase 5 (Precision):** Complete Epic 6 for advanced position feedback (optional for basic operation)

### Command Summary

All commands use CMD_* constants defined in `config_commands.h`:

| Category | Commands |
|----------|----------|
| Motion | CMD_MOVE, CMD_MOVR, CMD_VEL, CMD_STOP, CMD_HOME |
| Control | CMD_EN, CMD_BRAKE, CMD_CLR, CMD_RST |
| Status | CMD_STAT, CMD_POS, CMD_INFO, CMD_DIAG, CMD_LIM |
| Config | CMD_CFG*, CMD_SET, CMD_SAVE, CMD_LOAD, CMD_RATIO, CMD_ALIAS |
| I/O | CMD_DIN, CMD_DOUT, CMD_WIDTH, CMD_SERVOSTAT |
| Position Sync | CMD_ZSYNC, CMD_EVTFILT |
| System | CMD_ECHO, CMD_MODE, CMD_ERRCNT, CMD_CLRERR, CMD_LOG, CMD_I2C |

### Technical Constants Reference

All implementation uses constants from config headers - no magic numbers:
- GPIO assignments: `config_gpio.h`
- Command strings: `config_commands.h` (CMD_*, RESP_*, ERR_*, MSG_*)
- Timing values: `config_timing.h`
- Peripheral config: `config_peripherals.h`, `config_i2c.h`, `config_sr.h`
- Limits and buffers: `config_limits.h`
- Default values: `config_defaults.h`

---

_For implementation: Use the `create-story` workflow to generate individual story implementation plans from this epic breakdown._

_This document will be updated as epics are decomposed into stories._
