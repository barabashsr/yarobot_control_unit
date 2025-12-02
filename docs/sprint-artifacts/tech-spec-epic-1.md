# Epic Technical Specification: Foundation & Infrastructure

Date: 2025-12-02
Author: Sergey
Epic ID: 1
Status: Draft

---

## Overview

Epic 1 establishes the foundational infrastructure for the YaRobot Control Unit firmware. This epic creates the ESP-IDF project structure, configuration header framework, HAL layer stubs, and FreeRTOS task architecture that all subsequent epics will build upon. It does not implement motor control functionality directly, but rather ensures that all hardware peripherals are verified working, the build system is reliable, and the codebase follows the architecture's strict "no magic numbers" mandate.

This foundation enables a clean development workflow where subsequent epics can focus on feature implementation rather than infrastructure concerns. By the end of Epic 1, developers can build, flash, and monitor a bootable firmware that demonstrates proper ESP32-S3 configuration, I2C communication with expanders, SPI communication with shift registers, and the complete FreeRTOS task structure.

## Objectives and Scope

**In Scope:**
- ESP-IDF v5.4 project initialization for ESP32-S3 N16R8 (16MB Flash, 8MB PSRAM)
- Complete component directory structure per architecture specification
- All 11 configuration headers with compile-time constants
- HAL layer interfaces for GPIO, I2C, and SPI with stub implementations
- FreeRTOS task framework with dual-core separation and priority assignments
- Hardware peripheral verification (I2C buses, SPI shift registers, direct GPIO)
- Basic CMD_TEST/CMD_DIAG command for hardware diagnostics
- Build system with -Werror and CI-ready verification
- README.md with build instructions

**Out of Scope:**
- Motor control implementation (Epic 3)
- USB CDC command parsing beyond TEST/DIAG (Epic 2)
- Full driver implementations (subsequent epics)
- Motion profile generation (Epic 3)
- Safety monitoring logic (Epic 4)
- Configuration persistence (Epic 5)

## System Architecture Alignment

**Component References:**
- `main/` - Application entry point with app_main()
- `components/hal/` - Hardware abstraction layer (gpio_hal, i2c_hal, spi_hal)
- `components/drivers/` - Device driver stubs (mcp23017, tpic6b595, oled)
- `components/config/` - All configuration headers
- `components/control/` - Task framework stubs
- `components/interface/` - USB CDC basic functionality

**Architecture Constraints Applied:**

> **MANDATORY: Header-Only Configuration Requirement**
>
> Every configurable value MUST be defined in a header file. This is non-negotiable.
>
> - GPIO pins → `config_gpio.h` (never GPIO_NUM_2, always GPIO_X_STEP)
> - Timing values → `config_timing.h` (never 20, always TIMING_DIR_SETUP_US)
> - Buffer sizes → `config_limits.h` (never 256, always LIMIT_CMD_MAX_LENGTH)
> - Command strings → `config_commands.h` (never "MOVE", always CMD_MOVE)
> - Error codes → `config_commands.h` (never "E002", always ERR_INVALID_AXIS)
> - I2C addresses → `config_i2c.h` (never 0x20, always I2C_ADDR_MCP23017_0)
> - YAML keys → `config_yaml_schema.h` (never "velocity", always YAML_KEY_VELOCITY)
>
> This requirement MUST be validated in code review for ALL stories in this epic.

**Dual-Core Separation:**
- Core 0: Communication, safety, coordination tasks
- Core 1: Motion control tasks (time-critical)

**Decision References:**
- ADR-001: ESP32-S3-DevKitC-1 N16R8 platform
- ADR-002: Layered component architecture (HAL → Drivers → Control → Interface)
- ADR-003: TPIC6B595N shift registers for 24V logic
- ADR-004: Separate I2C bus for OLED display (I2C1 isolated from I2C0)
- Behavioral Decision #1: Motors disabled by default at power-on
- Behavioral Decision #2: Send EVENT BOOT notification at startup

## Detailed Design

### Services and Modules

| Module | Responsibility | Inputs | Outputs | Owner |
|--------|---------------|--------|---------|-------|
| `main/main.cpp` | Application entry, task creation | - | Boot sequence, tasks | Epic 1 |
| `hal/gpio_hal` | GPIO abstraction | Pin configs | GPIO operations | Story 1.4 |
| `hal/i2c_hal` | I2C bus abstraction | Port, address, data | I2C transactions | Story 1.4 |
| `hal/spi_hal` | SPI bus abstraction | Host, data | SPI transfers | Story 1.4 |
| `config/*.h` | Compile-time constants | - | #define values | Story 1.3 |
| `control/cmd_executor` | Basic command handling | CMD_TEST input | Diagnostic output | Story 1.6 |

### Data Models and Contracts

**Configuration Header Organization:**

```
components/config/include/
├── config.h              # Master include, FIRMWARE_VERSION, FEATURE_FLAGS
├── config_gpio.h         # GPIO_X_STEP, GPIO_I2C_SDA, etc.
├── config_peripherals.h  # RMT_CHANNEL_X, MCPWM_UNIT_Y, SPI_HOST_SR
├── config_timing.h       # TIMING_DIR_SETUP_US, TIMING_BRAKE_ENGAGE_MS
├── config_limits.h       # LIMIT_CMD_MAX_LENGTH, STACK_MOTION_TASK
├── config_commands.h     # CMD_MOVE, RESP_OK, ERR_INVALID_AXIS
├── config_i2c.h          # I2C_ADDR_MCP23017_0, MCP0_X_LIMIT_MIN
├── config_sr.h           # SR_X_DIR, SR_SET_BIT(), SR_SAFE_STATE
├── config_defaults.h     # DEFAULT_MAX_VELOCITY, E_AXIS_LIMIT_MAX
├── config_oled.h         # OLED_WIDTH, OLED_UPDATE_HZ
└── config_yaml_schema.h  # YAML_KEY_VELOCITY, NVS_KEY_AXIS_CONFIG
```

**sdkconfig.defaults:**

```
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_CONSOLE_USB_CDC=y
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
```

**partitions.csv:**

```csv
# Name,   Type, SubType, Offset,  Size,    Flags
nvs,      data, nvs,     0x9000,  0x6000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, 0x10000, 0x300000,
storage,  data, spiffs,  ,        0x100000,
```

### APIs and Interfaces

**HAL GPIO Interface (gpio_hal.h):**

```c
esp_err_t gpio_hal_init(void);
esp_err_t gpio_hal_set_direction(gpio_num_t pin, gpio_mode_t mode);
esp_err_t gpio_hal_set_level(gpio_num_t pin, uint32_t level);
int gpio_hal_get_level(gpio_num_t pin);
esp_err_t gpio_hal_set_interrupt(gpio_num_t pin, gpio_int_type_t type,
                                  gpio_isr_t handler, void* arg);
```

**HAL I2C Interface (i2c_hal.h):**

```c
esp_err_t i2c_hal_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
                       uint32_t freq_hz);
esp_err_t i2c_hal_write(i2c_port_t port, uint8_t addr,
                        const uint8_t* data, size_t len);
esp_err_t i2c_hal_read(i2c_port_t port, uint8_t addr,
                       uint8_t* data, size_t len);
esp_err_t i2c_hal_write_read(i2c_port_t port, uint8_t addr,
                             const uint8_t* wr, size_t wr_len,
                             uint8_t* rd, size_t rd_len);
```

**HAL SPI Interface (spi_hal.h):**

```c
esp_err_t spi_hal_init(spi_host_device_t host, gpio_num_t mosi,
                       gpio_num_t sclk, gpio_num_t cs);
esp_err_t spi_hal_transfer(spi_host_device_t host, const uint8_t* tx,
                           uint8_t* rx, size_t len);
```

**Basic Command Interface (Story 1.6):**

| Command | Response | Description |
|---------|----------|-------------|
| `TEST I2C` | `OK I2C0:0x20,0x21 I2C1:0x3C` | I2C bus scan results |
| `TEST SR` | `OK SR:PASS` or `ERROR SR:FAIL` | Shift register test |
| `TEST GPIO` | `OK GPIO:ESTOP=1,STEP_X=0...` | GPIO pin states |
| `DIAG` | Same as TEST | Alias for diagnostic |

### Workflows and Sequencing

**Boot Sequence:**

```
1. ESP32-S3 boot ROM execution
2. Second-stage bootloader loads app from factory partition
3. app_main() called
   │
   ├── 4. Initialize HAL layers (GPIO, I2C0, I2C1, SPI2)
   │
   ├── 5. Verify hardware (I2C scan, SR test)
   │       If fail → Log error, continue with degraded mode
   │
   ├── 6. Create FreeRTOS tasks (pinned to cores)
   │       Core 0: safety_monitor, usb_rx, usb_tx, cmd_executor, i2c_monitor
   │       Core 1: motion_X through motion_E, display, idle_monitor
   │
   ├── 7. Initialize USB CDC
   │
   └── 8. Send EVENT BOOT notification
          Format: "EVENT BOOT V1.0.0 AXES:8 STATE:IDLE"
```

**Task Creation Sequence:**

```
app_main()
├── xTaskCreatePinnedToCore(safety_monitor_task, "safety", STACK_SAFETY_TASK, NULL, 24, NULL, 0)
├── xTaskCreatePinnedToCore(usb_rx_task, "usb_rx", STACK_USB_RX_TASK, NULL, 10, NULL, 0)
├── xTaskCreatePinnedToCore(usb_tx_task, "usb_tx", STACK_USB_TX_TASK, NULL, 10, NULL, 0)
├── xTaskCreatePinnedToCore(cmd_executor_task, "cmd_exec", STACK_CMD_EXECUTOR_TASK, NULL, 12, NULL, 0)
├── xTaskCreatePinnedToCore(i2c_monitor_task, "i2c_mon", STACK_I2C_MONITOR_TASK, NULL, 8, NULL, 0)
├── for axis in X,Y,Z,A,B,C,D,E:
│   └── xTaskCreatePinnedToCore(motion_task, "motion_X", STACK_MOTION_TASK, &axis, 15, NULL, 1)
├── xTaskCreatePinnedToCore(display_task, "display", STACK_DISPLAY_TASK, NULL, 5, NULL, 1)
└── xTaskCreatePinnedToCore(idle_monitor_task, "idle_mon", STACK_IDLE_MONITOR_TASK, NULL, 4, NULL, 0)
```

## Non-Functional Requirements

### Performance

- **NFR-E1-1:** Build time < 60 seconds for incremental builds
- **NFR-E1-2:** Boot to EVENT BOOT notification < 1 second
- **NFR-E1-3:** I2C scan completes within 100ms
- **NFR-E1-4:** Shift register write cycle < 10µs

### Security

- No external network interfaces (USB only)
- No authentication required (internal team tool)
- No firmware OTA updates in MVP

### Reliability/Availability

- **NFR-E1-5:** All tasks must start successfully or log clear error
- **NFR-E1-6:** HAL stubs return ESP_OK without side effects
- **NFR-E1-7:** Hardware verification failures logged but don't prevent boot

### Observability

- **NFR-E1-8:** Each task logs creation with `ESP_LOGI(TAG, "Task %s started on core %d")`
- **NFR-E1-9:** I2C scan results logged at INFO level
- **NFR-E1-10:** Shift register test results logged at INFO level
- **NFR-E1-11:** Memory usage reported via `idf.py size`

## Dependencies and Integrations

**ESP-IDF Dependencies:**

| Component | Version | Purpose |
|-----------|---------|---------|
| esp-idf | 5.4 (LTS) | Framework, FreeRTOS SMP, drivers |
| espressif/mcp23017 | ^0.1.1 | I2C GPIO expander component |

**Development Environment Setup:**

Before running any `idf.py` commands, the terminal must be sourced for ESP-IDF:

```bash
# Source ESP-IDF environment (required before each session)
get_idf

# Verify environment is active
idf.py --version
```

> **Note:** The `get_idf` command is an alias that sources the ESP-IDF export script. If not configured, use the full path:
> ```bash
> . /Users/sergeybarabash/robo/esp/v5.4/esp-idf/export.sh
> ```

**idf_component.yml:**

```yaml
dependencies:
  espressif/mcp23017: "^0.1.1"
```

**Hardware Dependencies:**

- ESP32-S3-DevKitC-1 N16R8 development board
- 2x MCP23017 I2C GPIO expanders (addresses 0x20, 0x21)
- SSD1306 128x64 OLED display (address 0x3C)
- 5x TPIC6B595N shift registers (40-bit chain via SPI2)
- USB-C cable for programming and communication

## Acceptance Criteria (Authoritative)

1. **AC1:** `idf.py build` completes without errors or warnings (with -Werror)
2. **AC2:** `idf.py flash` successfully programs ESP32-S3 N16R8
3. **AC3:** `idf.py monitor` shows all 16 tasks created and running
4. **AC4:** sdkconfig contains all required settings (16MB flash, octal PSRAM, 1000Hz tick, USB CDC, 240MHz)
5. **AC5:** All 11 configuration headers exist and compile
6. **AC6:** `static_assert(LIMIT_NUM_SERVOS + LIMIT_NUM_STEPPERS + LIMIT_NUM_DISCRETE == LIMIT_NUM_AXES)` passes
7. **AC7:** Component directory structure matches architecture specification
8. **AC8:** HAL interfaces defined with stub implementations returning ESP_OK
9. **AC9:** I2C0 scan detects devices at 0x20 and 0x21
10. **AC10:** I2C1 scan detects device at 0x3C (OLED)
11. **AC11:** Shift register test passes (pattern write/verify)
12. **AC12:** CMD_TEST command returns hardware diagnostic results via USB
13. **AC13:** EVENT BOOT notification sent on startup
14. **AC14:** README.md documents build prerequisites and commands
15. **AC15:** No magic numbers in any source file (enforced by code review)

## Traceability Mapping

| AC | Spec Section | Component(s) | Test Approach |
|----|--------------|--------------|---------------|
| AC1 | Story 1.7 | All | `idf.py build` |
| AC2 | Story 1.1, 1.7 | main, partitions.csv | `idf.py flash` |
| AC3 | Story 1.5 | main, control | `idf.py monitor`, task list |
| AC4 | Story 1.1 | sdkconfig.defaults | Grep sdkconfig |
| AC5 | Story 1.3 | config/*.h | Compilation |
| AC6 | Story 1.3 | config_limits.h | static_assert |
| AC7 | Story 1.2 | components/* | Directory inspection |
| AC8 | Story 1.4 | hal/* | Compilation, stub returns |
| AC9 | Story 1.6 | hal/i2c_hal | I2C scan test |
| AC10 | Story 1.6 | hal/i2c_hal | I2C scan test |
| AC11 | Story 1.6 | hal/spi_hal | SR pattern test |
| AC12 | Story 1.6 | control/cmd_executor | USB terminal test |
| AC13 | Story 1.5 | main | Monitor boot output |
| AC14 | Story 1.7 | README.md | Documentation review |
| AC15 | All stories | All | Code review checklist |

## Risks, Assumptions, Open Questions

**Risks:**

| ID | Risk | Mitigation |
|----|------|------------|
| R1 | MCP23017 component version incompatibility | Pin to ^0.1.1, verify before implementation |
| R2 | PSRAM initialization failure | Test on actual N16R8 board early in Story 1.1 |
| R3 | I2C bus conflicts between I2C0/I2C1 | Architecture mandates separate buses; verify isolation |
| R4 | Shift register timing issues | Test with oscilloscope in Story 1.6 |

**Assumptions:**

| ID | Assumption | Validation |
|----|------------|------------|
| A1 | ESP-IDF 5.4 is stable LTS release | Check Espressif release notes |
| A2 | DevKitC-1 N16R8 has required GPIO pins available | Pin audit in architecture |
| A3 | 400kHz I2C is sufficient for MCP23017 polling | Verify in Story 1.6 |
| A4 | USB CDC works reliably with TinyUSB | Test in Story 1.1 |

**Open Questions:**

| ID | Question | Decision Needed By |
|----|----------|-------------------|
| Q1 | Should we use C or C++ for HAL implementations? | Story 1.4 start |
| Q2 | Memory budget for task stacks - current estimates sufficient? | Story 1.5 completion |

## Test Strategy Summary

**Unit Testing:**
- HAL stubs are inherently testable (return ESP_OK)
- Configuration headers validated via static_assert
- No complex logic to unit test in Epic 1

**Integration Testing:**
- I2C bus scan verifies MCP23017 connectivity
- SPI shift register pattern test verifies chain integrity
- USB CDC loopback (send TEST, verify response)
- Task creation verified via monitor output

**Hardware Verification (Story 1.6):**
- I2C0: Read MCP23017 registers, verify expected values
- I2C1: Initialize OLED, display "BOOT OK"
- SPI2: Write patterns 0xAAAAAA, 0x555555, verify latch
- GPIO: Toggle STEP pins, verify with oscilloscope/logic analyzer

**Build Verification (Story 1.7):**
- Clean build with -Werror
- Flash to device, verify boot
- Memory usage via `idf.py size`
- PSRAM detection in boot log

**Code Review Checklist:**
- [ ] No magic numbers - all values from config headers
- [ ] Header guards present on all .h files
- [ ] ESP_LOGI/LOGW/LOGE used for logging (not printf)
- [ ] xTaskCreatePinnedToCore used with correct core affinity
- [ ] Stack sizes from STACK_* constants
