# Story 1.3: Configuration Header Framework

Status: done

## Story

As a **developer**,
I want **all configuration headers created with compile-time constants and Doxygen documentation**,
so that **no magic numbers appear in code, hardware assignments are centralized, and the codebase is self-documenting**.

## Acceptance Criteria

1. **AC1:** Given the component structure exists, when I include `config.h` in any source file, then I have access to all configuration constants
2. **AC2:** All 11 headers exist in `firmware/components/config/include/`:
   - `config.h` (master include with FIRMWARE_VERSION, FEATURE_FLAGS)
   - `config_gpio.h` (all GPIO assignments per architecture)
   - `config_peripherals.h` (RMT, MCPWM, PCNT, LEDC, SPI assignments)
   - `config_timing.h` (all timing constants in ms/µs)
   - `config_limits.h` (buffer sizes, queue depths, axis counts, stack sizes)
   - `config_commands.h` (CMD_* command strings, RESP_* response prefixes, ERR_* error codes)
   - `config_i2c.h` (I2C addresses, MCP23017 pin mappings)
   - `config_sr.h` (shift register bit positions, helper macros)
   - `config_defaults.h` (default axis parameters in SI units)
   - `config_oled.h` (OLED display configuration)
   - `config_yaml_schema.h` (YAML keys, NVS mappings, validation macros)
3. **AC3:** All values match the architecture document exactly
4. **AC4:** `static_assert(LIMIT_NUM_SERVOS + LIMIT_NUM_STEPPERS + LIMIT_NUM_DISCRETE == LIMIT_NUM_AXES)` compiles successfully
5. **AC5:** Project builds successfully with `cd firmware && idf.py build`
6. **AC6:** All headers use Doxygen-style comments (`/** */`, `@brief`, `@note`, `@defgroup`)

## Tasks / Subtasks

- [x] **Task 1: Create config_gpio.h** (AC: 2, 3, 6)
  - [x] Define GPIO_X_STEP through GPIO_D_STEP (7 pins)
  - [x] Define GPIO_X_Z_SIGNAL through GPIO_B_Z_SIGNAL (5 pins)
  - [x] Define GPIO_SR_* (OE, CS, MOSI, SCLK - 4 pins)
  - [x] Define GPIO_I2C_SCL, GPIO_I2C_SDA (2 pins)
  - [x] Define GPIO_MCP0_INTA/B, GPIO_MCP1_INTA/B (4 pins)
  - [x] Define GPIO_E_STOP (1 pin)
  - [x] Define GPIO_OLED_SDA, GPIO_OLED_SCL (2 pins)
  - [x] Define GPIO_USB_DN, GPIO_USB_DP (2 pins)
  - [x] Add Doxygen file header and group documentation

- [x] **Task 2: Create config_peripherals.h** (AC: 2, 3, 6)
  - [x] Define RMT_CHANNEL_X, RMT_CHANNEL_Z, RMT_CHANNEL_A, RMT_CHANNEL_B
  - [x] Define MCPWM_GROUP_ID, MCPWM_TIMER_Y, MCPWM_TIMER_C
  - [x] Define PCNT_UNIT_Y, PCNT_UNIT_C
  - [x] Define LEDC_TIMER, LEDC_CHANNEL_D, LEDC_MODE
  - [x] Define SPI_HOST_SR
  - [x] Add Doxygen documentation

- [x] **Task 3: Create config_timing.h** (AC: 2, 3, 6)
  - [x] Define motion timing: TIMING_DIR_SETUP_US, TIMING_ENABLE_DELAY_US, TIMING_BRAKE_*
  - [x] Define communication timing: TIMING_CMD_RESPONSE_MS, TIMING_USB_RX_TIMEOUT_MS
  - [x] Define I2C timing: TIMING_I2C_POLL_MS, TIMING_I2C_TIMEOUT_MS, TIMING_I2C_RETRY_COUNT
  - [x] Define safety timing: TIMING_ESTOP_DEBOUNCE_MS, TIMING_LIMIT_DEBOUNCE_MS, TIMING_IDLE_TIMEOUT_S
  - [x] Define task periods: PERIOD_I2C_MONITOR_MS, PERIOD_DISPLAY_UPDATE_MS, PERIOD_IDLE_CHECK_MS
  - [x] Add Doxygen documentation

- [x] **Task 4: Create config_limits.h** (AC: 2, 3, 4, 6)
  - [x] Define buffer sizes: LIMIT_CMD_MAX_LENGTH, LIMIT_RESPONSE_MAX_LENGTH, LIMIT_YAML_BUFFER_SIZE
  - [x] Define queue depths: LIMIT_COMMAND_QUEUE_DEPTH, LIMIT_RESPONSE_QUEUE_DEPTH, etc.
  - [x] Define axis limits: LIMIT_NUM_AXES (8), LIMIT_NUM_SERVOS (5), LIMIT_NUM_STEPPERS (2), LIMIT_NUM_DISCRETE (1)
  - [x] Define motion limits: LIMIT_MAX_PULSE_FREQ_HZ, LIMIT_MIN_PULSE_FREQ_HZ
  - [x] Define stack sizes: STACK_SAFETY_TASK, STACK_USB_RX_TASK, etc.
  - [x] Add static_assert for axis count validation
  - [x] Add Doxygen documentation

- [x] **Task 5: Create config_commands.h** (AC: 2, 3, 6)
  - [x] Define command strings: CMD_MOVE, CMD_MOVR, CMD_VEL, CMD_STOP, etc.
  - [x] Define response prefixes: RESP_OK, RESP_ERROR, RESP_EVENT
  - [x] Define error codes: ERR_INVALID_COMMAND through ERR_CONFIG_BUFFER_OVERFLOW
  - [x] Define error messages: MSG_* corresponding to each ERR_* code
  - [x] Define event types: EVT_BOOT, EVT_MOTION_COMPLETE, EVT_LIMIT_TRIGGERED, etc.
  - [x] Define EndSwitchMode enum
  - [x] Add Doxygen documentation

- [x] **Task 6: Create config_i2c.h** (AC: 2, 3, 6)
  - [x] Define I2C bus config: I2C_PORT, I2C_FREQ_HZ
  - [x] Define MCP23017 addresses: I2C_ADDR_MCP23017_0 (0x20), I2C_ADDR_MCP23017_1 (0x21)
  - [x] Define MCP0 pin mappings: MCP0_X_LIMIT_MIN through MCP0_E_LIMIT_MAX (16 pins)
  - [x] Define MCP1 pin mappings: MCP1_X_ALARM_INPUT through MCP1_GP_IN_3 (16 pins)
  - [x] Add Doxygen documentation

- [x] **Task 7: Create config_sr.h** (AC: 2, 3, 6)
  - [x] Define per-axis bit positions: SR_X_DIR, SR_X_EN, SR_X_BRAKE, SR_X_ALARM_CLR (repeat for Y-E)
  - [x] Define GP outputs: SR_GP_OUT_0 through SR_GP_OUT_7
  - [x] Define helper macros: SR_DIR_BIT(), SR_EN_BIT(), SR_BRAKE_BIT(), SR_ALARM_CLR_BIT()
  - [x] Define bit manipulation macros: SR_SET_BIT(), SR_CLR_BIT(), SR_GET_BIT()
  - [x] Define SR_SAFE_STATE (0x0000000000ULL)
  - [x] Add Doxygen documentation

- [x] **Task 8: Create config_defaults.h** (AC: 2, 3, 6)
  - [x] Define math constants: CONST_PI, CONST_2PI, CONST_DEG_TO_RAD, CONST_RAD_TO_DEG
  - [x] Define linear axis defaults: DEFAULT_PULSES_PER_UNIT, DEFAULT_MAX_VELOCITY, etc.
  - [x] Define rotary axis defaults: DEFAULT_ROTARY_PULSES_PER_UNIT, DEFAULT_ROTARY_MAX_VEL, etc.
  - [x] Define E-axis constants: E_AXIS_PULSES_PER_UNIT, E_AXIS_LIMIT_MIN/MAX
  - [x] Add Doxygen documentation

- [x] **Task 9: Create config_oled.h** (AC: 2, 3, 6)
  - [x] Define OLED I2C config: I2C_OLED_PORT, I2C_OLED_FREQ_HZ
  - [x] Define display parameters: OLED_ADDRESS, OLED_WIDTH, OLED_HEIGHT, OLED_UPDATE_HZ
  - [x] Define layout constants: OLED_FONT_WIDTH, OLED_CHARS_PER_LINE, OLED_NUM_LINES
  - [x] Define timing: OLED_EVENT_DISPLAY_MS
  - [x] Add Doxygen documentation

- [x] **Task 10: Create config_yaml_schema.h** (AC: 2, 3, 6)
  - [x] Define root keys: YAML_KEY_VERSION, YAML_KEY_AXES, YAML_KEY_SYSTEM
  - [x] Define axis names: YAML_AXIS_X through YAML_AXIS_E + YAML_AXIS_NAMES array
  - [x] Define per-axis parameter keys: YAML_KEY_ALIAS, YAML_KEY_TYPE, YAML_KEY_LIMITS, etc.
  - [x] Define system parameter keys: YAML_KEY_DEBOUNCE_MS, YAML_KEY_IDLE_TIMEOUT_S
  - [x] Define NVS key mappings: NVS_NAMESPACE, NVS_KEY_PREFIX_AXIS, NVS_KEY_SUFFIX_*
  - [x] Define validation macros: YAML_VALIDATE_VELOCITY, YAML_VALIDATE_AXIS, etc.
  - [x] Add Doxygen documentation

- [x] **Task 11: Create config.h master include** (AC: 1, 2, 6)
  - [x] Include all 10 config headers
  - [x] Define FIRMWARE_NAME, FIRMWARE_VERSION_* constants
  - [x] Define feature flags: FEATURE_OLED_DISPLAY, FEATURE_Z_SIGNAL, FEATURE_STREAMING
  - [x] Add Doxygen file header

- [x] **Task 12: Update config component CMakeLists.txt** (AC: 5)
  - [x] Add INCLUDE_DIRS "include" if not already present
  - [x] Verify component builds with new headers

- [x] **Task 13: Verify build** (AC: 4, 5)
  - [x] Run `cd firmware && idf.py build`
  - [x] Confirm static_assert passes
  - [x] Confirm no build errors or warnings

## Dev Notes

### Architecture Constraints

> **MANDATORY: No Magic Numbers**
>
> Every configurable value MUST be defined in a header file. This is non-negotiable.
>
> - GPIO pins → `config_gpio.h` (never GPIO_NUM_2, always GPIO_X_STEP)
> - Timing values → `config_timing.h` (never 20, always TIMING_DIR_SETUP_US)
> - Buffer sizes → `config_limits.h` (never 256, always LIMIT_CMD_MAX_LENGTH)
> - Command strings → `config_commands.h` (never "MOVE", always CMD_MOVE)
> - Error codes → `config_commands.h` (never "E002", always ERR_INVALID_AXIS)
> - I2C addresses → `config_i2c.h` (never 0x20, always I2C_ADDR_MCP23017_0)

> **MANDATORY: Doxygen Documentation**
>
> All header files MUST use Doxygen-style comments for documentation:
>
> - File header with `@file`, `@brief`, `@author`, `@date`
> - Group definitions with `@defgroup` and `@{` ... `@}`
> - Each constant documented with `/** @brief ... */`
> - Use `@note` for important implementation details
> - Use `@warning` for critical constraints

### Doxygen Comment Template

```c
/**
 * @file config_gpio.h
 * @brief GPIO pin assignments for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note All GPIO assignments match the ESP32-S3-DevKitC-1 N16R8 board layout.
 *       See docs/gpio-assignment.md for physical pin mapping.
 */

#ifndef CONFIG_GPIO_H
#define CONFIG_GPIO_H

/**
 * @defgroup config_gpio GPIO Pin Assignments
 * @brief Hardware GPIO pin definitions
 * @{
 */

/**
 * @defgroup gpio_step Motor STEP Pulse Outputs
 * @brief Step pulse output pins for each motor axis
 * @{
 */

/** @brief X-axis servo step output (RMT CH0) - J1 pin 4 */
#define GPIO_X_STEP         GPIO_NUM_4

/** @} */ // end gpio_step

/** @} */ // end config_gpio

#endif // CONFIG_GPIO_H
```

### Header Dependencies

```
config.h (master)
├── config_gpio.h         (standalone)
├── config_peripherals.h  (standalone)
├── config_timing.h       (standalone)
├── config_limits.h       (standalone)
├── config_commands.h     (standalone)
├── config_i2c.h          (standalone)
├── config_sr.h           (standalone)
├── config_defaults.h     (standalone)
├── config_oled.h         (references config_gpio.h constants in comments only)
└── config_yaml_schema.h  (includes config_limits.h, config_defaults.h)
```

### Configuration Domains

| Header | Domain | Example Constants |
|--------|--------|-------------------|
| `config_gpio.h` | Hardware pins | `GPIO_X_STEP`, `GPIO_I2C_SDA` |
| `config_peripherals.h` | ESP32 peripherals | `RMT_CHANNEL_X`, `SPI_HOST_SR` |
| `config_timing.h` | Timing values | `TIMING_DIR_SETUP_US`, `PERIOD_I2C_MONITOR_MS` |
| `config_limits.h` | Sizes/counts | `LIMIT_NUM_AXES`, `STACK_MOTION_TASK` |
| `config_commands.h` | Protocol | `CMD_MOVE`, `ERR_INVALID_AXIS`, `EVT_BOOT` |
| `config_i2c.h` | I2C devices | `I2C_ADDR_MCP23017_0`, `MCP0_X_LIMIT_MIN` |
| `config_sr.h` | Shift registers | `SR_X_DIR`, `SR_SET_BIT()` |
| `config_defaults.h` | Default params | `DEFAULT_MAX_VELOCITY`, `E_AXIS_LIMIT_MAX` |
| `config_oled.h` | Display | `OLED_ADDRESS`, `OLED_WIDTH` |
| `config_yaml_schema.h` | Config schema | `YAML_KEY_VELOCITY`, `NVS_KEY_PREFIX_AXIS` |

### Compile-Time Validation

The headers enable static assertions that catch configuration errors at build time:

```c
// In config_limits.h
static_assert(LIMIT_NUM_SERVOS + LIMIT_NUM_STEPPERS + LIMIT_NUM_DISCRETE == LIMIT_NUM_AXES,
              "Axis count mismatch: servo + stepper + discrete must equal total axes");
```

### Learnings from Previous Story

**From Story 1-2 (Status: done)**

- Component directory structure successfully created with 16 components
- All CMakeLists.txt files use minimal `idf_component_register(INCLUDE_DIRS "include")` template
- Build verified successful with ESP-IDF 5.4
- The `config/` component already exists at `firmware/components/config/` with include/ directory

[Source: docs/sprint-artifacts/1-2-component-directory-structure.md#Dev-Agent-Record]

### References

- [Source: docs/architecture.md#Configuration-System] - Complete header specifications
- [Source: docs/architecture.md#config_gpio.h] - GPIO pin assignments
- [Source: docs/architecture.md#config_commands.h] - Command and error code definitions
- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Data-Models-and-Contracts] - Header organization
- [Source: docs/gpio-assignment.md] - Physical GPIO pin mapping

---

## Dev Agent Record

### Context Reference

- `docs/sprint-artifacts/1-3-configuration-header-framework.context.xml`

### Agent Model Used

claude-opus-4-5-20251101

### Debug Log References

### Completion Notes List

1. All 11 configuration headers created with full Doxygen documentation (@file, @brief, @defgroup, @note)
2. Values match architecture document: GPIO assignments per gpio-assignment.md, 8 axes (5 servo, 2 stepper, 1 discrete)
3. static_assert validates LIMIT_NUM_SERVOS(5) + LIMIT_NUM_STEPPERS(2) + LIMIT_NUM_DISCRETE(1) == LIMIT_NUM_AXES(8)
4. Build verified successful with ESP-IDF 5.4 - no errors or warnings
5. SI units used throughout per ROS2 REP-103 (meters, radians, seconds)
6. Header dependency chain: config_yaml_schema.h includes config_limits.h and config_defaults.h; all others standalone
7. 40-bit shift register chain with SR_SET_BIT/SR_CLR_BIT/SR_GET_BIT macros for safe bit manipulation
8. MCP23017 mappings: MCP0 for limit switches (16 pins), MCP1 for alarm inputs + InPos + GP inputs (16 pins)

### File List

**Created:**
- `firmware/components/config/include/config.h` - Master include with version and feature flags
- `firmware/components/config/include/config_gpio.h` - GPIO pin assignments (27 pins)
- `firmware/components/config/include/config_peripherals.h` - RMT/MCPWM/PCNT/LEDC/SPI assignments
- `firmware/components/config/include/config_timing.h` - Timing constants (motion, I2C, safety, task periods)
- `firmware/components/config/include/config_limits.h` - Buffer sizes, queue depths, axis counts with static_assert
- `firmware/components/config/include/config_commands.h` - Command strings, error codes, event types, EndSwitchMode enum
- `firmware/components/config/include/config_i2c.h` - I2C addresses and MCP23017 pin mappings (32 pins)
- `firmware/components/config/include/config_sr.h` - Shift register bit positions (40 bits) and helper macros
- `firmware/components/config/include/config_defaults.h` - Default axis parameters in SI units
- `firmware/components/config/include/config_oled.h` - OLED display configuration
- `firmware/components/config/include/config_yaml_schema.h` - YAML keys, NVS mappings, validation macros
- `firmware/components/config/CMakeLists.txt` - Component registration with INCLUDE_DIRS

**Modified:**
- `firmware/main/CMakeLists.txt` - Added `REQUIRES config` dependency
- `firmware/main/yarobot_control_unit.cpp` - Added config.h include and verification logs

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-03 | SM Agent (Bob) | Initial story draft with Doxygen requirement |
| 2025-12-03 | Dev Agent (Amelia) | Implemented all 13 tasks, created 11 headers + CMakeLists, build verified |
