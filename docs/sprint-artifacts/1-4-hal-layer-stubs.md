# Story 1.4: HAL Layer Stubs

Status: done

## Story

As a **developer**,
I want **Hardware Abstraction Layer interfaces defined with stub implementations**,
so that **drivers can be implemented against stable abstractions and unit tested**.

## Acceptance Criteria

1. **AC1:** Given configuration headers are complete, when I examine `firmware/components/hal/gpio_hal/include/gpio_hal.h`, then it contains all function declarations per architecture spec
2. **AC2:** Given configuration headers are complete, when I examine `firmware/components/hal/i2c_hal/include/i2c_hal.h`, then it contains all function declarations per architecture spec
3. **AC3:** Given configuration headers are complete, when I examine `firmware/components/hal/spi_hal/include/spi_hal.h`, then it contains all function declarations per architecture spec
4. **AC4:** All stub implementations return `ESP_OK` or appropriate placeholder values (0 for gpio_hal_get_level)
5. **AC5:** Each HAL header includes Doxygen-style documentation (@file, @brief, @param, @return)
6. **AC6:** Each HAL uses constants from configuration headers (never magic numbers)
7. **AC7:** Project builds successfully with `cd firmware && idf.py build`
8. **AC8:** All HAL components are registered in their respective CMakeLists.txt files with proper REQUIRES dependencies

## Tasks / Subtasks

- [x] **Task 1: Create gpio_hal component** (AC: 1, 4, 5, 6, 8)
  - [x] Create `firmware/components/yarobot_hal/gpio_hal/include/gpio_hal.h` with function declarations
  - [x] Create `firmware/components/yarobot_hal/gpio_hal/gpio_hal.c` with stub implementations
  - [x] Add Doxygen documentation for all functions
  - [x] Implement `gpio_hal_init()` returning ESP_OK
  - [x] Implement `gpio_hal_set_direction()` returning ESP_OK
  - [x] Implement `gpio_hal_set_level()` returning ESP_OK
  - [x] Implement `gpio_hal_get_level()` returning 0 (placeholder)
  - [x] Implement `gpio_hal_set_interrupt()` returning ESP_OK

- [x] **Task 2: Create i2c_hal component** (AC: 2, 4, 5, 6, 8)
  - [x] Create `firmware/components/yarobot_hal/i2c_hal/include/i2c_hal.h` with function declarations
  - [x] Create `firmware/components/yarobot_hal/i2c_hal/i2c_hal.c` with stub implementations
  - [x] Add Doxygen documentation for all functions
  - [x] Implement `i2c_hal_init()` returning ESP_OK
  - [x] Implement `i2c_hal_write()` returning ESP_OK
  - [x] Implement `i2c_hal_read()` returning ESP_OK
  - [x] Implement `i2c_hal_write_read()` returning ESP_OK

- [x] **Task 3: Create spi_hal component** (AC: 3, 4, 5, 6, 8)
  - [x] Create `firmware/components/yarobot_hal/spi_hal/include/spi_hal.h` with function declarations
  - [x] Create `firmware/components/yarobot_hal/spi_hal/spi_hal.c` with stub implementations
  - [x] Add Doxygen documentation for all functions
  - [x] Implement `spi_hal_init()` returning ESP_OK
  - [x] Implement `spi_hal_transfer()` returning ESP_OK

- [x] **Task 4: Update parent hal CMakeLists.txt** (AC: 8)
  - [x] Create `firmware/components/yarobot_hal/CMakeLists.txt` as umbrella component
  - [x] Component dependencies correctly specified (config, esp_driver_gpio, esp_driver_i2c, esp_driver_spi)

- [x] **Task 5: Verify build** (AC: 7)
  - [x] Run `cd firmware && idf.py build`
  - [x] Confirm no build errors or warnings
  - [x] Confirm HAL components are compiled and linked (libyarobot_hal.a contains all symbols)

## Dev Notes

### Architecture Constraints

> **ADR-002: Layered Component Architecture**
>
> HAL → Drivers → Control → Interface layering.
> Clear dependencies, unit testable, follows ESP-IDF conventions.
>
> [Source: docs/architecture.md#ADR-002]

> **MANDATORY: No Magic Numbers**
>
> HAL implementations must use configuration header constants:
> - GPIO pins → `config_gpio.h` (GPIO_X_STEP, GPIO_I2C_SDA, etc.)
> - I2C addresses → `config_i2c.h` (I2C_ADDR_MCP23017_0, etc.)
> - Timing values → `config_timing.h` (TIMING_I2C_TIMEOUT_MS, etc.)
>
> [Source: docs/architecture.md#Header-Only-Configuration-Requirement]

### HAL API Specifications

**gpio_hal.h (from architecture):**

```c
esp_err_t gpio_hal_init(void);
esp_err_t gpio_hal_set_direction(gpio_num_t pin, gpio_mode_t mode);
esp_err_t gpio_hal_set_level(gpio_num_t pin, uint32_t level);
int gpio_hal_get_level(gpio_num_t pin);
esp_err_t gpio_hal_set_interrupt(gpio_num_t pin, gpio_int_type_t type,
                                  gpio_isr_t handler, void* arg);
```

**i2c_hal.h (from architecture):**

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

**spi_hal.h (from architecture):**

```c
esp_err_t spi_hal_init(spi_host_device_t host, gpio_num_t mosi,
                       gpio_num_t sclk, gpio_num_t cs);
esp_err_t spi_hal_transfer(spi_host_device_t host, const uint8_t* tx,
                           uint8_t* rx, size_t len);
```

### Component Structure

```
firmware/components/hal/
├── CMakeLists.txt          # Umbrella or empty (subdirs handle registration)
├── gpio_hal/
│   ├── CMakeLists.txt      # idf_component_register(SRCS gpio_hal.c INCLUDE_DIRS include REQUIRES config)
│   ├── include/
│   │   └── gpio_hal.h
│   └── gpio_hal.c
├── i2c_hal/
│   ├── CMakeLists.txt      # idf_component_register(SRCS i2c_hal.c INCLUDE_DIRS include REQUIRES config)
│   ├── include/
│   │   └── i2c_hal.h
│   └── i2c_hal.c
└── spi_hal/
    ├── CMakeLists.txt      # idf_component_register(SRCS spi_hal.c INCLUDE_DIRS include REQUIRES config)
    ├── include/
    │   └── spi_hal.h
    └── spi_hal.c
```

### Stub Implementation Pattern

Each stub should follow this pattern:

```c
/**
 * @brief Initialize the GPIO HAL
 * @return ESP_OK on success
 * @note This is a stub implementation - returns ESP_OK without side effects
 */
esp_err_t gpio_hal_init(void)
{
    ESP_LOGI(TAG, "gpio_hal_init() stub called");
    return ESP_OK;
}
```

### Development Environment Setup

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

[Source: docs/sprint-artifacts/tech-spec-epic-1.md#Development-Environment-Setup]

### ESP-IDF Dependencies

The HAL stubs should include but not yet call ESP-IDF driver APIs:

- `driver/gpio.h` - for gpio_hal
- `driver/i2c_master.h` - for i2c_hal (ESP-IDF 5.4 new I2C driver)
- `driver/spi_master.h` - for spi_hal

### Testing Strategy

- **Build verification:** `idf.py build` completes without errors
- **Link verification:** HAL symbols are present in the binary
- **Future testability:** These stubs enable mocking in unit tests when real implementations are added

### Learnings from Previous Story

**From Story 1-3 (Status: done)**

- All 11 configuration headers created with Doxygen documentation
- Header dependency chain established: config_yaml_schema.h includes config_limits.h and config_defaults.h; all others standalone
- Build verified successful with ESP-IDF 5.4
- config component registered at `firmware/components/config/` with INCLUDE_DIRS "include"
- Use config.h as the master include to access all configuration constants

**Key Files to Reference:**
- `firmware/components/config/include/config_gpio.h` - GPIO pin definitions
- `firmware/components/config/include/config_i2c.h` - I2C addresses and MCP23017 mappings
- `firmware/components/config/include/config_peripherals.h` - SPI_HOST_SR and other peripheral assignments
- `firmware/components/config/include/config_timing.h` - I2C timeout values

[Source: docs/sprint-artifacts/1-3-configuration-header-framework.md#Dev-Agent-Record]

### Project Structure Notes

- HAL directory already exists at `firmware/components/hal/` from Story 1.2
- Subdirectory structure (gpio_hal, i2c_hal, spi_hal) matches architecture specification
- Each HAL should be a separate ESP-IDF component with its own CMakeLists.txt
- All HALs depend on config component for access to configuration headers

### References

- [Source: docs/architecture.md#ADR-002] - Layered architecture decision
- [Source: docs/architecture.md#APIs-and-Interfaces] - HAL API specifications
- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#APIs-and-Interfaces] - Complete HAL function signatures
- [Source: docs/epics.md#Story-1.4] - Story definition and acceptance criteria
- [Source: docs/sprint-artifacts/1-3-configuration-header-framework.md] - Previous story learnings

---

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/1-4-hal-layer-stubs.context.xml

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

N/A

### Completion Notes List

1. **Component Rename Required**: ESP-IDF has a built-in `hal` component, so our custom HAL was renamed from `firmware/components/hal/` to `firmware/components/yarobot_hal/` to avoid naming conflict.

2. **Umbrella Component Pattern**: Instead of separate ESP-IDF components for each HAL subdirectory, an umbrella component pattern was used. The parent `yarobot_hal/CMakeLists.txt` aggregates all sources and include directories.

3. **I2C Header Update**: Changed from `driver/i2c.h` (deprecated) to `driver/i2c_types.h` for ESP-IDF 5.4 compatibility. The `i2c_port_t` type is defined in `i2c_types.h`.

4. **ESP-IDF 5.4 Driver Dependencies**: Required components include `esp_driver_gpio`, `esp_driver_i2c`, and `esp_driver_spi` (new naming convention in ESP-IDF 5.x).

5. **AC Paths Deviation**: Story AC1-3 reference paths `firmware/components/hal/*/` but actual implementation is at `firmware/components/yarobot_hal/*/` due to naming conflict. All function signatures match the architecture spec exactly.

### File List

- `firmware/components/yarobot_hal/CMakeLists.txt` (created)
- `firmware/components/yarobot_hal/gpio_hal/include/gpio_hal.h` (created)
- `firmware/components/yarobot_hal/gpio_hal/gpio_hal.c` (created)
- `firmware/components/yarobot_hal/i2c_hal/include/i2c_hal.h` (created)
- `firmware/components/yarobot_hal/i2c_hal/i2c_hal.c` (created)
- `firmware/components/yarobot_hal/spi_hal/include/spi_hal.h` (created)
- `firmware/components/yarobot_hal/spi_hal/spi_hal.c` (created)

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-03 | SM Agent (Bob) | Initial story draft |
| 2025-12-04 | Dev Agent (Amelia) | Implemented all HAL stubs, build verified |
