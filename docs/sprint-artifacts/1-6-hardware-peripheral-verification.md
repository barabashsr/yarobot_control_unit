# Story 1.6: Hardware Peripheral Verification

Status: done

## Story

As a **developer**,
I want **to verify all hardware peripherals are communicating correctly**,
so that **I can confidently build drivers knowing the hardware foundation works**.

## Acceptance Criteria

1. **AC1:** Given I2C0 is initialized at I2C_FREQ_HZ on GPIO_I2C_SDA/GPIO_I2C_SCL, when I scan the bus, then devices respond at I2C_ADDR_MCP23017_0 (0x20) and I2C_ADDR_MCP23017_1 (0x21)
2. **AC2:** Given I2C0 is initialized, when I read/write to MCP23017 registers, then operations succeed without timeout
3. **AC3:** Given I2C1 is initialized at I2C_OLED_FREQ_HZ on GPIO_OLED_SDA/GPIO_OLED_SCL, when I scan the bus, then device responds at OLED_ADDRESS (0x3C)
4. **AC4:** Given I2C1 is operational, when OLED displays test pattern or "BOOT OK" message, then display is visible
5. **AC5:** Given I2C0 and I2C1 are both initialized, when I perform operations on I2C1, then I2C0 operations remain unaffected (bus isolation verified)
6. **AC6:** Given SPI2 is initialized with GPIO_SR_MOSI, GPIO_SR_SCLK, GPIO_SR_CS, GPIO_SR_OE, when I write test patterns (0xAAAAAA, 0x555555, 0x000000, 0xFFFFFF), then patterns shift through all 24 bits correctly
7. **AC7:** Given shift register OE pin is configured, when GPIO_SR_OE is HIGH, then outputs are disabled; when LOW, then outputs are enabled
8. **AC8:** Given STEP output pins are configured (GPIO_X_STEP through GPIO_D_STEP), when I toggle each pin, then logic analyzer confirms signal at correct GPIO
9. **AC9:** Given E-stop input (GPIO_E_STOP) is connected, when I read the pin, then correct logic level is returned
10. **AC10:** Given USB CDC is functional, when user sends `TEST I2C` command, then system responds with I2C scan results
11. **AC11:** Given USB CDC is functional, when user sends `TEST SR` command, then system responds with shift register test results (PASS/FAIL)
12. **AC12:** Given USB CDC is functional, when user sends `TEST GPIO` command, then system responds with GPIO pin states
13. **AC13:** Project builds successfully with `cd firmware && idf.py build`
14. **AC14:** `idf.py monitor` shows all test results logged via ESP_LOGI

## Tasks / Subtasks

- [x] **Task 1: Implement I2C0 HAL initialization and MCP23017 verification** (AC: 1, 2)
  - [x] Update `firmware/components/yarobot_hal/i2c_hal/i2c_hal.c` with ESP-IDF 5.4 I2C master driver
  - [x] Use GPIO_I2C_SDA, GPIO_I2C_SCL, I2C_FREQ_HZ from config_i2c.h
  - [x] Implement `i2c_hal_scan_bus()` function to detect devices
  - [x] Verify devices respond at 0x20 and 0x21
  - [x] Log results with ESP_LOGI

- [x] **Task 2: Implement I2C1 HAL initialization and OLED verification** (AC: 3, 4, 5)
  - [x] Initialize I2C1 bus on GPIO_OLED_SDA, GPIO_OLED_SCL at I2C_OLED_FREQ_HZ
  - [x] Scan I2C1 to verify device at OLED_ADDRESS (0x3C)
  - [x] Send OLED initialization sequence (SSD1306 display on commands)
  - [x] Display "BOOT OK" message on OLED
  - [x] Verify I2C0 operations still work after I2C1 init (bus isolation test)

- [x] **Task 3: Implement SPI HAL initialization and shift register test** (AC: 6, 7)
  - [x] Update `firmware/components/yarobot_hal/spi_hal/spi_hal.c` to initialize SPI2
  - [x] Configure GPIO_SR_MOSI, GPIO_SR_SCLK, GPIO_SR_CS as SPI pins
  - [x] Configure GPIO_SR_OE as output (active LOW to enable outputs)
  - [x] Implement `spi_hal_sr_write()` to shift bits into register chain
  - [x] Implement `spi_hal_sr_set_oe()` for output enable control

- [x] **Task 4: Implement GPIO direct pin verification** (AC: 8, 9)
  - [x] Create `test_gpio()` function in test_utils component
  - [x] Toggle each STEP pin (GPIO_X_STEP through GPIO_D_STEP) briefly
  - [x] Read GPIO_E_STOP and log state

- [x] **Task 5: Create test_utils driver component** (AC: 10, 11, 12)
  - [x] Create `firmware/components/drivers/test_utils/CMakeLists.txt`
  - [x] Create `firmware/components/drivers/test_utils/include/test_utils.h`
  - [x] Create `firmware/components/drivers/test_utils/test_utils.c`
  - [x] Implement `test_i2c()` — scans both I2C buses, returns results string
  - [x] Implement `test_sr()` — runs shift register pattern test, returns PASS/FAIL
  - [x] Implement `test_gpio()` — reads all GPIO states, returns formatted string
  - [x] Implement `test_all()` — runs all tests and combines results

- [x] **Task 6: Implement CMD_TEST command handler** (AC: 10, 11, 12)
  - [x] Update `firmware/components/control/tasks/task_stubs.c` cmd_executor_task
  - [x] Parse input for "TEST I2C", "TEST SR", "TEST GPIO", "DIAG", "HELP" commands
  - [x] Call corresponding test_utils functions
  - [x] Send results via printf() (USB CDC output)
  - [x] Response format: `OK I2C0:0x20,0x21 I2C1:0x3C` or `ERROR I2C0:NO_DEVICES`

- [x] **Task 7: Integrate hardware verification into boot sequence** (AC: 14)
  - [x] Create `hardware_init_and_verify()` function in yarobot_control_unit.cpp
  - [x] Initialize GPIO, I2C0, I2C1, SPI in sequence with verification
  - [x] Log hardware verification results during boot
  - [x] If any verification fails, log warning but continue boot (degraded mode)
  - [x] Display "BOOT OK" on OLED after verification

- [x] **Task 8: Build and verify** (AC: 13, 14)
  - [x] Run `get_idf` to source ESP-IDF environment
  - [x] Run `cd firmware && idf.py build`
  - [x] Confirm no build errors (fixed format-truncation and symbol collision)
  - [x] Flash with `idf.py flash`
  - [x] Monitor with `idf.py monitor -p /dev/cu.usbmodem1201`
  - [x] Send TEST I2C, TEST SR, TEST GPIO commands via terminal - ALL PASSED

## Dev Notes

### Architecture Constraints

> **ADR-004: Separate I2C Bus for OLED Display**
>
> The OLED display operates on I2C1, isolated from I2C0 which handles critical I/O (MCP23017 limit switches, alarms). This ensures display updates cannot cause timing issues with safety-critical polling.
>
> - I2C0: MCP23017 × 2 (addresses 0x20, 0x21) — limit switches, ALARM_INPUT, InPos
> - I2C1: SSD1306 OLED (address 0x3C) — status display
>
> [Source: docs/architecture.md#Decision-Summary]

> **MANDATORY: No Magic Numbers**
>
> All pin numbers, addresses, and frequencies must use constants from config headers:
> ```c
> // Use these (from config_i2c.h, config_gpio.h):
> i2c_hal_init(I2C_NUM_0, GPIO_I2C_SDA, GPIO_I2C_SCL, I2C_FREQ_HZ);
> i2c_hal_init(I2C_NUM_1, GPIO_OLED_SDA, GPIO_OLED_SCL, I2C_OLED_FREQ_HZ);
>
> // Never use raw numbers like:
> // i2c_hal_init(0, 21, 22, 400000);  // WRONG
> ```
>
> [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Architecture-Constraints]

### Hardware Reference

**I2C Bus Configuration:**

| Bus | SDA Pin | SCL Pin | Frequency | Devices |
|-----|---------|---------|-----------|---------|
| I2C0 | GPIO_I2C_SDA | GPIO_I2C_SCL | I2C_FREQ_HZ (400kHz) | MCP23017 @ 0x20, 0x21 |
| I2C1 | GPIO_OLED_SDA | GPIO_OLED_SCL | I2C_OLED_FREQ_HZ (400kHz) | SSD1306 @ 0x3C |

**Shift Register Chain (SPI2):**

| Pin | GPIO Constant | Purpose |
|-----|---------------|---------|
| MOSI | GPIO_SR_MOSI | Serial data in |
| SCLK | GPIO_SR_SCLK | Shift clock |
| CS | GPIO_SR_CS | Latch (rising edge) |
| OE | GPIO_SR_OE | Output enable (active LOW) |

**Chain Configuration:** 5× TPIC6B595N = 40 bits total, but only first 24 bits tested in this story (DIR, EN, Brake).

### I2C Scanning Code Pattern

```c
#include "driver/i2c_master.h"
#include "config_i2c.h"

esp_err_t i2c_scan_bus(i2c_port_t port, uint8_t* found_addrs, size_t* count) {
    *count = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        // Probe address with zero-length write
        i2c_master_bus_handle_t bus_handle;
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = I2C_FREQ_HZ,
        };
        i2c_master_dev_handle_t dev_handle;

        if (i2c_master_probe(bus_handle, addr, 50) == ESP_OK) {
            found_addrs[(*count)++] = addr;
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
    }

    return ESP_OK;
}
```

### Shift Register Test Pattern

```c
#include "config_gpio.h"

// Test patterns for shift register chain (24-bit)
static const uint32_t test_patterns[] = {
    0xAAAAAA,  // Alternating 1s (10101010...)
    0x555555,  // Alternating 0s (01010101...)
    0x000000,  // All zeros
    0xFFFFFF,  // All ones
};

esp_err_t test_shift_registers(void) {
    for (int i = 0; i < 4; i++) {
        spi_hal_sr_write(test_patterns[i]);
        vTaskDelay(pdMS_TO_TICKS(10));  // Allow settling
        // Note: Without readback, we verify by observing outputs
        ESP_LOGI(TAG, "SR: Wrote pattern 0x%06lX", test_patterns[i]);
    }

    // Leave in safe state (all zeros = brakes engaged, motors disabled)
    spi_hal_sr_write(0x000000);

    return ESP_OK;
}
```

### CMD_TEST Response Format

| Command | Success Response | Error Response |
|---------|-----------------|----------------|
| `TEST I2C` | `OK I2C0:0x20,0x21 I2C1:0x3C` | `ERROR I2C0:NO_DEVICES` |
| `TEST SR` | `OK SR:PASS` | `ERROR SR:FAIL` |
| `TEST GPIO` | `OK GPIO:ESTOP=1 STEPS=0` | `ERROR GPIO:READ_FAIL` |
| `DIAG` | (Same as TEST ALL) | (Same as TEST ALL) |

### Serial Port Configuration

**IMPORTANT: Use the correct serial port:**

```bash
# Flash and monitor
idf.py flash -p /dev/cu.usbmodem1201
idf.py monitor -p /dev/cu.usbmodem1201

# Or without port (auto-detect for flash):
idf.py flash
idf.py monitor -p /dev/cu.usbmodem1201
```

> **Note:** The USB_SERIAL_JTAG port `/dev/cu.usbmodem1201` is used for console output. If hardware has changed, verify correct port with `ls /dev/cu.usb*`.

### Development Environment Setup

**CRITICAL: Source ESP-IDF before running any idf.py commands:**

```bash
# Source ESP-IDF environment
get_idf

# Verify environment
idf.py --version
```

[Source: docs/sprint-artifacts/tech-spec-epic-1.md#Development-Environment-Setup]

### Learnings from Previous Story

**From Story 1-5 (Status: done)**

- **Console configuration**: Using USB_SERIAL_JTAG for logging (not USB_CDC)
- **Serial port**: `/dev/cu.usbmodem1201` for flash and monitor
- **Stack sizes**: Set to 8192 bytes (16384 for cmd_executor) to prevent overflow
- **Umbrella component pattern**: Use single CMakeLists.txt per major component
- **ESP-IDF 5.4 I2C**: Use `driver/i2c_types.h` and new I2C master driver API

**Key Files from 1-5:**
- `firmware/components/control/CMakeLists.txt` — umbrella pattern reference
- `firmware/components/control/tasks/task_stubs.c` — where to add CMD_TEST handling
- `firmware/main/yarobot_control_unit.cpp` — boot sequence location

[Source: docs/sprint-artifacts/1-5-freertos-task-framework.md#Dev-Agent-Record]

### Project Structure Notes

New component to create:
```
firmware/components/drivers/
└── test_utils/
    ├── CMakeLists.txt
    ├── include/
    │   └── test_utils.h
    └── test_utils.c
```

HAL components to update with real implementations:
```
firmware/components/yarobot_hal/
├── i2c/
│   └── i2c_hal.c          # UPDATE: Add real I2C init and scan
├── spi/
│   └── spi_hal.c          # UPDATE: Add real SPI init and transfer
└── gpio/
    └── gpio_hal.c         # UPDATE: Add real GPIO read/write
```

### References

- [Source: docs/architecture.md#Decision-Summary] — I2C bus separation decision
- [Source: docs/architecture.md#Integration-Points] — Hardware interface specifications
- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Detailed-Design] — HAL interfaces
- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Test-Strategy-Summary] — Hardware verification approach
- [Source: docs/epics.md#Story-1.6] — Story definition and acceptance criteria
- [Source: docs/sprint-artifacts/1-5-freertos-task-framework.md#Dev-Agent-Record] — Previous story learnings

---

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/stories/1-6-hardware-peripheral-verification.context.xml

### Agent Model Used

claude-opus-4-5-20251101

### Debug Log References

N/A

### Completion Notes List

1. **ESP-IDF 5.4 I2C Master Driver**: Used new `i2c_new_master_bus()` and `i2c_master_probe()` API instead of legacy `i2c_driver_install()`. Required `driver/i2c_types.h` instead of `driver/i2c.h`.

2. **Symbol Collision Fix**: Renamed `spi_hal_init()` to `yarobot_spi_init()` to avoid collision with ESP-IDF's internal `spi_hal_init()` in `components/hal/spi_hal.c`.

3. **Nested Component Discovery**: Added `EXTRA_COMPONENT_DIRS` to project CMakeLists.txt to enable ESP-IDF discovery of test_utils component under `components/drivers/test_utils/`.

4. **Format Truncation Fix**: Used precision specifiers (`%.38s`) in `test_all()` snprintf to satisfy GCC's `-Werror=format-truncation=` warning.

5. **OLED Display Init**: Implemented SSD1306 initialization sequence directly in `hardware_init_and_verify()` to display "BOOT OK" without requiring full OLED driver component.

6. **Dual I2C Bus Isolation**: Both I2C0 (MCP23017) and I2C1 (OLED) initialized independently with bus isolation verification in boot sequence.

### File List

**New Files:**
- `firmware/components/drivers/test_utils/CMakeLists.txt`
- `firmware/components/drivers/test_utils/include/test_utils.h`
- `firmware/components/drivers/test_utils/test_utils.c`

**Modified Files:**
- `firmware/CMakeLists.txt` — Added EXTRA_COMPONENT_DIRS for nested components
- `firmware/components/yarobot_hal/i2c_hal/include/i2c_hal.h` — Added `i2c_hal_scan_bus()`, `i2c_hal_is_initialized()`
- `firmware/components/yarobot_hal/i2c_hal/i2c_hal.c` — Full ESP-IDF 5.4 I2C master implementation
- `firmware/components/yarobot_hal/spi_hal/include/spi_hal.h` — Added `spi_hal_sr_write()`, `spi_hal_sr_set_oe()`, `spi_hal_is_initialized()`, renamed init to `yarobot_spi_init()`
- `firmware/components/yarobot_hal/spi_hal/spi_hal.c` — Full SPI implementation for shift registers
- `firmware/components/yarobot_hal/gpio_hal/gpio_hal.c` — Real GPIO configuration
- `firmware/components/control/tasks/task_stubs.c` — Added CMD_TEST, DIAG, HELP command handlers
- `firmware/main/yarobot_control_unit.cpp` — Added `hardware_init_and_verify()` function
- `firmware/components/config/include/config_i2c.h` — Changed to `driver/i2c_types.h`
- `firmware/components/config/include/config_oled.h` — Changed to `driver/i2c_types.h`

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-04 | SM Agent (Bob) | Initial story draft |
| 2025-12-04 | Dev Agent (Amelia) | Implemented all HAL drivers, test_utils component, CMD_TEST handlers, and boot sequence integration. Build successful. |
