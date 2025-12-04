# Story 1.1: ESP-IDF Project Initialization

Status: review

## Story

As a **developer**,
I want **an ESP-IDF project properly configured for ESP32-S3 N16R8**,
so that **I can build and flash firmware to the target hardware**.

## Acceptance Criteria

1. **AC1:** Given a fresh clone of the repository, when I run `get_idf && idf.py build`, then the project compiles without errors
2. **AC2:** The target is set to esp32s3 (`idf.py set-target esp32s3` was executed)
3. **AC3:** sdkconfig (configured via menuconfig) contains:
   - `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y` (Serial flasher config → Flash size → 16 MB)
   - `CONFIG_SPIRAM=y` and `CONFIG_SPIRAM_MODE_OCT=y` (Component config → Hardware Settings → SPI RAM config)
   - `CONFIG_FREERTOS_HZ=1000` (Component config → FreeRTOS → Kernel → configTICK_RATE_HZ)
   - `CONFIG_ESP_CONSOLE_USB_CDC=y` (Component config → ESP System Settings → Channel for console output → USB CDC)
   - `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y` (Component config → ESP System Settings → CPU frequency → 240 MHz)
4. **AC4:** `idf.py flash` successfully programs the device
5. **AC5:** `idf.py monitor` shows boot messages including "Hello from app_main" and PSRAM detection
6. **AC6:** `partitions.csv` exists with NVS, factory app, and storage partitions per architecture spec
7. **AC7:** `idf_component.yml` declares dependency on `espressif/mcp23017: "^0.1.1"`

## Tasks / Subtasks

- [x] **Task 1: Create ESP-IDF project using idf.py** (AC: 1, 2)
  - [x] Run `get_idf` to source ESP-IDF environment
  - [x] Navigate to `firmware/` directory (project created in dedicated firmware folder)
  - [x] Run `idf.py create-project -p . yarobot_control_unit` to generate standard structure
  - [x] Run `idf.py set-target esp32s3` to configure for ESP32-S3

- [x] **Task 2: Configure target and board via sdkconfig.defaults** (AC: 3)
  - [x] Created `sdkconfig.defaults` with all required settings (no interactive menuconfig needed)
  - [x] **Flash Configuration:** CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
  - [x] **PSRAM Configuration:** CONFIG_SPIRAM=y, CONFIG_SPIRAM_MODE_OCT=y
  - [x] **FreeRTOS Configuration:** CONFIG_FREERTOS_HZ=1000
  - [x] **Console Configuration:** CONFIG_ESP_CONSOLE_USB_CDC=y
  - [x] **CPU Configuration:** CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
  - [x] **Partition Table:** CONFIG_PARTITION_TABLE_CUSTOM=y, partitions.csv

- [x] **Task 3: Create partition table** (AC: 6)
  - [x] Create `partitions.csv` per architecture specification:
    - nvs: 0x6000 bytes at 0x9000
    - phy_init: 0x1000 bytes at 0xf000
    - factory: 0x300000 bytes at 0x10000 (3MB for app)
    - storage: 0x100000 bytes (1MB SPIFFS)
  - [x] Add partition table reference to sdkconfig.defaults

- [x] **Task 4: Customize main.cpp** (AC: 5)
  - [x] Renamed `main/yarobot_control_unit.c` to `.cpp`
  - [x] Add ESP_LOGI logging with "Hello from app_main" message
  - [x] Added PSRAM detection verification code
  - [x] Update `main/CMakeLists.txt` for .cpp

- [x] **Task 5: Add component dependencies** (AC: 7)
  - [x] Create `main/idf_component.yml` with espressif/mcp23017 ^0.1.1 dependency

- [x] **Task 6: Build project** (AC: 1)
  - [x] Run `idf.py reconfigure` to apply sdkconfig.defaults
  - [x] Run `idf.py build` and verify clean compilation
  - [x] Verify sdkconfig was generated with correct values

- [x] **Task 7: Flash and verify** (AC: 4, 5)
  - [x] Connect ESP32-S3-DevKitC-1 N16R8 via USB (both UART and USB-OTG ports)
  - [x] Run `idf.py flash` via UART port
  - [x] Run `idf.py monitor -p /dev/cu.usbmodem1101` via USB CDC port
  - [x] Verify PSRAM is detected: "PSRAM detected: 8388608 bytes (8.0 MB)"
  - [x] Verify USB CDC enumeration working

## Dev Notes

### Architecture Constraints

> **MANDATORY: Development Environment Setup**
>
> Before running any `idf.py` commands, source the ESP-IDF environment:
> ```bash
> get_idf
> # Or: . /Users/sergeybarabash/robo/esp/v5.4/esp-idf/export.sh
> ```

> **MANDATORY: Header-Only Configuration**
>
> This story creates project infrastructure only. No application constants should be hardcoded.
> All future constants must go in configuration headers (Story 1.3).

### Key Implementation Details

**Step 1: Create project with idf.py (generates standard structure):**
```bash
get_idf
cd /Users/sergeybarabash/robo/yarobot_control_unit
idf.py create-project -p . yarobot_control_unit
idf.py set-target esp32s3
```

This generates:
- `CMakeLists.txt` - Project root CMake
- `main/CMakeLists.txt` - Main component CMake
- `main/yarobot_control_unit.c` - Entry point (rename to .cpp)

**Step 2: Customize main.cpp (rename from .c):**
```cpp
#include "esp_log.h"

static const char* TAG = "main";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Hello from app_main");
    ESP_LOGI(TAG, "YaRobot Control Unit - Story 1.1 Complete");
}
```

**Step 3: Update main/CMakeLists.txt for .cpp:**
```cmake
idf_component_register(
    SRCS "yarobot_control_unit.cpp"
    INCLUDE_DIRS "."
)
```

**Step 4: Configure via menuconfig (idf.py menuconfig):**

| Menu Path | Setting | Value |
|-----------|---------|-------|
| Serial flasher config → Flash size | FLASHSIZE | 16 MB |
| Component config → Hardware Settings → SPI RAM config | SPIRAM | Enable |
| Component config → Hardware Settings → SPI RAM config → Mode | SPIRAM_MODE | Octal |
| Component config → FreeRTOS → Kernel → configTICK_RATE_HZ | FREERTOS_HZ | 1000 |
| Component config → ESP System Settings → Channel for console output | ESP_CONSOLE | USB CDC |
| Component config → ESP System Settings → CPU frequency | CPU_FREQ | 240 MHz |
| Partition Table → Partition Table | PARTITION_TABLE | Custom partition table CSV |
| Partition Table → Custom partition CSV file | PARTITION_TABLE_CUSTOM_FILENAME | partitions.csv |

**Resulting CONFIG_ values (verify in sdkconfig after save):**
```
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_CONSOLE_USB_CDC=y
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
```

**partitions.csv:**
```csv
# Name,   Type, SubType, Offset,  Size,    Flags
nvs,      data, nvs,     0x9000,  0x6000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, 0x10000, 0x300000,
storage,  data, spiffs,  ,        0x100000,
```

**main/idf_component.yml:**
```yaml
dependencies:
  espressif/mcp23017: "^0.1.1"
```

### Project Structure Notes

After this story, the project structure will be:
```
yarobot_control_unit/
├── CMakeLists.txt              # Project root CMake
├── sdkconfig.defaults          # Default configuration
├── partitions.csv              # Flash partition table
├── main/
│   ├── CMakeLists.txt          # Main component CMake
│   ├── main.cpp                # Application entry point
│   └── idf_component.yml       # Component dependencies
└── docs/                       # Existing documentation
```

### Verification Commands

```bash
# Source environment (required before each terminal session)
get_idf

# Create project and set target (Task 1)
idf.py create-project -p . yarobot_control_unit
idf.py set-target esp32s3

# Configure via menuconfig (Task 2)
idf.py menuconfig

# Build (after menuconfig and creating partitions.csv)
idf.py build

# Flash
idf.py flash

# Monitor (Ctrl+] to exit)
idf.py monitor

# Check memory usage
idf.py size

# Verify sdkconfig values after menuconfig
grep -E "CONFIG_ESPTOOLPY_FLASHSIZE|CONFIG_SPIRAM|CONFIG_FREERTOS_HZ|CONFIG_ESP_CONSOLE|CONFIG_ESP_DEFAULT_CPU" sdkconfig
```

**Expected grep output:**
```
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_CONSOLE_USB_CDC=y
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
```

**Expected boot log (idf.py monitor) should show:**
```
I (xxx) spiram: Found 8MB SPI RAM device
I (xxx) spiram: Speed: 80MHz
I (xxx) main: Hello from app_main
```

### References

**Project Documentation:**
- [Source: docs/architecture.md#Project-Initialization] - Development environment setup, sdkconfig settings
- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Data-Models-and-Contracts] - partitions.csv format
- [Source: docs/epics.md#Story-1.1] - Original acceptance criteria

**ESP-IDF Official Documentation:**
- [ESP32-S3 Kconfig Reference](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/kconfig-reference.html) - All CONFIG_ options
- [SPI Flash and External SPI RAM Configuration](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/flash_psram_config.html) - PSRAM configuration for N16R8
- [ESP-IDF Build System](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/build-system.html) - sdkconfig management

**Key Configuration Notes (from ESP-IDF docs):**
- R8 modules (8MB PSRAM) use Octal SPI mode - GPIO35, 36, 37 are reserved
- Octal PSRAM supports DTR mode only (vs STR for Quad)
- CONFIG_FREERTOS_HZ=1000 provides 1ms tick resolution (default is 100Hz)

## Dev Agent Record

### Context Reference

- `docs/sprint-artifacts/1-1-esp-idf-project-initialization.context.xml`

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

- Boot log verified via USB CDC monitor showing successful initialization
- PSRAM detection confirmed: 8,388,608 bytes (8.0 MB)

### Completion Notes List

- ESP-IDF project created in dedicated `firmware/` directory to keep project root clean
- Used `sdkconfig.defaults` instead of interactive menuconfig for reproducible builds
- All 7 acceptance criteria verified and passing
- USB CDC console requires connection to USB-OTG port (separate from UART flashing port)
- MCP23017 component (v0.1.1) automatically resolved via ESP Component Registry

### File List

**New files created:**
- `firmware/CMakeLists.txt` - Project root CMake
- `firmware/sdkconfig.defaults` - Default configuration settings
- `firmware/sdkconfig` - Generated full configuration
- `firmware/partitions.csv` - Flash partition table
- `firmware/dependencies.lock` - Component dependency lock file
- `firmware/main/CMakeLists.txt` - Main component CMake
- `firmware/main/yarobot_control_unit.cpp` - Application entry point
- `firmware/main/idf_component.yml` - Component dependencies
- `firmware/managed_components/` - Downloaded ESP components (mcp23017, i2c_bus)
- `firmware/build/` - Build artifacts (not tracked in git)

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-02 | SM Agent (Bob) | Initial story draft |
| 2025-12-03 | Dev Agent (Amelia) | Implementation complete - all ACs verified |
