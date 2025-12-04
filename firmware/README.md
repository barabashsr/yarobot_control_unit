# YaRobot Control Unit Firmware

ESP32-S3 N16R8 firmware for the 8-axis motion controller.

## Prerequisites

- **ESP-IDF v5.4** - Espressif IoT Development Framework
- **Python 3.x** - Required by ESP-IDF build system
- **CMake 3.16+** - Build system generator
- **get_idf alias** - Shell alias to source ESP-IDF environment

### ESP-IDF Installation

```bash
# Clone ESP-IDF v5.4
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git -b v5.4
cd esp-idf
./install.sh esp32s3

# Add get_idf alias to your shell profile (~/.zshrc or ~/.bashrc)
alias get_idf='. ~/esp/esp-idf/export.sh'
```

## Build Commands

```bash
# Source ESP-IDF environment (required once per terminal session)
get_idf

# Navigate to firmware directory
cd firmware

# Set target (first time only)
idf.py set-target esp32s3

# Build firmware
idf.py build

# Flash to device
idf.py flash

# Monitor serial output
idf.py monitor -p /dev/cu.usbmodem1201

# Combined flash and monitor
idf.py flash monitor -p /dev/cu.usbmodem1201

# Analyze memory usage
idf.py size

# Clean build (removes all build artifacts)
idf.py fullclean
```

## Hardware Configuration

- **MCU**: ESP32-S3 N16R8 (16MB Flash, 8MB PSRAM)
- **I2C0**: GPIO1 (SDA), GPIO2 (SCL) - MCP23017 expanders at 0x20, 0x21
- **I2C1**: GPIO42 (SDA), GPIO41 (SCL) - OLED display at 0x3C
- **SPI**: GPIO11 (MOSI), GPIO12 (SCLK), GPIO10 (CS) - TPIC6B595 shift registers
- **USB**: Native USB CDC for command interface

## Memory Budget (v1.0.0)

| Type | Used | Available | Usage |
|------|------|-----------|-------|
| Flash | ~273 KB | 3 MB | 9% |
| DRAM | ~65 KB | 334 KB | 19% |
| PSRAM | Heap | 8 MB | Available for runtime allocation |

## FreeRTOS Tasks (16 total)

| Core | Task | Priority | Purpose |
|------|------|----------|---------|
| 0 | safety | 24 | E-stop and safety monitoring |
| 0 | usb_rx | 10 | USB CDC receive |
| 0 | usb_tx | 10 | USB CDC transmit |
| 0 | cmd_exec | 12 | Command processing |
| 0 | i2c_mon | 8 | MCP23017 polling |
| 0 | idle_mon | 4 | Idle tracking |
| 1 | motion_X-E | 15 | 8 axis motion control |
| 1 | display | 5 | OLED update |

## USB Commands

Connect to the USB CDC port and send commands:

```
HELP        - Show available commands
TEST I2C    - Test I2C bus and MCP23017 devices
TEST SR     - Test shift register output
TEST GPIO   - Test GPIO configuration
DIAG        - Run all diagnostics
```

## Boot Sequence

On power-up, the firmware:
1. Initializes GPIO, I2C, and SPI peripherals
2. Verifies MCP23017 devices respond on I2C0
3. Initializes OLED display with "BOOT OK" message
4. Creates all 16 FreeRTOS tasks
5. Outputs `EVENT BOOT V1.0.0 AXES:8 STATE:IDLE` (or STATE:DEGRADED if hardware issues)

## Troubleshooting

### Serial Port Not Found

```bash
# List available USB serial ports
ls /dev/cu.usb*

# Common ports:
# /dev/cu.usbmodem1201     - USB_SERIAL_JTAG (console, ESP32-S3 native USB)
# /dev/cu.wchusbserial*    - External USB-UART adapter (CH340/CP2102)
```

If no port appears:
- Check USB cable supports data (not charge-only)
- Try different USB port
- Reboot device while holding BOOT button

### Build Errors

**"Command not found: idf.py"**
```bash
# Source ESP-IDF environment first
get_idf
```

**"CMake not found"**
```bash
# Install CMake
brew install cmake  # macOS
sudo apt install cmake  # Ubuntu
```

**"esp32s3 is not a valid target"**
```bash
# Install ESP32-S3 toolchain
cd ~/esp/esp-idf
./install.sh esp32s3
```

**Managed components not found**
```bash
# Components are fetched automatically on first build
# If issues persist, clean and rebuild
idf.py fullclean
idf.py build
```

### USB CDC Not Responding

- Wait 2-3 seconds after boot for USB enumeration
- Check if `EVENT BOOT` message appears in monitor output
- Try power cycling the device
- Verify correct port (native USB CDC, not JTAG)

### PSRAM Not Detected

If boot log shows "PSRAM not detected":
- Verify using ESP32-S3 N16R8 module (not N16 without PSRAM)
- Check sdkconfig has `CONFIG_SPIRAM=y`
- Module may be damaged

### MCP23017 Not Responding

If I2C verification fails:
- Check wiring: SDA=GPIO1, SCL=GPIO2
- Verify 3.3V power to MCP23017 boards
- Check I2C address jumpers (expect 0x20, 0x21)
- Use `TEST I2C` command for diagnostics

## Project Structure

```
firmware/
├── CMakeLists.txt          # Project configuration
├── sdkconfig               # ESP-IDF configuration
├── sdkconfig.defaults      # Default configuration values
├── main/
│   └── yarobot_control_unit.cpp  # Entry point and boot sequence
├── components/
│   ├── config/             # Configuration headers
│   ├── control/            # FreeRTOS task implementations
│   ├── drivers/            # Hardware drivers (OLED, shift registers)
│   ├── yarobot_hal/        # Hardware abstraction layer
│   ├── motor/              # Motor control (Epic 3)
│   ├── position/           # Position tracking (Epic 6)
│   └── pulse_gen/          # Pulse generation (Epic 3)
└── managed_components/     # Auto-fetched ESP-IDF components
```

## Related Documentation

- [Architecture](../docs/architecture.md) - System design
- [PRD](../docs/prd.md) - Product requirements
- [Epic 1 Tech Spec](../docs/sprint-artifacts/tech-spec-epic-1.md) - Foundation specifications
