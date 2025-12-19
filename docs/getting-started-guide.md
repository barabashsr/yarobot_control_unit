# YaRobot Control Unit - Getting Started Guide

A step-by-step guide for new developers to set up the environment and run motors.

---

## Part 1: Install ESP-IDF

ESP-IDF is the official development framework for ESP32 chips. It includes compiler, build tools, and libraries.

### Step 1.1: Install Prerequisites (Linux)

```bash
sudo apt update
sudo apt install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

### Step 1.2: Clone ESP-IDF

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.3 --recursive https://github.com/espressif/esp-idf.git
```

### Step 1.3: Install ESP-IDF Tools

```bash
cd ~/esp/esp-idf
./install.sh esp32s3
```

This downloads the compiler and tools (~1-2 GB).

### Step 1.4: Set Up Environment Alias

```bash
echo 'alias get_idf="source ~/esp/esp-idf/export.sh"' >> ~/.bashrc
source ~/.bashrc
```

### Step 1.5: Verify Installation

```bash
get_idf
idf.py --version
```

---

## Part 2: Build and Flash Firmware

### Step 2.1: Activate ESP-IDF Environment

```bash
get_idf
```

### Step 2.2: Navigate to Firmware Directory

```bash
cd /home/akhmedov/yarobot_control_unit/firmware
```

### Step 2.3: Build Firmware

```bash
idf.py build
```

### Step 2.4: Flash Firmware to ESP32

```bash
idf.py flash
```

### Step 2.5: Build + Flash + Monitor (All in One)

```bash
idf.py flash monitor
```

Press `Ctrl+]` to exit monitor.

---

## Part 3: Connect to ESP32 via USB

### Step 3.1: Find the USB Serial Port

After plugging in the ESP32-S3:

```bash
ls /dev/ttyACM*
```

Usually appears as `/dev/ttyACM0`.

To confirm:
```bash
dmesg | tail -20
```

### Step 3.2: Fix Permission Issues (if needed)

```bash
# Quick fix:
sudo chmod 666 /dev/ttyACM0

# Permanent fix (add user to dialout group):
sudo usermod -aG dialout $USER
# Then log out and log back in
```

### Step 3.3: Connect with Serial Terminal

**Option A: Using screen**
```bash
screen /dev/ttyACM0 115200
```
Exit: `Ctrl+A` then `K`, then `y`

**Option B: Using picocom**
```bash
picocom -b 115200 /dev/ttyACM0
```
Exit: `Ctrl+A` then `Ctrl+X`

**Option C: Using ESP-IDF monitor**
```bash
cd /home/akhmedov/yarobot_control_unit/firmware
idf.py monitor
```
Exit: `Ctrl+]`

---

## Part 4: Motor Control Commands

Once connected via serial terminal, you can send commands.

### Available Axes

| Axis | Type    | Step GPIO | Description      |
|------|---------|-----------|------------------|
| X    | Servo   | GPIO4     | RMT pulse gen    |
| Y    | Servo   | GPIO5     | MCPWM pulse gen  |
| Z    | Servo   | GPIO6     | RMT pulse gen    |
| A    | Servo   | GPIO7     | RMT pulse gen    |
| B    | Servo   | GPIO15    | RMT pulse gen    |
| C    | Stepper | GPIO16    | MCPWM pulse gen  |
| D    | Stepper | GPIO17    | LEDC pulse gen   |

### Basic Commands

```bash
# Enable motor (required before moving)
EN X 1

# Move to absolute position (degrees)
MOVE X 100.0

# Move with specific velocity (degrees/second)
MOVE X 200.0 360.0

# Query current position
POS X

# Stop motor (controlled deceleration)
STOP X

# Emergency stop
ESTOP X

# Disable motor
EN X 0
```

### Test Sequence

```bash
# 1. Enable X-axis motor
EN X 1

# 2. Move to 100 degrees
MOVE X 100.0

# 3. Check position
POS X

# 4. Move back to 0
MOVE X 0.0

# 5. Disable motor
EN X 0
```

### Test Another Axis (Y)

```bash
EN Y 1
MOVE Y 50.0
POS Y
STOP Y
EN Y 0
```

---

## Part 5: Cursor IDE Setup

### Option A: ESP-IDF Extension

1. Open Cursor
2. Press `Ctrl+Shift+X` (Extensions)
3. Search for "ESP-IDF" by Espressif
4. Install it
5. Press `Ctrl+Shift+P` → "ESP-IDF: Configure ESP-IDF Extension"
6. Select "Find ESP-IDF in your system" → point to `~/esp/esp-idf`

**Use commands:**
- `Ctrl+Shift+P` → "ESP-IDF: Build"
- `Ctrl+Shift+P` → "ESP-IDF: Flash"
- `Ctrl+Shift+P` → "ESP-IDF: Monitor"

### Option B: Use Terminal in Cursor

1. Open terminal in Cursor: `Ctrl+``
2. Run: `source ~/esp/esp-idf/export.sh`
3. Use normal `idf.py` commands

---

## Part 6: Troubleshooting

| Problem | Solution |
|---------|----------|
| `/dev/ttyACM0` not found | Check USB cable (must be data cable, not charge-only) |
| Permission denied | `sudo chmod 666 /dev/ttyACM0` or add to dialout group |
| Build fails | Run `get_idf` first to set environment |
| No response from ESP | Check if firmware is flashed, try reset button |
| Garbled serial output | Verify baud rate is 115200 |
| Motor doesn't move | Check `EN X 1` was sent first |

---

## Part 7: Useful idf.py Commands

| Command | Description |
|---------|-------------|
| `idf.py build` | Compile firmware |
| `idf.py flash` | Upload to ESP32 |
| `idf.py monitor` | Serial console |
| `idf.py flash monitor` | Flash + open console |
| `idf.py menuconfig` | Configure project options |
| `idf.py fullclean` | Clean all build files |
| `idf.py size` | Show firmware size breakdown |

---

## Part 8: Project Architecture Overview

### Key Files

| File | Purpose |
|------|---------|
| `firmware/main/yarobot_control_unit.cpp` | Entry point (`app_main`) |
| `firmware/components/motor/include/i_motor.h` | Motor interface |
| `firmware/components/motor/motor_base.cpp` | Motor implementation |
| `firmware/components/config/include/config_gpio.h` | Pin assignments |
| `firmware/components/interface/usb_cdc/` | USB serial communication |

### Motor Control Flow

```
1. motor_system_init()           → Creates all motor objects
2. motor_system_get_motor(axis)  → Get IMotor* for X/Y/Z/A/B/C/D/E
3. motor->enable(true)           → Enable the motor
4. motor->moveAbsolute(100, 360) → Move to 100° at 360°/s
5. motor->getPosition()          → Check current position
6. motor->stop()                 → Stop with deceleration
```

### Hardware Control

- **Step pulses:** Direct GPIO (RMT, MCPWM, or LEDC peripherals)
- **Direction/Enable:** Via TPIC6B595 shift registers over SPI
- **Position feedback:** PCNT hardware counters or software tracking

---

## Quick Start Checklist

- [ ] ESP-IDF installed (`idf.py --version` works)
- [ ] Environment activated (`get_idf`)
- [ ] Firmware built (`idf.py build`)
- [ ] Firmware flashed (`idf.py flash`)
- [ ] USB port found (`ls /dev/ttyACM*`)
- [ ] Serial terminal connected
- [ ] Test command sent (`EN X 1`)
- [ ] Motor moved (`MOVE X 10`)
