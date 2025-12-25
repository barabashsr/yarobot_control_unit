# Joystick Control Tool

Python host-side utility for manual control of YaRobot Control Unit using a PlayStation DualShock 4 controller.

## Features

- Manual axis jogging with configurable speeds
- Emergency stop (E-STOP) via Share button
- Brake control for servo axes (X, Y, Z, A, B)
- Enable/disable axis control with position loss clearing
- Home position save and restore
- Real-time status display with axis states
- Graceful disconnect handling

## Requirements

- Python 3.8+
- Sony DualShock 4 controller (CUH-ZCT2E) connected via Bluetooth
- YaRobot Control Unit connected via USB CDC serial

## Installation

```bash
cd tools/joystick_control

# Create virtual environment (required on modern Ubuntu/Debian)
python3 -m venv venv

# Install dependencies
./venv/bin/pip install -r requirements.txt
```

## Usage

```bash
cd tools/joystick_control

# Option 1: Run directly with venv python
./venv/bin/python joystick_control.py

# Option 2: Activate venv first
source venv/bin/activate
python joystick_control.py

# Specify config file
python joystick_control.py --config /path/to/config.yaml

# Override serial port
python joystick_control.py --port /dev/ttyUSB1
```

## Controller Mapping

### Axis Jog Buttons

| Button | Axis | Default Speed |
|--------|------|---------------|
| Cross (X) | X | 4 mm/s |
| Circle (O) | Y | 1 mm/s |
| Square | Z | 2 mm/s |
| Triangle | A | 1 mm/s |
| L1 | B | 1 mm/s |
| R1 | C | 2 mm/s |
| L2 | D | 2 mm/s |

### Modifier Buttons

| Button | Function |
|--------|----------|
| R2 (hold) | Negative direction modifier |
| D-pad Left + Axis | Toggle brake (servo axes only) |
| D-pad Up + Axis | Toggle enable + clear POSLOS |

### Special Function Buttons

| Button | Function |
|--------|----------|
| Share | **E-STOP** - Immediate emergency stop |
| Touchpad | Save current positions as home |
| Options | Move all axes to saved home positions |

## Configuration

Edit `config.yaml` to customize:

### Serial Settings

```yaml
serial:
  port: /dev/ttyUSB0    # Serial port
  baudrate: 115200      # Baud rate
  timeout: 1.0          # Read timeout in seconds
```

### Axis Speeds

```yaml
axes:
  X:
    button: 0           # DualShock 4 button ID
    speed: 0.004        # Jog speed in m/s (4 mm/s)
    has_brake: true     # Whether axis has brake hardware
```

### Home Positions

Home positions are automatically saved when you press the Touchpad button:

```yaml
home_positions:
  X: 0.150
  Y: 1.571
  Z: 0.000
  # ...
```

## Status Display

The tool displays a real-time status table:

```
==================================================
       YAROBOT JOYSTICK CONTROL TOOL
==================================================

+---------+-----------+---------------+
|  Axis   |  Enabled  |     Brake     |
+---------+-----------+---------------+
|    X    |  ENABLED  |   RELEASED    |
|    Y    |  ENABLED  |   RELEASED    |
|    Z    | DISABLED  |   ENGAGED     |
|    A    |  ENABLED  |   RELEASED    |
|    B    | DISABLED  |   ENGAGED     |
|    C    |  ENABLED  |  (no brake)   |
|    D    |  ENABLED  |  (no brake)   |
+---------+-----------+---------------+

[MODE] READY

--------------------------------------------------
Recent Commands:
[12:34:56] [TX] VEL X 0.004000
[12:34:56] [RX] OK
[12:34:57] [TX] STOP X
[12:34:57] [RX] OK
--------------------------------------------------
```

## Safety Features

- **E-STOP**: Share button sends `TEST ESTOP` immediately
- **Disconnect handling**: All axes stopped on serial or joystick disconnect
- **Ctrl+C**: Graceful shutdown with `STOP` command to all axes
- **Startup mode check**: Warns if system is in ESTOP or ERROR state

## ESP32 Commands Used

| Command | Purpose |
|---------|---------|
| `MODE` | Query/set system mode |
| `VEL {axis} {speed}` | Start continuous motion |
| `STOP {axis}` | Stop axis motion |
| `TEST ESTOP` | Trigger emergency stop |
| `POS` | Query all axis positions |
| `MOVE {axis} {pos}` | Move to absolute position |
| `BRAKE {axis} [0/1]` | Query/set brake state |
| `EN {axis} [0/1]` | Enable/disable axis |
| `POSOK {axis}` | Clear position loss flag |

## Troubleshooting

### No joystick found
- Ensure DualShock 4 is paired via Bluetooth
- Check that controller is in pairing mode (hold Share + PS button)

### Cannot connect to serial port
- Verify ESP32 is connected via USB
- Check port permissions: `sudo chmod 666 /dev/ttyUSB0`
- Verify correct port in config or use `--port` argument

### Brake commands fail
- Only servo axes (X, Y, Z, A, B) have brake hardware
- Axes C and D are steppers with no brakes

### System in E-STOP
- Clear E-STOP condition on the hardware
- Send `RST` command to reset the system
- Restart the joystick control tool

## License

Internal use only - YaRobot Team
