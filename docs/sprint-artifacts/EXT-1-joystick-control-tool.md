# Story EXT-1: Joystick Control Tool

Status: review

## Story

As a **machine operator**,
I want **to control robot axes manually using a PlayStation DualShock 4 controller**,
so that **I can jog axes, trigger emergency stop, manage brakes and enable states, and quickly return to home positions during testing and setup**.

## Overview

Python host-side utility that connects to ESP32 via USB serial and accepts input from a DualShock 4 gamepad connected via Bluetooth. Provides manual axis control, emergency stop, brake/enable management, and home position management.

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Controller | Sony DualShock 4 (CUH-ZCT2E) |
| Controller Connection | Bluetooth to host PC |
| Target | ESP32-S3 (yarobot_control_unit) |
| Target Connection | USB CDC serial, 115200 baud |

## Button Mapping

### Action Buttons

| Button | ID | Function | Command |
|--------|-----|----------|---------|
| Cross | 0 | X axis jog | `VEL X {speed}` / `STOP X` |
| Circle | 1 | Y axis jog | `VEL Y {speed}` / `STOP Y` |
| Square | 2 | Z axis jog | `VEL Z {speed}` / `STOP Z` |
| Triangle | 3 | A axis jog | `VEL A {speed}` / `STOP A` |
| L1 | 4 | B axis jog | `VEL B {speed}` / `STOP B` |
| R1 | 5 | C axis jog | `VEL C {speed}` / `STOP C` |
| L2 | 6 | D axis jog | `VEL D {speed}` / `STOP D` |

### Modifier Buttons

| Button | ID/Type | Function |
|--------|---------|----------|
| **R2** | 7 | **Direction modifier** - hold for negative jog |
| **D-pad Left** | hat(-1,0) | **Brake modifier** - toggle brake on/off |
| **D-pad Up** | hat(0,1) | **Enable modifier** - toggle enable + POSOK |

### Special Function Buttons

| Button | ID | Function | Command |
|--------|-----|----------|---------|
| **Share** | 8 | **E-STOP** | `TEST ESTOP` |
| **Options** | 9 | **Go Home** | `MOVE {axis} {pos}` per axis |
| **Touchpad** | 13 | **Save Home** | `POS` → save to YAML |

### Keyboard Controls

| Key | Function | Command |
|-----|----------|---------|
| **1-5** | **Go to box_N** | `MOVE {axis} {pos}` per axis |
| **Shift+1-5** | **Save box_N** | `POS` → save to YAML |

## Control Logic

### Startup Sequence

```
ON script_start:
    # 1. Connect to serial port
    CONNECT to serial port from config
    IF failed:
        PRINT "[ERROR] Cannot connect to {port}"
        EXIT

    # 2. Connect to joystick
    INIT pygame joystick
    WAIT for DualShock 4 connection
    IF failed:
        PRINT "[ERROR] No joystick found"
        EXIT

    # 3. Check and set system mode
    SEND "MODE"
    PARSE response → current_mode

    IF current_mode == "IDLE":
        SEND "MODE READY"
        IF OK:
            PRINT "[MODE] Set to READY"
        ELSE:
            PRINT "[WARN] Failed to set READY mode"
    ELIF current_mode == "READY":
        PRINT "[MODE] Already READY"
    ELIF current_mode == "ESTOP":
        PRINT "[WARN] System in E-STOP! Clear E-STOP and send RST command."
    ELIF current_mode == "ERROR":
        PRINT "[WARN] System in ERROR state! Resolve and restart."

    # 4. Query brake status for servo axes
    FOR each axis in [X, Y, Z, A, B]:
        SEND "BRAKE {axis}"
        PARSE response → store brake_state[axis] (ENGAGED/RELEASED)
    FOR each axis in [C, D]:
        brake_state[axis] = NO_BRAKE

    # 5. Initialize enable state tracking
    FOR each axis in [X, Y, Z, A, B, C, D]:
        enable_state[axis] = UNKNOWN

    # 6. Display status table
    DISPLAY status table

    PRINT "[READY] Joystick control active. Press Share for E-STOP."
```

### Axis Jog (Button 0-6)

```
ON button_press(axis_button):
    IF R2_pressed (modifier):
        SEND "VEL {axis} -{speed}"    # Negative direction
    ELSE:
        SEND "VEL {axis} {speed}"     # Positive direction

ON button_release(axis_button):
    SEND "STOP {axis}"
```

### Brake Toggle (D-pad Left + Axis Button)

```
ON button_press(axis_button) WITH dpad_left_pressed:
    IF axis NOT in [X, Y, Z, A, B]:
        PRINT "[WARN] Axis {axis} has no brake hardware"
        RETURN

    IF brake_state[axis] == ENGAGED:
        SEND "BRAKE {axis} 1"   # Release
        brake_state[axis] = RELEASED
    ELSE:
        SEND "BRAKE {axis} 0"   # Engage
        brake_state[axis] = ENGAGED
    DISPLAY status table
```

### Enable Toggle (D-pad Up + Axis Button)

```
ON button_press(axis_button) WITH dpad_up_pressed:
    IF enable_state[axis] == ENABLED:
        SEND "EN {axis} 0"       # Disable
        enable_state[axis] = DISABLED
    ELSE:
        SEND "EN {axis} 1"       # Enable
        SEND "POSOK {axis}"      # Clear position loss
        enable_state[axis] = ENABLED
    DISPLAY status table
```

### E-STOP (Share - Button 8)

```
ON button_press(SHARE):
    SEND "TEST ESTOP"
    PRINT "[!!!] E-STOP ACTIVATED"
    # Update all states
    FOR each axis:
        enable_state[axis] = DISABLED
    DISPLAY status table
```

### Save Home (Touchpad - Button 13)

```
ON button_press(TOUCHPAD):
    SEND "POS"
    PARSE response → extract X, Y, Z, A, B, C, D positions
    UPDATE config.yaml → home_positions section
    SAVE config.yaml
    PRINT "[SAVED] Home positions saved to config.yaml"
```

### Go Home (Options - Button 9)

```
ON button_press(OPTIONS):
    READ home_positions from config.yaml
    IF home_positions not set:
        PRINT "[ERROR] No home positions saved. Press Touchpad first."
        RETURN

    # Step 1: X and Z move together (parallel)
    SEND "MOVE X {home_position} {speed}"
    SEND "MOVE Z {home_position} {speed}"
    WAIT for "Axis N: motion complete" log messages for X and Z
    SLEEP 1 second

    # Step 2: A, B, C, D one by one
    FOR each axis in [A, B, C, D]:
        IF home_positions[axis] is not null:
            SEND "MOVE {axis} {home_position} {speed}"
            WAIT for "Axis N: motion complete" log message
            SLEEP 1 second

    # Step 3: Y last
    SEND "MOVE Y {home_position} {speed}"
    WAIT for "Axis 1: motion complete" log message
    SLEEP 1 second

    PRINT "[HOME] All axes at home position"
```

### Save Box Position (Shift+1-5)

```
ON key_press(1-5) WITH shift_pressed:
    box_num = key - '0'
    SEND "POS"
    PARSE response → extract X, Y, Z, A, B, C, D positions
    UPDATE config.yaml → box_positions.box_{num} section
    SAVE config.yaml
    PRINT "[SAVED] Box {num} position saved to config.yaml"
```

### Go to Box Position (1-5)

```
ON key_press(1-5):
    box_num = key - '0'
    READ box_positions.box_{num} from config.yaml
    IF box_positions.box_{num} not set:
        PRINT "[ERROR] Box {num} not saved. Press Shift+{num} first."
        RETURN

    # Step 1: X and Z move together (parallel)
    SEND "MOVE X {position} {speed}"
    SEND "MOVE Z {position} {speed}"
    WAIT for "Axis N: motion complete" log messages for X and Z
    SLEEP 1 second

    # Step 2: A, B, C, D one by one
    FOR each axis in [A, B, C, D]:
        IF box_positions[axis] is not null:
            SEND "MOVE {axis} {position} {speed}"
            WAIT for "Axis N: motion complete" log message
            SLEEP 1 second

    # Step 3: Y last
    SEND "MOVE Y {position} {speed}"
    WAIT for "Axis 1: motion complete" log message
    SLEEP 1 second

    PRINT "[BOX] Box {num} sequence complete"
```

## Acceptance Criteria

1. **AC1**: Script connects to ESP32 via configurable serial port (default /dev/ttyUSB0, 115200 baud)
2. **AC2**: Script connects to DualShock 4 controller via Bluetooth using pygame
3. **AC3**: On startup, script queries mode via `MODE` and sets to READY if in IDLE
4. **AC4**: Pressing axis button (0-6) sends `VEL {axis} {speed}` command
5. **AC5**: Releasing axis button sends `STOP {axis}` command
6. **AC6**: Holding R2 (modifier) + axis button sends negative speed value
7. **AC7**: Pressing Share button sends `TEST ESTOP` command immediately
8. **AC8**: Pressing Touchpad queries positions via `POS` and saves to YAML config
9. **AC9**: Pressing Options moves all axes sequentially to saved home positions
10. **AC10**: Script handles serial disconnect gracefully (stops all axes, shows error)
11. **AC11**: Script handles joystick disconnect gracefully (stops all axes, shows error)
12. **AC12**: Ctrl+C sends STOP to all axes before exiting
13. **AC13**: Console shows real-time feedback of commands sent and responses received
14. **AC14**: On startup, script queries brake status for all servo axes (X, Y, Z, A, B) via `BRAKE {axis}`
15. **AC15**: D-pad Left + axis button toggles brake state via `BRAKE {axis} 0/1`
16. **AC16**: D-pad Up + axis button toggles enable state via `EN {axis} 0/1`
17. **AC17**: When enabling axis, script also sends `POSOK {axis}` to clear position loss
18. **AC18**: Status table displays current enable and brake state for all axes
19. **AC19**: Status table updates after each state change
20. **AC20**: Pressing Shift+1-5 saves current positions as box_1..5 to config.yaml
21. **AC21**: Pressing 1-5 moves all axes to saved box_N positions using configured axis speeds
22. **AC22**: Status display shows keyboard controls for box positions

## Tasks / Subtasks

- [x] Task 1: Project setup (AC1, AC2)
  - [x] Create `tools/joystick_control/` directory structure
  - [x] Create `requirements.txt` with pygame, pyserial, pyyaml
  - [x] Create base `config.yaml` with default values

- [x] Task 2: Serial communication module (AC1)
  - [x] Implement serial connection with configurable port/baudrate
  - [x] Implement send_command() with response parsing
  - [x] Implement reconnection logic on disconnect

- [x] Task 3: Joystick input module (AC2)
  - [x] Initialize pygame joystick subsystem
  - [x] Detect and connect to DualShock 4
  - [x] Implement button press/release event handling
  - [x] Implement D-pad (hat) direction detection

- [x] Task 4: Startup sequence (AC3, AC14)
  - [x] Query current mode with MODE command
  - [x] Set MODE READY if currently IDLE
  - [x] Handle ESTOP/ERROR states with warnings
  - [x] Query brake status for axes X, Y, Z, A, B
  - [x] Initialize enable state tracking

- [x] Task 5: Axis jog control (AC4, AC5, AC6)
  - [x] Map buttons 0-6 to axes X-D
  - [x] Implement VEL command on button press
  - [x] Implement STOP command on button release
  - [x] Implement R2 modifier for negative direction

- [x] Task 6: E-STOP function (AC7)
  - [x] Implement Share button handler
  - [x] Send TEST ESTOP immediately on press
  - [x] Display prominent warning in console
  - [x] Update all axis states to disabled

- [x] Task 7: Home position management (AC8, AC9)
  - [x] Implement Touchpad handler for Save Home
  - [x] Parse POS response and extract all axis positions
  - [x] Save home_positions to config.yaml
  - [x] Implement Options handler for Go Home
  - [x] Sequential MOVE commands with status feedback

- [x] Task 8: Brake control (AC15, AC18, AC19)
  - [x] Implement D-pad Left modifier detection
  - [x] Toggle brake state with BRAKE {axis} 0/1
  - [x] Handle "no brake hardware" for axes C, D
  - [x] Update status table after toggle

- [x] Task 9: Enable control (AC16, AC17, AC18, AC19)
  - [x] Track enable state locally (starts as UNKNOWN)
  - [x] Implement D-pad Up modifier detection
  - [x] Toggle enable state with EN {axis} 0/1
  - [x] Send POSOK {axis} after enabling
  - [x] Update E-STOP handler to mark all disabled

- [x] Task 10: Status table display (AC18, AC19)
  - [x] Create formatted ASCII table for console
  - [x] Show Enable state per axis (ENABLED/DISABLED/UNKNOWN)
  - [x] Show Brake state per axis (ENGAGED/RELEASED/no brake)
  - [x] Update and redisplay after each state change
  - [x] Clear screen or use cursor positioning for clean updates

- [x] Task 11: Safety and error handling (AC10, AC11, AC12)
  - [x] Detect serial port disconnect
  - [x] Detect joystick disconnect
  - [x] Implement graceful shutdown on Ctrl+C
  - [x] Send STOP to all active axes on any disconnect

- [x] Task 12: Console UI (AC13)
  - [x] Real-time command/response logging
  - [x] Status indicators for connection state
  - [x] Clear feedback for E-STOP, Save Home, Go Home

- [x] Task 13: Documentation
  - [x] Create README.md with usage instructions
  - [x] Document YAML configuration options
  - [x] Document button mapping and controls
  - [x] Add example configuration

- [x] Task 14: Box positions feature (AC20, AC21, AC22)
  - [x] Add box_positions to Config dataclass
  - [x] Load/save box_positions from/to config.yaml
  - [x] Implement keyboard event handling for 1-5 and Shift+1-5
  - [x] Implement _do_save_box() to save current positions as box_N
  - [x] Implement _do_go_box() to move to box_N positions
  - [x] Update status display with keyboard controls help

## Speed Configuration (Initial Values)

| Axis | Speed (mm/s) | Min Allowed |
|------|--------------|-------------|
| X | 4 | 3.5 |
| Y | 1 | 0.48 |
| Z | 2 | 1.6 |
| A | 1 | 0.6 |
| B | 1 | 0.72 |
| C | 2 | 1.5 |
| D | 2 | 1.5 |

## YAML Configuration File

```yaml
# Joystick Control Tool Configuration
# tools/joystick_control/config.yaml

serial:
  port: /dev/ttyUSB0
  baudrate: 115200
  timeout: 1.0

joystick:
  connection: bluetooth

# Button assignments (DualShock 4)
buttons:
  # Modifiers
  direction_modifier: 7    # R2 - hold for negative jog direction
  brake_modifier: dpad_left    # D-pad Left - toggle brake
  enable_modifier: dpad_up     # D-pad Up - toggle enable

  # Special functions
  estop: 8             # Share
  go_home: 9           # Options
  save_home: 13        # Touchpad

# Axis configuration
axes:
  X:
    button: 0          # Cross
    speed: 4           # mm/s
    has_brake: true
  Y:
    button: 1          # Circle
    speed: 1           # mm/s
    has_brake: true
  Z:
    button: 2          # Square
    speed: 2           # mm/s
    has_brake: true
  A:
    button: 3          # Triangle
    speed: 1           # mm/s
    has_brake: true
  B:
    button: 4          # L1
    speed: 1           # mm/s
    has_brake: true
  C:
    button: 5          # R1
    speed: 2           # mm/s
    has_brake: false
  D:
    button: 6          # L2
    speed: 2           # mm/s
    has_brake: false

# Home positions (populated by Save Home function)
home_positions:
  X: null
  Y: null
  Z: null
  A: null
  B: null
  C: null
  D: null

# Box positions (populated by Shift+1-5 keyboard shortcuts)
box_positions:
  box_1: null
  box_2: null
  box_3: null
  box_4: null
  box_5: null
```

## Console Status Table

```
┌─────────────────────────────────────────┐
│           AXIS STATUS                   │
├───────┬───────────┬─────────────────────┤
│ Axis  │ Enabled   │ Brake               │
├───────┼───────────┼─────────────────────┤
│   X   │ ENABLED   │ RELEASED            │
│   Y   │ ENABLED   │ RELEASED            │
│   Z   │ DISABLED  │ ENGAGED             │
│   A   │ ENABLED   │ RELEASED            │
│   B   │ DISABLED  │ ENGAGED             │
│   C   │ ENABLED   │ (no brake)          │
│   D   │ ENABLED   │ (no brake)          │
└───────┴───────────┴─────────────────────┘

[MODE] READY
[X+] VEL X 4
[X-] STOP X
```

## File Structure

```
tools/
  joystick_control/
    joystick_control.py    # Main script
    config.yaml            # Configuration file
    requirements.txt       # Python dependencies
    README.md              # Usage documentation
```

## Dev Notes

### Dependencies
- Python 3.8+
- pygame (joystick input)
- pyserial (serial communication)
- pyyaml (configuration)

### ESP32 Commands Used

| Command | Purpose |
|---------|---------|
| `MODE` | Query current mode |
| `MODE READY` | Set mode to READY (from IDLE) |
| `VEL {axis} {speed}` | Start continuous motion |
| `STOP {axis}` | Stop axis motion |
| `STOP` | Stop all axes |
| `TEST ESTOP` | Trigger emergency stop |
| `POS` | Query all axis positions |
| `MOVE {axis} {position}` | Move to absolute position |
| `BRAKE {axis}` | Query brake state |
| `BRAKE {axis} 0` | Engage brake |
| `BRAKE {axis} 1` | Release brake |
| `EN {axis} 0` | Disable axis |
| `EN {axis} 1` | Enable axis |
| `POSOK {axis}` | Acknowledge position (clear POSLOS) |

### DualShock 4 D-pad Handling

The D-pad on DualShock 4 is reported as a "hat" in pygame:
- `joystick.get_hat(0)` returns tuple (x, y)
- Left: (-1, 0)
- Right: (1, 0)
- Up: (0, 1)
- Down: (0, -1)
- Diagonal combinations possible

### Brake Hardware

Only servo axes have physical brakes:
- **With brakes:** X, Y, Z, A, B (5 axes)
- **No brakes:** C, D (stepper axes)

### Mode State Machine

```
IDLE ──MODE READY──► READY ──MODE CONFIG──► CONFIG
  ▲                    │                      │
  └────MODE IDLE───────┴──────MODE IDLE───────┘

ESTOP ──(blocked)──► Cannot change mode until RST
ERROR ──(blocked)──► Cannot change mode until resolved
```

### Safety Considerations
- E-STOP (Share button) has highest priority
- All disconnects trigger automatic STOP ALL
- Script exit always sends STOP to active axes
- Home positions validated before Go Home sequence
- POSOK sent after enable to clear position loss state
- Mode set to READY on startup for motion commands

### References
- [Source: firmware/components/control/command_executor/velocity_handler.cpp] - VEL command
- [Source: firmware/components/control/command_executor/stop_handler.cpp] - STOP command
- [Source: firmware/components/control/command_executor/position_handler.cpp] - POS command
- [Source: firmware/components/control/command_executor/brake_handler.cpp] - BRAKE command
- [Source: firmware/components/control/command_executor/enable_handler.cpp] - EN command
- [Source: firmware/components/control/command_executor/posok_handler.cpp] - POSOK command
- [Source: firmware/components/control/command_executor/command_executor.c] - MODE command
- [Source: firmware/components/config/include/config_defaults.h] - Axis configuration

## Dev Agent Record

### Context Reference
- docs/sprint-artifacts/EXT-1-joystick-control-tool.context.xml

### Agent Model Used
Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References
- Implementation completed in single session
- All 13 tasks implemented in joystick_control.py

### Completion Notes List
- Created complete Python-based joystick control tool for DualShock 4
- Implemented SerialConnection class with thread-safe command/response handling
- Implemented JoystickController class using pygame for DualShock 4 input
- Implemented StatusDisplay class with real-time ASCII table rendering
- Main JoystickControlApp class orchestrates all functionality
- Startup sequence queries MODE and BRAKE status per architecture spec
- VEL/STOP commands sent on button press/release with R2 negative modifier
- E-STOP sends TEST ESTOP immediately, marks all axes disabled
- Home positions saved to config.yaml via POS command parsing
- Go Home moves all axes sequentially via MOVE commands
- Brake toggle via D-pad Left + axis button (BRAKE 0/1)
- Enable toggle via D-pad Up + axis button (EN 0/1 + POSOK)
- Graceful shutdown on Ctrl+C, serial disconnect, or joystick disconnect
- Speed values use SI units (m/s) matching firmware expectations
- Added box_positions feature: 5 named waypoints (box_1..5) separate from home
- Keyboard shortcuts: 1-5 to go to box_N, Shift+1-5 to save current position as box_N
- Box positions use configured axis speeds from config.axes[axis].speed
- Motion sequence for Go Home/Go Box: X+Z parallel first, then A,B,C,D sequential, Y last
- Motion complete detection via ESP-IDF log messages: "Axis N: motion complete at position" or "Axis N: target position already reached"
- Axis index to letter mapping: 0=X, 1=Y, 2=Z, 3=A, 4=B, 5=C, 6=D
- Event callback system with poll_events() for async serial event handling
- 1 second settle delay after each axis motion complete before starting next
- POS command uses multi-line response reading to capture full position data
- SerialConnection.event_callback for handling async events separately from command responses

### File List
- tools/joystick_control/joystick_control.py (NEW) - Main script, all functionality
- tools/joystick_control/config.yaml (NEW) - Configuration file
- tools/joystick_control/requirements.txt (NEW) - Python dependencies
- tools/joystick_control/README.md (NEW) - Usage documentation
