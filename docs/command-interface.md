# Motor Control Command Interface

## Overview
USB CDC text-based command interface for the yarobot_control_unit. Commands follow a simple ASCII protocol for easy testing and integration. Designed with ROS2 compatibility in mind - all axes provide uniform position/velocity interfaces regardless of underlying hardware.

## Command Format

```
<COMMAND> [AXIS] [PARAMS]\n
```

- Commands are case-insensitive
- Axis letters: X, Y, Z, A, B, C, D (or numbers 0-6)
- Parameters separated by spaces
- Line ending: \n or \r\n
- Response format: `OK [data]` or `ERROR [code] [message]`
- Event format: `EVENT <type> <data>`
- Maximum command length: 256 characters
- Float parameters support: scientific notation (1.23e-4) and decimal (123.45)

### Axis Mapping
- **X** (0): Railway X axis - RMT+DMA servo
- **Y** (1): Gripper Y axis - MCPWM+PCNT servo  
- **Z** (2): Selector Z axis - RMT+DMA servo
- **A** (3): Picker Z axis - RMT+DMA servo
- **B** (4): Picker Y axis - RMT+DMA servo
- **C** (5): Picker jaw - MCPWM+PCNT stepper with floating switch
- **D** (6): Picker retractor - LEDC stepper

## Motion Commands

### MOVE - Move to Absolute Position
```
MOVE <axis> <position> [velocity] [acceleration]
```

**Description**: Commands the specified axis to move to an absolute position using a trapezoidal or S-curve motion profile. The motor accelerates to the specified velocity (or default), maintains it, then decelerates to stop exactly at the target position.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6)
- `position`: Target position in configured units (mm or degrees)
- `velocity`: Maximum velocity during move (units/sec), optional - uses configured default if omitted
- `acceleration`: Maximum acceleration/deceleration (units/sec²), optional - uses configured default if omitted

**Behavior**:
- Validates position against software limits before executing
- For servo motors (0-3), uses RMT peripheral for precise pulse generation
- For steppers (4-5), uses MCPWM with pulse counting
- Non-blocking: returns immediately, motion executes asynchronously
- Overwrites any previous motion in progress

**Examples**:
```
MOVE 0 100.5              # Move axis 0 to 100.5mm at default speed
MOVE 1 45.0 10.0         # Move axis 1 to 45° at 10°/sec
MOVE 2 -50 20 100        # Move axis 2 to -50mm at 20mm/s with 100mm/s² accel
```

**Response**:
```
OK MOVING                 # Motion started successfully
ERROR E005 Position limit exceeded  # Position outside limits
ERROR E004 Axis not enabled         # Motor not enabled
```

### MOVR - Move Relative
```
MOVR <axis> <distance> [velocity] [acceleration]
```

**Description**: Commands the specified axis to move a relative distance from its current position. Positive values move forward/clockwise, negative values move backward/counter-clockwise.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6)
- `distance`: Relative distance to move in configured units (+/- for direction)
- `velocity`: Maximum velocity during move (units/sec), optional
- `acceleration`: Maximum acceleration/deceleration (units/sec²), optional

**Behavior**:
- Calculates target as current position + distance
- Validates resulting position against software limits
- Uses same motion profile as MOVE command
- Backlash compensation applied automatically on direction changes

**Examples**:
```
MOVR 0 10                # Move axis 0 forward 10mm from current position
MOVR 1 -5.5              # Move axis 1 backward 5.5 degrees
MOVR 2 25.0 50           # Move axis 2 forward 25mm at 50mm/s
```

**Response**:
```
OK MOVING                # Motion started
ERROR E005 Position limit exceeded  # Would exceed limits
```

### VEL - Velocity Mode
```
VEL <axis> <velocity> [acceleration]
```

**Description**: Commands the axis to move at a constant velocity until stopped or a limit is reached. Used for jogging, continuous motion, or controlled deceleration to stop.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6)
- `velocity`: Target velocity in units/sec (+/- for direction, 0 to stop)
- `acceleration`: Acceleration to reach velocity (units/sec²), optional

**Behavior**:
- Accelerates from current velocity to target velocity
- Maintains constant velocity indefinitely
- Automatically stops at software limits
- Velocity of 0 performs controlled deceleration to stop
- Overwrites any position move in progress

**Examples**:
```
VEL 0 50                 # Move axis 0 at constant 50mm/s forward
VEL 1 -25                # Move axis 1 at constant 25°/s backward  
VEL 0 0                  # Decelerate axis 0 to stop
VEL 2 100 500            # Accelerate axis 2 to 100mm/s at 500mm/s²
```

**Response**:
```
OK VELOCITY              # Velocity mode active
ERROR E005 Velocity limit exceeded  # Velocity too high
```

### STOP - Stop Motion
```
STOP <axis|ALL> [EMERGENCY]
```

**Description**: Stops motion on specified axis or all axes. Normal stop uses configured deceleration. Emergency stop halts immediately without deceleration.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6) or ALL for all axes
- `EMERGENCY`: Optional flag for immediate stop without deceleration

**Behavior**:
- Normal stop: Decelerates at configured rate to zero velocity
- Emergency stop: Immediately stops pulse generation (may lose position)
- ALL: Stops all axes simultaneously
- Clears any queued motion commands
- Emergency stop sets system flag requiring RST to clear

**Priority**: Highest priority command (255 for EMERGENCY, 200 for normal)

**Examples**:
```
STOP 0                   # Controlled stop of axis 0
STOP ALL                 # Controlled stop of all axes  
STOP 2 EMERGENCY         # Emergency stop axis 2
STOP ALL EMERGENCY       # System-wide emergency stop
```

**Response**:
```
OK STOPPED               # Motion stopped
OK EMERGENCY             # Emergency stop activated
```

## Status Commands

### POS - Get Current Position
```
POS <axis|ALL>
```

**Description**: Returns the current position of specified axis or all axes. Position is calculated from pulse counts and configured units per pulse.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6) or ALL for all axes

**Behavior**:
- Returns immediately with current position
- For servo axes (0-3), position verified by Z-signal when available
- For stepper axes (4-5), position based on pulse counting
- ALL returns comma-separated list of all axis positions

**Examples**:
```
POS 0                    # Get position of axis 0
POS ALL                  # Get all positions
```

**Response**:
```
OK 0:25.5               # Single axis at 25.5mm
OK 0:25.5,1:180.0,2:0.0,3:-45.2,4:10.0,5:0.0  # All axes
```

### STAT - Get Axis Status
```
STAT <axis|ALL>
```

**Description**: Returns comprehensive status information for specified axis including position, motion state, and configuration.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6) or ALL for all axes

**Status Fields**:
- `AXIS`: Axis number
- `POS`: Current position (units)
- `TGT`: Target position (units)
- `VEL`: Current velocity (units/sec)
- `MOVING`: Motion state (0=stopped, 1=moving)
- `ENABLED`: Motor enable state (0=disabled, 1=enabled)
- `COMPLETE`: Position complete signal for servos (0=not at target, 1=at target)
- `FAULT`: Error state (0=ok, 1=fault)

**Examples**:
```
STAT 0                   # Get status of axis 0
STAT ALL                 # Get status of all axes
```

**Response**:
```
OK AXIS:0 POS:25.5 TGT:100.0 VEL:15.2 MOVING:1 ENABLED:1 COMPLETE:0 FAULT:0
```

### INFO - Get System Information  
```
INFO
```

**Description**: Returns system identification and configuration information.

**Response Fields**:
- System name and version
- Number and types of axes
- System ready state
- Emergency stop status
- Available memory
- Uptime

**Example**:
```
INFO
```

**Response**:
```
OK YAROBOT_CONTROL_UNIT V1.0
AXES:6 (SERVO:0-3,STEPPER:4-5)
EMERGENCY:0
MEMORY:145KB/320KB
UPTIME:00:15:32
READY
```

## Configuration Commands

### SETU - Set Units Per Pulse
```
SETU <axis> <units_per_pulse>
```
- Configure the scaling factor

Examples:
```
SETU 0 0.001             # 1 pulse = 0.001mm
SETU 1 0.01              # 1 pulse = 0.01 degrees
```

### SETL - Set Limits
```
SETL <axis> <min> <max>
```
- Set software position limits

Examples:
```
SETL 0 -100 100          # Set axis 0 limits to ±100mm
SETL 1 0 360             # Set axis 1 limits to 0-360°
```

### SETV - Set Velocity Limits
```
SETV <axis> <max_velocity> <max_acceleration>
```

**Description**: Sets maximum velocity and acceleration limits for safe operation. These become the default values when velocity/acceleration parameters are omitted from motion commands.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6)
- `max_velocity`: Maximum allowed velocity (units/sec)
- `max_acceleration`: Maximum allowed acceleration (units/sec²)

**Behavior**:
- Limits motor performance to prevent mechanical damage
- Used as defaults for MOVE/MOVR commands
- Should be set conservatively for initial testing

**Examples**:
```
SETV 0 200 1000          # Max 200mm/s, 1000mm/s²
SETV 1 720 3600          # Max 720°/s (2 rev/s), 3600°/s²
```

**Response**:
```
OK SET                   # Velocity limits updated
```

### SETB - Set Backlash Compensation
```
SETB <axis> <backlash>
```
- Set backlash compensation in units

Examples:
```
SETB 0 0.05              # 0.05mm backlash compensation
```

## Control Commands

### EN - Enable/Disable Motor
```
EN <axis|ALL> <0|1>
```

**Description**: Controls motor enable state via I2C expander outputs. Motors must be enabled before any motion commands.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6) or ALL
- `state`: 0 = disable (motor freewheels), 1 = enable (motor holds position)

**Behavior**:
- Enable signal sent to motor driver via I2C expander
- Disabled motors cannot execute motion commands  
- Position maintained when disabled (open-loop assumption)
- ALL enables/disables all motors simultaneously

**Examples**:
```
EN 0 1                   # Enable axis 0
EN ALL 0                 # Disable all axes (power save)
EN 3 1                   # Enable servo axis 3
```

**Response**:
```
OK ENABLED               # Motor(s) enabled
OK DISABLED              # Motor(s) disabled
```

### BRAKE - Control Brake
```
BRAKE <axis|ALL> <0|1>
```

**Description**: Controls electromagnetic brake for holding position when motor is disabled. Brake control via I2C expander outputs.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6) or ALL
- `state`: 0 = release brake (motor can move), 1 = engage brake (axis locked)

**Behavior**:
- Typically engaged when motor disabled for safety
- Released before motion, engaged after stop
- 24V brake control through relay/driver
- Brake engagement has mechanical delay (~50ms)

**Examples**:
```
BRAKE 0 1                # Engage brake on axis 0
BRAKE ALL 0              # Release all brakes
BRAKE 2 1                # Lock axis 2 in position
```

**Response**:
```
OK BRAKE ENGAGED         # Brake(s) engaged
OK BRAKE RELEASED        # Brake(s) released  
```

## Calibration Commands

### HOME - Home Axis
```
HOME <axis|ALL> [TYPE]
```

**Description**: Executes homing sequence to establish absolute zero reference. Different strategies available based on axis type and hardware.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6) or ALL
- `TYPE`: Homing method - AUTO (default), SWITCH, ZSIGNAL
  - AUTO: Selects best method based on axis type
  - SWITCH: Home to end switch only
  - ZSIGNAL: Home to switch then refine with Z-signal (servos only)

**Behavior**:
- Moves to MIN end switch at reduced speed
- Backs off switch until released
- For ZSIGNAL: Searches for index pulse
- Sets current position as zero
- ALL homes axes sequentially to prevent collisions

**Examples**:
```
HOME 0                   # Auto-home axis 0
HOME ALL                 # Home all axes sequentially  
HOME 1 ZSIGNAL          # Force Z-signal homing for axis 1
HOME 4 SWITCH           # Switch-only homing for stepper
```

**Response**:
```
OK HOMING               # Homing started
OK HOMED POS:0.0        # Successfully homed at position 0
ERROR E007 Calibration required  # No end switch found
```

### CALB - Calibrate Backlash
```
CALB <axis>
```

**Description**: Automatically measures and sets backlash compensation by performing test movements and measuring hysteresis.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6)

**Procedure**:
1. Moves forward set distance
2. Records position after settling
3. Moves backward past start point
4. Moves forward to original distance
5. Measures position difference
6. Sets compensation automatically

**Requirements**:
- Axis must be homed first
- Sufficient travel space (±20mm from current)
- Motor enabled

**Examples**:
```
CALB 0                   # Measure backlash on axis 0
CALB 3                   # Calibrate servo axis 3
```

**Response**:
```
OK CALIBRATING          # Started calibration
OK BACKLASH:0.03        # Measured 0.03mm backlash
ERROR E007 Calibration required  # Axis not homed
```

### ZERO - Set Current Position as Zero
```
ZERO <axis|ALL>
```

**Description**: Sets the current position as the new zero reference without moving the motor. Useful for establishing work coordinates.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6) or ALL

**Behavior**:
- Updates position counter to zero
- Does not affect motor position
- Limits remain relative to new zero
- Previous home position lost
- Use HOME to restore machine zero

**Examples**:
```
ZERO 0                   # Set axis 0 current position as 0
ZERO ALL                 # Zero all axes at current positions
ZERO 2                   # Set new work origin for axis 2
```

**Response**:
```
OK ZEROED               # Position set to zero
```

## I/O Commands

### DIN - Read Digital Input
```
DIN <expander> <pin|ALL>
```

**Description**: Reads digital input state from I2C expander pins. Used for reading end switches, buttons, and auxiliary inputs.

**Parameters**:
- `expander`: I2C expander number (0-1)
  - 0: Address 0x20 (outputs/enables/brakes)
  - 1: Address 0x21 (inputs/switches)
- `pin`: Pin number (0-15) or ALL for all pins

**Pin Assignments (Expander 1)**:
- 0-11: End switches (MIN/MAX for axes 0-5)
- 12: Emergency stop button
- 13-14: User buttons
- 15: Reserved

**Examples**:
```
DIN 1 0                  # Read axis 0 MIN switch
DIN 1 ALL                # Read all inputs
DIN 1 12                 # Check E-stop button
```

**Response**:
```
OK 1                     # Single pin: 1=high (switch active)
OK 0                     # Single pin: 0=low (switch inactive)
OK 0b1010110100110011   # ALL: 16-bit binary state
```

### DOUT - Write Digital Output  
```
DOUT <expander> <pin> <value>
```

**Description**: Controls digital output pins on I2C expanders. Used for auxiliary outputs, status LEDs, and external device control.

**Parameters**:
- `expander`: I2C expander number (0-1)
- `pin`: Pin number (0-15)
- `value`: Output state (0=low, 1=high)

**Pin Assignments (Expander 0)**:
- 0-5: Motor enable outputs (managed by EN command)
- 6-11: Brake control outputs (managed by BRAKE command)
- 12-15: User outputs

**Caution**: Pins 0-11 on Expander 0 are managed by motor control system

**Examples**:
```
DOUT 0 12 1              # Set user output 12 high
DOUT 0 15 0              # Set user output 15 low
```

**Response**:
```
OK OUTPUT               # Output set successfully
ERROR E003 Invalid parameter  # Invalid pin or value
```

## System Commands

### SAVE - Save Configuration
```
SAVE [slot]
```

**Description**: Saves current system configuration to non-volatile storage (NVS). Includes all axis parameters, limits, and calibration data.

**Parameters**:
- `slot`: Configuration slot number (0-9), default 0

**Saved Data**:
- Units per pulse (SETU)
- Position limits (SETL) 
- Velocity/acceleration limits (SETV)
- Backlash compensation (SETB)
- Calibration offsets
- Current positions

**Examples**:
```
SAVE                     # Save to default slot 0
SAVE 1                   # Save to slot 1
SAVE 5                   # Save config backup to slot 5
```

**Response**:
```
OK SAVED TO SLOT 0      # Configuration saved
ERROR E010 Configuration error  # NVS write failed
```

### LOAD - Load Configuration
```
LOAD [slot]
```

**Description**: Restores system configuration from non-volatile storage. Applies all saved parameters and limits.

**Parameters**:
- `slot`: Configuration slot number (0-9), default 0

**Behavior**:
- Loads all axis parameters
- Does NOT restore positions (safety)
- Validates configuration integrity
- Requires motors to be disabled

**Examples**:
```
LOAD                     # Load from default slot 0
LOAD 1                   # Load configuration from slot 1
```

**Response**:
```
OK LOADED FROM SLOT 0   # Configuration restored
ERROR E010 Configuration error  # Invalid or corrupted data
```

### RST - Reset Controller
```
RST [FACTORY]
```

**Description**: Resets the controller state or restores factory defaults. Clears emergency stop conditions.

**Parameters**:
- `FACTORY`: Optional flag to reset all settings to factory defaults

**Behavior (Normal Reset)**:
- Clears emergency stop flag
- Cancels all active motions
- Maintains configuration
- Re-initializes hardware

**Behavior (Factory Reset)**:
- Erases all configuration slots
- Restores default parameters
- Requires re-calibration

**Examples**:
```
RST                      # Normal reset, clear E-stop
RST FACTORY              # Factory reset - CAUTION!
```

**Response**:
```
OK RESET                # System reset complete
OK FACTORY RESET        # Factory defaults restored
```

### ECHO - Enable/Disable Command Echo  
```
ECHO <0|1>
```

**Description**: Controls command echo for debugging. When enabled, all received commands are echoed back with '>' prefix.

**Parameters**:
- `0`: Disable echo (default)
- `1`: Enable echo

**Example Session with Echo**:
```
ECHO 1
OK ECHO ON
> POS 0
OK 0:25.5
> MOVE 0 50
OK MOVING
```

**Examples**:
```
ECHO 1                   # Enable command echo
ECHO 0                   # Disable command echo
```

**Response**:
```
OK ECHO ON              # Echo enabled
OK ECHO OFF             # Echo disabled
```

## Diagnostic Commands

### TEST - Run Self-Test
```
TEST [axis|ALL]
```

**Description**: Performs comprehensive self-test of motors, sensors, and I/O. Validates hardware connectivity and basic operation.

**Parameters**:
- `axis`: Single axis (0-5) or ALL for system test

**Test Sequence**:
1. Motor enable/disable cycle
2. Small forward/reverse movement
3. End switch detection
4. Position complete signal (servos)
5. Z-signal detection (servos)
6. Brake operation
7. I2C communication integrity

**Examples**:
```
TEST 0                   # Test axis 0 only
TEST ALL                 # Full system test
```

**Response**:
```
OK TESTING              # Test started
OK AXIS:0 MOTOR:OK SWITCH:OK ZSIGNAL:OK BRAKE:OK
OK AXIS:1 MOTOR:OK SWITCH:FAIL ZSIGNAL:OK BRAKE:OK
ERROR E008 Motor fault  # Motor not responding
```

### LOG - Set Log Level
```
LOG <NONE|ERROR|WARN|INFO|DEBUG>
```

**Description**: Sets the logging verbosity level for diagnostic output. Higher levels include all lower level messages.

**Log Levels**:
- `NONE`: No logging output
- `ERROR`: Critical errors only
- `WARN`: Warnings and errors
- `INFO`: Informational messages (default)
- `DEBUG`: Detailed debug output

**Examples**:
```
LOG ERROR                # Errors only
LOG INFO                 # Normal operation
LOG DEBUG                # Verbose debugging
```

**Response**:
```
OK LOG LEVEL:DEBUG      # Log level set
```

### DIAG - Get Diagnostic Data
```
DIAG <axis>
```

**Description**: Returns detailed diagnostic information for specified axis including counters, errors, and performance metrics.

**Parameters**:
- `axis`: Motor axis letter (X-D) or number (0-6)

**Diagnostic Fields**:
- `PULSES_SENT`: Total pulses generated
- `PULSES_RECEIVED`: Encoder feedback count (if available)
- `Z_COUNT`: Z-signal pulses detected (servos)
- `POS_COMPLETE`: Current state of position complete input
- `DIR_CHANGES`: Direction reversal count
- `ERRORS`: Cumulative error count
- `LAST_ERROR`: Most recent error code
- `RUNTIME`: Axis operation time (HH:MM:SS)
- `MAX_VEL`: Peak velocity achieved

**Examples**:
```
DIAG 0                   # Get diagnostics for axis 0
DIAG 3                   # Get servo axis 3 diagnostics
```

**Response**:
```
OK AXIS:0
PULSES_SENT:125000
PULSES_RECEIVED:124998
Z_COUNT:5
POS_COMPLETE:1
DIR_CHANGES:23
ERRORS:0
LAST_ERROR:NONE
RUNTIME:01:23:45
MAX_VEL:185.3
```

## Streaming Commands

### STREAM - Enable Position Streaming
```
STREAM <interval_ms> [axes]
```

**Description**: Enables automatic streaming of position data at regular intervals. Useful for real-time monitoring and data logging.

**Parameters**:
- `interval_ms`: Update interval in milliseconds (10-5000), 0 to stop
- `axes`: Comma-separated axis list or ALL (default)

**Behavior**:
- Streams position data with 'STRM' prefix
- Non-blocking - continues until stopped
- Minimum interval depends on system load
- Each stream message timestamped

**Examples**:
```
STREAM 100               # Stream all axes at 10Hz
STREAM 50 X,Y,Z         # Stream axes X,Y,Z at 20Hz  
STREAM 1000 C            # Stream axis C at 1Hz (includes object width)
STREAM 0                 # Stop all streaming
```

**Stream Output Format**:
```
STRM X:25.50,Y:180.00,Z:0.00,A:-45.25,B:10.00,C:0.00,D:100.00
STRM X:26.15,Y:180.00,Z:0.50,A:-45.25,B:10.00,C:0.00,D:100.00
```

**Response**:
```
OK STREAMING AT 100ms   # Streaming started
OK STREAM STOPPED       # Streaming stopped
```

## Event System

Events are asynchronous notifications sent when significant system events occur:

### Event Format
```
EVENT <type> <axis> <data>
```

### Event Types

#### OBJECT_DETECTED (C axis only)
```
EVENT OBJECT_DETECTED C WIDTH:12.5 TIME:1234567890
```
Triggered when C axis floating switch detects object during gripping.

#### POSITION_REACHED
```
EVENT POSITION_REACHED X POS:100.0 TIME:1234567890
```
Triggered when axis reaches target position (servo axes with position complete signal).

#### LIMIT_REACHED
```
EVENT LIMIT_REACHED Y LIMIT:MIN TIME:1234567890
```
Triggered when axis hits end switch.

#### E_STOP
```
EVENT E_STOP ALL TIME:1234567890
```
Triggered when emergency stop is activated.

#### I2C_ERROR
```
EVENT I2C_ERROR DEVICE:0x20 CODE:0x03 TIME:1234567890
```
Triggered on I2C communication failure.

## ROS2 Interface Notes

### Design Philosophy
- Universal real-time control device with ROS2 compatibility
- All axes provide uniform position/velocity interfaces
- Serial interface for development, ROS2 for production
- Events map to ROS2 topics
- Commands map to ROS2 services

### Testing Phases
1. **Phase 1**: Serial command testing via USB CDC
2. **Phase 2**: Mock ROS2 interface validation  
3. **Phase 3**: Full ros2_control hardware interface

### Future ROS2 Mappings
- Position states → `/joint_states` topic (50-100Hz)
- Velocity commands → `/cmd_vel` topic
- Events → Individual topics per event type
- Services → Move, Home, Calibrate operations

## Error Codes

| Code | Description |
|------|-------------|
| E001 | Invalid command |
| E002 | Invalid axis letter/number |
| E003 | Invalid parameter |
| E004 | Axis not enabled |
| E005 | Position limit exceeded |
| E006 | Emergency stop active |
| E007 | Calibration required |
| E008 | Motor fault |
| E009 | Communication error |
| E010 | Configuration error |
| E011 | Event buffer overflow |

## Command Examples Session

```
> INFO
< OK YAROBOT_CONTROL_UNIT V1.0
< AXES:6 (SERVO:0-3,STEPPER:4-5)
< READY

> EN ALL 1
< OK

> HOME 0
< OK HOMING
< OK HOMED POS:0.0

> MOVE 0 50
< OK MOVING

> POS 0
< OK 0:25.5

> STAT 0
< OK AXIS:0 POS:25.5 TGT:50.0 VEL:15.2 MOVING:1 ENABLED:1 COMPLETE:0

> POS 0
< OK 0:50.0

> STAT 0
< OK AXIS:0 POS:50.0 TGT:50.0 VEL:0.0 MOVING:0 ENABLED:1 COMPLETE:1
```

## Implementation Notes

1. Commands processed in order received
2. Motion commands are non-blocking (queued)
3. Status commands are immediate
4. USB buffer: 256 bytes max per command
5. Response timeout: 100ms typical
6. All positions in user units (mm/degrees)
7. Float parameters support decimal notation