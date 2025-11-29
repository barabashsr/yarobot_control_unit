# Configuration Commands

## Overview
This document defines configuration commands for the YaRobot Control Unit, covering motion parameters, limit switch behavior, signal polarity, and runtime adjustments. These commands support both ROS2 lifecycle management and runtime tuning.

**Note**: The primary configuration method is via YAML files (see [yaml-configuration-system.md](yaml-configuration-system.md)). The commands below provide runtime adjustments and query capabilities.

## Command Categories

### 1. Position and Calibration Commands

#### CLEAR - Clear Position
Resets axis position to zero without physical movement.
```
Command: CLEAR <axis>
Example: CLEAR X
Response: OK CLEAR X
Usage: During on_configure() or after position loss
```

#### CALZ - Calibrate with Z-Signal  
Initiates Z-signal calibration sequence for servo axes.
```
Command: CALZ <axis>
Example: CALZ Y
Response: OK CALZ Y (on success)
         ERROR CALZ Y NO_Z_SIGNAL (if no pulse detected)
         ERROR CALZ Y NOT_SERVO (for stepper axes)
```

### 2. Motion Parameter Commands

#### RATIO - Set Axis Scaling Ratio
Adjusts encoder/step scaling factor. Can be changed at runtime when axis is idle.
```
Command: RATIO <axis> <value> [SAVE]
Example: RATIO X 1.2345      # Temporary adjustment
         RATIO X 1.2345 SAVE # Persist to NVS
Response: OK RATIO X 1.2345
Range: 0.1 to 10.0
Default: 1.0
```

#### ACCEL - Set Acceleration Limit
```
Command: ACCEL <axis> <value> [SAVE]
Example: ACCEL X 1000
Response: OK ACCEL X 1000
Units: steps/s² or encoder counts/s²
```

#### MAXVEL - Set Maximum Velocity
```
Command: MAXVEL <axis> <value> [SAVE]
Example: MAXVEL X 5000  
Response: OK MAXVEL X 5000
Units: steps/s or encoder counts/s
```

### 3. Limit Switch Configuration

#### LIMCFG - Configure Limit Switch Behavior
Defines how axis responds to limit switch activation.
```
Command: LIMCFG <axis> <MIN|MAX> <mode>
Modes:
  - ESTOP: Trigger system-wide emergency stop
  - STOP: Stop only this axis, continue others
  - EVENT: Report event only, no automatic action
  
Example: LIMCFG X MIN STOP
         LIMCFG X MAX ESTOP
Response: OK LIMCFG X MIN STOP
Default: STOP for all limits
```

#### LIMMAP - Map Limit Switches to I/O (Compile-time Configuration)
Note: Limit switch mapping is defined at compile-time in the hardware configuration.
Physical pin assignments are fixed to prevent runtime safety issues.
See gpio-allocation-7motors.md for the mapping.

### 4. Servo Feedback Configuration

#### FEEDBACK - Configure Servo Feedback Signals
Enables or disables Z-signal and InPos for servo axes.
```
Command: FEEDBACK <axis> <Z|INPOS> <ON|OFF> [SAVE]
Example: FEEDBACK X Z ON          # Enable Z-signal for X
         FEEDBACK Y INPOS OFF     # Disable InPos for Y
         FEEDBACK X Z ON SAVE     # Enable and persist
Response: OK FEEDBACK X Z ON

Command: FEEDBACK <axis>          # Query current settings
Example: FEEDBACK X
Response: X Z:ON INPOS:ON
```

#### ZCONFIG - Configure Z-Signal Parameters
```
Command: ZCONFIG <axis> TOLERANCE <counts>
Example: ZCONFIG X TOLERANCE 10
Response: OK ZCONFIG X TOLERANCE 10
```

#### IPOSCONFIG - Configure InPos Parameters  
```
Command: IPOSCONFIG <axis> <param> <value>
Parameters:
  - SETTLING: Settling time in ms
  - TIMEOUT: Timeout in ms
  - REQUIRED: ON/OFF (block until confirmed)

Example: IPOSCONFIG X SETTLING 150
         IPOSCONFIG Y REQUIRED ON
Response: OK IPOSCONFIG X SETTLING 150
```

### 5. Signal Polarity Configuration

#### POLINV - Invert Signal Polarity
Configures active-high vs active-low logic for various signals.
```
Command: POLINV <axis> <signal> <ON|OFF>
Signals:
  - STEP: Step pulse polarity
  - DIR: Direction signal polarity  
  - EN: Enable signal polarity
  - LIMMIN: Minimum limit switch
  - LIMMAX: Maximum limit switch
  - ZSIG: Z-signal pulse (servos only)
  - INPOS: Position complete signal (servos only)

Example: POLINV X EN ON       # Enable is now active-low
         POLINV Y LIMMIN ON   # Limit switch is normally-closed
Response: OK POLINV X EN ON
Default: OFF (all signals active-high)
```

### 5. Query Commands

#### CONFIG - Query Configuration
Returns current configuration for specified scope.
```
Command: CONFIG <axis|LIMITS|POLARITY|ALL>
Example: CONFIG X
Response: 
  X RATIO:1.2345 ACCEL:1000 MAXVEL:5000
  X LIMMIN:STOP LIMMAX:ESTOP  
  X STEP:NORMAL DIR:NORMAL EN:INVERTED
  
Example: CONFIG LIMITS
Response:
  LIMITS X MIN:0x20.GPA0 MAX:0x20.GPA1
  LIMITS Y MIN:0x20.GPA2 MAX:0x20.GPA3
  ...
```

#### MAP - Query Name Mappings
Returns identifier mappings for axes and I/O.
```
Command: MAP [AXES|INPUTS|OUTPUTS|ALL]

Example: MAP AXES
Response:
  AXIS 0 X "railway"
  AXIS 1 Y "selector"
  AXIS 2 Z "lift"
  AXIS 3 A "picker_z"
  AXIS 4 B "picker_y"
  AXIS 5 C "jaw"
  AXIS 6 D "retractor"
  AXIS 7 E "linear"

Example: MAP INPUTS
Response:
  INPUT 0 0x20.GPA0 "railway_min"
  INPUT 1 0x20.GPA1 "railway_max"
  INPUT 2 0x20.GPA2 "selector_min"
  ...
  INPUT 11 0x20.GPB3 "jaw_float"
  INPUT 16 0x21.GPA0 "start_button"

Example: MAP ALL
Response:
  # Axes
  AXIS 0 X "railway"
  AXIS 1 Y "selector"
  ...
  # Inputs
  INPUT 0 0x20.GPA0 "railway_min"
  ...
  # Outputs
  OUTPUT 0 0x21.GPB0 "ready_led"
  OUTPUT 1 0x21.GPB1 "busy_led"
  ...
```

#### YAML - Export Current Configuration
Exports the complete configuration in YAML format.
```
Command: YAML EXPORT [section]

Example: YAML EXPORT
Response:
YAML START
# YaRobot Control Unit Configuration
# Generated: 2024-01-15 14:32:10
version: "1.0"
system:
  name: "YaRobot-001"
  mode: "standalone"
...
[complete YAML configuration]
YAML END

Example: YAML EXPORT AXES
Response:
YAML START
axes:
  X:
    letter: "X"
    number: 0
    alias: "railway"
    type: "servo"
    ratio: 1.2345
    ...
YAML END

Example: YAML EXPORT IO
Response:
YAML START
io_expanders:
  - address: 0x20
    inputs:
      GPA0: { number: 0, alias: "railway_min" }
      ...
YAML END
```

### 6. Persistence Commands

#### SAVE - Save Configuration
Writes current configuration to NVS.
```
Command: SAVE [axis|ALL]
Example: SAVE X      # Save only X axis config
         SAVE ALL    # Save all axes
Response: OK SAVE X
```

#### LOAD - Load Configuration  
Restores configuration from NVS.
```
Command: LOAD [axis|ALL]
Example: LOAD X
Response: OK LOAD X
```

## Runtime Configuration Rules

### When Configuration Can Change

| Command | During Motion | During E-Stop | ROS2 State Required |
|---------|--------------|---------------|-------------------|
| CLEAR   | No           | No            | INACTIVE          |
| CALZ    | No           | No            | INACTIVE          |
| RATIO   | No           | Yes           | Any               |
| ACCEL   | No           | Yes           | Any               |
| MAXVEL  | No           | Yes           | Any               |
| LIMCFG  | No           | Yes           | INACTIVE          |
| LIMMAP  | N/A (compile-time) | N/A     | N/A               |
| POLINV  | No           | No            | UNCONFIGURED      |

### Configuration Validation

1. **Limit Switch Mapping**
   - Cannot unmap a limit switch that's currently active
   - Each limit must be mapped before axis can be enabled
   - Duplicate mappings are rejected

2. **Ratio Constraints**
   - Must be positive and within range [0.1, 10.0]
   - Changes take effect on next move command
   - Affects position reporting immediately

3. **Polarity Changes**
   - Only allowed when axis is disabled
   - Requires confirmation for safety-critical signals (EN, limits)
   - Changes take effect immediately

## Default Configuration

```c
// Default values for all axes
typedef struct {
    float ratio = 1.0;
    float max_velocity = 10000.0;    // steps/s
    float acceleration = 5000.0;     // steps/s²
    
    // Limit behavior
    limit_action_t min_limit_action = LIMIT_STOP;
    limit_action_t max_limit_action = LIMIT_STOP;
    
    // Signal polarity (false = active high)
    bool invert_step = false;
    bool invert_dir = false;
    bool invert_enable = false;
    bool invert_min_limit = false;
    bool invert_max_limit = false;
    bool invert_z_signal = false;
    bool invert_inpos = false;
} axis_config_t;
```

## Error Responses

| Error | Description |
|-------|-------------|
| ERROR INVALID_AXIS | Unknown axis identifier |
| ERROR AXIS_MOVING | Cannot change while in motion |
| ERROR OUT_OF_RANGE | Parameter exceeds valid range |
| ERROR NOT_MAPPED | Limit switch not mapped |
| ERROR SAVE_FAILED | NVS storage error |
| ERROR NOT_SERVO | Command only valid for servo axes |
| ERROR ESTOP_ACTIVE | Cannot configure during E-stop |

## Implementation Notes

1. **NVS Storage**
   - Configuration stored with CRC32 validation
   - Namespace: "yarobot_cfg"
   - Key format: "axis_X_cfg" 

2. **Thread Safety**
   - Configuration changes are mutex-protected
   - Applied atomically at motion boundaries

3. **Event Generation**
   - CONFIG_CHANGED events notify subsystems
   - Include axis and changed parameters

4. **ROS2 Integration**
   - Configuration can be set via ROS2 parameters
   - Changes during runtime require reconfigure callback