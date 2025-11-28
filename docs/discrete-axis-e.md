# Discrete Axis E - Linear Drive Control

## Overview
Axis E provides control for a discrete two-state linear drive with constant speed operation. Unlike other axes that use motor drivers, this axis directly controls enable and direction signals through I/O expanders, making it suitable for pneumatic cylinders, linear actuators, or simple DC motor drives.

## Hardware Configuration

### Output Signals (via MCP23017 #2 at 0x21)
```
GPB6: E_ENABLE  - Enable/disable movement
GPB7: E_DIR     - Direction (extend/retract)
```

### Input Signals (via MCP23017 #1 at 0x20)
```
GPB6: E_LIMIT_MIN - Minimum position limit switch
GPB7: E_LIMIT_MAX - Maximum position limit switch
```

## Characteristics

### Motion Profile
- **Speed**: Constant velocity (configured, not variable)
- **Acceleration**: Instantaneous (on/off control)
- **Position**: Calculated based on time and configured speed
- **Feedback**: Limit switches only (no encoder)

### Operating Modes
1. **IDLE**: Not moving, enable = OFF
2. **EXTENDING**: Moving towards MAX limit, dir = 1
3. **RETRACTING**: Moving towards MIN limit, dir = 0
4. **AT_LIMIT**: Reached limit switch, auto-stop

## Command Interface

### Standard Motion Commands
```
# Absolute positioning (calculated)
G00 E<position>
Example: G00 E1000  # Move to calculated position 1000

# Relative positioning  
G91
G00 E100           # Move 100 units forward
G90

# Continuous motion
JOG E <direction>  # 1=extend, -1=retract, 0=stop
```

### Status Reporting
```
Command: ?
Response includes: E:500.0  # Calculated position

Command: STATUS E
Response: 
  E POS:500.0 TGT:1000.0 STATE:EXTENDING
  E MIN:0 MAX:1 ENABLED:1
```

### Configuration Commands
```
# Set movement speed (units/second)
SPEED E <value> [SAVE]
Example: SPEED E 100.0      # 100 units/second
Default: 50.0

# Set travel limits (calculated positions)
TRAVEL E <min> <max>
Example: TRAVEL E 0 2000    # 0 to 2000 units
```

## Position Calculation

Since axis E has no encoder feedback, position is calculated:

```c
typedef struct {
    float current_position;      // Calculated position
    float target_position;       // Commanded target
    float speed;                // Units per second
    TickType_t move_start_time; // Movement timestamp
    direction_t direction;       // Current direction
    bool at_min_limit;          // MIN limit switch state
    bool at_max_limit;          // MAX limit switch state
} axis_e_state_t;

// Position update (called at 100Hz)
void update_axis_e_position() {
    if (state.moving) {
        float elapsed = (xTaskGetTickCount() - move_start_time) / 1000.0;
        float distance = state.speed * elapsed;
        
        if (state.direction == DIR_EXTEND) {
            state.current_position = state.start_position + distance;
        } else {
            state.current_position = state.start_position - distance;
        }
        
        // Clamp to configured limits
        state.current_position = clamp(state.current_position, 
                                     config.min_travel, 
                                     config.max_travel);
    }
}
```

## Limit Switch Behavior

### Automatic Stopping
- Hitting MIN limit during retraction: Stop, set position to 0
- Hitting MAX limit during extension: Stop, set position to max_travel
- Opposite direction movement allowed from limits

### Homing Sequence
```
1. If not at MIN limit: Retract until MIN limit
2. Set current_position = 0
3. Optional: Extend to known position for verification
```

## Safety Features

1. **Limit Protection**: Cannot command beyond limit switches
2. **Position Sync**: Limit switches recalibrate calculated position
3. **Timeout Protection**: Maximum move time before error
4. **E-Stop Integration**: E_ENABLE immediately set to OFF

## Event Generation

```c
// E-axis specific events
EVENT_E_LIMIT_MIN     // Hit minimum limit
EVENT_E_LIMIT_MAX     // Hit maximum limit  
EVENT_E_POSITION      // Reached target (calculated)
EVENT_E_TIMEOUT       // Move timeout exceeded
```

## Implementation Structure

```c
// Axis E configuration
typedef struct {
    float speed;            // Movement speed (units/s)
    float min_travel;       // Minimum position (typically 0)
    float max_travel;       // Maximum position
    float position_tolerance; // "Close enough" threshold
    uint32_t timeout_ms;    // Maximum move duration
    bool invert_enable;     // Signal polarity
    bool invert_direction;  // Signal polarity
    bool invert_min_limit;  // Switch polarity
    bool invert_max_limit;  // Switch polarity
} axis_e_config_t;

// Control functions
void axis_e_move_to(float position);
void axis_e_jog(direction_t dir);
void axis_e_stop(void);
void axis_e_home(void);
bool axis_e_update(void);  // Called from task loop
```

## Integration Notes

1. **Unified Interface**: Uses same command parser as other axes
2. **Status Reporting**: Included in standard ? response
3. **Event System**: Generates standard motion events
4. **Error Handling**: Standard error codes apply

## Use Cases

- Pneumatic gripper positioning
- Part ejector mechanisms  
- Simple slide positioning
- Gate/door control
- Material pusher systems

## Limitations

1. **No precise positioning**: Calculated only, ±5% typical accuracy
2. **No velocity control**: Fixed speed operation
3. **No intermediate feedback**: Only end positions known
4. **Power loss**: Position lost, requires re-homing