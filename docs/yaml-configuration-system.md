# YAML Configuration System

## Overview
The YaRobot Control Unit uses YAML-based configuration for all system parameters, providing a human-readable, version-controllable configuration system that can be updated via USB without recompilation.

## Configuration Modes

### 1. OPERATION Mode (Default)
- Normal command processing
- Motion control active
- Configuration read-only

### 2. CONFIG Mode
- Motion disabled
- Accepts YAML uploads
- Configuration validation and storage

## Mode Switching Methods

### Method 1: Boot Pin
Hold GPIO45 LOW during reset to enter CONFIG mode.
```
┌─────────────┐
│   GPIO45    │──┐
│             │  │
│   ESP32-S3  │  ┴ 10kΩ → GND (CONFIG mode)
│             │  
│             │  ┴ 10kΩ → 3.3V (Normal boot)
└─────────────┘
```

### Method 2: Command Interface
```bash
# Enter configuration mode
MODE CONFIG
> CONFIRM MODE CONFIG? (Y/N)
Y
> OK CONFIG MODE ACTIVE

# Exit configuration mode
MODE OPERATE
> OK OPERATION MODE ACTIVE
```

### Method 3: Automatic Detection
System enters CONFIG mode if no valid configuration found in storage.

## YAML Transfer Protocol

### 1. Start Transfer
```bash
YAML START [size]
> READY YAML 4096 BYTES MAX
```

### 2. Send YAML Data
```bash
YAML DATA
<YAML content here>
YAML END
> OK YAML RECEIVED 1234 BYTES
```

### 3. Validate Configuration
```bash
YAML VALIDATE
> OK YAML VALID
# or
> ERROR YAML LINE 23: Invalid ratio value
```

### 4. Apply Configuration
```bash
YAML APPLY
> OK YAML APPLIED
> REBOOT REQUIRED
```

## YAML Configuration Structure

```yaml
# YaRobot Control Unit Configuration
version: "1.0"
system:
  name: "YaRobot-001"
  mode: "standalone"  # or "ros2"
  
motion:
  # Global motion parameters
  default_acceleration: 5000.0  # steps/s²
  default_max_velocity: 10000.0 # steps/s
  emergency_deceleration: 20000.0 # steps/s²

axes:
  X:
    # Axis identifiers
    letter: "X"      # G-code letter (for clarity in config)
    number: 0        # Internal axis number (0-7)
    alias: "railway" # Single friendly name for commands/display
    
    type: "servo"
    peripheral: "RMT_CH0"
    # Motion parameters
    ratio: 1.0
    max_velocity: 15000.0
    acceleration: 8000.0
    
    # Limit switches
    limits:
      min:
        action: "stop"  # stop, estop, event
        inverted: false
      max:
        action: "estop"
        inverted: false
    
    # Signal polarity
    signals:
      step_inverted: false
      dir_inverted: false
      enable_inverted: true  # Active low
      
    # Servo feedback configuration (optional)
    servo:
      # Z-signal (index pulse) configuration
      z_signal:
        enabled: true           # Use Z-signal for position verification
        pin: "GPIO40"          # GPIO pin for Z-signal input (if not using default)
        tolerance: 5           # Position error tolerance (encoder counts)
        counts_per_revolution: 10000
        inverted: false        # Signal polarity
        
      # InPos (position complete) configuration  
      inpos:
        enabled: true          # Use InPos signal from servo driver
        pin: "0x22.GPA0"      # I2C expander pin (if not using default)
        settling_ms: 100       # Time to wait after InPos before confirming
        inverted: false        # Signal polarity
        timeout_ms: 5000      # Max time to wait for InPos
        required: false       # If true, motion blocks until InPos confirmed
      
    # Homing
    homing:
      enabled: true
      direction: "negative"  # toward min limit
      fast_velocity: 5000.0
      slow_velocity: 500.0
      backoff_distance: 100.0
      
  Y:
    letter: "Y"
    number: 1
    alias: "selector"
    type: "servo"
    peripheral: "MCPWM0_0"
    # ... similar structure ...
    
  C:
    type: "stepper"
    peripheral: "MCPWM0_1"
    # Stepper-specific
    stepper:
      has_floating_switch: true
      measure_on_close: true
      
  D:
    type: "stepper"
    peripheral: "LEDC_CH0"
    stepper:
      discrete_positions: [0, 500, 1000, 1500, 2000]
      position_names: ["HOME", "LOAD", "PICK", "PLACE", "EJECT"]
      
  E:
    type: "discrete"
    control: "I2C_EXPANDER"
    discrete:
      speed: 100.0  # units/second
      min_travel: 0.0
      max_travel: 2000.0
      timeout_ms: 30000

# I/O Expander Configuration
io_expanders:
  - address: 0x20
    type: "MCP23017"
    description: "Limit switches"
    interrupt_pin: "GPIO3"
    # Named inputs - limit switches
    inputs:
      GPA0: { number: 0, alias: "railway_min" }
      GPA1: { number: 1, alias: "railway_max" }
      GPA2: { number: 2, alias: "selector_min" }
      GPA3: { number: 3, alias: "selector_max" }
      GPA4: { number: 4, alias: "picker_z_min" }
      GPA5: { number: 5, alias: "picker_z_max" }
      GPA6: { number: 6, alias: "picker_a_min" }
      GPA7: { number: 7, alias: "picker_a_max" }
      GPB0: { number: 8, alias: "picker_b_min" }
      GPB1: { number: 9, alias: "picker_b_max" }
      GPB2: { number: 10, alias: "jaw_min" }
      GPB3: { number: 11, alias: "jaw_float" }  # Floating switch
      GPB4: { number: 12, alias: "retractor_min" }
      GPB5: { number: 13, alias: "retractor_max" }
      GPB6: { number: 14, alias: "linear_min" }
      GPB7: { number: 15, alias: "linear_max" }
    
  - address: 0x21
    type: "MCP23017"
    description: "General I/O + E axis"
    interrupt_pin: "GPIO46"
    inputs:
      GPA0: { number: 16, alias: "start_button" }
      GPA1: { number: 17, alias: "stop_button" }
      GPA2: { number: 18, alias: "mode_switch" }
      GPA3: { number: 19, alias: "spare_in_3" }
      GPA4: { number: 20, alias: "spare_in_4" }
      GPA5: { number: 21, alias: "spare_in_5" }
      GPA6: { number: 22, alias: "spare_in_6" }
      GPA7: { number: 23, alias: "spare_in_7" }
    outputs:
      GPB0: { number: 0, alias: "ready_led" }
      GPB1: { number: 1, alias: "busy_led" }
      GPB2: { number: 2, alias: "error_led" }
      GPB3: { number: 3, alias: "spare_out_3" }
      GPB4: { number: 4, alias: "spare_out_4" }
      GPB5: { number: 5, alias: "spare_out_5" }
      GPB6: { number: 6, alias: "linear_enable" }  # E axis
      GPB7: { number: 7, alias: "linear_dir" }     # E axis
    
  - address: 0x22
    type: "MCP23017"
    description: "Servo feedback"
    inputs:
      GPA0: { number: 24, alias: "railway_ready" }
      GPA1: { number: 25, alias: "selector_ready" }
      GPA2: { number: 26, alias: "picker_z_ready" }
      GPA3: { number: 27, alias: "picker_a_ready" }
      GPA4: { number: 28, alias: "picker_b_ready" }

# Brake configuration
brakes:
  strategy: "on_disable"  # on_disable, on_estop, on_idle, never
  idle_timeout_ms: 5000
  axes_with_brakes: ["X", "Y", "Z", "A", "B"]

# Communication
communication:
  usb:
    baudrate: 115200
    echo: false
    
  ros2:
    enabled: false
    node_name: "yarobot_hw"
    namespace: "/yarobot"

# Safety
safety:
  estop_action: "brake_all"  # brake_all, disable_all
  position_loss_action: "estop"
  i2c_failure_action: "stop_affected"
  
# Display
display:
  type: "SSD1306"
  address: 0x3C
  update_rate_hz: 10
  pages: ["status", "positions", "errors"]
```

## Configuration Storage

### SPIFFS Filesystem
```
/config/
├── current.yaml      # Active configuration
├── default.yaml      # Factory defaults
├── backup.yaml       # Last known good
└── schema.yaml       # Validation schema
```

### NVS Backup
Critical parameters backed up in NVS:
- Axis ratios
- Limit configurations
- Last known positions

## Unified Naming System

### Overview
The YaRobot uses a simplified three-identifier system for all axes and I/O:
1. **Number**: Internal index for efficient array access
2. **Letter/Pin**: Traditional identifier (X,Y,Z for axes, GPA0 for I/O)
3. **Alias**: Single friendly name for commands and display

### Axis Naming
```yaml
# Format: number, letter, alias
X: { number: 0, alias: "railway" }
Y: { number: 1, alias: "selector" }
Z: { number: 2, alias: "lift" }
A: { number: 3, alias: "picker_z" }
B: { number: 4, alias: "picker_y" }
C: { number: 5, alias: "jaw" }
D: { number: 6, alias: "retractor" }
E: { number: 7, alias: "linear" }
```

### I/O Naming
```yaml
# Inputs: number, pin, alias
GPA0: { number: 0, alias: "railway_min" }
GPB3: { number: 11, alias: "jaw_float" }

# Outputs: number, pin, alias  
GPB0: { number: 0, alias: "ready_led" }
GPB6: { number: 6, alias: "linear_enable" }
```

### Command Usage
All three identifiers work in commands:
```bash
# Axis movement - all equivalent
G00 X100        # G-code letter
G00 0 100       # By number
MOVE railway 100 # By alias

# I/O commands - all equivalent  
IN GPA0         # By pin name
IN 0            # By number
IN railway_min  # By alias

OUT ready_led 1 # Set output by alias
OUT GPB0 1      # Set output by pin
OUT 0 1         # Set output by number
```

### Event Messages
Events use the same identifier that triggered them:
```bash
# If commanded by alias
MOVE railway 100
> EVENT MOTION_START railway
> EVENT POSITION_REACHED railway

# If commanded by letter
G00 X100
> EVENT MOTION_START X
> EVENT POSITION_REACHED X

# Limit switch events always use alias
> EVENT LIMIT_ACTIVATED railway_min
> EVENT INPUT_CHANGED start_button HIGH
```

### Status Responses
Status uses the same format as the query:
```bash
# Query by letter
? X
> X:125.5

# Query by alias  
? railway
> railway:125.5

# Query all (uses aliases)
?
> railway:125.5 selector:50.0 lift:75.3 jaw:30.0

# I/O status
IN railway_min
> railway_min:1

IN GPA0
> GPA0:1
```

### Error Messages
Errors include both identifiers for clarity:
```bash
> ERROR: railway (X) limit switch activated
> ERROR: jaw_float (GPB3) triggered during close
> ERROR: start_button (GPA0) stuck high
```

### Implementation Benefits
1. **Consistency**: Same name in command = same name in response
2. **Flexibility**: Use whatever identifier is convenient
3. **Clarity**: Aliases make logs and errors self-documenting
4. **Efficiency**: Numbers enable fast array indexing
5. **Compatibility**: G-code and pin names still work

## USB Transfer Examples

### Query Commands in Operation Mode

```bash
# Get axis mappings
MAP AXES
> AXIS 0 X "railway"
> AXIS 1 Y "selector"
> ...

# Export current config
YAML EXPORT
> YAML START
> # YaRobot Control Unit Configuration
> version: "1.0"
> ...
> YAML END

# Get version info
VERSION
> YAROBOT V1.0.0
> CONFIG: current.yaml (2024-01-15 14:32:10)
> MODE: OPERATION
```

### Using Serial Terminal
```bash
# 1. Enter config mode
MODE CONFIG
CONFIRM MODE CONFIG? (Y/N)
Y

# 2. Start YAML transfer
YAML START 2048
READY YAML 4096 BYTES MAX

# 3. Paste YAML content
YAML DATA
version: "1.0"
system:
  name: "YaRobot-Test"
...
YAML END

# 4. Validate
YAML VALIDATE
OK YAML VALID

# 5. Apply and reboot
YAML APPLY
OK YAML APPLIED
REBOOT
```

### Using Python Script
```python
import serial
import yaml
import time

def upload_config(port, yaml_file):
    ser = serial.Serial(port, 115200, timeout=1)
    
    # Enter config mode
    ser.write(b"MODE CONFIG\n")
    time.sleep(0.1)
    ser.write(b"Y\n")
    
    # Load YAML
    with open(yaml_file, 'r') as f:
        config = f.read()
    
    # Start transfer
    ser.write(f"YAML START {len(config)}\n".encode())
    time.sleep(0.1)
    
    # Send data
    ser.write(b"YAML DATA\n")
    ser.write(config.encode())
    ser.write(b"\nYAML END\n")
    
    # Validate and apply
    ser.write(b"YAML VALIDATE\n")
    response = ser.readline().decode()
    
    if "OK" in response:
        ser.write(b"YAML APPLY\n")
        print("Configuration uploaded successfully")
    else:
        print(f"Validation error: {response}")
    
    ser.close()

# Usage
upload_config('/dev/ttyUSB0', 'robot_config.yaml')
```

## Configuration Validation

### Schema Validation
- Required fields check
- Type validation
- Range validation
- Dependency checks

### Hardware Validation
- Peripheral availability
- GPIO conflicts
- I2C address verification
- Axis interdependencies

### Example Validation Errors
```
ERROR YAML LINE 45: axis.X.ratio must be between 0.1 and 10.0
ERROR YAML LINE 67: Peripheral MCPWM0_0 already assigned to axis Y
ERROR YAML LINE 89: GPIO3 already used for INT0
ERROR YAML: Missing required field 'axes.X.type'
```

## Mode Indicators

### LED Indicators
- **CONFIG Mode**: Blue LED blinking
- **OPERATION Mode**: Green LED solid
- **ERROR**: Red LED blinking

### OLED Display
```
┌─────────────────┐
│  CONFIG MODE    │
│  -------------  │
│  YAML: READY    │
│  Size: 0/4096   │
│                 │
│  [ESC] Cancel   │
└─────────────────┘
```

## Safety Features

1. **Validation Before Apply**
   - Never applies invalid configuration
   - Detailed error reporting

2. **Rollback Capability**
   ```bash
   YAML ROLLBACK
   > OK CONFIGURATION RESTORED
   ```

3. **Factory Reset**
   ```bash
   YAML FACTORY
   > CONFIRM FACTORY RESET? (Y/N)
   ```

4. **Configuration Lock**
   - Can lock configuration in production
   - Requires special unlock sequence

## Best Practices

1. **Version Control**
   - Keep YAML configs in git
   - Tag tested configurations
   
2. **Incremental Updates**
   - Test changes on single axis first
   - Validate on bench before deployment
   
3. **Documentation**
   - Comment complex settings
   - Document measurement units
   
4. **Backup Strategy**
   - Export config before changes
   - Keep known-good versions

## Integration with ROS2

When `communication.ros2.enabled: true`, additional parameters are exposed:
- Dynamic reconfigure for ratios
- Parameter server for limits
- Service calls for mode changes