# Motor Channel Letter Mapping

## Overview
Motor channel identification using letter designations X, Y, Z, A, B, C, D for the yarobot_control_unit 7-motor system.

## Channel Assignments

| Channel | Motor Type | Peripheral | Control Features | Physical Function |
|---------|-----------|-----------|------------------|-------------------|
| **X** | Servo | RMT CH0 + DMA | Full position, Z-signal, Pos complete | Railway X axis |
| **Y** | Servo | MCPWM0 T0 + PCNT0 | Full position, Z-signal, Pos complete | Gripper Y axis |
| **Z** | Servo | RMT CH1 + DMA | Full position, Z-signal, Pos complete | Selector Z axis |
| **A** | Servo | RMT CH2 + DMA | Full position, Z-signal, Pos complete | Picker Z axis |
| **B** | Servo | RMT CH3 + DMA | Full position, Z-signal, Pos complete | Picker Y axis |
| **C** | Stepper | MCPWM0 T1 + PCNT1 | Position + velocity, floating switch | Picker jaw (object measurement) |
| **D** | Stepper | LEDC | Position + velocity (calculated), discrete | Picker retractor |

## Channel Characteristics

### X-Axis (Servo) - Railway X axis
- **Type**: High-precision servo motor
- **Peripheral**: RMT Channel 0 + DMA
- **Features**:
  - Hardware pulse generation with DMA
  - Z-signal index pulse for absolute positioning
  - Position complete feedback from driver
  - High-speed linear motion for railway positioning
  - ROS2 position/velocity interfaces

### Y-Axis (Servo) - Gripper Y axis
- **Type**: Precision servo motor (slower movements)
- **Peripheral**: MCPWM0 Timer 0 + PCNT0 (dedicated)
- **Features**:
  - Fixed peripheral (no sharing)
  - Optimized for smooth, slow gripper movements
  - Hardware position counting
  - Precise object manipulation
  - ROS2 position/velocity interfaces

### Z-Axis (Servo) - Selector Z axis
- **Type**: High-precision servo motor
- **Peripheral**: RMT Channel 1 + DMA
- **Features**:
  - High-speed vertical motion for object selection
  - Hardware pulse generation with DMA
  - Z-signal for homing accuracy
  - Safety limits strictly enforced
  - ROS2 position/velocity interfaces

### A-Axis (Servo) - Picker Z axis
- **Type**: High-precision servo motor
- **Peripheral**: RMT Channel 2 + DMA
- **Features**:
  - Vertical positioning of picker mechanism
  - Hardware pulse generation with DMA
  - Z-signal for absolute positioning
  - Coordinated with B axis for picker movement
  - ROS2 position/velocity interfaces

### B-Axis (Servo) - Picker Y axis
- **Type**: High-precision servo motor
- **Peripheral**: RMT Channel 3 + DMA
- **Features**:
  - Lateral movement of picker mechanism
  - Hardware pulse generation with DMA
  - Coordinated with A axis for picker positioning
  - Z-signal for absolute positioning
  - ROS2 position/velocity interfaces

### C-Axis (Stepper) - Picker Jaw
- **Type**: Stepper motor with position feedback
- **Peripheral**: MCPWM0 Timer 1 + PCNT1 (dedicated)
- **Features**:
  - Floating switch for object width measurement
  - Hardware position counting for grip precision
  - Acts as measurement tool and gripper
  - Recalibrates each cycle via end switches
  - ROS2 position/velocity interfaces with measurement data

### D-Axis (Stepper) - Picker Retractor
- **Type**: Discrete position stepper
- **Peripheral**: LEDC (dedicated)
- **Features**:
  - Discrete retract/extend positions
  - Position/velocity calculated from duty cycle
  - Recalibrates each cycle via end switches
  - No encoder needed - "precise enough"
  - ROS2 position/velocity interfaces (calculated)

## Command Syntax

### Axis Identification in Commands
```bash
# Position commands
MOVE X 100.0      # Move X to 100mm
MOVE Y 50.0       # Move Y to 50mm
MOVE Z 75.0       # Move Z to 75mm
MOVE A 90.0       # Move A to 90°
MOVE B -45.0      # Move B to -45°
MOVE C 2          # Move C to position index 2
MOVE D 1          # Move D to ON state

# Relative moves
MOVR X 10.0       # Move X +10mm relative
MOVR A 45.0       # Rotate A +45° relative

# Velocity commands
VEL X 50.0        # Set X velocity to 50mm/s
VEL A 90.0        # Set A velocity to 90°/s

# Status commands
STAT X            # Get X status
STAT ALL          # Get all axes status

# Home commands
HOME X            # Home X axis
HOME ALL          # Home all axes
```

## Axis Letter to Index Mapping

```cpp
// Internal mapping
enum AxisIndex {
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2,
    AXIS_A = 3,
    AXIS_B = 4,
    AXIS_C = 5,
    AXIS_D = 6,
    AXIS_ALL = 0xFF
};

// Letter to index conversion
uint8_t letterToAxis(char letter) {
    switch (toupper(letter)) {
        case 'X': return AXIS_X;
        case 'Y': return AXIS_Y;
        case 'Z': return AXIS_Z;
        case 'A': return AXIS_A;
        case 'B': return AXIS_B;
        case 'C': return AXIS_C;
        case 'D': return AXIS_D;
        default: return AXIS_ALL;
    }
}

// Index to letter conversion
char axisToLetter(uint8_t axis) {
    if (axis > AXIS_D) return '?';
    return 'X' + axis;
}
```

## Multi-Axis Commands

### Coordinated Moves
```bash
# G-code style (future enhancement)
G0 X100 Y50 Z25   # Rapid move multiple axes
G1 X50 Y25 F100   # Linear move with feedrate

# Current implementation
MOVE X 100.0      # Sequential moves
MOVE Y 50.0
MOVE Z 25.0
```

### Group Commands
```bash
# Enable/disable groups
ENA XYZ           # Enable X, Y, and Z
DIS CD            # Disable C and D

# Stop groups  
STOP XY           # Stop X and Y
STOP ALL          # Emergency stop all
```

## Status Display Format

```
Axis status display:
X: 100.00mm → 150.00mm [MOVING] HW
Y:  50.00mm =  50.00mm [IDLE  ] HW  
Z:  25.00mm =  25.00mm [IDLE  ] HW
A:  90.00° → 180.00°  [MOVING] HW
B: -45.00° = -45.00°  [IDLE  ] HW
C:  POS2   =  POS2    [IDLE  ] HW
D:  OFF    →  ON      [MOVING] SW

Format: AXIS: current → target [STATE] MODE
MODE: HW=Hardware counter, SW=Software position
```

## OLED Display Mapping

```
Compact display (128x64):
Line 0: X: 100.0>150.0 ►
Line 1: Y:  50.0= 50.0 ✓
Line 2: Z:  25.0= 25.0 ✓
Line 3: A:  90.0>180.0 ►
Line 4: B: -45.0=-45.0 ✓
Line 5: C: POS2
Line 6: D: OFF→ON
Line 7: SYS:OK I2C:OK
```

## Configuration Storage

Each axis has its own NVS namespace:
```
NVS Namespace mapping:
"motor_0" → X-axis configuration
"motor_1" → Y-axis configuration
"motor_2" → Z-axis configuration
"motor_3" → A-axis configuration
"motor_4" → B-axis configuration
"motor_5" → C-axis configuration
"motor_6" → D-axis configuration

"calib_0" → X-axis calibration data
... etc
```

## Safety Considerations by Axis

### Z-Axis Special Safety
- Gravity compensation required
- Brake control on disable
- Slower deceleration rates
- Power-loss position save

### C/D Independent Control
- C axis: MCPWM for precise jaw control and measurement
- D axis: LEDC for simple retractor positioning
- Both axes can move simultaneously
- Independent calibration via end switches

## ROS2 Control Compatibility

### Design Philosophy
- Universal real-time control device
- Serial interface for development and testing
- ROS2 hardware interface compatibility built-in
- All axes provide position and velocity interfaces

### Testing Phases
1. **Phase 1**: Serial command testing (current)
2. **Phase 2**: Mock ROS2 interface validation
3. **Phase 3**: Full ROS2 control integration

### Interface Requirements
- Position state publishing (50-100Hz)
- Velocity command acceptance
- Actual vs commanded position reporting
- Real-time performance guarantees

## Example Usage Patterns

### Tool Change Sequence
```cpp
// Move to safe Z height
motors[AXIS_Z]->moveAbsolute(150.0);
waitForStop(AXIS_Z);

// Open gripper
motors[AXIS_D]->moveAbsolute(0);  // OFF position
waitForStop(AXIS_D);

// Select tool 3
motors[AXIS_C]->moveAbsolute(3);  // Position 3
waitForStop(AXIS_C);

// Close gripper
motors[AXIS_D]->moveAbsolute(1);  // ON position
waitForStop(AXIS_D);
```

### Coordinated XY Movement
```cpp
// Start both axes simultaneously
motors[AXIS_X]->setVelocity(50.0);
motors[AXIS_Y]->setVelocity(25.0);

motors[AXIS_X]->moveAbsolute(100.0);
motors[AXIS_Y]->moveAbsolute(50.0);

// Wait for both to complete
waitForStop(AXIS_X);
waitForStop(AXIS_Y);
```