# Motor Calibration Procedures

## Overview
This document describes the calibration procedures for the yarobot_control_unit motor control system, supporting both servo drives with Z-signal feedback and open-loop steppers.

## 1. Servo Drive Calibration (Axes 0-3)

### 1.1 Initial Homing Sequence

```
Procedure: SERVO_HOME_CALIBRATION
Prerequisites: 
- Motor enabled
- End switches functional
- Z-signal connected

Steps:
1. Set direction towards MIN end switch
2. Move at slow speed (10% max velocity)
3. Monitor end switch via I2C expander
4. When MIN switch triggered:
   a. Stop immediately
   b. Record rough position
   c. Back off 2mm (configurable)
   
5. Enable Z-signal interrupt detection
6. Move slowly towards MAX direction
7. On first Z-signal detection:
   a. Stop motion
   b. Set absolute zero position
   c. Reset pulse counter
   
8. Verify position complete signal active
9. Store calibration data
```

### 1.2 Backlash Measurement

```
Procedure: MEASURE_BACKLASH
Prerequisites: 
- Axis homed
- Z-signal calibrated

Steps:
1. Move forward 10mm at medium speed
2. Wait for position complete signal
3. Record position P1
4. Move backward 20mm
5. Wait for position complete signal
6. Move forward 10mm
7. Wait for position complete signal  
8. Record position P2
9. Backlash = |P2 - P1|
10. Store backlash compensation value
```

### 1.3 Z-Signal Verification

```
Procedure: VERIFY_Z_SIGNAL
Prerequisites:
- Axis homed
- Known pulses per revolution

Steps:
1. Clear Z-signal counter
2. Move exactly one revolution
3. Verify exactly one Z-signal received
4. Verify pulse count matches expected
5. If mismatch > threshold:
   a. Log error
   b. Reduce speed and retry
   c. Alert operator if persistent
```

## 2. Stepper Motor Calibration (Axes 4-5)

### 2.1 Open-Loop Homing

```
Procedure: STEPPER_HOME_CALIBRATION
Prerequisites:
- Motor enabled
- End switches functional

Steps:
1. Set direction towards MIN end switch
2. Move at slow speed with gentle acceleration
3. Monitor end switch via I2C expander
4. When MIN switch triggered:
   a. Stop with deceleration
   b. Back off 5mm
   c. Approach again at 50% speed
   
5. On second switch trigger:
   a. Back off 1mm
   b. Set home position
   c. Clear pulse counter
   
6. Move to safe position (10mm from switch)
7. Store calibration data
```

### 2.2 Maximum Travel Calibration

```
Procedure: MEASURE_TRAVEL_LIMITS
Prerequisites:
- Axis homed

Steps:
1. Start from home position
2. Move slowly towards MAX end switch
3. Count pulses during travel
4. When MAX switch triggered:
   a. Stop immediately
   b. Record pulse count as max_travel
   c. Calculate working range (max - margins)
   
5. Return to center position
6. Store travel limits
```

## 3. System-Wide Calibration

### 3.1 Multi-Axis Synchronization Check

```
Procedure: VERIFY_AXIS_SYNC
Prerequisites:
- All axes calibrated individually

Steps:
1. Home all axes sequentially
2. Move all axes to center positions
3. Perform synchronized moves:
   a. All axes +10mm
   b. Verify position complete signals
   c. All axes return to center
   
4. Check timing alignment
5. Adjust RMT/MCPWM timing if needed
```

### 3.2 Emergency Stop Verification

```
Procedure: VERIFY_EMERGENCY_STOP
Prerequisites:
- All axes operational

Steps:
1. Start multi-axis coordinated move
2. Trigger emergency stop mid-motion
3. Verify all axes:
   a. Stop within 5ms
   b. Disable signals activated
   c. Maintain position readings
   
4. Clear emergency condition
5. Verify axes can resume operation
6. Re-home if position uncertain
```

## 4. Calibration Data Storage

### 4.1 Per-Axis Calibration Data Structure

```c
typedef struct {
    uint32_t pulses_per_mm;        // Or pulses_per_degree
    uint32_t pulses_per_rev;       // For servo drives
    int32_t home_offset;           // Offset from switch to zero
    int32_t min_position;          // Software limit
    int32_t max_position;          // Software limit
    uint16_t backlash_pulses;      // Compensation value
    uint8_t z_signal_enabled;      // For servo drives
    uint32_t max_velocity;         // Pulses/second
    uint32_t max_acceleration;     // Pulses/second²
} AxisCalibrationData;
```

### 4.2 Persistent Storage

- Store calibration data in NVS (Non-Volatile Storage)
- Implement checksum validation
- Provide factory reset option
- Log calibration history with timestamps

## 5. Calibration Error Handling

### Common Error Conditions

1. **End switch not detected**: Timeout after max travel
2. **Z-signal missing**: No index pulse within one revolution  
3. **Position mismatch**: Pulse count vs Z-signal discrepancy
4. **Excessive backlash**: Beyond mechanical tolerance
5. **Motion blocked**: Current limit or stall detected

### Recovery Procedures

- Automatic retry with reduced speed
- Alert operator for manual intervention
- Fallback to safe mode operation
- Log all errors for diagnostics