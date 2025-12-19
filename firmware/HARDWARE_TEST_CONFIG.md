# Hardware Test Configuration (2025-12-18)

## Overview
Temporary axis configuration for real hardware testing. All axes configured with actual mechanical parameters.

## Modified Files

### 1. `components/config/include/config_defaults.h`
Added per-axis defines for pulses_per_rev and units_per_rev:

| Axis | Pulses/Rev | Units/Rev (mm) | Notes |
|------|------------|----------------|-------|
| X | 10000 | 350 | Servo, linear |
| Y | 10000 | 48 | Servo, linear |
| Z | 312 | 5 | Servo, linear (10000/32) |
| A | 625 | 3.8 | Servo, linear (10000/16) |
| B | 10000 | 72 | Servo, linear |
| C | 3200 | 48 | Stepper (200*16) |
| D | 3200 | 48 | Stepper (same as C) |

### 2. `components/control/motor_system/motor_system.cpp`
Updated `init_axis_configs()` to use per-axis defines instead of generic defaults.

## Restoration Instructions
To restore original configuration:

1. In `config_defaults.h`: Remove or comment out the per-axis defines section
2. In `motor_system.cpp`: Uncomment the original code block and comment out the hardware test code

Both files contain commented original values marked with "ORIGINAL" for easy restoration.

## Units
- Position units: mm (millimeters)
- 1 revolution = units_per_rev mm of travel (full stroke)
