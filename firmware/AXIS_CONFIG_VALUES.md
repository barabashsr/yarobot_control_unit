# Axis Configuration Values

## Default Configuration (Current)

Used for simulation/testing with degree units.

| Axis | Pulses/Rev | Units/Rev | Units | Type |
|------|------------|-----------|-------|------|
| X | 200 | 360 | degrees | Linear servo |
| Y | 200 | 360 | degrees | Linear servo |
| Z | 200 | 360 | degrees | Linear servo |
| A | 200 | 360 | degrees | Rotary servo |
| B | 200 | 360 | degrees | Rotary servo |
| C | 200 | 360 | degrees | Stepper |
| D | 200 | 360 | degrees | Stepper |
| E | 1 | 1 | binary | Discrete |

**Files:** Default values in `config_defaults.h`

---

## Hardware Test Configuration (2025-12-18)

Real hardware values with mm units. 1 revolution = full stroke travel.

| Axis | Pulses/Rev | Units/Rev (mm) | Stroke | Pulses/mm | Type |
|------|------------|----------------|--------|-----------|------|
| X | 10000 | 350 | 350 mm | 28.57 | Linear servo |
| Y | 10000 | 48 | 48 mm | 208.33 | Linear servo |
| Z | 312 (10000/32) | 5 | 5 mm | 62.4 | Linear servo |
| A | 625 (10000/16) | 3.8 | 3.8 mm | 164.47 | Linear servo |
| B | 10000 | 72 | 72 mm | 138.89 | Linear servo |
| C | 3200 (200*16) | 48 | 48 mm | 66.67 | Stepper |
| D | 3200 (200*16) | 48 | 48 mm | 66.67 | Stepper |

**Files:** Commented in `config_defaults.h` and `motor_system.cpp`

---

## How to Switch Configurations

### Enable Hardware Test Mode:

1. **config_defaults.h** - Uncomment lines 67-86:
```c
#define X_AXIS_PULSES_PER_REV       10000.0f
#define X_AXIS_UNITS_PER_REV        350.0f
// ... etc
```

2. **motor_system.cpp** - In `init_axis_configs()`:
   - Comment out lines 169-198 (default code)
   - Uncomment lines 205-238 (hardware test code)

### Restore Default Mode:

1. **config_defaults.h** - Comment out lines 67-86
2. **motor_system.cpp** - In `init_axis_configs()`:
   - Uncomment lines 169-198 (default code)
   - Comment out lines 205-238 (hardware test code)

---

## Notes

- Hardware test mode uses mm for position units
- Default mode uses degrees for position units
- Servo axes (X, Y, Z, A, B) have brakes - use `BRAKE X 1` to release before motion
- Enable axis before motion: `EN X 1`
