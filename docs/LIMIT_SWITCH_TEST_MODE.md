# Test Mode: Limit Switches and I2C Disabled

**Status:** ACTIVE - Limit switches AND I2C are currently DISABLED for testing

**Date:** 2024-12-24

**Purpose:** Allow testing robot motion without physical limit switches or I2C devices (MCP23017, OLED) connected.

---

## Changes Made

### 1. Limit Switch Enforcement Disabled

**File:** `firmware/components/control/safety_monitor/safety_monitor.c`

**Line ~722:** Changed EndSwitchMode initialization:
```c
// TEMPORARY TEST MODE: Limit switches DISABLED for testing without hardware
s_monitor.endswitch_mode[i] = END_SWITCH_MODE_NONE;  // Was: END_SWITCH_MODE_DEFAULT
```

**Effect:**
- Motion will NOT stop when limit switches trigger
- Directions NOT blocked at limits
- LIM command still works (shows switch states for debugging)

---

### 2. I2C Initialization Disabled

**File:** `firmware/main/yarobot_control_unit.cpp`

**Lines ~285-331:** Wrapped I2C init in `#if 0`:
- I2C0 (MCP23017 expanders) - DISABLED
- I2C1 (OLED display) - DISABLED
- I2C bus isolation check - DISABLED

---

### 3. I2C-Dependent Tasks Disabled

**File:** `firmware/main/yarobot_control_unit.cpp`

**Lines ~411-414:** `i2c_monitor_task` - DISABLED
**Lines ~426-429:** `display_task` - DISABLED

---

### 4. Safety Monitor MCP23017 Check Relaxed

**File:** `firmware/components/control/safety_monitor/safety_monitor.c`

**Lines ~700-712:** MCP23017 check now warns instead of failing:
```c
if (!mcp23017_wrapper_is_initialized()) {
    ESP_LOGW(TAG, "TEST MODE: MCP23017 not initialized - limit switches disabled");
}
```

---

### 5. Motor System MCP23017 Init Disabled

**File:** `firmware/components/control/motor_system/motor_system.cpp`

**Lines ~712-726:** MCP23017 wrapper init wrapped in `#if 0`

---

### 6. Safety Monitor Task Skips MCP Reads

**File:** `firmware/components/control/safety_monitor/safety_monitor.c`

**Lines ~625-654:** Task checks `mcp23017_wrapper_is_initialized()` at startup and skips all MCP reads if not available. Only E-stop GPIO monitoring remains active.

---

## To Re-enable Everything

### Step 1: Re-enable I2C in main

In `firmware/main/yarobot_control_unit.cpp`:

1. Change `#if 0` to `#if 1` on line ~287 (I2C init block)
2. Change `#if 0` to `#if 1` on line ~411 (i2c_monitor_task)
3. Change `#if 0` to `#if 1` on line ~426 (display_task)

### Step 2: Re-enable MCP23017 in motor_system

In `firmware/components/control/motor_system/motor_system.cpp`:

1. Change `#if 0` to `#if 1` on line ~714 (MCP23017 init)

### Step 3: Re-enable limit switch enforcement

In `firmware/components/control/safety_monitor/safety_monitor.c`:

1. Change line ~723 back to:
   ```c
   s_monitor.endswitch_mode[i] = END_SWITCH_MODE_DEFAULT;
   ```

2. Change `#if 0` to `#if 1` on line ~702 (MCP23017 check)

### Step 4: Rebuild

```bash
cd firmware && idf.py build && idf.py flash
```

---

## What's Still Working

- USB CDC communication
- All motor axes (X, Y, Z, A, B, C, D, E)
- Motion commands (MOVE, MOVR, VEL, STOP, etc.)
- Position tracking (PCNT)
- Shift register outputs (enables, brakes, directions)
- E-stop GPIO (if button connected)

---

## What's Disabled

| Feature | Status | Hardware |
|---------|--------|----------|
| Limit switches | DISABLED | MCP23017 #0 |
| InPos/ALARM feedback | DISABLED | MCP23017 #1 |
| OLED display | DISABLED | I2C OLED |
| I2C health monitoring | DISABLED | - |

---

## Safety Warning

**CAUTION:** With limit switches disabled, the robot can crash into physical stops!
- Use only for testing without mechanical constraints
- Keep hands clear of moving parts
- Be ready to hit E-stop if needed
- Do NOT use in production!
