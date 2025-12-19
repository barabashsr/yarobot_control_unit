# LedcPulseGenerator Integrated PCNT for D-Axis Position Tracking

**Document Type:** Architecture Decision Record (ADR)
**Status:** Implemented
**Date:** 2025-12-17
**Affected Component:** `LedcPulseGenerator` (firmware/components/pulse_gen/)

---

## 1. Problem Statement

### 1.1 Observed Issue

D-axis position tracking was not showing hardware PCNT values in debug output. When running `VEL D 2000`, the expected `pcnt=` debug messages showed `pcnt=0` always, while Y-axis correctly displayed incrementing PCNT values.

### 1.2 Root Cause Analysis

Investigation revealed a design inconsistency between `McpwmPulseGenerator` (Y, C axes) and `LedcPulseGenerator` (D axis):

| Component | PCNT Handling |
|-----------|---------------|
| `McpwmPulseGenerator` | PCNT unit passed in constructor, initialized internally |
| `LedcPulseGenerator` | No PCNT parameter, relied on external `position_tracker_` |

The `LedcPulseGenerator` was designed to use an external `PcntTracker` via `setPositionTracker()`, but this was never called in `motor_system.cpp`. The comment "MCPWM/LEDC generators implement IPositionTracker directly" was misleading:

- `McpwmPulseGenerator::getPosition()` reads hardware PCNT via `pcnt_unit_get_count()`
- `LedcPulseGenerator::getPosition()` returned software-estimated `pulse_count_` (not hardware PCNT)

### 1.3 Impact

- D-axis position tracking was software-estimated, not hardware-measured
- Position drift over time due to timing inaccuracies
- No visibility into actual pulse generation vs. expected

---

## 2. Solution: Integrated PCNT in LedcPulseGenerator

### 2.1 Design Decision

Modify `LedcPulseGenerator` to match `McpwmPulseGenerator` pattern: accept PCNT unit ID in constructor and manage PCNT hardware internally.

**Rationale:**
- Consistency with existing MCPWM implementation
- Eliminates need for external PcntTracker wiring
- Hardware PCNT counting is more accurate than software estimation
- GPIO already configured for internal loopback (INPUT_OUTPUT mode)

### 2.2 Alternative Considered

**External PcntTracker approach:**
- Create `PcntTracker` in `motor_system.cpp`
- Wire via `s_ledc_d.setPositionTracker(&tracker_d)`

**Rejected because:**
- Adds complexity to motor_system initialization
- Inconsistent with McpwmPulseGenerator pattern
- Requires maintaining separate tracker lifecycle

---

## 3. Implementation Details

### 3.1 Constructor Change

**Before:**
```cpp
LedcPulseGenerator(int gpio_num, ledc_timer_t timer, ledc_channel_t channel);
```

**After:**
```cpp
LedcPulseGenerator(int gpio_num, ledc_timer_t timer, ledc_channel_t channel, int pcnt_unit_id);
```

### 3.2 New Member Variables (ledc_pulse_gen.h)

```cpp
int pcnt_unit_id_;                    // PCNT unit index (PCNT_UNIT_D = 2)
pcnt_unit_handle_t pcnt_unit_;        // PCNT unit handle
pcnt_channel_handle_t pcnt_channel_;  // PCNT channel handle
```

### 3.3 PCNT Initialization (init())

```cpp
// PCNT Unit Configuration
pcnt_unit_config_t pcnt_unit_config = {};
pcnt_unit_config.high_limit = LIMIT_PCNT_HIGH_LIMIT;
pcnt_unit_config.low_limit = LIMIT_PCNT_LOW_LIMIT;
pcnt_unit_config.flags.accum_count = false;

pcnt_new_unit(&pcnt_unit_config, &pcnt_unit_);

// PCNT Channel - count rising edges on GPIO
pcnt_chan_config_t pcnt_chan_config = {};
pcnt_chan_config.edge_gpio_num = gpio_num_;
pcnt_chan_config.level_gpio_num = -1;

pcnt_new_channel(pcnt_unit_, &pcnt_chan_config, &pcnt_channel_);
pcnt_channel_set_edge_action(pcnt_channel_,
                             PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // Rising edge
                             PCNT_CHANNEL_EDGE_ACTION_HOLD);     // Falling edge

pcnt_unit_enable(pcnt_unit_);
pcnt_unit_start(pcnt_unit_);
```

### 3.4 Position Reading (getPulseCount())

**Before:**
```cpp
int64_t LedcPulseGenerator::getPulseCount() const {
    return pulse_count_.load(std::memory_order_relaxed);  // Software estimate
}
```

**After:**
```cpp
int64_t LedcPulseGenerator::getPulseCount() const {
    int count = 0;
    if (pcnt_unit_) {
        pcnt_unit_get_count(pcnt_unit_, &count);
    }
    return static_cast<int64_t>(count);  // Hardware PCNT
}
```

### 3.5 Reset Function Update

```cpp
esp_err_t LedcPulseGenerator::reset(int64_t position) {
    if (pcnt_unit_) {
        pcnt_unit_clear_count(pcnt_unit_);  // Clear hardware counter
    }
    pulse_count_.store(position, std::memory_order_relaxed);
    return ESP_OK;
}
```

### 3.6 Destructor Cleanup

```cpp
if (pcnt_channel_) {
    pcnt_del_channel(pcnt_channel_);
}
if (pcnt_unit_) {
    pcnt_unit_stop(pcnt_unit_);
    pcnt_del_unit(pcnt_unit_);
}
```

### 3.7 motor_system.cpp Update

**Before:**
```cpp
static LedcPulseGenerator s_ledc_d(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
```

**After:**
```cpp
static LedcPulseGenerator s_ledc_d(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D, PCNT_UNIT_D);
```

### 3.8 Debug Logging (throttled to 1Hz)

```cpp
// DEBUG: Log hardware PCNT vs estimated pulse count (every 1 second)
static int debug_counter = 0;
if (pcnt_unit_ && ++debug_counter % 1000 == 0) {
    int hw_pcnt = 0;
    pcnt_unit_get_count(pcnt_unit_, &hw_pcnt);
    ESP_LOGW(TAG, "DEBUG updateProfile: pcnt=%d, est=%lld, target=%ld, freq=%.1f, state=%d",
             hw_pcnt, current_count, (long)profile_.target_pulses, velocity, (int)current_state);
}
```

---

## 4. Files Changed

| File | Changes |
|------|---------|
| `config_limits.h` | Added `LIMIT_PCNT_OVERFLOW_RANGE` constant |
| `ledc_pulse_gen.h` | Added `#include "driver/pulse_cnt.h"`, new constructor parameter, PCNT member variables, overflow tracking members |
| `ledc_pulse_gen.cpp` | Constructor update, PCNT init/cleanup, overflow detection in `handleProfileUpdate()`, `getPulseCount()` reads hardware + overflow, `reset()` clears PCNT and overflow, throttled debug logging |
| `motor_system.cpp` | Pass `PCNT_UNIT_D` to D-axis constructor |
| `test_ledc_pulse_gen.cpp` | Updated all constructor calls (33 instances) |
| `test_pulse_cmd.cpp` | Updated 2 constructor calls |

---

## 5. Configuration Reference

From `config_peripherals.h`:
```cpp
#define PCNT_UNIT_D         2
#define LEDC_TIMER_D        LEDC_TIMER_2
#define LEDC_CHANNEL_D      LEDC_CHANNEL_2
```

From `config_gpio.h`:
```cpp
#define GPIO_D_STEP         GPIO_NUM_17
```

---

## 6. Verification

### 6.1 Expected Debug Output

```
W (xxxxx) LEDC_PULSE: DEBUG updateProfile: pcnt=1234, est=1230, target=5000, freq=2000.0, state=1
```

- `pcnt`: Hardware PCNT count (actual pulses)
- `est`: Software estimated count (for profile control)
- `target`: Target pulse count for move
- `freq`: Current frequency in Hz
- `state`: Profile state (0=IDLE, 1=ACCEL, 2=CRUISE, 3=DECEL)

### 6.2 Test Scenarios

1. `VEL D 2000` - Verify pcnt increments continuously
2. `MOVE D 10000` - Verify pcnt matches target at completion
3. Compare pcnt vs est to validate software estimation accuracy

---

## 7. PCNT Overflow Handling

### 7.1 Problem

ESP32 PCNT is a 16-bit counter. With `accum_count=false`, it wraps from 32767 to 0 after ~32K pulses. Without overflow detection, position would jump backwards.

### 7.2 Solution: Task-Based Overflow Detection

Same approach as McpwmPulseGenerator (see `pcnt-overflow-handling.md`):

**New Member Variables:**
```cpp
std::atomic<int32_t> overflow_count_;    // Number of PCNT overflows
int32_t prev_raw_pcnt_count_;            // Previous raw PCNT for delta detection
```

**Overflow Detection in handleProfileUpdate():**
```cpp
int hw_pcnt_raw = 0;
pcnt_unit_get_count(pcnt_unit_, &hw_pcnt_raw);

// Delta-based overflow detection: large negative jump means counter wrapped
int32_t delta = hw_pcnt_raw - prev_raw_pcnt_count_;
if (delta < -30000) {
    overflow_count_.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGW(TAG, "PCNT overflow detected: prev=%ld, curr=%d, delta=%ld",
             (long)prev_raw_pcnt_count_, hw_pcnt_raw, (long)delta);
}
prev_raw_pcnt_count_ = hw_pcnt_raw;
```

**Position Calculation with Overflow:**
```cpp
int64_t LedcPulseGenerator::getPulseCount() const {
    int count = 0;
    pcnt_unit_get_count(pcnt_unit_, &count);

    int32_t overflows = overflow_count_.load(std::memory_order_relaxed);
    return static_cast<int64_t>(count) +
           (static_cast<int64_t>(overflows) * LIMIT_PCNT_OVERFLOW_RANGE);
}
```

### 7.3 New Constant (config_limits.h)

```cpp
#define LIMIT_PCNT_OVERFLOW_RANGE   32768  // Pulses per overflow cycle
```

### 7.4 Initialization Points

| Function | Action |
|----------|--------|
| Constructor | `overflow_count_(0)`, `prev_raw_pcnt_count_(0)` |
| `init()` | Read initial PCNT, store as baseline |
| `startMove()` | **Do NOT reset overflow_count_** (position is absolute) |
| `startVelocity()` | **Do NOT reset overflow_count_** (position is absolute) |
| `reset()` | Clear PCNT, reset overflow_count_ and baseline (only place to reset) |

**Important Bug Fix (2025-12-17):** Initial implementation incorrectly reset `overflow_count_` in `startMove()` and `startVelocity()`, which caused position to jump backwards after overflow. The overflow counter must persist across moves to maintain absolute position tracking.

---

## 8. Impact on Architecture

### 7.1 Epic/PRD Updates Needed

- **D-Axis Position Tracking**: Now uses hardware PCNT (same as Y, C axes)
- **LedcPulseGenerator API**: Constructor signature changed (breaking change for tests)
- **Position Accuracy**: Hardware counting eliminates software estimation drift

### 7.2 Completed Enhancements

- **Overflow handling**: Implemented task-based delta detection matching McpwmPulseGenerator pattern
- **Absolute position tracking**: PCNT counter with overflow accumulation supports unlimited range

---

## 8. References

- **Follow-up ADR: `ledc-direction-aware-position.md`** (Direction-aware position tracking - supersedes some details here)
- Related ADR: `pcnt-overflow-handling.md` (McpwmPulseGenerator overflow detection)
- ESP-IDF PCNT: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/pcnt.html
- Internal loopback: GPIO configured as INPUT_OUTPUT for LEDC output + PCNT input
