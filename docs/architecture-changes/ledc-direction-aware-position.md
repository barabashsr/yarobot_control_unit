# LedcPulseGenerator Direction-Aware Position Tracking

**Document Type:** Architecture Decision Record (ADR)
**Status:** Implemented
**Date:** 2025-12-17
**Affected Component:** `LedcPulseGenerator` (firmware/components/pulse_gen/)
**Related ADR:** `ledc-integrated-pcnt.md`

---

## 1. Problem Statement

### 1.1 Observed Issues

After integrating PCNT into LedcPulseGenerator, several issues were discovered:

1. **Velocity stuck at 100 Hz**: Motor never accelerated beyond minimum frequency
2. **Position increasing in reverse**: When moving to position 0 from position 67000, PCNT kept incrementing
3. **Stop command not working**: First `STOP D` didn't stop the motor, required second command
4. **Overflow count reset between moves**: Position jumped backwards after PCNT overflow

### 1.2 Root Cause Analysis

| Issue | Root Cause |
|-------|------------|
| Velocity stuck at 100 Hz | `velocityAtPosition(current_count)` used software estimate (always 0 due to integer truncation) |
| Position increasing in reverse | PCNT only counts rising edges - doesn't know direction |
| Stop not working | Completion check used `pulse_count_` (software estimate = 0), so `0 >= 30` was always false |
| Overflow reset | `startMove()`/`startVelocity()` incorrectly reset `overflow_count_` to 0 |

### 1.3 Reference Implementation

McpwmPulseGenerator (Y axis) handles these issues correctly by:
- Tracking direction in software and applying sign to position delta
- Using stable reference points (`last_completed_position_`, `pcnt_at_last_completion_`)
- Using hardware PCNT for all position/velocity calculations

---

## 2. Solution: Direction-Aware Position Tracking

### 2.1 Design Decision

Implement the same position tracking pattern as McpwmPulseGenerator:

1. **PCNT always counts UP** (rising edges only)
2. **Direction applied in software** during position calculation
3. **Stable reference points** updated only at motion completion
4. **Hardware PCNT** used for velocity calculation and completion detection

### 2.2 Key Formula

```cpp
// From McpwmPulseGenerator::profileTimerCallback()
int32_t pcnt_delta = hw_pcnt - pcnt_at_last_completion_;
int64_t position_delta = current_dir ? pcnt_delta : -pcnt_delta;  // Apply direction!
int64_t new_position = last_completed_position_ + position_delta;
```

---

## 3. Implementation Details

### 3.1 New Member Variables (ledc_pulse_gen.h)

```cpp
// Direction-aware position tracking (same pattern as McpwmPulseGenerator)
int64_t last_completed_position_;        ///< Absolute position at last motion completion
int32_t pcnt_at_last_completion_;        ///< Raw PCNT value at last motion completion
std::atomic<int64_t> absolute_position_; ///< Current absolute position (direction-aware)
```

### 3.2 Position Calculation (handleProfileUpdate)

```cpp
// Calculate absolute PCNT with overflow contribution (raw hardware count)
int32_t overflows = overflow_count_.load(std::memory_order_relaxed);
int32_t hw_pcnt = hw_pcnt_raw + (overflows * LIMIT_PCNT_OVERFLOW_RANGE);

// Calculate position delta from last completion (STABLE reference point)
// Apply direction sign: forward = positive delta, reverse = negative delta
int32_t pcnt_delta = hw_pcnt - pcnt_at_last_completion_;
bool current_dir = direction_;
int64_t position_delta = current_dir ? pcnt_delta : -pcnt_delta;

// Update absolute position (SINGLE SOURCE OF TRUTH for position queries)
int64_t new_position = last_completed_position_ + position_delta;
absolute_position_.store(new_position, std::memory_order_relaxed);
```

### 3.3 getPulseCount() Returns Direction-Aware Position

```cpp
int64_t LedcPulseGenerator::getPulseCount() const
{
    // Return direction-aware absolute position (updated in handleProfileUpdate)
    return absolute_position_.load(std::memory_order_relaxed);
}
```

### 3.4 Reference Points Saved on Completion

```cpp
// In handleProfileUpdate() when motion completes:
last_completed_position_ = absolute_position_.load(std::memory_order_relaxed);
pcnt_at_last_completion_ = hw_pcnt;

// In stopImmediate():
last_completed_position_ = absolute_position_.load(std::memory_order_relaxed);
if (pcnt_unit_) {
    int count = 0;
    pcnt_unit_get_count(pcnt_unit_, &count);
    int32_t overflows = overflow_count_.load(std::memory_order_relaxed);
    pcnt_at_last_completion_ = count + (overflows * LIMIT_PCNT_OVERFLOW_RANGE);
}
```

### 3.5 Raw PCNT for Profile Tracking

`pcnt_start_` stores raw hw_pcnt (not direction-aware position) for profile completion check:

```cpp
// In startMove(), startVelocity(), stop():
if (pcnt_unit_) {
    int count = 0;
    pcnt_unit_get_count(pcnt_unit_, &count);
    int32_t overflows = overflow_count_.load(std::memory_order_relaxed);
    pcnt_start_ = count + (overflows * LIMIT_PCNT_OVERFLOW_RANGE);
}

// Completion check uses raw PCNT progress (always positive):
int64_t pcnt_progress = hw_pcnt - static_cast<int32_t>(pcnt_start_);
if (pcnt_progress >= profile_.target_pulses) { /* complete */ }
```

### 3.6 Velocity Calculation Uses PCNT Progress

```cpp
// Use pcnt_progress (hardware PCNT) instead of current_count (software estimate)
float target_velocity = velocityAtPosition(pcnt_progress);
```

---

## 4. Bug Fixes Summary

### 4.1 Overflow Count Reset (Critical)

**Bug:** `startMove()` and `startVelocity()` reset `overflow_count_` to 0, losing position history.

**Fix:** Removed overflow reset. Only `reset()` should clear overflow history.

```cpp
// REMOVED from startMove() and startVelocity():
// overflow_count_.store(0, std::memory_order_relaxed);  // BUG!

// Comment added:
// Note: Do NOT reset overflow_count_ here - it tracks absolute position across moves
// Only reset() should clear the overflow history
```

### 4.2 Software Estimate Integer Truncation

**Bug:** `pulses_this_interval = velocity * 0.001` truncates to 0 for frequencies < 1000 Hz.

**Fix:** Use hardware PCNT (`pcnt_progress`) instead of software estimate for:
- Velocity calculation: `velocityAtPosition(pcnt_progress)`
- Completion check: `pcnt_progress >= profile_.target_pulses`
- State machine transitions

### 4.3 Stop Command Not Working

**Bug:** `stop()` calculated `target_pulses` from software estimate (0), and completion check `0 >= 30` was always false.

**Fix:**
- `stop()` uses raw hw_pcnt for `pcnt_start_`
- Completion check uses `pcnt_progress` (hardware PCNT delta)

### 4.4 target_pulses Type Change

**Change:** `LedcTrapezoidalProfile::target_pulses` changed from `int32_t` to `int64_t` to support large absolute positions.

```cpp
struct LedcTrapezoidalProfile {
    int64_t target_pulses;  // Was int32_t
    // ...
};
```

---

## 5. Files Changed

| File | Changes |
|------|---------|
| `ledc_pulse_gen.h` | Added `last_completed_position_`, `pcnt_at_last_completion_`, `absolute_position_`; Changed `target_pulses` to `int64_t` |
| `ledc_pulse_gen.cpp` | Direction-aware position calculation, reference point saving, raw PCNT usage, bug fixes |

---

## 6. Data Flow

```
LEDC PWM Output
      │
      ▼
  GPIO (INPUT_OUTPUT mode - internal loopback)
      │
      ▼
  PCNT Hardware Counter (counts rising edges, always UP)
      │
      ▼
handleProfileUpdate() [every 1ms]
      │
      ├─► hw_pcnt = raw_count + (overflows * 32768)
      │
      ├─► pcnt_delta = hw_pcnt - pcnt_at_last_completion_
      │
      ├─► position_delta = direction ? +pcnt_delta : -pcnt_delta
      │
      ├─► absolute_position_ = last_completed_position_ + position_delta
      │
      └─► pcnt_progress = hw_pcnt - pcnt_start_  (for profile completion)
```

---

## 7. Verification

### 7.1 Expected Debug Output

```
W (xxx) LEDC_PULSE: DEBUG updateProfile: pos=12345, hw_pcnt=45678, progress=1000, target=37000, freq=600.0, state=1, dir=FWD
```

- `pos`: Direction-aware absolute position (decreases in reverse)
- `hw_pcnt`: Raw hardware PCNT (always increases)
- `progress`: Pulses generated since move start (for completion check)
- `target`: Target pulses for this move
- `freq`: Current frequency (should ramp up during acceleration)
- `state`: 1=ACCEL, 2=CRUISE, 3=DECEL, 4=STOPPING
- `dir`: FWD or REV

### 7.2 Test Scenarios

1. **Forward velocity then reverse move:**
   ```
   VEL D 2000      # pos increases, hw_pcnt increases
   STOP D          # decelerates and stops
   MOVE D 0        # pos decreases, hw_pcnt still increases, dir=REV
   ```

2. **Overflow handling:**
   ```
   VEL D 2000      # run until hw_pcnt > 32768
   # Should see: "PCNT overflow detected"
   STOP D
   MOVE D 0        # position should still track correctly
   ```

3. **Acceleration verification:**
   ```
   MOVE D 10000    # freq should ramp: 100 → 200 → 400 → 600
   ```

---

## 8. Comparison with McpwmPulseGenerator

| Aspect | McpwmPulseGenerator (Y) | LedcPulseGenerator (D) |
|--------|------------------------|------------------------|
| PWM Source | MCPWM peripheral | LEDC peripheral |
| PCNT Config | Same (rising edge, no level GPIO) | Same |
| Direction Control | Shift register | Shift register |
| Position Calculation | `last_completed_position_ + signed_delta` | Same |
| Overflow Detection | Task-based delta detection | Same |
| Profile Update Interval | 20ms | 1ms |

---

## 9. References

- Related ADR: `ledc-integrated-pcnt.md` (PCNT integration)
- Related ADR: `pcnt-overflow-handling.md` (Overflow detection pattern)
- McpwmPulseGenerator: `components/pulse_gen/mcpwm_pulse_gen.cpp` lines 440-496
- ESP-IDF PCNT: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/pcnt.html
