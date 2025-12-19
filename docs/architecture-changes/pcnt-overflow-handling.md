# PCNT Overflow Handling for Continuous Position Tracking

**Document Type:** Architecture Decision Record (ADR)
**Status:** Implemented
**Date:** 2025-12-16
**Affected Component:** `McpwmPulseGenerator` (firmware/components/pulse_gen/)

---

## 1. Problem Statement

### 1.1 Observed Issue

During velocity mode operation (`VEL Y 2000`), the PCNT-based position counter exhibited incorrect behavior after approximately 32,767 pulses:

```
W (364546) MCPWM_PULSE: DEBUG updateProfile: pcnt=25280, ...
W (365546) MCPWM_PULSE: DEBUG updateProfile: pcnt=-6885, ...  // WRONG! Should be ~25880
```

The position counter jumped backwards by ~32,000 instead of incrementing by ~600 (expected at 600 Hz over 1 second).

### 1.2 Root Cause Analysis

The ESP32 PCNT (Pulse Counter) hardware is a **16-bit counter**. With the configuration used (`accum_count=false`), the counter behavior was:

| Configuration | Value |
|---------------|-------|
| `high_limit` | 32767 (INT16_MAX) |
| `low_limit` | -32768 (INT16_MIN) |
| `accum_count` | false |

**Critical Discovery:** With `accum_count=false`, when PCNT reaches `high_limit` (32767), it **wraps to 0**, not to `low_limit` (-32768).

Evidence from debug logs:
```
PCNT overflow detected: prev=32761, curr=7, delta=-32754
```

The counter went from 32761 → 0 → 7 (not to -32768).

### 1.3 Why ISR-Based Overflow Handling Was Disabled

The original implementation used PCNT watch point ISR callbacks for overflow detection, but this was intentionally disabled due to bugs:

```cpp
// DISABLED: Watch point ISR was causing stale watch point bugs that led to
// premature motion stopping and position corruption (180° instead of 360°).
// Task-based polling in updateProfile() is now the sole stop mechanism.
```

With ISR disabled, no mechanism existed to track overflows, causing position corruption after 32,767 pulses.

---

## 2. Solution: Task-Based Overflow Detection

### 2.1 Design Decision

Implement overflow detection in the polling task (`profileTimerCallback`) rather than re-enabling the problematic ISR. This approach:

- Runs every 20ms (50 Hz), sufficient for 600 Hz pulse rate
- Avoids ISR timing issues that caused the original bugs
- Provides deterministic behavior from task context

### 2.2 Detection Algorithm

**Key Insight:** PCNT only increments (configured for rising edge increment). A large *decrease* in the raw count indicates wrap-around.

```cpp
int32_t delta = count - prev_raw_pcnt_count_;
if (delta < -30000) {
    // Overflow detected: counter wrapped from ~32767 to ~0
    overflow_count_.fetch_add(1, std::memory_order_relaxed);
}
prev_raw_pcnt_count_ = count;
```

**Why -30000 threshold?**
- Normal delta: +12 pulses per 20ms at 600 Hz (always positive)
- Overflow delta: ~-32754 (from 32761 to 7)
- Threshold of -30000 provides margin while reliably detecting wraps

### 2.3 Position Calculation

```cpp
// PCNT_OVERFLOW_RANGE = 32768 (not 65536!)
int32_t hw_pcnt = raw_count + (overflow_count * MCPWM_PCNT_OVERFLOW_RANGE);
```

**Why 32768, not 65536?**

| Assumption | Overflow Range | Result |
|------------|----------------|--------|
| Wrap 32767 → -32768 | 65536 | Incorrect (doubled position) |
| Wrap 32767 → 0 | 32768 | Correct (continuous counting) |

The counter range is 0 to 32767 (32768 values), not -32768 to 32767 (65536 values), because it wraps to 0.

---

## 3. Implementation Details

### 3.1 New Constants (mcpwm_pulse_gen.h)

```cpp
/** @brief PCNT high limit (16-bit signed max) */
static constexpr int16_t MCPWM_PCNT_HIGH_LIMIT = INT16_MAX;  // 32767

/** @brief PCNT low limit (16-bit signed min) */
static constexpr int16_t MCPWM_PCNT_LOW_LIMIT = INT16_MIN;   // -32768

/**
 * @brief PCNT overflow range for position accumulation
 *
 * With accum_count=false, the PCNT counter wraps from 32767 to 0.
 * Each overflow represents 32768 pulses (0 to 32767 inclusive).
 */
static constexpr int32_t MCPWM_PCNT_OVERFLOW_RANGE = 32768;
```

### 3.2 New Member Variable

```cpp
int32_t prev_raw_pcnt_count_;  // Previous raw PCNT reading for overflow detection
```

### 3.3 Initialization Points

| Function | Action |
|----------|--------|
| `init()` | `prev_raw_pcnt_count_ = init_pcnt_count` |
| `startMove()` | `prev_raw_pcnt_count_ = current_pcnt` |
| `startVelocity()` | `prev_raw_pcnt_count_ = current_pcnt` |

### 3.4 Overflow Detection (profileTimerCallback)

```cpp
int count = 0;
pcnt_unit_get_count(pcnt_unit_, &count);

// Task-based overflow detection
int32_t delta = count - self->prev_raw_pcnt_count_;
if (delta < -30000) {
    self->overflow_count_.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGW(TAG, "PCNT overflow detected: prev=%ld, curr=%d, delta=%ld, overflow_count=%ld",
             (long)self->prev_raw_pcnt_count_, count, (long)delta,
             (long)self->overflow_count_.load(std::memory_order_relaxed));
}
self->prev_raw_pcnt_count_ = count;

int32_t overflows = self->overflow_count_.load(std::memory_order_relaxed);
int32_t hw_pcnt = count + (overflows * MCPWM_PCNT_OVERFLOW_RANGE);
```

---

## 4. Verification

### 4.1 Expected Log Output

Before fix:
```
pcnt=32508 → pcnt=-6885  // Jump of ~32000 backwards
```

After fix:
```
PCNT overflow detected: prev=32761, curr=7, delta=-32754, overflow_count=1
pcnt=32508 → pcnt=33110  // Continuous increment (~600 pulses/sec)
```

### 4.2 Test Scenario

1. Start velocity mode: `VEL Y 2000`
2. Run for >60 seconds (>32,767 pulses at 600 Hz)
3. Verify position continues incrementing smoothly through overflow boundaries
4. Stop motor and verify final position is accurate

---

## 5. Files Changed

| File | Changes |
|------|---------|
| `mcpwm_pulse_gen.h` | Added PCNT constants, `prev_raw_pcnt_count_` member |
| `mcpwm_pulse_gen.cpp` | Overflow detection logic, constant references updated |

---

## 6. Alternatives Considered

### 6.1 Re-enable ISR-Based Overflow Handling

**Rejected:** Original ISR caused stale watch point bugs leading to position corruption. Task-based approach is more reliable.

### 6.2 Use `accum_count=true`

**Rejected:** ESP-IDF documentation and previous testing showed this prevents `pcnt_unit_clear_count()` from working correctly, causing stale counts between moves.

### 6.3 Use 64-bit Software Counter

**Rejected:** Would require disabling hardware PCNT limits entirely. Current approach leverages hardware efficiently.

---

## 7. Lessons Learned

1. **ESP32 PCNT with `accum_count=false` wraps to 0, not to `low_limit`**
2. **Task-based polling at 50 Hz is sufficient for detecting overflow at pulse rates up to ~1.5 MHz** (30000 pulses / 20ms)
3. **Delta-based detection is more robust than threshold-based** (checking decrease vs. checking specific value ranges)

---

## 8. References

- ESP-IDF PCNT Documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/pcnt.html
- Original ISR disable commit: "DISABLED: Watch point ISR was causing stale watch point bugs"
- Related story: Y-axis continuous position tracking
