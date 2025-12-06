# FastAccelStepper Migration Strategy: RMT Pulse Generator Refactor

**Document Type:** Implementation Strategy
**Status:** Approved for Implementation
**Date:** 2025-12-06
**Supersedes:** `rmt-encoder-refactor.md` (Draft)
**Affected Components:** `RmtPulseGenerator`, `config_limits.h`, `config_peripherals.h`

---

## 1. Executive Summary

This document defines the strategy for porting FastAccelStepper library patterns to the YaRobot `RmtPulseGenerator` implementation. The goal is to replace the current DMA streaming architecture with an on-demand callback encoder pattern while preserving the `IPulseGenerator` interface.

### 1.1 Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Target Axes | X, Z, A, B (RMT only) | Y, C use MCPWM; D uses LEDC - unchanged |
| RMT Resolution | 16 MHz | FastAccelStepper proven; 62.5ns adequate for 500 kHz |
| Position Tracking | PCNT hardware | 5 spare units available (3-7); real-time accuracy |
| Integration | Port patterns to IPulseGenerator | No external dependency; clean interface |
| Mid-Motion | Full blending (velocity + target) | PRD requirement for smooth transitions |

### 1.2 Benefits

- **All 4 RMT channels usable simultaneously** (current: channel exhaustion issue)
- **Real-time position tracking** via dedicated PCNT units
- **Mid-motion velocity and target changes** with smooth blending
- **Reduced CPU overhead** (callback-based vs continuous DMA refill)
- **Simplified code** (no refill task, no double-buffer management)

---

## 2. Architecture Overview

### 2.1 Current Architecture (DMA Streaming) - Problems

```
┌─────────────────────────────────────────────────────────────┐
│           CURRENT: DMA STREAMING (PROBLEMATIC)              │
└─────────────────────────────────────────────────────────────┘

  startMove()
      │
      ▼
  ┌─────────────────────┐
  │ calculateProfile()  │  FPU operations
  └──────────┬──────────┘
             │
             ▼
  ┌─────────────────────┐     ┌─────────────────────┐
  │ Prime Buffer A      │────▶│ Prime Buffer B      │
  │ (48 symbols)        │     │ (48 symbols)        │
  └──────────┬──────────┘     └──────────┬──────────┘
             │                           │
             ▼                           ▼
  ┌────────────────────────────────────────────────┐
  │     RMT DMA TX (with_dma = true)               │
  │     mem_block_symbols = 48                     │
  │     PROBLEM: Exhausts all TX channel slots!    │
  └──────────────────────┬─────────────────────────┘
                         │
                         ▼ TX-Done ISR (~10kHz at 500kHz output)
  ┌────────────────────────────────────────────────┐
  │ ISR: Swap buffers, signal refill task          │
  │ PROBLEM: High CPU overhead                     │
  └──────────────────────┬─────────────────────────┘
                         │
                         ▼
  ┌────────────────────────────────────────────────┐
  │ Refill Task (Core 1, high priority)            │
  │ - fillBuffer() with FPU sqrt()                 │
  │ - PROBLEM: Must complete in <96µs window       │
  └────────────────────────────────────────────────┘
```

**Issues:**
1. Channel exhaustion: `rmt_tx_register_to_group: no free tx channels`
2. High ISR rate: ~10 kHz callback rate at max speed
3. Tight timing: Refill must complete before buffer exhaustion
4. Position tracking: Software-based, potential lag

### 2.2 New Architecture (Callback Encoder + PCNT)

```
┌─────────────────────────────────────────────────────────────┐
│        NEW: CALLBACK ENCODER + PCNT POSITION TRACKING       │
└─────────────────────────────────────────────────────────────┘

  startMove() / startVelocity() / setVelocity() / setTarget()
      │
      ▼
  ┌────────────────────────────────────────────────┐
  │ Ramp Generator (existing profile calculation)  │
  │ - Trapezoidal profile                          │
  │ - Per-axis accel/decel from config             │
  │ - Real-time parameter updates supported        │
  └──────────────────────┬─────────────────────────┘
                         │
                         ▼
  ┌────────────────────────────────────────────────┐
  │ Command Queue (32 entries, ring buffer)        │
  │ Entry: { ticks_per_step, step_count, flags }   │
  │ - Filled by ramp generator (task context)      │
  │ - Read by encoder callback (ISR context)       │
  └──────────────────────┬─────────────────────────┘
                         │
                         │ RMT requests symbols on-demand
                         ▼
  ┌────────────────────────────────────────────────┐
  │ RMT Callback Encoder (ISR context)             │
  │ - Reads command from queue                     │
  │ - Generates PART_SIZE (24) symbols             │
  │ - No FPU operations                            │
  │ - Adaptive steps per command for latency bound │
  └──────────────────────┬─────────────────────────┘
                         │
                         ▼
  ┌────────────────────────────────────────────────┐
  │ RMT Hardware (NO DMA)                          │
  │ mem_block_symbols = 48                         │
  │ with_dma = false                               │
  │ trans_queue_depth = 1                          │
  │ resolution_hz = 16,000,000                     │
  └──────────────────────┬─────────────────────────┘
                         │
                         │ Step pulses output to GPIO
                         ▼
  ┌────────────────────────────────────────────────┐
  │ PCNT Hardware (Real-Time Position)             │
  │ - Dedicated unit per axis (X=3, Z=4, A=5, B=6) │
  │ - Counts actual pulses from step GPIO          │
  │ - ISR on overflow (±32767) accumulates to i64  │
  │ - getPosition() always accurate, <1µs latency  │
  └────────────────────────────────────────────────┘
```

**Benefits:**
- `with_dma = false` → All 4 RMT channels available
- On-demand callback → Lower CPU overhead
- PCNT hardware → Real-time position, no software tracking needed
- Command queue → Supports mid-motion changes

---

## 3. Configuration Changes

### 3.1 Changes to `config_limits.h`

```c
/**
 * @file config_limits.h
 * @brief Hardware limits and compile-time constraints
 */

/* ============================================================
 * RMT PULSE GENERATOR CONFIGURATION
 * ============================================================ */

/**
 * @defgroup limits_rmt RMT Pulse Generator Configuration
 * @brief Configuration for FastAccelStepper-pattern RMT pulse generation
 * @{
 */

/**
 * @brief RMT clock resolution (Hz)
 *
 * FastAccelStepper uses 16 MHz providing 62.5ns per tick.
 * At 500 kHz output: period = 32 ticks (adequate precision).
 * At 200 kHz output: period = 80 ticks (good precision).
 *
 * @note Changed from 80 MHz to match FastAccelStepper proven configuration.
 */
#define LIMIT_RMT_RESOLUTION_HZ         16000000

/**
 * @brief RMT memory block size (symbols per channel)
 *
 * ESP32-S3 has SOC_RMT_MEM_WORDS_PER_CHANNEL = 48 symbols per channel.
 * With callback encoder (no DMA), we use exactly one channel's worth.
 *
 * @note Must equal SOC_RMT_MEM_WORDS_PER_CHANNEL for ESP32-S3.
 */
#define LIMIT_RMT_MEM_BLOCK_SYMBOLS     48

/**
 * @brief Symbols per encoder callback (half of memory block)
 *
 * RMT double-buffers internally. Encoder fills PART_SIZE symbols per call.
 * At 500 kHz: 24 symbols = 48µs worth of pulses.
 *
 * @note FastAccelStepper pattern: PART_SIZE = RMT_SIZE >> 1
 */
#define LIMIT_RMT_PART_SIZE             (LIMIT_RMT_MEM_BLOCK_SYMBOLS >> 1)

/**
 * @brief Command queue depth (entries)
 *
 * Ring buffer holding motion commands. FastAccelStepper uses 32.
 * With 20ms forward planning at 500 kHz = ~40 commands needed.
 * 32 entries provides ~16ms buffer at worst case.
 *
 * @note Must be power of 2 for efficient modulo via bitmask.
 */
#define LIMIT_RMT_QUEUE_LEN             32

/**
 * @brief Queue length bitmask for efficient modulo
 */
#define LIMIT_RMT_QUEUE_LEN_MASK        (LIMIT_RMT_QUEUE_LEN - 1)

/**
 * @brief Minimum command period in RMT ticks
 *
 * FastAccelStepper: MIN_CMD_TICKS = TICKS_PER_S / 5000 = 3200 ticks.
 * This limits maximum step frequency to ~5 kHz per command.
 * Actual max frequency achieved by batching multiple steps per command.
 *
 * At 16 MHz: 3200 ticks = 200µs minimum command duration.
 */
#define LIMIT_RMT_MIN_CMD_TICKS         3200

/**
 * @brief Maximum steps per single command
 *
 * Limits how many steps can be batched in one queue entry.
 * At high frequencies, more steps per command = lower ISR rate.
 * At low frequencies, fewer steps per command = lower latency.
 *
 * 255 = uint8_t max (FastAccelStepper compatible)
 */
#define LIMIT_RMT_MAX_STEPS_PER_CMD     255

/**
 * @brief Forward planning time in RMT ticks
 *
 * How far ahead to fill the command queue.
 * FastAccelStepper: ~20ms = TICKS_PER_S / 50 = 320,000 ticks.
 *
 * @note Longer planning = smoother motion but more latency for changes.
 */
#define LIMIT_RMT_FORWARD_PLANNING_TICKS (LIMIT_RMT_RESOLUTION_HZ / 50)

/**
 * @brief Ramp generator task period (ms)
 *
 * How often the ramp generator refills the command queue.
 * FastAccelStepper uses 4ms. Must be < forward planning time.
 */
#define LIMIT_RMT_RAMP_TASK_PERIOD_MS   4

/**
 * @brief Direction pin setup time (µs)
 *
 * Time between direction pin change and first step pulse.
 * Motor driver dependent. Typical: 5-20µs.
 */
#define LIMIT_RMT_DIR_SETUP_US          20

/**
 * @brief Maximum pulse frequency (Hz) - hardware limit
 *
 * At 16 MHz resolution with 50% duty cycle:
 * Min period = MIN_CMD_TICKS = 3200 ticks
 * But with batching, effective max = resolution / 32 = 500 kHz
 *
 * @note Unchanged from previous implementation.
 */
#define LIMIT_MAX_PULSE_FREQ_HZ         500000

/**
 * @brief Default maximum pulse frequency (Hz) - safe per-axis default
 *
 * @note Unchanged from previous implementation.
 */
#define DEFAULT_MAX_PULSE_FREQ_HZ       200000

/**
 * @brief Minimum pulse frequency (Hz)
 *
 * @note Unchanged from previous implementation.
 */
#define LIMIT_MIN_PULSE_FREQ_HZ         1

/** @} */ // end of limits_rmt


/* ============================================================
 * PCNT CONFIGURATION (Updated for RMT position tracking)
 * ============================================================ */

/**
 * @defgroup limits_pcnt PCNT Configuration
 * @brief Pulse counter configuration for position tracking
 * @{
 */

/**
 * @brief Total PCNT units available on ESP32-S3
 *
 * @note ESP32-S3 has 8 PCNT units (0-7), not 4 as previously documented!
 */
#define LIMIT_PCNT_TOTAL_UNITS          8

/**
 * @brief PCNT high limit for overflow detection
 *
 * ESP32-S3 PCNT is 16-bit signed (-32768 to +32767).
 * Set watch point slightly below max to catch overflow before wrap.
 */
#define LIMIT_PCNT_HIGH_LIMIT           32767

/**
 * @brief PCNT low limit for overflow detection
 */
#define LIMIT_PCNT_LOW_LIMIT            (-32767)

/** @} */ // end of limits_pcnt
```

### 3.2 Changes to `config_peripherals.h`

```c
/**
 * @file config_peripherals.h
 * @brief ESP32-S3 peripheral assignments
 */

/* ============================================================
 * PCNT UNIT ASSIGNMENTS (Updated)
 * ============================================================ */

/**
 * @defgroup periph_pcnt PCNT Unit Assignments
 * @brief Pulse counter units for position tracking
 *
 * ESP32-S3 has 8 PCNT units (0-7).
 * - Units 0-2: MCPWM/LEDC axes (Y, C, D) - existing
 * - Units 3-6: RMT axes (X, Z, A, B) - NEW for FastAccelStepper migration
 * - Unit 7: Reserved for future use
 * @{
 */

/** @brief PCNT unit for Y-axis (MCPWM) - unchanged */
#define PCNT_UNIT_Y                     0

/** @brief PCNT unit for C-axis (MCPWM) - unchanged */
#define PCNT_UNIT_C                     1

/** @brief PCNT unit for D-axis (LEDC loopback) - unchanged */
#define PCNT_UNIT_D                     2

/** @brief PCNT unit for X-axis (RMT) - NEW */
#define PCNT_UNIT_X                     3

/** @brief PCNT unit for Z-axis (RMT) - NEW */
#define PCNT_UNIT_Z                     4

/** @brief PCNT unit for A-axis (RMT) - NEW */
#define PCNT_UNIT_A                     5

/** @brief PCNT unit for B-axis (RMT) - NEW */
#define PCNT_UNIT_B                     6

/** @} */ // end of periph_pcnt


/* ============================================================
 * RMT CHANNEL ASSIGNMENTS (Unchanged)
 * ============================================================ */

/**
 * @defgroup periph_rmt RMT Channel Assignments
 * @brief RMT TX channels for step pulse generation
 *
 * ESP32-S3 has 4 RMT TX channels (0-3).
 * All 4 now usable with callback encoder (no DMA).
 * @{
 */

/** @brief RMT channel for X-axis step pulses */
#define RMT_CHANNEL_X                   0

/** @brief RMT channel for Z-axis step pulses */
#define RMT_CHANNEL_Z                   1

/** @brief RMT channel for A-axis step pulses */
#define RMT_CHANNEL_A                   2

/** @brief RMT channel for B-axis step pulses */
#define RMT_CHANNEL_B                   3

/** @} */ // end of periph_rmt
```

### 3.3 Changes to `config_timing.h`

```c
/**
 * @file config_timing.h
 * @brief Timing parameters for motion control
 */

/* ============================================================
 * RMT TIMING (Updated for FastAccelStepper)
 * ============================================================ */

/**
 * @brief RMT ramp generator task period (ms)
 *
 * How often the task refills command queue.
 * FastAccelStepper default: 4ms.
 *
 * @note Replaces continuous refill task with periodic fill.
 */
#define TIMING_RMT_RAMP_TASK_PERIOD_MS  4

/**
 * @brief RMT direction pin setup time (µs)
 *
 * Delay between direction pin change and first step.
 * Motor driver specific. TB6600: 5µs, DM542: 5µs.
 *
 * @note Using conservative 20µs for safety.
 */
#define TIMING_RMT_DIR_SETUP_US         20

/**
 * @brief RMT stop latency maximum (µs)
 *
 * Maximum time for stopImmediate() to halt pulses.
 * With queue-based system: time to drain current command.
 * Worst case: PART_SIZE symbols at min frequency.
 */
#define TIMING_RMT_STOP_LATENCY_US      100
```

---

## 4. Interface Changes

### 4.1 IPulseGenerator Interface - Additions

The existing interface is preserved. New methods are added:

```cpp
/**
 * @file i_pulse_generator.h
 * @brief Abstract interface for pulse generation
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

// Forward declarations
class IPositionTracker;

/**
 * @brief Callback type for motion completion notification
 * @param pulse_count Total pulses generated during motion
 * @param completed True if motion completed normally, false if stopped
 */
using MotionCompleteCallback = void (*)(int64_t pulse_count, bool completed);

/**
 * @brief Abstract interface for pulse generators
 *
 * Implementations: RmtPulseGenerator, McpwmPulseGenerator, LedcPulseGenerator
 */
class IPulseGenerator {
public:
    virtual ~IPulseGenerator() = default;

    // ============================================================
    // EXISTING INTERFACE (UNCHANGED)
    // ============================================================

    /**
     * @brief Initialize the pulse generator hardware
     * @return ESP_OK on success
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Start a position move (exact pulse count)
     *
     * Generates exactly |pulses| step pulses with trapezoidal profile.
     * Sign indicates direction. Fires completion callback when done.
     *
     * @param pulses Target pulse count (negative = reverse direction)
     * @param max_velocity Maximum velocity in pulses/second
     * @param acceleration Acceleration in pulses/second^2
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE if already running
     */
    virtual esp_err_t startMove(int32_t pulses, float max_velocity,
                                float acceleration) = 0;

    /**
     * @brief Start continuous velocity mode
     *
     * Accelerates to target velocity and maintains until stop() called.
     *
     * @param velocity Target velocity in pulses/second (sign = direction)
     * @param acceleration Acceleration in pulses/second^2
     * @return ESP_OK on success
     */
    virtual esp_err_t startVelocity(float velocity, float acceleration) = 0;

    /**
     * @brief Controlled stop with deceleration
     *
     * Decelerates to zero using specified rate. Fires completion callback.
     *
     * @param deceleration Deceleration rate in pulses/second^2
     * @return ESP_OK on success
     */
    virtual esp_err_t stop(float deceleration) = 0;

    /**
     * @brief Emergency stop - immediate halt
     *
     * Stops within TIMING_RMT_STOP_LATENCY_US. Does NOT fire callback.
     */
    virtual void stopImmediate() = 0;

    /**
     * @brief Check if motion is in progress
     * @return true if generating pulses
     */
    virtual bool isRunning() const = 0;

    /**
     * @brief Get current pulse count
     *
     * For RMT axes with PCNT: Returns hardware counter value (real-time).
     * For other axes: Returns software-tracked count.
     *
     * @return Total pulses since last reset (signed for direction)
     */
    virtual int64_t getPulseCount() const = 0;

    /**
     * @brief Get current instantaneous velocity
     * @return Current velocity in pulses/second
     */
    virtual float getCurrentVelocity() const = 0;

    /**
     * @brief Set completion callback
     * @param cb Callback function (called from task context, not ISR)
     */
    virtual void setCompletionCallback(MotionCompleteCallback cb) = 0;

    /**
     * @brief Set position tracker for real-time updates
     *
     * @deprecated For RMT axes, position is tracked via PCNT hardware.
     *             This method is retained for LEDC axis compatibility.
     *
     * @param tracker Position tracker instance
     */
    virtual void setPositionTracker(IPositionTracker* tracker) = 0;

    // ============================================================
    // NEW INTERFACE (FastAccelStepper migration)
    // ============================================================

    /**
     * @brief Change velocity during motion (smooth transition)
     *
     * Atomically updates target velocity. Ramp generator applies change
     * on next cycle using configured acceleration for smooth transition.
     *
     * @param new_velocity New target velocity in pulses/second
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not running
     *
     * @note Direction change requires stop first (returns ESP_ERR_INVALID_ARG)
     */
    virtual esp_err_t setVelocity(float new_velocity) = 0;

    /**
     * @brief Change target position during motion (smooth blending)
     *
     * Recalculates profile to smoothly transition to new target.
     * Current velocity is preserved; profile adjusts accel/decel phases.
     *
     * @param new_pulses New target pulse count (absolute, not relative)
     * @param max_velocity Maximum velocity for remainder of motion
     * @param acceleration Acceleration/deceleration rate
     * @return ESP_OK on success
     *
     * @note If new target is behind current position, decelerates,
     *       reverses direction, and continues to new target.
     */
    virtual esp_err_t setTarget(int32_t new_pulses, float max_velocity,
                                float acceleration) = 0;

    /**
     * @brief Reset position counter to specified value
     *
     * Only valid when not running. Resets both internal counter and
     * PCNT hardware (if applicable).
     *
     * @param position New position value (default 0)
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE if running
     */
    virtual esp_err_t resetPosition(int64_t position = 0) = 0;
};
```

### 4.2 New Methods Summary

| Method | Purpose | When to Use |
|--------|---------|-------------|
| `setVelocity()` | Change speed during motion | VEL command override, jogging |
| `setTarget()` | Change destination during motion | MOVE while moving, trajectory blend |
| `resetPosition()` | Zero or preset position | Homing complete, coordinate reset |

---

## 5. Implementation Details

### 5.1 New Data Structures

```cpp
/**
 * @file rmt_pulse_gen_types.h
 * @brief Type definitions for FastAccelStepper-pattern RMT implementation
 */

#pragma once

#include <cstdint>
#include <atomic>

/**
 * @brief Single motion command in the queue
 *
 * Matches FastAccelStepper queue_entry structure for compatibility.
 * Uses ticks (not frequency) for ISR efficiency - no division needed.
 */
struct StepCommand {
    uint16_t ticks;             ///< Period between steps in RMT ticks (16 MHz)
    uint8_t steps;              ///< Number of steps in this command (0 = pause)
    uint8_t flags;              ///< Control flags (see CMD_FLAG_*)
};

// Command flags
constexpr uint8_t CMD_FLAG_DIRECTION    = 0x01;  ///< Direction: 1=forward, 0=reverse
constexpr uint8_t CMD_FLAG_TOGGLE_DIR   = 0x02;  ///< Toggle direction before this command
constexpr uint8_t CMD_FLAG_LAST         = 0x04;  ///< Last command in motion sequence

/**
 * @brief Ramp generator state
 */
enum class RampState : uint8_t {
    IDLE,           ///< No motion
    ACCELERATING,   ///< Ramping up to target velocity
    CRUISING,       ///< At constant target velocity
    DECELERATING,   ///< Ramping down to stop
    REVERSING       ///< Decelerating for direction change
};

/**
 * @brief Motion profile parameters (atomic for thread safety)
 */
struct RampParameters {
    std::atomic<float> target_velocity{0.0f};    ///< Target velocity (pulses/s)
    std::atomic<float> acceleration{0.0f};       ///< Acceleration rate (pulses/s^2)
    std::atomic<int32_t> target_position{0};     ///< Target position (pulses)
    std::atomic<bool> position_mode{false};      ///< True = position mode, false = velocity mode
    std::atomic<bool> params_changed{false};     ///< Flag: parameters updated, recalc needed
};

/**
 * @brief Queue state tracking
 */
struct QueueState {
    volatile int32_t position;      ///< Position at queue end (planned)
    volatile bool direction;        ///< Direction at queue end
    volatile uint32_t ticks_queued; ///< Total ticks in queue (for forward planning)
};
```

### 5.2 RmtPulseGenerator Class Structure

```cpp
/**
 * @file rmt_pulse_gen.h
 * @brief RMT-based pulse generator using FastAccelStepper patterns
 */

#pragma once

#include "i_pulse_generator.h"
#include "rmt_pulse_gen_types.h"
#include "driver/rmt_tx.h"
#include "driver/pulse_cnt.h"
#include <atomic>

/**
 * @brief RMT pulse generator with callback encoder and PCNT position tracking
 *
 * Implements FastAccelStepper architecture:
 * - Command queue filled by ramp generator task
 * - RMT encoder callback reads from queue (ISR context)
 * - PCNT hardware tracks actual position in real-time
 */
class RmtPulseGenerator : public IPulseGenerator {
public:
    /**
     * @brief Construct RMT pulse generator for specified axis
     *
     * @param axis_id Axis identifier (AXIS_X, AXIS_Z, AXIS_A, AXIS_B)
     * @param step_pin GPIO for step output
     * @param dir_pin GPIO for direction output
     * @param rmt_channel RMT channel number (0-3)
     * @param pcnt_unit PCNT unit number for position tracking
     */
    RmtPulseGenerator(uint8_t axis_id, gpio_num_t step_pin, gpio_num_t dir_pin,
                      uint8_t rmt_channel, uint8_t pcnt_unit);

    ~RmtPulseGenerator() override;

    // IPulseGenerator interface implementation
    esp_err_t init() override;
    esp_err_t startMove(int32_t pulses, float max_velocity,
                        float acceleration) override;
    esp_err_t startVelocity(float velocity, float acceleration) override;
    esp_err_t stop(float deceleration) override;
    void stopImmediate() override;
    bool isRunning() const override;
    int64_t getPulseCount() const override;
    float getCurrentVelocity() const override;
    void setCompletionCallback(MotionCompleteCallback cb) override;
    void setPositionTracker(IPositionTracker* tracker) override;

    // New interface methods
    esp_err_t setVelocity(float new_velocity) override;
    esp_err_t setTarget(int32_t new_pulses, float max_velocity,
                        float acceleration) override;
    esp_err_t resetPosition(int64_t position = 0) override;

private:
    // ============================================================
    // HARDWARE HANDLES
    // ============================================================

    rmt_channel_handle_t rmt_channel_{nullptr};
    rmt_encoder_handle_t rmt_encoder_{nullptr};
    pcnt_unit_handle_t pcnt_unit_{nullptr};
    pcnt_channel_handle_t pcnt_channel_{nullptr};

    // ============================================================
    // CONFIGURATION (immutable after construction)
    // ============================================================

    const uint8_t axis_id_;
    const gpio_num_t step_pin_;
    const gpio_num_t dir_pin_;
    const uint8_t rmt_channel_num_;
    const uint8_t pcnt_unit_num_;

    // ============================================================
    // COMMAND QUEUE (FastAccelStepper pattern)
    // ============================================================

    StepCommand queue_[LIMIT_RMT_QUEUE_LEN];
    std::atomic<uint8_t> read_idx_{0};      ///< ISR reads from here
    std::atomic<uint8_t> write_idx_{0};     ///< Task writes to here
    QueueState queue_end_{};                 ///< State at queue end

    // ============================================================
    // RAMP GENERATOR STATE
    // ============================================================

    RampParameters params_;                  ///< Motion parameters (atomic)
    RampState ramp_state_{RampState::IDLE};
    float current_velocity_{0.0f};           ///< Instantaneous velocity
    uint32_t current_ticks_{0};              ///< Current step period in ticks
    int32_t ramp_steps_up_{0};               ///< Steps in acceleration phase
    int32_t ramp_steps_down_{0};             ///< Steps in deceleration phase

    // ============================================================
    // POSITION TRACKING
    // ============================================================

    std::atomic<int64_t> pcnt_overflow_acc_{0};  ///< PCNT overflow accumulator
    std::atomic<bool> direction_{true};          ///< Current direction

    // ============================================================
    // STATE FLAGS
    // ============================================================

    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> rmt_stopped_{true};
    std::atomic<bool> completion_pending_{false};

    // ============================================================
    // CALLBACKS
    // ============================================================

    MotionCompleteCallback completion_callback_{nullptr};
    IPositionTracker* position_tracker_{nullptr};  ///< Legacy (deprecated for RMT)

    // ============================================================
    // PRIVATE METHODS
    // ============================================================

    // Initialization
    esp_err_t initRmt();
    esp_err_t initPcnt();
    esp_err_t createEncoder();

    // Queue operations
    bool isQueueFull() const;
    bool isQueueEmpty() const;
    uint8_t queueSpace() const;
    uint32_t ticksInQueue() const;
    bool pushCommand(const StepCommand& cmd);
    void clearQueue();

    // Ramp generation
    void fillQueue();
    StepCommand generateNextCommand();
    void recalculateRamp();
    uint32_t calculateTicks(uint32_t ramp_steps) const;

    // Motion control
    void startMotion();
    void stopMotion();

    // Position
    int16_t readPcntRaw() const;

    // Static callbacks (ISR context)
    static size_t IRAM_ATTR encodeCallback(
        const void* data, size_t data_size,
        size_t symbols_written, size_t symbols_free,
        rmt_symbol_word_t* symbols, bool* done, void* arg);

    static bool IRAM_ATTR onPcntOverflow(
        pcnt_unit_handle_t unit,
        const pcnt_watch_event_data_t* event_data, void* user_ctx);

    static bool IRAM_ATTR onTransmitDone(
        rmt_channel_handle_t channel,
        const rmt_tx_done_event_data_t* event_data, void* user_ctx);

    // Symbol generation (ISR context - no FPU)
    void IRAM_ATTR fillSymbols(rmt_symbol_word_t* symbols,
                                const StepCommand& cmd);
};
```

### 5.3 Key Method Implementations

#### 5.3.1 Initialization

```cpp
esp_err_t RmtPulseGenerator::init() {
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Initialize direction pin
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << dir_pin_),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&dir_conf));
    gpio_set_level(dir_pin_, direction_ ? 1 : 0);

    // Initialize RMT with callback encoder
    ESP_ERROR_CHECK(initRmt());

    // Initialize PCNT for position tracking
    ESP_ERROR_CHECK(initPcnt());

    initialized_ = true;
    return ESP_OK;
}

esp_err_t RmtPulseGenerator::initRmt() {
    // Create RMT TX channel (NO DMA)
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = step_pin_,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LIMIT_RMT_RESOLUTION_HZ,  // 16 MHz
        .mem_block_symbols = LIMIT_RMT_MEM_BLOCK_SYMBOLS,  // 48
        .trans_queue_depth = 1,
        .intr_priority = 0,
        .flags = {
            .invert_out = 0,
            .with_dma = 0,  // CRITICAL: No DMA
            .io_loop_back = 0,
            .io_od_mode = 0
        }
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &rmt_channel_));

    // Create callback encoder
    ESP_ERROR_CHECK(createEncoder());

    // Register transmit done callback
    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = onTransmitDone
    };
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(rmt_channel_, &cbs, this));

    // Enable channel
    ESP_ERROR_CHECK(rmt_enable(rmt_channel_));

    return ESP_OK;
}

esp_err_t RmtPulseGenerator::createEncoder() {
    // Create simple encoder with callback
    rmt_simple_encoder_config_t enc_config = {
        .callback = encodeCallback,
        .arg = this,
        .min_chunk_size = LIMIT_RMT_PART_SIZE  // 24 symbols
    };
    return rmt_new_simple_encoder(&enc_config, &rmt_encoder_);
}

esp_err_t RmtPulseGenerator::initPcnt() {
    // Create PCNT unit
    pcnt_unit_config_t unit_config = {
        .low_limit = LIMIT_PCNT_LOW_LIMIT,
        .high_limit = LIMIT_PCNT_HIGH_LIMIT,
        .intr_priority = 0,
        .flags = {
            .accum_count = 1  // Enable accumulator mode
        }
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_));

    // Create PCNT channel watching step pin
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = step_pin_,
        .level_gpio_num = dir_pin_,  // Use direction pin for up/down
        .flags = {
            .invert_edge_input = 0,
            .invert_level_input = 0,
            .virt_edge_io_level = 0,
            .virt_level_io_level = 0,
            .io_loop_back = 0
        }
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_, &chan_config, &pcnt_channel_));

    // Configure channel: count on rising edge, direction from level
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel_,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // Rising edge
        PCNT_CHANNEL_EDGE_ACTION_HOLD));    // Falling edge

    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel_,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,     // Level high: count up
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE)); // Level low: count down

    // Add watch points for overflow detection
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_, LIMIT_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_, LIMIT_PCNT_LOW_LIMIT));

    // Register overflow callback
    pcnt_event_callbacks_t pcnt_cbs = {
        .on_reach = onPcntOverflow
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit_, &pcnt_cbs, this));

    // Enable and start PCNT
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_));

    return ESP_OK;
}
```

#### 5.3.2 Encoder Callback (ISR Context)

```cpp
size_t IRAM_ATTR RmtPulseGenerator::encodeCallback(
    const void* data, size_t data_size,
    size_t symbols_written, size_t symbols_free,
    rmt_symbol_word_t* symbols, bool* done, void* arg)
{
    RmtPulseGenerator* self = static_cast<RmtPulseGenerator*>(arg);
    *done = false;

    // Need at least PART_SIZE symbols available
    if (symbols_free < LIMIT_RMT_PART_SIZE) {
        return 0;
    }

    // Check if stopped
    if (self->rmt_stopped_.load(std::memory_order_acquire)) {
        *done = true;
        return 0;
    }

    // Check queue
    uint8_t rp = self->read_idx_.load(std::memory_order_acquire);
    uint8_t wp = self->write_idx_.load(std::memory_order_acquire);

    if (rp == wp) {
        // Queue empty - stop transmission
        self->rmt_stopped_.store(true, std::memory_order_release);
        self->completion_pending_.store(true, std::memory_order_release);
        *done = true;
        return 0;
    }

    // Get command from queue
    const StepCommand& cmd = self->queue_[rp & LIMIT_RMT_QUEUE_LEN_MASK];

    // Advance read pointer
    self->read_idx_.store(rp + 1, std::memory_order_release);

    // Handle direction toggle
    if (cmd.flags & CMD_FLAG_TOGGLE_DIR) {
        // Toggle direction pin (ISR-safe GPIO operation)
        bool new_dir = !self->direction_.load(std::memory_order_relaxed);
        gpio_set_level(self->dir_pin_, new_dir ? 1 : 0);
        self->direction_.store(new_dir, std::memory_order_relaxed);
    }

    // Fill RMT symbols from command
    self->fillSymbols(symbols, cmd);

    return LIMIT_RMT_PART_SIZE;
}

void IRAM_ATTR RmtPulseGenerator::fillSymbols(
    rmt_symbol_word_t* symbols, const StepCommand& cmd)
{
    const uint16_t ticks = cmd.ticks;
    const uint8_t steps = cmd.steps;
    const uint16_t half_ticks = ticks >> 1;

    if (steps == 0) {
        // Pause command - fill with idle symbols
        for (int i = 0; i < LIMIT_RMT_PART_SIZE; i++) {
            symbols[i].duration0 = half_ticks;
            symbols[i].level0 = 0;
            symbols[i].duration1 = half_ticks;
            symbols[i].level1 = 0;
        }
        return;
    }

    // Calculate symbols needed
    // Each step needs one full pulse (high + low)
    // Distribute steps across PART_SIZE symbols

    if (steps >= LIMIT_RMT_PART_SIZE) {
        // High frequency: one step per symbol
        for (int i = 0; i < LIMIT_RMT_PART_SIZE; i++) {
            symbols[i].duration0 = half_ticks;
            symbols[i].level0 = 1;  // Step pulse HIGH
            symbols[i].duration1 = half_ticks;
            symbols[i].level1 = 0;  // Step pulse LOW
        }
    } else {
        // Lower frequency: spread steps with padding
        int symbol_idx = 0;
        int step_count = 0;

        // Calculate spacing between steps
        int spacing = LIMIT_RMT_PART_SIZE / steps;

        for (int i = 0; i < LIMIT_RMT_PART_SIZE; i++) {
            if (step_count < steps && (i % spacing == 0)) {
                // Generate step pulse
                symbols[i].duration0 = half_ticks;
                symbols[i].level0 = 1;
                symbols[i].duration1 = half_ticks;
                symbols[i].level1 = 0;
                step_count++;
            } else {
                // Generate idle symbol
                symbols[i].duration0 = half_ticks;
                symbols[i].level0 = 0;
                symbols[i].duration1 = half_ticks;
                symbols[i].level1 = 0;
            }
        }
    }
}
```

#### 5.3.3 Position Tracking with PCNT

```cpp
int64_t RmtPulseGenerator::getPulseCount() const {
    // Read PCNT hardware counter
    int16_t pcnt_raw = readPcntRaw();

    // Combine with overflow accumulator
    int64_t overflow = pcnt_overflow_acc_.load(std::memory_order_acquire);

    return overflow + pcnt_raw;
}

int16_t RmtPulseGenerator::readPcntRaw() const {
    int count = 0;
    pcnt_unit_get_count(pcnt_unit_, &count);
    return static_cast<int16_t>(count);
}

bool IRAM_ATTR RmtPulseGenerator::onPcntOverflow(
    pcnt_unit_handle_t unit,
    const pcnt_watch_event_data_t* event_data,
    void* user_ctx)
{
    RmtPulseGenerator* self = static_cast<RmtPulseGenerator*>(user_ctx);

    // Accumulate overflow
    if (event_data->watch_point_value == LIMIT_PCNT_HIGH_LIMIT) {
        // Positive overflow
        self->pcnt_overflow_acc_.fetch_add(LIMIT_PCNT_HIGH_LIMIT,
                                            std::memory_order_relaxed);
    } else if (event_data->watch_point_value == LIMIT_PCNT_LOW_LIMIT) {
        // Negative overflow
        self->pcnt_overflow_acc_.fetch_add(LIMIT_PCNT_LOW_LIMIT,
                                            std::memory_order_relaxed);
    }

    return false;  // Don't yield
}

esp_err_t RmtPulseGenerator::resetPosition(int64_t position) {
    if (running_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Reset PCNT hardware
    pcnt_unit_clear_count(pcnt_unit_);

    // Set overflow accumulator to desired position
    pcnt_overflow_acc_.store(position, std::memory_order_release);

    return ESP_OK;
}
```

#### 5.3.4 Mid-Motion Velocity Change

```cpp
esp_err_t RmtPulseGenerator::setVelocity(float new_velocity) {
    if (!running_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Check for direction change (not allowed mid-motion)
    bool new_direction = (new_velocity >= 0);
    bool current_direction = direction_.load(std::memory_order_acquire);

    if (new_direction != current_direction) {
        return ESP_ERR_INVALID_ARG;  // Must stop first for direction change
    }

    // Update target velocity atomically
    params_.target_velocity.store(fabsf(new_velocity), std::memory_order_release);
    params_.params_changed.store(true, std::memory_order_release);

    return ESP_OK;
}
```

#### 5.3.5 Mid-Motion Target Change (Blending)

```cpp
esp_err_t RmtPulseGenerator::setTarget(int32_t new_pulses, float max_velocity,
                                       float acceleration) {
    // Get current position from PCNT
    int64_t current_pos = getPulseCount();

    // Calculate remaining distance
    int32_t remaining = new_pulses - static_cast<int32_t>(current_pos);

    // Update parameters atomically
    params_.target_position.store(new_pulses, std::memory_order_relaxed);
    params_.target_velocity.store(max_velocity, std::memory_order_relaxed);
    params_.acceleration.store(acceleration, std::memory_order_relaxed);
    params_.position_mode.store(true, std::memory_order_relaxed);
    params_.params_changed.store(true, std::memory_order_release);

    // If direction reversal needed, ramp generator will handle it
    // by transitioning through REVERSING state

    return ESP_OK;
}
```

#### 5.3.6 Ramp Generator Queue Fill

```cpp
void RmtPulseGenerator::fillQueue() {
    // Check for parameter changes
    if (params_.params_changed.exchange(false, std::memory_order_acq_rel)) {
        recalculateRamp();
    }

    // Fill queue while space available and motion not complete
    while (queueSpace() > 0 && ramp_state_ != RampState::IDLE) {
        StepCommand cmd = generateNextCommand();

        if (cmd.ticks == 0) {
            // Motion complete
            break;
        }

        pushCommand(cmd);

        // Update queue state
        if (cmd.flags & CMD_FLAG_DIRECTION) {
            queue_end_.position += cmd.steps;
        } else {
            queue_end_.position -= cmd.steps;
        }
        queue_end_.ticks_queued += cmd.ticks * cmd.steps;

        if (cmd.flags & CMD_FLAG_LAST) {
            ramp_state_ = RampState::IDLE;
            break;
        }
    }
}

StepCommand RmtPulseGenerator::generateNextCommand() {
    StepCommand cmd = {};

    // Get current parameters
    float target_vel = params_.target_velocity.load(std::memory_order_acquire);
    float accel = params_.acceleration.load(std::memory_order_acquire);

    // State machine for ramp generation
    switch (ramp_state_) {
        case RampState::ACCELERATING: {
            // Calculate velocity at current ramp position
            float velocity = sqrtf(2.0f * accel * static_cast<float>(ramp_steps_up_));

            if (velocity >= target_vel) {
                // Reached target velocity
                velocity = target_vel;
                ramp_state_ = RampState::CRUISING;
            }

            // Convert to ticks
            current_velocity_ = velocity;
            current_ticks_ = (velocity > 0) ?
                (LIMIT_RMT_RESOLUTION_HZ / static_cast<uint32_t>(velocity)) :
                LIMIT_RMT_MIN_CMD_TICKS;

            // Calculate steps for this command
            uint8_t steps = calculateStepsForLatency(current_ticks_);
            ramp_steps_up_ += steps;

            cmd.ticks = current_ticks_;
            cmd.steps = steps;
            cmd.flags = direction_.load() ? CMD_FLAG_DIRECTION : 0;
            break;
        }

        case RampState::CRUISING: {
            // Constant velocity
            if (params_.position_mode.load()) {
                // Check if need to start deceleration
                int32_t target = params_.target_position.load();
                int32_t decel_distance = calculateDecelDistance(current_velocity_, accel);
                int32_t remaining = target - queue_end_.position;

                if (remaining <= decel_distance) {
                    ramp_state_ = RampState::DECELERATING;
                    ramp_steps_down_ = 0;
                }
            }

            uint8_t steps = calculateStepsForLatency(current_ticks_);
            cmd.ticks = current_ticks_;
            cmd.steps = steps;
            cmd.flags = direction_.load() ? CMD_FLAG_DIRECTION : 0;
            break;
        }

        case RampState::DECELERATING: {
            // Calculate velocity from decel ramp
            float velocity = sqrtf(2.0f * accel *
                static_cast<float>(calculateDecelDistance(current_velocity_, accel) - ramp_steps_down_));

            if (velocity <= LIMIT_MIN_PULSE_FREQ_HZ) {
                // Motion complete
                cmd.ticks = 0;
                cmd.steps = 0;
                cmd.flags = CMD_FLAG_LAST;
                break;
            }

            current_velocity_ = velocity;
            current_ticks_ = LIMIT_RMT_RESOLUTION_HZ / static_cast<uint32_t>(velocity);

            uint8_t steps = calculateStepsForLatency(current_ticks_);
            ramp_steps_down_ += steps;

            cmd.ticks = current_ticks_;
            cmd.steps = steps;
            cmd.flags = direction_.load() ? CMD_FLAG_DIRECTION : 0;
            break;
        }

        case RampState::REVERSING: {
            // Decelerate to zero, then change direction
            // Similar to DECELERATING but triggers direction toggle at end
            // ... (implementation similar to DECELERATING with direction change)
            break;
        }

        default:
            cmd.ticks = 0;
            break;
    }

    return cmd;
}

/**
 * @brief Calculate steps per command for bounded latency
 *
 * At high frequencies, more steps per command reduces ISR rate.
 * At low frequencies, fewer steps bounds velocity change latency.
 *
 * Target: max 10ms latency at any frequency.
 */
uint8_t RmtPulseGenerator::calculateStepsForLatency(uint32_t ticks) const {
    // Frequency in Hz
    uint32_t freq = LIMIT_RMT_RESOLUTION_HZ / ticks;

    // Max 10ms worth of steps
    uint32_t max_steps = freq / 100;  // freq * 0.01s

    // Clamp to valid range
    if (max_steps < 1) max_steps = 1;
    if (max_steps > LIMIT_RMT_MAX_STEPS_PER_CMD) {
        max_steps = LIMIT_RMT_MAX_STEPS_PER_CMD;
    }

    return static_cast<uint8_t>(max_steps);
}
```

---

## 6. Factory and Axis Configuration

### 6.1 Updated Factory Method

```cpp
/**
 * @file pulse_gen_factory.cpp
 * @brief Factory for creating axis-specific pulse generators
 */

#include "pulse_gen_factory.h"
#include "rmt_pulse_gen.h"
#include "mcpwm_pulse_gen.h"
#include "ledc_pulse_gen.h"

std::unique_ptr<IPulseGenerator> PulseGenFactory::create(AxisId axis) {
    switch (axis) {
        // RMT axes (FastAccelStepper pattern + PCNT)
        case AXIS_X:
            return std::make_unique<RmtPulseGenerator>(
                AXIS_X,
                GPIO_STEP_X, GPIO_DIR_X,
                RMT_CHANNEL_X, PCNT_UNIT_X);

        case AXIS_Z:
            return std::make_unique<RmtPulseGenerator>(
                AXIS_Z,
                GPIO_STEP_Z, GPIO_DIR_Z,
                RMT_CHANNEL_Z, PCNT_UNIT_Z);

        case AXIS_A:
            return std::make_unique<RmtPulseGenerator>(
                AXIS_A,
                GPIO_STEP_A, GPIO_DIR_A,
                RMT_CHANNEL_A, PCNT_UNIT_A);

        case AXIS_B:
            return std::make_unique<RmtPulseGenerator>(
                AXIS_B,
                GPIO_STEP_B, GPIO_DIR_B,
                RMT_CHANNEL_B, PCNT_UNIT_B);

        // MCPWM axes (unchanged)
        case AXIS_Y:
            return std::make_unique<McpwmPulseGenerator>(
                AXIS_Y,
                GPIO_STEP_Y, GPIO_DIR_Y,
                MCPWM_TIMER_Y, PCNT_UNIT_Y);

        case AXIS_C:
            return std::make_unique<McpwmPulseGenerator>(
                AXIS_C,
                GPIO_STEP_C, GPIO_DIR_C,
                MCPWM_TIMER_C, PCNT_UNIT_C);

        // LEDC axis (unchanged)
        case AXIS_D:
            return std::make_unique<LedcPulseGenerator>(
                AXIS_D,
                GPIO_STEP_D, GPIO_DIR_D,
                LEDC_CHANNEL_D, PCNT_UNIT_D);

        default:
            return nullptr;
    }
}
```

---

## 7. Migration Phases

### Phase 1: Configuration Updates
**Files:** `config_limits.h`, `config_peripherals.h`, `config_timing.h`
- Add new RMT constants (16 MHz resolution, queue parameters)
- Add PCNT unit assignments for X, Z, A, B
- Update timing constants

### Phase 2: Data Structures
**Files:** `rmt_pulse_gen_types.h` (new)
- Define `StepCommand` structure
- Define `RampState` enum
- Define `RampParameters` structure
- Define `QueueState` structure

### Phase 3: PCNT Integration
**Files:** `rmt_pulse_gen.cpp`
- Implement `initPcnt()`
- Implement `onPcntOverflow()` ISR callback
- Implement `getPulseCount()` using PCNT
- Implement `resetPosition()`

### Phase 4: RMT Callback Encoder
**Files:** `rmt_pulse_gen.cpp`
- Implement `initRmt()` with callback encoder
- Implement `encodeCallback()` ISR
- Implement `fillSymbols()` ISR
- Remove DMA buffer management

### Phase 5: Ramp Generator
**Files:** `rmt_pulse_gen.cpp`
- Implement command queue management
- Implement `fillQueue()` task function
- Implement `generateNextCommand()` state machine
- Implement `calculateStepsForLatency()`

### Phase 6: Motion Control Methods
**Files:** `rmt_pulse_gen.cpp`
- Implement `startMove()` using queue
- Implement `startVelocity()` using queue
- Implement `stop()` with deceleration
- Implement `stopImmediate()`

### Phase 7: Mid-Motion Changes
**Files:** `rmt_pulse_gen.cpp`
- Implement `setVelocity()` for velocity changes
- Implement `setTarget()` for target blending
- Implement ramp recalculation on parameter change

### Phase 8: Integration and Testing
- Update factory to use new implementation
- Run existing unit tests (must pass)
- Add new tests for mid-motion changes
- Hardware verification on all 4 axes simultaneously

---

## 8. Test Strategy

### 8.1 Preserved Tests (Must Pass)

```cpp
// All existing IPulseGenerator tests unchanged
TEST(RmtPulseGenerator, InitializesCorrectly);
TEST(RmtPulseGenerator, StartMoveGeneratesExactPulseCount);
TEST(RmtPulseGenerator, StartVelocityAcceleratesToTarget);
TEST(RmtPulseGenerator, StopDeceleratesCorrectly);
TEST(RmtPulseGenerator, StopImmediateStopsWithoutCallback);
TEST(RmtPulseGenerator, CompletionCallbackFires);
TEST(RmtPulseGenerator, FrequencyRangeValidation);
```

### 8.2 New Tests

```cpp
// Multi-channel operation (critical!)
TEST(RmtPulseGenerator, FourChannelsSimultaneously) {
    // Create and init all 4 RMT generators
    auto x = factory.create(AXIS_X);
    auto z = factory.create(AXIS_Z);
    auto a = factory.create(AXIS_A);
    auto b = factory.create(AXIS_B);

    // Start all at 200 kHz
    x->startVelocity(200000, 1000000);
    z->startVelocity(200000, 1000000);
    a->startVelocity(200000, 1000000);
    b->startVelocity(200000, 1000000);

    // Verify all running
    EXPECT_TRUE(x->isRunning());
    EXPECT_TRUE(z->isRunning());
    EXPECT_TRUE(a->isRunning());
    EXPECT_TRUE(b->isRunning());

    // Wait and verify pulses
    vTaskDelay(pdMS_TO_TICKS(100));
    EXPECT_GT(x->getPulseCount(), 15000);  // ~20000 expected
    EXPECT_GT(z->getPulseCount(), 15000);
    EXPECT_GT(a->getPulseCount(), 15000);
    EXPECT_GT(b->getPulseCount(), 15000);
}

// PCNT position tracking accuracy
TEST(RmtPulseGenerator, PcntPositionAccuracy) {
    auto gen = factory.create(AXIS_X);
    gen->init();

    // Move exact count
    gen->startMove(10000, 100000, 500000);
    waitForCompletion(gen.get());

    // PCNT should match exactly
    EXPECT_EQ(gen->getPulseCount(), 10000);
}

// Mid-motion velocity change
TEST(RmtPulseGenerator, VelocityChangeMidMotion) {
    auto gen = factory.create(AXIS_X);
    gen->init();

    // Start at 100 kHz
    gen->startVelocity(100000, 500000);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Change to 50 kHz
    EXPECT_EQ(gen->setVelocity(50000), ESP_OK);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Verify velocity changed (approximately)
    float vel = gen->getCurrentVelocity();
    EXPECT_NEAR(vel, 50000, 5000);  // ±10% tolerance
}

// Mid-motion target change
TEST(RmtPulseGenerator, TargetChangeMidMotion) {
    auto gen = factory.create(AXIS_X);
    gen->init();

    // Start move to 10000
    gen->startMove(10000, 100000, 500000);
    vTaskDelay(pdMS_TO_TICKS(30));  // Partial completion

    // Change target to 20000
    EXPECT_EQ(gen->setTarget(20000, 100000, 500000), ESP_OK);
    waitForCompletion(gen.get());

    // Should reach new target
    EXPECT_EQ(gen->getPulseCount(), 20000);
}

// Position reset
TEST(RmtPulseGenerator, PositionReset) {
    auto gen = factory.create(AXIS_X);
    gen->init();

    // Move some pulses
    gen->startMove(5000, 100000, 500000);
    waitForCompletion(gen.get());
    EXPECT_EQ(gen->getPulseCount(), 5000);

    // Reset to zero
    EXPECT_EQ(gen->resetPosition(0), ESP_OK);
    EXPECT_EQ(gen->getPulseCount(), 0);

    // Reset to arbitrary value
    EXPECT_EQ(gen->resetPosition(1000), ESP_OK);
    EXPECT_EQ(gen->getPulseCount(), 1000);
}

// Cannot reset while running
TEST(RmtPulseGenerator, ResetWhileRunningFails) {
    auto gen = factory.create(AXIS_X);
    gen->init();

    gen->startVelocity(100000, 500000);
    EXPECT_EQ(gen->resetPosition(0), ESP_ERR_INVALID_STATE);
    gen->stopImmediate();
}
```

---

## 9. Verification Checklist

### 9.1 Functional Requirements

| Requirement | Test | Status |
|-------------|------|--------|
| Position queryable <10ms latency | PCNT direct read | |
| 1000 Hz motion update rate | Ramp task at 4ms | |
| No jitter >1ms | Hardware pulse generation | |
| 8 simultaneous axes | 4 RMT + 2 MCPWM + 1 LEDC + E | |
| Mid-motion velocity change | setVelocity() | |
| Mid-motion target change | setTarget() | |
| Smooth blending | Trapezoidal profile | |
| 500 kHz max frequency | Per-axis configurable | |
| 200 kHz default | config_limits.h | |

### 9.2 Configuration Constants

| Constant | Value | Verified |
|----------|-------|----------|
| LIMIT_RMT_RESOLUTION_HZ | 16,000,000 | |
| LIMIT_RMT_MEM_BLOCK_SYMBOLS | 48 | |
| LIMIT_RMT_PART_SIZE | 24 | |
| LIMIT_RMT_QUEUE_LEN | 32 | |
| LIMIT_RMT_MIN_CMD_TICKS | 3200 | |
| PCNT_UNIT_X | 3 | |
| PCNT_UNIT_Z | 4 | |
| PCNT_UNIT_A | 5 | |
| PCNT_UNIT_B | 6 | |

---

## 10. References

- FastAccelStepper Library: https://github.com/gin66/FastAccelStepper
- ESP-IDF RMT Driver: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/rmt.html
- ESP-IDF PCNT Driver: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/pcnt.html
- ESP32-S3 Technical Reference Manual (RMT, PCNT chapters)
- YaRobot PRD: `/docs/prd.md`
- YaRobot Architecture: `/docs/architecture.md`
