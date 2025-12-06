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
| Position Tracking | Encoder callback | ESP32-S3 has only 4 PCNT units; 3 already used (Y,C,D) |
| Integration | Port patterns to IPulseGenerator | No external dependency; clean interface |
| Mid-Motion | Full blending (velocity + target) | PRD requirement for smooth transitions |

### 1.2 Hardware Constraints

**PCNT Units on ESP32-S3:** Only 4 units (0-3) available, NOT 8.
- Unit 0: Y-axis (MCPWM)
- Unit 1: C-axis (MCPWM)
- Unit 2: D-axis (LEDC loopback)
- Unit 3: **Only 1 spare** - insufficient for 4 RMT axes

**Solution:** Track position in encoder callback (ISR context) by counting pulses as they're generated. This provides bounded latency position updates.

### 1.3 Benefits

- **All 4 RMT channels usable simultaneously** (current: channel exhaustion issue)
- **Bounded-latency position tracking** via encoder callback (<10ms at all frequencies)
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

### 2.2 New Architecture (Callback Encoder + Software Position Tracking)

```
┌─────────────────────────────────────────────────────────────┐
│     NEW: CALLBACK ENCODER + ENCODER-BASED POSITION TRACKING │
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
  │ - TRACKS POSITION: pulse_count += cmd.steps    │
  │ - Calls IPositionTracker::addPulses()          │
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
  └────────────────────────────────────────────────┘

  Position Update Latency (bounded by steps per command):
  ┌──────────────┬────────────────┬──────────────────┐
  │ Frequency    │ Steps/Command  │ Update Latency   │
  ├──────────────┼────────────────┼──────────────────┤
  │ 200 kHz      │ 32 (max)       │ 0.16 ms          │
  │ 50 kHz       │ 32 (max)       │ 0.64 ms          │
  │ 10 kHz       │ 32 (max)       │ 3.2 ms           │
  │ 1 kHz        │ 10             │ 10 ms            │
  │ 100 Hz       │ 1              │ 10 ms            │
  └──────────────┴────────────────┴──────────────────┘
```

**Benefits:**
- `with_dma = false` → All 4 RMT channels available
- On-demand callback → Lower CPU overhead
- Position tracked in callback → <10ms latency at all frequencies
- Command queue → Supports mid-motion changes
- No additional hardware required

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
 * With 10ms forward planning at 500 kHz = ~20 commands needed.
 * 32 entries provides sufficient buffer with margin.
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
 * Set to 10ms (160,000 ticks at 16MHz) to meet <10ms blend response latency.
 *
 * Trade-off: shorter planning = faster blend response but higher risk of
 * queue underrun at very high step rates. 10ms provides 5000 steps buffer
 * at 500kHz max frequency - sufficient margin.
 *
 * @note Reduced from FastAccelStepper's 20ms to meet PRD <10ms requirement.
 */
#define LIMIT_RMT_FORWARD_PLANNING_TICKS (LIMIT_RMT_RESOLUTION_HZ / 100)  // 10ms

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
 * PCNT CONFIGURATION (Unchanged - used by MCPWM/LEDC axes only)
 * ============================================================ */

/**
 * @defgroup limits_pcnt PCNT Configuration
 * @brief Pulse counter configuration for position tracking
 * @{
 */

/**
 * @brief Total PCNT units available on ESP32-S3
 *
 * @note ESP32-S3 has only 4 PCNT units (0-3).
 *       All are allocated: Y=0, C=1, D=2, spare=3
 *       RMT axes use encoder callback position tracking instead.
 */
#define LIMIT_PCNT_TOTAL_UNITS          4

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
 * PCNT UNIT ASSIGNMENTS (Unchanged)
 * ============================================================ */

/**
 * @defgroup periph_pcnt PCNT Unit Assignments
 * @brief Pulse counter units for position tracking
 *
 * ESP32-S3 has only 4 PCNT units (0-3).
 * - Unit 0: Y-axis (MCPWM)
 * - Unit 1: C-axis (MCPWM)
 * - Unit 2: D-axis (LEDC loopback)
 * - Unit 3: Reserved/spare
 *
 * RMT axes (X, Z, A, B) use encoder callback position tracking
 * instead of PCNT due to hardware limitation.
 * @{
 */

/** @brief PCNT unit for Y-axis (MCPWM) - unchanged */
#define PCNT_UNIT_Y                     0

/** @brief PCNT unit for C-axis (MCPWM) - unchanged */
#define PCNT_UNIT_C                     1

/** @brief PCNT unit for D-axis (LEDC loopback) - unchanged */
#define PCNT_UNIT_D                     2

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

### 4.1 IPulseGenerator Interface - No New Methods

**Key architectural decision:** Mid-motion blending is handled at the **ServoMotor level**, not the IPulseGenerator level (see `/docs/architecture.md`).

The existing `IPulseGenerator` interface is **unchanged**. The critical change is behavioral:

**`startMove()` now works while running** - Calling `startMove()` while motion is in progress atomically replaces the current motion profile. The ramp generator smoothly transitions from current velocity to the new profile.

```cpp
/**
 * @file i_pulse_generator.h
 * @brief Abstract interface for pulse generation (UNCHANGED)
 */

class IPulseGenerator {
public:
    virtual ~IPulseGenerator() = default;

    virtual esp_err_t init() = 0;

    /**
     * @brief Start a position move (exact pulse count)
     *
     * Generates exactly |pulses| step pulses with trapezoidal profile.
     * Sign indicates direction. Fires completion callback when done.
     *
     * **CRITICAL CHANGE:** Can be called while running.
     * If called during active motion, atomically replaces the profile.
     * Ramp generator blends from current velocity to new profile.
     *
     * @param pulses Target pulse count (negative = reverse direction)
     * @param max_velocity Maximum velocity in pulses/second
     * @param acceleration Acceleration in pulses/second^2
     * @return ESP_OK on success
     */
    virtual esp_err_t startMove(int32_t pulses, float max_velocity,
                                float acceleration) = 0;

    virtual esp_err_t startVelocity(float velocity, float acceleration) = 0;
    virtual esp_err_t stop(float deceleration) = 0;
    virtual void stopImmediate() = 0;
    virtual bool isRunning() const = 0;
    virtual int64_t getPulseCount() const = 0;
    virtual float getCurrentVelocity() const = 0;
    virtual void setCompletionCallback(MotionCompleteCallback cb) = 0;
    virtual void setPositionTracker(IPositionTracker* tracker) = 0;
};
```

### 4.2 Blending Architecture (Motor Level)

From `/docs/architecture.md`:
> "New MOVE commands during active motion blend to the new target. This is non-negotiable."

From `/firmware/components/motor/include/i_motor.h`:
> "If called during motion, blends to the new target (no 'axis busy' error)"

Blending is orchestrated by **MotorBase** (the common base class for ServoMotor and StepperMotor):

```
User Request (MOVE while moving)
        │
        ▼
┌─────────────────────────────────────┐
│ MotorBase::moveAbsolute()           │
│ - Gets current position/velocity    │
│ - Converts SI units to pulses:      │
│   · velocityToFrequency()           │
│   · accelerationToPulses()          │
│ - Computes relative pulses needed   │
└──────────────────┬──────────────────┘
                   │
                   ▼
┌─────────────────────────────────────┐
│ IPulseGenerator::startMove()        │
│ (called while running - ALLOWED)    │
│ - Atomically replaces profile       │
│ - Ramp generator picks up smoothly  │
└─────────────────────────────────────┘
```

**Key insight:** MotorBase provides ~90% of IMotor implementation shared by all motor types. The motion math (unit conversion, profile calculation) is in MotorBase, not individual motor classes. This keeps the IPulseGenerator simple and focused on pulse generation, while MotorBase handles trajectory planning and coordinate systems

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
 *
 * @note Direction reversal is handled at MotorBase level, not here.
 *       See Section 5.3.5.1 "Direction Reversal Responsibility".
 */
enum class RampState : uint8_t {
    IDLE,           ///< No motion
    ACCELERATING,   ///< Ramping up to target velocity
    CRUISING,       ///< At constant target velocity
    DECELERATING    ///< Ramping down to stop
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
 * @brief RMT pulse generator with callback encoder and software position tracking
 *
 * Implements FastAccelStepper architecture:
 * - Command queue filled by ramp generator task
 * - RMT encoder callback reads from queue (ISR context)
 * - Position tracked in encoder callback (pulse_count_ incremented per command)
 * - IPositionTracker::addPulses() called from ISR for external tracking
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
     */
    RmtPulseGenerator(uint8_t axis_id, gpio_num_t step_pin, gpio_num_t dir_pin,
                      uint8_t rmt_channel);

    ~RmtPulseGenerator() override;

    // IPulseGenerator interface implementation (unchanged)
    esp_err_t init() override;
    esp_err_t startMove(int32_t pulses, float max_velocity,
                        float acceleration) override;  // Now works while running!
    esp_err_t startVelocity(float velocity, float acceleration) override;
    esp_err_t stop(float deceleration) override;
    void stopImmediate() override;
    bool isRunning() const override;
    int64_t getPulseCount() const override;
    float getCurrentVelocity() const override;
    void setCompletionCallback(MotionCompleteCallback cb) override;
    void setPositionTracker(IPositionTracker* tracker) override;

private:
    // ============================================================
    // HARDWARE HANDLES
    // ============================================================

    rmt_channel_handle_t rmt_channel_{nullptr};
    rmt_encoder_handle_t rmt_encoder_{nullptr};

    // ============================================================
    // CONFIGURATION (immutable after construction)
    // ============================================================

    const uint8_t axis_id_;
    const gpio_num_t step_pin_;
    const gpio_num_t dir_pin_;
    const uint8_t rmt_channel_num_;

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
    // POSITION TRACKING (Software - encoder callback based)
    // ============================================================

    std::atomic<int64_t> pulse_count_{0};        ///< Total pulses (updated in encoder callback)
    std::atomic<bool> direction_{true};          ///< Current direction

    // ============================================================
    // STATE FLAGS
    // ============================================================

    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> rmt_stopped_{true};
    std::atomic<bool> completion_pending_{false};
    bool last_chunk_had_steps_{false};           ///< For direction pause insertion (ISR only)

    // ============================================================
    // RAMP TASK (Event-driven, one per axis)
    // ============================================================

    TaskHandle_t ramp_task_handle_{nullptr};
    SemaphoreHandle_t ramp_semaphore_{nullptr};  ///< Wake-up signal for queue refill

    // ============================================================
    // CALLBACKS
    // ============================================================

    MotionCompleteCallback completion_callback_{nullptr};
    IPositionTracker* position_tracker_{nullptr};  ///< Called from encoder ISR

    // ============================================================
    // PRIVATE METHODS
    // ============================================================

    // Initialization
    esp_err_t initRmt();
    esp_err_t createEncoder();
    void createRampTask();

    // Queue operations
    bool isQueueFull() const;
    bool isQueueEmpty() const;
    uint8_t queueSpace() const;
    uint32_t ticksInQueue() const;
    bool pushCommand(const StepCommand& cmd);
    void clearQueue();

    // Ramp generation (task context)
    static void rampTaskFunc(void* arg);
    void fillQueue();
    StepCommand generateNextCommand();
    void recalculateRamp();
    uint32_t calculateTicks(uint32_t ramp_steps) const;

    // ISR helpers
    void IRAM_ATTR wakeRampTask();

    // Motion control
    void startMotion();
    void startRmtTransmission();
    void stopMotion();

    // Static callbacks (ISR context)
    static size_t IRAM_ATTR encodeCallback(
        const void* data, size_t data_size,
        size_t symbols_written, size_t symbols_free,
        rmt_symbol_word_t* symbols, bool* done, void* arg);

    static bool IRAM_ATTR onTransmitDone(
        rmt_channel_handle_t channel,
        const rmt_tx_done_event_data_t* event_data, void* user_ctx);

    // Symbol generation (ISR context - no FPU)
    size_t IRAM_ATTR fillSymbols(rmt_symbol_word_t* symbols,
                                  StepCommand* cmd);  ///< Returns steps generated
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

    // Position tracking: using encoder callback (pulse_count_ atomic)
    // No PCNT initialization needed - ESP32-S3 only has 4 PCNT units

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
```

#### 5.3.2 Encoder Callback and Symbol Generation (ISR Context)

> **ISR Safety Requirements:**
> - All code in this section runs in ISR context
> - NO floating-point operations (FPU not available)
> - NO blocking calls, NO mutex locks
> - NO memory allocation
> - Use only atomic operations for shared state
> - All functions must have `IRAM_ATTR` attribute

##### RMT Symbol Format

```
RMT Symbol (32-bit word) = Two 16-bit entries sent sequentially

Each 16-bit entry:
┌─────┬───────────────────────────┐
│ Bit │ Meaning                   │
├─────┼───────────────────────────┤
│  15 │ Output level (1=HIGH)     │
│ 0-14│ Duration in ticks (0-32767)│
└─────┴───────────────────────────┘

Step Pulse Symbol (50% duty cycle):
  High half: duration1 | 0x8000  (level=1)
  Low half:  duration0           (level=0)
  Combined:  (duration0 << 16) | (duration1 | 0x8000)

Pause Symbol (output stays low):
  0x00020002 = 2 ticks low, 2 ticks low (minimal idle)
```

##### Encoder Callback Implementation

```cpp
/**
 * @brief RMT encoder callback - generates step pulses on demand (ISR context)
 *
 * Called by RMT driver when it needs more symbols. Always fills exactly
 * LIMIT_RMT_PART_SIZE (24) symbols per call. Handles:
 * - Direction change with setup time pause
 * - Partial command consumption (steps > PART_SIZE)
 * - Position tracking via atomic increment
 *
 * @note ISR CONTEXT - No FPU, no blocking, no allocation!
 */
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
        // Queue empty - signal completion and stop
        self->rmt_stopped_.store(true, std::memory_order_release);
        self->completion_pending_.store(true, std::memory_order_release);
        *done = true;
        return 0;
    }

    // Get current command (may be partially consumed)
    StepCommand* cmd = &self->queue_[rp & LIMIT_RMT_QUEUE_LEN_MASK];

    // Fill symbols using the command
    size_t steps_generated = self->fillSymbols(symbols, cmd);

    // ============================================================
    // POSITION TRACKING (ISR context - atomic operations only)
    // ============================================================
    // Position updated when steps are QUEUED for output (not when actually output)
    // Systematic offset: up to PART_SIZE (24) steps ahead of actual position
    // At 1000 steps/mm: offset = 0.024mm (acceptable for most applications)
    if (steps_generated > 0) {
        bool dir = self->direction_.load(std::memory_order_relaxed);
        int64_t delta = dir ? static_cast<int64_t>(steps_generated)
                            : -static_cast<int64_t>(steps_generated);
        self->pulse_count_.fetch_add(delta, std::memory_order_relaxed);

        // Notify IPositionTracker if set
        // CRITICAL: IPositionTracker::addPulses() MUST be ISR-safe:
        // - Use atomic operations only (std::atomic)
        // - No mutex locks, no blocking calls
        // - No memory allocation
        // - No FPU operations
        if (self->position_tracker_) {
            self->position_tracker_->addPulses(delta);
        }
    }

    // Wake ramp task if queue running low (< 8 entries)
    uint8_t queue_depth = (wp - rp) & LIMIT_RMT_QUEUE_LEN_MASK;
    if (queue_depth < 8) {
        self->wakeRampTask();
    }

    return LIMIT_RMT_PART_SIZE;
}

/**
 * @brief Fill RMT symbols from command (ISR context - no FPU!)
 *
 * Generates exactly LIMIT_RMT_PART_SIZE symbols per call.
 * Handles partial command consumption for large step counts.
 *
 * @param symbols Output buffer for RMT symbols
 * @param cmd Command to process (may be modified for partial consumption)
 * @return Number of steps actually generated (for position tracking)
 */
size_t IRAM_ATTR RmtPulseGenerator::fillSymbols(
    rmt_symbol_word_t* symbols, StepCommand* cmd)
{
    // ============================================================
    // DIRECTION CHANGE HANDLING
    // ============================================================
    // If direction toggle requested AND we just output steps,
    // insert pause symbols for direction setup time
    if (cmd->flags & CMD_FLAG_TOGGLE_DIR) {
        if (last_chunk_had_steps_) {
            // Insert pause for direction setup time
            // Pause duration = LIMIT_RMT_DIR_SETUP_US converted to ticks
            // At 16MHz: 20µs = 320 ticks, spread across PART_SIZE symbols
            uint16_t pause_ticks_per_symbol =
                (LIMIT_RMT_DIR_SETUP_US * (LIMIT_RMT_RESOLUTION_HZ / 1000000) +
                 LIMIT_RMT_PART_SIZE - 1) / LIMIT_RMT_PART_SIZE;

            for (uint8_t i = 0; i < LIMIT_RMT_PART_SIZE; i++) {
                // Pause symbol: both halves low, equal duration
                symbols[i].duration0 = pause_ticks_per_symbol;
                symbols[i].level0 = 0;
                symbols[i].duration1 = pause_ticks_per_symbol;
                symbols[i].level1 = 0;
            }

            last_chunk_had_steps_ = false;
            // Don't toggle yet - will toggle on next callback
            return 0;  // No steps generated
        }

        // Toggle direction pin (ISR-safe GPIO operation)
        bool new_dir = !direction_.load(std::memory_order_relaxed);
        gpio_set_level(dir_pin_, new_dir ? 1 : 0);
        direction_.store(new_dir, std::memory_order_relaxed);

        // Clear toggle flag
        cmd->flags &= ~CMD_FLAG_TOGGLE_DIR;
    }

    uint8_t steps = cmd->steps;
    uint16_t ticks = cmd->ticks;

    // ============================================================
    // PAUSE COMMAND (steps == 0)
    // ============================================================
    if (steps == 0) {
        last_chunk_had_steps_ = false;

        // Fill with idle symbols, distribute ticks across PART_SIZE
        uint16_t ticks_per_symbol = ticks / LIMIT_RMT_PART_SIZE;
        uint16_t remainder = ticks % LIMIT_RMT_PART_SIZE;

        for (uint8_t i = 0; i < LIMIT_RMT_PART_SIZE; i++) {
            uint16_t sym_ticks = ticks_per_symbol + (i < remainder ? 1 : 0);
            uint16_t half = sym_ticks >> 1;
            // Pause: both halves low
            symbols[i].duration0 = half;
            symbols[i].level0 = 0;
            symbols[i].duration1 = sym_ticks - half;
            symbols[i].level1 = 0;
        }

        // Advance to next command (pause fully consumed)
        read_idx_.store(read_idx_.load(std::memory_order_relaxed) + 1,
                        std::memory_order_release);
        return 0;  // No steps generated
    }

    // ============================================================
    // STEP GENERATION
    // ============================================================
    last_chunk_had_steps_ = true;

    // Determine how many steps to generate this callback
    uint8_t steps_to_do = steps;
    if (steps_to_do > LIMIT_RMT_PART_SIZE) {
        steps_to_do = LIMIT_RMT_PART_SIZE;
    }

    // Calculate 50% duty cycle timing (integer math only!)
    uint16_t half_ticks_high = ticks >> 1;
    uint16_t half_ticks_low = ticks - half_ticks_high;

    if (steps_to_do < LIMIT_RMT_PART_SIZE) {
        // Need to stretch: fill unused symbols with idle first
        uint8_t idle_count = LIMIT_RMT_PART_SIZE - steps_to_do;

        // Generate idle symbols first (maintain timing)
        for (uint8_t i = 0; i < idle_count; i++) {
            symbols[i].duration0 = 2;  // Minimal idle
            symbols[i].level0 = 0;
            symbols[i].duration1 = 2;
            symbols[i].level1 = 0;
        }

        // Then generate step pulses
        for (uint8_t i = 0; i < steps_to_do; i++) {
            // Step pulse: HIGH for half_ticks_high, LOW for half_ticks_low
            symbols[idle_count + i].duration0 = half_ticks_low;
            symbols[idle_count + i].level0 = 0;      // LOW first
            symbols[idle_count + i].duration1 = half_ticks_high;
            symbols[idle_count + i].level1 = 1;      // HIGH second (step edge)
        }
    } else {
        // All PART_SIZE symbols are step pulses
        for (uint8_t i = 0; i < LIMIT_RMT_PART_SIZE; i++) {
            symbols[i].duration0 = half_ticks_low;
            symbols[i].level0 = 0;
            symbols[i].duration1 = half_ticks_high;
            symbols[i].level1 = 1;
        }
    }

    // Update command or advance to next
    uint8_t remaining = steps - steps_to_do;
    if (remaining == 0) {
        // Command fully consumed - advance read index
        read_idx_.store(read_idx_.load(std::memory_order_relaxed) + 1,
                        std::memory_order_release);
    } else {
        // Partial consumption - update remaining steps in place
        cmd->steps = remaining;
    }

    return steps_to_do;  // Return actual steps generated for position tracking
}
```

#### 5.3.3 Position Tracking (Software - Encoder Callback)

```cpp
/**
 * @brief Get current pulse count
 *
 * Position is tracked in the encoder callback by incrementing pulse_count_
 * atomically for each command processed. This provides bounded-latency
 * position updates (see latency table in Section 2.2).
 *
 * @return Total pulses since reset (signed for direction)
 */
int64_t RmtPulseGenerator::getPulseCount() const {
    return pulse_count_.load(std::memory_order_acquire);
}

/**
 * @brief Get current instantaneous velocity
 *
 * Velocity is tracked in the ramp generator based on current_ticks_.
 *
 * @return Current velocity in pulses/second
 */
float RmtPulseGenerator::getCurrentVelocity() const {
    if (!running_) {
        return 0.0f;
    }
    return current_velocity_;
}

/**
 * @brief Reset position counter
 *
 * Only valid when not running.
 *
 * @param position New position value (default 0)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if running
 */
esp_err_t RmtPulseGenerator::resetPosition(int64_t position) {
    if (running_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Reset software counter
    pulse_count_.store(position, std::memory_order_release);

    // Also notify position tracker if set
    if (position_tracker_) {
        position_tracker_->reset(position);
    }

    return ESP_OK;
}
```

**Position Update Latency Analysis:**

The position is updated in the encoder callback when each command is processed.
The latency depends on how many steps are batched per command:

| Frequency | Steps/Cmd | Callback Rate | Max Position Lag |
|-----------|-----------|---------------|------------------|
| 200 kHz | 32 | 6,250/s | 0.16 ms |
| 50 kHz | 32 | 1,562/s | 0.64 ms |
| 10 kHz | 32 | 312/s | 3.2 ms |
| 1 kHz | 10 | 100/s | 10 ms |
| 100 Hz | 1 | 100/s | 10 ms |

**All frequencies meet the PRD requirement of <10ms position query latency.**

#### 5.3.4 Event-Driven Ramp Task (Per-Axis)

Each axis has its own ramp generator task that fills the command queue. The task is **event-driven** using a semaphore, not polled.

##### Task Configuration

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Stack Size | 4096 bytes | FPU context save + local variables + margin |
| Priority | `configTIMER_TASK_PRIORITY - 1` (5) | Below timers, above application tasks |
| Core | Core 1 | Motion core per architecture.md |
| Task per axis | Yes (4 tasks for X,Z,A,B) | Simpler queue ownership, no mutex needed |

##### Task Creation

```cpp
/**
 * @brief Create ramp generator task for this axis
 *
 * Called from init(). Creates:
 * - Binary semaphore for wake-up signaling
 * - FreeRTOS task pinned to Core 1 (motion core)
 */
void RmtPulseGenerator::createRampTask() {
    // Create binary semaphore for wake-up
    ramp_semaphore_ = xSemaphoreCreateBinary();
    configASSERT(ramp_semaphore_ != nullptr);

    // Task name includes axis letter for debugging
    char task_name[16];
    snprintf(task_name, sizeof(task_name), "ramp_%c",
             "XZAB"[rmt_channel_num_]);  // Channel 0-3 = X,Z,A,B

    // Create task pinned to Core 1 (motion core)
    BaseType_t result = xTaskCreatePinnedToCore(
        rampTaskFunc,                           // Task function
        task_name,                              // Name for debugging
        4096,                                   // Stack size (bytes)
        this,                                   // Parameter (this pointer)
        configTIMER_TASK_PRIORITY - 1,          // Priority (5)
        &ramp_task_handle_,                     // Handle output
        1                                       // Core 1 (motion core)
    );
    configASSERT(result == pdPASS);
}
```

##### Task Function

```cpp
/**
 * @brief Ramp generator task - one per axis (TASK context - FPU allowed)
 *
 * Wakes up when:
 * 1. startMove()/startVelocity() posts semaphore (new motion)
 * 2. Encoder callback posts semaphore (queue running low)
 * 3. 4ms timeout (fallback for queue refill)
 *
 * @note TASK CONTEXT - FPU operations allowed (sqrtf, floating-point)
 */
void RmtPulseGenerator::rampTaskFunc(void* arg) {
    RmtPulseGenerator* self = static_cast<RmtPulseGenerator*>(arg);

    while (true) {
        // Wait for wake-up signal or timeout
        if (xSemaphoreTake(self->ramp_semaphore_, pdMS_TO_TICKS(LIMIT_RMT_RAMP_TASK_PERIOD_MS))) {
            // Semaphore taken - new motion or queue low
        }
        // Either way, check if queue needs filling

        if (self->ramp_state_ != RampState::IDLE) {
            self->fillQueue();  // FPU operations OK here
        }

        // Handle completion notification (task context, not ISR)
        if (self->completion_pending_.exchange(false)) {
            self->running_ = false;
            self->ramp_state_ = RampState::IDLE;
            if (self->completion_callback_) {
                self->completion_callback_(self->pulse_count_.load(), true);
            }
        }
    }
}

/**
 * @brief Wake ramp task from encoder callback (ISR context)
 *
 * Called when queue is running low (< 8 entries remaining).
 * Uses xSemaphoreGiveFromISR for ISR safety.
 */
void IRAM_ATTR RmtPulseGenerator::wakeRampTask() {
    BaseType_t higher_priority_woken = pdFALSE;
    xSemaphoreGiveFromISR(ramp_semaphore_, &higher_priority_woken);
    if (higher_priority_woken) {
        portYIELD_FROM_ISR();
    }
}
```

##### Execution Context Summary

| Function | Context | FPU | Blocking | Notes |
|----------|---------|-----|----------|-------|
| `createRampTask()` | Task | Yes | Yes | Called from init() |
| `rampTaskFunc()` | Task | Yes | Yes | Main ramp loop |
| `fillQueue()` | Task | Yes | No | Generates commands |
| `generateNextCommand()` | Task | Yes | No | sqrtf() for velocity |
| `recalculateRamp()` | Task | Yes | No | Profile recalculation |
| `wakeRampTask()` | ISR | No | No | Just semaphore give |
| `encodeCallback()` | ISR | No | No | Symbol generation |
| `fillSymbols()` | ISR | No | No | Integer math only |

#### 5.3.5 startMove() with Mid-Motion Blending

**Critical:** `startMove()` now works while motion is in progress. This enables mid-motion blending at the motor level (see Section 4.2).

##### Mid-Motion Blend Strategy

When `startMove()` is called during active motion:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      QUEUE DRAIN + BLEND STRATEGY                        │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  Time ──────────────────────────────────────────────────────────────►   │
│                                                                          │
│  Queue State:   [cmd1][cmd2][cmd3][cmd4]  ←── Already queued            │
│                        ↑                                                 │
│                        │ startMove() called here                         │
│                        │                                                 │
│  What happens:                                                           │
│  1. Existing queue entries CONTINUE to be consumed (no clearing)        │
│  2. Ramp generator captures current_velocity_ from last generated cmd   │
│  3. New profile calculated: current velocity → new target               │
│  4. NEW commands appended to queue (blend occurs naturally)             │
│  5. Smooth velocity transition guaranteed                                │
│                                                                          │
│  Velocity Profile:                                                       │
│       ▲                                                                  │
│       │      ╱╲                                                          │
│       │     ╱  ╲    ╱────  ← New target velocity                        │
│       │    ╱    ╲  ╱                                                     │
│       │   ╱      ╲╱  ← Blend point (smooth transition)                  │
│       │  ╱                                                               │
│       └──────────────────────────────────►                              │
│           Old motion    │    Blended motion                              │
│                      startMove()                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**Key Design Decisions:**
- **Queue is NOT cleared** on mid-motion change - ensures smooth velocity continuity
- **Trade-off:** Smoother motion vs higher blend latency (up to 10ms with current planning time)
- **Direction reversal:** Handled by MotorBase layer (see Section 5.3.5.1)

##### Implementation

```cpp
esp_err_t RmtPulseGenerator::startMove(int32_t pulses, float max_velocity,
                                       float acceleration) {
    // Validate parameters
    if (acceleration <= 0 || max_velocity <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Atomic profile replacement - works whether running or not

    // 1. Capture current state for smooth blending
    float current_vel = current_velocity_;  // Task context - safe to read
    int64_t current_pos = pulse_count_.load(std::memory_order_acquire);

    // 2. Calculate new profile from current state
    bool new_direction = (pulses >= 0);
    int32_t abs_pulses = (pulses >= 0) ? pulses : -pulses;

    // 3. Update parameters atomically
    params_.target_position.store(abs_pulses, std::memory_order_relaxed);
    params_.target_velocity.store(max_velocity, std::memory_order_relaxed);
    params_.acceleration.store(acceleration, std::memory_order_relaxed);
    params_.position_mode.store(true, std::memory_order_relaxed);

    // 4. Handle direction change - DELEGATE TO MOTORBASE
    // RmtPulseGenerator does NOT handle direction reversal internally.
    // MotorBase is responsible for:
    //   a) Detecting direction change (current vs new)
    //   b) Calling stop() to decelerate to zero
    //   c) Waiting for stop completion
    //   d) Calling startMove() with new direction
    // This keeps RmtPulseGenerator simple and focused on pulse generation.
    direction_.store(new_direction, std::memory_order_relaxed);

    // 5. Determine ramp state based on current velocity
    if (running_) {
        // Already running - blend from current velocity
        // ramp_state_ remains as-is; fillQueue() will recalculate profile
    } else {
        // Starting fresh
        ramp_state_ = RampState::ACCELERATING;
        ramp_steps_up_ = 0;
    }

    params_.params_changed.store(true, std::memory_order_release);

    // 6. Start motion if not already running
    if (!running_) {
        running_ = true;
        rmt_stopped_ = false;
        startRmtTransmission();
    }

    // 7. Wake ramp task to process new profile immediately
    xSemaphoreGive(ramp_semaphore_);

    return ESP_OK;
}
```

##### 5.3.5.1 Direction Reversal Responsibility

**Decision:** Direction reversal is handled at **MotorBase level**, NOT in RmtPulseGenerator.

**Rationale:**
- Keeps IPulseGenerator interface simple
- MotorBase already handles unit conversion and coordinate systems
- Cleaner separation of concerns
- Easier testing (mock pulse generator doesn't need reversal logic)

**MotorBase Reversal Flow:**
```cpp
// In MotorBase::moveAbsolute() - simplified
esp_err_t MotorBase::moveAbsolute(float position, float velocity) {
    int32_t pulses = positionToPulses(position);
    bool new_direction = (pulses >= 0);
    bool current_direction = (pulse_gen_->getCurrentVelocity() >= 0);

    if (pulse_gen_->isRunning() && new_direction != current_direction) {
        // Direction reversal required
        // 1. Stop current motion (decelerate to zero)
        pulse_gen_->stop(config_.max_deceleration);
        // 2. Wait for stop (via completion callback or polling)
        // 3. Then call startMove with reversed direction
    }
    // Normal case: same direction or starting fresh
    return pulse_gen_->startMove(pulses, velocityToFrequency(velocity),
                                  accelerationToPulses(config_.max_acceleration));
}
```

**Note:** Direction reversal is orchestrated by MotorBase, keeping RmtPulseGenerator focused on pulse generation.

---

### 5.4 Latency Analysis

This section documents the two distinct latency types in the system.

#### 5.4.1 Position Reporting Latency

**Definition:** How stale is `getPulseCount()` relative to actual motor position?

**Bounded by:** Steps-per-command (`calculateStepsForLatency()`)

| Frequency | Steps/Command | Position Update Latency |
|-----------|---------------|-------------------------|
| 500 kHz | 50 | 0.1 ms |
| 200 kHz | 32 | 0.16 ms |
| 50 kHz | 32 | 0.64 ms |
| 10 kHz | 32 | 3.2 ms |
| 1 kHz | 10 | 10 ms |
| 100 Hz | 1 | 10 ms |

**Additional offset:** Position is updated when steps are GENERATED (queued to RMT hardware buffer), not when steps are actually OUTPUT to GPIO. This creates a systematic offset of up to PART_SIZE (24) steps ahead of actual motor position.

- At 1000 steps/mm: offset = 0.024mm (negligible)
- At 100 steps/mm: offset = 0.24mm (may matter for high-precision)

**PRD Mapping:** This latency applies to "Position queryable <10ms" requirement. ✓ Met at all frequencies.

#### 5.4.2 Blend Response Latency

**Definition:** How long until a new `startMove()` target affects motor motion?

**Bounded by:** Forward planning time (queue depth)

```
startMove() called
       │
       ▼
┌─────────────────────────────────────────────────────┐
│ Existing queue drains:  ~0-10ms (planning time)    │
│ New profile calculated: <1ms                        │
│ New commands queued:    immediate                   │
└─────────────────────────────────────────────────────┘
       │
       ▼
New velocity profile takes effect: 0-10ms worst case
```

**With 10ms forward planning:**
- Best case: 0ms (queue nearly empty)
- Typical: 5ms (half-full queue)
- Worst case: 10ms (queue full)

**Trade-off:** Shorter planning = faster blend response, but higher risk of queue underrun (jerky motion if CPU can't keep up).

**PRD Mapping:** This latency affects "Command-to-motion latency under 10ms" for mid-motion changes. ✓ Met with 10ms planning time.

#### 5.4.3 Command Response Latency (Initial Motion)

**Definition:** Time from `startMove()` call to first step pulse (when starting from IDLE).

**Components:**
1. Parameter update: <1µs (atomic stores)
2. Semaphore give: <1µs
3. Task wake + context switch: <100µs
4. Queue fill (first command): <100µs
5. RMT transmission start: <10µs

**Total:** <1ms for initial motion start

**PRD Mapping:** "Command response latency must not exceed 10ms" ✓ Easily met.

---

#### 5.4.4 Ramp Generator Queue Fill

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
        // RMT axes (FastAccelStepper pattern + encoder callback position tracking)
        case AXIS_X:
            return std::make_unique<RmtPulseGenerator>(
                AXIS_X,
                GPIO_STEP_X, GPIO_DIR_X,
                RMT_CHANNEL_X);

        case AXIS_Z:
            return std::make_unique<RmtPulseGenerator>(
                AXIS_Z,
                GPIO_STEP_Z, GPIO_DIR_Z,
                RMT_CHANNEL_Z);

        case AXIS_A:
            return std::make_unique<RmtPulseGenerator>(
                AXIS_A,
                GPIO_STEP_A, GPIO_DIR_A,
                RMT_CHANNEL_A);

        case AXIS_B:
            return std::make_unique<RmtPulseGenerator>(
                AXIS_B,
                GPIO_STEP_B, GPIO_DIR_B,
                RMT_CHANNEL_B);

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

### Phase 3: Encoder Callback Position Tracking
**Files:** `rmt_pulse_gen.cpp`
- Position tracked via `pulse_count_` atomic in encoder callback (ISR context)
- `getPulseCount()` returns `pulse_count_.load()` directly
- `resetPosition()` sets `pulse_count_.store(position)` when not running
- No PCNT hardware needed (ESP32-S3 only has 4 units, 3 already used)
- Bounded latency: <10ms at all frequencies (see Section 2.2 table)

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

// Encoder callback position tracking accuracy
TEST(RmtPulseGenerator, EncoderPositionAccuracy) {
    auto gen = factory.create(AXIS_X);
    gen->init();

    // Move exact count
    gen->startMove(10000, 100000, 500000);
    waitForCompletion(gen.get());

    // Encoder callback tracking should match exactly
    EXPECT_EQ(gen->getPulseCount(), 10000);
}

// Position tracking latency (must be <10ms at all frequencies)
TEST(RmtPulseGenerator, PositionTrackingLatency) {
    auto gen = factory.create(AXIS_X);
    gen->init();

    // Start at slow speed (worst case for latency)
    gen->startVelocity(1000, 10000);  // 1 kHz

    // Position should update within 10ms
    vTaskDelay(pdMS_TO_TICKS(15));
    int64_t pos1 = gen->getPulseCount();

    vTaskDelay(pdMS_TO_TICKS(15));
    int64_t pos2 = gen->getPulseCount();

    // Should have updated (not stuck at 0)
    EXPECT_GT(pos2, pos1);

    gen->stopImmediate();
}

// startMove() while running (atomic profile replacement)
TEST(RmtPulseGenerator, StartMoveWhileRunning) {
    auto gen = factory.create(AXIS_X);
    gen->init();

    // Start move to 10000
    gen->startMove(10000, 100000, 500000);
    vTaskDelay(pdMS_TO_TICKS(30));  // Partial completion

    // Call startMove() again with new target (ALLOWED - atomic replacement)
    EXPECT_EQ(gen->startMove(20000, 100000, 500000), ESP_OK);
    waitForCompletion(gen.get());

    // Should reach new target (relative from current position)
    // Note: Exact count depends on blending behavior
    EXPECT_GT(gen->getPulseCount(), 15000);
}

// Blending test (motor level orchestration)
TEST(RmtPulseGenerator, BlendingFromMotorLevel) {
    // This test demonstrates the blending pattern:
    // ServoMotor calculates blend trajectory, calls startMove()
    auto gen = factory.create(AXIS_X);
    gen->init();

    // Initial move
    gen->startMove(10000, 100000, 500000);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Simulate motor-level blend: get current state
    int64_t current_pos = gen->getPulseCount();
    float current_vel = gen->getCurrentVelocity();

    // Motor calculates relative move to new target
    int32_t new_relative_pulses = 5000;  // Motor-calculated blend

    // startMove() with relative pulses (works while running)
    EXPECT_EQ(gen->startMove(new_relative_pulses, 100000, 500000), ESP_OK);

    // Verify still running and will complete
    EXPECT_TRUE(gen->isRunning());
}
```

---

## 9. Verification Checklist

### 9.1 Functional Requirements

| Requirement | Test | Status |
|-------------|------|--------|
| Position queryable <10ms latency | Encoder callback tracking | |
| 1000 Hz motion update rate | Ramp task at 4ms | |
| No jitter >1ms | Hardware pulse generation | |
| 8 simultaneous axes | 4 RMT + 2 MCPWM + 1 LEDC + E | |
| Mid-motion blending | startMove() while running | |
| Smooth transitions | Atomic profile replacement | |
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
| LIMIT_RMT_MAX_STEPS_PER_CMD | 255 | |
| LIMIT_RMT_FORWARD_PLANNING_TICKS | 160,000 (10ms) | |
| LIMIT_RMT_RAMP_TASK_PERIOD_MS | 4 | |

**Note:** RMT axes (X, Z, A, B) use encoder callback position tracking.
No additional PCNT units required (ESP32-S3 only has 4 units, 3 already used).

---

## 10. ESP-IDF 5.x Specific Notes

### 10.1 RMT API Changes (IDF 4 → IDF 5)

ESP-IDF 5.x introduced a new RMT driver API. Key differences:

| IDF 4.x | IDF 5.x | Notes |
|---------|---------|-------|
| `rmt_driver_install()` | `rmt_new_tx_channel()` | Channel creation |
| `rmt_write_items()` | `rmt_transmit()` | Transmission |
| `rmt_set_tx_thr_intr_en()` | Callback encoder | Threshold interrupt |
| Manual buffer fill | `rmt_new_simple_encoder()` | On-demand callback |

### 10.2 Simple Encoder Pattern (ESP-IDF 5.x)

The key to FastAccelStepper pattern on ESP-IDF 5.x is `rmt_new_simple_encoder()`:

```cpp
#include "driver/rmt_tx.h"

// Encoder callback signature
size_t IRAM_ATTR encode_callback(
    const void* data, size_t data_size,
    size_t symbols_written, size_t symbols_free,
    rmt_symbol_word_t* symbols, bool* done, void* arg);

// Create encoder with callback
rmt_simple_encoder_config_t enc_config = {
    .callback = encode_callback,
    .arg = this,                           // User context
    .min_chunk_size = LIMIT_RMT_PART_SIZE  // 24 symbols minimum per call
};
rmt_encoder_handle_t encoder;
rmt_new_simple_encoder(&enc_config, &encoder);
```

**Callback behavior:**
- Called on-demand when RMT needs more symbols
- Must return exactly `min_chunk_size` symbols (or 0 if done)
- Set `*done = true` when transmission should end
- Runs in ISR context - no FPU, no blocking calls

### 10.3 RMT Channel Configuration (No DMA)

```cpp
rmt_tx_channel_config_t config = {
    .gpio_num = step_pin,
    .clk_src = RMT_CLK_SRC_DEFAULT,       // 80 MHz APB clock
    .resolution_hz = 16000000,             // 16 MHz (62.5ns per tick)
    .mem_block_symbols = 48,               // ESP32-S3: SOC_RMT_MEM_WORDS_PER_CHANNEL
    .trans_queue_depth = 1,
    .intr_priority = 0,                    // Default priority
    .flags = {
        .invert_out = 0,
        .with_dma = 0,                     // CRITICAL: No DMA
        .io_loop_back = 0,
        .io_od_mode = 0
    }
};
```

**Key settings:**
- `with_dma = 0` → All 4 RMT TX channels available (DMA consumes extra channel slots)
- `trans_queue_depth = 1` → Simple queue (FastAccelStepper pattern doesn't need more)
- `resolution_hz = 16000000` → 16 MHz for FastAccelStepper compatibility

### 10.4 References for ESP-IDF 5.x RMT

- [ESP-IDF 5.x RMT Driver](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-reference/peripherals/rmt.html)
- [ESP-IDF 5.x Migration Guide](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/migration-guides/release-5.x/5.0/peripherals.html#rmt-driver)
- FastAccelStepper IDF5 Implementation: `/examples/FastAccelStepper/src/StepperISR_idf5_esp32_rmt.cpp`

---

## 11. References

- FastAccelStepper Library (local copy): `/examples/FastAccelStepper/`
- ESP-IDF RMT Driver: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/rmt.html
- ESP-IDF PCNT Driver: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/pcnt.html
- ESP32-S3 Technical Reference Manual (RMT, PCNT chapters)
- YaRobot PRD: `/docs/prd.md`
- YaRobot Architecture: `/docs/architecture.md`
- Motor Implementation: `/firmware/components/motor/include/motor_base.h`

---

## 12. Implementation Gaps & Resolution Status

> **Review Date:** 2025-12-06
> **Reviewer:** Architecture Review (Winston + Sergey)
> **Status:** ✅ ALL GAPS RESOLVED - Ready for Implementation Review

This section documents gaps identified during architecture review. All gaps have been addressed in the document sections referenced below.

---

### 12.1 Code Consistency: rmt_fill_buffer Adaptation ✅ RESOLVED

**Gap:** Section 5.3.2 showed mixed type systems (YaRobot vs FastAccelStepper types).

**Resolution:** Unified implementation in Section 5.3.2 using only YaRobot types:
- `encodeCallback()` uses `StepCommand`, `queue_[]`
- `fillSymbols()` fully implemented with YaRobot constants
- FastAccelStepper reference code removed

**Verification:** See Section 5.3.2 "Encoder Callback and Symbol Generation (ISR Context)"

---

### 12.2 Missing Implementation: fillSymbols() ✅ RESOLVED

**Gap:** `fillSymbols()` was declared but not implemented.

**Resolution:** Complete implementation added in Section 5.3.2 including:
- RMT symbol format diagram (bit layout documentation)
- 50% duty cycle calculation with integer math
- Partial command consumption for steps > PART_SIZE
- Pause symbol generation

**Verification:** See Section 5.3.2 "##### RMT Symbol Format" and `fillSymbols()` implementation

---

### 12.3 Missing Implementation: Direction Pause Insertion ✅ RESOLVED

**Gap:** Direction setup time pause insertion was not shown.

**Resolution:** Direction handling added to `fillSymbols()` in Section 5.3.2:
- Pause ticks calculation: `LIMIT_RMT_DIR_SETUP_US * (LIMIT_RMT_RESOLUTION_HZ / 1000000)`
- `last_chunk_had_steps_` flag tracks if pause needed
- Two-phase direction change (pause first, toggle on next callback)

**Verification:** See Section 5.3.2 "DIRECTION CHANGE HANDLING" block in `fillSymbols()`

---

### 12.4 Queue Behavior on Mid-Motion Blend ✅ RESOLVED

**Gap:** Section 5.3.5 `startMove()` says "atomically replaces the profile" but doesn't specify what happens to commands already in the queue.

**Decision:** Let existing queue entries drain naturally; ramp generator calculates blend trajectory from current state.

**Resolution:** Full documentation added in Section 5.3.5 "Mid-Motion Blend Strategy" diagram including:
- Queue drain behavior with visual diagram
- Velocity profile illustration showing smooth blend
- Trade-off documentation (smoother motion vs blend latency)

**Verification:** See Section 5.3.5 "##### Mid-Motion Blend Strategy"

---

### 12.5 Forward Planning Time Reduction ✅ RESOLVED

**Gap:** Current `LIMIT_RMT_FORWARD_PLANNING_TICKS` = 320,000 ticks = 20ms. With "let drain" strategy, worst-case blend latency is ~20ms, exceeding PRD's <10ms requirement.

**Decision:** Reduce forward planning to ~10ms to meet latency requirement.

**Resolution:** Section 3.1 updated with:
```c
#define LIMIT_RMT_FORWARD_PLANNING_TICKS (LIMIT_RMT_RESOLUTION_HZ / 100)  // 10ms
```

**Verification:** See Section 3.1 `config_limits.h` line defining `LIMIT_RMT_FORWARD_PLANNING_TICKS`

---

### 12.6 FPU Context Clarification ✅ RESOLVED

**Gap:** Ramp generator uses `sqrtf()` for velocity calculation. Document doesn't clarify which functions run in task context (FPU allowed) vs ISR context (FPU forbidden on ESP32).

**Decision:** Add explicit context annotations.

**Resolution:** Section 5.3.4 "##### Execution Context Summary" table added with full context/FPU mapping for all functions. All ISR functions marked with `IRAM_ATTR`.

**Verification:** See Section 5.3.4 "##### Execution Context Summary"

---

### 12.7 Ramp Task Configuration ✅ RESOLVED

**Gap:** Document shows `rampTaskFunc()` but doesn't specify task creation parameters (stack size, priority, core affinity).

**Decision:** One task per axis (4 separate tasks for 4 RMT axes).

**Resolution:** Section 5.3.4 "##### Task Configuration" and "##### Task Creation" added with:
- Task parameters table (stack, priority, core)
- `createRampTask()` implementation with semaphore creation
- Task naming per axis

**Verification:** See Section 5.3.4 "#### 5.3.4 Event-Driven Ramp Task (Per-Axis)"

---

### 12.8 IPositionTracker ISR Safety Requirement ✅ RESOLVED

**Gap:** `IPositionTracker::addPulses()` is called from encoder callback (ISR context). Document doesn't specify ISR safety requirements.

**Decision:** Add explicit ISR safety requirement.

**Resolution:** Section 5.3.2 encoder callback includes ISR safety comment block:
```cpp
// CRITICAL: IPositionTracker::addPulses() MUST be ISR-safe:
// - Use atomic operations only (std::atomic)
// - No mutex locks, no blocking calls
// - No memory allocation
// - No FPU operations
```

**Verification:** See Section 5.3.2 position tracking code block (lines 943-951)

---

### 12.9 Dual Latency Documentation ✅ RESOLVED

**Gap:** Document conflates two different latencies:
1. **Position reporting latency** - how stale is `getPulseCount()` (bounded by steps-per-command)
2. **Blend response latency** - how long until new target affects motion (bounded by queue depth)

**Decision:** Document both latencies separately.

**Resolution:** Section 5.4 "### 5.4 Latency Analysis" added with three distinct subsections:
- 5.4.1 Position Reporting Latency (with table)
- 5.4.2 Blend Response Latency (with diagram)
- 5.4.3 Command Response Latency (initial motion)

**Verification:** See Section 5.4

---

### 12.10 REVERSING State Implementation ✅ RESOLVED

**Gap:** `RampState::REVERSING` mentioned in Section 5.1 but implementation in Section 5.3.6 is incomplete ("... similar to DECELERATING with direction change").

**Decision:** Direction reversal is handled at MotorBase level (option b).

**Resolution:**
- Section 5.3.5.1 documents that MotorBase handles direction reversal
- `RampState::REVERSING` removed from enum (unused)
- MotorBase reversal flow documented with code example

**Verification:** See Section 5.3.5.1 "Direction Reversal Responsibility"

---

### 12.11 Position Tracking Systematic Offset ✅ RESOLVED

**Gap:** Position is updated when steps are generated in encoder callback, not when steps are OUTPUT to GPIO. This creates systematic offset of up to PART_SIZE steps.

**Decision:** Accept ~24 step offset as negligible at typical resolutions.

**Resolution:** Offset documented in:
- Section 5.3.2 position tracking comment block (lines 934-936)
- Section 5.4.1 "Additional offset" note with calculations

**Verification:** See Section 5.4.1 "Position Reporting Latency"

---

### Review Checklist

All gaps have been addressed and verified:

- [x] 12.1 rmt_fill_buffer uses YaRobot types
- [x] 12.2 fillSymbols() fully implemented
- [x] 12.3 Direction pause insertion documented
- [x] 12.4 Queue blend behavior documented
- [x] 12.5 Forward planning reduced to 10ms
- [x] 12.6 FPU context table added
- [x] 12.7 Task creation code added
- [x] 12.8 ISR safety requirement documented
- [x] 12.9 Dual latency analysis added
- [x] 12.10 REVERSING removed from enum; MotorBase handles reversal
- [x] 12.11 Position offset documented

**Status:** ✅ ALL GAPS RESOLVED - Document ready for implementation
