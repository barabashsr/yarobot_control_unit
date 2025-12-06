# RMT Pulse Generator Refactor: DMA Streaming → Encoder-Based Architecture

**Document Type:** Architecture Change Proposal
**Status:** Draft for Discussion
**Date:** 2025-12-06
**Affected Component:** `RmtPulseGenerator` (firmware/components/pulse_gen/)

---

## 1. Problem Statement

### 1.1 Current Issue

The current `RmtPulseGenerator` implementation uses **DMA streaming** with large buffers (`LIMIT_RMT_BUFFER_SYMBOLS = 512`). This causes:

1. **Channel Exhaustion**: ESP32-S3 has `SOC_RMT_MEM_WORDS_PER_CHANNEL = 48`. Requesting 512 symbols with DMA consumes ~11 memory blocks, exhausting all 4 TX channel slots with a single channel.

2. **Error**: `rmt_tx_register_to_group(167): no free tx channels` when creating second RMT channel.

3. **Workaround Limitation**: Reducing buffer to 48 symbols works but increases ISR overhead significantly (~20x more callbacks at high frequencies).

### 1.2 Root Cause

The architecture was designed for unlimited motion length via DMA streaming, but this conflicts with ESP32-S3 hardware constraints. The Espressif official example uses a fundamentally different approach.

---

## 2. Proposed Solution: Encoder-Based Architecture

Adopt the **Espressif official stepper motor example** pattern:

| Aspect | Current (DMA Streaming) | Proposed (Encoder-Based) |
|--------|------------------------|--------------------------|
| `mem_block_symbols` | 512 (causes issues) | 64 (safe) |
| `with_dma` | `true` | `false` |
| Profile generation | Real-time in ISR/task | Pre-computed lookup tables |
| Uniform speed | ISR buffer refill | Hardware `loop_count` |
| Refill task | Required (high priority) | **Eliminated** |
| CPU overhead | Higher (ISR callbacks) | Lower (hardware handles) |
| Max channels | 1 (with 512 buffer) | **4** (all axes) |

---

## 3. Architecture Comparison

### 3.1 Current Architecture (DMA Streaming)

```
┌─────────────────────────────────────────────────────────────┐
│                 CURRENT: DMA STREAMING                       │
└─────────────────────────────────────────────────────────────┘

  startMove(1000, 50kHz, 100k)
          │
          ▼
  ┌───────────────────┐
  │ Calculate Profile │  ← Real-time trapezoidal calculation
  │ (FPU operations)  │
  └────────┬──────────┘
           │
           ▼
  ┌───────────────────┐     ┌───────────────────┐
  │ Prime Buffer A    │────▶│ Prime Buffer B    │
  │ (512 symbols)     │     │ (512 symbols)     │
  └────────┬──────────┘     └────────┬──────────┘
           │                         │
           ▼                         ▼
  ┌────────────────────────────────────────────┐
  │          RMT DMA TX (streaming)             │
  │  ┌────────────┐    ┌────────────┐          │
  │  │ Buffer A   │───▶│ Buffer B   │───▶ ...  │
  │  │ (TX)       │    │ (queued)   │          │
  │  └────────────┘    └────────────┘          │
  └──────────────────────┬─────────────────────┘
                         │
                         ▼ TX-Done ISR
  ┌───────────────────────────────────────────┐
  │ ISR: Update counters, swap buffers        │
  │      Signal refill task                    │
  └──────────────────────┬────────────────────┘
                         │
                         ▼
  ┌───────────────────────────────────────────┐
  │ Refill Task (Core 1, high priority)       │
  │ - fillBuffer() with FPU calculations      │
  │ - rmt_transmit() next buffer              │
  │ - Runs every ~1ms at 500kHz               │
  └───────────────────────────────────────────┘

  Problems:
  - 512 symbols × with_dma = exhausts all TX channels
  - High CPU overhead from refill task
  - Complex ISR/task synchronization
```

### 3.2 Proposed Architecture (Encoder-Based)

```
┌─────────────────────────────────────────────────────────────┐
│               PROPOSED: ENCODER-BASED                        │
└─────────────────────────────────────────────────────────────┘

  startMove(1000, 50kHz, 100k)
          │
          ▼
  ┌───────────────────────────────────────────┐
  │ Calculate Profile & Build Curve Table     │
  │ (one-time, before motion starts)          │
  │                                           │
  │ Accel: 50 sample points (smoothstep)      │
  │ Decel: 50 sample points (smoothstep)      │
  │ Uniform: single frequency value           │
  └────────┬──────────────────────────────────┘
           │
           ▼
  ┌───────────────────────────────────────────┐
  │        Queue 3 Transmissions              │
  │                                           │
  │  1. Accel Encoder (loop_count=0)          │
  │     → 50 symbols, varying frequency       │
  │                                           │
  │  2. Uniform Encoder (loop_count=N)        │
  │     → 1 symbol, hardware loops N times    │
  │                                           │
  │  3. Decel Encoder (loop_count=0)          │
  │     → 50 symbols, varying frequency       │
  └────────┬──────────────────────────────────┘
           │
           ▼
  ┌───────────────────────────────────────────┐
  │     RMT Hardware Executes Autonomously    │
  │                                           │
  │  [Accel] → [Uniform×N] → [Decel] → Done   │
  │                                           │
  │  No ISR callbacks during motion!          │
  │  (only final TX-Done for completion)      │
  └────────┬──────────────────────────────────┘
           │
           ▼ Single TX-Done ISR at end
  ┌───────────────────────────────────────────┐
  │ ISR: Fire completion callback             │
  │      Update final pulse count             │
  └───────────────────────────────────────────┘

  Benefits:
  - mem_block_symbols=64, with_dma=false
  - All 4 TX channels available
  - Near-zero CPU during motion
  - No refill task needed
  - Simpler, more robust
```

---

## 4. Interface Preservation (CRITICAL)

### 4.1 IPulseGenerator Interface - NO CHANGES

```cpp
// ALL SIGNATURES PRESERVED EXACTLY
class IPulseGenerator {
public:
    virtual esp_err_t init() = 0;
    virtual esp_err_t startMove(int32_t pulses, float max_velocity, float acceleration) = 0;
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

### 4.2 Behavioral Contract - PRESERVED

| Behavior | Current | Proposed | Status |
|----------|---------|----------|--------|
| `startMove()` executes trapezoidal profile | ✓ | ✓ | Same |
| `startVelocity()` runs until `stop()` | ✓ | ✓ | Same |
| `stop()` decelerates and fires callback | ✓ | ✓ | Same |
| `stopImmediate()` stops without callback | ✓ | ✓ | Same |
| `getPulseCount()` returns total pulses | ✓ | ✓ | Same |
| Position tracker updated during motion | Per buffer (~1ms) | Per phase (~ms) | **Slightly different timing** |
| Completion callback fires on normal end | ✓ | ✓ | Same |

### 4.3 Position Tracking - Minor Timing Change

**Current**: `addPulses()` called every ~1ms (each DMA buffer completion)
**Proposed**: `addPulses()` called at phase transitions (accel→cruise→decel→done)

**Impact**: Position updates less frequently but still accurate at completion.
**Mitigation**: Can add intermediate callbacks if needed (see Section 6.2).

---

## 5. Implementation Plan

### 5.1 New Files to Create

```
firmware/components/pulse_gen/
├── stepper_motor_encoder.h      # Curve + Uniform encoder headers
├── stepper_motor_encoder.cpp    # Encoder implementations (from Espressif example)
└── rmt_pulse_gen.cpp            # Modified (see changes below)
```

### 5.2 Changes to RmtPulseGenerator

#### 5.2.1 Remove (DMA-specific code)

```cpp
// DELETE these members:
- rmt_symbol_word_t buffer_a_[LIMIT_RMT_BUFFER_SYMBOLS];
- rmt_symbol_word_t buffer_b_[LIMIT_RMT_BUFFER_SYMBOLS];
- std::atomic<bool> use_buffer_a_;
- std::atomic<size_t> symbols_in_current_buffer_;
- std::atomic<size_t> symbols_in_next_buffer_;
- TaskHandle_t refill_task_handle_;
- std::atomic<bool> refill_pending_;
- std::atomic<bool> task_should_exit_;

// DELETE these methods:
- size_t fillBuffer(rmt_symbol_word_t*, size_t);
- void primeBuffers();
- static void refillTaskEntry(void*);
- void refillTaskLoop();
- void handleRefillRequest();
```

#### 5.2.2 Add (Encoder-specific code)

```cpp
// ADD these members:
+ rmt_encoder_handle_t accel_encoder_;
+ rmt_encoder_handle_t uniform_encoder_;
+ rmt_encoder_handle_t decel_encoder_;
+ uint32_t accel_sample_points_;      // Curve resolution
+ uint32_t decel_sample_points_;

// ADD/MODIFY these methods:
+ esp_err_t createEncoders();
+ esp_err_t queueMotionPhases();
+ void updatePositionTrackerForPhase(MotionPhase phase, int64_t pulses);
```

#### 5.2.3 Modify init()

```cpp
esp_err_t RmtPulseGenerator::init()
{
    // CHANGE: RMT configuration
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = gpio_num_,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,          // 1 MHz (not 80 MHz)
        .mem_block_symbols = 64,           // CHANGED from 512
        .trans_queue_depth = 10,           // CHANGED from 4
        // .flags.with_dma = false         // REMOVED (default is false)
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &channel_handle_));

    // CHANGE: Create encoders instead of copy encoder
    ESP_ERROR_CHECK(createEncoders());

    // CHANGE: Register callback (still needed for completion)
    rmt_tx_event_callbacks_t cbs = { .on_trans_done = onTxDone };
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(channel_handle_, &cbs, this));

    ESP_ERROR_CHECK(rmt_enable(channel_handle_));

    // REMOVED: No refill task creation

    initialized_ = true;
    return ESP_OK;
}
```

#### 5.2.4 Modify startMove()

```cpp
esp_err_t RmtPulseGenerator::startMove(int32_t pulses, float max_velocity, float acceleration)
{
    direction_ = (pulses > 0);
    int32_t abs_pulses = abs(pulses);

    // Calculate profile (same as before)
    calculateTrapezoidalProfile(abs_pulses, max_velocity, acceleration);

    // Reset state
    pulse_count_.store(0);
    state_.store(ProfileState::ACCELERATING);

    // Notify position tracker
    if (position_tracker_) {
        position_tracker_->setDirection(direction_);
    }

    // NEW: Reconfigure encoders for this motion
    updateAccelEncoder(profile_.accel_pulses,
                       LIMIT_MIN_PULSE_FREQ_HZ,
                       profile_.cruise_velocity);

    updateDecelEncoder(profile_.decel_pulses,
                       profile_.cruise_velocity,
                       LIMIT_MIN_PULSE_FREQ_HZ);

    // NEW: Queue all three phases
    rmt_transmit_config_t tx_config = {};

    // Phase 1: Acceleration (loop_count = 0, encoder generates N symbols)
    tx_config.loop_count = 0;
    uint32_t accel_samples = profile_.accel_pulses;
    ESP_ERROR_CHECK(rmt_transmit(channel_handle_, accel_encoder_,
                                  &accel_samples, sizeof(accel_samples), &tx_config));

    // Phase 2: Uniform speed (loop_count = cruise_pulses)
    if (profile_.cruise_pulses > 0) {
        tx_config.loop_count = profile_.cruise_pulses;
        uint32_t uniform_freq = (uint32_t)profile_.cruise_velocity;
        ESP_ERROR_CHECK(rmt_transmit(channel_handle_, uniform_encoder_,
                                      &uniform_freq, sizeof(uniform_freq), &tx_config));
    }

    // Phase 3: Deceleration (loop_count = 0)
    tx_config.loop_count = 0;
    uint32_t decel_samples = profile_.decel_pulses;
    ESP_ERROR_CHECK(rmt_transmit(channel_handle_, decel_encoder_,
                                  &decel_samples, sizeof(decel_samples), &tx_config));

    return ESP_OK;
}
```

#### 5.2.5 Modify startVelocity()

```cpp
esp_err_t RmtPulseGenerator::startVelocity(float velocity, float acceleration)
{
    direction_ = (velocity >= 0);
    float abs_velocity = fabs(velocity);

    // Calculate accel phase
    int32_t accel_pulses = (int32_t)((abs_velocity * abs_velocity) / (2.0f * acceleration));

    // Reset state
    pulse_count_.store(0);
    state_.store(ProfileState::ACCELERATING);
    mode_ = MotionMode::VELOCITY;
    target_velocity_ = abs_velocity;

    if (position_tracker_) {
        position_tracker_->setDirection(direction_);
    }

    // Update accel encoder
    updateAccelEncoder(accel_pulses, LIMIT_MIN_PULSE_FREQ_HZ, abs_velocity);

    // Queue acceleration phase
    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    uint32_t accel_samples = accel_pulses;
    ESP_ERROR_CHECK(rmt_transmit(channel_handle_, accel_encoder_,
                                  &accel_samples, sizeof(accel_samples), &tx_config));

    // Queue infinite uniform phase (loop_count = -1)
    tx_config.loop_count = -1;  // Infinite loop
    uint32_t uniform_freq = (uint32_t)abs_velocity;
    ESP_ERROR_CHECK(rmt_transmit(channel_handle_, uniform_encoder_,
                                  &uniform_freq, sizeof(uniform_freq), &tx_config));

    return ESP_OK;
}
```

#### 5.2.6 Modify stop()

```cpp
esp_err_t RmtPulseGenerator::stop(float deceleration)
{
    if (state_.load() == ProfileState::IDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    // Calculate decel distance from current velocity
    float current_vel = current_velocity_.load();
    int32_t decel_pulses = (int32_t)((current_vel * current_vel) / (2.0f * deceleration));

    // Disable current transmission
    rmt_disable(channel_handle_);
    rmt_enable(channel_handle_);

    // Update decel encoder
    updateDecelEncoder(decel_pulses, current_vel, LIMIT_MIN_PULSE_FREQ_HZ);

    // Queue deceleration phase
    state_.store(ProfileState::DECELERATING);
    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    uint32_t decel_samples = decel_pulses;
    ESP_ERROR_CHECK(rmt_transmit(channel_handle_, decel_encoder_,
                                  &decel_samples, sizeof(decel_samples), &tx_config));

    return ESP_OK;
}
```

#### 5.2.7 Simplify ISR Callback

```cpp
bool IRAM_ATTR RmtPulseGenerator::onTxDone(
    rmt_channel_handle_t channel,
    const rmt_tx_done_event_data_t* event_data,
    void* user_data)
{
    RmtPulseGenerator* self = static_cast<RmtPulseGenerator*>(user_data);

    // Update pulse count based on completed phase
    // (event_data->num_symbols tells us how many were sent)
    self->pulse_count_.fetch_add(event_data->num_symbols, std::memory_order_relaxed);

    // Update position tracker
    if (self->position_tracker_) {
        self->position_tracker_->addPulses(event_data->num_symbols);
    }

    // Check if all phases complete (no more queued transmissions)
    // This happens when the last phase (decel) finishes
    if (/* last transmission check */) {
        self->state_.store(ProfileState::IDLE, std::memory_order_release);
        self->current_velocity_.store(0.0f, std::memory_order_relaxed);

        // Signal completion (can't call callback from ISR)
        // Use a simpler mechanism than task notification
        self->completion_pending_.store(true, std::memory_order_release);
    }

    return false;
}
```

---

## 6. Considerations & Trade-offs

### 6.1 Advantages of Encoder Approach

| Advantage | Impact |
|-----------|--------|
| All 4 RMT channels available | Can run X, Z, A, B simultaneously |
| No refill task | Simpler code, less CPU overhead |
| Hardware handles timing | More precise pulse timing |
| Pre-computed curves | No FPU in ISR context |
| Lower memory usage | 64 symbols vs 512×2 |

### 6.2 Disadvantages / Limitations

| Limitation | Mitigation |
|------------|------------|
| Position updates less frequent | Accept for RMT axes (final count accurate) |
| Curve resolution limited by sample points | Use 100+ samples for smooth motion |
| Can't change velocity mid-motion | stopImmediate() + new startMove() |
| Encoder reconfiguration overhead | Only at motion start, acceptable |

### 6.3 Velocity Mode Complexity

The `startVelocity()` with infinite loop (`loop_count = -1`) works but `stop()` requires:
1. Disable RMT channel (interrupts infinite loop)
2. Queue deceleration phase
3. Wait for decel to complete

This is slightly more complex but achievable.

---

## 7. Testing Strategy

### 7.1 Unit Tests (Preserve Existing)

```cpp
// All existing tests should pass unchanged:
TEST(RmtPulseGenerator, InitializesCorrectly);
TEST(RmtPulseGenerator, StartMoveGeneratesCorrectPulseCount);
TEST(RmtPulseGenerator, StartVelocityAcceleratesToTarget);
TEST(RmtPulseGenerator, StopDeceleratesCorrectly);
TEST(RmtPulseGenerator, StopImmediateStopsWithoutCallback);
TEST(RmtPulseGenerator, CompletionCallbackFires);
TEST(RmtPulseGenerator, PositionTrackerUpdated);
```

### 7.2 New Tests

```cpp
// New tests for encoder-based behavior:
TEST(RmtPulseGenerator, FourChannelsSimultaneous);  // Critical!
TEST(RmtPulseGenerator, CurveEncoderSmoothness);
TEST(RmtPulseGenerator, UniformEncoderAccuracy);
TEST(RmtPulseGenerator, VelocityModeInfiniteLoop);
TEST(RmtPulseGenerator, StopDuringVelocityMode);
```

### 7.3 Hardware Tests

```
PULSE X 50000        # Start X at 50kHz
PULSE Z 50000        # Start Z at 50kHz (should work now!)
PULSE A 50000        # Start A at 50kHz
PULSE B 50000        # Start B at 50kHz
# All 4 axes running simultaneously!

MOVE X 10000 50000 100000  # Position mode
# Verify exact pulse count
```

---

## 8. Implementation Phases

### Phase 1: Create Stepper Motor Encoders
- Copy `stepper_motor_encoder.c/h` from Espressif example
- Adapt to project coding style
- Add to CMakeLists.txt
- **Estimate**: 2 hours

### Phase 2: Refactor RmtPulseGenerator::init()
- Change RMT configuration (mem_block_symbols=64, no DMA)
- Create curve + uniform encoders
- Remove refill task creation
- **Estimate**: 2 hours

### Phase 3: Refactor startMove()
- Replace buffer priming with encoder queuing
- Calculate and configure curve encoders
- Queue accel → uniform → decel phases
- **Estimate**: 3 hours

### Phase 4: Refactor startVelocity() and stop()
- Implement infinite loop for velocity mode
- Implement controlled stop with decel encoder
- **Estimate**: 3 hours

### Phase 5: Simplify ISR and Completion
- Remove buffer swap logic
- Simplify pulse counting
- Update position tracker integration
- **Estimate**: 2 hours

### Phase 6: Testing
- Run existing tests
- Add new multi-channel tests
- Hardware verification
- **Estimate**: 4 hours

**Total Estimate**: ~16 hours

---

## 9. Questions for Discussion

1. **Position Tracking Frequency**: Is per-phase update (vs per-buffer) acceptable for RMT axes?

2. **Curve Resolution**: How many sample points for accel/decel curves? (Espressif uses 500)

3. **Velocity Mode**: Should we support mid-motion velocity changes, or is stop+restart acceptable?

4. **Resolution Trade-off**: 1 MHz (Espressif) vs 80 MHz (current). Lower resolution = less precision but simpler math. Is 1 MHz adequate for 500 kHz max output?

5. **Encoder Memory**: Curve encoder stores lookup table. Pre-allocate for worst case, or dynamically size?

---

## 10. Industry Research: Alternative Implementations

### 10.1 FastAccelStepper Library

**Source**: [github.com/gin66/FastAccelStepper](https://github.com/gin66/FastAccelStepper)

**Key Metrics**:
- 200,000 steps/second on ESP32
- ~6,500 Hz ISR rate at max speed (31 steps per batch)
- 32-entry command queue

**Architecture**:
```
┌─────────────────────────────────────────────────────────────┐
│              FASTACCELSTEPPER: COMMAND QUEUE                 │
└─────────────────────────────────────────────────────────────┘

  setSpeedInHz() / moveTo()
          │
          ▼
  ┌───────────────────────────────────────────┐
  │ Ramp Generator (background task, 4ms)     │
  │ - Calculates next commands                │
  │ - Fills 32-entry command queue            │
  │ - 20ms forward planning buffer            │
  └────────┬──────────────────────────────────┘
           │
           ▼
  ┌───────────────────────────────────────────┐
  │ Command Queue (ring buffer)               │
  │ Entry: { ticks, steps (0-255), dir }      │
  │ ~31 steps batched per RMT transmission    │
  └────────┬──────────────────────────────────┘
           │
           ▼
  ┌───────────────────────────────────────────┐
  │ RMT Encoder Callback (ISR context)        │
  │ - Reads commands from queue               │
  │ - Encodes to RMT symbols                  │
  │ - Updates position tracking               │
  └────────┬──────────────────────────────────┘
           │
           ▼
  ┌───────────────────────────────────────────┐
  │ RMT Hardware (no DMA)                     │
  │ mem_block_symbols = 2 × PART_SIZE         │
  │ with_dma = false                          │
  │ trans_queue_depth = 1                     │
  └───────────────────────────────────────────┘
```

**RMT Configuration**:
```c
rmt_tx_channel_config_t tx_config = {
    .mem_block_symbols = 2 * PART_SIZE,  // ~64-96 symbols
    .resolution_hz = 16000000,           // 16 MHz
    .trans_queue_depth = 1,
    .flags.with_dma = false,             // NO DMA!
};
```

**Key Insight**: Uses **callback encoder** that reads from command queue in ISR context, not pre-computed tables. Allows real-time velocity changes by modifying queue entries.

---

### 10.2 FluidNC / Grbl_ESP32

**Source**: [github.com/bdring/FluidNC](https://github.com/bdring/FluidNC)

**Key Metrics**:
- Up to 120,000 steps/second
- 12 motors on 6 axes supported
- Segment-based motion planning

**Architecture**:
```yaml
# FluidNC Configuration
stepping:
  engine: RMT
  segments: 12          # Number of segment buffers
  pulse_us: 4           # 4µs pulse width
  dir_delay_us: 0
```

**Key Insight**: FluidNC uses **segment buffers** (default 12) and plans motion in segments. The RMT peripheral reads pre-planned sequences from segment buffers. Max frequency = 1,000,000 / (pulse_us × 2) = 125 kHz with 4µs pulses.

---

### 10.3 ESP32 LinuxCNC Motion Controller

**Source**: [github.com/wezhunter/ESP32_LinuxCNC_MotionController_RealTime](https://github.com/wezhunter/ESP32_LinuxCNC_MotionController_RealTime)

**Key Metrics**:
- 200 kHz per stepper
- 300-400 kHz total (combined axes)
- 6 axes supported

**Architecture**:
- Uses FastAccelStepper library internally
- Dual-core: Core 0 handles UDP + motor generation, Core 1 spare
- PCNT peripheral unused (available for encoder feedback)

---

### 10.4 Comparison Matrix

| Project | Max Freq | RMT Config | DMA | Velocity Change | Position Update |
|---------|----------|------------|-----|-----------------|-----------------|
| **Espressif Example** | ~500 kHz | mem=64, queue=10 | No | No (pre-computed) | Per phase |
| **FastAccelStepper** | 200 kHz | mem=64, queue=1 | No | Yes (queue) | Per batch (~5ms) |
| **FluidNC** | 125 kHz | segments=12 | No | Yes (segments) | Per segment |
| **Current YaRobot** | 500 kHz | mem=512, queue=4 | Yes | Yes (real-time) | Per buffer (~1ms) |

---

## 11. Proposed Hybrid Approach (Option C)

Based on research and user requirements:
- **Position tracking < 10ms** ✓
- **Mid-motion velocity changes** ✓
- **All 4 RMT channels** ✓
- **No external PCNT required** ✓

### 11.1 Key Insight: On-Demand Callback Architecture

RMT hardware requests symbols **when its buffer needs refilling**, not on a timer.
This is fundamentally different from the current DMA streaming approach.

```
┌─────────────────────────────────────────────────────────────┐
│           ON-DEMAND CALLBACK ENCODER ARCHITECTURE            │
└─────────────────────────────────────────────────────────────┘

  startMove() / startVelocity() / setVelocity()
          │
          ▼
  ┌───────────────────────────────────────────┐
  │ Profile Generator (existing code)         │
  │ - Trapezoidal/S-curve calculation         │
  │ - Per-axis accel/decel from config        │
  │ - Real-time velocity updates supported    │
  └────────┬──────────────────────────────────┘
           │
           ▼
  ┌───────────────────────────────────────────┐
  │ Command Queue (ring buffer, 32 entries)   │
  │ Entry: { ticks_per_step, step_count }     │
  │ - Filled by profile generator task        │
  │ - Read by RMT encoder callback (on-demand)│
  └────────┬──────────────────────────────────┘
           │
           │ RMT requests more symbols when buffer low
           ▼
  ┌───────────────────────────────────────────┐
  │ RMT Callback Encoder (ISR context)        │
  │ - Reads next command from queue           │
  │ - Generates ADAPTIVE steps per call       │
  │ - Tracks position (pulse_count += steps)  │
  │ - No external PCNT needed                 │
  └────────┬──────────────────────────────────┘
           │
           ▼
  ┌───────────────────────────────────────────┐
  │ RMT Hardware (NO DMA)                     │
  │ mem_block_symbols = 48-64                 │
  │ trans_queue_depth = 1                     │
  │ resolution_hz = 1,000,000 (1 MHz)         │
  └───────────────────────────────────────────┘
```

### 11.2 Adaptive Steps Per Command (Critical for Low-Speed Latency)

**Problem:** At slow speeds, fixed step count causes high velocity-change latency.

**Solution:** Vary steps per command based on frequency to bound latency.

```cpp
/**
 * @brief Calculate steps per command for bounded latency
 *
 * Target: max 10ms latency for velocity changes at any frequency
 *
 * @param frequency_hz Current output frequency
 * @return steps per command (1 to MAX_STEPS_PER_COMMAND)
 */
uint16_t calculateStepsPerCommand(uint32_t frequency_hz) {
    // Max 10ms worth of steps
    uint16_t max_steps = frequency_hz / 100;  // freq * 0.01s

    // Clamp to valid range
    if (max_steps < 1) max_steps = 1;
    if (max_steps > LIMIT_RMT_MAX_STEPS_PER_COMMAND) {
        max_steps = LIMIT_RMT_MAX_STEPS_PER_COMMAND;
    }

    return max_steps;
}
```

**Resulting Latency Bounds:**

| Frequency | Steps/Command | Max Velocity Change Latency |
|-----------|---------------|----------------------------|
| 200 kHz   | 32 (clamped)  | 0.16 ms                    |
| 50 kHz    | 32 (clamped)  | 0.64 ms                    |
| 10 kHz    | 32 (clamped)  | 3.2 ms                     |
| 1 kHz     | 10            | **10 ms**                  |
| 100 Hz    | 1             | **10 ms**                  |
| 10 Hz     | 1             | **100 ms** (unavoidable)   |

### 11.3 Position Tracking Without PCNT

Position is tracked **in the encoder callback** by counting pulses as they're generated:

```cpp
// In encoder callback (ISR context)
size_t StepperQueueEncoder::encode(
    const void* data, size_t size,
    rmt_symbol_word_t* symbols, size_t symbols_size)
{
    StepCommand cmd;
    if (!queue_->pop(cmd)) {
        // Queue empty - motion complete or underflow
        return 0;
    }

    // Calculate adaptive step count
    uint16_t steps = calculateStepsPerCommand(cmd.frequency_hz);
    steps = min(steps, cmd.remaining_steps);

    // Generate RMT symbols
    uint32_t period_ticks = resolution_hz_ / cmd.frequency_hz;
    for (int i = 0; i < steps; i++) {
        symbols[i].duration0 = period_ticks / 2;
        symbols[i].level0 = 1;
        symbols[i].duration1 = period_ticks / 2;
        symbols[i].level1 = 0;
    }

    // POSITION TRACKING - this is the source of truth
    pulse_count_.fetch_add(steps, std::memory_order_relaxed);

    return steps;
}
```

**Position Update Frequency:**

| Output Freq | Steps/Callback | Callbacks/sec | Update Interval |
|-------------|----------------|---------------|-----------------|
| 200 kHz     | 32             | 6,250         | 0.16 ms         |
| 50 kHz      | 32             | 1,562         | 0.64 ms         |
| 10 kHz      | 32             | 312           | 3.2 ms          |
| 1 kHz       | 10             | 100           | 10 ms           |
| 100 Hz      | 1              | 100           | 10 ms           |

**All frequencies meet <10ms position update requirement** ✓

### 11.4 RMT Configuration Changes

| Parameter | Current (DMA) | New (Callback) | Reason |
|-----------|---------------|----------------|--------|
| `mem_block_symbols` | 512 | **48** | ESP32-S3 limit per channel |
| `with_dma` | `true` | **`false`** | Allows all 4 channels |
| `trans_queue_depth` | 4 | **1** | On-demand refill |
| `resolution_hz` | 80 MHz | **1 MHz** | Adequate for 500 kHz output |

### 11.5 Command Queue Entry Structure

```cpp
/**
 * @brief Single motion command in the queue
 *
 * Uses ticks (not frequency) for ISR efficiency - no division needed.
 */
struct StepCommand {
    uint32_t ticks_per_step;  // Period in RMT ticks (resolution_hz / freq)
    uint16_t remaining_steps; // Steps remaining in this command
    uint8_t flags;            // Direction, last_command, etc.
};

// Flags
#define CMD_FLAG_DIRECTION   0x01  // 1 = forward, 0 = reverse
#define CMD_FLAG_LAST        0x02  // Last command in motion
#define CMD_FLAG_INVALID     0x80  // Command invalidated (velocity change)
```

### 11.6 Velocity Change Handling

Two approaches depending on mode:

**Velocity Mode (startVelocity):** Direct atomic update, encoder reads immediately:

```cpp
esp_err_t setVelocity(float new_velocity) {
    // Atomic update - next encoder callback uses new value
    uint32_t new_ticks = resolution_hz_ / fabsf(new_velocity);
    target_ticks_.store(new_ticks, std::memory_order_release);

    // Direction change requires decel to zero first
    if ((new_velocity >= 0) != direction_) {
        return ESP_ERR_INVALID_ARG;  // Must stop first
    }
    return ESP_OK;
}

// In encoder callback for velocity mode:
size_t encode_velocity_mode(...) {
    uint32_t ticks = target_ticks_.load(std::memory_order_acquire);
    // Generate symbols at current target frequency
    // No queue needed - always uses latest target
}
```

**Position Mode (startMove):** Recalculate profile, invalidate stale commands:

```cpp
esp_err_t updateTarget(int32_t new_target, float new_velocity) {
    // Increment epoch to invalidate queued commands
    command_epoch_.fetch_add(1, std::memory_order_release);

    // Recalculate remaining profile
    int64_t current_pos = pulse_count_.load();
    recalculateProfile(new_target - current_pos, new_velocity);

    // Queue will be refilled with new commands on next task run
    return ESP_OK;
}

// In encoder callback - skip stale commands:
size_t encode(...) {
    StepCommand cmd;
    while (queue_->peek(cmd)) {
        if (cmd.epoch != command_epoch_.load()) {
            queue_->pop(cmd);  // Discard stale
            continue;
        }
        break;  // Found valid command
    }
    // ... encode valid command
}
```

### 11.7 ROS2 Trajectory Controller Integration

For continuous trajectory following, use velocity mode with position feedback:

```cpp
/**
 * @brief Called by ROS2 trajectory controller at ~100 Hz
 *
 * @param target_pos Desired position at this time
 * @param target_vel Desired velocity at this time
 */
void onTrajectoryPoint(float target_pos, float target_vel) {
    float current_pos = (float)pulse_count_.load() / pulses_per_unit_;
    float pos_error = target_pos - current_pos;

    // Simple P control (or PID for better tracking)
    float Kp = 10.0f;  // Tune based on system dynamics
    float velocity_correction = Kp * pos_error;

    float commanded_vel = target_vel + velocity_correction;
    setVelocity(commanded_vel);
}
```

Position feedback comes from `pulse_count_` tracked in encoder callback.
No external PCNT needed.

---

## 12. Detailed Implementation Strategy (Based on FastAccelStepper)

Reference implementation cloned to: `examples/FastAccelStepper/`

### 12.1 FastAccelStepper Key Constants (ESP32-S3 + IDF 5.x)

From `fas_arch/common_esp32.h` and `fas_arch/common_esp32_idf5.h`:

```c
// Timing
#define TICKS_PER_S       16000000L   // 16 MHz resolution
#define MIN_CMD_TICKS     3200        // TICKS_PER_S / 5000 = minimum period

// Queue
#define QUEUE_LEN         32          // 32 queue entries (ring buffer)

// RMT (ESP32-S3 specific)
#define RMT_SIZE          48          // SOC_RMT_MEM_WORDS_PER_CHANNEL
#define PART_SIZE         24          // RMT_SIZE >> 1 = symbols per callback
```

### 12.2 FastAccelStepper Data Structures

**Queue Entry (from `StepperISR.h`):**
```c
struct queue_entry {
    uint8_t steps;           // 1-255 steps per entry (0 = pause only)
    uint8_t toggle_dir : 1;  // Toggle direction before this entry
    uint8_t countUp : 1;     // Direction flag
    uint8_t moreThanOneStep : 1;
    uint8_t hasSteps : 1;
    uint16_t ticks;          // Period between steps in ticks
};
```

**Queue Management:**
```c
class StepperQueue {
    struct queue_entry entry[QUEUE_LEN];  // 32 entries
    volatile uint8_t read_idx;             // ISR reads from here
    volatile uint8_t next_write_idx;       // Task writes to here
    // ...
};
```

### 12.3 FastAccelStepper RMT Configuration (IDF 5.x)

From `StepperISR_idf5_esp32_rmt.cpp`:

```c
// Encoder setup
rmt_simple_encoder_config_t enc_config = {
    .callback = encode_commands,
    .arg = this,
    .min_chunk_size = PART_SIZE  // 24 symbols
};
rmt_new_simple_encoder(&enc_config, &_tx_encoder);

// Channel setup
rmt_tx_channel_config_t config = {
    .gpio_num = step_pin,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = TICKS_PER_S,        // 16 MHz
    .mem_block_symbols = 2 * PART_SIZE,  // 48 symbols
    .trans_queue_depth = 1,
    .intr_priority = 0,
    .flags.invert_out = 0,
    .flags.with_dma = 0,                 // NO DMA!
};
```

### 12.4 FastAccelStepper Encoder Callback Pattern

From `StepperISR_idf5_esp32_rmt.cpp`:

```c
static size_t IRAM_ATTR encode_commands(
    const void *data, size_t data_size,
    size_t symbols_written, size_t symbols_free,
    rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    StepperQueue *q = (StepperQueue *)arg;
    *done = false;

    // Need at least PART_SIZE symbols available
    if (symbols_free < PART_SIZE) {
        return 0;
    }

    // Check for stop condition
    if (q->_rmtStopped) {
        *done = true;
        return 0;
    }

    // Check queue empty
    uint8_t rp = q->read_idx;
    if (rp == q->next_write_idx) {
        q->_rmtStopped = true;
        // Generate pause symbols
        ENTER_PAUSE(MIN_CMD_TICKS);
        return PART_SIZE;
    }

    // Fill buffer from queue entry
    rmt_fill_buffer(q, true, &symbols[0].val);
    return PART_SIZE;  // Always return fixed size
}
```

### 12.5 Mapping to YaRobot IPulseGenerator

**Adaptation Strategy:** Keep `IPulseGenerator` interface, replace internals.

| FastAccelStepper | YaRobot Equivalent |
|------------------|-------------------|
| `StepperQueue` | `RmtPulseGenerator` internal queue |
| `queue_entry` | `StepCommand` |
| `addQueueEntry()` | Profile generator fills queue |
| `rmt_fill_buffer()` | Custom `encode()` callback |
| `read_idx/next_write_idx` | Atomic ring buffer indices |

**New RmtPulseGenerator Structure:**

```cpp
class RmtPulseGenerator : public IPulseGenerator {
private:
    // === RMT Hardware ===
    rmt_channel_handle_t channel_handle_;
    rmt_encoder_handle_t encoder_handle_;
    gpio_num_t step_pin_;
    gpio_num_t dir_pin_;

    // === Command Queue (FastAccelStepper pattern) ===
    static constexpr size_t QUEUE_LEN = 32;
    struct StepCommand {
        uint16_t ticks;      // Period in RMT ticks (16 MHz)
        uint8_t steps;       // Steps in this command (1-255)
        uint8_t flags;       // Direction, last, etc.
    };
    StepCommand queue_[QUEUE_LEN];
    std::atomic<uint8_t> read_idx_{0};
    std::atomic<uint8_t> write_idx_{0};

    // === State ===
    std::atomic<int64_t> pulse_count_{0};  // Position tracking
    std::atomic<bool> is_running_{false};
    std::atomic<bool> stopped_{true};

    // === Profile Generator (existing) ===
    TrapezoidalProfile profile_;
    std::atomic<float> target_velocity_;
    std::atomic<bool> velocity_changed_;

    // === Encoder Callback (ISR context) ===
    static size_t IRAM_ATTR encodeCallback(
        const void *data, size_t data_size,
        size_t symbols_written, size_t symbols_free,
        rmt_symbol_word_t *symbols, bool *done, void *arg);

    void fillSymbols(rmt_symbol_word_t *symbols, const StepCommand& cmd);

public:
    // IPulseGenerator interface (unchanged)
    esp_err_t init() override;
    esp_err_t startMove(int32_t pulses, float max_velocity, float acceleration) override;
    esp_err_t startVelocity(float velocity, float acceleration) override;
    esp_err_t stop(float deceleration) override;
    void stopImmediate() override;
    bool isRunning() const override;
    int64_t getPulseCount() const override;
    float getCurrentVelocity() const override;
    void setCompletionCallback(MotionCompleteCallback cb) override;

    // New: mid-motion velocity change
    esp_err_t setVelocity(float new_velocity);
};
```

### 12.6 Key Implementation Details

**1. RMT Initialization:**

```cpp
esp_err_t RmtPulseGenerator::init() {
    // Create encoder with callback
    rmt_simple_encoder_config_t enc_cfg = {
        .callback = encodeCallback,
        .arg = this,
        .min_chunk_size = PART_SIZE  // 24
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&enc_cfg, &encoder_handle_));

    // Create TX channel
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = step_pin_,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 16000000,     // 16 MHz
        .mem_block_symbols = 48,       // 2 * PART_SIZE
        .trans_queue_depth = 1,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &channel_handle_));

    // Register completion callback
    rmt_tx_event_callbacks_t cbs = { .on_trans_done = onTransDone };
    rmt_tx_register_event_callbacks(channel_handle_, &cbs, this);

    ESP_ERROR_CHECK(rmt_enable(channel_handle_));
    return ESP_OK;
}
```

**2. Encoder Callback (ISR-safe):**

```cpp
size_t IRAM_ATTR RmtPulseGenerator::encodeCallback(
    const void *data, size_t data_size,
    size_t symbols_written, size_t symbols_free,
    rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    RmtPulseGenerator *self = static_cast<RmtPulseGenerator*>(arg);
    *done = false;

    if (symbols_free < PART_SIZE) {
        return 0;
    }

    if (self->stopped_.load(std::memory_order_acquire)) {
        *done = true;
        return 0;
    }

    uint8_t rp = self->read_idx_.load(std::memory_order_acquire);
    uint8_t wp = self->write_idx_.load(std::memory_order_acquire);

    if (rp == wp) {
        // Queue empty - stop
        self->stopped_.store(true, std::memory_order_release);
        *done = true;
        return 0;
    }

    // Get command and advance read pointer
    const StepCommand& cmd = self->queue_[rp & (QUEUE_LEN - 1)];
    self->read_idx_.store(rp + 1, std::memory_order_release);

    // Fill RMT symbols
    self->fillSymbols(symbols, cmd);

    // POSITION TRACKING - increment pulse count
    self->pulse_count_.fetch_add(cmd.steps, std::memory_order_relaxed);

    return PART_SIZE;
}
```

**3. Symbol Generation (Adaptive):**

```cpp
void IRAM_ATTR RmtPulseGenerator::fillSymbols(
    rmt_symbol_word_t *symbols, const StepCommand& cmd)
{
    uint16_t ticks = cmd.ticks;
    uint8_t steps = cmd.steps;

    if (steps == 0) {
        // Pause command - fill with idle symbols
        uint16_t ticks_per_symbol = ticks / PART_SIZE;
        for (int i = 0; i < PART_SIZE; i++) {
            symbols[i].duration0 = ticks_per_symbol / 2;
            symbols[i].level0 = 0;
            symbols[i].duration1 = ticks_per_symbol / 2;
            symbols[i].level1 = 0;
        }
        return;
    }

    // Adaptive: distribute steps across PART_SIZE symbols
    // Each symbol is one step pulse at the command frequency

    uint16_t half_ticks = ticks / 2;

    if (steps >= PART_SIZE) {
        // High speed: one step per symbol
        for (int i = 0; i < PART_SIZE; i++) {
            symbols[i].duration0 = half_ticks;
            symbols[i].level0 = 1;
            symbols[i].duration1 = half_ticks;
            symbols[i].level1 = 0;
        }
    } else {
        // Low speed: steps < PART_SIZE
        // Spread steps across symbols with padding
        uint8_t gap = PART_SIZE / steps;
        uint8_t step_idx = 0;

        for (int i = 0; i < PART_SIZE; i++) {
            if (step_idx < steps && (i % gap) == 0) {
                // Step pulse
                symbols[i].duration0 = half_ticks;
                symbols[i].level0 = 1;
                symbols[i].duration1 = half_ticks;
                symbols[i].level1 = 0;
                step_idx++;
            } else {
                // Idle padding
                symbols[i].duration0 = half_ticks;
                symbols[i].level0 = 0;
                symbols[i].duration1 = half_ticks;
                symbols[i].level1 = 0;
            }
        }
    }
}
```

**4. Queue Fill (Task Context):**

```cpp
void RmtPulseGenerator::refillQueue() {
    // Check for velocity change
    if (velocity_changed_.exchange(false, std::memory_order_acq_rel)) {
        recalculateProfile(target_velocity_.load());
    }

    // Fill queue while space available and motion not complete
    while (queueSpace() > 0 && !profile_.isComplete()) {
        StepCommand cmd = generateNextCommand();
        pushCommand(cmd);

        if (cmd.flags & CMD_FLAG_LAST) {
            break;
        }
    }
}

uint8_t RmtPulseGenerator::queueSpace() const {
    uint8_t rp = read_idx_.load(std::memory_order_acquire);
    uint8_t wp = write_idx_.load(std::memory_order_acquire);
    return QUEUE_LEN - (wp - rp);
}

bool RmtPulseGenerator::pushCommand(const StepCommand& cmd) {
    uint8_t wp = write_idx_.load(std::memory_order_acquire);
    uint8_t rp = read_idx_.load(std::memory_order_acquire);

    if ((wp - rp) >= QUEUE_LEN) {
        return false;  // Queue full
    }

    queue_[wp & (QUEUE_LEN - 1)] = cmd;
    write_idx_.store(wp + 1, std::memory_order_release);
    return true;
}
```

### 12.7 Configuration Parameters (to add to config_limits.h)

```c
/**
 * @defgroup limits_rmt RMT Pulse Generator Configuration
 * @brief Configuration for RMT-based stepper/servo pulse generation
 * @{
 */

/** @brief RMT clock resolution (Hz) - 16 MHz provides 62.5ns precision */
#define LIMIT_RMT_RESOLUTION_HZ     16000000

/** @brief RMT memory block size (symbols) - ESP32-S3 has 48 per channel */
#define LIMIT_RMT_MEM_BLOCK_SYMBOLS 48

/** @brief Symbols per encoder callback - half of mem_block */
#define LIMIT_RMT_PART_SIZE         24

/** @brief Command queue depth - FastAccelStepper uses 32 */
#define LIMIT_RMT_QUEUE_LEN         32

/** @brief Minimum command period in ticks (16 MHz / 5000 = 3200) */
#define LIMIT_RMT_MIN_CMD_TICKS     3200

/** @brief Maximum step frequency (Hz) - 16 MHz / MIN_CMD_TICKS / 2 */
#define LIMIT_RMT_MAX_STEP_FREQ     200000

/** @} */
```

### 12.8 Resolved Configuration

| Parameter | Value | Source |
|-----------|-------|--------|
| Resolution | 16 MHz | FastAccelStepper default |
| Memory block | 48 symbols | ESP32-S3 SOC limit |
| Part size | 24 symbols | RMT_SIZE / 2 |
| Queue depth | 32 entries | FastAccelStepper proven |
| Max frequency | 200 kHz | 16 MHz / 3200 / 2 |
| Accel limits | Per-axis in config_defaults.h | Already exists |

### 12.9 Implementation Phases

**Phase 1: Core Encoder (from FastAccelStepper)**
- Copy `StepperISR_idf5_esp32_rmt.cpp` patterns
- Implement `rmt_simple_encoder` callback
- Implement ring buffer queue
- Test basic pulse generation

**Phase 2: IPulseGenerator Wrapper**
- Implement `init()`, `startMove()`, `stopImmediate()`
- Connect profile generator to queue
- Add position tracking in callback

**Phase 3: Velocity Mode**
- Implement `startVelocity()`, `stop()`
- Add `setVelocity()` for mid-motion changes
- Test continuous velocity control

**Phase 4: Integration**
- Replace current `RmtPulseGenerator` implementation
- Verify existing tests pass
- Test 4 channels simultaneously

---

## 13. References

- [ESP-IDF Stepper Motor Example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/rmt/stepper_motor)
- [FastAccelStepper Library](https://github.com/gin66/FastAccelStepper) - Command queue architecture
- [FastAccelStepper ESP32 Implementation](https://deepwiki.com/gin66/FastAccelStepper/3.2-esp32-implementation)
- [FluidNC CNC Firmware](https://github.com/bdring/FluidNC) - Segment-based motion
- [ESP32 LinuxCNC Motion Controller](https://github.com/wezhunter/ESP32_LinuxCNC_MotionController_RealTime) - High precision multi-axis
- [ESP32-S3 RMT TRM](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [ESP32 Forum: RMT DMA Channel Issues](https://esp32.com/viewtopic.php?t=42301)

---

## Appendix A: Espressif Stepper Motor Encoder (Reference)

```c
// From esp-idf/examples/peripherals/rmt/stepper_motor/main/stepper_motor_encoder.c

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *copy_encoder;
    uint32_t resolution;
    uint32_t sample_points;
    rmt_symbol_word_t *curve_table;  // Pre-computed frequency curve
} rmt_stepper_curve_encoder_t;

// Smoothstep interpolation for smooth acceleration
static inline float smoothstep(float x) {
    return x * x * (3.0f - 2.0f * x);
}

esp_err_t rmt_new_stepper_motor_curve_encoder(
    const stepper_motor_curve_encoder_config_t *config,
    rmt_encoder_handle_t *ret_encoder)
{
    // Allocate encoder + curve table
    rmt_stepper_curve_encoder_t *encoder = calloc(1, sizeof(...));
    encoder->curve_table = calloc(config->sample_points, sizeof(rmt_symbol_word_t));

    // Pre-compute frequency curve using smoothstep
    for (int i = 0; i < config->sample_points; i++) {
        float progress = (float)i / (float)(config->sample_points - 1);
        float smooth = smoothstep(progress);
        float freq = config->start_freq_hz +
                     (config->end_freq_hz - config->start_freq_hz) * smooth;

        uint32_t period_ticks = config->resolution / freq;
        encoder->curve_table[i].duration0 = period_ticks / 2;
        encoder->curve_table[i].level0 = 1;
        encoder->curve_table[i].duration1 = period_ticks / 2;
        encoder->curve_table[i].level1 = 0;
    }

    *ret_encoder = &encoder->base;
    return ESP_OK;
}
```
