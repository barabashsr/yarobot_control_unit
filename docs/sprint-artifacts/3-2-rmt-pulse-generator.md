# Story 3.2: RMT Pulse Generator

Status: ready-for-dev

## Story

As a **developer**,
I want **RMT-based pulse generation for servo axes X, Z, A, B**,
so that **I can generate precise STEP pulses up to 500 kHz with DMA streaming for high-speed servo motion**.

## Acceptance Criteria

1. **AC1:** Given RMT channels RMT_CHANNEL_X (0), RMT_CHANNEL_Z (1), RMT_CHANNEL_A (2), RMT_CHANNEL_B (3) are configured, when pulse generation is requested, then STEP pulses are generated on GPIO_X_STEP, GPIO_Z_STEP, GPIO_A_STEP, GPIO_B_STEP respectively
2. **AC2:** Given I call `startMove(10000, 200000, 1000000)` (10000 pulses at 200kHz max velocity with 1M pulses/s² accel), when RMT generates pulses, then exactly 10000 pulses are output with ±1% timing accuracy
3. **AC3:** Given pulse generation is active, when I measure the STEP signal, then pulse width is 50% duty cycle (±5%)
4. **AC4:** Given I request frequencies from LIMIT_MIN_PULSE_FREQ_HZ (1 Hz) to LIMIT_MAX_PULSE_FREQ_HZ (500000 Hz), when valid, then pulses are generated at the requested frequency; when invalid, then ESP_ERR_INVALID_ARG is returned
5. **AC5:** Given direction change is needed before motion, when pulse generation starts, then TIMING_DIR_SETUP_US (20µs) delay occurs after DIR signal change before first STEP pulse
6. **AC6:** Given motion completes (pulse count reached), when the last pulse is sent, then MotionCompleteCallback fires with total_pulses count
7. **AC7:** Given I call `startVelocity(100000, 500000)` (100kHz velocity, 500k accel), when RMT generates pulses, then continuous pulses are generated until `stop()` is called
8. **AC8:** Given I call `stop(1000000)` during motion, when deceleration completes, then motion stops gracefully with controlled deceleration profile
9. **AC9:** Given I call `stopImmediate()` during motion, when executed, then pulse generation stops within 10µs with no further pulses
10. **AC10:** Given multiple RMT channels are active simultaneously, when 4 axes run at 200kHz each, then no interference or timing degradation occurs between channels
11. **AC11:** Given `isRunning()` is called during motion, when motion is active, then true is returned; when idle, false is returned
12. **AC12:** Given `getCurrentVelocity()` is called during motion, when motion is active, then current pulse frequency (Hz) is returned as float
13. **AC13:** Given compile-time configuration, when DEFAULT_MAX_PULSE_FREQ_HZ is set to 200000, then this value is used as the per-axis default maximum velocity limit

## Tasks / Subtasks

- [ ] **Task 1: Update pulse frequency limits in config_limits.h** (AC: 4, 13)
  - [ ] Change LIMIT_MAX_PULSE_FREQ_HZ from 100000 to 500000 (hardware capability)
  - [ ] Add DEFAULT_MAX_PULSE_FREQ_HZ = 200000 (default axis max, configurable)
  - [ ] Add LIMIT_RMT_RESOLUTION_HZ = 80000000 (80 MHz RMT clock)
  - [ ] Add static_assert validating DEFAULT <= LIMIT_MAX
  - [ ] Update Doxygen comments explaining the distinction

- [ ] **Task 2: Create pulse_gen component structure** (AC: 1)
  - [ ] Create `firmware/components/pulse_gen/include/i_pulse_generator.h` with IPulseGenerator interface
  - [ ] Create `firmware/components/pulse_gen/include/rmt_pulse_gen.h` with RmtPulseGenerator class declaration
  - [ ] Create `firmware/components/pulse_gen/rmt_pulse_gen.cpp` with implementation
  - [ ] Create `firmware/components/pulse_gen/CMakeLists.txt` with REQUIRES: driver, config, esp_driver_rmt

- [ ] **Task 3: Define IPulseGenerator interface** (AC: 2, 6-9, 11-12)
  - [ ] Define pure virtual methods:
    - `esp_err_t init()`
    - `esp_err_t startMove(int32_t pulses, float max_velocity, float acceleration)`
    - `esp_err_t startVelocity(float velocity, float acceleration)`
    - `esp_err_t stop(float deceleration)`
    - `void stopImmediate()`
    - `bool isRunning() const`
    - `int64_t getPulseCount() const`
    - `float getCurrentVelocity() const`
  - [ ] Define callback type: `using MotionCompleteCallback = std::function<void(int64_t total_pulses)>`
  - [ ] Define `void setCompletionCallback(MotionCompleteCallback cb)`
  - [ ] Add virtual destructor

- [ ] **Task 4: Implement RmtPulseGenerator constructor and init** (AC: 1)
  - [ ] Constructor takes: rmt_channel_id, gpio_num, and optional resolution_hz (default 80MHz)
  - [ ] `init()` creates RMT TX channel via `rmt_new_tx_channel()`
  - [ ] Configure with `rmt_tx_channel_config_t`:
    - `gpio_num` from constructor
    - `clk_src = RMT_CLK_SRC_DEFAULT` (80MHz on ESP32-S3)
    - `resolution_hz = LIMIT_RMT_RESOLUTION_HZ` (80MHz = 12.5ns resolution)
    - `mem_block_symbols = 64` (minimum for DMA)
    - `trans_queue_depth = 4` (for double-buffering)
    - `flags.with_dma = true` (enable DMA for streaming)
  - [ ] Create RMT encoder for pulse generation via `rmt_new_copy_encoder()`
  - [ ] Enable channel via `rmt_enable()`

- [ ] **Task 5: Implement trapezoidal profile generator** (AC: 2, 7, 8)
  - [ ] Create internal profile state machine: IDLE, ACCELERATING, CRUISING, DECELERATING
  - [ ] Implement `calculateProfile(pulses, max_velocity, acceleration)`:
    - Calculate accel distance: `v² / (2 * a)`
    - If total distance < 2 * accel_distance → triangular profile (no cruise)
    - Else → full trapezoidal profile with cruise phase
  - [ ] Store profile parameters: accel_pulses, cruise_pulses, decel_pulses, cruise_velocity
  - [ ] Implement velocity-at-position calculation for streaming buffer fill

- [ ] **Task 6: Implement DMA streaming double-buffer** (AC: 2, 7)
  - [ ] Create symbol buffer (rmt_symbol_word_t array) for DMA
  - [ ] Implement `fillBuffer()` to generate next batch of RMT symbols:
    - Each symbol = one pulse (HIGH for half period, LOW for half period)
    - Symbol duration varies based on current velocity from profile
    - Track position within profile (accel/cruise/decel phase)
  - [ ] Register TX done callback via `rmt_tx_register_event_callbacks()`
  - [ ] On TX done callback → refill buffer with next symbols
  - [ ] Handle end-of-motion: send fewer symbols, mark complete

- [ ] **Task 7: Implement startMove()** (AC: 2, 5, 6)
  - [ ] Validate pulse_count > 0, velocity > 0, acceleration > 0
  - [ ] Validate velocity <= LIMIT_MAX_PULSE_FREQ_HZ
  - [ ] Calculate direction from pulse_count sign
  - [ ] Wait TIMING_DIR_SETUP_US if direction changed (from shift register)
  - [ ] Calculate trapezoidal profile
  - [ ] Prime initial buffer
  - [ ] Start RMT transmission via `rmt_transmit()`
  - [ ] Set state to RUNNING
  - [ ] Return ESP_OK

- [ ] **Task 8: Implement startVelocity()** (AC: 7)
  - [ ] Validate velocity range (can be negative for reverse)
  - [ ] Set target velocity and acceleration
  - [ ] Enter continuous mode (no target pulse count)
  - [ ] Prime buffer and start RMT transmission
  - [ ] Continue generating symbols until stop() called

- [ ] **Task 9: Implement stop() and stopImmediate()** (AC: 8, 9)
  - [ ] `stop(deceleration)`:
    - Calculate decel profile from current velocity
    - Transition to DECELERATING state
    - Complete remaining pulses with decel profile
    - Fire callback on completion
  - [ ] `stopImmediate()`:
    - Call `rmt_disable()` immediately
    - Clear DMA buffers
    - Set state to IDLE
    - Do NOT fire completion callback (aborted)

- [ ] **Task 10: Implement status methods** (AC: 11, 12)
  - [ ] `isRunning()`: return state != IDLE
  - [ ] `getPulseCount()`: return atomic pulse counter (updated in ISR)
  - [ ] `getCurrentVelocity()`: return current frequency from profile state

- [ ] **Task 11: Implement completion callback** (AC: 6)
  - [ ] Store callback via `setCompletionCallback()`
  - [ ] On motion complete (all pulses sent):
    - Call callback with total_pulses from ISR context
    - Use FreeRTOS task notification to defer to non-ISR if needed
  - [ ] On stopImmediate(): do NOT call callback

- [ ] **Task 12: Create unit tests** (AC: 1-13)
  - [ ] Create `firmware/components/pulse_gen/test/test_rmt_pulse_gen.cpp`
  - [ ] Test init() returns ESP_OK for all 4 channels
  - [ ] Test startMove() with various pulse counts and frequencies
  - [ ] Test frequency limits (1 Hz to 500 kHz)
  - [ ] Test invalid parameters return ESP_ERR_INVALID_ARG
  - [ ] Test stop() decelerates correctly
  - [ ] Test stopImmediate() stops within timing requirement
  - [ ] Test completion callback fires with correct pulse count
  - [ ] Test multi-channel simultaneous operation

- [ ] **Task 13: Hardware verification** (AC: 2, 3, 10)
  - [ ] Verify pulse timing at 10kHz, 100kHz, 200kHz, 500kHz with oscilloscope
  - [ ] Verify 50% duty cycle across frequency range
  - [ ] Verify DIR setup timing (20µs before first STEP)
  - [ ] Verify 4-channel simultaneous operation without interference
  - [ ] Measure buffer refill time under load

## Dev Notes

### Architecture Constraints

> **Header-Only Configuration (from Tech Spec)**
>
> Every configurable value MUST be defined in a header file. No magic numbers in source code. Pulse frequency limits from `config_limits.h`, RMT channel assignments from `config_peripherals.h`, GPIO pins from `config_gpio.h`.
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

> **Streaming Double-Buffer Pulse Generation (from Tech Spec)**
>
> All pulse generation uses streaming double-buffer architecture. Short moves, long moves, and continuous jogging use the same infrastructure. No special-case code paths.
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

### Core Concepts

**RMT Peripheral Characteristics:**
- ESP32-S3 RMT has 4 TX channels, each independently timed
- 80 MHz clock → 12.5ns resolution → 500 kHz max at 50% duty
- DMA mode enables unlimited motion length via streaming
- Each RMT symbol encodes one level transition with duration

**Pulse Symbol Calculation:**
- At 80 MHz, one tick = 12.5ns
- For 200 kHz pulse: period = 5µs = 400 ticks
- 50% duty: HIGH for 200 ticks, LOW for 200 ticks
- For 500 kHz pulse: period = 2µs = 160 ticks
- 50% duty: HIGH for 80 ticks, LOW for 80 ticks

**Trapezoidal Profile:**
```
Velocity
   ^
   |     ┌────────────┐
   |    /              \
   |   /                \
   |  /                  \
   | /                    \
   └──────────────────────────> Position
     Accel   Cruise   Decel
```

**Double-Buffer Strategy:**
1. Prime buffer A with initial symbols
2. Start transmission
3. On TX-done interrupt: buffer A transmitted → refill buffer B
4. On next TX-done: buffer B transmitted → refill buffer A
5. Repeat until motion complete

**Frequency Configuration:**
- `LIMIT_MAX_PULSE_FREQ_HZ = 500000` — Hardware capability (absolute max)
- `DEFAULT_MAX_PULSE_FREQ_HZ = 200000` — Default per-axis limit (safe default)
- Per-axis max can be configured between 1 Hz and 500 kHz
- Profile generator respects the configured per-axis max

### Project Structure Notes

**New Component:**
- `firmware/components/pulse_gen/` — Pulse generator implementations
- `firmware/components/pulse_gen/include/i_pulse_generator.h` — Interface
- `firmware/components/pulse_gen/include/rmt_pulse_gen.h` — RMT implementation header
- `firmware/components/pulse_gen/rmt_pulse_gen.cpp` — RMT implementation
- `firmware/components/pulse_gen/CMakeLists.txt` — Build configuration
- `firmware/components/pulse_gen/test/test_rmt_pulse_gen.cpp` — Unit tests

**Config Files to Update:**
- `firmware/components/config/include/config_limits.h` — Add frequency constants

**Existing Files Referenced:**
- `firmware/components/config/include/config_peripherals.h` — RMT_CHANNEL_X/Z/A/B
- `firmware/components/config/include/config_gpio.h` — GPIO_X_STEP, etc.
- `firmware/components/config/include/config_timing.h` — TIMING_DIR_SETUP_US

### Learnings from Previous Story

**From Story 3-1-shift-register-driver (Status: done)**

- **SPI HAL Layer Available**: Use existing `spi_hal` for any SPI needs; pattern established for HAL wrappers
- **config_sr.h Complete**: Shift register bit positions defined; `sr_set_direction()` available for DIR control before motion
- **Thread Safety Pattern**: FreeRTOS mutex pattern established; RMT callbacks should use same approach for thread safety
- **ISR-Safe Functions**: Pattern for ISR-safe functions (no mutex, direct register access) established in `sr_emergency_disable_all()`
- **Build Passes**: No outstanding build issues from Epic 3 foundation

**Files to Reuse:**
- `firmware/components/drivers/tpic6b595/include/tpic6b595.h` — Call `sr_set_direction()` before motion start
- `firmware/components/config/include/config_sr.h` — Bit position macros if needed

[Source: docs/sprint-artifacts/3-1-shift-register-driver.md#Completion-Notes-List]

### References

- [Source: docs/epics.md#Story-3.2] — Story definition and IPulseGenerator interface
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Services-and-Modules] — RmtPulseGenerator module
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#APIs-and-Interfaces] — IPulseGenerator interface
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Performance] — RMT buffer refill timing requirements
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Acceptance-Criteria] — AC2 pulse accuracy
- [ESP-IDF RMT Driver](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-reference/peripherals/rmt.html) — Official RMT documentation

---

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/3-2-rmt-pulse-generator.context.xml

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

### Completion Notes List

### File List

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-04 | SM Agent (Bob) | Initial story draft with 500kHz max, 200kHz default |
