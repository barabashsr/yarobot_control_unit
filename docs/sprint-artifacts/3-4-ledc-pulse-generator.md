# Story 3.4: LEDC Pulse Generator (D Axis)

Status: drafted

## Story

As a **developer**,
I want **LEDC-based pulse generation for D axis stepper with software position tracking**,
so that **the D-axis stepper motor can execute motion commands through the unified IPulseGenerator interface**.

## Acceptance Criteria

1. **AC1:** Given LEDC_TIMER_D and LEDC_CHANNEL_D are configured from `config_peripherals.h`, when pulse generation is requested, then STEP pulses are generated on GPIO_D_STEP (from `config_gpio.h`)
2. **AC2:** Given I call `startMove(2000, 10000, 100000)` (2000 pulses at 10kHz max velocity with 100k pulses/s² accel), when LEDC generates pulses, then exactly 2000 pulses are output with trapezoidal velocity profile
3. **AC3:** Given no hardware PCNT for D axis, when motion runs, then software counter tracks pulse count accurately via high-resolution timer callbacks
4. **AC4:** Given target pulse count is reached, when the last pulse is sent, then LEDC timer stops and MotionCompleteCallback fires with total_pulses count
5. **AC5:** Given direction change is needed before motion, when pulse generation starts, then TIMING_DIR_SETUP_US (20µs) delay occurs after DIR signal change before first STEP pulse
6. **AC6:** Given I call `startVelocity(10000, 100000)` (10kHz velocity, 100k accel), when LEDC generates pulses, then continuous pulses are generated until `stop()` is called
7. **AC7:** Given I call `stop(100000)` during motion, when deceleration completes, then motion stops gracefully with controlled deceleration profile
8. **AC8:** Given I call `stopImmediate()` during motion, when executed, then LEDC stops within LIMIT_STOP_LATENCY_US and completion callback is NOT fired
9. **AC9:** Given `isRunning()` is called during motion, when motion is active, then true is returned; when idle, false is returned
10. **AC10:** Given `getCurrentVelocity()` is called during motion, when motion is active, then current pulse frequency (Hz) is returned as float
11. **AC11:** Given `getPulseCount()` is called, when queried, then software counter returns accurate count matching generated pulses
12. **AC12:** Given pulse frequency range 1 Hz to DEFAULT_MAX_PULSE_FREQ_HZ, when startMove is called with valid frequency, then pulses are generated at requested frequency
13. **AC13 (MANDATORY):** Given any implementation code, when reviewed, then NO hardcoded values, magic numbers, or inline constants exist; ALL configuration values (GPIO pins, timer IDs, channel IDs, timing values, frequency limits, resolution bits, duty cycle) MUST be defined in dedicated header files (`config_gpio.h`, `config_peripherals.h`, `config_timing.h`, `config_limits.h`) and referenced via named constants

## Tasks / Subtasks

- [ ] **Task 1: Add LEDC configuration constants to config headers** (AC: 13)
  - [ ] Add to `config_peripherals.h`:
    - `LEDC_RESOLUTION_BITS` (10 bits = 1024 levels, adequate for D axis)
    - `LEDC_DUTY_CYCLE_PERCENT` (50 for symmetric pulses)
    - `LEDC_CLK_FREQ_HZ` (use APB clock 80MHz or define appropriate source)
  - [ ] Add to `config_limits.h` (if not present):
    - `LIMIT_STOP_LATENCY_US` (100µs max stop time)
    - `LIMIT_LEDC_MAX_FREQ_HZ` (max frequency for LEDC given resolution)
  - [ ] Verify all LEDC-related constants use named definitions, no literals

- [ ] **Task 2: Create LedcPulseGenerator class** (AC: 1, 13)
  - [ ] Create `firmware/components/pulse_gen/include/ledc_pulse_gen.h` with class declaration
  - [ ] Create `firmware/components/pulse_gen/ledc_pulse_gen.cpp` with implementation
  - [ ] Implement IPulseGenerator interface (same as RMT/MCPWM)
  - [ ] Constructor takes: gpio_num (use `GPIO_D_STEP`), timer (use `LEDC_TIMER_D`), channel (use `LEDC_CHANNEL_D`)
  - [ ] Update `firmware/components/pulse_gen/CMakeLists.txt` with REQUIRES: esp_driver_ledc
  - [ ] **CRITICAL**: All values from config headers - NO inline numbers

- [ ] **Task 3: Implement LEDC timer and channel initialization** (AC: 1, 13)
  - [ ] `init()` configures LEDC timer via `ledc_timer_config()`:
    - `speed_mode = LEDC_MODE_D` (from `config_peripherals.h`)
    - `duty_resolution = LEDC_RESOLUTION_BITS` (from `config_peripherals.h`)
    - `timer_num = LEDC_TIMER_D` (from `config_peripherals.h`)
    - `freq_hz` = initial frequency (will be updated per motion)
    - `clk_cfg = LEDC_AUTO_CLK`
  - [ ] Configure LEDC channel via `ledc_channel_config()`:
    - `gpio_num = GPIO_D_STEP` (from `config_gpio.h`)
    - `speed_mode = LEDC_MODE_D`
    - `channel = LEDC_CHANNEL_D`
    - `timer_sel = LEDC_TIMER_D`
    - `duty = (1 << LEDC_RESOLUTION_BITS) * LEDC_DUTY_CYCLE_PERCENT / 100`
  - [ ] Initially paused (duty = 0) until startMove called

- [ ] **Task 4: Implement software pulse counting** (AC: 3, 11)
  - [ ] Create high-resolution timer (esp_timer) for pulse counting
  - [ ] Timer callback increments atomic pulse counter
  - [ ] Timer period = 1 / current_frequency (matches LEDC output)
  - [ ] Use `std::atomic<int64_t>` for thread-safe pulse count
  - [ ] Track direction separately for signed position

- [ ] **Task 5: Implement trapezoidal profile generator** (AC: 2, 6, 7)
  - [ ] Reuse profile calculation logic from RmtPulseGenerator/McpwmPulseGenerator
  - [ ] Create internal profile state machine: IDLE, ACCELERATING, CRUISING, DECELERATING
  - [ ] Profile timer callback updates LEDC frequency via `ledc_set_freq()`
  - [ ] Calculate frequency steps for smooth acceleration/deceleration

- [ ] **Task 6: Implement startMove()** (AC: 2, 5)
  - [ ] Validate pulse_count > 0, velocity > 0, acceleration > 0
  - [ ] Validate velocity <= DEFAULT_MAX_PULSE_FREQ_HZ
  - [ ] Calculate direction from pulse_count sign
  - [ ] Call shift register `sr_set_direction()` for DIR signal
  - [ ] Wait TIMING_DIR_SETUP_US (20µs) if direction changed
  - [ ] Calculate trapezoidal profile (accel, cruise, decel phases)
  - [ ] Reset software pulse counter to 0
  - [ ] Start counting timer
  - [ ] Set LEDC duty to configured value (starts pulse output)
  - [ ] Set state to RUNNING
  - [ ] Return ESP_OK

- [ ] **Task 7: Implement startVelocity()** (AC: 6)
  - [ ] Validate velocity range (can be negative for reverse)
  - [ ] Set target velocity and acceleration
  - [ ] Enter continuous mode (no target pulse count)
  - [ ] Start LEDC output and counting timer
  - [ ] Continue until stop() called
  - [ ] Software counter tracks position throughout

- [ ] **Task 8: Implement stop() and stopImmediate()** (AC: 7, 8)
  - [ ] `stop(deceleration)`:
    - Calculate decel profile from current velocity
    - Transition to DECELERATING state
    - Reduce frequency progressively via profile timer
    - Stop LEDC when velocity reaches zero (duty = 0)
    - Fire callback on completion
  - [ ] `stopImmediate()`:
    - Set LEDC duty to 0 immediately via `ledc_set_duty()` + `ledc_update_duty()`
    - Stop counting timer
    - Read final pulse count
    - Set state to IDLE
    - Do NOT fire completion callback (aborted)
  - [ ] Verify stop latency < LIMIT_STOP_LATENCY_US

- [ ] **Task 9: Implement status methods** (AC: 9, 10, 11)
  - [ ] `isRunning()`: return state != IDLE
  - [ ] `getPulseCount()`: return atomic pulse counter value
  - [ ] `getCurrentVelocity()`: return current LEDC frequency from `ledc_get_freq()`

- [ ] **Task 10: Implement completion callback** (AC: 4)
  - [ ] Store callback via `setCompletionCallback()`
  - [ ] When target pulse count reached (in counting timer callback):
    - Stop LEDC output (duty = 0)
    - Stop counting timer
    - Use FreeRTOS task notification to defer callback from timer ISR to task context
  - [ ] On stopImmediate(): do NOT call callback

- [ ] **Task 11: Create unit tests** (AC: 1-13)
  - [ ] Create `firmware/components/pulse_gen/test/test_ledc_pulse_gen.cpp`
  - [ ] Test init() returns ESP_OK
  - [ ] Test startMove() with various pulse counts and frequencies
  - [ ] Test software counter matches commanded pulses
  - [ ] Test completion callback fires with correct pulse count
  - [ ] Test frequency limits (1 Hz to `DEFAULT_MAX_PULSE_FREQ_HZ`)
  - [ ] Test invalid parameters return ESP_ERR_INVALID_ARG
  - [ ] Test stop() decelerates correctly
  - [ ] Test stopImmediate() stops within timing requirement
  - [ ] Test direction setup delay applied
  - [ ] **VERIFY**: All test values use named constants from config headers (no magic numbers)

- [ ] **Task 12: Hardware verification** (AC: 2, 8, 12)
  - [ ] Verify pulse timing at 1kHz, 10kHz, 50kHz with oscilloscope
  - [ ] Verify 50% duty cycle across frequency range
  - [ ] Verify DIR setup timing (TIMING_DIR_SETUP_US before first STEP)
  - [ ] Verify software counter accuracy vs oscilloscope pulse count
  - [ ] Measure stop latency (< LIMIT_STOP_LATENCY_US)

- [ ] **Task 13: Code Review - No Magic Numbers** (AC: 13)
  - [ ] Review `ledc_pulse_gen.cpp` for any hardcoded numeric values
  - [ ] Verify all GPIO pins referenced via `config_gpio.h` constants
  - [ ] Verify all timer/channel IDs referenced via `config_peripherals.h` constants
  - [ ] Verify all timing values referenced via `config_timing.h` constants
  - [ ] Verify all limits referenced via `config_limits.h` constants
  - [ ] Run grep check: `grep -E "[^A-Z_][0-9]{2,}" ledc_pulse_gen.cpp` should return minimal results

## Dev Notes

### Architecture Constraints

> **Header-Only Configuration (from Tech Spec)**
>
> Every configurable value MUST be defined in a header file. No magic numbers in source code. LEDC timer/channel IDs from `config_peripherals.h`, GPIO pins from `config_gpio.h`, timing values from `config_timing.h`.
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

> **Streaming Double-Buffer Pulse Generation (from Tech Spec)**
>
> All pulse generation uses streaming double-buffer architecture. For LEDC, this means progressive frequency updates during motion rather than pre-computed buffers.
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

### Core Concepts

**LEDC vs RMT/MCPWM:**
- LEDC is the simplest pulse generation peripheral
- No hardware pulse counting (unlike MCPWM+PCNT)
- No DMA streaming (unlike RMT)
- Adequate for D-axis stepper with lower precision requirements
- Uses software timer for pulse counting

**LEDC Configuration:**
- 10-bit resolution = 1024 duty levels
- 50% duty cycle = 512 at 10-bit resolution
- Frequency set via `ledc_set_freq()`
- Duty set via `ledc_set_duty()` + `ledc_update_duty()`

**Software Pulse Counting:**
- High-resolution esp_timer with period = 1/frequency
- Timer callback increments atomic counter
- Less precise than hardware PCNT but adequate for D axis
- Position derived from counter × direction

**D-Axis Characteristics:**
- Single stepper motor (not servo)
- No external encoder feedback
- Position tracked via software counter only
- Used for discrete positioning (not high-speed continuous motion)

### Project Structure Notes

**Files to Create:**
- `firmware/components/pulse_gen/include/ledc_pulse_gen.h` - Class declaration
- `firmware/components/pulse_gen/ledc_pulse_gen.cpp` - Implementation
- `firmware/components/pulse_gen/test/test_ledc_pulse_gen.cpp` - Unit tests

**Existing Files to Update:**
- `firmware/components/pulse_gen/CMakeLists.txt` - Add ledc_pulse_gen.cpp, esp_driver_ledc to REQUIRES
- `firmware/components/config/include/config_peripherals.h` - Add LEDC_RESOLUTION_BITS, LEDC_DUTY_CYCLE_PERCENT
- `firmware/components/config/include/config_limits.h` - Add LIMIT_STOP_LATENCY_US, LIMIT_LEDC_MAX_FREQ_HZ (if needed)

**Config Files Referenced:**
- `firmware/components/config/include/config_peripherals.h` - LEDC_TIMER_D, LEDC_CHANNEL_D, LEDC_MODE_D
- `firmware/components/config/include/config_gpio.h` - GPIO_D_STEP
- `firmware/components/config/include/config_timing.h` - TIMING_DIR_SETUP_US
- `firmware/components/config/include/config_limits.h` - LIMIT_MAX_PULSE_FREQ_HZ, DEFAULT_MAX_PULSE_FREQ_HZ

### Learnings from Previous Story

**From Story 3-3-mcpwm-pulse-generator-with-pcnt (Status: review)**

- **IPulseGenerator Interface Available**: Full interface at `firmware/components/pulse_gen/include/i_pulse_generator.h` - implement same virtual methods
- **Trapezoidal Profile Algorithm**: Profile calculation logic in `rmt_pulse_gen.cpp` and `mcpwm_pulse_gen.cpp` can be reused
- **Atomic State Management**: Use `std::atomic<State>` for thread-safe state - follow same pattern
- **Completion Callback Pattern**: Use FreeRTOS task notification to defer callback from ISR/timer to task context
- **Direction Setup**: Call `sr_set_direction()` before motion, wait TIMING_DIR_SETUP_US
- **No Magic Numbers (AC15)**: All config values use named constants; verified via grep

**Files to Reuse:**
- `firmware/components/pulse_gen/include/i_pulse_generator.h` - Implement this interface
- `firmware/components/drivers/tpic6b595/include/tpic6b595.h` - Call `sr_set_direction()` before motion start

[Source: docs/sprint-artifacts/3-3-mcpwm-pulse-generator-with-pcnt.md#Completion-Notes-List]

### References

- [Source: docs/epics.md#Story-3.4] - Story definition and acceptance criteria
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Services-and-Modules] - LedcPulseGenerator module
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#APIs-and-Interfaces] - IPulseGenerator interface
- [Source: docs/architecture.md#Peripheral-Assignments] - LEDC_TIMER_D, LEDC_CHANNEL_D
- [ESP-IDF LEDC Driver](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-reference/peripherals/ledc.html) - Official LEDC documentation
- [ESP-IDF High Resolution Timer](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-reference/system/esp_timer.html) - esp_timer for pulse counting

---

## Dev Agent Record

### Context Reference

<!-- Path(s) to story context XML will be added here by context workflow -->

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List

### File List

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-05 | SM Agent (Bob) | Initial story draft for LEDC pulse generator with software counting |
