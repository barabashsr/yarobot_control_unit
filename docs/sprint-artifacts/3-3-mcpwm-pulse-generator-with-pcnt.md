# Story 3.3: MCPWM Pulse Generator with PCNT (Y, C Axes)

Status: drafted

## Story

As a **developer**,
I want **MCPWM-based pulse generation for Y and C axes with hardware pulse counting**,
so that **I can generate STEP pulses and track position via PCNT for these axes**.

## Acceptance Criteria

1. **AC1:** Given MCPWM timers MCPWM_TIMER_Y (0) and MCPWM_TIMER_C (1) are configured with group MCPWM_GROUP_ID (0), when pulse generation is requested, then STEP pulses are generated on GPIO_Y_STEP (GPIO5) and GPIO_C_STEP (GPIO16) respectively
2. **AC2:** Given ESP-IDF v5.x `io_loop_back` flag is enabled on GPIO, when MCPWM output and PCNT input are configured on same GPIO, then internal GPIO matrix routes MCPWM output to PCNT input without external loopback wire
3. **AC3:** Given I call `startMove(5000, 25000, 500000)` (5000 pulses at 25kHz max velocity with 500k pulses/s² accel), when MCPWM generates pulses, then exactly 5000 pulses are output with ±1% timing accuracy and PCNT_UNIT_Y or PCNT_UNIT_C counts match commanded pulses
4. **AC4:** Given pulse generation is active, when I query `getPulseCount()`, then PCNT unit returns accurate count matching generated pulses
5. **AC5:** Given PCNT high limit is set to target pulse count, when limit is reached, then MCPWM stops automatically via PCNT limit callback
6. **AC6:** Given direction change is needed before motion, when pulse generation starts, then TIMING_DIR_SETUP_US (20µs) delay occurs after DIR signal change before first STEP pulse
7. **AC7:** Given motion completes (PCNT limit reached), when the last pulse is sent, then MotionCompleteCallback fires with total_pulses count from PCNT
8. **AC8:** Given I call `startVelocity(50000, 500000)` (50kHz velocity, 500k accel), when MCPWM generates pulses, then continuous pulses are generated until `stop()` is called and PCNT tracks position throughout
9. **AC9:** Given I call `stop(1000000)` during motion, when deceleration completes, then motion stops gracefully with controlled deceleration profile
10. **AC10:** Given I call `stopImmediate()` during motion, when executed, then MCPWM stops within 100µs (PCNT callback + PWM stop latency per NFR)
11. **AC11:** Given both Y and C axes are active simultaneously, when generating pulses at 25kHz each, then no interference or timing degradation occurs between channels
12. **AC12:** Given `isRunning()` is called during motion, when motion is active, then true is returned; when idle, false is returned
13. **AC13:** Given `getCurrentVelocity()` is called during motion, when motion is active, then current pulse frequency (Hz) is returned as float
14. **AC14:** Given C axis is a stepper (not servo), when motion completes, then position is calculated solely from PCNT count (no external feedback)

## Tasks / Subtasks

- [ ] **Task 1: Create McpwmPulseGenerator class** (AC: 1, 2)
  - [ ] Create `firmware/components/pulse_gen/include/mcpwm_pulse_gen.h` with class declaration
  - [ ] Create `firmware/components/pulse_gen/mcpwm_pulse_gen.cpp` with implementation
  - [ ] Constructor takes: timer_id (MCPWM_TIMER_Y or MCPWM_TIMER_C), gpio_num, pcnt_unit_id
  - [ ] Store MCPWM group ID (MCPWM_GROUP_ID = 0)
  - [ ] Update `firmware/components/pulse_gen/CMakeLists.txt` with REQUIRES: esp_driver_mcpwm, esp_driver_pcnt

- [ ] **Task 2: Implement MCPWM timer and operator initialization** (AC: 1)
  - [ ] `init()` creates MCPWM timer via `mcpwm_new_timer()`:
    - `group_id = MCPWM_GROUP_ID`
    - `clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT`
    - `resolution_hz = 10000000` (10MHz = 100ns resolution)
    - `count_mode = MCPWM_TIMER_COUNT_MODE_UP`
    - `period_ticks` = initial period (variable based on frequency)
  - [ ] Create MCPWM operator via `mcpwm_new_operator()` and connect to timer
  - [ ] Create MCPWM generator via `mcpwm_new_generator()`:
    - `gen_gpio_num` from constructor
    - Configure for 50% duty cycle
  - [ ] Set generator actions for UP counting: HIGH on zero, LOW on compare

- [ ] **Task 3: Implement PCNT initialization with io_loop_back** (AC: 2, 4, 5, 7)
  - [ ] Create PCNT unit via `pcnt_new_unit()`:
    - `high_limit = INT16_MAX` (initial, updated per move)
    - `low_limit = INT16_MIN`
    - `flags.accum_count = true` (accumulate across overflows for 64-bit range)
  - [ ] Create PCNT channel via `pcnt_new_channel()`:
    - `edge_gpio_num = gpio_num` (same as MCPWM output)
    - `level_gpio_num = -1` (no level signal)
  - [ ] Configure GPIO for internal loopback:
    - `gpio_set_direction(gpio_num, GPIO_MODE_INPUT_OUTPUT)`
    - Use `io_loop_back` flag in PCNT channel config
  - [ ] Set channel edge actions: increment on rising edge
  - [ ] Register PCNT watch points for limit callback

- [ ] **Task 4: Implement PCNT limit callback for motion stop** (AC: 5, 7)
  - [ ] Register `pcnt_unit_register_event_callbacks()` with on_reach callback
  - [ ] In callback:
    - Stop MCPWM timer immediately via `mcpwm_timer_start_stop(MCPWM_TIMER_STOP_FULL)`
    - Read final PCNT count
    - Set state to IDLE
    - Queue task notification for completion callback
  - [ ] Handle PCNT overflow events for extended range tracking

- [ ] **Task 5: Implement trapezoidal profile generator** (AC: 3, 8, 9)
  - [ ] Reuse profile calculation logic from RmtPulseGenerator (same algorithm)
  - [ ] Create internal profile state machine: IDLE, ACCELERATING, CRUISING, DECELERATING
  - [ ] Implement frequency update callback using MCPWM compare value changes
  - [ ] Profile generates batches of pulses at varying frequencies

- [ ] **Task 6: Implement startMove()** (AC: 3, 6, 7)
  - [ ] Validate pulse_count > 0, velocity > 0, acceleration > 0
  - [ ] Validate velocity <= LIMIT_MAX_PULSE_FREQ_HZ
  - [ ] Calculate direction from pulse_count sign
  - [ ] Call shift register `sr_set_direction()` for DIR signal
  - [ ] Wait TIMING_DIR_SETUP_US (20µs) if direction changed
  - [ ] Calculate trapezoidal profile
  - [ ] Reset PCNT count to 0
  - [ ] Set PCNT watch point to target pulse count (handle 16-bit limit)
  - [ ] Enable PCNT unit
  - [ ] Start MCPWM timer via `mcpwm_timer_start_stop(MCPWM_TIMER_START_NO_STOP)`
  - [ ] Set state to RUNNING
  - [ ] Return ESP_OK

- [ ] **Task 7: Implement startVelocity()** (AC: 8)
  - [ ] Validate velocity range (can be negative for reverse)
  - [ ] Set target velocity and acceleration
  - [ ] Enter continuous mode (no target pulse count, no PCNT limit watch)
  - [ ] Start MCPWM timer
  - [ ] Continue generating pulses until stop() called
  - [ ] PCNT tracks position throughout for reporting

- [ ] **Task 8: Implement stop() and stopImmediate()** (AC: 9, 10)
  - [ ] `stop(deceleration)`:
    - Calculate decel profile from current velocity
    - Transition to DECELERATING state
    - Reduce frequency progressively
    - Stop timer when velocity reaches zero
    - Fire callback on completion
  - [ ] `stopImmediate()`:
    - Call `mcpwm_timer_start_stop(MCPWM_TIMER_STOP_FULL)` immediately
    - Read final PCNT count
    - Set state to IDLE
    - Do NOT fire completion callback (aborted)
  - [ ] Verify stop latency <100µs per NFR

- [ ] **Task 9: Implement status methods** (AC: 4, 12, 13)
  - [ ] `isRunning()`: return state != IDLE
  - [ ] `getPulseCount()`: read PCNT count + overflow tracking for 64-bit value
  - [ ] `getCurrentVelocity()`: calculate from current MCPWM period

- [ ] **Task 10: Implement completion callback** (AC: 7)
  - [ ] Store callback via `setCompletionCallback()`
  - [ ] On motion complete (PCNT limit reached):
    - Call callback with total_pulses from PCNT
    - Use FreeRTOS task notification to defer from ISR to task context
  - [ ] On stopImmediate(): do NOT call callback

- [ ] **Task 11: Create unit tests** (AC: 1-14)
  - [ ] Create `firmware/components/pulse_gen/test/test_mcpwm_pulse_gen.cpp`
  - [ ] Test init() returns ESP_OK for both Y and C channels
  - [ ] Test startMove() with various pulse counts and frequencies
  - [ ] Test PCNT count matches commanded pulses
  - [ ] Test PCNT limit callback stops MCPWM correctly
  - [ ] Test frequency limits (1 Hz to 500 kHz)
  - [ ] Test invalid parameters return ESP_ERR_INVALID_ARG
  - [ ] Test stop() decelerates correctly
  - [ ] Test stopImmediate() stops within timing requirement
  - [ ] Test completion callback fires with correct pulse count
  - [ ] Test dual-channel simultaneous operation (Y and C together)

- [ ] **Task 12: Hardware verification** (AC: 3, 10, 11)
  - [ ] Verify pulse timing at 10kHz, 25kHz, 100kHz with oscilloscope
  - [ ] Verify 50% duty cycle across frequency range
  - [ ] Verify DIR setup timing (20µs before first STEP)
  - [ ] Verify PCNT count accuracy vs oscilloscope pulse count
  - [ ] Verify dual-channel simultaneous operation without interference
  - [ ] Measure PCNT callback to MCPWM stop latency (<100µs)

## Dev Notes

### Architecture Constraints

> **Header-Only Configuration (from Tech Spec)**
>
> Every configurable value MUST be defined in a header file. No magic numbers in source code. MCPWM timer/group IDs from `config_peripherals.h`, GPIO pins from `config_gpio.h`, PCNT unit IDs from `config_peripherals.h`.
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

> **Streaming Double-Buffer Pulse Generation (from Tech Spec)**
>
> All pulse generation uses streaming double-buffer architecture. For MCPWM, this means progressive frequency updates during motion rather than pre-computed buffers.
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

### Core Concepts

**MCPWM + PCNT Internal Routing:**
- ESP32-S3 GPIO matrix allows internal routing of outputs back to inputs
- Configure GPIO as `GPIO_MODE_INPUT_OUTPUT` for bidirectional operation
- PCNT channel with `edge_gpio_num` same as MCPWM generator `gen_gpio_num`
- No external loopback wire needed between pins
- Reference: https://esp32.com/viewtopic.php?t=30817

**MCPWM Timer Configuration:**
- 10MHz resolution = 100ns tick granularity
- Period ticks = resolution_hz / target_frequency
- For 100kHz: period = 10000000 / 100000 = 100 ticks
- For 25kHz: period = 10000000 / 25000 = 400 ticks
- 50% duty: compare value = period / 2

**PCNT Limit-Based Stop:**
- PCNT 16-bit counter with high/low limit watch points
- Watch point callback fires when limit reached
- Callback stops MCPWM timer immediately
- For counts > 32767: use overflow tracking + final limit

**Axis Differences:**
- **Y Axis**: Servo motor, PCNT provides backup counting (encoder is primary)
- **C Axis**: Stepper motor, PCNT is sole source of position (no encoder)

### Project Structure Notes

**Files to Create:**
- `firmware/components/pulse_gen/include/mcpwm_pulse_gen.h` - Class declaration
- `firmware/components/pulse_gen/mcpwm_pulse_gen.cpp` - Implementation
- `firmware/components/pulse_gen/test/test_mcpwm_pulse_gen.cpp` - Unit tests

**Existing Files to Update:**
- `firmware/components/pulse_gen/CMakeLists.txt` - Add esp_driver_mcpwm, esp_driver_pcnt to REQUIRES

**Config Files Referenced:**
- `firmware/components/config/include/config_peripherals.h` - MCPWM_GROUP_ID, MCPWM_TIMER_Y, MCPWM_TIMER_C, PCNT_UNIT_Y, PCNT_UNIT_C
- `firmware/components/config/include/config_gpio.h` - GPIO_Y_STEP, GPIO_C_STEP
- `firmware/components/config/include/config_timing.h` - TIMING_DIR_SETUP_US
- `firmware/components/config/include/config_limits.h` - LIMIT_MAX_PULSE_FREQ_HZ, LIMIT_MCPWM_BUFFER_PULSES

### Learnings from Previous Story

**From Story 3-2-rmt-pulse-generator (Status: review)**

- **IPulseGenerator Interface Available**: Full interface at `firmware/components/pulse_gen/include/i_pulse_generator.h` - implement same virtual methods
- **Trapezoidal Profile Algorithm**: Profile calculation logic in `rmt_pulse_gen.cpp` can be reused or refactored into shared utility
- **Atomic State Management**: RMT uses `std::atomic<State>` for thread-safe state - follow same pattern
- **Completion Callback Pattern**: Use FreeRTOS task notification to defer callback from ISR to task context
- **Unit Test Structure**: Test file structure established at `test_rmt_pulse_gen.cpp` - follow same patterns

**Files to Reuse:**
- `firmware/components/pulse_gen/include/i_pulse_generator.h` - Implement this interface
- `firmware/components/drivers/tpic6b595/include/tpic6b595.h` - Call `sr_set_direction()` before motion start

[Source: docs/sprint-artifacts/3-2-rmt-pulse-generator.md#Completion-Notes-List]

### References

- [Source: docs/epics.md#Story-3.3] - Story definition and acceptance criteria
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Services-and-Modules] - McpwmPulseGenerator module
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#APIs-and-Interfaces] - IPulseGenerator interface
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Performance] - MCPWM stop latency requirements (<100µs)
- [Source: docs/architecture.md#Peripheral-Assignments] - MCPWM_TIMER_Y, MCPWM_TIMER_C, PCNT_UNIT_Y, PCNT_UNIT_C
- [ESP-IDF MCPWM Driver](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-reference/peripherals/mcpwm.html) - Official MCPWM documentation
- [ESP-IDF PCNT Driver](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-reference/peripherals/pcnt.html) - Official PCNT documentation
- [ESP32 Forum: MCPWM+PCNT io_loop_back](https://esp32.com/viewtopic.php?t=30817) - Internal routing technique

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
| 2025-12-04 | SM Agent (Bob) | Initial story draft for MCPWM pulse generator with PCNT |
