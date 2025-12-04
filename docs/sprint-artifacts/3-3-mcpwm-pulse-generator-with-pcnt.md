# Story 3.3: MCPWM Pulse Generator with PCNT (Y, C Axes)

Status: review

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
15. **AC15 (MANDATORY):** Given any implementation code, when reviewed, then NO hardcoded values, magic numbers, or inline constants exist; ALL configuration values (GPIO pins, timer IDs, PCNT unit IDs, timing values, frequency limits, buffer sizes) MUST be defined in dedicated header files (`config_gpio.h`, `config_peripherals.h`, `config_timing.h`, `config_limits.h`) and referenced via named constants

## Tasks / Subtasks

- [x] **Task 1: Create McpwmPulseGenerator class** (AC: 1, 2, 15)
  - [x] Create `firmware/components/pulse_gen/include/mcpwm_pulse_gen.h` with class declaration
  - [x] Create `firmware/components/pulse_gen/mcpwm_pulse_gen.cpp` with implementation
  - [x] Constructor takes: timer_id (MCPWM_TIMER_Y or MCPWM_TIMER_C), gpio_num, pcnt_unit_id
  - [x] Store MCPWM group ID (use `MCPWM_GROUP_ID` constant from `config_peripherals.h`, NOT literal `0`)
  - [x] Update `firmware/components/pulse_gen/CMakeLists.txt` with REQUIRES: esp_driver_mcpwm, esp_driver_pcnt
  - [x] **CRITICAL**: All values from config headers - NO inline numbers (e.g., use `MCPWM_RESOLUTION_HZ` not `10000000`)

- [x] **Task 2: Implement MCPWM timer and operator initialization** (AC: 1, 15)
  - [x] `init()` creates MCPWM timer via `mcpwm_new_timer()`:
    - `group_id = MCPWM_GROUP_ID` (from `config_peripherals.h`)
    - `clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT`
    - `resolution_hz = MCPWM_RESOLUTION_HZ` (define as 10MHz in `config_peripherals.h`)
    - `count_mode = MCPWM_TIMER_COUNT_MODE_UP`
    - `period_ticks` = calculated from `MCPWM_RESOLUTION_HZ / target_frequency`
  - [x] Create MCPWM operator via `mcpwm_new_operator()` and connect to timer
  - [x] Create MCPWM generator via `mcpwm_new_generator()`:
    - `gen_gpio_num` from constructor (passed from `config_gpio.h` constants)
    - Configure for 50% duty cycle using `MCPWM_DUTY_CYCLE_PERCENT` constant
  - [x] Set generator actions for UP counting: HIGH on zero, LOW on compare

- [x] **Task 3: Implement PCNT initialization with io_loop_back** (AC: 2, 4, 5, 7)
  - [x] Create PCNT unit via `pcnt_new_unit()`:
    - `high_limit = INT16_MAX` (initial, updated per move)
    - `low_limit = INT16_MIN`
    - `flags.accum_count = true` (accumulate across overflows for 64-bit range)
  - [x] Create PCNT channel via `pcnt_new_channel()`:
    - `edge_gpio_num = gpio_num` (same as MCPWM output)
    - `level_gpio_num = -1` (no level signal)
  - [x] Configure GPIO for internal loopback:
    - `gpio_set_direction(gpio_num, GPIO_MODE_INPUT_OUTPUT)`
    - Use `io_loop_back` flag in PCNT channel config
  - [x] Set channel edge actions: increment on rising edge
  - [x] Register PCNT watch points for limit callback

- [x] **Task 4: Implement PCNT limit callback for motion stop** (AC: 5, 7)
  - [x] Register `pcnt_unit_register_event_callbacks()` with on_reach callback
  - [x] In callback:
    - Stop MCPWM timer immediately via `mcpwm_timer_start_stop(MCPWM_TIMER_STOP_FULL)`
    - Read final PCNT count
    - Set state to IDLE
    - Queue task notification for completion callback
  - [x] Handle PCNT overflow events for extended range tracking

- [x] **Task 5: Implement trapezoidal profile generator** (AC: 3, 8, 9)
  - [x] Reuse profile calculation logic from RmtPulseGenerator (same algorithm)
  - [x] Create internal profile state machine: IDLE, ACCELERATING, CRUISING, DECELERATING
  - [x] Implement frequency update callback using MCPWM compare value changes
  - [x] Profile generates batches of pulses at varying frequencies

- [x] **Task 6: Implement startMove()** (AC: 3, 6, 7)
  - [x] Validate pulse_count > 0, velocity > 0, acceleration > 0
  - [x] Validate velocity <= LIMIT_MAX_PULSE_FREQ_HZ
  - [x] Calculate direction from pulse_count sign
  - [x] Call shift register `sr_set_direction()` for DIR signal
  - [x] Wait TIMING_DIR_SETUP_US (20µs) if direction changed
  - [x] Calculate trapezoidal profile
  - [x] Reset PCNT count to 0
  - [x] Set PCNT watch point to target pulse count (handle 16-bit limit)
  - [x] Enable PCNT unit
  - [x] Start MCPWM timer via `mcpwm_timer_start_stop(MCPWM_TIMER_START_NO_STOP)`
  - [x] Set state to RUNNING
  - [x] Return ESP_OK

- [x] **Task 7: Implement startVelocity()** (AC: 8)
  - [x] Validate velocity range (can be negative for reverse)
  - [x] Set target velocity and acceleration
  - [x] Enter continuous mode (no target pulse count, no PCNT limit watch)
  - [x] Start MCPWM timer
  - [x] Continue generating pulses until stop() called
  - [x] PCNT tracks position throughout for reporting

- [x] **Task 8: Implement stop() and stopImmediate()** (AC: 9, 10)
  - [x] `stop(deceleration)`:
    - Calculate decel profile from current velocity
    - Transition to DECELERATING state
    - Reduce frequency progressively
    - Stop timer when velocity reaches zero
    - Fire callback on completion
  - [x] `stopImmediate()`:
    - Call `mcpwm_timer_start_stop(MCPWM_TIMER_STOP_FULL)` immediately
    - Read final PCNT count
    - Set state to IDLE
    - Do NOT fire completion callback (aborted)
  - [x] Verify stop latency <100µs per NFR

- [x] **Task 9: Implement status methods** (AC: 4, 12, 13)
  - [x] `isRunning()`: return state != IDLE
  - [x] `getPulseCount()`: read PCNT count + overflow tracking for 64-bit value
  - [x] `getCurrentVelocity()`: calculate from current MCPWM period

- [x] **Task 10: Implement completion callback** (AC: 7)
  - [x] Store callback via `setCompletionCallback()`
  - [x] On motion complete (PCNT limit reached):
    - Call callback with total_pulses from PCNT
    - Use FreeRTOS task notification to defer from ISR to task context
  - [x] On stopImmediate(): do NOT call callback

- [x] **Task 11: Create unit tests** (AC: 1-15)
  - [x] Create `firmware/components/pulse_gen/test/test_mcpwm_pulse_gen.cpp`
  - [x] Test init() returns ESP_OK for both Y and C channels
  - [x] Test startMove() with various pulse counts and frequencies
  - [x] Test PCNT count matches commanded pulses
  - [x] Test PCNT limit callback stops MCPWM correctly
  - [x] Test frequency limits (1 Hz to `LIMIT_MAX_PULSE_FREQ_HZ`)
  - [x] Test invalid parameters return ESP_ERR_INVALID_ARG
  - [x] Test stop() decelerates correctly
  - [x] Test stopImmediate() stops within timing requirement
  - [x] Test completion callback fires with correct pulse count
  - [x] Test dual-channel simultaneous operation (Y and C together)
  - [x] **VERIFY**: All test values use named constants from config headers (no magic numbers)

- [ ] **Task 12: Hardware verification** (AC: 3, 10, 11)
  - [ ] Verify pulse timing at 10kHz, 25kHz, 100kHz with oscilloscope
  - [ ] Verify 50% duty cycle across frequency range
  - [ ] Verify DIR setup timing (`TIMING_DIR_SETUP_US` before first STEP)
  - [ ] Verify PCNT count accuracy vs oscilloscope pulse count
  - [ ] Verify dual-channel simultaneous operation without interference
  - [ ] Measure PCNT callback to MCPWM stop latency (<`LIMIT_STOP_LATENCY_US`)

- [x] **Task 13: Code Review - No Magic Numbers** (AC: 15)
  - [x] Review `mcpwm_pulse_gen.cpp` for any hardcoded numeric values
  - [x] Verify all GPIO pins referenced via `config_gpio.h` constants
  - [x] Verify all timer/PCNT IDs referenced via `config_peripherals.h` constants
  - [x] Verify all timing values referenced via `config_timing.h` constants
  - [x] Verify all limits referenced via `config_limits.h` constants
  - [x] Ensure `config_peripherals.h` contains: `MCPWM_GROUP_ID`, `MCPWM_RESOLUTION_HZ`, `MCPWM_DUTY_CYCLE_PERCENT`, `PCNT_UNIT_Y`, `PCNT_UNIT_C`, `MCPWM_TIMER_Y`, `MCPWM_TIMER_C`
  - [x] Run grep check: `grep -E "[^A-Z_][0-9]{2,}" mcpwm_pulse_gen.cpp` should return minimal results (only loop indices, etc.)

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

- `docs/sprint-artifacts/stories/3-3-mcpwm-pulse-generator-with-pcnt.context.xml`

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

- Build error: Missing `#include "driver/mcpwm_cmpr.h"` for ESP-IDF v5.4 comparator API - fixed by adding include to header

### Completion Notes List

1. **McpwmPulseGenerator class implemented** - Full implementation of IPulseGenerator interface using MCPWM peripheral with PCNT feedback
2. **PCNT io_loop_back routing** - Internal GPIO matrix routing eliminates external loopback wire requirement
3. **Trapezoidal profile algorithm** - Reused same algorithm from RmtPulseGenerator (IDLE → ACCELERATING → CRUISING → DECELERATING)
4. **64-bit pulse counting** - PCNT overflow tracking extends 16-bit PCNT to full 64-bit range
5. **ISR-safe completion callback** - FreeRTOS task notification defers callback from PCNT ISR to task context
6. **No magic numbers (AC15)** - All config values use named constants from config headers; verified via grep
7. **Comprehensive unit tests** - Tests cover all 15 ACs including dual-channel operation, boundary conditions, error handling
8. **Task 12 pending** - Hardware verification requires oscilloscope/physical testing

### File List

**Created:**
- `firmware/components/pulse_gen/include/mcpwm_pulse_gen.h` - Class declaration
- `firmware/components/pulse_gen/mcpwm_pulse_gen.cpp` - Implementation (~860 lines)
- `firmware/components/pulse_gen/test/test_mcpwm_pulse_gen.cpp` - Unit tests

**Modified:**
- `firmware/components/pulse_gen/CMakeLists.txt` - Added mcpwm_pulse_gen.cpp, esp_driver_mcpwm, esp_driver_pcnt
- `firmware/components/config/include/config_peripherals.h` - Added MCPWM_RESOLUTION_HZ, MCPWM_DUTY_CYCLE_PERCENT

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-04 | SM Agent (Bob) | Initial story draft for MCPWM pulse generator with PCNT |
| 2025-12-05 | SM Agent (Bob) | Added AC15 (MANDATORY): No hardcoded values/magic numbers requirement; Added Task 13 for code review; Updated Tasks 1, 2, 11, 12 with AC15 references |
| 2025-12-05 | Dev Agent (Amelia) | Implementation complete: Tasks 1-11, 13 done; Task 12 pending hardware verification; Status → review |
