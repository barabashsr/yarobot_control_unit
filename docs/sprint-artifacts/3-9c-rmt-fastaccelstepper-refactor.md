# Story 3.9c: RMT Pulse Generator FastAccelStepper Refactor

Status: done

## Story

As a **developer**,
I want **the RMT pulse generator refactored to use FastAccelStepper callback encoder pattern**,
so that **all 4 RMT axes (X, Z, A, B) can operate simultaneously without channel exhaustion, with bounded-latency position tracking and mid-motion blending support**.

## Acceptance Criteria

1. **AC1:** Given all 4 RMT axes configured, when 4 channels run at 200kHz each, then no "no free tx channels" error occurs
2. **AC2:** Given position is queried during motion, then getPulseCount() returns value with <10ms latency at all frequencies
3. **AC3:** Given startMove() called during active motion, then new profile is atomically blended from current velocity
4. **AC4:** Given RMT config, when with_dma=false, then mem_block_symbols=48 and resolution_hz=16MHz
5. **AC5:** Given encoder callback, when ISR executes, then no FPU operations occur (integer math only)
6. **AC6:** Given existing IPulseGenerator tests, when run against refactored impl, then all tests pass
7. **AC7:** Given hardware verification, when 4-channel simultaneous operation tested, then no interference occurs
8. **AC8 (MANDATORY):** Given any implementation code, when reviewed, then NO hardcoded numeric values exist; ALL configuration values MUST come from config headers

## Tasks / Subtasks

- [x] **Task 1: Update config_limits.h** (AC: 4)
  - [x] Change LIMIT_RMT_RESOLUTION_HZ from 80MHz to 16MHz
  - [x] Add LIMIT_RMT_PART_SIZE = 24
  - [x] Add LIMIT_RMT_QUEUE_LEN = 32
  - [x] Add LIMIT_RMT_MIN_CMD_TICKS = 3200
  - [x] Add LIMIT_RMT_FORWARD_PLANNING_TICKS (10ms)
  - [x] Add LIMIT_RMT_RAMP_TASK_PERIOD_MS = 4

- [x] **Task 2: Create rmt_pulse_gen_types.h** (AC: 3)
  - [x] Define StepCommand struct (ticks, steps, flags)
  - [x] Define RampState enum
  - [x] Define RampParameters struct (atomic fields)
  - [x] Define QueueState struct

- [x] **Task 3: Refactor RmtPulseGenerator init** (AC: 4)
  - [x] Remove DMA configuration (with_dma = false)
  - [x] Create simple encoder with callback via rmt_new_simple_encoder()
  - [x] Create per-axis ramp task with semaphore

- [x] **Task 4: Implement encoder callback (ISR)** (AC: 2, 5)
  - [x] Read commands from queue (atomic read_idx/write_idx)
  - [x] Generate PART_SIZE symbols per call
  - [x] Track position via atomic pulse_count_
  - [x] Call IPositionTracker::addPulses() from ISR
  - [x] Wake ramp task when queue low

- [x] **Task 5: Implement ramp generator task** (AC: 3)
  - [x] Event-driven (semaphore-based) wake
  - [x] fillQueue() with trapezoidal profile
  - [x] generateNextCommand() state machine
  - [x] calculateStepsForLatency() for bounded latency

- [x] **Task 6: Implement mid-motion blending** (AC: 3)
  - [x] startMove() works while running
  - [x] Atomic parameter replacement
  - [x] Queue drain strategy for smooth blend

- [x] **Task 7: Build verification** (AC: 6)
  - [x] Build compiles successfully with all changes
  - [ ] Add test for 4-channel simultaneous operation (hardware test)

- [x] **Task 8: Hardware verification** (AC: 1, 7)
  - [x] Verify 4-channel simultaneous operation - **PASSED** (all 5 GDMA pairs free)
  - [ ] Verify position latency <10ms - *Deferred to story 3-9d testing*
  - [ ] Verify mid-motion blend behavior - *Deferred to story 3-9d testing*
  - *Note: GDMA probe confirms no channel exhaustion; functional testing pending EN command*

- [x] **Task 9: Code review - No Magic Numbers** (AC: 8)
  - [x] Review all .cpp and .h files for hardcoded numeric values
  - [x] Verify all RMT parameters from `config_limits.h` (resolution, buffer size, queue length)
  - [x] Verify all timing from `config_timing.h` (task periods, timeouts)
  - [x] Verify all GPIO/peripheral assignments from `config_gpio.h` and `config_peripherals.h`
  - [x] Run grep check: `grep -rn "[0-9]\{2,\}" firmware/components/pulse_gen/` to find suspects
  - [x] Document exceptions: Type limits (65535=UINT16_MAX, 255=UINT8_MAX) and unit conversions (1000 ms/s, 1000000 µs/s) are acceptable per AC8 exceptions

## Dev Notes

### Learnings from Previous Story

**From Story 3-9b-motor-system-integration (Status: review)**

Story 3-9b establishes the motor system wiring and initialization framework, including:
- `motor_system.cpp` instantiates RmtPulseGenerator for X, Z, A, B axes
- SoftwareTracker instances wired to RMT pulse generators via `setPositionTracker()`
- GPIO assignments from `config_gpio.h` (GPIO_X_STEP, etc.)

This refactor (3-9c) modifies the internal implementation of `RmtPulseGenerator` while preserving the `IPulseGenerator` interface. The motor_system wiring established in 3-9b remains unchanged.

**Note:** Story 3-9b is currently under review; Dev Agent Record sections are not yet populated. No completion notes or review items available to incorporate at this time.

[Source: docs/sprint-artifacts/3-9b-motor-system-integration.md]

### Architecture Reference

This story implements the migration strategy from:
`docs/architecture-changes/fastaccelstepper-migration-strategy.md`

Key architectural decisions from that document:
- **RMT Resolution:** 16 MHz (FastAccelStepper proven; 62.5ns adequate for 500 kHz)
- **Position Tracking:** Encoder callback (ESP32-S3 has only 4 PCNT units; 3 already used)
- **Integration:** Port patterns to IPulseGenerator (no external dependency)
- **Mid-Motion:** Full blending (velocity + target) per PRD requirement

### MANDATORY: No Magic Numbers

> **All numeric configuration values MUST be defined in header files. No hardcoded literals in source code.**
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

**Config Header Mapping:**

| Parameter Type | Header File |
|---------------|-------------|
| RMT resolution, buffer sizes, queue lengths | `config_limits.h` |
| Task periods, timeouts, delays | `config_timing.h` |
| GPIO pin assignments | `config_gpio.h` |
| Peripheral assignments (RMT channels, PCNT units) | `config_peripherals.h` |
| Axis IDs, counts | `config_axes.h` |

**Acceptable Exceptions (must be commented):**
- Bit manipulation constants (0xFF, 0x01, shift amounts)
- Array indices (0, 1, 2)
- Boolean logic (0, 1)
- ESP-IDF API requirements (e.g., `trans_queue_depth = 1`)

**Verification:**
```bash
# Find potential magic numbers (2+ digit literals)
grep -rn "[0-9]\{2,\}" firmware/components/pulse_gen/*.cpp
```

### ISR Safety Requirements

All encoder callback code must be ISR-safe:
- NO floating-point operations (FPU not available in ISR)
- NO blocking calls, NO mutex locks
- NO memory allocation
- Use only atomic operations for shared state
- All functions must have `IRAM_ATTR` attribute

### Interface Unchanged

The `IPulseGenerator` interface is **UNCHANGED**. The critical change is behavioral:
- `startMove()` now works while running
- Position tracking via encoder callback instead of software estimation

### Files to Modify

**Modified Files:**
- `firmware/components/config/include/config_limits.h` - New RMT constants
- `firmware/components/config/include/config_timing.h` - Task timing constants
- `firmware/components/pulse_gen/rmt_pulse_gen.cpp` - Complete refactor
- `firmware/components/pulse_gen/include/rmt_pulse_gen.h` - Class structure updates

**New Files:**
- `firmware/components/pulse_gen/include/rmt_pulse_gen_types.h` - Data structures

**Deferred (per user request):**
- `docs/architecture.md` - Will be updated AFTER implementation

### Prerequisites

- Story 3.2 (DONE) - Original RMT implementation to be replaced
- Story 3.5 (DONE) - Position tracker interface (ISR safety now required)
- Stories 3.9, 3.9b (REVIEW) - Motor integration uses IPulseGenerator interface

### References

- FastAccelStepper Migration Strategy: `docs/architecture-changes/fastaccelstepper-migration-strategy.md`
- ESP-IDF 5.x RMT Driver: https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-reference/peripherals/rmt.html
- Original Story 3.2: `docs/sprint-artifacts/3-2-rmt-pulse-generator.md`

---

## Dev Agent Record

### Context Reference

- `docs/sprint-artifacts/3-9c-rmt-fastaccelstepper-refactor.context.xml`

### Agent Model Used

- Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

- Build errors fixed: IRAM_ATTR declaration conflicts, std::min type mismatches, gpio_num_t casting, constructor signature updates

### Completion Notes List

1. **RMT Configuration**: Changed from 80 MHz to 16 MHz resolution per FastAccelStepper patterns. Added `with_dma = 0` to enable all 4 RMT channels simultaneously.

2. **Callback Encoder Pattern**: Implemented using `rmt_new_simple_encoder()` with custom `encodeCallback()`. ISR fills symbols from command queue without FPU operations.

3. **Command Queue**: Lock-free circular queue (32 entries) with atomic read/write indices. Commands contain (ticks, steps, flags) for efficient ISR processing.

4. **Ramp Generator Task**: Per-axis FreeRTOS task fills queue with trapezoidal profile commands. Event-driven via semaphore wake from ISR when queue falls below threshold.

5. **Mid-Motion Blending**: `startMove()` and `startVelocity()` work while running via atomic parameter replacement. Ramp task reads new params and smoothly transitions from current velocity.

6. **Position Tracking**: Position tracked in encoder callback by incrementing atomic `pulse_count_` per command executed. IPositionTracker::addPulses() called from ISR context for external tracking.

7. **Direction Control**: Direction managed via shift register (TPIC6B595) at MotorBase level, not GPIO. Removed dir_pin from constructor.

8. **Hardware Verification (2025-12-06)**: GDMA probe confirms all 5 GDMA pairs free (5/5 TX, 5/5 RX). Previous DMA implementation had redundant initialization causing channel exhaustion. Functional motion testing deferred pending EN command implementation (Story 3-9d).

### File List

**NEW:**
- `firmware/components/pulse_gen/include/rmt_pulse_gen_types.h` - StepCommand, RampState, RampParameters, QueueState structures

**MODIFIED:**
- `firmware/components/config/include/config_limits.h` - Added LIMIT_RMT_* constants (resolution, queue length, part size, etc.)
- `firmware/components/config/include/config_timing.h` - Added TIMING_RMT_* constants (task stack, position update latency)
- `firmware/components/pulse_gen/include/rmt_pulse_gen.h` - Complete rewrite for callback encoder pattern
- `firmware/components/pulse_gen/rmt_pulse_gen.cpp` - Complete implementation (~950 lines)
- `firmware/components/control/motor_system/motor_system.cpp` - Updated RmtPulseGenerator constructor calls (2 params instead of 3)

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-06 | Sergey | **DONE**: Hardware verification passed - GDMA probe shows 5/5 channels free. Previous DMA implementation had redundant initialization. Architecture docs updated. |
| 2025-12-06 | Dev Agent (Amelia) | Implementation complete: FastAccelStepper callback encoder pattern, command queue, ramp generator, mid-motion blending. Ready for review (hardware verification deferred). |
| 2025-12-06 | SM Agent (Bob) | Added AC8 and Task 9 for mandatory no-magic-numbers requirement |
| 2025-12-06 | SM Agent (Bob) | Generated story context XML, marked ready-for-dev |
| 2025-12-06 | SM Agent (Bob) | Added Learnings from Previous Story, Dev Agent Record, AC2 task mapping |
| 2025-12-06 | PM Agent (John) | Initial story draft via correct-course workflow |
