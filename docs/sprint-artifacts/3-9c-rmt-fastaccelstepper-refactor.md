# Story 3.9c: RMT Pulse Generator FastAccelStepper Refactor

Status: ready-for-dev

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

- [ ] **Task 1: Update config_limits.h** (AC: 4)
  - [ ] Change LIMIT_RMT_RESOLUTION_HZ from 80MHz to 16MHz
  - [ ] Add LIMIT_RMT_PART_SIZE = 24
  - [ ] Add LIMIT_RMT_QUEUE_LEN = 32
  - [ ] Add LIMIT_RMT_MIN_CMD_TICKS = 3200
  - [ ] Add LIMIT_RMT_FORWARD_PLANNING_TICKS (10ms)
  - [ ] Add LIMIT_RMT_RAMP_TASK_PERIOD_MS = 4

- [ ] **Task 2: Create rmt_pulse_gen_types.h** (AC: 3)
  - [ ] Define StepCommand struct (ticks, steps, flags)
  - [ ] Define RampState enum
  - [ ] Define RampParameters struct (atomic fields)
  - [ ] Define QueueState struct

- [ ] **Task 3: Refactor RmtPulseGenerator init** (AC: 4)
  - [ ] Remove DMA configuration (with_dma = false)
  - [ ] Create simple encoder with callback via rmt_new_simple_encoder()
  - [ ] Create per-axis ramp task with semaphore

- [ ] **Task 4: Implement encoder callback (ISR)** (AC: 2, 5)
  - [ ] Read commands from queue (atomic read_idx/write_idx)
  - [ ] Generate PART_SIZE symbols per call
  - [ ] Track position via atomic pulse_count_
  - [ ] Call IPositionTracker::addPulses() from ISR
  - [ ] Wake ramp task when queue low

- [ ] **Task 5: Implement ramp generator task** (AC: 3)
  - [ ] Event-driven (semaphore-based) wake
  - [ ] fillQueue() with trapezoidal profile
  - [ ] generateNextCommand() state machine
  - [ ] calculateStepsForLatency() for bounded latency

- [ ] **Task 6: Implement mid-motion blending** (AC: 3)
  - [ ] startMove() works while running
  - [ ] Atomic parameter replacement
  - [ ] Queue drain strategy for smooth blend

- [ ] **Task 7: Run existing tests** (AC: 6)
  - [ ] Verify all Story 3.2 tests pass
  - [ ] Add test for 4-channel simultaneous operation

- [ ] **Task 8: Hardware verification** (AC: 1, 7)
  - [ ] Verify 4-channel simultaneous operation
  - [ ] Verify position latency <10ms
  - [ ] Verify mid-motion blend behavior

- [ ] **Task 9: Code review - No Magic Numbers** (AC: 8)
  - [ ] Review all .cpp and .h files for hardcoded numeric values
  - [ ] Verify all RMT parameters from `config_limits.h` (resolution, buffer size, queue length)
  - [ ] Verify all timing from `config_timing.h` (task periods, timeouts)
  - [ ] Verify all GPIO/peripheral assignments from `config_gpio.h` and `config_peripherals.h`
  - [ ] Run grep check: `grep -rn "[0-9]\{2,\}" firmware/components/pulse_gen/` to find suspects
  - [ ] Document any exceptions with explicit comments explaining why literal is acceptable

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

<!-- To be filled during implementation -->

### Debug Log References

<!-- To be filled during implementation -->

### Completion Notes List

<!-- To be filled upon story completion -->

### File List

<!-- To be filled during implementation: NEW/MODIFIED files -->

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-06 | SM Agent (Bob) | Added AC8 and Task 9 for mandatory no-magic-numbers requirement |
| 2025-12-06 | SM Agent (Bob) | Generated story context XML, marked ready-for-dev |
| 2025-12-06 | SM Agent (Bob) | Added Learnings from Previous Story, Dev Agent Record, AC2 task mapping |
| 2025-12-06 | PM Agent (John) | Initial story draft via correct-course workflow |
