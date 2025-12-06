# Story 3.6: Motor Base Class & Servo Motor

Status: done

## Story

As a **developer**,
I want **the motor abstraction layer with servo motor implementation**,
so that **servo axes (X, Y, Z, A, B) can execute motion commands through a unified interface**.

## Acceptance Criteria

1. **AC1:** Given an `IMotor` interface is defined, when I implement it, then all motor types share the same API (`moveAbsolute()`, `moveRelative()`, `moveVelocity()`, `stop()`, `stopImmediate()`, `getPosition()`, `isMoving()`, `enable()`, `getState()`)
2. **AC2:** Given a `ServoMotor` is constructed with an `IPulseGenerator`, `IPositionTracker`, `ShiftRegisterController` reference, and `AxisConfig`, when `init()` is called, then all dependencies are validated and servo motor is ready for motion commands
3. **AC3:** Given `moveAbsolute(position, velocity)` is called with valid parameters, when executed, then:
   - Target position is validated against soft limits (`config_.limit_min`, `config_.limit_max`)
   - Direction is calculated and set via shift register (`sr_set_direction()`)
   - SI units converted to pulses: `pulses = (target - current) * pulses_per_unit`
   - Velocity converted to frequency: `freq = velocity * pulses_per_unit`
   - Pulse generator `startMove()` is invoked
   - State is set to `AXIS_STATE_MOVING`
4. **AC4:** Given `moveRelative(delta, velocity)` is called, when executed, then it behaves as `moveAbsolute(current_position + delta, velocity)`
5. **AC5:** Given `moveVelocity(velocity)` is called, when executed, then:
   - Velocity is clamped to `config_.max_velocity`
   - Direction is set via shift register based on velocity sign
   - Pulse generator `startVelocity()` is invoked for continuous motion
   - Motion continues until `stop()` or limit reached
6. **AC6:** Given `stop()` is called during motion, when executed, then pulse generator decelerates using configured acceleration and state transitions to `AXIS_STATE_IDLE` on completion
7. **AC7:** Given `stopImmediate()` is called, when executed, then pulse generator halts immediately (no decel ramp) and state transitions to `AXIS_STATE_IDLE`
8. **AC8:** Given `enable(true)` is called, when executed, then:
   - Shift register EN bit is set via `sr_set_enable(axis, true)`
   - `TIMING_ENABLE_DELAY_US` elapses before motion is allowed
   - State transitions from `AXIS_STATE_DISABLED` to `AXIS_STATE_IDLE`
9. **AC9:** Given `enable(false)` is called while moving, when executed, then:
   - Any active motion stops immediately
   - Shift register EN bit is cleared
   - State transitions to `AXIS_STATE_DISABLED`
10. **AC10:** Given `getPosition()` is called, when executed, then current position in SI units is returned: `position = pulse_count / pulses_per_unit`
11. **AC11:** Given `getState()` returns `AxisState`, when queried, then valid states are: `AXIS_STATE_DISABLED`, `AXIS_STATE_IDLE`, `AXIS_STATE_MOVING`, `AXIS_STATE_ERROR`, `AXIS_STATE_UNHOMED`
12. **AC12:** Given motion completes (pulse generator fires completion callback), when handled, then:
   - State transitions to `AXIS_STATE_IDLE`
   - If motion complete callback is registered, it is invoked with axis and final position
   - `EVT_MOTION_COMPLETE` event is ready to be published (event publishing in Story 3.11)
13. **AC13:** Given a mid-motion `moveAbsolute()` is called (motion blending), when executed, then profile generator recalculates on-the-fly for smooth transition to new target (no "axis busy" error)
14. **AC14:** Given position exceeds soft limits, when `moveAbsolute()` or `moveRelative()` is called, then `ESP_ERR_INVALID_ARG` is returned and no motion occurs
15. **AC15:** Given axis is disabled (`AXIS_STATE_DISABLED`), when motion command is called, then `ESP_ERR_INVALID_STATE` is returned
16. **AC16 (MANDATORY):** Given any implementation code, when reviewed, then NO hardcoded values exist; ALL configuration values (timing, limits, GPIO) MUST be from config headers

## Tasks / Subtasks

- [x] **Task 1: Create IMotor interface** (AC: 1, 16)
  - [x] Create `firmware/components/motor/include/i_motor.h` with interface declaration
  - [x] Define virtual methods: `moveAbsolute()`, `moveRelative()`, `moveVelocity()`, `stop()`, `stopImmediate()`, `getPosition()`, `getVelocity()`, `isMoving()`, `isEnabled()`, `enable()`, `getState()`, `getConfig()`, `setConfig()`
  - [x] Include `AxisState` enum definition
  - [x] Include `AxisConfig` struct definition (or reference from config)
  - [x] Add `using MotionCompleteCallback = std::function<void(uint8_t axis, float position)>`
  - [x] Add `setMotionCompleteCallback()` virtual method
  - [x] Add Doxygen documentation for interface contract
  - [x] **CRITICAL**: Interface must be header-only, no implementation

- [x] **Task 2: Create AxisState enum and AxisConfig struct** (AC: 11, 16)
  - [x] Define `AxisState` enum in `motor_types.h` or `i_motor.h`:
    ```cpp
    typedef enum {
        AXIS_STATE_DISABLED,    // Motor disabled, no motion possible
        AXIS_STATE_IDLE,        // Enabled, not moving
        AXIS_STATE_MOVING,      // Active motion in progress
        AXIS_STATE_ERROR,       // Error condition
        AXIS_STATE_UNHOMED      // Power-on state, position unknown
    } AxisState;
    ```
  - [x] Define `AxisConfig` struct with: `pulses_per_rev`, `units_per_rev`, `is_rotary`, `limit_min`, `limit_max`, `max_velocity`, `max_acceleration`, `backlash`, `home_offset`, `alias[LIMIT_ALIAS_MAX_LENGTH + 1]`
  - [x] Add `getPulsesPerUnit()` helper: `return pulses_per_rev / units_per_rev`

- [x] **Task 3: Implement ServoMotor class** (AC: 2-15)
  - [x] Create `firmware/components/motor/include/servo_motor.h` with class declaration
  - [x] Create `firmware/components/motor/servo_motor.cpp` with implementation
  - [x] Constructor takes: `IPulseGenerator*`, `IPositionTracker*`, `ShiftRegisterController*`, `uint8_t axis_id`, `AxisConfig config`
  - [x] Store `pulse_count_` as `std::atomic<int64_t>` — single source of truth for position
  - [x] Derive `current_position_` from `pulse_count_ / pulses_per_unit`
  - [x] Implement `init()` — validate dependencies, set initial state to `AXIS_STATE_UNHOMED`

- [x] **Task 4: Implement position moves** (AC: 3, 4, 14, 15)
  - [x] `moveAbsolute(float position, float velocity)`:
    - Validate state is not `DISABLED` → return `ESP_ERR_INVALID_STATE`
    - Validate position against limits → return `ESP_ERR_INVALID_ARG`
    - Calculate direction: `forward = (position > current_position_)`
    - Set direction via `shift_reg_->setDirection(axis_id_, forward)`
    - Wait `TIMING_DIR_SETUP_US` (from `config_timing.h`)
    - Calculate pulse delta: `(position - current_position_) * pulses_per_unit`
    - Calculate frequency: `velocity * pulses_per_unit`
    - Call `pulse_gen_->startMove(pulses, frequency, acceleration)`
    - Set `state_ = AXIS_STATE_MOVING`
  - [x] `moveRelative(float delta, float velocity)`:
    - Calculate target: `current_position_ + delta`
    - Delegate to `moveAbsolute(target, velocity)`

- [x] **Task 5: Implement velocity mode** (AC: 5)
  - [x] `moveVelocity(float velocity)`:
    - Validate state is not `DISABLED`
    - Clamp velocity to `±config_.max_velocity`
    - Set direction based on velocity sign
    - Wait `TIMING_DIR_SETUP_US`
    - Call `pulse_gen_->startVelocity(velocity_hz, config_.max_acceleration)`
    - Set `state_ = AXIS_STATE_MOVING`
  - [x] Note: Soft limit checking during velocity mode is deferred to Story 3.10

- [x] **Task 6: Implement stop methods** (AC: 6, 7)
  - [x] `stop()`:
    - Call `pulse_gen_->stop(config_.max_acceleration)`
    - State transitions to `IDLE` via completion callback
  - [x] `stopImmediate()`:
    - Call `pulse_gen_->stopImmediate()`
    - Set `state_ = AXIS_STATE_IDLE` immediately

- [x] **Task 7: Implement enable/disable** (AC: 8, 9)
  - [x] `enable(bool en)`:
    - If `en == true`:
      - Call `shift_reg_->setEnable(axis_id_, true)`
      - Wait `TIMING_ENABLE_DELAY_US`
      - Set `state_ = AXIS_STATE_IDLE` (or `UNHOMED` if not homed)
    - If `en == false`:
      - If moving, call `stopImmediate()`
      - Call `shift_reg_->setEnable(axis_id_, false)`
      - Set `state_ = AXIS_STATE_DISABLED`

- [x] **Task 8: Implement status methods** (AC: 10, 11)
  - [x] `getPosition()`: Return `pulse_count_.load() / pulses_per_unit`
  - [x] `getVelocity()`: Delegate to `pulse_gen_->getCurrentVelocity() / pulses_per_unit`
  - [x] `isMoving()`: Return `state_ == AXIS_STATE_MOVING`
  - [x] `isEnabled()`: Return `state_ != AXIS_STATE_DISABLED`
  - [x] `getState()`: Return `state_`

- [x] **Task 9: Implement motion completion callback** (AC: 12)
  - [x] Register completion callback with pulse generator in `init()`
  - [x] In callback:
    - Update `pulse_count_` with final pulse count
    - Set `state_ = AXIS_STATE_IDLE`
    - If `motion_complete_cb_` registered, invoke with axis and position

- [x] **Task 10: Implement motion blending** (AC: 13)
  - [x] In `moveAbsolute()`, if already moving:
    - Do NOT return error
    - Call `pulse_gen_->startMove()` with new target — profile generator handles blend
  - [x] Per architecture: "Blend to new target on mid-motion MOVE"

- [x] **Task 11: Update motor component CMakeLists.txt** (AC: 1-16)
  - [x] Create `firmware/components/motor/CMakeLists.txt`
  - [x] Add source files: `servo_motor.cpp`
  - [x] Add REQUIRES: `config`, `pulse_gen`, `position`, `tpic6b595`
  - [x] Add INCLUDE_DIRS: `include`

- [x] **Task 12: Create unit tests** (AC: 1-16)
  - [x] Create `firmware/components/motor/test/test_servo_motor.cpp`
  - [x] Test IMotor interface implementation
  - [x] Test `moveAbsolute()` with mock pulse generator
  - [x] Test `moveRelative()` delegates correctly
  - [x] Test `moveVelocity()` invokes `startVelocity()`
  - [x] Test limit validation rejects out-of-bounds positions
  - [x] Test disabled axis rejects motion commands
  - [x] Test `enable()`/`disable()` state transitions
  - [x] Test motion completion callback invocation
  - [x] Test motion blending (mid-motion re-target)
  - [x] **VERIFY**: All test values use named constants from config headers

- [x] **Task 13: Code Review - No Magic Numbers** (AC: 16)
  - [x] Review all .cpp files for hardcoded numeric values
  - [x] Verify all timing values from `config_timing.h`
  - [x] Verify all limit values from `config_limits.h`
  - [x] Verify all shift register references from `config_sr.h`
  - [x] Run grep check for magic numbers

## Dev Notes

### Architecture Constraints

> **Pulse Count Authority (from Tech Spec)**
>
> `pulse_count_` is the single source of truth for position. `current_position_` is derived from `pulse_count_ / pulses_per_unit`. All position modifications go through `pulse_count_` first.
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

> **SI Units Convention (from Tech Spec)**
>
> All external interfaces use SI units (meters, radians, seconds). Internal pulse domain uses pulses/pulses-per-second. Conversion happens in motor layer only.
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

> **Motion Blending (from Architecture)**
>
> New MOVE commands during active motion blend to new target. No "axis busy" errors. Profile generator recalculates on-the-fly.
>
> [Source: docs/architecture.md#Behavioral-Decisions]

> **Header-Only Configuration (from Tech Spec)**
>
> Every configurable value MUST be defined in a header file. No magic numbers in source code.
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

### Core Concepts

**Composition Pattern:**
```cpp
class ServoMotor : public IMotor {
private:
    IPulseGenerator* pulse_gen_;        // Injected: RMT or MCPWM
    IPositionTracker* position_tracker_; // Injected: Software or PCNT
    ShiftRegisterController* shift_reg_; // Shared: DIR/EN/BRAKE control
    uint8_t axis_id_;                    // X=0, Y=1, Z=2, A=3, B=4
    AxisConfig config_;                  // Per-axis configuration
    std::atomic<int64_t> pulse_count_;   // Position authority
    std::atomic<AxisState> state_;       // Current state
};
```

**Unit Conversion:**
```cpp
// External → Internal (SI to pulses)
float pulses_per_unit = config_.pulses_per_rev / config_.units_per_rev;
int32_t pulses = static_cast<int32_t>((target_pos - current_pos) * pulses_per_unit);
float frequency = velocity * pulses_per_unit;

// Internal → External (pulses to SI)
float position = pulse_count_.load() / pulses_per_unit;
```

**Axis Assignment:**
| Axis | ID | Pulse Generator | Position Tracker |
|------|----|-----------------|------------------|
| X | 0 | RmtPulseGenerator (CH0) | SoftwareTracker |
| Y | 1 | McpwmPulseGenerator (Timer0) | PcntTracker |
| Z | 2 | RmtPulseGenerator (CH1) | SoftwareTracker |
| A | 3 | RmtPulseGenerator (CH2) | SoftwareTracker |
| B | 4 | RmtPulseGenerator (CH3) | SoftwareTracker |

### Project Structure Notes

**Files to Create:**
- `firmware/components/motor/include/i_motor.h` - IMotor interface
- `firmware/components/motor/include/motor_types.h` - AxisState, AxisConfig
- `firmware/components/motor/include/servo_motor.h` - ServoMotor class
- `firmware/components/motor/servo_motor.cpp` - ServoMotor implementation
- `firmware/components/motor/CMakeLists.txt` - Component build config
- `firmware/components/motor/test/test_servo_motor.cpp` - Unit tests

**Config Files Referenced:**
- `firmware/components/config/include/config_timing.h` - TIMING_DIR_SETUP_US, TIMING_ENABLE_DELAY_US
- `firmware/components/config/include/config_limits.h` - LIMIT_ALIAS_MAX_LENGTH
- `firmware/components/config/include/config_sr.h` - SR_X_DIR, SR_X_EN, etc.
- `firmware/components/config/include/config_defaults.h` - DEFAULT_MAX_VELOCITY, etc.

### Learnings from Previous Story

**From Story 3-5-position-tracker-interface (Status: done)**

- **IPositionTracker Interface**: Available at `firmware/components/position/include/i_position_tracker.h` with `init()`, `reset()`, `getPosition()`, `setDirection()` methods.
- **SoftwareTracker**: For X, Z, A, B axes. Uses `addPulses()` method — already integrated with pulse generators via `setPositionTracker()`.
- **PcntTracker**: For Y, C, D axes. Hardware PCNT provides real-time position. D axis uses GPIO internal loopback (Story 3.5c).
- **Position Tracker Integration**: All pulse generators have `setPositionTracker()` method from Story 3.5b.
- **Atomic Pattern**: Use `std::atomic<int64_t>` for thread-safe pulse count, `std::atomic<AxisState>` for state.
- **Config Constants**: All PCNT/timing constants in respective config headers — follow same pattern.

**Files to Integrate With:**
- `firmware/components/pulse_gen/include/i_pulse_generator.h` - IPulseGenerator interface
- `firmware/components/position/include/i_position_tracker.h` - IPositionTracker interface
- `firmware/components/tpic6b595/include/shift_register_controller.h` - ShiftRegisterController

[Source: docs/sprint-artifacts/3-5-position-tracker-interface.md#Completion-Notes-List]

### References

- [Source: docs/epics.md#Story-3.6] - Story definition and acceptance criteria
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#APIs-and-Interfaces] - IMotor interface definition
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Services-and-Modules] - ServoMotor module
- [Source: docs/architecture.md#Motor-Control-Pattern] - Composition pattern, unit conversion
- [Source: docs/architecture.md#Behavioral-Decisions] - Motion blending decision (#6)

---

## Dev Agent Record

### Context Reference

- `docs/sprint-artifacts/stories/3-6-motor-base-class-servo-motor.context.xml`

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

- Implementation followed architecture constraints: pulse_count_ as single source of truth
- SI unit conversion happens in motor layer only (pulses_per_unit = pulses_per_rev / units_per_rev)
- Used composition pattern with injected IPulseGenerator and IPositionTracker
- Motion blending implemented by allowing startMove() during active motion (no axis busy error)
- All timing values from config_timing.h (TIMING_DIR_SETUP_US, TIMING_ENABLE_DELAY_US)
- All default values from config_defaults.h

### Completion Notes List

- **IMotor interface**: Header-only interface at `i_motor.h` with all required methods
- **AxisState & AxisConfig**: Defined in `motor_types.h` with createDefaultLinear/Rotary helpers
- **ServoMotor implementation**: Full implementation with composition pattern
- **Unit conversion**: velocityToFrequency() and accelerationToPulses() methods
- **State machine**: DISABLED → UNHOMED → IDLE → MOVING with proper transitions
- **Thread safety**: std::atomic for pulse_count_ and state_
- **Unit tests**: Comprehensive mock-based tests covering all ACs
- **Build verified**: Compiles successfully with no warnings

### File List

**New Files:**
- `firmware/components/motor/include/i_motor.h` - IMotor interface (header-only)
- `firmware/components/motor/include/motor_types.h` - AxisState enum, AxisConfig struct
- `firmware/components/motor/include/servo_motor.h` - ServoMotor class declaration
- `firmware/components/motor/servo_motor.cpp` - ServoMotor implementation
- `firmware/components/motor/test/test_servo_motor.cpp` - Unit tests with mocks

**Modified Files:**
- `firmware/components/motor/CMakeLists.txt` - Updated with source files and dependencies

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-05 | SM Agent (Bob) | Initial story draft for motor base class and servo motor implementation |
| 2025-12-06 | Dev Agent (Amelia) | Implemented IMotor interface, AxisState/AxisConfig, ServoMotor class, and unit tests |
| 2025-12-06 | Dev Agent (Amelia) | Senior Developer Review notes appended |

---

## Senior Developer Review (AI)

### Review Metadata
- **Reviewer:** Sergey
- **Date:** 2025-12-06
- **Review Outcome:** ✅ **APPROVE**

### Summary

Comprehensive systematic review of Story 3.6 Motor Base Class & Servo Motor implementation. All 16 acceptance criteria are fully implemented with evidence. All 13 tasks marked complete have been verified. The implementation follows architecture constraints, uses proper configuration headers (no magic numbers), and includes comprehensive unit tests with mocks. Code quality is excellent with proper thread safety, clean interfaces, and good documentation.

### Acceptance Criteria Coverage

| AC# | Description | Status | Evidence |
|-----|-------------|--------|----------|
| AC1 | IMotor interface defined with unified API | IMPLEMENTED | `i_motor.h:41-241` - all required methods present |
| AC2 | ServoMotor construction with dependency validation | IMPLEMENTED | `servo_motor.h:61-64`, `servo_motor.cpp:37-99` |
| AC3 | moveAbsolute with limit validation, unit conversion, direction | IMPLEMENTED | `servo_motor.cpp:102-169` |
| AC4 | moveRelative delegates to moveAbsolute | IMPLEMENTED | `servo_motor.cpp:171-176` |
| AC5 | moveVelocity with velocity clamping and startVelocity | IMPLEMENTED | `servo_motor.cpp:178-225` |
| AC6 | stop() with controlled deceleration | IMPLEMENTED | `servo_motor.cpp:227-247` |
| AC7 | stopImmediate() halts without decel | IMPLEMENTED | `servo_motor.cpp:249-265` |
| AC8 | enable(true) sets EN, waits delay, transitions state | IMPLEMENTED | `servo_motor.cpp:294-322` |
| AC9 | enable(false) stops motion, clears EN, transitions to DISABLED | IMPLEMENTED | `servo_motor.cpp:323-345` |
| AC10 | getPosition() returns SI units | IMPLEMENTED | `servo_motor.cpp:267-272` |
| AC11 | getState() returns valid AxisState | IMPLEMENTED | `motor_types.h:30-36`, `servo_motor.cpp:350-353` |
| AC12 | Motion completion callback invoked | IMPLEMENTED | `servo_motor.cpp:391-408` |
| AC13 | Motion blending (no axis busy error) | IMPLEMENTED | `servo_motor.cpp:105-109` - only rejects DISABLED |
| AC14 | Limit validation rejects out-of-bounds | IMPLEMENTED | `servo_motor.cpp:111-116` |
| AC15 | Disabled axis rejects motion commands | IMPLEMENTED | `servo_motor.cpp:105-109` |
| AC16 | No hardcoded values | IMPLEMENTED | Uses `config_timing.h`, `config_defaults.h`, `config_limits.h` |

**Summary: 16 of 16 acceptance criteria fully implemented**

### Task Completion Validation

| Task | Marked As | Verified As | Evidence |
|------|-----------|-------------|----------|
| Task 1: Create IMotor interface | Complete ✓ | VERIFIED | `i_motor.h` exists with all methods |
| Task 2: Create AxisState/AxisConfig | Complete ✓ | VERIFIED | `motor_types.h:30-163` |
| Task 3: Implement ServoMotor class | Complete ✓ | VERIFIED | `servo_motor.h`, `servo_motor.cpp` |
| Task 4: Implement position moves | Complete ✓ | VERIFIED | `servo_motor.cpp:102-176` |
| Task 5: Implement velocity mode | Complete ✓ | VERIFIED | `servo_motor.cpp:178-225` |
| Task 6: Implement stop methods | Complete ✓ | VERIFIED | `servo_motor.cpp:227-265` |
| Task 7: Implement enable/disable | Complete ✓ | VERIFIED | `servo_motor.cpp:294-348` |
| Task 8: Implement status methods | Complete ✓ | VERIFIED | `servo_motor.cpp:267-292, 350-358` |
| Task 9: Implement motion completion callback | Complete ✓ | VERIFIED | `servo_motor.cpp:386-408` |
| Task 10: Implement motion blending | Complete ✓ | VERIFIED | moveAbsolute allows re-entry |
| Task 11: Update CMakeLists.txt | Complete ✓ | VERIFIED | `CMakeLists.txt:1-13` |
| Task 12: Create unit tests | Complete ✓ | VERIFIED | `test_servo_motor.cpp` (908 lines) |
| Task 13: Code review - no magic numbers | Complete ✓ | VERIFIED | grep confirmed only computational constants |

**Summary: 13 of 13 completed tasks verified, 0 questionable, 0 false completions**

### Test Coverage and Gaps

- **Unit tests**: Comprehensive mock-based tests in `test_servo_motor.cpp`
- **AC Coverage**: Tests tagged with `[ACx]` markers covering all 16 ACs
- **Mock implementations**: `MockPulseGenerator`, `MockPositionTracker` with full interface coverage
- **Edge cases**: Zero-distance move, velocity when not moving, state transitions

No test gaps identified.

### Architectural Alignment

- ✅ **Pulse count as single source of truth**: `std::atomic<int64_t> pulse_count_` at `servo_motor.h:151`
- ✅ **SI units convention**: All external APIs use meters/radians, conversion in motor layer
- ✅ **Composition pattern**: Injected `IPulseGenerator*` and `IPositionTracker*`
- ✅ **Motion blending**: No "axis busy" error on mid-motion commands
- ✅ **Thread safety**: Atomic operations for `pulse_count_` and `state_`
- ✅ **Header-only config**: All values from `config_timing.h`, `config_defaults.h`, `config_limits.h`

### Security Notes

No security concerns. This is a hardware abstraction layer with no external input parsing or network interfaces.

### Best-Practices and References

- ESP-IDF v5.x patterns followed correctly
- FreeRTOS-safe atomic operations
- Proper use of `esp_rom_delay_us()` for timing
- Clean dependency injection pattern for testability

### Action Items

**Advisory Notes:**
- Note: The `enable()` method transitions to `AXIS_STATE_UNHOMED` after enable (when not homed), which is more nuanced than AC8's simple "IDLE" statement. This is intentional and correct behavior - tests explicitly accept both states.
- Note: Consider adding integration tests with real hardware in future stories
