# Story 3.6: Motor Base Class & Servo Motor

Status: ready-for-dev

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

- [ ] **Task 1: Create IMotor interface** (AC: 1, 16)
  - [ ] Create `firmware/components/motor/include/i_motor.h` with interface declaration
  - [ ] Define virtual methods: `moveAbsolute()`, `moveRelative()`, `moveVelocity()`, `stop()`, `stopImmediate()`, `getPosition()`, `getVelocity()`, `isMoving()`, `isEnabled()`, `enable()`, `getState()`, `getConfig()`, `setConfig()`
  - [ ] Include `AxisState` enum definition
  - [ ] Include `AxisConfig` struct definition (or reference from config)
  - [ ] Add `using MotionCompleteCallback = std::function<void(uint8_t axis, float position)>`
  - [ ] Add `setMotionCompleteCallback()` virtual method
  - [ ] Add Doxygen documentation for interface contract
  - [ ] **CRITICAL**: Interface must be header-only, no implementation

- [ ] **Task 2: Create AxisState enum and AxisConfig struct** (AC: 11, 16)
  - [ ] Define `AxisState` enum in `motor_types.h` or `i_motor.h`:
    ```cpp
    typedef enum {
        AXIS_STATE_DISABLED,    // Motor disabled, no motion possible
        AXIS_STATE_IDLE,        // Enabled, not moving
        AXIS_STATE_MOVING,      // Active motion in progress
        AXIS_STATE_ERROR,       // Error condition
        AXIS_STATE_UNHOMED      // Power-on state, position unknown
    } AxisState;
    ```
  - [ ] Define `AxisConfig` struct with: `pulses_per_rev`, `units_per_rev`, `is_rotary`, `limit_min`, `limit_max`, `max_velocity`, `max_acceleration`, `backlash`, `home_offset`, `alias[LIMIT_ALIAS_MAX_LENGTH + 1]`
  - [ ] Add `getPulsesPerUnit()` helper: `return pulses_per_rev / units_per_rev`

- [ ] **Task 3: Implement ServoMotor class** (AC: 2-15)
  - [ ] Create `firmware/components/motor/include/servo_motor.h` with class declaration
  - [ ] Create `firmware/components/motor/servo_motor.cpp` with implementation
  - [ ] Constructor takes: `IPulseGenerator*`, `IPositionTracker*`, `ShiftRegisterController*`, `uint8_t axis_id`, `AxisConfig config`
  - [ ] Store `pulse_count_` as `std::atomic<int64_t>` — single source of truth for position
  - [ ] Derive `current_position_` from `pulse_count_ / pulses_per_unit`
  - [ ] Implement `init()` — validate dependencies, set initial state to `AXIS_STATE_UNHOMED`

- [ ] **Task 4: Implement position moves** (AC: 3, 4, 14, 15)
  - [ ] `moveAbsolute(float position, float velocity)`:
    - Validate state is not `DISABLED` → return `ESP_ERR_INVALID_STATE`
    - Validate position against limits → return `ESP_ERR_INVALID_ARG`
    - Calculate direction: `forward = (position > current_position_)`
    - Set direction via `shift_reg_->setDirection(axis_id_, forward)`
    - Wait `TIMING_DIR_SETUP_US` (from `config_timing.h`)
    - Calculate pulse delta: `(position - current_position_) * pulses_per_unit`
    - Calculate frequency: `velocity * pulses_per_unit`
    - Call `pulse_gen_->startMove(pulses, frequency, acceleration)`
    - Set `state_ = AXIS_STATE_MOVING`
  - [ ] `moveRelative(float delta, float velocity)`:
    - Calculate target: `current_position_ + delta`
    - Delegate to `moveAbsolute(target, velocity)`

- [ ] **Task 5: Implement velocity mode** (AC: 5)
  - [ ] `moveVelocity(float velocity)`:
    - Validate state is not `DISABLED`
    - Clamp velocity to `±config_.max_velocity`
    - Set direction based on velocity sign
    - Wait `TIMING_DIR_SETUP_US`
    - Call `pulse_gen_->startVelocity(velocity_hz, config_.max_acceleration)`
    - Set `state_ = AXIS_STATE_MOVING`
  - [ ] Note: Soft limit checking during velocity mode is deferred to Story 3.10

- [ ] **Task 6: Implement stop methods** (AC: 6, 7)
  - [ ] `stop()`:
    - Call `pulse_gen_->stop(config_.max_acceleration)`
    - State transitions to `IDLE` via completion callback
  - [ ] `stopImmediate()`:
    - Call `pulse_gen_->stopImmediate()`
    - Set `state_ = AXIS_STATE_IDLE` immediately

- [ ] **Task 7: Implement enable/disable** (AC: 8, 9)
  - [ ] `enable(bool en)`:
    - If `en == true`:
      - Call `shift_reg_->setEnable(axis_id_, true)`
      - Wait `TIMING_ENABLE_DELAY_US`
      - Set `state_ = AXIS_STATE_IDLE` (or `UNHOMED` if not homed)
    - If `en == false`:
      - If moving, call `stopImmediate()`
      - Call `shift_reg_->setEnable(axis_id_, false)`
      - Set `state_ = AXIS_STATE_DISABLED`

- [ ] **Task 8: Implement status methods** (AC: 10, 11)
  - [ ] `getPosition()`: Return `pulse_count_.load() / pulses_per_unit`
  - [ ] `getVelocity()`: Delegate to `pulse_gen_->getCurrentVelocity() / pulses_per_unit`
  - [ ] `isMoving()`: Return `state_ == AXIS_STATE_MOVING`
  - [ ] `isEnabled()`: Return `state_ != AXIS_STATE_DISABLED`
  - [ ] `getState()`: Return `state_`

- [ ] **Task 9: Implement motion completion callback** (AC: 12)
  - [ ] Register completion callback with pulse generator in `init()`
  - [ ] In callback:
    - Update `pulse_count_` with final pulse count
    - Set `state_ = AXIS_STATE_IDLE`
    - If `motion_complete_cb_` registered, invoke with axis and position

- [ ] **Task 10: Implement motion blending** (AC: 13)
  - [ ] In `moveAbsolute()`, if already moving:
    - Do NOT return error
    - Call `pulse_gen_->startMove()` with new target — profile generator handles blend
  - [ ] Per architecture: "Blend to new target on mid-motion MOVE"

- [ ] **Task 11: Update motor component CMakeLists.txt** (AC: 1-16)
  - [ ] Create `firmware/components/motor/CMakeLists.txt`
  - [ ] Add source files: `servo_motor.cpp`
  - [ ] Add REQUIRES: `config`, `pulse_gen`, `position`, `tpic6b595`
  - [ ] Add INCLUDE_DIRS: `include`

- [ ] **Task 12: Create unit tests** (AC: 1-16)
  - [ ] Create `firmware/components/motor/test/test_servo_motor.cpp`
  - [ ] Test IMotor interface implementation
  - [ ] Test `moveAbsolute()` with mock pulse generator
  - [ ] Test `moveRelative()` delegates correctly
  - [ ] Test `moveVelocity()` invokes `startVelocity()`
  - [ ] Test limit validation rejects out-of-bounds positions
  - [ ] Test disabled axis rejects motion commands
  - [ ] Test `enable()`/`disable()` state transitions
  - [ ] Test motion completion callback invocation
  - [ ] Test motion blending (mid-motion re-target)
  - [ ] **VERIFY**: All test values use named constants from config headers

- [ ] **Task 13: Code Review - No Magic Numbers** (AC: 16)
  - [ ] Review all .cpp files for hardcoded numeric values
  - [ ] Verify all timing values from `config_timing.h`
  - [ ] Verify all limit values from `config_limits.h`
  - [ ] Verify all shift register references from `config_sr.h`
  - [ ] Run grep check for magic numbers

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

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List

### File List

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-05 | SM Agent (Bob) | Initial story draft for motor base class and servo motor implementation |
