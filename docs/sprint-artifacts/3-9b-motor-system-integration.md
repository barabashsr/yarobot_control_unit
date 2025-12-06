# Story 3.9b: Motor System Integration & Initialization

Status: ready-for-review

## Story

As a **developer**,
I want **the motor system fully wired and initialized at startup**,
so that **MOVE/MOVR commands can control actual hardware and motion_tasks run real-time trajectory generation**.

## Acceptance Criteria

1. **AC1:** Given system boots, when `motor_system_init()` completes, then all 8 motors are instantiated and initialized
2. **AC2:** Given motor system is initialized, when MotionController receives moveAbsolute(), then the correct motor's pulse generator executes the move
3. **AC3:** Given app_main() runs, when tasks start, then each `motion_task(axis)` has access to its corresponding IMotor instance
4. **AC4:** Given X axis, when MoveHandler processes `MOVE X 100`, then ServoMotor[X] uses RmtPulseGenerator with GPIO_X_STEP
5. **AC5:** Given Y axis, when motion executes, then ServoMotor[Y] uses McpwmPulseGenerator with GPIO_Y_STEP and PCNT feedback
6. **AC6:** Given C axis (stepper), when motion executes, then StepperMotor[C] uses McpwmPulseGenerator with GPIO_C_STEP
7. **AC7:** Given D axis (stepper), when motion executes, then StepperMotor[D] uses LedcPulseGenerator with GPIO_D_STEP and PCNT via GPIO loopback
8. **AC8:** Given E axis (discrete), when motion executes, then DiscreteAxis uses shift register for DIR/EN control
9. **AC9:** Given CommandDispatcher is initialized, when system starts, then MoveHandler and MovrHandler are registered with CMD_MOVE and CMD_MOVR
10. **AC10:** Given motion_task runs, when motor state is MOVING, then task executes trajectory profile updates at high frequency (configurable tick rate)
11. **AC11:** Given motion_task runs, when motor state is IDLE, then task blocks on notification queue (no CPU waste)
12. **AC12:** Given all pulse generators, when instantiated, then they use GPIOs from `config_gpio.h` (GPIO_X_STEP, etc.)
13. **AC13:** Given all position trackers, when instantiated, then they use PCNT units from `config_peripherals.h`
14. **AC14 (MANDATORY):** Given any implementation code, when reviewed, then NO hardcoded numeric values exist; ALL configuration values MUST come from config headers

## Tasks / Subtasks

- [ ] **Task 1: Create motor_system module declaration** (AC: 1, 14)
  - [ ] Create `firmware/components/control/motor_system/include/motor_system.h`
  - [ ] Declare `esp_err_t motor_system_init(void)` - master initialization function
  - [ ] Declare `MotionController* motor_system_get_controller(void)` - accessor for handlers
  - [ ] Declare `IMotor* motor_system_get_motor(uint8_t axis_id)` - accessor for motion_tasks
  - [ ] Declare `void motor_system_notify_task(uint8_t axis_id)` - wake motion_task
  - [ ] Add Doxygen documentation

- [ ] **Task 2: Instantiate pulse generators** (AC: 4, 5, 6, 7, 12, 14)
  - [ ] Create `firmware/components/control/motor_system/motor_system.cpp`
  - [ ] Instantiate RmtPulseGenerator for X, Z, A, B axes using GPIO_X_STEP, GPIO_Z_STEP, GPIO_A_STEP, GPIO_B_STEP
  - [ ] Instantiate McpwmPulseGenerator for Y axis using GPIO_Y_STEP with MCPWM timer 0
  - [ ] Instantiate McpwmPulseGenerator for C axis using GPIO_C_STEP with MCPWM timer 1
  - [ ] Instantiate LedcPulseGenerator for D axis using GPIO_D_STEP with LEDC channel 0
  - [ ] Use static allocation or singleton pattern for lifetime management
  - [ ] **CRITICAL**: All GPIO numbers from `config_gpio.h`, peripheral assignments from `config_peripherals.h`

- [ ] **Task 3: Instantiate position trackers** (AC: 5, 7, 13, 14)
  - [ ] Instantiate PcntTracker for Y axis (MCPWM internal routing)
  - [ ] Instantiate SoftwareTracker for X, Z, A, B axes (updated via RMT callback)
  - [ ] Instantiate PcntTracker for C axis (MCPWM internal routing)
  - [ ] Instantiate PcntTracker for D axis (GPIO loopback from LEDC output)
  - [ ] Instantiate TimeTracker for E axis (discrete - time-based position)
  - [ ] Wire trackers to pulse generators via `setPositionTracker()`
  - [ ] **CRITICAL**: PCNT unit assignments from `config_peripherals.h`

- [ ] **Task 4: Create axis configuration instances** (AC: 1, 14)
  - [ ] Define static AxisConfig array for 8 axes
  - [ ] Initialize with defaults from `config_defaults.h`:
    - `DEFAULT_PULSES_PER_REV`, `DEFAULT_UNITS_PER_REV`
    - `DEFAULT_MAX_VELOCITY`, `DEFAULT_MAX_ACCELERATION`
    - `LIMIT_POSITION_MIN`, `LIMIT_POSITION_MAX`
  - [ ] Mark servo axes (X-B) as `is_rotary = false` (linear)
  - [ ] Mark stepper axes (C-D) appropriately per application
  - [ ] Mark discrete axis (E) with `limit_min = 0.0`, `limit_max = 1.0`

- [ ] **Task 5: Instantiate motor objects** (AC: 1, 4, 5, 6, 7, 8, 14)
  - [ ] Create ServoMotor instances for X, Y, Z, A, B with:
    - Corresponding pulse generator
    - Corresponding position tracker
    - Axis ID from `config_axes.h` (AXIS_X, AXIS_Y, etc.)
    - Axis configuration from Task 4
  - [ ] Create StepperMotor instances for C, D with:
    - Corresponding pulse generator
    - Corresponding position tracker (PCNT-based)
    - Axis ID and configuration
  - [ ] Create DiscreteAxis instance for E with:
    - Shift register reference (sr_set_direction, sr_set_enable)
    - Time tracker
    - Axis ID AXIS_E
  - [ ] Store all motors in `IMotor* motors_[LIMIT_NUM_AXES]` array

- [ ] **Task 6: Initialize MotionController** (AC: 2, 9)
  - [ ] Call `MotionController::init(motors_)` with motor array
  - [ ] Register motion complete callbacks on MotionController
  - [ ] Store MotionController instance for `motor_system_get_controller()`

- [ ] **Task 7: Register command handlers** (AC: 9)
  - [ ] Include `move_handler.h` and `movr_handler.h`
  - [ ] Call `move_handler_register(dispatcher, motion_controller)` (or equivalent)
  - [ ] Call `movr_handler_register(dispatcher, motion_controller)` (or equivalent)
  - [ ] **CRITICAL**: Registration uses CMD_MOVE, CMD_MOVR from `config_commands.h`

- [ ] **Task 8: Implement motion_task real-time loop** (AC: 3, 10, 11)
  - [ ] Replace stub in `task_stubs.c` with real implementation
  - [ ] Create task notification queue/semaphore per axis
  - [ ] Main loop structure:
    ```cpp
    void motion_task(void* arg) {
        uint8_t axis_id = (uint8_t)(intptr_t)arg;
        IMotor* motor = motor_system_get_motor(axis_id);

        for (;;) {
            // Block until notified or timeout
            if (xTaskNotifyWait(0, ULONG_MAX, NULL, pdMS_TO_TICKS(TIMING_MOTION_TASK_IDLE_MS))) {
                // Process motion updates
                while (motor->isMoving()) {
                    // Profile generator update cycle
                    motor->updateTrajectory();  // If needed
                    vTaskDelay(pdMS_TO_TICKS(TIMING_MOTION_TASK_TICK_MS));
                }
            }
        }
    }
    ```
  - [ ] Add `motor_system_notify_task(axis)` to wake task when motion starts
  - [ ] **CRITICAL**: Tick intervals from `config_timing.h`

- [ ] **Task 9: Wire motor_system_init() into app_main()** (AC: 1, 2)
  - [ ] Add `#include "motor_system.h"` to `yarobot_control_unit.cpp`
  - [ ] Call `motor_system_init()` after hardware_init_and_verify() but before task creation
  - [ ] Handle init failure gracefully (log error, continue in degraded mode)

- [ ] **Task 10: Create motor_system CMakeLists.txt** (AC: 1)
  - [ ] Create `firmware/components/control/motor_system/CMakeLists.txt`
  - [ ] REQUIRES: `motor`, `pulse_gen`, `position`, `config`, `tpic6b595`, `event_manager`
  - [ ] Add to control component build

- [ ] **Task 11: Add timing constants to config_timing.h** (AC: 10, 11, 14)
  - [ ] Add `TIMING_MOTION_TASK_TICK_MS` (suggested: 1-5ms for smooth motion)
  - [ ] Add `TIMING_MOTION_TASK_IDLE_MS` (suggested: 100ms idle timeout)
  - [ ] Document purpose of each constant

- [ ] **Task 12: Create integration test** (AC: 1-9)
  - [ ] Create `firmware/components/control/motor_system/test/test_motor_system.cpp`
  - [ ] Test motor_system_init() succeeds
  - [ ] Test motor_system_get_motor() returns valid pointers for all axes
  - [ ] Test motor_system_get_controller() returns valid controller
  - [ ] Test MoveHandler can execute through full chain (command → controller → motor → pulse gen)
  - [ ] Use mock or stub pulse generators for unit testing

- [ ] **Task 13: Code review - No Magic Numbers** (AC: 14)
  - [ ] Review all .cpp and .h files for hardcoded numeric values
  - [ ] Verify all GPIO pins from `config_gpio.h`
  - [ ] Verify all peripheral assignments from `config_peripherals.h`
  - [ ] Verify all timing from `config_timing.h`
  - [ ] Verify all axis IDs from `config_axes.h`
  - [ ] Run grep check for hardcoded numbers

## Dev Notes

### Core Concepts

**Motor System Wiring Diagram:**

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                    motor_system.cpp                          │
                    │                                                              │
                    │  ┌──────────────────────────────────────────────────────┐   │
                    │  │ Pulse Generators (static instances)                    │   │
                    │  │  RmtPulseGen[0] → GPIO_X_STEP (GPIO_NUM_4)            │   │
                    │  │  RmtPulseGen[1] → GPIO_Z_STEP (GPIO_NUM_6)            │   │
                    │  │  RmtPulseGen[2] → GPIO_A_STEP (GPIO_NUM_7)            │   │
                    │  │  RmtPulseGen[3] → GPIO_B_STEP (GPIO_NUM_15)           │   │
                    │  │  McpwmPulseGen[0] → GPIO_Y_STEP (GPIO_NUM_5)          │   │
                    │  │  McpwmPulseGen[1] → GPIO_C_STEP (GPIO_NUM_16)         │   │
                    │  │  LedcPulseGen → GPIO_D_STEP (GPIO_NUM_17)             │   │
                    │  └──────────────────────────────────────────────────────┘   │
                    │                         ↓                                    │
                    │  ┌──────────────────────────────────────────────────────┐   │
                    │  │ Position Trackers                                      │   │
                    │  │  SoftwareTracker[X,Z,A,B] ← RMT callbacks             │   │
                    │  │  PcntTracker[Y,C] ← MCPWM internal route              │   │
                    │  │  PcntTracker[D] ← GPIO loopback                       │   │
                    │  │  TimeTracker[E] ← time-based                          │   │
                    │  └──────────────────────────────────────────────────────┘   │
                    │                         ↓                                    │
                    │  ┌──────────────────────────────────────────────────────┐   │
                    │  │ Motor Instances                                        │   │
                    │  │  ServoMotor[0-4] → X, Y, Z, A, B                       │   │
                    │  │  StepperMotor[0-1] → C, D                              │   │
                    │  │  DiscreteAxis → E                                      │   │
                    │  └──────────────────────────────────────────────────────┘   │
                    │                         ↓                                    │
                    │  ┌──────────────────────────────────────────────────────┐   │
                    │  │ MotionController                                       │   │
                    │  │  motors_[LIMIT_NUM_AXES] array                         │   │
                    │  │  moveAbsolute() / moveRelative()                       │   │
                    │  └──────────────────────────────────────────────────────┘   │
                    └─────────────────────────────────────────────────────────────┘
                                              ↓
                    ┌─────────────────────────────────────────────────────────────┐
                    │ CommandDispatcher (existing)                                 │
                    │  CMD_MOVE → MoveHandler → MotionController::moveAbsolute()  │
                    │  CMD_MOVR → MovrHandler → MotionController::moveRelative()  │
                    └─────────────────────────────────────────────────────────────┘
```

**motion_task Role:**

Each of the 8 motion_tasks (one per axis, Core 1, priority 15) provides:
1. **Real-time trajectory updates** - For motion blending and velocity mode
2. **Profile generation** - Trapezoidal velocity profile calculation
3. **Watchdog supervision** - Detect stalled motion or unexpected states
4. **CPU efficiency** - Block on notification when idle (no polling)

The pulse generators handle microsecond-level timing; motion_tasks handle millisecond-level trajectory updates.

### Architecture Constraints

> **Header-Only Configuration (from Tech Spec)**
>
> Every configurable value MUST be defined in a header file. No magic numbers in source code.
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

> **Dual-Core Separation**
>
> Core 0 handles communication and safety, Core 1 handles motion control.
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

**GPIO to Peripheral Mapping (from config_gpio.h):**

| Axis | GPIO | Pulse Gen | Position Tracker |
|------|------|-----------|------------------|
| X | GPIO_NUM_4 | RMT CH0 | SoftwareTracker |
| Y | GPIO_NUM_5 | MCPWM T0 | PcntTracker |
| Z | GPIO_NUM_6 | RMT CH1 | SoftwareTracker |
| A | GPIO_NUM_7 | RMT CH2 | SoftwareTracker |
| B | GPIO_NUM_15 | RMT CH3 | SoftwareTracker |
| C | GPIO_NUM_16 | MCPWM T1 | PcntTracker |
| D | GPIO_NUM_17 | LEDC CH0 | PcntTracker (loopback) |
| E | (shift reg) | N/A | TimeTracker |

### Learnings from Previous Story

**From Story 3-9-motion-controller-cmd-move-movr (Status: review)**

- **Integration Gap Identified**: MotionController and handlers implemented but NOT wired to motor system
- **Files Created**:
  - `motion_controller.h`, `motion_controller.cpp` - ready to use
  - `move_handler.h`, `move_handler.cpp` - ready to register
  - `movr_handler.h`, `movr_handler.cpp` - ready to register
- **IMotor Interface**: All motors implement `moveAbsolute()`, `moveRelative()`, `isEnabled()`, `getPosition()`, `getState()`
- **Motion Completion Callback**: `setMotionCompleteCallback()` pattern established
- **Error Returns**: `ESP_ERR_INVALID_STATE` for disabled axis, `ESP_ERR_INVALID_ARG` for invalid position

[Source: docs/sprint-artifacts/3-9-motion-controller-cmd-move-movr.md#Completion-Notes-List]

### Project Structure Notes

**Files to Create:**
- `firmware/components/control/motor_system/include/motor_system.h`
- `firmware/components/control/motor_system/motor_system.cpp`
- `firmware/components/control/motor_system/CMakeLists.txt`
- `firmware/components/control/motor_system/test/test_motor_system.cpp`

**Files to Modify:**
- `firmware/main/yarobot_control_unit.cpp` - Add motor_system_init() call
- `firmware/components/control/tasks/task_stubs.c` - Replace motion_task stub
- `firmware/components/config/include/config_timing.h` - Add motion task timing constants

### References

- [Source: docs/sprint-artifacts/3-9-motion-controller-cmd-move-movr.md#Completion-Notes-List] - Integration gap identification
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Services-and-Modules] - Component responsibilities
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#APIs-and-Interfaces] - IPulseGenerator, IPositionTracker
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints] - Header-only configuration
- [Source: docs/architecture.md#Behavioral-Decisions-Summary] - Dual-core separation

---

## Dev Agent Record

### Context Reference

<!-- Path(s) to story context XML will be added here by context workflow -->

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

### Completion Notes List

### File List

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-06 | SM Agent (Bob) | Initial story draft for motor system integration and initialization |
