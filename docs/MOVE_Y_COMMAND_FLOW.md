# MOVE Y 0 0.001 - Complete Execution Flow

This document describes the complete execution flow when the command `MOVE Y 0 0.001` is sent to the YaRobot Control Unit.

## Command Breakdown

| Parameter | Value | Description |
|-----------|-------|-------------|
| Command | `MOVE` | Absolute position move command |
| Axis | `Y` | Y-axis (servo motor with MCPWM) |
| Position | `0` | Target position: 0 degrees |
| Velocity | `0.001` | Velocity: 0.001 deg/s |

---

## Execution Flow Diagram

```
USB Serial Input
       │
       ▼
┌──────────────────┐
│ Command Parser   │  Parse "MOVE Y 0 0.001"
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Command Executor │  Check STATE_READY, dispatch to handler
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Move Handler     │  Validate axis, extract params
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Motion Controller│  Get motor, call moveAbsolute()
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ ServoMotor       │  Check limits, calculate pulses, set direction
│ (MotorBase)      │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ MCPWM Pulse Gen  │  Configure profile, start MCPWM timer
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Hardware Layer   │  MCPWM generates pulses, PCNT counts
│ (MCPWM + PCNT)   │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Profile Timer    │  20ms periodic: update velocity, check completion
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Motion Complete  │  Stop MCPWM, invoke callback, publish event
└──────────────────┘
```

---

## Stage 1: Command Parsing

**File:** `firmware/components/interface/command_parser/command_parser.c`

The USB serial interface receives `MOVE Y 0 0.001\r\n` and parses it into:

```c
ParsedCommand cmd = {
    .verb = "MOVE",
    .axis = 'Y',
    .param_count = 2,
    .params = {0.0f, 0.001f}  // position, velocity
};
```

---

## Stage 2: Command Executor Dispatch

**File:** `firmware/components/control/command_executor/command_executor.c`

```c
// Check system state (line 68)
static const CommandEntry s_builtin_commands[] = {
    { CMD_MOVE, handle_move, STATE_READY }  // MOVE only allowed in READY state
};

// Dispatch to registered handler
esp_err_t dispatch_command(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // 1. Find handler for "MOVE" verb
    // 2. Check is_state_allowed(STATE_READY)
    // 3. If blocked: return "ERROR E012 Command blocked in current mode"
    // 4. If allowed: call handle_move(cmd, response, resp_len)
}
```

**Important:** System must be in `STATE_READY` mode. Use `MODE READY` to enable motion commands.

---

## Stage 3: Move Handler

**File:** `firmware/components/control/command_executor/move_handler.cpp:22-86`

```cpp
esp_err_t handle_move(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // 1. Validate axis character 'Y' (line 36)
    if (!is_valid_axis(cmd->axis)) {
        return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
    }

    // 2. Extract parameters (line 48-54)
    float position = cmd->params[0];    // 0.0
    float velocity = cmd->params[1];    // 0.001

    // 3. Convert axis char to index (line 57)
    int8_t axis_id = axis_to_index('Y');  // Returns AXIS_Y = 1

    // 4. Get motion controller singleton (line 64)
    MotionController* controller = getMotionController();

    // 5. Execute move (line 71)
    esp_err_t ret = controller->moveAbsolute(axis_id, position, velocity);

    // 6. Return "OK\r\n" or error response
    if (ret == ESP_OK) {
        return format_ok(response, resp_len);  // "OK\r\n"
    }
}
```

---

## Stage 4: Motion Controller

**File:** `firmware/components/control/motion_controller/motion_controller.cpp:123-164`

```cpp
esp_err_t MotionController::moveAbsolute(uint8_t axis, float position, float velocity)
{
    // axis = 1 (AXIS_Y), position = 0.0, velocity = 0.001

    // 1. Validate axis ID (line 131)
    if (axis >= LIMIT_NUM_AXES) return ESP_ERR_INVALID_ARG;

    // 2. Get motor pointer (line 136)
    IMotor* motor = motors_[1];  // s_servo_y (ServoMotor instance)

    // 3. Check if motor is enabled (line 143)
    if (!motor->isEnabled()) {
        return ESP_ERR_INVALID_STATE;  // "ERROR E008 Axis not enabled"
    }

    // 4. Apply default velocity if needed (line 149-152)
    float move_velocity = velocity;  // 0.001 deg/s
    if (move_velocity <= 0.0f) {
        move_velocity = DEFAULT_MAX_VELOCITY;  // 1080 deg/s
    }

    // 5. Delegate to motor (line 158)
    return motor->moveAbsolute(position, move_velocity);
}
```

---

## Stage 5: ServoMotor (MotorBase)

**File:** `firmware/components/motor/motor_base.cpp:104-179`

```cpp
esp_err_t MotorBase::moveAbsolute(float position, float velocity)
{
    // position = 0.0, velocity = 0.001

    // 1. Check state (line 109-113)
    AxisState current_state = state_.load();
    if (current_state == AXIS_STATE_DISABLED) {
        return ESP_ERR_INVALID_STATE;
    }

    // 2. Validate soft limits (line 116-120)
    // config_.limit_min = -360000.0, config_.limit_max = 360000.0
    if (position < config_.limit_min || position > config_.limit_max) {
        return ESP_ERR_INVALID_ARG;  // "ERROR E007 Position limit exceeded"
    }

    // 3. Clamp velocity to max (line 123-124)
    velocity = std::min(velocity, config_.max_velocity);  // min(0.001, 1080) = 0.001

    // 4. Get current position (line 128)
    float current_pos = getPosition();  // e.g., 100.0 degrees

    // 5. Calculate delta and direction (line 132-133)
    float delta = position - current_pos;  // 0.0 - 100.0 = -100.0
    bool forward = (delta >= 0);           // false (reverse)

    // 6. Check zero-distance move (line 136-139)
    if (std::fabs(delta) < (1.0f / config_.getPulsesPerUnit())) {
        return ESP_OK;  // Already at target
    }

    // 7. Set direction via shift register (line 143)
    esp_err_t err = setDirection(forward);  // SR_Y_DIR bit = 0 (reverse)
    // Calls: sr_set_direction(AXIS_Y, false) -> TPIC6B595 shift register

    // 8. Wait direction setup time (line 150)
    waitDirectionSetup();  // 5µs delay (TIMING_DIR_SETUP_US)

    // 9. Convert to pulse domain (line 153-156)
    float pulses_per_unit = config_.getPulsesPerUnit();
    // pulses_per_unit = pulses_per_rev / units_per_rev = 200 / 360 = 0.556

    int32_t pulses = static_cast<int32_t>(std::fabs(delta) * pulses_per_unit);
    // pulses = |−100.0| * 0.556 = 55 pulses

    float freq_hz = velocityToFrequency(velocity);
    // freq_hz = velocity * pulses_per_unit = 0.001 * 0.556 = 0.000556 Hz
    // BUT: clamped to MIN (10 Hz) ... so freq_hz = 10 Hz minimum

    float accel_pulses = accelerationToPulses(config_.max_acceleration);
    // accel = 10800 deg/s² * 0.556 pulses/deg = 6000 pulses/s²

    // 10. Set position tracker direction (line 162)
    position_tracker_->setDirection(forward);

    // 11. Start pulse generation (line 167)
    err = pulse_gen_->startMove(forward ? pulses : -pulses, freq_hz, accel_pulses);
    // startMove(-55, 10.0, 6000.0)

    // 12. Update state (line 174)
    state_.store(AXIS_STATE_MOVING);

    return ESP_OK;
}
```

### Unit Conversion Formulas

```
pulses_per_unit = pulses_per_rev / units_per_rev
                = 200 / 360
                = 0.5556 pulses/degree

pulses = delta_degrees * pulses_per_unit
       = 100.0 * 0.5556
       = 55 pulses

freq_hz = velocity_deg_s * pulses_per_unit
        = 0.001 * 0.5556
        = 0.000556 Hz (clamped to minimum 10 Hz)

accel_pulses = acceleration_deg_s2 * pulses_per_unit
             = 10800 * 0.5556
             = 6000 pulses/s²
```

---

## Stage 6: MCPWM Pulse Generator

**File:** `firmware/components/pulse_gen/mcpwm_pulse_gen.cpp:727-907`

```cpp
esp_err_t McpwmPulseGenerator::startMove(int32_t pulses, float max_velocity, float acceleration)
{
    // pulses = -55, max_velocity = 10.0 Hz, acceleration = 6000.0 pulses/s²

    // 1. Handle direction (line 750-751)
    direction_ = (pulses > 0);  // false (reverse)
    int32_t abs_pulses = std::abs(pulses);  // 55

    // 2. Check if already running (line 755-780)
    McpwmProfileState current_state = state_.load();
    if (current_state != McpwmProfileState::IDLE) {
        // Update target smoothly (motion blending)
        // ... no hardware re-init
        return ESP_OK;
    }

    // 3. Set direction via shift register (line 793-803)
    if (direction_ != last_direction_) {
        uint8_t axis = SR_AXIS_Y;  // Y axis
        sr_set_direction(axis, direction_);
        sr_update();  // Clock out to TPIC6B595
        ets_delay_us(TIMING_DIR_SETUP_US);  // 5µs
        last_direction_ = direction_;
    }

    // 4. Calculate trapezoidal profile (line 806-807)
    mode_ = McpwmMotionMode::POSITION;
    calculateTrapezoidalProfile(abs_pulses, max_velocity, acceleration);

    // Trapezoidal profile calculation:
    // - accel_pulses: pulses during acceleration phase
    // - cruise_pulses: pulses at constant velocity
    // - decel_pulses: pulses during deceleration phase
    // - cruise_velocity: target velocity (Hz)

    // 5. Save starting position (line 813)
    move_start_position_ = absolute_position_.load();

    // 6. Reset counters (line 818-824)
    pulse_count_.store(0);
    overflow_count_.store(0);
    current_velocity_.store(MIN_MOTION_START_FREQ);  // 300 Hz start
    profile_.current_pulse = 0;

    // 7. Read PCNT baseline (line 828-830)
    int current_pcnt = 0;
    pcnt_unit_get_count(pcnt_unit_, &current_pcnt);
    move_start_pcnt_count_ = current_pcnt;

    // 8. Configure PCNT watch point (line 845)
    int32_t watch_point_target = move_start_pcnt_count_ + abs_pulses;
    configurePcntWatchPoint(watch_point_target);

    // 9. Set initial frequency (line 857)
    setFrequency(MIN_MOTION_START_FREQ);  // 300 Hz

    // 10. Restore generator HIGH action (line 865-870)
    mcpwm_generator_set_action_on_timer_event(gen_handle_,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                      MCPWM_TIMER_EVENT_EMPTY,
                                      MCPWM_GEN_ACTION_HIGH));

    // 11. Set state and start timer (line 882-884)
    state_.store(McpwmProfileState::ACCELERATING);
    mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_START_NO_STOP);

    // 12. Start profile update timer (line 892)
    esp_timer_start_periodic(profile_timer_, 20000);  // 20ms period

    return ESP_OK;
}
```

---

## Stage 7: Hardware Layer

### MCPWM Timer Configuration

```
MCPWM Timer Y (timer_id = 0)
├── Resolution: 100 kHz (MCPWM_TIMER_RESOLUTION_HZ)
├── Period: variable (sets pulse frequency)
├── Duty: 50% (comparator at period/2)
└── GPIO: GPIO_Y_STEP (GPIO 5)

Pulse Generation:
- Timer counts 0 → period → 0 (up-down mode)
- At count=0: GPIO goes HIGH
- At count=period/2: GPIO goes LOW
- Result: 50% duty square wave at configured frequency
```

### PCNT Counter Configuration

```
PCNT Unit Y (unit_id = 0)
├── High limit: 32767 (PCNT_HIGH_LIMIT)
├── Low limit: -32768 (PCNT_LOW_LIMIT)
├── Edge GPIO: GPIO 5 (same as MCPWM output via loopback)
├── Count mode: increment on rising edge
└── Overflow handling: ISR increments overflow_count_
```

### GPIO Loopback

```
┌─────────────┐     ┌─────────────┐
│ MCPWM Timer │────▶│   GPIO 5    │────▶ To Servo Driver
│  Generator  │     │  (Output)   │
└─────────────┘     └──────┬──────┘
                           │
                    Internal Loopback
                           │
                    ┌──────▼──────┐
                    │   GPIO 5    │
                    │  (Input)    │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │ PCNT Unit   │
                    │  Counter    │
                    └─────────────┘
```

---

## Stage 8: Profile Timer Callback (20ms periodic)

**File:** `firmware/components/pulse_gen/mcpwm_pulse_gen.cpp:436-475`

```cpp
void McpwmPulseGenerator::profileTimerCallback(void* arg)
{
    McpwmPulseGenerator* self = static_cast<McpwmPulseGenerator*>(arg);

    // 1. Read PCNT hardware count (line 450-451)
    int count = 0;
    pcnt_unit_get_count(self->pcnt_unit_, &count);

    // 2. Calculate position with overflow handling (line 453-454)
    int32_t overflows = self->overflow_count_.load();
    int32_t hw_pcnt = count + (overflows * PCNT_HIGH_LIMIT);

    // 3. Calculate position delta from last completion (line 457-459)
    int32_t pcnt_delta = hw_pcnt - self->pcnt_at_last_completion_;
    bool current_dir = self->direction_;
    int64_t position_delta = current_dir ? pcnt_delta : -pcnt_delta;

    // 4. Update absolute position (line 462-463)
    int64_t new_position = self->last_completed_position_ + position_delta;
    self->absolute_position_.store(new_position);

    // 5. Update pulse_count_ for profile tracking (line 467-468)
    int64_t current_count = hw_pcnt - self->move_start_pcnt_count_;
    self->pulse_count_.store(current_count);

    // 6. Call updateProfile() if not IDLE (line 471-474)
    McpwmProfileState current = self->state_.load();
    if (current != McpwmProfileState::IDLE) {
        self->updateProfile();
    }
}
```

---

## Stage 9: Profile Update (Trapezoidal Velocity)

**File:** `firmware/components/pulse_gen/mcpwm_pulse_gen.cpp:477-600`

```cpp
void McpwmPulseGenerator::updateProfile()
{
    // Get current pulse count
    int64_t current_count = pulse_count_.load();
    profile_.current_pulse = current_count;

    // COMPLETION CHECK (line 515-539)
    if (mode_ == McpwmMotionMode::POSITION && current_count >= profile_.target_pulses) {
        // Stop MCPWM timer
        mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_STOP_EMPTY);

        // Force GPIO LOW
        mcpwm_generator_set_action_on_timer_event(gen_handle_,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                          MCPWM_TIMER_EVENT_EMPTY,
                                          MCPWM_GEN_ACTION_LOW));

        // Calculate final position from PCNT
        int64_t actual_pulses = current_count;
        int64_t delta = direction_ ? actual_pulses : -actual_pulses;
        int64_t new_abs_pos = move_start_position_ + delta;
        absolute_position_.store(new_abs_pos);

        // Update stable reference points
        pcnt_at_last_completion_ = move_start_pcnt_count_ + current_count;
        last_completed_position_ = new_abs_pos;

        // Transition to IDLE
        state_.store(McpwmProfileState::IDLE);

        // Stop profile timer
        esp_timer_stop(profile_timer_);

        // Invoke completion callback
        if (!completion_notified_.exchange(true)) {
            if (completion_callback_) {
                completion_callback_(axis_id_, getPosition());
            }
        }
        return;
    }

    // VELOCITY PROFILE UPDATE (trapezoidal)
    // Accelerating → Cruising → Decelerating phases
    // Adjusts MCPWM frequency based on position in profile
}
```

### Trapezoidal Velocity Profile

```
Velocity
   ▲
   │      ┌────────────────┐
   │     /│                │\
   │    / │    Cruise      │ \
   │   /  │                │  \
   │  /   │                │   \
   │ /    │                │    \
   └──────┴────────────────┴─────▶ Position (pulses)
     Accel    Cruise        Decel
```

---

## Stage 10: Motion Complete Callback

**File:** `firmware/components/motor/motor_base.cpp:389-447`

```cpp
// Callback registered during motor init
pulse_gen_->setCompletionCallback(
    [this](uint8_t /*unused*/, float position) {
        onMotionComplete(position);
    }
);

void MotorBase::onMotionComplete(float position)
{
    // 1. Sync position from tracker
    syncPositionFromTracker();

    // 2. Update state to IDLE
    state_.store(AXIS_STATE_IDLE);

    // 3. Invoke user callback (registered by MotionController)
    if (motion_complete_callback_) {
        motion_complete_callback_(axis_id_, position);
    }
}
```

**File:** `firmware/components/control/motion_controller/motion_controller.cpp:335-351`

```cpp
void MotionController::onMotionComplete(uint8_t axis, float position)
{
    // Publish EVT_MOTION_COMPLETE event
    Event event = {
        .type = EVTTYPE_MOTION_COMPLETE,
        .axis = axis,               // 1 (AXIS_Y)
        .data = { .position = position },
        .timestamp = esp_timer_get_time()
    };

    event_publish(&event);  // Published to event subscribers
}
```

---

## Summary: Complete Timeline

| Step | Component | Action | Time |
|------|-----------|--------|------|
| 1 | USB Serial | Receive "MOVE Y 0 0.001\r\n" | T+0 |
| 2 | Command Parser | Parse into ParsedCommand struct | T+0.1ms |
| 3 | Command Executor | Check STATE_READY, dispatch | T+0.2ms |
| 4 | Move Handler | Validate, call controller | T+0.3ms |
| 5 | Motion Controller | Check enabled, delegate to motor | T+0.4ms |
| 6 | MotorBase | Calculate pulses, set direction | T+0.5ms |
| 7 | MCPWM PulseGen | Configure profile, start timer | T+1ms |
| 8 | Profile Timer | Update velocity every 20ms | T+21ms, T+41ms, ... |
| 9 | PCNT | Count pulses continuously | Ongoing |
| 10 | Completion | Stop timer, callback, event | T+Xms (motion dependent) |

---

## Key Files Reference

| Component | File Path |
|-----------|-----------|
| Command Parser | `firmware/components/interface/command_parser/command_parser.c` |
| Command Executor | `firmware/components/control/command_executor/command_executor.c` |
| Move Handler | `firmware/components/control/command_executor/move_handler.cpp` |
| Motion Controller | `firmware/components/control/motion_controller/motion_controller.cpp` |
| Motor Base | `firmware/components/motor/motor_base.cpp` |
| Servo Motor | `firmware/components/motor/servo_motor.cpp` |
| MCPWM Pulse Gen | `firmware/components/pulse_gen/mcpwm_pulse_gen.cpp` |
| PCNT Tracker | `firmware/components/position/pcnt_tracker.cpp` |
| Shift Register | `firmware/components/drivers/tpic6b595/tpic6b595.c` |
| Config Defaults | `firmware/components/config/include/config_defaults.h` |

---

## Configuration Constants

```c
// Axis configuration (Y axis default)
DEFAULT_PULSES_PER_REV      = 200.0f    // Pulses per motor revolution
DEFAULT_UNITS_PER_REV       = 360.0f    // Degrees per revolution
DEFAULT_MAX_VELOCITY        = 1080.0f   // deg/s (180 RPM)
DEFAULT_MAX_ACCELERATION    = 10800.0f  // deg/s²

// Hardware
GPIO_Y_STEP                 = GPIO_NUM_5
MCPWM_TIMER_Y              = 0
PCNT_UNIT_Y                = 0
MCPWM_TIMER_RESOLUTION_HZ  = 100000     // 100 kHz

// Timing
TIMING_DIR_SETUP_US        = 5          // Direction setup delay
TIMING_POSITION_UPDATE_MS  = 20         // Profile update interval

// Limits
LIMIT_MIN_PULSE_FREQ_HZ    = 10         // Minimum pulse frequency
LIMIT_MAX_PULSE_FREQ_HZ    = 100000     // Maximum pulse frequency
```
