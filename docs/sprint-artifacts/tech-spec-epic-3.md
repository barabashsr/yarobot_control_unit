# Epic Technical Specification: Motor Control Core

Date: 2025-12-04
Author: Sergey
Epic ID: 3
Status: Draft

---

## Overview

Epic 3 delivers the core motor control functionality that enables users to move all 8 motor axes (X, Y, Z, A, B, C, D, E) through unified text commands. This epic transforms the foundation established in Epics 1-2 into a working motion control system.

The implementation provides a unified abstraction layer where servo motors, stepper motors, and discrete actuators all respond identically to MOVE/MOVR/VEL/STOP commands. Under this abstraction, different pulse generation technologies (RMT+DMA for X/Z/A/B, MCPWM+PCNT for Y/C, LEDC for D, time-based for E) are hidden from the command interface.

This epic represents the core product value—after completion, users can control motors via simple commands like `MOVE X 0.100` without understanding the underlying hardware complexity.

## Objectives and Scope

**In Scope:**

- 5x TPIC6B595N shift register chain driver for DIR/EN/BRAKE/ALARM_CLR signals
- RMT pulse generator for servo axes X, Z, A, B (up to 500 kHz pulses, 200 kHz default max)
- MCPWM pulse generator with internal PCNT routing for Y, C axes
- LEDC pulse generator with software counting for D axis stepper
- Position tracking interfaces (PCNT, software, time-based)
- IMotor abstraction with ServoMotor, StepperMotor, and DiscreteAxis implementations
- Motion controller coordinating all 8 axes
- CMD_MOVE, CMD_MOVR, CMD_VEL, CMD_STOP, CMD_EN, CMD_POS commands
- Motion completion events (EVT_MOTION_COMPLETE)

**Out of Scope:**

- Limit switch monitoring and automatic stops (Epic 4)
- Brake control strategies and E-stop handling (Epic 4)
- Homing sequences (Epic 4)
- Configuration via YAML (Epic 5)
- OLED display (Epic 5)
- Z-signal synchronization and InPos confirmation (Epic 6)
- Multi-axis synchronized motion (post-MVP)
- S-curve acceleration profiles (post-MVP)

## System Architecture Alignment

**Components Referenced:**

| Component | Directory | Responsibility |
|-----------|-----------|----------------|
| tpic6b595 driver | `components/drivers/tpic6b595/` | 40-bit shift register chain via SPI |
| pulse_gen | `components/pulse_gen/` | IPulseGenerator implementations |
| position | `components/position/` | IPositionTracker implementations |
| motor | `components/motor/` | IMotor abstraction and implementations |
| control | `components/control/motion_controller/` | Motion coordination |
| command_executor | `components/control/command_executor/` | Command handlers |

**MANDATORY Architecture Constraints:**

1. **Header-Only Configuration** — Every configurable value MUST be defined in a header file. No magic numbers in source code. GPIO pins from `config_gpio.h`, timing from `config_timing.h`, limits from `config_limits.h`, commands from `config_commands.h`, shift register bits from `config_sr.h`, defaults from `config_defaults.h`.

2. **Streaming Double-Buffer Pulse Generation** — All pulse generation uses streaming double-buffer architecture. Short moves, long moves, and continuous jogging use the same infrastructure. No special-case code paths.

3. **SI Units Convention** — All external interfaces use SI units (meters, radians, seconds). Internal pulse domain uses pulses/pulses-per-second. Conversion happens in motor layer only.

4. **Motion Blending** — New MOVE commands during active motion blend to new target. No "axis busy" errors. Profile generator recalculates on-the-fly.

5. **Dual-Core Separation** — Core 0 handles communication and safety, Core 1 handles motion control.

6. **Pulse Count Authority** — `pulse_count_` is the single source of truth for position. `current_position_` is derived from `pulse_count_ / pulses_per_unit`. All position modifications go through pulse_count_ first.

## Detailed Design

### Services and Modules

| Module | Location | Inputs | Outputs | Owner Story |
|--------|----------|--------|---------|-------------|
| ShiftRegisterController | `drivers/tpic6b595/` | Axis ID, signal type, value | 40-bit SPI chain update | 3.1 |
| RmtPulseGenerator | `pulse_gen/rmt_pulse_gen.cpp` | Pulse count, velocity, accel | GPIO STEP pulses (X,Z,A,B) | 3.2 |
| McpwmPulseGenerator | `pulse_gen/mcpwm_pulse_gen.cpp` | Pulse count, velocity, accel | GPIO STEP pulses + PCNT (Y,C) | 3.3 |
| LedcPulseGenerator | `pulse_gen/ledc_pulse_gen.cpp` | Pulse count, velocity, accel | GPIO STEP pulses (D) | 3.4 |
| PcntTracker | `position/pcnt_tracker.cpp` | PCNT events | Position in pulses | 3.5 |
| SoftwareTracker | `position/software_tracker.cpp` | Callback from pulse gen | Position in pulses | 3.5 |
| TimeTracker | `position/time_tracker.cpp` | Time elapsed, known speed | Position 0.0/1.0 | 3.5 |
| ServoMotor | `motor/servo_motor.cpp` | SI position/velocity commands | Pulse gen calls | 3.6 |
| StepperMotor | `motor/stepper_motor.cpp` | SI position/velocity commands | Pulse gen + PCNT calls | 3.7 |
| DiscreteAxis | `motor/discrete_axis.cpp` | SI position (0.0/1.0) | DIR/EN via shift register | 3.8 |
| MotionController | `control/motion_controller/` | Command parse results | Motor method calls | 3.9-3.11 |
| MoveHandler | `control/command_executor/` | MOVE command | MotionController::moveAbsolute | 3.9 |
| VelHandler | `control/command_executor/` | VEL command | MotionController::moveVelocity | 3.10 |

### Data Models and Contracts

**AxisConfig Structure (stored in NVS):**

```c
struct AxisConfig {
    // Unit conversion (two fundamental parameters)
    uint32_t pulses_per_rev;      // Driver PA14 setting or gear ratio
    float units_per_rev;          // Physical travel per revolution (m or rad)
    bool is_rotary;               // true = radians, false = meters

    // Derived: pulses_per_unit = pulses_per_rev / units_per_rev

    // Soft limits (SI units)
    float limit_min;              // meters or radians
    float limit_max;              // meters or radians

    // Motion parameters (SI units)
    float max_velocity;           // m/s or rad/s
    float max_acceleration;       // m/s² or rad/s²

    // Compensation
    float backlash;               // meters or radians
    float home_offset;            // meters or radians

    // Identity
    char alias[LIMIT_ALIAS_MAX_LENGTH + 1];
};
```

**AxisState Enum:**

```c
typedef enum {
    AXIS_STATE_DISABLED,      // Motor disabled, no motion possible
    AXIS_STATE_IDLE,          // Enabled, not moving
    AXIS_STATE_MOVING,        // Active motion in progress
    AXIS_STATE_ERROR,         // Error condition (cleared by EN 0 then EN 1)
    AXIS_STATE_UNHOMED        // Power-on state, position unknown
} AxisState;
```

**Shift Register Bit Map (from config_sr.h):**

| Bits | Axis | SR_x_DIR | SR_x_EN | SR_x_BRAKE | SR_x_ALARM_CLR |
|------|------|----------|---------|------------|----------------|
| 0-3 | X | 0 | 1 | 2 | 3 |
| 4-7 | Y | 4 | 5 | 6 | 7 |
| 8-11 | Z | 8 | 9 | 10 | 11 |
| 12-15 | A | 12 | 13 | 14 | 15 |
| 16-19 | B | 16 | 17 | 18 | 19 |
| 20-23 | C | 20 | 21 | 22 | 23 |
| 24-27 | D | 24 | 25 | 26 | 27 |
| 28-31 | E | 28 | 29 | - | - |
| 32-39 | GP | SR_GP_OUT_0 - SR_GP_OUT_7 | - | - | - |

### APIs and Interfaces

**IPulseGenerator Interface:**

```cpp
class IPulseGenerator {
public:
    virtual ~IPulseGenerator() = default;

    // Motion control
    virtual esp_err_t startMove(int32_t pulses, float max_velocity, float acceleration) = 0;
    virtual esp_err_t startVelocity(float velocity, float acceleration) = 0;
    virtual esp_err_t stop(float deceleration) = 0;
    virtual void stopImmediate() = 0;

    // Status
    virtual bool isRunning() const = 0;
    virtual int64_t getPulseCount() const = 0;
    virtual float getCurrentVelocity() const = 0;

    // Callbacks
    using MotionCompleteCallback = std::function<void(int64_t total_pulses)>;
    virtual void setCompletionCallback(MotionCompleteCallback cb) = 0;
};
```

**IPositionTracker Interface:**

```cpp
class IPositionTracker {
public:
    virtual ~IPositionTracker() = default;
    virtual esp_err_t init() = 0;
    virtual esp_err_t reset(int64_t position = 0) = 0;
    virtual int64_t getPosition() const = 0;
    virtual void setDirection(bool forward) = 0;
};
```

**IMotor Interface:**

```cpp
class IMotor {
public:
    virtual ~IMotor() = default;

    // Position moves (SI units)
    virtual esp_err_t moveAbsolute(float position, float velocity) = 0;
    virtual esp_err_t moveRelative(float delta, float velocity) = 0;

    // Velocity mode (SI units)
    virtual esp_err_t moveVelocity(float velocity) = 0;

    // Stop commands
    virtual esp_err_t stop() = 0;              // Controlled deceleration
    virtual esp_err_t stopImmediate() = 0;     // Emergency stop

    // Status (SI units)
    virtual float getPosition() const = 0;
    virtual float getVelocity() const = 0;
    virtual bool isMoving() const = 0;
    virtual bool isEnabled() const = 0;
    virtual AxisState getState() const = 0;

    // Enable/disable
    virtual esp_err_t enable(bool en) = 0;
};
```

**ShiftRegisterController API:**

```c
esp_err_t sr_init(void);
esp_err_t sr_set_direction(uint8_t axis, bool forward);
esp_err_t sr_set_enable(uint8_t axis, bool enable);
esp_err_t sr_set_brake(uint8_t axis, bool release);
esp_err_t sr_set_alarm_clear(uint8_t axis, bool active);
esp_err_t sr_set_gp_output(uint8_t pin, bool level);
esp_err_t sr_update(void);                    // Latch to outputs
void sr_emergency_disable_all(void);          // ISR-safe, all LOW
uint64_t sr_get_state(void);                  // Read 40-bit state
```

**Command Protocol (from config_commands.h):**

| Command | Format | Response |
|---------|--------|----------|
| CMD_MOVE | `MOVE <axis> <position> [velocity]` | `OK` or `ERROR <code> <msg>` |
| CMD_MOVR | `MOVR <axis> <delta> [velocity]` | `OK` or `ERROR <code> <msg>` |
| CMD_VEL | `VEL <axis> <velocity>` | `OK` or `ERROR <code> <msg>` |
| CMD_STOP | `STOP [axis]` | `OK` |
| CMD_EN | `EN <axis> <0\|1>` | `OK` or `ERROR <code> <msg>` |
| CMD_POS | `POS [axis]` | `OK <axis> <position>` or all positions |

**Event Notifications:**

| Event | Format | Condition |
|-------|--------|-----------|
| EVT_MOTION_COMPLETE | `EVENT DONE <axis> <position>` | Motion completed successfully |
| EVT_MOTION_ERROR | `EVENT ERROR <axis> <code>` | Motion failed |

### Workflows and Sequencing

**Motion Command Execution Flow:**

```
Host                    CommandParser           MotionController          Motor              PulseGen
  |                           |                        |                    |                    |
  |--- "MOVE X 0.100" ------->|                        |                    |                    |
  |                           |--- parseCommand() --->|                    |                    |
  |                           |                        |--- moveAbsolute()-->|                   |
  |                           |                        |                    |--- validate limits |
  |                           |                        |                    |--- toPulses() ---->|
  |                           |                        |                    |                    |--- startMove()
  |                           |                        |                    |                    |    [PRIMING]
  |<------ "OK" ---------------|<----------------------|<------------------|                    |    [STREAMING]
  |                           |                        |                    |                    |    ...
  |                           |                        |                    |                    |    [DRAINING]
  |                           |                        |                    |<-- onComplete() ---|    [IDLE]
  |                           |                        |<-- callback -------|                    |
  |<-- "EVENT DONE X 0.100" --|<----------------------|                    |                    |
```

**Enable/Disable Sequence:**

```
1. EN X 1 received
2. sr_set_enable(0, true) called
3. TIMING_ENABLE_DELAY_US (50µs) wait
4. Axis state → IDLE
5. OK response sent

1. EN X 0 received (while moving)
2. motor->stopImmediate() called
3. sr_set_enable(0, false) called
4. Axis state → DISABLED
5. OK response sent
```

**Shift Register Update Sequence:**

```
1. Prepare 40-bit shadow register
2. Set direction bit before motion
3. TIMING_DIR_SETUP_US (20µs) wait
4. Start pulse generation
5. On completion: evaluate brake strategy (Epic 4)
```

## Non-Functional Requirements

### Performance

| Requirement | Target | Source | Measurement |
|-------------|--------|--------|-------------|
| NFR1: Pulse timing accuracy | ±1% up to 500kHz | PRD NFR1 | Oscilloscope verification |
| NFR2: Command response latency | <10ms | PRD NFR2 | Time from CR to OK |
| NFR3: Multi-axis degradation | None with 8 axes | PRD NFR3 | Profile test with all axes |
| NFR5: Motion jitter | <1ms | PRD NFR5 | Oscilloscope pulse timing |
| NFR6: Motion update rate | 1000Hz | PRD NFR6 | Profile fill rate |
| RMT buffer refill | <1ms per 512 symbols | Architecture | Profile timing |
| MCPWM stop latency | <100µs | Architecture | PCNT callback to PWM stop |

**Pulse Generation Performance:**

- RMT: 4 channels × 500kHz max = 2M pulses/sec aggregate capacity (200kHz default per axis)
- MCPWM: 2 timers × 500kHz max with PCNT internal routing
- LEDC: 1 channel, adequate for D-axis stepper requirements
- Double-buffer streaming: unlimited motion length, no buffer constraints

### Security

| Requirement | Implementation | Notes |
|-------------|----------------|-------|
| Input validation | All commands validated against limits | ERR_POSITION_LIMIT, ERR_INVALID_AXIS |
| No unauthorized motion | Motors disabled at startup | Requires explicit EN command |
| Soft limit enforcement | Position validated before motion | Cannot command beyond limits |
| State machine integrity | AxisState transitions validated | Invalid state transitions rejected |

No network connectivity, no authentication required. Physical access = full control (internal tool).

### Reliability/Availability

| Requirement | Target | Source | Implementation |
|-------------|--------|--------|----------------|
| NFR7: Continuous operation | 24+ hours | PRD NFR7 | No memory leaks, stable tasks |
| NFR8: USB disconnect handling | Graceful | PRD NFR8 | Motion continues, events queued |
| NFR12: Brake engagement | <50ms | PRD NFR12 | Direct SPI path (Epic 4) |
| Position consistency | Maintained on error | Architecture | Pulse count atomic updates |
| Motion abort safety | Controlled decel | Architecture | DMA buffer drain on stop |

**Failure Modes:**

- USB disconnect: Motion continues to completion, events queued (lost if buffer full)
- Pulse generator error: Motion stops, axis enters ERROR state, requires re-enable
- SPI failure: Emergency disable all (all bits LOW), brakes engage

### Observability

| Signal | Mechanism | Purpose |
|--------|-----------|---------|
| Motion state | AxisState enum | Track per-axis status |
| Position | getPosition() | SI units, derived from pulse count |
| Velocity | getCurrentVelocity() | Real-time pulse rate |
| Events | EVT_MOTION_COMPLETE, EVT_MOTION_ERROR | Host notification |
| Debug logging | ESP_LOG macros | Development tracing |

**Logging Tags:**

- `SHIFT_REG`: Shift register operations
- `RMT_PULSE`: RMT pulse generation
- `MCPWM_PULSE`: MCPWM pulse generation
- `LEDC_PULSE`: LEDC pulse generation
- `MOTOR`: Motor abstraction layer
- `MOTION_CTRL`: Motion controller
- `CMD_EXEC`: Command executor

## Dependencies and Integrations

**Epic Dependencies:**

| Dependency | Type | Notes |
|------------|------|-------|
| Epic 1 (Foundation) | Hard | HAL, GPIO init, SPI init, FreeRTOS tasks |
| Epic 2 (Communication) | Hard | USB CDC, command parser, event system |

**ESP-IDF Components:**

| Component | Purpose | Version Constraint |
|-----------|---------|-------------------|
| driver/rmt | RMT pulse generation | ESP-IDF 5.4+ (new RMT API) |
| driver/mcpwm | MCPWM pulse generation | ESP-IDF 5.x |
| driver/ledc | LEDC pulse generation | ESP-IDF 5.x |
| driver/pcnt | Pulse counting | ESP-IDF 5.x |
| driver/spi_master | Shift register SPI | ESP-IDF 5.x |

**Internal Component Dependencies:**

```
motor/
  ├── depends on: pulse_gen/, position/, drivers/tpic6b595/
  └── uses: events/event_manager/

pulse_gen/
  ├── depends on: config/, yarobot_hal/
  └── uses: FreeRTOS (task notifications)

control/motion_controller/
  ├── depends on: motor/, events/
  └── uses: interface/command_parser/ (via command_executor)
```

**Hardware Integration Points:**

| Signal | GPIO/Peripheral | Direction |
|--------|-----------------|-----------|
| X_STEP | GPIO_X_STEP via RMT CH0 | Output |
| Y_STEP | GPIO_Y_STEP via MCPWM Timer 0 | Output |
| Z_STEP | GPIO_Z_STEP via RMT CH1 | Output |
| A_STEP | GPIO_A_STEP via RMT CH2 | Output |
| B_STEP | GPIO_B_STEP via RMT CH3 | Output |
| C_STEP | GPIO_C_STEP via MCPWM Timer 1 | Output |
| D_STEP | GPIO_D_STEP via LEDC | Output |
| SR_MOSI | GPIO_SR_MOSI via SPI2 | Output |
| SR_SCK | GPIO_SR_CLK via SPI2 | Output |
| SR_LATCH | GPIO_SR_LATCH | Output |
| SR_OE | GPIO_SR_OE | Output |

**External Driver Interface:**

All axes output STEP/DIR/EN to external motor drivers (servo drives or stepper drivers). Signal levels are 24V via TPIC6B595N open-drain outputs.

## Acceptance Criteria (Authoritative)

Derived from PRD FR1-10, FR43-44, FR48:

| # | Criteria | Testable Statement | Source |
|---|----------|-------------------|--------|
| AC1 | 8-axis control | System controls all 8 axes (X-E) independently via MOVE commands | FR1 |
| AC2 | Pulse frequency | STEP pulses generated up to 500kHz with ±1% timing accuracy (200kHz default max) | FR2 |
| AC3 | Servo STEP/DIR | 5 servo axes (X,Y,Z,A,B) generate STEP/DIR signals | FR3 |
| AC4 | Stepper + count | 2 stepper axes (C,D) generate pulses with position counting | FR4 |
| AC5 | Discrete actuator | E axis controls discrete actuator via time-based positioning | FR5 |
| AC6 | Absolute move | `MOVE X 0.100` moves X axis to 0.100m position | FR6 |
| AC7 | Relative move | `MOVR X 0.025` moves X axis +0.025m from current position | FR7 |
| AC8 | Jog mode | `VEL X 0.050` jogs X axis at 0.050 m/s until STOP | FR8 |
| AC9 | Stop command | `STOP X` decelerates and stops X axis motion | FR9 |
| AC10 | Enable/disable | `EN X 1` enables axis, `EN X 0` disables and stops motion | FR10 |
| AC11 | Position report | `POS X` returns current position in SI units | FR43 |
| AC12 | Motion status | `isMoving()` correctly reflects motion state | FR44 |
| AC13 | Motion event | `EVENT DONE X <pos>` sent on motion completion | FR48 |
| AC14 | Shift register | 40-bit chain updates DIR/EN/BRAKE/ALARM_CLR correctly | Story 3.1 |
| AC15 | Emergency disable | `sr_emergency_disable_all()` sets all bits LOW within 100µs | Story 3.1 |
| AC16 | Multi-axis parallel | X and Y can move simultaneously without interference | FR1 |
| AC17 | Blend on re-target | New MOVE during motion blends smoothly to new target | Arch |
| AC18 | Soft limit reject | Position beyond config limits returns ERR_POSITION_LIMIT | FR17 |
| AC19 | Disabled axis reject | MOVE on disabled axis returns ERR_AXIS_NOT_ENABLED | FR10 |
| AC20 | Direction setup | DIR signal stable for TIMING_DIR_SETUP_US before first STEP | Arch |

## Traceability Mapping

| AC | Spec Section | Component(s) | API(s) | Test Approach |
|----|--------------|--------------|--------|---------------|
| AC1 | Services & Modules | MotionController, all motors | moveAbsolute() | Integration: command all 8 axes |
| AC2 | Performance NFRs | RmtPulseGenerator | startMove() | Oscilloscope verification |
| AC3 | APIs/IMotor | ServoMotor | moveAbsolute() | GPIO output verification |
| AC4 | APIs/IMotor | StepperMotor, PcntTracker | moveAbsolute(), getPosition() | PCNT readback test |
| AC5 | APIs/IMotor | DiscreteAxis | moveAbsolute() | Timing + GPIO verification |
| AC6 | Workflows/Motion | MoveHandler | execute() | Serial command test |
| AC7 | Workflows/Motion | MoveHandler | execute() | Serial command test |
| AC8 | APIs/IPulseGen | VelHandler | startVelocity() | Serial command test |
| AC9 | APIs/IPulseGen | StopHandler | stop() | Verify decel profile |
| AC10 | APIs/ShiftReg | EnableHandler | sr_set_enable() | Enable/disable sequence |
| AC11 | APIs/IMotor | PosHandler | getPosition() | Position readback |
| AC12 | APIs/IMotor | StatusHandler | isMoving() | Poll during motion |
| AC13 | Workflows/Motion | EventManager | publish(EVT_MOTION_COMPLETE) | Subscribe + verify |
| AC14 | Data Models/SR | ShiftRegisterController | sr_set_*(), sr_update() | Logic analyzer trace |
| AC15 | APIs/ShiftReg | ShiftRegisterController | sr_emergency_disable_all() | ISR timing test |
| AC16 | Workflows | MotionController | moveAbsolute() × 2 | Dual axis command |
| AC17 | Workflows | ServoMotor | moveAbsolute() mid-motion | Re-target test |
| AC18 | Data Models | ServoMotor | moveAbsolute() | Out-of-range test |
| AC19 | Workflows | MotionController | moveAbsolute() | Disabled axis test |
| AC20 | Workflows/SR | ServoMotor | dir setup delay | Oscilloscope verification |

## Risks, Assumptions, Open Questions

**Risks:**

| Risk | Impact | Mitigation |
|------|--------|------------|
| R1: RMT double-buffer timing tight at 500kHz | High pulse rate may miss refill deadline | Profile buffer fill time, validate at 500kHz before production use |
| R2: MCPWM+PCNT internal routing undocumented | May require different approach for Y/C | Verify with test program before full implementation |
| R3: SPI shift register chain timing | 40-bit transfer time affects DIR setup | Measure actual SPI timing, adjust delays if needed |
| R4: Multi-axis DMA contention | Multiple axes at high speed may starve CPU | Profile aggregate bandwidth, prioritize motion task |
| R5: Motion blending edge cases | Complex trajectories may produce unexpected behavior | Extensive testing of blend scenarios |

**Assumptions:**

| Assumption | Basis | Validation |
|------------|-------|------------|
| A1: ESP-IDF 5.4 RMT API stable | LTS release | Already in use for Epic 1-2 |
| A2: 80MHz RMT resolution adequate | 12.5ns tick = 500kHz at 50% duty (160 ticks/period) | Oscilloscope verification |
| A3: Single motion task per axis sufficient | No coordinated motion in MVP | Architecture decision |
| A4: Atomic pulse_count updates thread-safe | std::atomic<int64_t> on ESP32-S3 | Platform documentation |
| A5: TPIC6B595N responds within spec | Datasheet claims <1µs propagation | Verified in hardware |

**Open Questions:**

| Question | Impact | Resolution Path |
|----------|--------|-----------------|
| Q1: Best LEDC counting strategy for D axis? | Low - D axis is simple stepper | Evaluate timer ISR vs PCNT software during 3.4 |
| Q2: Motion blending decel-then-accel optimal? | Medium - affects smoothness | Test actual hardware response in 3.9 |
| Q3: Brake engagement timing critical for servos? | Medium - handled in Epic 4 | Defer to Story 4.5 |

## Test Strategy Summary

**Test Levels:**

| Level | Scope | Tools | Coverage |
|-------|-------|-------|----------|
| Unit | Individual components | Unity (ESP-IDF) | Interfaces, edge cases |
| Integration | Component interactions | On-target tests | API contracts |
| System | End-to-end commands | Serial terminal + hardware | All ACs |
| Hardware | Electrical signals | Oscilloscope, logic analyzer | Timing, levels |

**Key Test Cases by Story:**

| Story | Critical Tests |
|-------|----------------|
| 3.1 | Shift register bit mapping, emergency disable timing, SPI throughput |
| 3.2 | RMT pulse accuracy at 10/100/200/500kHz, buffer underrun prevention |
| 3.3 | MCPWM+PCNT internal routing, count accuracy, stop timing |
| 3.4 | LEDC frequency accuracy, software count tracking |
| 3.5 | Position tracker reset, direction handling, overflow |
| 3.6 | ServoMotor limit validation, unit conversion, enable sequence |
| 3.7 | StepperMotor PCNT verification, position accuracy |
| 3.8 | DiscreteAxis time-based positioning, binary limits |
| 3.9 | MOVE/MOVR command parsing, multi-axis parallel, limit rejection |
| 3.10 | VEL continuous motion, STOP deceleration, EN state changes, POS format |
| 3.11 | Event timing, event ordering, subscriber delivery |

**Edge Cases to Validate:**

- Motion blend: MOVE during MOVE, VEL during MOVE, MOVE during STOP
- Boundary: Zero-length move, move to current position
- Limits: Exactly at limit, beyond limit, negative positions
- Enable: Enable during motion, disable during motion
- Multi-axis: All 8 axes simultaneously, interleaved commands

**Hardware Verification Criteria:**

- STEP pulse: 50% duty cycle ±5%, frequency accuracy ±1%
- DIR setup: Minimum TIMING_DIR_SETUP_US before first STEP
- EN timing: TIMING_ENABLE_DELAY_US between EN and first motion
- SR chain: All 40 bits correctly mapped, emergency disable <100µs

**Test Environment:**

- ESP32-S3-DevKitC-1 with external motor drivers connected
- USB serial for command interface (115200 baud)
- Oscilloscope for pulse verification
- Logic analyzer for shift register timing
