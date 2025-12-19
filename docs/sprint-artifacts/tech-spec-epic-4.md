# Epic Technical Specification: Safety & I/O Systems

Date: 2025-12-17
Author: Sergey
Epic ID: 4
Status: Draft

---

## Overview

Epic 4 delivers the safety-critical monitoring and I/O systems that protect hardware, operators, and ensure reliable operation of the YaRobot Control Unit. Building on the motor control foundation established in Epic 3, this epic implements real-time monitoring of limit switches, emergency stop handling, brake control strategies, driver alarm management, and general-purpose I/O capabilities.

The implementation centers on the MCP23017 I2C expanders for input monitoring and extends the existing TPIC6B595N shift register chain for output control. Safety is prioritized through interrupt-driven detection, fail-safe brake logic, and immediate E-stop response. After this epic, the system monitors all 14 limit switches in real-time, responds to emergency stop within 1ms, controls brakes appropriately for vertical axes, detects and clears driver alarms, provides general-purpose I/O access, and can home all axes to known positions.

This epic covers FR11-18, FR33, FR35-42, FR45, FR49, FR56-64 from the PRD, representing the safety and I/O functionality required for reliable operation.

## Objectives and Scope

**In Scope:**

- MCP23017 I2C expander driver for limit switches and status inputs
- Real-time limit switch monitoring (14 switches across 7 axes)
- Automatic motion stop on limit switch activation
- Emergency stop system with <1ms response time
- Brake control with configurable strategies (ON_DISABLE, ON_ESTOP, ON_IDLE, MANUAL)
- Position loss detection and acknowledgment
- General-purpose digital I/O (4 inputs via MCP23017, 8 outputs via shift registers)
- Servo feedback processing (InPos signal reading)
- C-axis floating switch for object width measurement
- Error tracking, logging, and recovery
- I2C communication health monitoring
- Homing sequences using limit switches
- Driver alarm monitoring (ALARM_INPUT) and clearance (ALARM_CLEAR)

**Out of Scope:**

- Z-signal position synchronization (Epic 6)
- InPos-based motion confirmation with timeout (Epic 6)
- Position drift correction (Epic 6)
- OLED display integration (Epic 5)
- YAML configuration system (Epic 5)
- Advanced homing with Z-signal (Epic 6)
- Temperature monitoring (post-MVP)

## System Architecture Alignment

**Components Referenced:**

| Component | Directory | Responsibility |
|-----------|-----------|----------------|
| mcp23017 driver | `managed_components/espressif__mcp23017/` | I2C GPIO expander (ESP Component Registry) |
| safety_monitor | `components/control/safety_monitor/` | E-stop, limits, brakes, faults |
| command_executor | `components/control/command_executor/` | LIM, RST, BRAKE, DIN, DOUT, HOME, CLR handlers |
| tpic6b595 driver | `components/drivers/tpic6b595/` | Shift register chain (already implemented in Epic 3) |
| event_manager | `components/events/event_manager/` | Safety event publication |
| i2c_hal | `components/hal/i2c_hal/` | I2C bus abstraction |

**MANDATORY Architecture Constraints (from bmm-workflow-status.yaml):**

1. **Header-Only Configuration** — Every configurable value MUST be defined in a header file. No magic numbers in source code:
   - GPIO pins: `GPIO_MCP0_INTA`, `GPIO_MCP0_INTB`, `GPIO_MCP1_INTA`, `GPIO_MCP1_INTB`, `GPIO_E_STOP`
   - I2C addresses: `I2C_ADDR_MCP23017_0` (0x20), `I2C_ADDR_MCP23017_1` (0x21)
   - Timing: `TIMING_ESTOP_RESPONSE_MS`, `TIMING_BRAKE_ENGAGE_MS`, `TIMING_DEBOUNCE_MS`, `TIMING_I2C_POLL_MS`
   - Commands: `CMD_LIM`, `CMD_RST`, `CMD_BRAKE`, `CMD_HOME`, `CMD_CLR`, `CMD_DIN`, `CMD_DOUT`
   - Errors: `ERR_LIMIT_ACTIVE`, `ERR_EMERGENCY_STOP`, `ERR_ALARM_ACTIVE`

2. **Dual-Core Separation** — Core 0 handles communication and safety monitoring (safety_monitor_task at priority 24), Core 1 handles motion control.

3. **Event-Driven Safety** — Safety task is fully event-driven using interrupts and FreeRTOS task notifications. No polling in critical paths.

4. **Fail-Safe Brake Logic** — Brake outputs are active-low; power loss causes all brakes to engage mechanically.

5. **SI Units Convention** — All external interfaces use SI units. Homing velocities in m/s or rad/s.

**Behavioral Decisions Applicable to Epic 4:**

| # | Decision | Rationale |
|---|----------|-----------|
| 21 | EndSwitchMode per switch (MIN/MAX independent) | Four modes: NONE, HARD_STOP, RESTRICT, EVENT_ONLY |
| 22 | Always stop first on limit trigger | Eliminates velocity-check race condition |
| 23 | Controlled deceleration for all stops | Never hard stop; prevents mechanical damage |
| 24 | Both-limits fault detection | If MIN and MAX both active, set FAULT state |
| 25 | BOOT → LIMIT events → LIMITS_SCANNED | Host receives boot event, limit status, scan complete |
| 26 | Full state events during homing | EVENT HOMING axis state POS:pos for each transition |
| 27 | Z-signal fallback configurable | auto/confirm/fail per axis (Epic 6 implements Z-signal) |
| 28 | Backoff collision detection | If opposite limit hit during backoff, abort homing |
| 29 | Fully event-driven safety task | No polling; interrupts + software timers |

## Detailed Design

### Services and Modules

| Module | Location | Inputs | Outputs | Owner Story |
|--------|----------|--------|---------|-------------|
| MCP23017Driver | `managed_components/espressif__mcp23017/` | I2C commands | Port reads, interrupt config | 4.1 |
| LimitMonitor | `components/control/safety_monitor/` | MCP23017 interrupts | Limit state cache, events | 4.2 |
| LimitStopController | `components/control/safety_monitor/` | Limit events | Motion stop signals | 4.3 |
| EstopHandler | `components/control/safety_monitor/` | GPIO_E_STOP interrupt | SR emergency disable, events | 4.4 |
| BrakeController | `components/control/safety_monitor/` | Enable/disable commands | SR brake bits | 4.5 |
| PositionLossDetector | `components/control/safety_monitor/` | Power cycle, E-stop events | POSLOS flags, events | 4.6 |
| DigitalIOManager | `components/control/command_executor/` | DIN/DOUT commands | MCP reads, SR writes | 4.7 |
| ServoFeedbackReader | `components/control/safety_monitor/` | MCP23017 #1 Port B | InPos status | 4.8 |
| FloatingSwitchHandler | `components/control/safety_monitor/` | C-axis limit interrupt | Width measurement, events | 4.9 |
| ErrorManager | `components/control/safety_monitor/` | Error conditions | Error log, recovery state | 4.10 |
| I2CHealthMonitor | `components/control/safety_monitor/` | I2C transactions | Health status, recovery | 4.11 |
| ErrorLogger | `components/control/safety_monitor/` | Error events | Log entries (RAM ring buffer) | 4.12 |
| HomingController | `components/control/motion_controller/` | HOME command | Homing state machine | 4.13 |
| AlarmManager | `components/control/safety_monitor/` | ALARM_INPUT, CLR command | ALARM_CLEAR pulses, events | 4.14 |

### Data Models and Contracts

**LimitSwitchState Structure:**

```c
typedef struct {
    uint8_t axis;                    // AXIS_X..AXIS_E
    bool min_active;                 // MIN limit triggered
    bool max_active;                 // MAX limit triggered
    EndSwitchMode min_mode;          // NONE, HARD_STOP, RESTRICT, EVENT_ONLY
    EndSwitchMode max_mode;
    bool polarity_inverted;          // true = NC (normally closed)
    int64_t last_change_us;          // Timestamp for debouncing
} LimitSwitchState;
```

**EndSwitchMode Enum:**

```c
typedef enum {
    ENDSWITCH_NONE,       // Limit switch disabled/not present
    ENDSWITCH_HARD_STOP,  // Immediate stop, block motion in that direction
    ENDSWITCH_RESTRICT,   // Stop motion, but allow move away
    ENDSWITCH_EVENT_ONLY  // Generate event only, no automatic stop
} EndSwitchMode;
```

**BrakeStrategy Enum:**

```c
typedef enum {
    BRAKE_ON_DISABLE,  // Engage when axis disabled (default)
    BRAKE_ON_ESTOP,    // Engage only on E-stop
    BRAKE_ON_IDLE,     // Engage after TIMING_IDLE_TIMEOUT_MS
    BRAKE_MANUAL       // Only via CMD_BRAKE command
} BrakeStrategy;
```

**HomingState Enum:**

```c
typedef enum {
    HOMING_IDLE,
    HOMING_SEEK_LIMIT,    // Moving toward home limit
    HOMING_LIMIT_HIT,     // Limit contacted
    HOMING_BACKOFF,       // Backing off limit
    HOMING_SEEK_SLOW,     // Slow approach for precision
    HOMING_COMPLETE,      // Homing successful
    HOMING_FAILED         // Homing aborted (error)
} HomingState;
```

**MCP23017 Pin Mapping (from config_i2c.h):**

| Address | Port | Pins | Function |
|---------|------|------|----------|
| 0x20 | A | GPA0-7 | X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX, A_MIN, A_MAX |
| 0x20 | B | GPB0-7 | B_MIN, B_MAX, C_MIN, C_MAX (floating), D_MIN, D_MAX, E_MIN, E_MAX |
| 0x21 | A | GPA0-6 | ALARM_INPUT_X through ALARM_INPUT_D (7 pins), 1 spare |
| 0x21 | B | GPB0-4 | InPos_X through InPos_B (5 servo axes), GPB5-7 spare inputs |

**Shift Register Bit Map for Epic 4 (from config_sr.h):**

| Bits | Function | Notes |
|------|----------|-------|
| 2,6,10,14,18 | SR_X_BRAKE..SR_B_BRAKE | 5 brake outputs (servo axes only) |
| 3,7,11,15,19,23,27 | SR_X_ALARM_CLR..SR_D_ALARM_CLR | 7 alarm clear outputs |
| 32-39 | SR_GP_OUT_0..SR_GP_OUT_7 | 8 general purpose outputs |

### APIs and Interfaces

**MCP23017 Driver API (espressif/mcp23017 component):**

```c
// Initialization
esp_err_t mcp23017_init(mcp23017_handle_t *handle, i2c_port_t port, uint8_t addr);

// Port configuration (all inputs for our use case)
esp_err_t mcp23017_set_direction(mcp23017_handle_t handle, uint8_t port, uint8_t dir);
esp_err_t mcp23017_set_pullup(mcp23017_handle_t handle, uint8_t port, uint8_t pullup);

// Reading
esp_err_t mcp23017_read_port(mcp23017_handle_t handle, uint8_t port, uint8_t *value);
esp_err_t mcp23017_read_pin(mcp23017_handle_t handle, uint8_t port, uint8_t pin, bool *value);

// Interrupt configuration
esp_err_t mcp23017_set_interrupt_on_change(mcp23017_handle_t handle, uint8_t port, uint8_t mask);
esp_err_t mcp23017_get_interrupt_flag(mcp23017_handle_t handle, uint8_t port, uint8_t *flags);
esp_err_t mcp23017_get_interrupt_capture(mcp23017_handle_t handle, uint8_t port, uint8_t *capture);
```

**Safety Monitor API:**

```c
// Initialization
esp_err_t safety_monitor_init(void);
esp_err_t safety_monitor_start(void);

// Limit switch interface
esp_err_t safety_get_limit_state(uint8_t axis, LimitSwitchState *state);
esp_err_t safety_get_all_limits(uint16_t *limit_bitmap);  // 2 bits per axis

// E-stop interface
bool safety_is_estop_active(void);
esp_err_t safety_clear_estop(void);  // Only works if button released

// Brake interface
esp_err_t safety_set_brake(uint8_t axis, bool engage);
esp_err_t safety_set_brake_strategy(uint8_t axis, BrakeStrategy strategy);

// Position loss interface
esp_err_t safety_set_position_loss(uint8_t axis, bool lost);
esp_err_t safety_acknowledge_position(uint8_t axis);

// Alarm interface
esp_err_t safety_get_alarm_state(uint8_t axis, bool *active);
esp_err_t safety_clear_alarm(uint8_t axis);
```

**Homing API:**

```c
// Homing control
esp_err_t homing_start(uint8_t axis);
esp_err_t homing_abort(uint8_t axis);
HomingState homing_get_state(uint8_t axis);

// Homing configuration (from AxisConfig)
// - homing_velocity: m/s or rad/s
// - homing_backoff: distance to back off from limit
// - home_direction: +1 or -1 (toward MIN or MAX)
```

**Command Handlers (Epic 4):**

| Command | Handler | Parameters | Response |
|---------|---------|------------|----------|
| LIM | lim_handler | [axis] | OK X:00 Y:01... or OK X MIN:0 MAX:1 |
| RST | rst_handler | none | OK or ERROR (if E-stop still active) |
| BRAKE | brake_handler | axis 0/1 | OK |
| HOME | home_handler | axis | OK (homing starts) |
| CLR | clr_handler | axis | OK (alarm clear pulse sent) |
| DIN | din_handler | [pin/alias] | OK 0b1010 or OK DIN2 1 |
| DOUT | dout_handler | pin value | OK |
| POSOK | posok_handler | [axis] | OK |

### Workflows and Sequencing

**Limit Switch Detection Flow:**

```
MCP23017 #0 Pin Change
        │
        ▼
GPIO_MCP0_INTA/B Interrupt (ESP32)
        │
        ▼
ISR: xTaskNotifyFromISR(safety_task, NOTIFY_LIMIT)
        │
        ▼
safety_monitor_task wakes (priority 24)
        │
        ├──► Read MCP0 Port A/B via I2C
        │
        ├──► Identify which switch(es) changed
        │
        ├──► Apply debounce filter (TIMING_DEBOUNCE_MS)
        │
        ├──► Update limit_state_cache[]
        │
        ├──► For each active limit with HARD_STOP mode:
        │    └──► motion_controller_stop_axis(axis)
        │
        └──► event_publish(EVTTYPE_LIMIT_TRIGGERED)
             └──► EVENT LIMIT X MAX
```

**E-Stop Response Flow:**

```
GPIO_E_STOP Falls (Button Pressed)
        │
        ▼
ISR: sr_emergency_disable_all() [< 1ms]
        │
        ├──► Clear all EN bits in shift register
        ├──► Engage all brakes (SR bits = 0)
        └──► Set GPIO_SR_OE HIGH (tristate)
        │
        ▼
ISR: xTaskNotifyFromISR(safety_task, NOTIFY_ESTOP)
        │
        ▼
safety_monitor_task:
        │
        ├──► Set system_mode = MODE_ESTOP
        ├──► Mark all axes as UNHOMED
        └──► event_publish(EVTTYPE_ESTOP_CHANGED)
             └──► EVENT ESTOP ACTIVE
```

**Homing Sequence (Limit-Only, Epic 4):**

```
HOME X Command Received
        │
        ▼
HOMING_SEEK_LIMIT
        │
        ├──► Set direction toward MIN limit
        ├──► Start motion at homing_velocity
        │
        ▼
MIN Limit Activates
        │
        ▼
HOMING_LIMIT_HIT
        │
        ├──► Stop motion
        ├──► EVENT HOMING X LIMIT_HIT POS:xxx
        │
        ▼
HOMING_BACKOFF
        │
        ├──► Move away by homing_backoff distance
        │
        ▼
HOMING_SEEK_SLOW
        │
        ├──► Move toward limit at homing_velocity / 10
        │
        ▼
MIN Limit Activates Again
        │
        ▼
HOMING_COMPLETE
        │
        ├──► Stop motion
        ├──► Set position = home_offset
        ├──► Mark axis as HOMED
        └──► EVENT HOMING X COMPLETE POS:0.000
```

**Driver Alarm Detection and Clearance:**

```
ALARM_INPUT_X Goes Active (MCP23017 #1 Port A)
        │
        ▼
GPIO_MCP1_INTA Interrupt
        │
        ▼
safety_monitor_task:
        │
        ├──► Read MCP1 Port A to identify alarm source
        ├──► Stop X axis motion immediately
        ├──► Set X axis state = ERROR
        └──► EVENT ALARM X
        │
        ▼
User Sends: CLR X
        │
        ▼
clr_handler:
        │
        ├──► Assert SR_X_ALARM_CLR (shift register bit 3)
        ├──► Wait TIMING_ALARM_CLEAR_PULSE_MS (100ms)
        ├──► Deassert SR_X_ALARM_CLR
        ├──► Wait TIMING_ALARM_CHECK_DELAY_MS (50ms)
        ├──► Read ALARM_INPUT_X from MCP1
        │
        ├──► If cleared: OK, clear ERROR state
        └──► If still active: ERROR ALARM_PERSIST
```

## Non-Functional Requirements

### Performance

| Requirement | Target | Source | Validation |
|-------------|--------|--------|------------|
| E-stop response time | < 1ms | NFR12, FR12 | Oscilloscope measurement from GPIO to SR latch |
| Limit switch detection | < 5ms + debounce | NFR4 | Time from MCP interrupt to motion stop |
| I2C polling cycle | 5ms (TIMING_I2C_POLL_MS) | NFR4 | Fallback for missed interrupts |
| Brake engagement | < 50ms from trigger | NFR12 | Software timer + SR update |
| Homing accuracy | ±1 pulse | PRD | Compare homed position across cycles |
| Alarm clear pulse | 100ms (TIMING_ALARM_CLEAR_PULSE_MS) | epics.md | Configurable per driver requirements |

**Critical Path Analysis:**

- E-stop ISR → sr_emergency_disable_all() executes in <100µs (direct SPI write)
- Limit interrupt → safety_task wake → I2C read → motion stop: <10ms total
- MCP23017 I2C read at 400kHz: ~200µs per port read

### Security

| Requirement | Implementation | Source |
|-------------|---------------|--------|
| Input validation | All GPIO and axis indices validated before use | OWASP embedded |
| I2C address validation | Only I2C_ADDR_MCP23017_0/1 accepted | config_i2c.h |
| Command bounds checking | All parameters validated in command handlers | Epic 2 patterns |
| Fail-safe defaults | Brakes engage on power loss, motors disabled on boot | FR15, ADR-001 |
| No sensitive data | System contains no credentials or secrets | N/A |

### Reliability/Availability

| Requirement | Target | Implementation |
|-------------|--------|----------------|
| Continuous operation | 24+ hours without crash | NFR7 |
| I2C error recovery | Auto-retry with exponential backoff | Story 4.11 |
| Graceful degradation | Continue with polling if interrupts fail | Story 4.2 |
| Position persistence | Survive I2C glitches without position loss | Story 4.6 |
| Watchdog integration | Safety task heartbeat monitored | Architecture |

**I2C Bus Recovery Strategy:**

1. Timeout on I2C transaction → retry once
2. Second failure → reinitialize I2C bus
3. Third failure → generate EVENT I2C_ERROR, switch to degraded mode
4. Degraded mode: slower polling, no interrupt reliance

### Observability

| Signal | Type | Location |
|--------|------|----------|
| Limit switch events | EVENT LIMIT axis MIN/MAX | USB CDC |
| E-stop state changes | EVENT ESTOP ACTIVE/INACTIVE | USB CDC |
| Homing progress | EVENT HOMING axis state POS:pos | USB CDC |
| Alarm detection | EVENT ALARM axis | USB CDC |
| Position loss | EVENT POSLOS axis | USB CDC |
| I2C health | DIAG command output | USB CDC |
| Error log | ERRLOG command (ring buffer) | USB CDC |

**Diagnostic Commands:**

- `DIAG` - System diagnostics including I2C health, task stack usage
- `ERRLOG` - Read error log ring buffer
- `ERRCLR` - Clear error log

## Dependencies and Integrations

**External Dependencies:**

| Dependency | Version | Source | Purpose |
|------------|---------|--------|---------|
| ESP-IDF | 5.4 LTS | Espressif | Framework, I2C driver, GPIO |
| espressif/mcp23017 | 0.1.1 | ESP Component Registry | MCP23017 I2C expander driver |
| espressif/i2c_bus | (mcp23017 dep) | ESP Component Registry | I2C bus abstraction |
| FreeRTOS | (ESP-IDF bundled) | Espressif | Task notifications, mutexes |

**Internal Dependencies (Prerequisites):**

| Component | Epic/Story | Status | Required For |
|-----------|------------|--------|--------------|
| Shift Register Driver | Epic 3 / Story 3.1 | Done | Brake control, ALARM_CLEAR, GP outputs |
| Event Manager | Epic 2 / Story 2.7 | Done | Safety event publication |
| Command Dispatcher | Epic 2 / Story 2.4 | Done | New command handlers |
| Motion Controller | Epic 3 / Story 3.9 | Done | Motion stop interface |
| I2C HAL | Epic 1 / Story 1.6 | Done | I2C bus access |

**Integration Points:**

| System | Interface | Direction | Data |
|--------|-----------|-----------|------|
| Host PC | USB CDC | Bidirectional | Commands, responses, events |
| MCP23017 #0 | I2C0 @ 400kHz | Input | 16 limit switch states |
| MCP23017 #1 | I2C0 @ 400kHz | Input | 7 ALARM_INPUT, 5 InPos, 4 spare |
| TPIC6B595N chain | SPI2 | Output | Brakes, ALARM_CLEAR, GP outputs |
| Motor Drivers | STEP/DIR/EN | Output | Motion commands (existing) |

## Acceptance Criteria (Authoritative)

**Story 4.1: MCP23017 I/O Expander Driver**
1. AC1: Both MCP23017 devices respond at addresses 0x20 and 0x21 on I2C0
2. AC2: All ports configured as inputs with pull-ups enabled
3. AC3: Interrupt-on-change configured for limit switch ports (MCP0)
4. AC4: Interrupt-on-change configured for ALARM_INPUT port (MCP1 Port A)
5. AC5: Read operations complete within 500µs

**Story 4.2: Limit Switch Monitoring**
6. AC6: All 14 limit switches readable via LIM command
7. AC7: Interrupt triggers within 1ms of switch change
8. AC8: Debounce filtering prevents false triggers (TIMING_DEBOUNCE_MS)
9. AC9: Polarity inversion configurable per switch (NO/NC)
10. AC10: LIM command returns correct format: `OK X:00 Y:01...`

**Story 4.3: Limit Switch Motion Stop**
11. AC11: Motion stops immediately when limit activates (HARD_STOP mode)
12. AC12: EVENT LIMIT axis MIN/MAX published within 10ms
13. AC13: Motion away from limit is allowed
14. AC14: Soft limits prevent motion beyond configured bounds
15. AC15: Both-limits-active triggers FAULT state

**Story 4.4: Emergency Stop System**
16. AC16: E-stop disables all motors within 1ms (TIMING_ESTOP_RESPONSE_MS)
17. AC17: All brakes engage on E-stop
18. AC18: EVENT ESTOP ACTIVE published
19. AC19: All motion commands rejected in ESTOP mode
20. AC20: RST command clears E-stop only if button released

**Story 4.5: Brake Control System**
21. AC21: BRAKE_ON_DISABLE engages brake when axis disabled
22. AC22: BRAKE_ON_ESTOP engages brake only on E-stop
23. AC23: BRAKE_ON_IDLE engages after timeout when axis idle
24. AC24: CMD_BRAKE provides manual control
25. AC25: Brake timing correct: engage before disable, release after enable

**Story 4.6: Position Loss Detection**
26. AC26: All axes marked UNHOMED on power cycle
27. AC27: All axes flagged POSLOS after E-stop
28. AC28: EVENT POSLOS axis published on detection
29. AC29: POSOK command clears position loss flag
30. AC30: Motion blocked on axis with POSLOS until acknowledged

**Story 4.7: General-Purpose Digital I/O**
31. AC31: DIN command reads 4 spare inputs from MCP23017 #1
32. AC32: DOUT command controls 8 outputs via shift register SR4
33. AC33: Input debouncing applied (TIMING_DEBOUNCE_MS)
34. AC34: I/O aliases configurable and usable in commands

**Story 4.8: Servo Feedback Processing**
35. AC35: InPos signals readable from MCP23017 #1 Port B
36. AC36: InPos state included in STAT response

**Story 4.9: C-Axis Floating Switch**
37. AC37: Floating switch detection distinct from hard limit
38. AC38: Width measurement calculated on floating switch trigger
39. AC39: EVENT WIDTH C value published
40. AC40: GETWIDTH command returns last measured width

**Story 4.10: Error Tracking & Recovery**
41. AC41: Error counts tracked per category
42. AC42: ERROR events published with context
43. AC43: Error state clearable via RST or axis re-enable
44. AC44: Unsafe operations blocked in ERROR state

**Story 4.11: I2C Communication Health**
45. AC45: I2C transaction timeouts detected
46. AC46: Auto-retry on transient failures
47. AC47: Bus recovery attempted on persistent failures
48. AC48: EVENT I2C_ERROR published on health degradation

**Story 4.12: Error Logging**
49. AC49: Error log stored in RAM ring buffer (LIMIT_ERROR_LOG_SIZE entries)
50. AC50: ERRLOG command returns recent errors
51. AC51: ERRCLR command clears error log
52. AC52: Errors include timestamp and context

**Story 4.13: Homing Sequences**
53. AC53: HOME command starts homing sequence
54. AC54: Two-stage homing: fast approach, slow final
55. AC55: EVENT HOMING axis state POS:pos for each transition
56. AC56: Backoff collision detection aborts homing
57. AC57: Position set to home_offset on completion
58. AC58: Axis marked HOMED on success

**Story 4.14: Driver Alarm Monitoring & Clearance**
59. AC59: ALARM_INPUT signals monitored via MCP23017 #1 Port A
60. AC60: EVENT ALARM axis published on alarm detection
61. AC61: Axis motion stopped on alarm
62. AC62: CLR command sends ALARM_CLEAR pulse
63. AC63: Alarm state verified after clear pulse
64. AC64: Motion blocked on axis with active alarm

## Traceability Mapping

| AC | Spec Section | Component | Test Idea |
|----|--------------|-----------|-----------|
| AC1-5 | Data Models, APIs | mcp23017 driver | I2C scan, register read/write |
| AC6-10 | Workflows | LimitMonitor | Switch toggle, LIM command |
| AC11-15 | Workflows | LimitStopController | Motion + limit trigger |
| AC16-20 | Workflows | EstopHandler | E-stop button press |
| AC21-25 | Data Models | BrakeController | Enable/disable with scope |
| AC26-30 | Data Models | PositionLossDetector | Power cycle, E-stop test |
| AC31-34 | APIs | DigitalIOManager | DIN/DOUT commands |
| AC35-36 | Data Models | ServoFeedbackReader | InPos query |
| AC37-40 | Workflows | FloatingSwitchHandler | C-axis closing motion |
| AC41-44 | APIs | ErrorManager | Fault injection |
| AC45-48 | NFR Reliability | I2CHealthMonitor | I2C bus fault injection |
| AC49-52 | Observability | ErrorLogger | Error sequence, ERRLOG |
| AC53-58 | Workflows | HomingController | HOME command sequence |
| AC59-64 | Workflows | AlarmManager | Alarm simulation, CLR |

## Risks, Assumptions, Open Questions

**Risks:**

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| R1: MCP23017 interrupt unreliable | Medium | High | Polling fallback at 5ms, interrupt watchdog |
| R2: I2C bus lockup | Medium | High | Bus recovery procedure, dedicated safety GPIO for E-stop |
| R3: Brake timing insufficient | Low | High | Hardware testing, configurable timing |
| R4: Homing position inaccuracy | Medium | Medium | Two-stage homing, configurable backoff |
| R5: Alarm clear not working | Medium | Medium | Multiple clear attempts, manual override |

**Assumptions:**

| Assumption | Validation |
|------------|------------|
| A1: E-stop GPIO is independent of I2C | Hardware design review |
| A2: MCP23017 interrupt latency <1ms | Datasheet confirms <100µs |
| A3: Limit switches are debounced in hardware | May need longer software debounce |
| A4: Motor drivers support ALARM_CLEAR pulse | Driver documentation review |
| A5: espressif/mcp23017 component is stable | Component registry reviews |

**Open Questions:**

| Question | Owner | Status |
|----------|-------|--------|
| Q1: Should E-stop have hardware interlock (bypass software)? | Sergey | Open |
| Q2: What is optimal debounce time for mechanical switches? | Hardware test | Open |
| Q3: Should we support multiple homing strategies per axis? | Sergey | Open |
| Q4: Is alarm auto-clear on motion start desired? | Sergey | Open |

## Test Strategy Summary

**Unit Tests (Host-Based):**

- Mock MCP23017 responses for limit switch logic
- Mock shift register for brake control logic
- Test debounce state machine
- Test homing state machine transitions
- Test error log ring buffer

**Integration Tests (ESP32 Target):**

- I2C communication with real MCP23017s
- Interrupt latency measurement
- E-stop response time measurement (oscilloscope)
- Brake timing verification
- Full homing sequence

**Hardware Tests:**

| Test | Equipment | Pass Criteria |
|------|-----------|---------------|
| E-stop timing | Oscilloscope | GPIO fall → SR latch < 1ms |
| Limit detection | Signal generator | Interrupt within 1ms |
| Brake engagement | Scope + relay click | Correct sequence, timing |
| I2C stress | Bus analyzer | No lockups under load |
| Homing repeatability | Position readout | ±1 pulse over 10 cycles |

**Test Coverage Targets:**

- Unit tests: All state machines, edge cases
- Integration tests: All command handlers, all events
- Hardware tests: All safety-critical paths

**Test Locations:**

- `firmware/components/control/safety_monitor/test/`
- `firmware/components/control/command_executor/test/`
- `firmware/test/integration/safety/`
