# End Switch Operation - Architecture Decisions Summary

> **Document Purpose**: Captures all decisions made regarding end switch operation from the operator/master software perspective. These decisions should be incorporated into `architecture.md`.

## Decision Categories

1. [Homing & Events](#1-homing--events)
2. [EndSwitchMode Configuration](#2-endswitchmode-configuration)
3. [Limit Switch Behavior](#3-limit-switch-behavior)
4. [Boot Sequence](#4-boot-sequence)
5. [Calibration](#5-calibration)
6. [Safety Architecture](#6-safety-architecture)
7. [YAML Configuration](#7-yaml-configuration)
8. [STAT Command Enhancement](#8-stat-command-enhancement)

---

## 1. Homing & Events

### Decision 1.1: Homing State Events
**Gap**: Host has no visibility into homing intermediate states.

**Decision**: Send full state events during homing with position data.

```
EVENT HOMING <axis> <state> POS:<current_pos>
```

**States**: `SEEK_LIMIT`, `LIMIT_HIT`, `BACKOFF`, `SEEK_ZSIGNAL`, `COMPLETE`, `FAILED`

**Example**:
```
EVENT HOMING X SEEK_LIMIT POS:0.150
EVENT HOMING X LIMIT_HIT POS:-0.002
EVENT HOMING X BACKOFF POS:-0.002
EVENT HOMING X SEEK_ZSIGNAL POS:0.003
EVENT HOMING X COMPLETE POS:0.000
```

### Decision 1.2: Limit Context During Homing
**Gap**: No distinction between limit hit during normal motion vs homing.

**Decision**: Host tracks homing state locally. Single `EVENT LIMIT` format; context inferred from whether HOME command was issued. During homing, limit hits are reported via `EVENT HOMING <axis> LIMIT_HIT` instead.

### Decision 1.3: Z-Signal Timeout & Fallback
**Gap**: What if Z-signal never found during homing?

**Decision**: Configurable behavior per axis in YAML.

```yaml
z_signal:
  enabled: true
  fallback: auto|confirm|fail  # Default: auto
```

- **auto**: After 3 retries, auto-fallback to limit-only homing; send `EVENT HOMING_DEGRADED <axis>`
- **confirm**: After 3 retries, send `EVENT HOMING_PAUSED`; require explicit `HOME <axis> SWITCH`
- **fail**: After 3 retries, abort with `EVENT HOMING X FAILED ZSIGNAL_TIMEOUT`

### Decision 1.4: Homing Backoff Collision
**Gap**: What if backoff hits opposite limit?

**Decision**: Abort homing with error.

```
EVENT HOMING <axis> FAILED BACKOFF_COLLISION
```

### Decision 1.5: Z-Signal During Backoff
**Gap**: Should Z-signal be processed during backoff phase?

**Decision**: Ignore Z-signal during backoff. Only process during explicit `SEEK_ZSIGNAL` phase.

---

## 2. EndSwitchMode Configuration

### Decision 2.1: Available Modes
**Gap**: What end switch response modes should exist?

**Decision**: Four modes (all use controlled deceleration, not hard stop):

| Mode | Behavior | Use Case |
|------|----------|----------|
| `NONE` | No switch present; skip limit handling | Continuous rotation axes |
| `HARD_STOP` | Decelerate to stop, set error, restrict motion | Crash protection (default) |
| `RESTRICT` | Decelerate to stop, allow away-motion, no error | Working envelope, object detection |
| `EVENT_ONLY` | Send event, don't stop motion | Position markers, optical sensors |

> **Important**: All stop modes use controlled deceleration to prevent mechanical damage. No instantaneous velocity discontinuity.

### Decision 2.2: Independent MIN/MAX Configuration
**Gap**: Should each switch be configured independently?

**Decision**: Yes. MIN and MAX switches configured separately per axis.

```yaml
switches:
  min:
    mode: RESTRICT      # For object detection (C axis)
    debounce_ms: 20     # Optional override
  max:
    mode: HARD_STOP     # Hard limit protection
```

### Decision 2.3: Default Mode
**Gap**: What if EndSwitchMode not configured?

**Decision**: Default to `HARD_STOP` (safest).

### Decision 2.4: NONE Mode Homing
**Gap**: How to home axes with no switches?

**Decision**: Block homing. `HOME` command returns error for `NONE` mode axes. Position is relative only.

```
HOME C
ERROR E007 Axis has no limit switches
```

---

## 3. Limit Switch Behavior

### Decision 3.1: Power-On Limit Detection
**Gap**: What if axis is on limit at power-on?

**Decision**: Event + restricted motion.

- At boot: scan all limits
- Send `EVENT LIMIT <axis> MIN|MAX` for each active limit
- Set `can_move_positive`/`can_move_negative` flags immediately
- Send `EVENT LIMITS_SCANNED` after all limit events sent

### Decision 3.2: Both Limits Active (Fault)
**Gap**: What if both MIN and MAX active simultaneously?

**Decision**: Critical error - indicates wiring fault.

```
EVENT SWITCH_FAULT <axis> BOTH_ACTIVE
```
- Set axis to FAULT state
- Block all motion on that axis
- Requires physical inspection and `CLR` command after fix

### Decision 3.3: Stop-First Strategy
**Gap**: Race condition between velocity check and limit trigger.

**Decision**: Always stop first on any limit trigger, then check direction.

1. ISR triggers → notify safety task
2. Safety task stops axis immediately (controlled decel)
3. Check if motion was toward or away from limit
4. If was away: motion already stopped, host must re-issue command
5. No auto-resume (safer, more predictable)

### Decision 3.4: Motion Restriction Query
**Gap**: How does host discover current motion restrictions?

**Decision**: Host receives `EVENT LIMIT` and tracks state. Additionally, STAT command enhanced (see Section 8).

### Decision 3.5: Away-Motion Commands
**Gap**: What commands work for moving away from triggered limit?

**Decision**: Any motion command (MOVE/MOVR/VEL) works if target/direction is away from active limit.

- `MOVE X 50` allowed if `X_MIN` triggered and `50 > current_pos`
- `MOVR X 10` allowed if moving in positive direction when MIN triggered
- Validation checks target position against active limits

### Decision 3.6: Position Move vs Soft Limit
**Gap**: Should MOVE commands that would hit soft limit be rejected or executed?

**Decision**: Execute motion, stop at soft limit, send event with original target.

```
EVENT SOFT_LIMIT X POS:0.495 TARGET:0.520
```

### Decision 3.7: CLR Command Behavior
**Gap**: Can CLR clear error while still on limit?

**Decision**: Clear always. CLR clears error flag regardless of limit state; motion restrictions remain based on current switch state.

```
CLR X
OK                    # Error cleared, but if still on limit, motion still restricted
```

---

## 4. Boot Sequence

### Decision 4.1: Boot Event Order
**Gap**: When are limit events sent relative to BOOT?

**Decision**: BOOT first, then limit events, then scan complete.

```
EVENT BOOT V1.0.0 AXES:8 STATE:IDLE
EVENT LIMIT X MIN
EVENT LIMIT Z MAX
EVENT LIMITS_SCANNED
```

### Decision 4.2: Scan Complete Event
**Gap**: How does host know initial limit scan is done?

**Decision**: Explicit completion event.

```
EVENT LIMITS_SCANNED
```

Host waits for this before assuming full system state is known.

---

## 5. Calibration

### Decision 5.1: Axis Length Calibration
**Gap**: Current homing only establishes one reference point.

**Decision**: Add `CALIBRATE LENGTH` command.

```
CALIBRATE LENGTH <axis>
```

**Behavior**:
1. Always seek MIN limit first (known starting point)
2. Record position
3. Travel to MAX limit
4. Record distance as axis length
5. Store in config

### Decision 5.2: Calibration Timeout
**Gap**: What if limit unreachable during calibration?

**Decision**: Travel limit timeout using `mechanical_max_travel` from YAML.

```yaml
X:
  mechanical_max_travel: 0.600  # From datasheet, used for calibration timeout
```

If axis travels beyond `mechanical_max_travel` without hitting limit, abort with error.

### Decision 5.3: Soft Limit Buffer
**Gap**: How far before hard limit should soft limit deceleration start?

**Decision**: Configurable per axis with sensible default.

```yaml
soft_limit_buffer: 0.005  # 5mm default for linear
# or
soft_limit_buffer: 0.087  # ~5° default for rotary
```

Soft limit = hardware limit - buffer.

---

## 6. Safety Architecture

### Decision 6.1: Interrupt vs Polling
**Gap**: Is limit switch polling redundant with interrupts?

**Decision**: Interrupt only. Remove polling for limit switches.

- MCP23017 INTA/INTB interrupts are primary detection method
- No periodic limit polling in safety task
- Reduces CPU overhead

### Decision 6.2: Fully Event-Driven Safety
**Gap**: With no polling, how are time-based checks handled?

**Decision**: FreeRTOS software timers for all time-based safety checks.

- Brake idle timers: `xTimerCreate` with callback to notify safety task
- Motion watchdog: Software timer, callback notifies on timeout
- No periodic polling loop; safety task waits on notifications only

### Decision 6.3: I2C Handling
**Gap**: How to handle multiple MCP23017 interrupt sources?

**Decision**: Sequential handling is sufficient.

- ISRs set notification bits only (no I2C in ISR)
- Safety task reads MCP23017 #0 (limits) first, then #1 (alarms)
- ~100µs per read at 400kHz; total <1ms for both devices
- MCP23017 component handles INTA/INTB separation internally

### Decision 6.4: Debounce Configuration
**Gap**: Is 10ms debounce sufficient?

**Decision**: Configurable with system default + per-switch override.

```yaml
system:
  default_switch_debounce_ms: 20  # System-wide default

axes:
  X:
    switches:
      min:
        mode: HARD_STOP
        debounce_ms: 10  # Override for optical sensor (faster)
      max:
        mode: HARD_STOP
        # Uses system default (20ms)
```

---

## 7. YAML Configuration

### Decision 7.1: Complete Switch Configuration Structure

```yaml
system:
  default_switch_debounce_ms: 20

axes:
  X:
    # ... existing config ...
    mechanical_max_travel: 0.600    # For calibration timeout
    soft_limit_buffer: 0.005        # 5mm before hard limit

    switches:
      min:
        mode: HARD_STOP             # NONE, HARD_STOP, RESTRICT, EVENT_ONLY
        debounce_ms: 20             # Optional, overrides system default
      max:
        mode: HARD_STOP

    homing:
      velocity: 0.05
      velocity_slow: 0.01
      backoff: 0.005
      direction: min

    z_signal:
      enabled: true
      drift_threshold: 10
      fallback: auto                # auto, confirm, fail

  C:  # Picker jaw - object detection example
    switches:
      min:
        mode: RESTRICT              # Object detection, no error
      max:
        mode: HARD_STOP             # Hard limit protection

  E:  # Discrete axis - no switches example
    switches:
      min:
        mode: NONE
      max:
        mode: NONE
```

---

## 8. STAT Command Enhancement

### Decision 8.1: Enhanced STAT Response
**Gap**: STAT doesn't show limit switch state or mode.

**Decision**: Add limit fields to STAT response.

**Current**:
```
OK AXIS:0 POS:25.5 TGT:100.0 VEL:15.2 MOVING:1 ENABLED:1 COMPLETE:0 FAULT:0
```

**Enhanced**:
```
OK AXIS:0 POS:25.5 TGT:100.0 VEL:15.2 MOVING:1 ENABLED:1 COMPLETE:0 FAULT:0 LIMIT_MIN:0 LIMIT_MAX:1 MODE_MIN:RESTRICT MODE_MAX:HARD_STOP
```

New fields:
- `LIMIT_MIN`: Current MIN switch state (0=inactive, 1=active)
- `LIMIT_MAX`: Current MAX switch state (0=inactive, 1=active)
- `MODE_MIN`: Configured EndSwitchMode for MIN switch
- `MODE_MAX`: Configured EndSwitchMode for MAX switch

---

## Event Summary

### New/Modified Events

| Event | Format | When |
|-------|--------|------|
| `HOMING` | `EVENT HOMING <axis> <state> POS:<pos>` | Each homing state transition |
| `HOMING_DEGRADED` | `EVENT HOMING_DEGRADED <axis>` | Z-signal fallback to limit-only |
| `LIMIT` | `EVENT LIMIT <axis> MIN\|MAX` | Limit switch triggered |
| `LIMITS_SCANNED` | `EVENT LIMITS_SCANNED` | Boot limit scan complete |
| `SWITCH_FAULT` | `EVENT SWITCH_FAULT <axis> BOTH_ACTIVE` | Both limits active (wiring fault) |
| `SOFT_LIMIT` | `EVENT SOFT_LIMIT <axis> POS:<pos> TARGET:<tgt>` | Position move truncated at soft limit |

---

## Implementation Priority

1. **High Priority (Safety Critical)**
   - EndSwitchMode implementation (HARD_STOP, RESTRICT, EVENT_ONLY, NONE)
   - Power-on limit detection and restriction
   - Both-limits-active fault detection
   - Stop-first strategy on limit trigger

2. **Medium Priority (Functionality)**
   - Homing state events
   - STAT command enhancement
   - Soft limit buffer configuration
   - CALIBRATE LENGTH command

3. **Lower Priority (Polish)**
   - Configurable debounce per switch
   - Z-signal fallback configuration
   - EVENT_ONLY mode for position markers

---

## Architecture Document Updates Required

The following sections of `architecture.md` need updates:

1. **Safety & Error Handling** - Add EndSwitchMode table, update limit switch handling code
2. **Homing Workflow** - Add state events, Z-signal fallback logic
3. **Configuration Constants** - Add new YAML keys for switch config
4. **Event Types** - Add new event definitions
5. **Boot Sequence** - Document limit scan and LIMITS_SCANNED event
6. **Command Reference** - Add CALIBRATE LENGTH, update STAT response format
