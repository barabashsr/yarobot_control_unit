# Story 4.2: Limit Switch Monitoring

Status: done

## Story

As a **user**,
I want **limit switches monitored continuously**,
So that **motion stops before mechanical damage occurs**.

## Acceptance Criteria

### AC1: All 14 Limit Switches Readable via LIM Command
**Given** 14 limit switches are connected (2 per 7 axes, E axis has none)
**When** I send `LIM` command
**Then** response shows all limit states: `OK X:00 Y:00 Z:01 A:00 B:00 C:10 D:00 E:--`
**And** bit0=min, bit1=max, --=no limits (E axis)

### AC2: Single Axis Limit Query
**Given** I send `LIM X`
**When** command executes
**Then** response is `OK X MIN:0 MAX:0` (or MIN:1/MAX:1 if triggered)

### AC3: Interrupt Triggers Within 1ms of Switch Change
**Given** MCP23017 #0 interrupts are configured (via Story 4-1)
**When** any limit switch changes state
**Then** ESP32 GPIO ISR fires within 1ms
**And** ISR signals safety_monitor_task via task notification (MCP_NOTIFY_LIMIT)
**And** safety task reads MCP0 to identify which switch triggered

### AC4: Debounce Filtering Prevents False Triggers
**Given** limit switch bounces on activation
**When** debounce period elapses (TIMING_DEBOUNCE_MS)
**Then** only stable state changes are reported
**And** no spurious limit events are generated

### AC5: Polarity Inversion Configurable per Switch
**Given** limit switch polarity may be NO (normally open) or NC (normally closed)
**When** configuration is loaded
**Then** each switch's polarity is read from config (DEFAULT_LIMIT_POLARITY)
**And** readings are inverted if NC (normally closed)

## Tasks / Subtasks

- [x] **Task 1: Create safety_monitor component** (AC: #1-5)
  - [x] Create `components/control/safety_monitor/` component structure
  - [x] Create CMakeLists.txt with dependencies: mcp23017_wrapper, config, freertos
  - [x] Create `safety_monitor.h` header with public API
  - [x] Test: Component builds successfully

- [x] **Task 2: Implement limit switch state cache** (AC: #1, #2)
  - [x] Define `limit_state_t` structure in header (16-bit for 2 switches x 8 axes)
  - [x] Implement `safety_monitor_get_limit_state()` to read cached state
  - [x] Implement `safety_monitor_get_axis_limits(axis)` for single axis query
  - [x] Cache updated on interrupt or poll
  - [x] Test: Cache returns valid data after initialization

- [x] **Task 3: Implement safety_monitor_task** (AC: #3)
  - [x] Create FreeRTOS task at priority 24 (highest, safety-critical)
  - [x] Task waits on notification bits: MCP_NOTIFY_LIMIT, MCP_NOTIFY_ALARM
  - [x] On notification: read MCP0 via mcp23017_wrapper_get_int_capture()
  - [x] Identify which switches changed state
  - [x] Update limit state cache
  - [x] Test: Task wakes on interrupt notification

- [x] **Task 4: Implement debounce filtering** (AC: #4)
  - [x] Add TIMING_DEBOUNCE_MS to config_timing.h (default 10ms)
  - [x] Track last state change timestamp per switch
  - [x] Ignore state changes within debounce window
  - [x] Test: Rapid toggles filtered to single state change

- [x] **Task 5: Implement polarity inversion** (AC: #5)
  - [x] Add DEFAULT_LIMIT_POLARITY to config_defaults.h (0=NO, 1=NC)
  - [x] Add per-axis limit polarity configuration
  - [x] Apply polarity inversion when reading switch state
  - [x] Test: NC switch reads correctly with inversion

- [x] **Task 6: Implement CMD_LIM command handler** (AC: #1, #2)
  - [x] Register `CMD_LIM` in command dispatcher
  - [x] Implement handler for `LIM` (all axes)
  - [x] Implement handler for `LIM <axis>` (single axis)
  - [x] Format response per spec: `OK X:00 Y:00...` or `OK X MIN:0 MAX:0`
  - [x] Handle E axis special case (no limits)
  - [x] Test: LIM command returns correct format

- [x] **Task 7: Add configuration constants** (AC: #1-5)
  - [x] Add to `config_i2c.h`: MCP0 limit switch pin mappings (MCP0_X_LIMIT_MIN, etc.)
  - [x] Add to `config_timing.h`: TIMING_DEBOUNCE_MS
  - [x] Add to `config_defaults.h`: DEFAULT_LIMIT_POLARITY
  - [x] Test: Build succeeds, constants accessible

- [x] **Task 8: Unit tests for safety_monitor** (AC: #1-5)
  - [x] Create `test/test_safety_monitor.c`
  - [x] Test: Limit state cache initialization
  - [x] Test: Single axis limit query
  - [x] Test: Debounce filtering
  - [x] Test: Polarity inversion

- [x] **Task 9: Integration test - interrupt response** (AC: #3)
  - [x] Create `test/integration/safety_monitor/test_limit_interrupt.c`
  - [x] Test: Interrupt to task notification latency < 1ms
  - [x] Test: Correct switch identification from INTCAP
  - [x] Test: Cache update on interrupt

## Dev Notes

### Relevant Architecture Patterns and Constraints

**Dual-Core Separation:**
- safety_monitor_task runs on Core 0 at priority 24 (highest)
- Must not block or call Core 1 motion control directly
- Use task notifications and event system for cross-core communication

**Interrupt-Driven Safety:**
- Primary detection via MCP23017 interrupts (configured in Story 4-1)
- Polling at TIMING_I2C_POLL_MS (5ms) serves only as fallback for missed interrupts
- Safety response time: TIMING_I2C_POLL_MS + TIMING_SAFETY_RESPONSE_MS

**I2C Bus Sharing:**
- MCP23017 wrapper provides thread-safe access (mutex protected)
- Use `mcp23017_wrapper_read_port()` and `mcp23017_wrapper_get_int_capture()` from Story 4-1

**Limit Switch Pin Mapping (MCP23017 #0 at 0x20):**

| Axis | Min Switch | Max Switch |
|------|------------|------------|
| X | MCP0_GPA0 | MCP0_GPA1 |
| Y | MCP0_GPA2 | MCP0_GPA3 |
| Z | MCP0_GPA4 | MCP0_GPA5 |
| A | MCP0_GPA6 | MCP0_GPA7 |
| B | MCP0_GPB0 | MCP0_GPB1 |
| C | MCP0_GPB2 | MCP0_GPB3 (floating switch) |
| D | MCP0_GPB4 | MCP0_GPB5 |
| E | MCP0_GPB6 | MCP0_GPB7 |

**Note:** E axis has no physical limit switches (discrete axis). C axis max switch is the floating switch for width measurement (Story 4.9).

### Source Tree Components to Touch

- `components/control/safety_monitor/` - NEW: Safety monitoring component
- `components/control/command_executor/` - Add CMD_LIM handler
- `components/config/include/config_i2c.h` - Add limit switch pin mappings
- `components/config/include/config_timing.h` - Add TIMING_DEBOUNCE_MS
- `components/config/include/config_defaults.h` - Add DEFAULT_LIMIT_POLARITY

### Testing Standards

Per project testing strategy:
- Unit tests in `firmware/components/control/safety_monitor/test/`
- Integration tests in `firmware/test/integration/safety_monitor/`
- Hardware tests require limit switch actuation

### Project Structure Notes

- New component follows existing pattern: `components/control/<component_name>/`
- Uses mcp23017_wrapper from Story 4-1 for I2C access
- Command handler follows patterns established in Epic 2

### Learnings from Previous Story

**From Story 4-1-mcp23017-io-expander-driver (Status: done)**

- **MCP23017 Wrapper Available**: `mcp23017_wrapper.h` provides high-level API for both MCP23017 devices. Use `mcp23017_wrapper_read_port()` for polled reads and `mcp23017_wrapper_get_int_capture()` for interrupt-driven reads.

- **Task Notification Pattern**: ISRs use `xTaskNotifyFromISR()` with bit masks:
  - `MCP_NOTIFY_MCP0_INTA` (bit 0) - Port A limits (X,Y,Z,A)
  - `MCP_NOTIFY_MCP0_INTB` (bit 1) - Port B limits (B,C,D,E)
  - `MCP_NOTIFY_LIMIT` - Combined mask for limit switches

- **Interrupt Handler Registration**: Call `mcp23017_wrapper_register_notify_task(safety_monitor_task_handle)` before `mcp23017_wrapper_install_isr()` to receive notifications.

- **Test Patterns**: Follow patterns in `test_mcp23017_wrapper.c` for hardware-dependent tests with graceful skip.

- **Config Constants Already Defined**:
  - `I2C_ADDR_MCP23017_0` (0x20) - Limit switches
  - `GPIO_MCP0_INTA`, `GPIO_MCP0_INTB` - Interrupt GPIOs
  - `TIMING_I2C_READ_TIMEOUT_US` (500) - Read timeout

[Source: docs/sprint-artifacts/4-1-mcp23017-io-expander-driver.md#Dev-Agent-Record]

### References

- [Source: docs/sprint-artifacts/tech-spec-epic-4.md#Story-4.2-Limit-Switch-Monitoring] - AC6-AC10 definitions
- [Source: docs/epics.md#Story-4.2-Limit-Switch-Monitoring] - User story and technical notes
- [Source: docs/sprint-artifacts/tech-spec-epic-4.md#MCP23017-Pin-Mapping] - Pin assignments
- [Source: docs/architecture.md] - Dual-core separation, safety task priority
- [Source: docs/sprint-artifacts/4-1-mcp23017-io-expander-driver.md] - MCP23017 wrapper API

## Dev Agent Record

### Context Reference

- `docs/sprint-artifacts/4-2-limit-switch-monitoring.context.xml` (generated 2025-12-17)

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

### Completion Notes List

### File List

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-17 | SM Agent (Bob) | Initial story draft from epics.md and tech-spec-epic-4.md |
