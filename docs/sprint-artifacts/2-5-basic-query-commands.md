# Story 2.5: Basic Query Commands (CMD_INFO, CMD_STAT, CMD_ECHO)

Status: done

## Story

As a **user**,
I want **to query system information and test communication**,
so that **I can verify the controller is working and check its status**.

## Acceptance Criteria

1. **AC1:** Given I send `ECHO hello world`, when command is processed, then response is `OK hello world\r\n`
2. **AC2:** Given I send `ECHO` (no arguments), when command is processed, then response is `OK\r\n`
3. **AC3:** Given I send `INFO`, when command is processed, then response is `OK YAROBOT_CONTROL_UNIT 1.0.0\r\n` (using FIRMWARE_NAME and FIRMWARE_VERSION from config.h)
4. **AC4:** Given I send `STAT`, when command is processed, then response is `OK MODE:READY ESTOP:0 AXES:8 UPTIME:12345\r\n` with current mode, E-stop status, axis count, and milliseconds since boot
5. **AC5:** Given I send `STAT X`, when command is processed, then response is `OK X POS:0.000 EN:0 MOV:0 ERR:0 LIM:00\r\n` with axis position, enable, moving, error, and limit switch states
6. **AC6:** Given I send `STAT Z`, when command is processed for axis Z, then response includes correct axis letter and status fields matching AC5 format
7. **AC7:** Given uptime query via STAT, when system has been running for N milliseconds, then UPTIME value equals esp_timer_get_time() / 1000
8. **AC8:** Given motor control not yet implemented, when querying axis status, then position returns 0.000, enabled/moving/error return 0, and limits return 00
9. **AC9:** Given command responses, when any response is generated, then it terminates with `\r\n` (CR+LF)
10. **AC10:** Given INFO handler, when formatting response, then FIRMWARE_NAME and FIRMWARE_VERSION_STRING constants from config.h are used

## Tasks / Subtasks

- [x] **Task 1: Update config.h with firmware constants** (AC: 3, 10)
  - [x] Add or verify FIRMWARE_NAME = "YAROBOT_CONTROL_UNIT"
  - [x] Add or verify FIRMWARE_VERSION = "1.0.0"
  - [x] Add FIRMWARE_VERSION_MAJOR/MINOR/PATCH if needed

- [x] **Task 2: Enhance handle_info() implementation** (AC: 3, 10)
  - [x] Include config.h for FIRMWARE_NAME, FIRMWARE_VERSION
  - [x] Format response as `OK %s %s` using constants
  - [x] Remove hardcoded "AXES:8" (not in spec for INFO)

- [x] **Task 3: Implement handle_stat() for system status** (AC: 4, 7)
  - [x] Get current mode from get_system_state()
  - [x] Get E-stop status (placeholder 0 until Epic 4)
  - [x] Use LIMIT_NUM_AXES for axis count
  - [x] Calculate uptime using esp_timer_get_time() / 1000
  - [x] Format: `OK MODE:%s ESTOP:%d AXES:%d UPTIME:%lld`

- [x] **Task 4: Implement handle_stat() for axis status** (AC: 5, 6, 8)
  - [x] Check if cmd->axis is set
  - [x] Validate axis is valid (X-E) using is_valid_axis()
  - [x] Return placeholder status: POS:0.000 EN:0 MOV:0 ERR:0 LIM:00
  - [x] Format: `OK %c POS:%.3f EN:%d MOV:%d ERR:%d LIM:%02X`
  - [x] Return error if invalid axis specified

- [x] **Task 5: Verify handle_echo() implementation** (AC: 1, 2)
  - [x] Verify ECHO with text returns `OK <text>`
  - [x] Verify ECHO without args returns `OK`
  - [x] Both already implemented in story 2-4, just verify

- [x] **Task 6: Create/update unit tests** (AC: 1-10)
  - [x] Test ECHO with text
  - [x] Test ECHO without args
  - [x] Test INFO format matches spec
  - [x] Test STAT system status format
  - [x] Test STAT with valid axis (X, Y, Z, A, B, C, D, E)
  - [x] Test STAT with invalid axis returns error
  - [x] Test uptime increases over time
  - [x] Test all responses end with \r\n

- [x] **Task 7: Build verification** (AC: 1-10)
  - [x] Run `idf.py build` - verify no errors
  - [x] Flash and test commands via USB CDC

## Dev Notes

### Architecture Constraints

> **Query Commands (from Epic Spec)**
>
> - CMD_ECHO is essential for communication testing
> - CMD_INFO provides version for compatibility checking
> - CMD_STAT provides comprehensive status (FR22, FR25, FR44, FR50)
> - Axis status returns placeholder values until motor control implemented
> - Use `esp_timer_get_time()` for uptime
>
> [Source: docs/epics.md#Story-2.5]

> **Response Format (from Architecture)**
>
> - All responses terminate with `\r\n`
> - Success: `OK [data]`
>
> [Source: docs/architecture.md#Communication]

### Core Concepts

**This is a "flesh out the stubs" story:**

Story 2-4 already created stub handlers for ECHO, INFO, STAT in `command_executor.c`. This story upgrades them to full implementations with correct formatting.

**Current Stub Status:**
- `handle_echo()` - Already functional, just needs verification
- `handle_info()` - Returns hardcoded string, needs to use config.h constants
- `handle_stat()` - Returns basic state, needs MODE/ESTOP/AXES/UPTIME format

**Key Data Sources:**
- Mode: `get_system_state()` from command_executor.h
- E-stop: Placeholder 0 (real implementation in Epic 4)
- Axes: `LIMIT_NUM_AXES` from config_limits.h
- Uptime: `esp_timer_get_time() / 1000` (microseconds to milliseconds)
- Axis status: All placeholder 0s until Epic 3 (motor control)

### Response Format Examples

| Command | Response |
|---------|----------|
| `ECHO` | `OK\r\n` |
| `ECHO hello` | `OK hello\r\n` |
| `ECHO hello world` | `OK hello world\r\n` |
| `INFO` | `OK YAROBOT_CONTROL_UNIT 1.0.0\r\n` |
| `STAT` | `OK MODE:READY ESTOP:0 AXES:8 UPTIME:12345\r\n` |
| `STAT X` | `OK X POS:0.000 EN:0 MOV:0 ERR:0 LIM:00\r\n` |
| `STAT Y` | `OK Y POS:0.000 EN:0 MOV:0 ERR:0 LIM:00\r\n` |
| `STAT Q` | `ERROR E002 Invalid axis\r\n` |

### Axis Status Fields

| Field | Description | Placeholder |
|-------|-------------|-------------|
| POS | Current position in SI units | 0.000 |
| EN | Enabled state (0=disabled, 1=enabled) | 0 |
| MOV | Moving state (0=idle, 1=moving) | 0 |
| ERR | Error state (0=OK, 1=error) | 0 |
| LIM | Limit switches (bit0=min, bit1=max, hex) | 00 |

### Learnings from Previous Story

**From Story 2-4 (Status: done)**

- Command handlers exist in `firmware/components/control/command_executor/command_executor.c`
- Stubs already work: ECHO, INFO, STAT, MODE
- `format_ok_data()` and `format_error()` from response_formatter.h handle CR+LF
- State management via get_system_state()/set_system_state()
- Case-insensitive command matching via strcasecmp

**Files to Modify:**
- `firmware/components/control/command_executor/command_executor.c` - Enhance handlers
- `firmware/components/config/include/config.h` - Add FIRMWARE_NAME/VERSION if missing

### References

- [Source: docs/epics.md#Story-2.5] - Story definition
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md] - API specifications
- [Source: firmware/components/control/command_executor/command_executor.c] - Current stubs
- [Source: firmware/components/config/include/config_limits.h] - LIMIT_NUM_AXES
- [Source: firmware/components/interface/command_parser/include/response_formatter.h] - Formatting API

---

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/2-5-basic-query-commands.context.xml

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

- Build successful after adding esp_driver_ledc, esp_driver_mcpwm, esp_driver_pcnt, driver to config component
- Build successful after adding esp_timer to control component

### Completion Notes List

1. **config.h already had FIRMWARE_NAME, FIRMWARE_VERSION_STRING** - verified present (Task 1)
2. **handle_info() enhanced** - now uses config.h constants, removed hardcoded AXES:8 (Task 2)
3. **handle_stat() system status** - returns MODE, ESTOP, AXES, UPTIME format (Task 3)
4. **handle_stat() axis status** - validates axis with is_valid_axis(), returns placeholder values (Task 4)
5. **handle_echo() verified** - already correct from Story 2-4 (Task 5)
6. **Unit tests updated** - added tests for new response formats, invalid axis, uptime increase (Task 6)
7. **Config component fix** - added missing esp_driver_* dependencies to config CMakeLists.txt
8. **Control component fix** - added esp_timer dependency for esp_timer_get_time()

**Architecture Note:** The config component CMakeLists.txt was updated to include driver dependencies (esp_driver_ledc, esp_driver_mcpwm, esp_driver_pcnt, driver) required by config_peripherals.h. This should be documented in architecture.

### File List

**Modified:**
- firmware/components/control/command_executor/command_executor.c - Enhanced handle_info(), handle_stat()
- firmware/components/control/command_executor/test/test_command_executor.c - Updated tests for new formats
- firmware/components/control/CMakeLists.txt - Added esp_timer dependency
- firmware/components/config/CMakeLists.txt - Added driver dependencies

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-04 | SM Agent (Bob) | Initial story draft |
| 2025-12-04 | Dev Agent (Amelia) | Implementation complete - all handlers enhanced, tests updated, build verified |
