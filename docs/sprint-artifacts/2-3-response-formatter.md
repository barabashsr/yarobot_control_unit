# Story 2.3: Response Formatter

Status: done

## Story

As a **user**,
I want **consistent, parseable response formats**,
so that **I can programmatically process controller output**.

## Acceptance Criteria

1. **AC1:** Given command processing completes successfully, when a response is generated, then it follows the format `OK [data]\r\n`
2. **AC2:** Given command processing fails, when an error response is generated, then it follows the format `ERROR <code> <message>\r\n`
3. **AC3:** Given an asynchronous event occurs, when an event notification is generated, then it follows the format `EVENT <type> <axis> [data]\r\n`
4. **AC4:** Given format_ok() is called, when generating simple acknowledgment, then response is exactly `OK\r\n`
5. **AC5:** Given format_ok_data() is called with position data, when generating position response, then format is `OK X 123.456\r\n`
6. **AC6:** Given format_error() is called with ERR_INVALID_COMMAND, when generating error, then response is `ERROR E001 Invalid command\r\n`
7. **AC7:** Given format_event() is called for motion complete, when generating event, then format is `EVENT DONE X 100.000\r\n`
8. **AC8:** Given format_event() is called for limit trigger, when generating event, then format is `EVENT LIMIT Y MIN\r\n`
9. **AC9:** Given format_event() is called for E-stop, when generating event, then format is `EVENT ESTOP ACTIVE\r\n`
10. **AC10:** Given any response, when terminator is added, then it is exactly `\r\n` (CR+LF)
11. **AC11:** Given response buffer size, when formatting, then output never exceeds LIMIT_RESPONSE_MAX_LENGTH
12. **AC12:** Given multiple tasks generating events, when format functions are called concurrently, then operations are thread-safe
13. **AC13:** Given simple command response, when timing is measured, then response is generated within TIMING_CMD_RESPONSE_MS (10ms)
14. **AC14:** All response/error/event string literals use constants from config_commands.h (RESP_OK, RESP_ERROR, RESP_EVENT, ERR_*, MSG_*)

## Tasks / Subtasks

- [x] **Task 1: Create Response Formatter Header** (AC: 1-3, 11, 14)
  - [x] Create `firmware/components/interface/command_parser/include/response_formatter.h`
  - [x] Define API functions per Tech Spec:
    ```c
    esp_err_t format_ok(char* buf, size_t len);
    esp_err_t format_ok_data(char* buf, size_t len, const char* fmt, ...);
    esp_err_t format_error(char* buf, size_t len, const char* code, const char* msg);
    esp_err_t format_event(char* buf, size_t len, const Event* event);
    ```
  - [x] Include config_commands.h for RESP_*, ERR_*, MSG_* constants
  - [x] Include Event struct definition (from event_manager.h or define locally)

- [x] **Task 2: Implement format_ok()** (AC: 4, 10, 11)
  - [x] Generate `OK\r\n` response
  - [x] Check buffer length before writing
  - [x] Return ESP_ERR_INVALID_SIZE if buffer too small
  - [x] Use RESP_OK constant from config_commands.h

- [x] **Task 3: Implement format_ok_data()** (AC: 1, 5, 10, 11, 14)
  - [x] Support printf-style format string and varargs
  - [x] Generate `OK <formatted_data>\r\n`
  - [x] Use vsnprintf for safe formatting
  - [x] Append `\r\n` terminator
  - [x] Return ESP_ERR_INVALID_SIZE if result truncated
  - [x] Use RESP_OK constant

- [x] **Task 4: Implement format_error()** (AC: 2, 6, 10, 11, 14)
  - [x] Generate `ERROR <code> <message>\r\n`
  - [x] Use snprintf for safe formatting
  - [x] Append `\r\n` terminator
  - [x] Use RESP_ERROR constant
  - [x] Validate code and message are not NULL

- [x] **Task 5: Define Event Structure** (AC: 3, 7-9)
  - [x] Define EventType enum matching Tech Spec:
    ```c
    typedef enum {
        EVTTYPE_MOTION_COMPLETE,
        EVTTYPE_MOTION_ERROR,
        EVTTYPE_LIMIT_TRIGGERED,
        EVTTYPE_ESTOP_CHANGED,
        EVTTYPE_MODE_CHANGED,
        EVTTYPE_ERROR,
        EVTTYPE_WIDTH_MEASURED,
        EVTTYPE_BOOT,
    } EventType;
    ```
  - [x] Define Event struct matching Tech Spec:
    ```c
    typedef struct {
        EventType type;
        uint8_t axis;           // 0-7 or 0xFF for system-wide
        union {
            float position;
            float width;
            uint8_t error_code;
            uint8_t limit_state;
            bool estop_active;
        } data;
        int64_t timestamp;
    } Event;
    ```

- [x] **Task 6: Implement format_event()** (AC: 3, 7-9, 10, 11, 14)
  - [x] Generate event string based on EventType
  - [x] Handle EVT_MOTION_COMPLETE: `EVENT DONE <axis> <position>`
  - [x] Handle EVT_LIMIT_TRIGGERED: `EVENT LIMIT <axis> MIN|MAX`
  - [x] Handle EVT_ESTOP_CHANGED: `EVENT ESTOP ACTIVE|RELEASED`
  - [x] Handle EVT_BOOT: `EVENT BOOT V1.0.0 AXES:8 STATE:IDLE`
  - [x] Handle EVT_MODE_CHANGED: `EVENT MODE <new_mode>`
  - [x] Use RESP_EVENT constant and EVT_* type strings from config_commands.h
  - [x] Append `\r\n` terminator

- [x] **Task 7: Ensure Thread Safety** (AC: 12)
  - [x] All format functions operate on caller-provided buffer (no shared state)
  - [x] No static variables that could cause race conditions
  - [x] Document thread-safety in header comments

- [x] **Task 8: Integrate with Build System** (AC: 1)
  - [x] Add response_formatter.c to command_parser CMakeLists.txt SRCS
  - [x] Verify clean build

- [x] **Task 9: Test Response Formatter** (AC: 1-14)
  - [x] Create test_response_formatter.c in test/ directory
  - [x] Test format_ok() output
  - [x] Test format_ok_data() with various format strings
  - [x] Test format_error() with all ERR_* codes
  - [x] Test format_event() for each EventType
  - [x] Test buffer overflow protection
  - [x] Test \r\n termination on all outputs
  - [x] Verify constants from config_commands.h are used

## Dev Notes

### Architecture Constraints

> **Response Formatter API (from Tech Spec 2.3)**
>
> ```c
> esp_err_t format_ok(char* buf, size_t len);
> esp_err_t format_ok_data(char* buf, size_t len, const char* fmt, ...);
> esp_err_t format_error(char* buf, size_t len, const char* code, const char* msg);
> esp_err_t format_event(char* buf, size_t len, const Event* event);
> ```
>
> [Source: docs/sprint-artifacts/tech-spec-epic-2.md#APIs-and-Interfaces]

> **Response Format (from Architecture)**
>
> - All responses terminate with `\r\n`
> - Success: `OK [data]`
> - Error: `ERROR <code> <message>`
> - Event: `EVENT <type> <axis> [data]`
>
> [Source: docs/architecture.md#Communication]

> **Performance Requirements**
>
> - Command response latency: <10ms (NFR2, FR20)
> - Use snprintf for safe formatting
> - Thread-safe: multiple tasks may generate events
>
> [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Performance]

### Response Format Examples

| Type | Example | Notes |
|------|---------|-------|
| Simple OK | `OK\r\n` | Acknowledgment only |
| OK with data | `OK X 123.456\r\n` | Position query |
| OK with mode | `OK READY\r\n` | Mode query |
| Error | `ERROR E001 Invalid command\r\n` | Unknown command |
| Error | `ERROR E002 Invalid axis\r\n` | Bad axis letter |
| Event | `EVENT DONE X 100.000\r\n` | Motion complete |
| Event | `EVENT LIMIT Y MIN\r\n` | Limit switch |
| Event | `EVENT ESTOP ACTIVE\r\n` | E-stop activated |
| Event | `EVENT BOOT V1.0.0 AXES:8 STATE:IDLE\r\n` | System boot |

### Buffer Sizes

| Constant | Value | Purpose |
|----------|-------|---------|
| LIMIT_RESPONSE_MAX_LENGTH | 256 | Max response buffer |
| ERR_* codes | 4 chars | E001-E0xx format |
| MSG_* messages | ~30 chars | Human-readable |

### Project Structure Notes

**Files to Create/Modify:**
```
firmware/components/interface/command_parser/
├── include/
│   ├── command_parser.h      (existing)
│   └── response_formatter.h  (NEW)
├── command_parser.c          (existing)
├── response_formatter.c      (NEW)
├── CMakeLists.txt            (MODIFY - add response_formatter.c)
└── test/
    ├── test_command_parser.c (existing)
    └── test_response_formatter.c (NEW)
```

### Learnings from Previous Story

**From Story 2-2 (Status: done)**

- Command parser component structure established in `firmware/components/interface/command_parser/`
- CMakeLists.txt pattern established - add .c files to SRCS list
- Header files go in `include/` subdirectory
- Test files go in `test/` subdirectory
- Thread-safety achieved via stack-allocated buffers (no static state)
- config_commands.h contains all CMD_*, RESP_*, ERR_*, MSG_* constants
- Parser uses ParsedCommand struct - formatter will work with response buffers
- Build system integration verified working

**Files Created in 2-2:**
- `firmware/components/interface/command_parser/include/command_parser.h`
- `firmware/components/interface/command_parser/command_parser.c`
- `firmware/components/interface/command_parser/test/test_command_parser.c`

[Source: docs/sprint-artifacts/2-2-command-parser.md#File-List]

### Implementation Notes

**CRITICAL: Use Constants, Not Hardcoded Strings**

All formatting MUST use constants from `config_commands.h`:

| Format Element | Constant | Example Value |
|----------------|----------|---------------|
| Success prefix | `RESP_OK` | `"OK"` |
| Error prefix | `RESP_ERROR` | `"ERROR"` |
| Event prefix | `RESP_EVENT` | `"EVENT"` |
| Invalid command | `ERR_INVALID_COMMAND` | `"E001"` |
| Invalid command msg | `MSG_INVALID_COMMAND` | `"Invalid command"` |
| Motion complete type | `EVT_MOTION_COMPLETE` | `"DONE"` |
| Limit event type | `EVT_LIMIT_TRIGGERED` | `"LIMIT"` |
| E-stop event type | `EVT_ESTOP_ACTIVATED` | `"ESTOP"` |
| Boot event type | `EVT_BOOT` | `"BOOT"` |

**Thread Safety Design:**
- All functions take caller-provided buffer as parameter
- No static/global buffers inside formatter functions
- Multiple tasks can call format functions simultaneously with their own buffers
- Events from ISR use separate queue (event_manager handles this)

**Float Formatting:**
- Position values formatted with 3 decimal places: `%.3f`
- Ensure locale-independent formatting (no comma decimal separator)

### References

- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#APIs-and-Interfaces] - Response Formatter API
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Data-Models-and-Contracts] - Event structure
- [Source: docs/epics.md#Story-2.3] - Story definition with format examples
- [Source: docs/sprint-artifacts/2-2-command-parser.md] - Previous story implementation
- [Source: firmware/components/config/include/config_commands.h] - Protocol constants

---

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/stories/2-3-response-formatter.context.xml

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

N/A

### Completion Notes List

- All 9 tasks completed successfully
- EventType enum uses EVTTYPE_ prefix to avoid conflict with config_commands.h EVT_* string constants
- LimitState enum added for type-safe limit switch state handling
- All format functions use snprintf/vsnprintf for buffer safety
- Thread safety ensured via caller-provided buffers (no static state)
- Build verified successful with `idf.py build`
- 30+ test cases covering all ACs in test_response_formatter.c

### File List

- `firmware/components/interface/command_parser/include/response_formatter.h` (NEW)
- `firmware/components/interface/command_parser/response_formatter.c` (NEW)
- `firmware/components/interface/command_parser/CMakeLists.txt` (MODIFIED)
- `firmware/components/interface/command_parser/test/test_response_formatter.c` (NEW)

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-04 | SM Agent (Bob) | Initial story draft from Epic 2 Tech Spec |
| 2025-12-04 | Dev Agent (Amelia) | Implemented response formatter - all tasks complete, ready for review |
