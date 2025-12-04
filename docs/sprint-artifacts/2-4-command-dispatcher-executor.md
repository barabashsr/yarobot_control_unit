# Story 2.4: Command Dispatcher & Executor

Status: done

## Story

As a **developer**,
I want **commands routed to appropriate handlers based on verb**,
so that **command processing is modular and extensible**.

## Acceptance Criteria

1. **AC1:** Given a ParsedCommand is ready, when the dispatcher processes it, then it looks up the handler in the command table using `cmd->verb`
2. **AC2:** Given a command table entry is found, when state validation runs, then the handler is only invoked if `current_state & entry.allowed_states` is non-zero
3. **AC3:** Given state validation fails, when generating response, then format is `ERROR E012 Command blocked in current mode\r\n` (ERR_MODE_BLOCKED, MSG_MODE_BLOCKED)
4. **AC4:** Given command verb is not in command table, when generating response, then format is `ERROR E001 Invalid command\r\n` (ERR_INVALID_COMMAND, MSG_INVALID_COMMAND)
5. **AC5:** Given cmd_executor_init() is called, when initialization completes, then all built-in command handlers are registered in the table
6. **AC6:** Given cmd_executor_register() is called, when adding a new entry, then the command becomes available for dispatch
7. **AC7:** Given dispatch_command() is called, when handler executes successfully, then response buffer contains handler's response
8. **AC8:** Given dispatch_command() is called, when handler returns error, then response buffer contains formatted error
9. **AC9:** Given cmd_executor_task processes a command, then it logs the command at DEBUG level for troubleshooting (FR26: command history)
10. **AC10:** Given get_system_state() is called, when querying state, then current SystemState enum value is returned
11. **AC11:** Given set_system_state() is called, when changing state, then system state is updated atomically
12. **AC12:** Given is_state_allowed() is called with a bitmask, when checking permission, then returns true if current state matches mask
13. **AC13:** Given multiple tasks may call dispatch_command(), when executing handlers, then shared state access is mutex-protected

## Tasks / Subtasks

- [x] **Task 1: Create Command Executor Component Structure** (AC: 5, 6)
  - [x] Create directory `firmware/components/control/command_executor/`
  - [x] Create `firmware/components/control/command_executor/include/command_executor.h`
  - [x] Create `firmware/components/control/command_executor/command_executor.c`
  - [x] Create `firmware/components/control/command_executor/CMakeLists.txt`
  - [x] Add component dependencies: `config`, `command_parser`, `freertos`

- [x] **Task 2: Define SystemState Enum and State Management** (AC: 10-12)
  - [x] Define SystemState enum in header:
    ```c
    typedef enum {
        STATE_IDLE   = 0x01,    // Initial state after boot
        STATE_READY  = 0x02,    // Normal operation
        STATE_CONFIG = 0x04,    // Configuration mode
        STATE_ESTOP  = 0x08,    // Emergency stop active
        STATE_ERROR  = 0x10,    // Axis error state
        STATE_ANY    = 0xFF     // Allowed in any state
    } SystemState;
    ```
  - [x] Implement get_system_state() returning current state
  - [x] Implement set_system_state() with atomic update
  - [x] Implement is_state_allowed() checking bitmask
  - [x] Use static volatile for thread-safe state variable

- [x] **Task 3: Define CommandHandler and CommandEntry Types** (AC: 1, 2)
  - [x] Define CommandHandler function pointer type:
    ```c
    typedef esp_err_t (*CommandHandler)(const ParsedCommand* cmd, char* response, size_t resp_len);
    ```
  - [x] Define CommandEntry struct:
    ```c
    typedef struct {
        const char* verb;        // CMD_* constant
        CommandHandler handler;
        uint32_t allowed_states; // Bitmask of valid SystemState values
    } CommandEntry;
    ```

- [x] **Task 4: Implement Command Table and Registration** (AC: 5, 6)
  - [x] Create static command table array with initial capacity (32 entries)
  - [x] Implement cmd_executor_init() to initialize table and register built-in commands
  - [x] Implement cmd_executor_register() to add entries at runtime
  - [x] Built-in commands for Epic 2: CMD_ECHO, CMD_INFO, CMD_STAT, CMD_MODE (stub handlers initially)

- [x] **Task 5: Implement dispatch_command()** (AC: 1-4, 7, 8)
  - [x] Look up verb in command table (case-insensitive comparison using strcasecmp)
  - [x] If not found, format ERR_INVALID_COMMAND response and return
  - [x] If found, check is_state_allowed(entry.allowed_states)
  - [x] If state blocked, format ERR_MODE_BLOCKED response and return
  - [x] Call handler and return its result
  - [x] If handler returns error, ensure response is properly formatted

- [x] **Task 6: Implement Stub Command Handlers** (AC: 7)
  - [x] handle_echo() - return `OK [input text]` for ECHO command
  - [x] handle_info() - return `OK YAROBOT_CONTROL_UNIT 1.0.0 AXES:8` placeholder
  - [x] handle_stat() - return stub system/axis status
  - [x] handle_mode() - query/set mode (stub transitions for now)
  - [x] All handlers use format_ok_data() or format_error() from response_formatter.h

- [x] **Task 7: Add Thread Safety** (AC: 13)
  - [x] Create mutex for command table access (registration)
  - [x] State variable already atomic (single word)
  - [x] Handlers receive their own response buffer (no shared buffer)
  - [x] Document thread-safety guarantees in header

- [x] **Task 8: Add Command Logging** (AC: 9)
  - [x] Log each command at ESP_LOGD level with verb and parameters
  - [x] Include axis if present
  - [x] Use tag "CMD_EXEC" for filtering
  - [x] Format: `CMD_EXEC: verb=%s axis=%c params=%d`

- [x] **Task 9: Integrate with Build System** (AC: 5)
  - [x] Add command_executor.c to control component SRCS in CMakeLists.txt
  - [x] Add command_executor/include to control component INCLUDE_DIRS
  - [x] Verify component compiles with `idf.py build`

- [x] **Task 10: Create Unit Tests** (AC: 1-13)
  - [x] Create `firmware/components/control/command_executor/test/test_command_executor.c`
  - [x] Test dispatch with valid command in table
  - [x] Test dispatch with unknown command (ERR_INVALID_COMMAND)
  - [x] Test dispatch with state-blocked command (ERR_MODE_BLOCKED)
  - [x] Test state management (get/set/is_allowed)
  - [x] Test registration of new commands
  - [x] Test handler return value propagation

## Dev Notes

### Architecture Constraints

> **Command Handler Pattern (from Architecture)**
>
> ```c
> typedef esp_err_t (*CommandHandler)(const ParsedCommand* cmd, Response* resp);
>
> static const CommandEntry command_table[] = {
>     { CMD_MOVE,  handle_move,  STATE_READY | STATE_MOVING },
>     { CMD_STOP,  handle_stop,  STATE_ANY },
>     { CMD_EN,    handle_enable, STATE_READY },
>     // ...
> };
>
> esp_err_t dispatch_command(const ParsedCommand* cmd, Response* resp) {
>     for (const auto& entry : command_table) {
>         if (strcmp(cmd->verb, entry.verb) == 0) {
>             if (!(current_state & entry.allowed_states)) {
>                 return ERR_MODE_BLOCKED;
>             }
>             return entry.handler(cmd, resp);
>         }
>     }
>     return ERR_INVALID_COMMAND;
> }
> ```
>
> [Source: docs/architecture.md#Command-Handler-Pattern]

> **Command Executor API (from Tech Spec)**
>
> ```c
> esp_err_t cmd_executor_init(void);
> esp_err_t cmd_executor_register(const CommandEntry* entry);
> esp_err_t dispatch_command(const ParsedCommand* cmd, char* response, size_t resp_len);
>
> // System state management
> SystemState get_system_state(void);
> esp_err_t set_system_state(SystemState new_state);
> bool is_state_allowed(uint32_t allowed_mask);
> ```
>
> [Source: docs/sprint-artifacts/tech-spec-epic-2.md#APIs-and-Interfaces]

> **Mode State Machine (from Tech Spec)**
>
> ```
>                 ┌─────────────┐
>     Power On ──►│    IDLE     │◄──────┐
>                 └──────┬──────┘       │
>                        │ MODE READY   │ RST (after ESTOP release)
>                        ▼              │
>                 ┌─────────────┐       │
>         ┌──────►│   READY     │◄──────┼───────┐
>         │       └──────┬──────┘       │       │
>         │              │ MODE CONFIG  │       │
>         │              ▼              │       │
> MODE    │       ┌─────────────┐       │       │ MODE READY
> READY   └───────┤   CONFIG    │       │       │
>                 └─────────────┘       │       │
>                                       │       │
>     E-STOP ISR or STOP EMERGENCY ─────┼───────┤
>                 ┌─────────────┐       │       │
>                 │   ESTOP     │───────┘       │
>                 └─────────────┘               │
>                        ▲                      │
>                        │ Axis error detected  │
>                 ┌─────────────┐               │
>                 │   ERROR     │───────────────┘
>                 └─────────────┘  RST
> ```
>
> [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Mode-Transition-State-Machine]

### Core Concepts

**Command Dispatch Architecture:**
- Centralized routing of parsed commands to appropriate handlers
- State-aware dispatch: commands blocked if current state not in allowed_states bitmask
- Handler functions are stateless - receive all context as parameters
- Thread-safe: multiple tasks can dispatch commands safely

**SystemState Bitmask Design:**
- Each state is a power of 2 (0x01, 0x02, 0x04, 0x08, 0x10)
- Allows commands to specify multiple allowed states: `STATE_READY | STATE_CONFIG`
- `STATE_ANY` (0xFF) allows command in all states
- State checked before handler invocation, not within handler

**Command Table:**
- Static table of CommandEntry structs
- Searched linearly (acceptable for <30 commands)
- Verb comparison is case-insensitive (strcasecmp)
- Registration API allows adding commands from other components

### Implementation Details

**File Structure:**
```
firmware/components/control/command_executor/
├── include/
│   └── command_executor.h
├── command_executor.c
├── CMakeLists.txt
└── test/
    └── test_command_executor.c
```

**Initial Command Table (Epic 2):**
| Command | Handler | Allowed States | Notes |
|---------|---------|----------------|-------|
| CMD_ECHO | handle_echo | STATE_ANY | Communication test |
| CMD_INFO | handle_info | STATE_ANY | Version info |
| CMD_STAT | handle_stat | STATE_ANY | System/axis status |
| CMD_MODE | handle_mode | STATE_ANY | Query/set mode |

**Stub Handler Implementation:**
- Handlers return ESP_OK on success, ESP_FAIL or specific error on failure
- Response formatting uses response_formatter.h functions
- Stubs return placeholder data until real implementations in stories 2.5, 2.6

**Thread Safety:**
- `current_state`: Single 32-bit word, atomic read/write on ESP32
- Command table: Protected by mutex during registration (init-time only for built-ins)
- Response buffer: Caller-provided, no shared buffers

**Logging Format (FR26):**
```c
ESP_LOGD("CMD_EXEC", "verb=%s axis=%c params=%d", cmd->verb, cmd->axis, cmd->param_count);
```

### Project Structure Notes

**Component Location:**
- Path: `firmware/components/control/command_executor/`
- Category: control (command processing and state management)

**Dependencies:**
- `config` - for CMD_*, ERR_*, MSG_* constants
- `command_parser` - for ParsedCommand type
- `freertos` - for mutex primitives

**Integration Point:**
- cmd_executor_task (from Epic 1) will call dispatch_command()
- Currently task framework exists but not wired to dispatcher

### Learnings from Previous Story

**From Story 2-3 (Status: done)**

- Response formatter established in `firmware/components/interface/command_parser/`
- Event and EventType types defined in response_formatter.h
- format_ok(), format_ok_data(), format_error(), format_event() functions available
- All formatting uses constants from config_commands.h
- Thread safety achieved via caller-provided buffers
- CMakeLists.txt pattern: add .c files to SRCS, headers in include/

**Files Created in 2-3:**
- `firmware/components/interface/command_parser/include/response_formatter.h`
- `firmware/components/interface/command_parser/response_formatter.c`
- `firmware/components/interface/command_parser/test/test_response_formatter.c`

**Key Reuse:**
- Use format_ok_data() for successful command responses
- Use format_error() for error responses
- Use existing ParsedCommand from command_parser.h
- Use ERR_*, MSG_* constants from config_commands.h

[Source: docs/sprint-artifacts/2-3-response-formatter.md#File-List]

### References

- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#APIs-and-Interfaces] - Command Executor API
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Data-Models-and-Contracts] - SystemState, CommandEntry
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Mode-Transition-State-Machine] - Mode transitions
- [Source: docs/epics.md#Story-2.4] - Story definition
- [Source: docs/architecture.md#Command-Handler-Pattern] - Handler pattern
- [Source: firmware/components/config/include/config_commands.h] - Protocol constants
- [Source: firmware/components/interface/command_parser/include/command_parser.h] - ParsedCommand struct
- [Source: firmware/components/interface/command_parser/include/response_formatter.h] - Formatting API

---

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/2-4-command-dispatcher-executor.context.xml

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

- Build integration required adding command_executor to control umbrella component (not separate component)
- ESP-IDF doesn't auto-discover nested component directories - must add sources to parent CMakeLists.txt

### Completion Notes List

- Implemented complete command dispatcher with state-aware routing
- SystemState enum uses bitmask design (power of 2 values) for flexible allowed_states specification
- Command table supports 32 entries, built-in commands: ECHO, INFO, STAT, MODE
- Thread safety: mutex for registration, atomic state variable, caller-provided response buffers
- Case-insensitive verb lookup using strcasecmp
- All responses use format_ok_data()/format_error() from response_formatter.h
- DEBUG-level logging at tag "CMD_EXEC" for command history (FR26)
- Comprehensive unit tests covering all 13 acceptance criteria

### File List

**Created:**
- `firmware/components/control/command_executor/include/command_executor.h` - Public API header
- `firmware/components/control/command_executor/command_executor.c` - Implementation
- `firmware/components/control/command_executor/CMakeLists.txt` - Component build config (not used, integrated into control)
- `firmware/components/control/command_executor/test/test_command_executor.c` - Unit tests

**Modified:**
- `firmware/components/control/CMakeLists.txt` - Added command_executor sources and includes
- `docs/sprint-artifacts/sprint-status.yaml` - Updated story status

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-04 | SM Agent (Bob) | Initial story draft from Epic 2 Tech Spec |
| 2025-12-04 | Dev Agent (Amelia) | Implemented command dispatcher, state management, stub handlers, tests |
| 2025-12-04 | Dev Agent (Amelia) | Story completed - all ACs verified working on hardware |

### Completion Notes
**Completed:** 2025-12-04
**Definition of Done:** All acceptance criteria met, code reviewed, tests passing
