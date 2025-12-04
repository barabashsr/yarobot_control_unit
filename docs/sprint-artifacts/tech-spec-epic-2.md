# Epic Technical Specification: Communication & Command Interface

Date: 2025-12-04
Author: Sergey
Epic ID: 2
Status: Draft

---

## Overview

Epic 2 establishes the primary user-facing interface for the YaRobot Control Unit: a USB CDC serial communication system with text-based command parsing and formatted responses. This epic transforms the hardware foundation from Epic 1 into an interactive system that accepts human-readable commands and provides structured, parseable responses. It implements the command dispatch architecture that all subsequent motor control and configuration commands will use, making it the communication backbone for the entire system.

The implementation leverages the existing FreeRTOS task framework (usb_rx, usb_tx, cmd_executor tasks from Epic 1) and builds upon the verified USB CDC peripheral configuration (CONFIG_ESP_CONSOLE_USB_CDC=y).

## Objectives and Scope

**In Scope:**
- USB CDC serial interface with bidirectional communication
- Text command parsing with structured output (ParsedCommand)
- Consistent response formatting (OK, ERROR, EVENT)
- Command dispatcher with state validation and handler routing
- Basic query commands (ECHO, INFO, STAT)
- Operational mode management (IDLE, READY, CONFIG, ESTOP, ERROR)
- Event publish/subscribe system for asynchronous notifications
- Command history logging for debugging (FR26)

**Out of Scope:**
- Motor control commands (Epic 3)
- Safety-related commands like RST (Epic 4)
- Configuration upload/download (Epic 5)
- Z-signal and InPos handling (Epic 6)
- ROS2 integration (post-MVP)

## System Architecture Alignment

**Components Referenced:**
- `firmware/components/interface/usb_cdc/` - USB serial interface implementation
- `firmware/components/interface/command_parser/` - Text parsing and response formatting
- `firmware/components/control/command_executor/` - Command dispatch and execution
- `firmware/components/events/event_manager/` - Pub/sub event system

**Architecture Constraints Applied:**

| Constraint | Application in Epic 2 |
|------------|----------------------|
| Dual-core separation | USB RX/TX and cmd_executor run on Core 0 (communication core) |
| SI Units | All position/velocity values in responses use meters/radians |
| Command response <10ms | Simple commands (ECHO, INFO, STAT) must respond within TIMING_CMD_RESPONSE_MS |
| Event-driven | Mode changes and errors published as events, not polled |
| Motors disabled at startup | MODE starts as IDLE, not READY; explicit transition required |
| BOOT event on startup | System sends EVENT BOOT notification when ready |

**MANDATORY Constraints from Architecture:**
- All responses terminate with `\r\n`
- Error codes use ERR_* constants from config_commands.h
- Event format: `EVENT <type> <axis> [data]`
- CONFIG mode blocks all motion commands (FR55)
- ESTOP mode only allows status queries and RST command

## Detailed Design

### Services and Modules

| Module | Responsibility | Inputs | Outputs | Owner |
|--------|---------------|--------|---------|-------|
| usb_cdc | Raw USB communication | USB hardware events | Character streams to/from queues | interface/usb_cdc |
| command_parser | Parse text → ParsedCommand | Raw text line | ParsedCommand struct | interface/command_parser |
| response_formatter | Format responses | Response data | Formatted text string | interface/command_parser |
| command_executor | Dispatch & execute | ParsedCommand | Response string | control/command_executor |
| event_manager | Pub/sub events | Event structs | Callback invocations | events/event_manager |

### Data Models and Contracts

**ParsedCommand Structure:**
```c
typedef struct {
    char verb[16];           // Command verb (CMD_MOVE, CMD_STOP, etc.)
    char axis;               // Axis letter ('X'-'E') or '\0' if none
    float params[4];         // Numeric parameters
    uint8_t param_count;     // Number of parameters parsed (0-4)
    char str_param[32];      // String parameter (for CMD_ALIAS, etc.)
    bool has_str_param;      // Whether str_param is valid
} ParsedCommand;
```

**Event Structure:**
```c
typedef enum {
    EVT_MOTION_COMPLETE,
    EVT_MOTION_ERROR,
    EVT_LIMIT_TRIGGERED,
    EVT_ESTOP_CHANGED,
    EVT_MODE_CHANGED,
    EVT_ERROR,
    EVT_WIDTH_MEASURED,
    EVT_BOOT,
    // ... extensible
} EventType;

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
    int64_t timestamp;      // esp_timer_get_time()
} Event;
```

**System State Enum:**
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

**Command Table Entry:**
```c
typedef esp_err_t (*CommandHandler)(const ParsedCommand* cmd, char* response, size_t resp_len);

typedef struct {
    const char* verb;        // CMD_* constant from config_commands.h
    CommandHandler handler;
    uint32_t allowed_states; // Bitmask of valid SystemState values
} CommandEntry;
```

### APIs and Interfaces

**USB CDC API (usb_cdc.h):**
```c
esp_err_t usb_cdc_init(void);
esp_err_t usb_cdc_send(const char* data, size_t len);
esp_err_t usb_cdc_send_line(const char* line);  // Adds \r\n
bool usb_cdc_is_connected(void);

// Queue handles for task communication
extern QueueHandle_t usb_rx_queue;  // Raw lines from USB
extern QueueHandle_t usb_tx_queue;  // Formatted responses to USB
```

**Command Parser API (command_parser.h):**
```c
esp_err_t parser_init(void);
esp_err_t parse_command(const char* line, ParsedCommand* cmd);
bool is_valid_axis(char axis);
uint8_t axis_to_index(char axis);  // 'X'→0, 'Y'→1, ..., 'E'→7
char index_to_axis(uint8_t index); // 0→'X', 1→'Y', ..., 7→'E'
```

**Response Formatter API (response_formatter.h):**
```c
esp_err_t format_ok(char* buf, size_t len);
esp_err_t format_ok_data(char* buf, size_t len, const char* fmt, ...);
esp_err_t format_error(char* buf, size_t len, const char* code, const char* msg);
esp_err_t format_event(char* buf, size_t len, const Event* event);
```

**Command Executor API (command_executor.h):**
```c
esp_err_t cmd_executor_init(void);
esp_err_t cmd_executor_register(const CommandEntry* entry);
esp_err_t dispatch_command(const ParsedCommand* cmd, char* response, size_t resp_len);

// System state management
SystemState get_system_state(void);
esp_err_t set_system_state(SystemState new_state);
bool is_state_allowed(uint32_t allowed_mask);
```

**Event Manager API (event_manager.h):**
```c
typedef void (*EventCallback)(const Event* event, void* ctx);

esp_err_t event_manager_init(void);
esp_err_t event_subscribe(EventType type, EventCallback callback, void* ctx);
esp_err_t event_unsubscribe(EventType type, EventCallback callback);
esp_err_t event_publish(const Event* event);
esp_err_t event_publish_from_isr(const Event* event, BaseType_t* woken);
```

### Workflows and Sequencing

**Boot Sequence:**
```
1. app_main() → tasks created (Epic 1)
2. usb_cdc_init() → USB enumeration
3. event_manager_init() → event system ready
4. cmd_executor_init() → handlers registered
5. System state → IDLE
6. event_publish(EVT_BOOT) → "EVENT BOOT V1.0.0 AXES:8 STATE:IDLE"
```

**Command Processing Flow:**
```
usb_rx_task         cmd_executor_task      usb_tx_task
    │                      │                    │
    │ USB RX interrupt     │                    │
    ├─► Read characters    │                    │
    │   until \n           │                    │
    │   Push to rx_queue ──┼────────────────────┤
    │                      │                    │
    │                      ├─► Receive line     │
    │                      │   parse_command()  │
    │                      │   validate state   │
    │                      │   dispatch_command()│
    │                      │   format response  │
    │                      │   Push to tx_queue─┼────────────────►│
    │                      │                    │ Write to USB    │
    │                      │                    │                 │
```

**Mode Transition State Machine:**
```
                    ┌─────────────┐
    Power On ──────►│    IDLE     │◄──────┐
                    └──────┬──────┘       │
                           │ MODE READY   │ RST (after ESTOP release)
                           ▼              │
                    ┌─────────────┐       │
            ┌──────►│   READY     │◄──────┼───────┐
            │       └──────┬──────┘       │       │
            │              │ MODE CONFIG  │       │
            │              ▼              │       │
    MODE    │       ┌─────────────┐       │       │ MODE READY
    READY   └───────┤   CONFIG    │       │       │
                    └─────────────┘       │       │
                                          │       │
    E-STOP ISR or STOP EMERGENCY ─────────┼───────┤
                    ┌─────────────┐       │       │
                    │   ESTOP     │───────┘       │
                    └─────────────┘               │
                           ▲                      │
                           │ Axis error detected  │
                    ┌─────────────┐               │
                    │   ERROR     │───────────────┘
                    └─────────────┘  RST
```

## Non-Functional Requirements

### Performance

| Requirement | Target | Measurement | Source |
|-------------|--------|-------------|--------|
| Command response latency | <10ms | Time from \n received to response sent | NFR2, FR20 |
| USB throughput | 115200 baud effective | Sustained character rate | NFR13 |
| Event queue processing | No overflow at 100 events/sec | Queue high watermark | FR24 |
| Parser throughput | >1000 commands/sec | Profiling | Internal |

### Security

- No authentication required (internal engineering tool)
- Input validation prevents buffer overflows (LIMIT_CMD_MAX_LENGTH = 128)
- Malformed commands return error, never crash
- No sensitive data exposed via commands

### Reliability/Availability

| Requirement | Target | Source |
|-------------|--------|--------|
| Continuous operation | 24+ hours without crash | NFR7 |
| USB reconnection | Automatic re-enumeration | NFR8 |
| Command rejection rate | 0% for valid commands | FR19 |
| Event delivery | No loss under normal load | FR24 |

**Error Recovery:**
- USB disconnect/reconnect handled gracefully
- Queue overflow logs warning, drops oldest
- Parse errors return structured error, continue processing

### Observability

**Logging Signals:**
- `CMD_RX`: Every received command at DEBUG level (FR26)
- `CMD_TX`: Every sent response at DEBUG level
- `CMD_ERR`: Parse/dispatch errors at WARN level
- `EVT_PUB`: Published events at DEBUG level
- `USB_STATE`: Connect/disconnect at INFO level

**Metrics:**
- `cmd_rx_count`: Total commands received
- `cmd_err_count`: Parse/validation errors
- `evt_queue_high_watermark`: Peak event queue usage
- `uptime_ms`: Milliseconds since boot

## Dependencies and Integrations

**ESP-IDF Components:**
| Component | Version | Purpose |
|-----------|---------|---------|
| tinyusb (via ESP-IDF) | IDF bundled | USB CDC ACM implementation |
| freertos | IDF bundled | Task, queue, mutex primitives |
| esp_timer | IDF bundled | Timestamps for events |

**Internal Dependencies:**
| Component | Dependency | Notes |
|-----------|------------|-------|
| command_parser | config_commands.h | CMD_*, ERR_*, MSG_* constants |
| command_executor | config_limits.h | Queue depths, buffer sizes |
| event_manager | config_limits.h | LIMIT_EVENT_QUEUE_DEPTH |
| usb_cdc | FreeRTOS tasks from Epic 1 | usb_rx_task, usb_tx_task |

**External Integrations:**
- Host serial terminals (minicom, screen, PuTTY, pyserial)
- Future: ROS2 serial_driver package

## Acceptance Criteria (Authoritative)

1. **AC2.1**: Device enumerates as USB CDC ACM when connected via USB-C
2. **AC2.2**: Characters typed in terminal appear in firmware RX queue within 10ms
3. **AC2.3**: Characters sent by firmware appear in terminal within 10ms
4. **AC2.4**: USB disconnect/reconnect resumes communication without device reset
5. **AC2.5**: ECHO command returns input text with OK prefix
6. **AC2.6**: INFO command returns firmware name and version
7. **AC2.7**: STAT command returns system mode, E-stop status, axis count, uptime
8. **AC2.8**: STAT X returns axis-specific position, enable, moving, error, limit states
9. **AC2.9**: Unknown commands return ERROR E001 with message
10. **AC2.10**: Commands blocked by mode return ERROR E003 with message
11. **AC2.11**: MODE query returns current mode
12. **AC2.12**: MODE CONFIG blocks motion commands (verified with stub handler)
13. **AC2.13**: Mode transitions generate EVENT MODE notifications
14. **AC2.14**: Events are delivered to subscribers within 5ms of publish
15. **AC2.15**: Parser correctly handles case-insensitive commands and axes
16. **AC2.16**: Parser returns error for malformed input without crashing
17. **AC2.17**: All responses terminated with \r\n
18. **AC2.18**: EVENT BOOT sent after system initialization completes

## Traceability Mapping

| AC | Spec Section | Component(s) | Test Approach |
|----|--------------|--------------|---------------|
| AC2.1 | USB CDC API | usb_cdc | Manual: Connect and verify enumeration |
| AC2.2 | Workflows/Sequencing | usb_cdc, usb_rx_task | Timing test with pyserial |
| AC2.3 | Workflows/Sequencing | usb_cdc, usb_tx_task | Timing test with pyserial |
| AC2.4 | Reliability | usb_cdc | Manual: Disconnect/reconnect cycle |
| AC2.5 | Command Handlers | command_executor | Unit test + integration test |
| AC2.6 | Command Handlers | command_executor | Integration test |
| AC2.7 | Command Handlers | command_executor | Integration test |
| AC2.8 | Command Handlers | command_executor | Integration test (stub values) |
| AC2.9 | Command Executor API | command_executor | Unit test |
| AC2.10 | Command Executor API | command_executor | State machine test |
| AC2.11 | Command Handlers | command_executor | Integration test |
| AC2.12 | Mode Transition | command_executor | State validation test |
| AC2.13 | Event Manager API | event_manager | Subscriber callback test |
| AC2.14 | Performance | event_manager | Timing measurement |
| AC2.15 | Command Parser API | command_parser | Unit tests with varied input |
| AC2.16 | Command Parser API | command_parser | Fuzz testing / edge cases |
| AC2.17 | Response Formatter | response_formatter | Output verification |
| AC2.18 | Boot Sequence | event_manager | Monitor output on boot |

## Risks, Assumptions, Open Questions

**Risks:**
| ID | Risk | Likelihood | Impact | Mitigation |
|----|------|------------|--------|------------|
| R1 | USB enumeration timing varies across host OS | Medium | Low | Test on Windows, macOS, Linux |
| R2 | Event queue overflow under heavy load | Low | Medium | Monitor watermark, tune LIMIT_EVENT_QUEUE_DEPTH |
| R3 | Command parsing edge cases cause crashes | Low | High | Comprehensive input validation, fuzz testing |

**Assumptions:**
- A1: TinyUSB CDC driver in ESP-IDF 5.4 is stable and well-tested
- A2: 115200 baud is sufficient for command throughput (proven in Epic 1)
- A3: USB cable length <2m (lab environment, no EMI issues)
- A4: Single host connection at a time (no USB hub complications)

**Open Questions:**
- Q1: Should we support binary protocol option for future high-throughput use? **Decision: No, text-only for MVP; binary as post-MVP if needed**
- Q2: Maximum simultaneous event subscribers? **Decision: 8 subscribers per event type (LIMIT_EVENT_SUBSCRIBERS)**

## Test Strategy Summary

**Test Levels:**

1. **Unit Tests** (host-based, mocked hardware):
   - command_parser: Parse various inputs, edge cases, malformed data
   - response_formatter: Output format verification
   - event_manager: Subscription, unsubscription, callback delivery

2. **Integration Tests** (on-device):
   - USB CDC roundtrip: Echo test via pyserial
   - Command flow: Send command → verify response
   - Mode transitions: Full state machine coverage
   - Event delivery: Subscribe, trigger, verify callback

3. **System Tests** (manual verification):
   - Multi-terminal compatibility (minicom, screen, PuTTY, pyserial)
   - USB disconnect/reconnect recovery
   - Sustained operation (1 hour minimum)

**Coverage Targets:**
- All 18 acceptance criteria covered by tests
- 100% of error paths have corresponding test
- All mode transition paths exercised
- All event types can be published and received
