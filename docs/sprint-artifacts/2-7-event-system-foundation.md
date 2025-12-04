# Story 2.7: Event System Foundation

Status: ready-for-dev

## Story

As a **developer**,
I want **an event publish/subscribe system**,
so that **subsystems can communicate asynchronously without tight coupling**.

## Acceptance Criteria

1. **AC1:** Given the event manager is initialized, when `event_manager_init()` is called, then it returns ESP_OK and the system is ready for subscriptions
2. **AC2:** Given the event manager is initialized, when a publisher calls `event_publish()`, then all registered subscribers for that event type receive the event
3. **AC3:** Given a callback is registered via `event_subscribe()`, when an event of that type is published, then the callback is invoked with the event and context pointer
4. **AC4:** Given a callback is registered, when `event_unsubscribe()` is called with the same callback, then subsequent events do not invoke that callback
5. **AC5:** Given the event queue is full (LIMIT_EVENT_QUEUE_DEPTH = 32), when another event is published, then ERR_EVENT_OVERFLOW is logged and the oldest event is dropped
6. **AC6:** Given an event is published from ISR context, when `event_publish_from_isr()` is used, then the event is queued without blocking and `*woken` is set if higher priority task is woken
7. **AC7:** Given events are published, when the event processor runs, then events are delivered in FIFO order (no reordering)
8. **AC8:** Given events are published, when subscribers are notified, then delivery occurs within 5ms of publish (AC2.14 from tech spec)
9. **AC9:** Given a USB event subscriber is registered at init, when any event is published, then it is formatted using `format_event()` and sent to host as `EVENT <type> [axis] [data]`
10. **AC10:** Given system boot completes, when event_manager_init() is called, then EVT_BOOT event is published with firmware version and axis count

## Tasks / Subtasks

- [ ] **Task 1: Create event_manager component structure** (AC: 1)
  - [ ] Create `firmware/components/events/event_manager/` directory
  - [ ] Create CMakeLists.txt with REQUIRES for freertos, esp_timer, config
  - [ ] Create `include/event_manager.h` with public API
  - [ ] Create `event_manager.c` with implementation

- [ ] **Task 2: Define Event types and structures** (AC: 2, 3)
  - [ ] Define EventType enum in event_manager.h (EVT_MOTION_COMPLETE, EVT_MOTION_ERROR, EVT_LIMIT_TRIGGERED, EVT_ESTOP_CHANGED, EVT_MODE_CHANGED, EVT_ERROR, EVT_WIDTH_MEASURED, EVT_BOOT)
  - [ ] Define Event struct with type, axis, data union, timestamp
  - [ ] Define EventCallback function pointer type
  - [ ] Add LIMIT_EVENT_QUEUE_DEPTH (32) and LIMIT_EVENT_SUBSCRIBERS (8) to config_limits.h

- [ ] **Task 3: Implement event_manager_init()** (AC: 1, 10)
  - [ ] Create FreeRTOS queue with LIMIT_EVENT_QUEUE_DEPTH
  - [ ] Initialize subscriber list (static array or linked list)
  - [ ] Create event_processor_task for async delivery
  - [ ] Publish EVT_BOOT event with version info from INFO_FW_VERSION

- [ ] **Task 4: Implement event_subscribe() and event_unsubscribe()** (AC: 3, 4)
  - [ ] Validate EventType is in valid range
  - [ ] Store callback and context in subscriber list (per event type)
  - [ ] Return ESP_ERR_NO_MEM if max subscribers reached
  - [ ] Unsubscribe finds and removes matching callback
  - [ ] Use mutex for thread-safe subscriber list modification

- [ ] **Task 5: Implement event_publish()** (AC: 2, 5, 7)
  - [ ] Populate timestamp via esp_timer_get_time()
  - [ ] Send to queue with xQueueSend (no wait, fail if full)
  - [ ] On queue full: log ERR_EVENT_OVERFLOW warning, drop oldest via xQueueReceive + discard
  - [ ] Track queue high watermark for observability

- [ ] **Task 6: Implement event_publish_from_isr()** (AC: 6)
  - [ ] Use xQueueSendFromISR() for non-blocking ISR-safe publish
  - [ ] Set BaseType_t* woken if higher priority task needs to run
  - [ ] Caller responsible for portYIELD_FROM_ISR if woken

- [ ] **Task 7: Implement event_processor_task** (AC: 2, 7, 8)
  - [ ] Block on queue with xQueueReceive (portMAX_DELAY)
  - [ ] For each received event, iterate subscribers for that EventType
  - [ ] Invoke each subscriber callback with event and context
  - [ ] Measure delivery latency, log warning if > 5ms

- [ ] **Task 8: Register USB event subscriber** (AC: 9)
  - [ ] Create usb_event_subscriber callback function
  - [ ] Subscribe to all event types at init (EVT_MOTION_COMPLETE through EVT_BOOT)
  - [ ] Format event using format_event() from response_formatter.h
  - [ ] Send formatted string via usb_cdc_send_line() or tx_queue

- [ ] **Task 9: Update response_formatter for new event types** (AC: 9, 10)
  - [ ] Ensure format_event() handles all EventType values
  - [ ] EVT_BOOT format: `EVENT BOOT <version> AXES:<count> STATE:<mode>`
  - [ ] Verify EVENT output format matches spec: `EVENT <type> [axis] [data]`

- [ ] **Task 10: Wire event_manager into command_executor** (AC: 10)
  - [ ] Replace publish_mode_event() placeholder with real event_publish() call
  - [ ] Call event_manager_init() from cmd_executor_init() or app_main()
  - [ ] Register USB subscriber after event_manager_init()

- [ ] **Task 11: Create unit tests** (AC: 1-10)
  - [ ] Test event_manager_init() returns ESP_OK
  - [ ] Test subscribe/unsubscribe callback delivery
  - [ ] Test multiple subscribers receive same event
  - [ ] Test event order preserved (FIFO)
  - [ ] Test queue overflow handling
  - [ ] Test ISR publish variant (mock ISR context)
  - [ ] Test all event types can be formatted
  - [ ] Test EVT_BOOT published on init

- [ ] **Task 12: Build verification** (AC: 1-10)
  - [ ] Run `idf.py build` - no errors
  - [ ] Run unit tests - all pass
  - [ ] Verify EVT_BOOT appears on serial after flash

## Dev Notes

### Architecture Constraints

> **Event-Driven Architecture (from Architecture)**
>
> - Mode changes and errors published as events, not polled
> - Events are delivered to subscribers within 5ms of publish
> - Event format: `EVENT <type> <axis> [data]`
>
> [Source: docs/sprint-artifacts/tech-spec-epic-2.md#System-Architecture-Alignment]

> **Event Manager API (from Tech Spec)**
>
> ```c
> typedef void (*EventCallback)(const Event* event, void* ctx);
>
> esp_err_t event_manager_init(void);
> esp_err_t event_subscribe(EventType type, EventCallback callback, void* ctx);
> esp_err_t event_unsubscribe(EventType type, EventCallback callback);
> esp_err_t event_publish(const Event* event);
> esp_err_t event_publish_from_isr(const Event* event, BaseType_t* woken);
> ```
>
> [Source: docs/sprint-artifacts/tech-spec-epic-2.md#APIs-and-Interfaces]

### Core Concepts

**Pub/Sub Pattern for Loose Coupling:**

The event system decouples publishers (motion controller, safety system, mode manager) from subscribers (USB output, logging, display). This enables:
- Motion task publishes EVT_MOTION_COMPLETE without knowing who cares
- USB task subscribes and formats for host output
- Future subscribers (display, ROS2) can be added without changing publishers

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
        const char* mode_name;  // For EVT_MODE_CHANGED
    } data;
    int64_t timestamp;      // esp_timer_get_time()
} Event;
```

**Thread Safety:**

- Subscriber list protected by mutex
- Event queue is FreeRTOS queue (thread-safe)
- ISR variant uses xQueueSendFromISR (non-blocking)
- Callbacks invoked from event_processor_task context

**USB Event Output Format:**

```
EVENT DONE X 100.000       // Motion complete
EVENT LIMIT Y MIN          // Limit triggered
EVENT ESTOP ACTIVE         // E-stop activated
EVENT MODE READY           // Mode changed
EVENT BOOT V1.0.0 AXES:8 STATE:IDLE  // Boot complete
```

### Project Structure Notes

**New Component:**
- `firmware/components/events/event_manager/` - New component
- `firmware/components/events/event_manager/include/event_manager.h` - Public API
- `firmware/components/events/event_manager/event_manager.c` - Implementation
- `firmware/components/events/event_manager/test/test_event_manager.c` - Unit tests

**Modified Files:**
- `firmware/components/config/include/config_limits.h` - Add LIMIT_EVENT_QUEUE_DEPTH, LIMIT_EVENT_SUBSCRIBERS
- `firmware/components/control/command_executor/command_executor.c` - Call event_manager_init(), replace publish_mode_event()
- `firmware/components/interface/command_parser/response_formatter.c` - Ensure all EventType values handled

### Learnings from Previous Story

**From Story 2-6 (Status: done)**

- **Event Foundation Exists**: `publish_mode_event()` already logs mode changes via ESP_LOGI - this story replaces it with real event_publish()
- **Event Struct Updated**: Event struct already has `mode_name` field for EVT_MODE_CHANGED
- **format_event() Updated**: Response formatter already handles EVTTYPE_MODE_CHANGED formatting
- **TODO in Code**: Story 2-6 left TODO comment for "Story 2-7: full event_publish() integration"

**Files Modified in 2-6:**
- `firmware/components/interface/command_parser/include/response_formatter.h` - Event struct with mode_name
- `firmware/components/interface/command_parser/response_formatter.c` - EVTTYPE_MODE_CHANGED formatting
- `firmware/components/control/command_executor/command_executor.c` - publish_mode_event() placeholder

**Reuse from 2-6:**
- Event struct definition (extend, don't recreate)
- format_event() function (verify handles all types)
- State machine integration pattern

[Source: docs/sprint-artifacts/2-6-mode-management.md#Completion-Notes-List]

### References

- [Source: docs/epics.md#Story-2.7] - Story definition and API specification
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#APIs-and-Interfaces] - Event Manager API
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Data-Models-and-Contracts] - Event structure
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Acceptance-Criteria] - AC2.13, AC2.14, AC2.18
- [Source: docs/architecture.md] - Event-driven architecture pattern
- [Source: docs/sprint-artifacts/2-6-mode-management.md] - Previous story learnings

---

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/2-7-event-system-foundation.context.xml

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List

### File List

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-04 | SM Agent (Bob) | Initial story draft |
