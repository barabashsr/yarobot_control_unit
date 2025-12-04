# Story 2.1: USB CDC Serial Interface

Status: ready-for-dev

## Story

As a **user**,
I want **to connect to the controller via USB serial**,
so that **I can send commands and receive responses from any terminal program**.

## Acceptance Criteria

1. **AC1:** Given the device is powered via USB-C, when I connect from a host computer, then the device enumerates as a USB CDC ACM device
2. **AC2:** Given USB CDC is enumerated, when I open a serial connection, then I can connect at 115200 baud (default)
3. **AC3:** Given a serial connection is open, when I type characters in the terminal, then characters are received by the firmware within 10ms
4. **AC4:** Given a serial connection is open, when firmware sends characters, then they appear in my terminal within 10ms
5. **AC5:** Given USB cable is disconnected during operation, when cable is reconnected, then device re-enumerates without requiring power cycle
6. **AC6:** Given USB re-enumeration occurs, when serial connection is reopened, then communication resumes normally
7. **AC7:** Given the USB CDC interface is operational, when tested with common terminal programs (minicom, screen, PuTTY, pyserial), then all work correctly
8. **AC8:** Given text is typed in terminal, when line is terminated with `\n` or `\r\n`, then the complete line is pushed to the command RX queue
9. **AC9:** Given text is added to TX queue, when usb_tx_task processes it, then text is sent to host with `\r\n` termination

## Tasks / Subtasks

- [ ] **Task 1: Create USB CDC Component Structure** (AC: 1)
  - [ ] Create `firmware/components/interface/usb_cdc/` directory
  - [ ] Create `CMakeLists.txt` with component dependencies (esp_tinyusb)
  - [ ] Create `usb_cdc.h` header with public API
  - [ ] Create `usb_cdc.c` implementation file
  - [ ] Create `usb_cdc_private.h` for internal types if needed

- [ ] **Task 2: Implement USB CDC Initialization** (AC: 1, 2)
  - [ ] Implement `usb_cdc_init()` function
  - [ ] Configure TinyUSB CDC descriptor (VID/PID can use ESP defaults)
  - [ ] Set up CDC ACM configuration (115200 baud, 8N1)
  - [ ] Create RX queue (LIMIT_COMMAND_QUEUE_DEPTH from config_limits.h)
  - [ ] Create TX queue (LIMIT_RESPONSE_QUEUE_DEPTH from config_limits.h)
  - [ ] Export queue handles as extern for task communication
  - [ ] Register USB event callbacks for connect/disconnect

- [ ] **Task 3: Implement usb_rx_task** (AC: 3, 8)
  - [ ] Read characters from USB CDC into line buffer
  - [ ] Accumulate until `\n` or `\r\n` detected
  - [ ] Strip trailing CR/LF characters
  - [ ] Push complete line to `usb_rx_queue`
  - [ ] Handle buffer overflow (discard partial line, log warning)
  - [ ] Use LIMIT_CMD_MAX_LENGTH (128) for line buffer size
  - [ ] Task priority: 10 on Core 0 (per architecture)

- [ ] **Task 4: Implement usb_tx_task** (AC: 4, 9)
  - [ ] Wait on `usb_tx_queue` with portMAX_DELAY
  - [ ] When item received, send via `tud_cdc_write()`
  - [ ] Append `\r\n` if not already present
  - [ ] Flush with `tud_cdc_write_flush()`
  - [ ] Task priority: 10 on Core 0 (per architecture)

- [ ] **Task 5: Implement Connection State Management** (AC: 5, 6)
  - [ ] Track DTR/RTS line state changes
  - [ ] Handle USB suspend/resume events
  - [ ] Clear RX buffer on disconnect
  - [ ] Implement `usb_cdc_is_connected()` function
  - [ ] Log connect/disconnect events at INFO level

- [ ] **Task 6: Create Public API Functions** (AC: 3, 4, 7)
  - [ ] `usb_cdc_send(const char* data, size_t len)` - raw send
  - [ ] `usb_cdc_send_line(const char* line)` - send with `\r\n`
  - [ ] Both return ESP_OK/ESP_FAIL
  - [ ] Both push to TX queue (non-blocking with timeout)

- [ ] **Task 7: Integrate with Main Application** (AC: 1-9)
  - [ ] Call `usb_cdc_init()` from `app_main()` before task creation
  - [ ] Verify tasks are created on Core 0
  - [ ] Ensure USB enumeration happens in boot sequence
  - [ ] Update task list in boot log if needed

- [ ] **Task 8: Test with Terminal Programs** (AC: 7)
  - [ ] Test with `idf.py monitor` (uses pyserial internally)
  - [ ] Test with `screen /dev/cu.usbmodem* 115200`
  - [ ] Document device path on macOS (`/dev/cu.usbmodem*`)
  - [ ] Verify bidirectional echo works

## Dev Notes

### Architecture Constraints

> **USB CDC Configuration (from Tech Spec 2.1)**
>
> - Use ESP-IDF TinyUSB component with CDC class
> - Line-based input (commands terminated by `\n` or `\r\n`)
> - RX queue: LIMIT_COMMAND_QUEUE_DEPTH items
> - TX queue: LIMIT_RESPONSE_QUEUE_DEPTH items
> - usb_rx_task and usb_tx_task run on Core 0 with priority 10
> - All responses terminated with `\r\n`
>
> [Source: docs/sprint-artifacts/tech-spec-epic-2.md#USB-CDC-API]

> **Performance Requirements (from Tech Spec NFR)**
>
> - Command response latency: <10ms
> - USB throughput: 115200 baud effective
> - No character loss under normal load
>
> [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Performance]

> **USB CDC API Contract**
>
> ```c
> esp_err_t usb_cdc_init(void);
> esp_err_t usb_cdc_send(const char* data, size_t len);
> esp_err_t usb_cdc_send_line(const char* line);  // Adds \r\n
> bool usb_cdc_is_connected(void);
>
> // Queue handles for task communication
> extern QueueHandle_t usb_rx_queue;  // Raw lines from USB
> extern QueueHandle_t usb_tx_queue;  // Formatted responses to USB
> ```
>
> [Source: docs/sprint-artifacts/tech-spec-epic-2.md#APIs-and-Interfaces]

### Task Configuration

| Task | Core | Priority | Stack | Purpose |
|------|------|----------|-------|---------|
| usb_rx_task | 0 | 10 | 4096 | Read USB → RX queue |
| usb_tx_task | 0 | 10 | 4096 | TX queue → USB |

### Buffer Sizes

| Constant | Value | Purpose |
|----------|-------|---------|
| LIMIT_CMD_MAX_LENGTH | 128 | Max command line length |
| LIMIT_COMMAND_QUEUE_DEPTH | 8 | RX queue depth |
| LIMIT_RESPONSE_QUEUE_DEPTH | 16 | TX queue depth |
| LIMIT_RESPONSE_MAX_LENGTH | 256 | Max response line length |

### Project Structure Notes

**New Files:**
```
firmware/components/interface/
└── usb_cdc/
    ├── CMakeLists.txt
    ├── include/
    │   └── usb_cdc.h
    ├── usb_cdc.c
    └── usb_cdc_private.h (optional)
```

**Existing Infrastructure:**
- `CONFIG_ESP_CONSOLE_USB_CDC=y` already configured in sdkconfig.defaults
- Epic 1 created stub usb_rx_task and usb_tx_task in main app
- This story implements the actual USB CDC component they will use

### Learnings from Previous Story

**From Story 1-7 (Status: review)**

- **EVENT BOOT**: Already implemented - sends `EVENT BOOT V1.0.0 AXES:8 STATE:IDLE`
- **USB CDC Works**: Verified in 1-7 that USB CDC enumerates and responds to HELP command
- **Task Creation**: All 16 tasks confirmed in boot log
- **Memory Budget**: Flash at 9%, DRAM at 19% - plenty of headroom
- **Serial Port**: Device at `/dev/cu.wchusbserial5A7A0286681` for flash, USB CDC at `/dev/cu.usbmodem*`

**Key Implementation Notes from 1-7:**
- The USB CDC basic functionality exists but needs to be refactored into proper component
- Current implementation may be minimal - this story formalizes the API and queue structure
- Existing code uses ESP-IDF console, this story creates dedicated usb_cdc component

[Source: docs/sprint-artifacts/1-7-build-verification-documentation.md#Dev-Agent-Record]

### ESP-IDF TinyUSB Integration

```c
// Required includes
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

// Initialization pattern
tinyusb_config_t tusb_cfg = {
    .device_descriptor = NULL,  // Use default
    .string_descriptor = NULL,  // Use default
    .external_phy = false,
};

tinyusb_config_cdcacm_t cdc_cfg = {
    .usb_dev = TINYUSB_USBDEV_0,
    .cdc_port = TINYUSB_CDC_ACM_0,
    .rx_unread_buf_sz = 256,
    .callback_rx = &cdc_rx_callback,
    .callback_rx_wanted_char = NULL,
    .callback_line_state_changed = &cdc_line_state_callback,
    .callback_line_coding_changed = NULL,
};
```

### References

- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#USB-CDC-API] - USB CDC API specification
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Workflows-and-Sequencing] - Command processing flow
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Performance] - Performance requirements
- [Source: docs/epics.md#Story-2.1] - Story definition
- [Source: docs/architecture.md#Peripheral-Usage] - USB peripheral assignment
- [Source: docs/sprint-artifacts/1-7-build-verification-documentation.md#Dev-Agent-Record] - Previous story learnings

---

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/stories/2-1-usb-cdc-serial-interface.context.xml

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List

### File List

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-04 | SM Agent (Bob) | Initial story draft from Epic 2 Tech Spec |
