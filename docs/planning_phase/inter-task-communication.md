# Inter-Task Communication Design

## Overview
Detailed design of queues, events, and notifications for task communication in the yarobot_control_unit. Emphasis on lock-free, deterministic communication patterns.

## Communication Architecture

```
┌─────────────┐     Commands      ┌──────────────┐
│   USB RX    │ ──────────────> │   Command    │
│    Task     │                  │    Queue     │
└─────────────┘                  └──────┬───────┘
                                        │
┌─────────────┐     Safety Events       v
│   Safety    │ <────────────── ┌──────────────┐
│  Monitor    │                 │   Command    │
└──────┬──────┘                 │  Executor    │
       │                        └──────┬───────┘
       │ System Events                 │
       v                               v
┌─────────────┐                ┌──────────────┐
│Event Groups │                │  Response    │
│             │                │    Queue     │
└─────────────┘                └──────┬───────┘
                                      │
                              ┌───────v──────┐
                              │   USB TX     │
                              │    Task      │
                              └──────────────┘
```

## 1. Queue Definitions

### Command Queue (USB RX → Command Executor)

```cpp
// Command structure optimized for queue efficiency
typedef struct {
    uint8_t type;           // Command type enum
    uint8_t axis;           // 0-5 or 0xFF for ALL
    uint8_t flags;          // EMERGENCY, etc.
    uint8_t param_count;    // Number of float params
    float params[4];        // Up to 4 parameters
    uint32_t timestamp;     // When received
} Command_t;

// Queue creation
#define COMMAND_QUEUE_LENGTH    32
#define COMMAND_QUEUE_SIZE      sizeof(Command_t)

QueueHandle_t command_queue;

void createCommandQueue() {
    command_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, COMMAND_QUEUE_SIZE);
    if (command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create command queue");
        abort();
    }
    
    // Add queue to registry for debugging
    vQueueAddToRegistry(command_queue, "CmdQueue");
}

// Producer (USB RX Task)
void sendCommand(const Command_t* cmd) {
    if (xQueueSend(command_queue, cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
        // Queue full - send immediate error response
        Response_t resp = {
            .status = RESPONSE_ERROR,
            .error_code = ERROR_QUEUE_FULL,
        };
        strcpy(resp.message, "ERROR E009 Command queue full");
        xQueueSend(response_queue, &resp, 0);
    }
}

// Consumer (Command Executor)
bool receiveCommand(Command_t* cmd, TickType_t timeout) {
    return xQueueReceive(command_queue, cmd, timeout) == pdTRUE;
}
```

### Response Queue (Multiple Producers → USB TX)

```cpp
typedef struct {
    uint8_t status;         // OK, ERROR, INFO, STREAM
    uint8_t error_code;     // If status = ERROR
    char message[256];      // Response text
    bool needs_ack;         // For critical responses
} Response_t;

#define RESPONSE_QUEUE_LENGTH   32

QueueHandle_t response_queue;

// Thread-safe response sending
void sendResponse(const char* format, ...) {
    Response_t resp = {0};
    va_list args;
    
    va_start(args, format);
    vsnprintf(resp.message, sizeof(resp.message), format, args);
    va_end(args);
    
    // Determine status from message
    if (strncmp(resp.message, "OK", 2) == 0) {
        resp.status = RESPONSE_OK;
    } else if (strncmp(resp.message, "ERROR", 5) == 0) {
        resp.status = RESPONSE_ERROR;
    }
    
    xQueueSend(response_queue, &resp, pdMS_TO_TICKS(100));
}
```

### Safety Event Queue (ISRs → Safety Monitor)

```cpp
typedef struct {
    uint8_t event_type;     // LIMIT, ESTOP, FAULT, etc.
    uint8_t axis;           // Affected axis (if applicable)
    uint16_t gpio_state;    // Current GPIO states
    uint32_t timestamp;     // When detected
} SafetyEvent_t;

#define SAFETY_QUEUE_LENGTH     64  // Larger for burst events

QueueHandle_t safety_queue;

// ISR-safe sending
static BaseType_t IRAM_ATTR sendSafetyEventFromISR(SafetyEvent_t* event) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    event->timestamp = esp_timer_get_time();
    xQueueSendFromISR(safety_queue, event, &xHigherPriorityTaskWoken);
    
    return xHigherPriorityTaskWoken;
}
```

## 2. Event Group Design

### System Event Group

```cpp
// Event bit definitions
#define SYS_EVENT_READY           (1 << 0)   // System initialized
#define SYS_EVENT_EMERGENCY_STOP  (1 << 1)   // E-stop active
#define SYS_EVENT_FAULT           (1 << 2)   // Any fault active
#define SYS_EVENT_CALIBRATING     (1 << 3)   // Calibration in progress
#define SYS_EVENT_USB_CONNECTED   (1 << 4)   // USB host connected
#define SYS_EVENT_I2C_OK          (1 << 5)   // I2C bus healthy
#define SYS_EVENT_MOTION_ACTIVE   (1 << 6)   // Any axis moving
#define SYS_EVENT_IDLE_TIMEOUT    (1 << 7)   // Idle timeout occurred

EventGroupHandle_t system_events;

// Helper functions for clean code
bool isSystemReady() {
    EventBits_t bits = xEventGroupGetBits(system_events);
    return (bits & SYS_EVENT_READY) && 
           !(bits & (SYS_EVENT_EMERGENCY_STOP | SYS_EVENT_FAULT));
}

bool isEmergencyStopped() {
    return xEventGroupGetBits(system_events) & SYS_EVENT_EMERGENCY_STOP;
}

// Wait for system ready with timeout
bool waitForSystemReady(TickType_t timeout) {
    EventBits_t bits = xEventGroupWaitBits(
        system_events,
        SYS_EVENT_READY | SYS_EVENT_I2C_OK,  // Wait for these
        pdFALSE,                              // Don't clear on exit
        pdTRUE,                               // Wait for all bits
        timeout
    );
    
    return (bits & (SYS_EVENT_READY | SYS_EVENT_I2C_OK)) == 
           (SYS_EVENT_READY | SYS_EVENT_I2C_OK);
}
```

### Motion Event Group (Per Axis)

```cpp
// Motion events per axis
#define MOTION_EVENT_START        (1 << 0)   // Start motion
#define MOTION_EVENT_COMPLETE     (1 << 1)   // Motion finished
#define MOTION_EVENT_ABORT        (1 << 2)   // Abort current motion
#define MOTION_EVENT_LIMIT_HIT    (1 << 3)   // Limit switch triggered
#define MOTION_EVENT_FAULT        (1 << 4)   // Motor fault
#define MOTION_EVENT_CALIBRATE    (1 << 5)   // Start calibration
#define MOTION_EVENT_POS_COMPLETE (1 << 6)   // Position complete signal

EventGroupHandle_t motion_events[6];  // One per axis

// Motion control signaling
void startMotion(uint8_t axis) {
    xEventGroupSetBits(motion_events[axis], MOTION_EVENT_START);
}

void abortMotion(uint8_t axis) {
    xEventGroupSetBits(motion_events[axis], MOTION_EVENT_ABORT);
}

// Motion task waiting
void motionTask(void* param) {
    uint8_t axis = (uint8_t)(uintptr_t)param;
    
    while (1) {
        EventBits_t events = xEventGroupWaitBits(
            motion_events[axis],
            MOTION_EVENT_START | MOTION_EVENT_ABORT | MOTION_EVENT_CALIBRATE,
            pdTRUE,      // Clear bits on exit
            pdFALSE,     // Wait for any bit
            portMAX_DELAY
        );
        
        if (events & MOTION_EVENT_START) {
            executeMotionProfile(axis);
        }
        
        if (events & MOTION_EVENT_ABORT) {
            stopMotionImmediately(axis);
        }
        
        if (events & MOTION_EVENT_CALIBRATE) {
            executeCalibration(axis);
        }
    }
}
```

## 3. Task Notifications

### Direct-to-Task Notifications

```cpp
// Safety monitor notifications
#define NOTIFY_EMERGENCY_STOP     (1 << 0)
#define NOTIFY_LIMIT_SWITCH       (1 << 1)
#define NOTIFY_I2C_CHECK          (1 << 2)
#define NOTIFY_WATCHDOG_FEED      (1 << 3)

// E-stop ISR using direct notification
static void IRAM_ATTR emergency_stop_isr(void* arg) {
    // Hardware stop (as before)
    stopAllMotionHardware();
    
    // Direct notification - faster than queue
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(safety_monitor_handle,
                       NOTIFY_EMERGENCY_STOP,
                       eSetBits,
                       &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Safety monitor receiving notifications
void safetyMonitorTask(void* param) {
    uint32_t notification_value;
    
    while (1) {
        // Wait for any notification
        if (xTaskNotifyWait(0, ULONG_MAX, &notification_value, 
                           pdMS_TO_TICKS(1000)) == pdTRUE) {
            
            if (notification_value & NOTIFY_EMERGENCY_STOP) {
                handleEmergencyStop();
            }
            
            if (notification_value & NOTIFY_LIMIT_SWITCH) {
                processLimitSwitches();
            }
            
            if (notification_value & NOTIFY_I2C_CHECK) {
                performI2CHealthCheck();
            }
        } else {
            // Timeout - periodic checks
            feedWatchdog();
        }
    }
}
```

### Motion Completion Notifications

```cpp
// RMT completion callback with notification
static bool IRAM_ATTR rmt_tx_done_callback(
    rmt_channel_handle_t channel,
    const rmt_tx_done_event_data_t *edata,
    void *user_data
) {
    uint8_t axis = (uint8_t)(uintptr_t)user_data;
    
    // Notify motion task
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(motion_task_handles[axis],
                       axis,  // Send axis number as notification value
                       eSetValueWithOverwrite,
                       &xHigherPriorityTaskWoken);
    
    return xHigherPriorityTaskWoken == pdTRUE;
}
```

## 4. Mutex Usage

### I2C Bus Mutex

```cpp
SemaphoreHandle_t i2c_mutex;

// Thread-safe I2C access
esp_err_t i2c_write_safe(uint8_t addr, uint16_t data) {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire I2C mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t ret = i2c_master_write_to_device(
        I2C_NUM_0, addr, (uint8_t*)&data, 2,
        pdMS_TO_TICKS(10)
    );
    
    xSemaphoreGive(i2c_mutex);
    return ret;
}
```

### Position Data Protection

```cpp
// Per-axis position data structure
typedef struct {
    float current_position;
    float target_position;
    float velocity;
    bool is_moving;
    SemaphoreHandle_t mutex;
} AxisPosition_t;

AxisPosition_t axis_positions[6];

// Thread-safe position update
void updatePosition(uint8_t axis, float new_position) {
    if (xSemaphoreTake(axis_positions[axis].mutex, portMAX_DELAY) == pdTRUE) {
        axis_positions[axis].current_position = new_position;
        xSemaphoreGive(axis_positions[axis].mutex);
    }
}

// Thread-safe position read
float getPosition(uint8_t axis) {
    float position = 0;
    
    if (xSemaphoreTake(axis_positions[axis].mutex, portMAX_DELAY) == pdTRUE) {
        position = axis_positions[axis].current_position;
        xSemaphoreGive(axis_positions[axis].mutex);
    }
    
    return position;
}
```

## 5. Message Pools (For Complex Data)

### Motion Profile Pool

```cpp
// Pool for motion profile data
typedef struct {
    uint8_t axis;
    float distance;
    float max_velocity;
    float acceleration;
    rmt_symbol_word_t symbols[1024];
    size_t symbol_count;
} MotionProfile_t;

#define PROFILE_POOL_SIZE   8  // Support 8 queued profiles

StaticQueue_t profile_pool_storage;
uint8_t profile_pool_buffer[PROFILE_POOL_SIZE * sizeof(MotionProfile_t*)];
QueueHandle_t profile_pool;

MotionProfile_t profile_instances[PROFILE_POOL_SIZE];

void createProfilePool() {
    // Create pool as queue of pointers
    profile_pool = xQueueCreateStatic(
        PROFILE_POOL_SIZE,
        sizeof(MotionProfile_t*),
        profile_pool_buffer,
        &profile_pool_storage
    );
    
    // Initialize pool with profile instances
    for (int i = 0; i < PROFILE_POOL_SIZE; i++) {
        MotionProfile_t* profile = &profile_instances[i];
        xQueueSend(profile_pool, &profile, 0);
    }
}

// Allocate profile from pool
MotionProfile_t* allocateProfile(TickType_t timeout) {
    MotionProfile_t* profile = NULL;
    xQueueReceive(profile_pool, &profile, timeout);
    return profile;
}

// Return profile to pool
void freeProfile(MotionProfile_t* profile) {
    xQueueSend(profile_pool, &profile, 0);
}
```

## 6. Streaming Data Management

### Position Stream Buffer

```cpp
// Ring buffer for streaming position data
typedef struct {
    uint32_t timestamp;
    float positions[6];
} PositionSnapshot_t;

#define STREAM_BUFFER_SIZE  (sizeof(PositionSnapshot_t) * 100)

StreamBufferHandle_t position_stream;

void createStreamBuffer() {
    position_stream = xStreamBufferCreate(STREAM_BUFFER_SIZE, 
                                         sizeof(PositionSnapshot_t));
}

// Producer (sampling task)
void positionSamplingTask(void* param) {
    PositionSnapshot_t snapshot;
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(stream_interval_ms));
        
        if (streaming_enabled) {
            snapshot.timestamp = esp_timer_get_time();
            
            // Collect all positions
            for (int i = 0; i < 6; i++) {
                snapshot.positions[i] = getPosition(i);
            }
            
            // Send to stream buffer
            xStreamBufferSend(position_stream, &snapshot, 
                            sizeof(snapshot), 0);
        }
    }
}

// Consumer (USB TX)
void sendStreamData() {
    PositionSnapshot_t snapshot;
    size_t received;
    
    received = xStreamBufferReceive(position_stream, &snapshot,
                                  sizeof(snapshot), 0);
    
    if (received == sizeof(snapshot)) {
        char buffer[256];
        int len = snprintf(buffer, sizeof(buffer), "STRM ");
        
        for (int i = 0; i < 6; i++) {
            if (stream_axis_mask & (1 << i)) {
                len += snprintf(buffer + len, sizeof(buffer) - len,
                              "%d:%.2f,", i, snapshot.positions[i]);
            }
        }
        
        buffer[len-1] = '\n';  // Replace last comma
        usb_cdc_write(buffer, len);
    }
}
```

## 7. Communication Patterns

### Command-Response Pattern

```cpp
// Synchronous command execution with response
void executeCommandWithResponse(Command_t* cmd) {
    Response_t response = {0};
    
    // Execute command
    esp_err_t result = executeCommand(cmd, &response);
    
    if (result == ESP_OK) {
        response.status = RESPONSE_OK;
    } else {
        response.status = RESPONSE_ERROR;
        response.error_code = result;
    }
    
    // Always send response
    xQueueSend(response_queue, &response, portMAX_DELAY);
}
```

### Event-Driven Pattern

```cpp
// Motion complete event propagation
void onMotionComplete(uint8_t axis) {
    // Update motion events
    xEventGroupSetBits(motion_events[axis], MOTION_EVENT_COMPLETE);
    
    // Clear system motion bit if no axes moving
    if (!anyAxisMoving()) {
        xEventGroupClearBits(system_events, SYS_EVENT_MOTION_ACTIVE);
    }
    
    // Send completion response if needed
    if (motion_needs_response[axis]) {
        sendResponse("OK AXIS:%d COMPLETE", axis);
        motion_needs_response[axis] = false;
    }
}
```

### Producer-Consumer Pattern

```cpp
// Multi-producer response handling
void logResponse(const char* format, ...) {
    Response_t resp = {0};
    va_list args;
    
    va_start(args, format);
    int len = vsnprintf(resp.message, sizeof(resp.message), format, args);
    va_end(args);
    
    // Non-blocking send with drop on overflow
    if (xQueueSend(response_queue, &resp, 0) != pdTRUE) {
        // Increment drop counter for diagnostics
        response_drops++;
    }
}
```

## 8. Deadlock Prevention

### Lock Ordering

```cpp
// Always acquire locks in same order to prevent deadlock
// Order: i2c_mutex -> position_mutex[0..5] -> profile_pool

void complexOperation(uint8_t axis) {
    // Take mutexes in defined order
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    xSemaphoreTake(axis_positions[axis].mutex, portMAX_DELAY);
    
    // Do work...
    
    // Release in reverse order
    xSemaphoreGive(axis_positions[axis].mutex);
    xSemaphoreGive(i2c_mutex);
}
```

### Timeout Protection

```cpp
// Use timeouts to detect potential deadlocks
bool safeMutexTake(SemaphoreHandle_t mutex, const char* name) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex: %s", name);
        
        // Log task states for debugging
        char buffer[512];
        vTaskList(buffer);
        ESP_LOGE(TAG, "Task states:\n%s", buffer);
        
        return false;
    }
    return true;
}
```

## Testing Communication

```cpp
TEST_CASE("Queue overflow handling") {
    // Fill command queue
    Command_t cmd = { .type = CMD_MOVE };
    
    for (int i = 0; i < COMMAND_QUEUE_LENGTH + 5; i++) {
        xQueueSend(command_queue, &cmd, 0);
    }
    
    // Verify queue full
    TEST_ASSERT_EQUAL(COMMAND_QUEUE_LENGTH, 
                     uxQueueMessagesWaiting(command_queue));
}

TEST_CASE("Event group race conditions") {
    // Set/clear bits from multiple tasks
    // Verify no race conditions
}
```