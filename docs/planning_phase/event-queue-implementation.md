# Event Queue Implementation

## Overview
The YaRobot uses a FreeRTOS queue-based event system for inter-task communication. Events flow from producers (motion, I/O, sensors) to consumers (USB, OLED, state machines) through a central priority-aware queue.

## Event Structure

```c
typedef enum {
    // Motion events (0x00-0x1F)
    EVENT_MOTION_START = 0x00,
    EVENT_MOTION_COMPLETE,
    EVENT_POSITION_REACHED,
    EVENT_VELOCITY_ZERO,
    
    // Limit events (0x20-0x3F)
    EVENT_LIMIT_MIN,
    EVENT_LIMIT_MAX,
    EVENT_LIMIT_CLEARED,
    
    // I/O events (0x40-0x5F)
    EVENT_INPUT_CHANGED,
    EVENT_OUTPUT_SET,
    EVENT_BUTTON_PRESS,
    EVENT_BUTTON_RELEASE,
    
    // Servo feedback events (0x60-0x7F)
    EVENT_Z_SIGNAL,
    EVENT_INPOS_ACTIVE,
    EVENT_INPOS_LOST,
    EVENT_POSITION_ERROR,
    
    // System events (0x80-0x9F)
    EVENT_ESTOP_PRESSED,
    EVENT_ESTOP_CLEARED,
    EVENT_CONFIG_CHANGED,
    EVENT_MODE_CHANGED,
    
    // Error events (0xA0-0xBF)
    EVENT_I2C_ERROR,
    EVENT_MOTOR_FAULT,
    EVENT_TIMEOUT,
    EVENT_OVERTEMP,
    
    // Special events (0xC0-0xFF)
    EVENT_JAW_OBJECT_DETECTED,
    EVENT_CALIBRATION_COMPLETE,
    EVENT_HOMING_COMPLETE,
    EVENT_HEARTBEAT
} event_type_t;

typedef struct {
    event_type_t type;          // Event type
    uint8_t source_id;          // Axis number or I/O pin number
    union {
        uint32_t value;         // Generic value
        float position;         // Position for motion events
        struct {
            uint16_t old_state; // For input change events
            uint16_t new_state;
        } io;
        struct {
            uint8_t code;       // Error code
            uint8_t device;     // Device address/ID
        } error;
    } data;
    TickType_t timestamp;       // When event occurred
    char source_name[16];       // Alias of source (e.g., "railway")
} event_t;
```

## Queue Configuration

```c
// Queue parameters
#define EVENT_QUEUE_LENGTH      64
#define EVENT_QUEUE_ITEM_SIZE   sizeof(event_t)
#define EVENT_QUEUE_WAIT_TIME   pdMS_TO_TICKS(10)

// Priority levels for sending
#define EVENT_PRIORITY_CRITICAL 0    // Send to front (E-stop, limits)
#define EVENT_PRIORITY_NORMAL   1    // Send to back (normal operation)

// Global queue handle
QueueHandle_t g_event_queue;

// Queue sets for multiple consumers
QueueSetHandle_t g_event_queue_set;
```

## Event Producers

### Motion Task
```c
void motion_event_position_reached(axis_t* axis) {
    event_t event = {
        .type = EVENT_POSITION_REACHED,
        .source_id = axis->number,
        .data.position = axis->current_position,
        .timestamp = xTaskGetTickCount(),
    };
    strncpy(event.source_name, axis->alias, 15);
    
    xQueueSend(g_event_queue, &event, EVENT_QUEUE_WAIT_TIME);
}
```

### I/O Interrupt Handler
```c
void IRAM_ATTR gpio_isr_handler(void* arg) {
    io_pin_t* pin = (io_pin_t*)arg;
    event_t event = {
        .type = EVENT_INPUT_CHANGED,
        .source_id = pin->number,
        .timestamp = xTaskGetTickCountFromISR()
    };
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(g_event_queue, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

### Limit Switch Handler (Critical)
```c
void limit_switch_triggered(axis_t* axis, bool is_max) {
    event_t event = {
        .type = is_max ? EVENT_LIMIT_MAX : EVENT_LIMIT_MIN,
        .source_id = axis->number,
        .timestamp = xTaskGetTickCount()
    };
    strncpy(event.source_name, axis->alias, 15);
    
    // Critical event - send to front
    xQueueSendToFront(g_event_queue, &event, 0);
}
```

## Event Consumers

### USB Communication Task
```c
void usb_task(void* param) {
    event_t event;
    
    while (1) {
        if (xQueueReceive(g_event_queue, &event, portMAX_DELAY)) {
            // Format and send to USB
            usb_send_event(&event);
            
            // Forward to other consumers if needed
            forward_event_to_consumers(&event);
        }
    }
}

void usb_send_event(event_t* event) {
    char buffer[64];
    
    switch (event->type) {
        case EVENT_POSITION_REACHED:
            snprintf(buffer, sizeof(buffer), 
                    "EVENT POSITION_REACHED %s %.2f\n",
                    event->source_name, event->data.position);
            break;
            
        case EVENT_LIMIT_MIN:
            snprintf(buffer, sizeof(buffer),
                    "EVENT LIMIT_MIN %s\n",
                    event->source_name);
            break;
            
        // ... other event types
    }
    
    usb_write_string(buffer);
}
```

### OLED Display Consumer
```c
// Circular buffer for event history
typedef struct {
    event_t events[10];
    uint8_t head;
    uint8_t count;
    TickType_t last_event_time;
} event_history_t;

event_history_t g_event_history = {0};

void oled_event_consumer(event_t* event) {
    // Add to history
    g_event_history.events[g_event_history.head] = *event;
    g_event_history.head = (g_event_history.head + 1) % 10;
    if (g_event_history.count < 10) g_event_history.count++;
    g_event_history.last_event_time = event->timestamp;
    
    // Trigger display update
    oled_request_event_display();
}
```

### State Machine Consumer
```c
void state_machine_process_event(event_t* event) {
    switch (event->type) {
        case EVENT_ESTOP_PRESSED:
            enter_estop_state();
            break;
            
        case EVENT_LIMIT_MIN:
        case EVENT_LIMIT_MAX:
            if (axis_config[event->source_id].limit_action == LIMIT_ESTOP) {
                trigger_estop("Limit switch triggered");
            } else {
                stop_axis(event->source_id);
            }
            break;
            
        case EVENT_JAW_OBJECT_DETECTED:
            record_object_width(event->data.position);
            break;
    }
}
```

## Event Filtering and Routing

```c
// Event filter table
typedef struct {
    event_type_t type;
    bool to_usb;
    bool to_oled;
    bool to_state_machine;
    bool to_ros2;
} event_route_t;

const event_route_t event_routing[] = {
    // Type                    USB  OLED  SM   ROS2
    {EVENT_POSITION_REACHED,   true, true, true, true},
    {EVENT_LIMIT_MIN,          true, true, true, false},
    {EVENT_HEARTBEAT,          false,false,false,false},
    {EVENT_I2C_ERROR,          true, true, true, false},
    // ... etc
};

void forward_event_to_consumers(event_t* event) {
    const event_route_t* route = find_route(event->type);
    if (!route) return;
    
    if (route->to_usb) usb_queue_event(event);
    if (route->to_oled) oled_event_consumer(event);
    if (route->to_state_machine) state_machine_process_event(event);
    if (route->to_ros2) ros2_publish_event(event);
}
```

## Queue Management

### Initialization
```c
void event_queue_init(void) {
    // Create main event queue
    g_event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, EVENT_QUEUE_ITEM_SIZE);
    configASSERT(g_event_queue);
    
    // Add to queue registry for debugging
    vQueueAddToRegistry(g_event_queue, "EventQueue");
    
    // Create queue set for multiple consumers
    g_event_queue_set = xQueueCreateSet(EVENT_QUEUE_LENGTH);
    xQueueAddToSet(g_event_queue, g_event_queue_set);
}
```

### Overflow Handling
```c
void event_send_safe(event_t* event) {
    if (xQueueSend(g_event_queue, event, 0) != pdTRUE) {
        // Queue full - log and increment counter
        g_event_stats.queue_overflows++;
        
        // For critical events, force space
        if (is_critical_event(event->type)) {
            event_t dummy;
            xQueueReceive(g_event_queue, &dummy, 0);  // Remove oldest
            xQueueSend(g_event_queue, event, 0);      // Add new
        }
    }
}
```

### Statistics and Monitoring
```c
typedef struct {
    uint32_t events_sent;
    uint32_t events_received;
    uint32_t queue_overflows;
    uint32_t max_queue_depth;
    uint32_t events_by_type[256];  // Count per event type
} event_statistics_t;

event_statistics_t g_event_stats = {0};

void event_stats_update(event_t* event) {
    g_event_stats.events_sent++;
    g_event_stats.events_by_type[event->type]++;
    
    UBaseType_t current_depth = uxQueueMessagesWaiting(g_event_queue);
    if (current_depth > g_event_stats.max_queue_depth) {
        g_event_stats.max_queue_depth = current_depth;
    }
}
```

## Debug Commands

```bash
# Query event statistics
EVENT STATS
> EVENTS: SENT:1234 RECV:1234 OVERFLOW:0
> QUEUE: CUR:3 MAX:45 SIZE:64
> TOP: POS_REACH:423 LIM:89 ESTOP:2

# Monitor events in real-time
EVENT MONITOR ON
> EVENT MONITOR ACTIVE
> EVENT POSITION_REACHED railway 125.50
> EVENT INPUT_CHANGED start_button HIGH
> EVENT LIMIT_MIN selector

# Filter specific events
EVENT FILTER MOTION
> EVENT FILTER: MOTION ONLY
> EVENT MOTION_START railway
> EVENT POSITION_REACHED railway 125.50

# Clear event history
EVENT CLEAR
> EVENT HISTORY CLEARED
```

## Performance Considerations

1. **ISR Safety**: Events from ISRs use minimal processing
2. **Queue Size**: 64 events handles burst scenarios
3. **Timestamp**: TickType_t for minimal overhead
4. **Routing Table**: Compile-time for speed
5. **Critical Events**: Send to front for immediate handling

## Integration Notes

1. All tasks check event queue regularly
2. USB task has highest consumer priority
3. OLED updates triggered by events, not polling
4. State machine processes safety events first
5. Event history maintained for debugging