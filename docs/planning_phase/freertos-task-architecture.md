# FreeRTOS Task Architecture

## Overview
Multi-tasking architecture for yarobot_control_unit using FreeRTOS. Designed for real-time motor control with proper priority management and inter-task communication.

## Task List and Priorities

```cpp
// FreeRTOS priority levels (0-31, higher number = higher priority)
#define PRIORITY_EMERGENCY_ISR    31  // ISR level (not a task)
#define PRIORITY_MOTION_CONTROL   25  // RMT/MCPWM updates
#define PRIORITY_SAFETY_MONITOR   24  // E-stop, limits, faults
#define PRIORITY_USB_RX           20  // Command reception
#define PRIORITY_USB_TX           19  // Response transmission
#define PRIORITY_COMMAND_EXECUTOR 15  // Command processing
#define PRIORITY_I2C_MONITOR      10  // I2C health checks
#define PRIORITY_DISPLAY_UPDATE   5   // OLED updates
#define PRIORITY_IDLE_MONITOR     3   // Motor idle timeout
#define PRIORITY_DIAGNOSTICS      2   // Status logging
```

## Core Tasks

### 1. Emergency Stop ISR (GPIO Interrupt)
Not a FreeRTOS task - runs in interrupt context for fastest response.

```cpp
// GPIO ISR configuration
static void IRAM_ATTR emergency_stop_isr(void* arg) {
    // Immediate hardware stop - no FreeRTOS calls
    
    // Stop all RMT channels
    RMT.conf_ch[0].conf1.tx_start = 0;
    RMT.conf_ch[1].conf1.tx_start = 0;
    RMT.conf_ch[2].conf1.tx_start = 0;
    RMT.conf_ch[3].conf1.tx_start = 0;
    
    // Stop MCPWM timers
    MCPWM0.timer[0].mode.start = 0;
    MCPWM0.timer[1].mode.start = 0;
    
    // Notify safety task via direct-to-task notification
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(safety_monitor_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void configureEmergencyStop() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // Active low
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_NUM_12),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_12, emergency_stop_isr, NULL);
}
```

### 2. Safety Monitor Task
Handles all safety-related events and error management.

```cpp
void safetyMonitorTask(void* pvParameters) {
    uint32_t notification_value;
    EventBits_t safety_events;
    
    // Event bits
    #define EVENT_EMERGENCY_STOP    (1 << 0)
    #define EVENT_LIMIT_SWITCH      (1 << 1)
    #define EVENT_I2C_FAILURE      (1 << 2)
    #define EVENT_POSITION_ERROR   (1 << 3)
    
    while (1) {
        // Wait for safety events (blocks until event)
        if (xTaskNotifyWait(0, ULONG_MAX, &notification_value, portMAX_DELAY)) {
            
            if (notification_value & EVENT_EMERGENCY_STOP) {
                ESP_LOGE(TAG, "EMERGENCY STOP ACTIVATED");
                
                // Disable all motors via I2C (if possible)
                for (int i = 0; i < 6; i++) {
                    i2c_expander_clear_pin(0x20, i);  // Motor enables
                }
                
                // Set system error state
                ErrorManager::getInstance().triggerCriticalError(
                    CriticalError::EMERGENCY_STOP
                );
                
                // Notify other tasks
                xEventGroupSetBits(system_event_group, EVENT_SYSTEM_HALT);
            }
            
            if (notification_value & EVENT_LIMIT_SWITCH) {
                // Process limit switch interrupt
                processLimitSwitches();
            }
        }
    }
}
```

### 3. USB RX Task (Command Reception)
High priority to avoid losing commands.

```cpp
void usbRxTask(void* pvParameters) {
    uint8_t rx_buffer[64];
    LineBuffer line_buffer;
    
    // Set CPU affinity to Core 0
    vTaskCoreAffinitySet(NULL, (1 << 0));
    
    while (1) {
        // Read from USB CDC (blocks until data available)
        int len = usb_cdc_read(rx_buffer, sizeof(rx_buffer), portMAX_DELAY);
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (line_buffer.addChar(rx_buffer[i])) {
                    // Complete command received
                    Command cmd = CommandParser::parse(line_buffer.getLine());
                    
                    if (cmd.type != Command::INVALID) {
                        // Send to executor queue
                        if (xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
                            // Queue full - send error response
                            Response resp = {"ERROR E009 Command queue full"};
                            xQueueSend(response_queue, &resp, 0);
                        }
                    }
                    
                    line_buffer.reset();
                }
            }
        }
    }
}
```

### 4. Command Executor Task
Processes commands from queue, manages motor operations.

```cpp
void commandExecutorTask(void* pvParameters) {
    Command cmd;
    
    // Set CPU affinity to Core 1 (separate from USB)
    vTaskCoreAffinitySet(NULL, (1 << 1));
    
    while (1) {
        // Wait for command (blocks)
        if (xQueueReceive(command_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            
            // Check system state
            EventBits_t events = xEventGroupGetBits(system_event_group);
            if (events & EVENT_SYSTEM_HALT) {
                // Only RST command allowed during halt
                if (cmd.type != Command::RST) {
                    Response resp = {"ERROR E006 Emergency stop active"};
                    xQueueSend(response_queue, &resp, 0);
                    continue;
                }
            }
            
            // Execute command
            Response resp = CommandExecutor::execute(cmd);
            
            // Send response
            xQueueSend(response_queue, &resp, pdMS_TO_TICKS(10));
        }
    }
}
```

### 5. USB TX Task
Sends responses back to host.

```cpp
void usbTxTask(void* pvParameters) {
    Response resp;
    
    while (1) {
        // Wait for response (blocks)
        if (xQueueReceive(response_queue, &resp, portMAX_DELAY) == pdTRUE) {
            // Add line ending
            strcat(resp.message, "\r\n");
            
            // Send via USB CDC
            usb_cdc_write((uint8_t*)resp.message, strlen(resp.message));
        }
    }
}
```

### 6. I2C Monitor Task
Periodic health checks and I2C communication management.

```cpp
void i2cMonitorTask(void* pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);  // 100ms period
    
    while (1) {
        // Wait for next period
        vTaskDelayUntil(&last_wake_time, period);
        
        // Check I2C health
        for (int i = 0; i < 2; i++) {
            uint8_t addr = (i == 0) ? 0x20 : 0x21;
            uint16_t data;
            
            if (i2c_read_word(addr, &data) != ESP_OK) {
                expander_error_count[i]++;
                
                if (expander_error_count[i] > 10) {
                    // Notify safety task
                    xTaskNotify(safety_monitor_handle, 
                               EVENT_I2C_FAILURE, eSetBits);
                }
            } else {
                expander_error_count[i] = 0;
            }
        }
        
        // Update cached I2C states
        updateI2CCache();
    }
}
```

### 7. Motion Control Tasks
One task per RMT channel for complex profiles, shared task for MCPWM.

```cpp
// RMT completion callback
static bool IRAM_ATTR rmt_done_callback(
    rmt_channel_handle_t channel,
    const rmt_tx_done_event_data_t *edata,
    void *user_data
) {
    uint8_t axis = (uint8_t)(uintptr_t)user_data;
    
    // Notify motion complete
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(motion_task_handles[axis],
                       MOTION_COMPLETE, eSetBits,
                       &xHigherPriorityTaskWoken);
    
    return xHigherPriorityTaskWoken == pdTRUE;
}

void motionControlTask(void* pvParameters) {
    uint8_t axis = (uint8_t)(uintptr_t)pvParameters;
    uint32_t notification;
    
    while (1) {
        // Wait for motion command or completion
        if (xTaskNotifyWait(0, ULONG_MAX, &notification, portMAX_DELAY)) {
            
            if (notification & MOTION_START) {
                // Start RMT transmission
                rmt_transmit(rmt_channels[axis], 
                           motion_profiles[axis],
                           profile_lengths[axis],
                           &tx_config);
            }
            
            if (notification & MOTION_COMPLETE) {
                // Update position
                motors[axis]->updatePosition();
                
                // Check position complete signal
                if (gpio_get_level(pos_complete_pins[axis])) {
                    motors[axis]->setPositionComplete(true);
                }
            }
        }
    }
}
```

### 8. Display Update Task
Low priority OLED updates.

```cpp
void displayUpdateTask(void* pvParameters) {
    char display_buffer[128];
    TickType_t last_update = 0;
    const TickType_t update_period = pdMS_TO_TICKS(100);  // 10Hz
    
    while (1) {
        vTaskDelayUntil(&last_update, update_period);
        
        // Build display content
        oled_clear();
        
        // Show active axes and positions
        for (int i = 0; i < 6; i++) {
            if (motors[i]->isEnabled()) {
                float pos = motors[i]->getCurrentPosition();
                float tgt = motors[i]->getTargetPosition();
                
                snprintf(display_buffer, sizeof(display_buffer),
                        "%d: C:%.1f T:%.1f %s",
                        i, pos, tgt,
                        motors[i]->isMoving() ? ">" : "=");
                
                oled_print_line(i, display_buffer);
            }
        }
        
        // Show any active errors
        if (ErrorManager::getInstance().hasErrors()) {
            oled_print_line(7, "ERROR!");
        }
        
        oled_update();
    }
}
```

### 9. Idle Monitor Task
Handles motor idle timeout and power saving.

```cpp
void idleMonitorTask(void* pvParameters) {
    // Configurable timeout (default 5 minutes)
    uint32_t idle_timeout_ms = 300000;
    uint32_t last_activity_time[6] = {0};
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Check every second
        
        uint32_t now = xTaskGetTickCount();
        
        for (int i = 0; i < 6; i++) {
            if (motors[i]->isEnabled() && !motors[i]->isMoving()) {
                if ((now - last_activity_time[i]) > pdMS_TO_TICKS(idle_timeout_ms)) {
                    // Disable motor to save power
                    ESP_LOGI(TAG, "Axis %d idle timeout - disabling", i);
                    motors[i]->disable();
                    
                    // Notify via response
                    Response resp;
                    snprintf(resp.message, sizeof(resp.message),
                            "INFO: Axis %d disabled due to idle timeout", i);
                    xQueueSend(response_queue, &resp, 0);
                }
            } else {
                // Update activity time
                last_activity_time[i] = now;
            }
        }
    }
}

// Command to set idle timeout
void setIdleTimeout(uint32_t timeout_seconds) {
    if (timeout_seconds == 0) {
        idle_timeout_ms = UINT32_MAX;  // Effectively disable
    } else {
        idle_timeout_ms = timeout_seconds * 1000;
    }
}
```

## Inter-Task Communication

### 1. Queues

```cpp
// Command flow
QueueHandle_t command_queue;      // USB RX -> Executor
QueueHandle_t response_queue;     // Executor -> USB TX

// Safety events
QueueHandle_t safety_event_queue; // ISRs -> Safety Monitor

// Create queues
void createQueues() {
    command_queue = xQueueCreate(32, sizeof(Command));
    response_queue = xQueueCreate(32, sizeof(Response));
    safety_event_queue = xQueueCreate(64, sizeof(SafetyEvent));
    
    configASSERT(command_queue != NULL);
    configASSERT(response_queue != NULL);
    configASSERT(safety_event_queue != NULL);
}
```

### 2. Event Groups

```cpp
EventGroupHandle_t system_event_group;

// Event bits
#define EVENT_SYSTEM_READY    (1 << 0)
#define EVENT_SYSTEM_HALT     (1 << 1)
#define EVENT_CALIBRATING     (1 << 2)
#define EVENT_USB_CONNECTED   (1 << 3)
#define EVENT_I2C_READY       (1 << 4)

void createEventGroups() {
    system_event_group = xEventGroupCreate();
    configASSERT(system_event_group != NULL);
}
```

### 3. Mutexes/Semaphores

```cpp
// Resource protection
SemaphoreHandle_t i2c_mutex;        // I2C bus access
SemaphoreHandle_t position_mutex[6]; // Per-axis position data

void createMutexes() {
    i2c_mutex = xSemaphoreCreateMutex();
    
    for (int i = 0; i < 6; i++) {
        position_mutex[i] = xSemaphoreCreateMutex();
    }
}
```

## Task Creation and Stack Sizes

```cpp
void createTasks() {
    BaseType_t ret;
    
    // Safety Monitor (Core 0)
    ret = xTaskCreatePinnedToCore(
        safetyMonitorTask,
        "safety",
        4096,  // Stack size
        NULL,
        PRIORITY_SAFETY_MONITOR,
        &safety_monitor_handle,
        0      // Core 0
    );
    configASSERT(ret == pdPASS);
    
    // USB RX (Core 0)
    ret = xTaskCreatePinnedToCore(
        usbRxTask,
        "usb_rx",
        2048,
        NULL,
        PRIORITY_USB_RX,
        &usb_rx_handle,
        0
    );
    
    // Command Executor (Core 1)
    ret = xTaskCreatePinnedToCore(
        commandExecutorTask,
        "cmd_exec",
        8192,  // Larger stack for complex operations
        NULL,
        PRIORITY_COMMAND_EXECUTOR,
        &cmd_executor_handle,
        1      // Core 1
    );
    
    // USB TX (Core 1)
    ret = xTaskCreatePinnedToCore(
        usbTxTask,
        "usb_tx",
        2048,
        NULL,
        PRIORITY_USB_TX,
        &usb_tx_handle,
        1
    );
    
    // I2C Monitor (Core 0)
    ret = xTaskCreate(
        i2cMonitorTask,
        "i2c_mon",
        2048,
        NULL,
        PRIORITY_I2C_MONITOR,
        &i2c_monitor_handle
    );
    
    // Motion Control Tasks (4 for RMT)
    for (int i = 0; i < 4; i++) {
        char name[16];
        snprintf(name, sizeof(name), "motion_%d", i);
        
        ret = xTaskCreatePinnedToCore(
            motionControlTask,
            name,
            2048,
            (void*)(uintptr_t)i,
            PRIORITY_MOTION_CONTROL,
            &motion_task_handles[i],
            1  // Core 1 for motion
        );
    }
    
    // Display Update (any core)
    ret = xTaskCreate(
        displayUpdateTask,
        "display",
        4096,
        NULL,
        PRIORITY_DISPLAY_UPDATE,
        &display_handle
    );
    
    // Idle Monitor
    ret = xTaskCreate(
        idleMonitorTask,
        "idle_mon",
        2048,
        NULL,
        PRIORITY_IDLE_MONITOR,
        NULL
    );
}
```

## Memory Management

### Stack Usage Monitoring

```cpp
void printTaskStats() {
    char stats_buffer[1024];
    vTaskList(stats_buffer);
    ESP_LOGI(TAG, "Task Stats:\n%s", stats_buffer);
    
    // Get runtime stats
    vTaskGetRunTimeStats(stats_buffer);
    ESP_LOGI(TAG, "Runtime Stats:\n%s", stats_buffer);
}

// Stack overflow hook
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    ESP_LOGE(TAG, "Stack overflow in task: %s", pcTaskName);
    
    // Trigger emergency stop
    emergency_stop_isr(NULL);
    
    // System will reset via watchdog
    while(1);
}
```

### Heap Management

```cpp
// Monitor heap usage
void heapMonitorTask(void* pvParameters) {
    while (1) {
        size_t free_heap = esp_get_free_heap_size();
        size_t min_free = esp_get_minimum_free_heap_size();
        
        if (free_heap < 10000) {  // Less than 10KB free
            ESP_LOGW(TAG, "Low heap: %d bytes free (min: %d)", 
                    free_heap, min_free);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000));  // Check every 10s
    }
}
```

## CPU Core Assignment Strategy

### Core 0 (Protocol/Safety)
- Safety Monitor (highest priority)
- USB RX (command reception)
- I2C Monitor
- WiFi (future)

### Core 1 (Motion/Control)
- Command Executor
- Motion Control Tasks
- USB TX
- Display updates

This separation ensures motion control isn't affected by communication overhead.