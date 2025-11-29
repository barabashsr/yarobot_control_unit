# Safety Interlocks and Fault Detection

## Overview
Comprehensive safety system design for yarobot_control_unit including hardware interlocks, software safety checks, fault detection, and recovery mechanisms to ensure safe operation.

## Safety Architecture

```
┌─────────────────────────────────────────────┐
│            HARDWARE INTERLOCKS              │
│  ┌─────────┐  ┌──────────┐  ┌───────────┐ │
│  │ E-Stop  │  │  Limits  │  │  Driver   │ │
│  │  GPIO   │  │   I2C    │  │  Faults   │ │
│  └────┬────┘  └─────┬────┘  └─────┬─────┘ │
└───────┼─────────────┼─────────────┼────────┘
        │             │             │
┌───────v─────────────v─────────────v────────┐
│           SAFETY MONITOR TASK               │
│  - Immediate response to hardware faults    │
│  - Coordinate safety actions                │
│  - Error state management                   │
└─────────────────┬───────────────────────────┘
                  │
┌─────────────────v───────────────────────────┐
│          SOFTWARE SAFETY CHECKS             │
│  - Position limits                          │
│  - Velocity limits                          │
│  - Following error                          │
│  - Communication timeout                    │
└─────────────────────────────────────────────┘
```

## Hardware Safety Interlocks

### 1. Emergency Stop (E-Stop)
```cpp
// Direct GPIO interrupt - fastest response
#define GPIO_E_STOP    GPIO_NUM_12
#define E_STOP_ACTIVE_LEVEL  0  // Active low

static void IRAM_ATTR emergency_stop_isr(void* arg) {
    // CRITICAL: Stop all motion immediately in hardware
    
    // 1. Stop all RMT channels (axes 0-3)
    RMT.conf_ch[0].conf1.tx_start = 0;
    RMT.conf_ch[1].conf1.tx_start = 0;
    RMT.conf_ch[2].conf1.tx_start = 0;
    RMT.conf_ch[3].conf1.tx_start = 0;
    
    // 2. Stop MCPWM timers (axes 4-5)
    MCPWM0.timer[0].mode.start = 0;
    MCPWM0.timer[1].mode.start = 0;
    
    // 3. Force all STEP outputs low
    GPIO.out_w1tc = (1 << GPIO_STEP_0) | (1 << GPIO_STEP_1) | 
                    (1 << GPIO_STEP_2) | (1 << GPIO_STEP_3) |
                    (1 << GPIO_STEP_4) | (1 << GPIO_STEP_5);
    
    // 4. Notify safety task (non-blocking)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(safety_monitor_handle, 
                       NOTIFY_EMERGENCY_STOP,
                       eSetBits,
                       &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// E-stop monitoring and recovery
void monitorEmergencyStop() {
    static bool estop_was_active = false;
    bool estop_active = (gpio_get_level(GPIO_E_STOP) == E_STOP_ACTIVE_LEVEL);
    
    if (estop_active && !estop_was_active) {
        // E-stop just activated
        ESP_LOGE(TAG, "EMERGENCY STOP ACTIVATED");
        system_state = STATE_EMERGENCY_STOP;
        
        // Disable all motors via I2C (secondary protection)
        for (int i = 0; i < 6; i++) {
            setMotorEnable(i, false);
        }
        
    } else if (!estop_active && estop_was_active) {
        // E-stop released - do NOT auto-restart
        ESP_LOGW(TAG, "E-stop released - use RST command to reset");
    }
    
    estop_was_active = estop_active;
}
```

### 2. Limit Switch Protection
```cpp
// Limit switches on I2C expander with interrupt
typedef struct {
    uint8_t axis;
    bool min_limit;
    bool max_limit;
} LimitEvent_t;

// I2C expander interrupt handler
static void IRAM_ATTR limit_switch_isr(void* arg) {
    // Schedule immediate I2C read in safety task
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(safety_monitor_handle,
                       NOTIFY_LIMIT_SWITCH,
                       eSetBits,
                       &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Process limit switches in task context
void processLimitSwitches() {
    uint16_t switch_states = 0;
    
    // Read both I2C expanders
    uint16_t exp0_data, exp1_data;
    if (i2c_read_word(0x20, &exp0_data) == ESP_OK &&
        i2c_read_word(0x21, &exp1_data) == ESP_OK) {
        
        // Combine limit switch data
        switch_states = ((exp1_data & 0x0FFF) << 4) | (exp0_data & 0x000F);
        
        // Check each axis
        for (int axis = 0; axis < 6; axis++) {
            bool min_triggered = (switch_states & (1 << (axis * 2))) != 0;
            bool max_triggered = (switch_states & (1 << (axis * 2 + 1))) != 0;
            
            if (min_triggered || max_triggered) {
                handleLimitSwitch(axis, min_triggered, max_triggered);
            }
        }
    }
}

void handleLimitSwitch(uint8_t axis, bool min_limit, bool max_limit) {
    ESP_LOGW(TAG, "Axis %d hit %s limit", axis, 
             min_limit ? "MIN" : "MAX");
    
    // Stop motion in the triggered direction only
    if (motors[axis]->isMoving()) {
        float velocity = motors[axis]->getCurrentVelocity();
        
        if ((min_limit && velocity < 0) || (max_limit && velocity > 0)) {
            // Stop immediately
            motors[axis]->stopImmediate();
            
            // Set limit flag
            motor_state[axis].limit_triggered = true;
            motor_state[axis].at_min_limit = min_limit;
            motor_state[axis].at_max_limit = max_limit;
            
            // Notify user
            sendResponse("ERROR E003 Axis %d limit switch triggered", axis);
        }
    }
    
    // Still allow motion away from limit
}
```

### 3. Driver Fault Detection
```cpp
// Servo driver fault signals (position complete inverted = fault)
void checkDriverFaults() {
    for (int axis = 0; axis < 4; axis++) {  // Servos only
        if (motors[axis]->isEnabled()) {
            bool pos_complete = gpio_get_level(pos_complete_pins[axis]);
            
            // Check for fault condition
            if (!pos_complete && !motors[axis]->isMoving()) {
                // Driver reporting fault
                handleDriverFault(axis);
            }
        }
    }
}

void handleDriverFault(uint8_t axis) {
    ESP_LOGE(TAG, "Driver fault detected on axis %d", axis);
    
    // Immediate actions
    motors[axis]->disable();
    motor_state[axis].driver_fault = true;
    
    // Log fault details
    FaultRecord_t fault = {
        .timestamp = esp_timer_get_time(),
        .axis = axis,
        .type = FAULT_DRIVER,
        .position = motors[axis]->getCurrentPosition(),
        .target = motors[axis]->getTargetPosition()
    };
    logFault(&fault);
    
    sendResponse("ERROR E004 Driver fault axis %d", axis);
}
```

## Software Safety Checks

### 1. Position Limit Checking
```cpp
class PositionLimitChecker {
private:
    float soft_min[6];
    float soft_max[6];
    
public:
    bool checkMove(uint8_t axis, float target_position) {
        // Check software limits
        if (target_position < soft_min[axis]) {
            ESP_LOGW(TAG, "Axis %d target %.2f below soft min %.2f",
                     axis, target_position, soft_min[axis]);
            return false;
        }
        
        if (target_position > soft_max[axis]) {
            ESP_LOGW(TAG, "Axis %d target %.2f above soft max %.2f",
                     axis, target_position, soft_max[axis]);
            return false;
        }
        
        // Check if we're at a limit switch
        if (motor_state[axis].at_min_limit && target_position < 
            motors[axis]->getCurrentPosition()) {
            ESP_LOGW(TAG, "Axis %d at MIN limit - cannot move negative");
            return false;
        }
        
        if (motor_state[axis].at_max_limit && target_position > 
            motors[axis]->getCurrentPosition()) {
            ESP_LOGW(TAG, "Axis %d at MAX limit - cannot move positive");
            return false;
        }
        
        return true;
    }
    
    void setSoftLimits(uint8_t axis, float min, float max) {
        if (min >= max) {
            ESP_LOGE(TAG, "Invalid limits: min %.2f >= max %.2f", min, max);
            return;
        }
        
        soft_min[axis] = min;
        soft_max[axis] = max;
        ESP_LOGI(TAG, "Axis %d soft limits: [%.2f, %.2f]", 
                 axis, min, max);
    }
};
```

### 2. Velocity Limit Enforcement
```cpp
class VelocityLimiter {
private:
    float max_velocity[6];
    float emergency_decel[6];  // For safety stops
    
public:
    float limitVelocity(uint8_t axis, float requested_velocity) {
        float limited = requested_velocity;
        
        // Clamp to maximum
        if (fabs(limited) > max_velocity[axis]) {
            limited = copysignf(max_velocity[axis], requested_velocity);
            ESP_LOGW(TAG, "Axis %d velocity limited: %.2f -> %.2f",
                     axis, requested_velocity, limited);
        }
        
        return limited;
    }
    
    float getEmergencyDeceleration(uint8_t axis) {
        // 2x normal acceleration for emergency
        return emergency_decel[axis];
    }
};
```

### 3. Following Error Detection
```cpp
// For servo motors - detect if position is deviating
class FollowingErrorMonitor {
private:
    typedef struct {
        float max_error;        // Maximum allowed error
        float current_error;    // Current following error
        uint32_t error_count;   // Consecutive error samples
        bool fault_active;      // Following error fault
    } FollowingError_t;
    
    FollowingError_t errors[4];  // Servos only
    
public:
    void update(uint8_t axis) {
        if (axis >= 4) return;  // Servos only
        
        if (!motors[axis]->isMoving()) {
            errors[axis].error_count = 0;
            return;
        }
        
        // Calculate following error
        float expected_pos = motors[axis]->getExpectedPosition();
        float actual_pos = motors[axis]->getCurrentPosition();
        float error = fabs(expected_pos - actual_pos);
        
        errors[axis].current_error = error;
        
        if (error > errors[axis].max_error) {
            errors[axis].error_count++;
            
            if (errors[axis].error_count > 10) {  // 10 consecutive
                // Trigger following error fault
                triggerFollowingErrorFault(axis);
            }
        } else {
            errors[axis].error_count = 0;
        }
    }
    
private:
    void triggerFollowingErrorFault(uint8_t axis) {
        ESP_LOGE(TAG, "Following error fault axis %d: %.2f mm",
                 axis, errors[axis].current_error);
        
        // Stop motion
        motors[axis]->stopImmediate();
        errors[axis].fault_active = true;
        
        // Log fault
        FaultRecord_t fault = {
            .timestamp = esp_timer_get_time(),
            .axis = axis,
            .type = FAULT_FOLLOWING_ERROR,
            .position = motors[axis]->getCurrentPosition(),
            .target = motors[axis]->getTargetPosition(),
            .error = errors[axis].current_error
        };
        logFault(&fault);
        
        sendResponse("ERROR E005 Following error axis %d", axis);
    }
};
```

### 4. Motion Supervision
```cpp
// Detect stuck motors, runaway conditions
class MotionSupervisor {
private:
    typedef struct {
        uint32_t motion_start_time;
        float start_position;
        float target_position;
        uint32_t no_motion_count;
        float last_position;
    } MotionSupervision_t;
    
    MotionSupervision_t supervision[6];
    
public:
    void startSupervision(uint8_t axis, float target) {
        supervision[axis].motion_start_time = esp_timer_get_time() / 1000;
        supervision[axis].start_position = motors[axis]->getCurrentPosition();
        supervision[axis].target_position = target;
        supervision[axis].no_motion_count = 0;
        supervision[axis].last_position = supervision[axis].start_position;
    }
    
    void checkMotion(uint8_t axis) {
        if (!motors[axis]->isMoving()) return;
        
        float current_pos = motors[axis]->getCurrentPosition();
        float position_change = fabs(current_pos - supervision[axis].last_position);
        
        // Check for stuck motor
        if (position_change < 0.001) {  // Less than 1 micron movement
            supervision[axis].no_motion_count++;
            
            if (supervision[axis].no_motion_count > 50) {  // 500ms stuck
                ESP_LOGE(TAG, "Axis %d appears stuck", axis);
                handleStuckMotor(axis);
            }
        } else {
            supervision[axis].no_motion_count = 0;
        }
        
        // Check for runaway (moving opposite direction)
        float expected_direction = copysignf(1.0f, 
            supervision[axis].target_position - supervision[axis].start_position);
        float actual_direction = copysignf(1.0f, 
            current_pos - supervision[axis].last_position);
        
        if (expected_direction * actual_direction < 0 && position_change > 0.1) {
            ESP_LOGE(TAG, "Axis %d runaway detected!", axis);
            handleRunawayMotor(axis);
        }
        
        supervision[axis].last_position = current_pos;
        
        // Check for timeout
        uint32_t elapsed = (esp_timer_get_time() / 1000) - 
                          supervision[axis].motion_start_time;
        float distance = fabs(supervision[axis].target_position - 
                            supervision[axis].start_position);
        float max_time = (distance / motors[axis]->getMaxVelocity()) * 2000 + 5000;
        
        if (elapsed > max_time) {
            ESP_LOGE(TAG, "Axis %d motion timeout", axis);
            handleMotionTimeout(axis);
        }
    }
    
private:
    void handleStuckMotor(uint8_t axis) {
        motors[axis]->stopImmediate();
        motor_state[axis].stuck_fault = true;
        sendResponse("ERROR E007 Motor %d stuck", axis);
    }
    
    void handleRunawayMotor(uint8_t axis) {
        // CRITICAL - stop all motion
        for (int i = 0; i < 6; i++) {
            motors[i]->stopImmediate();
        }
        system_state = STATE_CRITICAL_FAULT;
        sendResponse("ERROR E008 Motor %d runaway - SYSTEM HALT", axis);
    }
    
    void handleMotionTimeout(uint8_t axis) {
        motors[axis]->stopImmediate();
        sendResponse("ERROR E009 Motor %d timeout", axis);
    }
};
```

## Communication Safety

### 1. USB Timeout Detection
```cpp
class CommunicationMonitor {
private:
    uint32_t last_command_time;
    uint32_t command_timeout_ms = 30000;  // 30 seconds default
    bool timeout_enabled = true;
    
public:
    void onCommandReceived() {
        last_command_time = esp_timer_get_time() / 1000;
    }
    
    void checkTimeout() {
        if (!timeout_enabled) return;
        
        uint32_t now = esp_timer_get_time() / 1000;
        if ((now - last_command_time) > command_timeout_ms) {
            handleCommunicationTimeout();
        }
    }
    
private:
    void handleCommunicationTimeout() {
        ESP_LOGW(TAG, "Communication timeout - stopping all motion");
        
        // Stop all axes
        for (int i = 0; i < 6; i++) {
            if (motors[i]->isMoving()) {
                motors[i]->stopControlled();
            }
        }
        
        // Don't disable motors - just stop motion
        sendResponse("WARNING Communication timeout");
    }
};
```

### 2. Command Validation
```cpp
class CommandValidator {
public:
    bool validateCommand(const Command_t* cmd) {
        // Check command type
        if (cmd->type >= CMD_MAX) {
            ESP_LOGW(TAG, "Invalid command type: %d", cmd->type);
            return false;
        }
        
        // Check axis number
        if (cmd->axis >= 6 && cmd->axis != 0xFF) {
            ESP_LOGW(TAG, "Invalid axis: %d", cmd->axis);
            return false;
        }
        
        // Validate parameters based on command
        switch (cmd->type) {
            case CMD_MOVE:
            case CMD_MOVR:
                return validateMoveCommand(cmd);
                
            case CMD_VEL:
                return validateVelocityCommand(cmd);
                
            case CMD_HOME:
                return validateHomeCommand(cmd);
                
            default:
                return true;  // Other commands have looser validation
        }
    }
    
private:
    bool validateMoveCommand(const Command_t* cmd) {
        if (cmd->param_count < 1) return false;
        
        float position = cmd->params[0];
        
        // Check for NaN or infinity
        if (!isfinite(position)) {
            ESP_LOGW(TAG, "Invalid position: %f", position);
            return false;
        }
        
        // Range check (±10 meters is reasonable)
        if (fabs(position) > 10000.0f) {
            ESP_LOGW(TAG, "Position out of range: %f", position);
            return false;
        }
        
        return true;
    }
    
    bool validateVelocityCommand(const Command_t* cmd) {
        if (cmd->param_count < 1) return false;
        
        float velocity = cmd->params[0];
        
        if (!isfinite(velocity)) return false;
        
        // Reasonable velocity limit
        if (fabs(velocity) > 1000.0f) {  // 1 m/s
            ESP_LOGW(TAG, "Velocity out of range: %f", velocity);
            return false;
        }
        
        return true;
    }
};
```

## Fault Recovery

### 1. Fault State Machine
```cpp
typedef enum {
    FAULT_NONE = 0,
    FAULT_LIMIT_SWITCH,
    FAULT_DRIVER,
    FAULT_FOLLOWING_ERROR,
    FAULT_COMMUNICATION,
    FAULT_STUCK_MOTOR,
    FAULT_RUNAWAY,
    FAULT_EMERGENCY_STOP
} FaultType_t;

typedef struct {
    FaultType_t type;
    uint8_t axis;
    uint32_t timestamp;
    float position;
    float target;
    float error;
    char description[64];
} FaultRecord_t;

class FaultManager {
private:
    FaultRecord_t active_faults[16];
    uint8_t fault_count = 0;
    
public:
    void addFault(const FaultRecord_t* fault) {
        if (fault_count < 16) {
            active_faults[fault_count++] = *fault;
            xEventGroupSetBits(system_events, SYS_EVENT_FAULT);
        }
    }
    
    bool clearFault(uint8_t axis, FaultType_t type) {
        bool cleared = false;
        
        // Remove matching faults
        for (int i = 0; i < fault_count; i++) {
            if (active_faults[i].axis == axis && 
                active_faults[i].type == type) {
                // Shift remaining faults
                for (int j = i; j < fault_count - 1; j++) {
                    active_faults[j] = active_faults[j + 1];
                }
                fault_count--;
                cleared = true;
                i--;  // Check same position again
            }
        }
        
        // Clear system fault bit if no faults remain
        if (fault_count == 0) {
            xEventGroupClearBits(system_events, SYS_EVENT_FAULT);
        }
        
        return cleared;
    }
    
    void clearAllFaults() {
        fault_count = 0;
        xEventGroupClearBits(system_events, SYS_EVENT_FAULT);
        
        // Clear motor-specific fault flags
        for (int i = 0; i < 6; i++) {
            motor_state[i].limit_triggered = false;
            motor_state[i].driver_fault = false;
            motor_state[i].stuck_fault = false;
        }
    }
    
    void getFaultSummary(char* buffer, size_t size) {
        if (fault_count == 0) {
            snprintf(buffer, size, "No active faults");
            return;
        }
        
        int offset = snprintf(buffer, size, "Faults(%d):", fault_count);
        
        for (int i = 0; i < fault_count && offset < size - 1; i++) {
            offset += snprintf(buffer + offset, size - offset,
                             " [Axis%d:%s]",
                             active_faults[i].axis,
                             getFaultName(active_faults[i].type));
        }
    }
};
```

### 2. Recovery Procedures
```cpp
// Command: CLR <axis>
Response cmdClearFaults(uint8_t axis) {
    if (axis == 0xFF) {
        // Clear all faults
        faultManager.clearAllFaults();
        return Response::ok("All faults cleared");
    }
    
    if (axis >= 6) {
        return Response::error("Invalid axis");
    }
    
    // Clear specific axis faults
    bool cleared = false;
    
    if (motor_state[axis].limit_triggered) {
        // Check if we're still on limit
        uint16_t switches = readLimitSwitches();
        bool min = (switches & (1 << (axis * 2))) != 0;
        bool max = (switches & (1 << (axis * 2 + 1))) != 0;
        
        if (!min && !max) {
            motor_state[axis].limit_triggered = false;
            motor_state[axis].at_min_limit = false;
            motor_state[axis].at_max_limit = false;
            cleared = true;
        } else {
            return Response::error("Still on limit switch");
        }
    }
    
    if (motor_state[axis].driver_fault) {
        // Try to clear driver fault
        motors[axis]->enable();
        vTaskDelay(pdMS_TO_TICKS(100));
        
        if (gpio_get_level(pos_complete_pins[axis])) {
            motor_state[axis].driver_fault = false;
            cleared = true;
        } else {
            motors[axis]->disable();
            return Response::error("Driver fault persists");
        }
    }
    
    return cleared ? Response::ok("Faults cleared") : 
                    Response::error("No faults to clear");
}

// Command: RST
Response cmdReset() {
    // Only allow reset if E-stop is not active
    if (gpio_get_level(GPIO_E_STOP) == E_STOP_ACTIVE_LEVEL) {
        return Response::error("Cannot reset - E-stop active");
    }
    
    ESP_LOGI(TAG, "System reset requested");
    
    // Clear all faults
    faultManager.clearAllFaults();
    
    // Reset system state
    system_state = STATE_IDLE;
    xEventGroupClearBits(system_events, SYS_EVENT_EMERGENCY_STOP);
    xEventGroupSetBits(system_events, SYS_EVENT_READY);
    
    // Re-enable motors
    for (int i = 0; i < 6; i++) {
        motors[i]->reset();
    }
    
    return Response::ok("System reset complete");
}
```

## Safety Task Implementation

```cpp
void safetyMonitorTask(void* pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz monitoring
    
    // Instantiate safety monitors
    PositionLimitChecker positionLimits;
    VelocityLimiter velocityLimiter;
    FollowingErrorMonitor followingError;
    MotionSupervisor motionSupervisor;
    CommunicationMonitor commMonitor;
    
    while (1) {
        uint32_t notifications = 0;
        
        // Check for immediate notifications
        if (xTaskNotifyWait(0, ULONG_MAX, &notifications, 0) == pdTRUE) {
            if (notifications & NOTIFY_EMERGENCY_STOP) {
                handleEmergencyStop();
            }
            
            if (notifications & NOTIFY_LIMIT_SWITCH) {
                processLimitSwitches();
            }
        }
        
        // Periodic safety checks
        monitorEmergencyStop();
        checkDriverFaults();
        
        // Check each active axis
        for (int i = 0; i < 6; i++) {
            if (motors[i]->isEnabled()) {
                // Following error (servos only)
                if (i < 4) {
                    followingError.update(i);
                }
                
                // Motion supervision
                if (motors[i]->isMoving()) {
                    motionSupervisor.checkMotion(i);
                }
            }
        }
        
        // Communication timeout
        commMonitor.checkTimeout();
        
        // Wait for next period
        vTaskDelayUntil(&last_wake, period);
    }
}
```

This completes the comprehensive documentation of all the architecture topics we planned to address. The system now has:

1. ✅ Error handling and recovery
2. ✅ FreeRTOS task architecture
3. ✅ Inter-task communication
4. ✅ OLED display (simplified)
5. ✅ NVS organization
6. ✅ Power-on initialization sequence
7. ✅ Safety interlocks and fault detection

All topics have been thoroughly documented with practical implementation examples that follow the user's requirements for direct GPIO control, simple operation, and MVP focus with trapezoidal motion profiles.