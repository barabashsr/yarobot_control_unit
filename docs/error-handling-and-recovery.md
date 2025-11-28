# Error Handling and Recovery Strategies

## Error Categories

### 1. Critical Errors (System Stop Required)
These errors require immediate system halt and manual intervention.

```cpp
enum class CriticalError {
    EMERGENCY_STOP,          // E-stop button pressed
    I2C_COMMUNICATION_LOST,  // Cannot communicate with expanders
    POSITION_TRACKING_LOST,  // Z-signal mismatch on servo
    HARDWARE_WATCHDOG,       // Hardware watchdog triggered
    STACK_OVERFLOW,          // FreeRTOS stack overflow
    MEMORY_ALLOCATION_FAIL,  // Critical malloc failure
};
```

**Recovery Strategy**: 
- Stop all motor pulses immediately
- Disable all motors via direct GPIO (bypass I2C if needed)
- Set error LED pattern
- Log error to NVS
- Require RST command to recover

### 2. Motor-Specific Errors (Single Axis Stop)
These affect individual motors but don't require full system stop.

```cpp
enum class MotorError {
    LIMIT_SWITCH_HIT,        // End switch triggered during motion
    POSITION_LIMIT_EXCEEDED, // Software limit reached
    FOLLOWING_ERROR,         // Position error too large (servos)
    STALL_DETECTED,          // No motion detected (future)
    DRIVER_FAULT,            // Driver error signal (future)
    CALIBRATION_FAILED,      // Homing sequence failed
};
```

**Recovery Strategy**:
- Stop affected motor only
- Maintain position tracking if possible
- Report error via status
- Allow retry after clearing condition

### 3. Communication Errors (Degraded Operation)
Non-critical communication issues that allow continued operation.

```cpp
enum class CommError {
    USB_BUFFER_OVERFLOW,     // Command buffer full
    INVALID_COMMAND,         // Parsing error
    RESPONSE_TIMEOUT,        // Host not reading responses
    I2C_RETRY_SUCCESS,       // I2C recovered after retry
};
```

**Recovery Strategy**:
- Log error
- Continue operation
- Implement retry logic
- Report in diagnostics

## Error Detection Implementation

### 1. I2C Communication Monitor

```cpp
class I2CMonitor {
private:
    static constexpr uint32_t I2C_TIMEOUT_MS = 10;
    static constexpr uint32_t MAX_RETRIES = 3;
    static constexpr uint32_t HEALTH_CHECK_INTERVAL_MS = 100;
    
    struct ExpanderHealth {
        uint32_t last_success_time;
        uint32_t error_count;
        bool is_healthy;
    };
    
    ExpanderHealth expander_health_[2];
    TimerHandle_t health_check_timer_;
    
public:
    esp_err_t writeWithRetry(uint8_t addr, uint16_t data) {
        esp_err_t ret;
        
        for (int retry = 0; retry < MAX_RETRIES; retry++) {
            ret = i2c_master_write_to_device(
                I2C_NUM_0, addr, (uint8_t*)&data, 2, 
                pdMS_TO_TICKS(I2C_TIMEOUT_MS)
            );
            
            if (ret == ESP_OK) {
                // Update health status
                uint8_t idx = (addr == 0x20) ? 0 : 1;
                expander_health_[idx].last_success_time = xTaskGetTickCount();
                expander_health_[idx].error_count = 0;
                expander_health_[idx].is_healthy = true;
                return ESP_OK;
            }
            
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        // Failed after retries
        handleI2CFailure(addr);
        return ret;
    }
    
private:
    void handleI2CFailure(uint8_t addr) {
        uint8_t idx = (addr == 0x20) ? 0 : 1;
        expander_health_[idx].error_count++;
        
        if (expander_health_[idx].error_count > 10) {
            expander_health_[idx].is_healthy = false;
            
            // Critical error - stop all motors
            ErrorManager::getInstance().triggerCriticalError(
                CriticalError::I2C_COMMUNICATION_LOST
            );
        }
    }
    
    static void healthCheckTimerCallback(TimerHandle_t timer) {
        I2CMonitor* monitor = (I2CMonitor*)pvTimerGetTimerID(timer);
        monitor->performHealthCheck();
    }
    
    void performHealthCheck() {
        // Read interrupt flags from both expanders
        uint16_t int_flags;
        
        for (int i = 0; i < 2; i++) {
            uint8_t addr = (i == 0) ? 0x20 : 0x21;
            
            if (readWithRetry(addr, &int_flags) != ESP_OK) {
                ESP_LOGW(TAG, "Expander %d health check failed", i);
            }
        }
    }
};
```

### 2. Position Tracking Verification (Servos)

```cpp
class PositionTracker {
private:
    static constexpr int32_t MAX_POSITION_ERROR = 50;  // pulses
    
    struct AxisTracking {
        int32_t pulse_count;
        int32_t z_count;
        int32_t expected_pulses_per_rev;
        uint32_t last_z_time;
    };
    
    AxisTracking tracking_[4];  // Servo axes only
    
public:
    void IRAM_ATTR onZSignal(uint8_t axis) {
        auto& track = tracking_[axis];
        
        // Check if pulse count matches expected
        int32_t error = track.pulse_count - track.expected_pulses_per_rev;
        
        if (abs(error) > MAX_POSITION_ERROR) {
            // Position tracking error
            xQueueSendFromISR(error_queue_, &axis, NULL);
        }
        
        // Reset for next revolution
        track.pulse_count = 0;
        track.z_count++;
        track.last_z_time = esp_timer_get_time();
    }
    
    void checkPositionHealth(uint8_t axis) {
        auto& track = tracking_[axis];
        uint32_t now = esp_timer_get_time();
        
        // If moving but no Z-signal for too long
        if (is_moving_[axis] && 
            (now - track.last_z_time) > Z_SIGNAL_TIMEOUT_US) {
            
            ErrorManager::getInstance().setMotorError(
                axis, MotorError::FOLLOWING_ERROR
            );
        }
    }
};
```

### 3. Limit Switch Handling

```cpp
class LimitSwitchMonitor {
private:
    // Debounce and state tracking
    struct SwitchState {
        bool current_state;
        bool debounced_state;
        uint32_t last_change_time;
        uint32_t debounce_ms;
    };
    
    SwitchState switches_[12];  // 6 axes * 2 switches
    
    // Interrupt handler for expander
    static void IRAM_ATTR expanderInterruptISR(void* arg) {
        uint32_t event = LIMIT_SWITCH_EVENT;
        xQueueSendFromISR(event_queue_, &event, NULL);
    }
    
public:
    void processLimitSwitchEvent() {
        // Read all switch states from expander
        uint16_t switch_states;
        i2c_expander_read(0x21, &switch_states);
        
        // Check each axis
        for (int axis = 0; axis < 6; axis++) {
            bool min_switch = switch_states & (1 << (axis * 2));
            bool max_switch = switch_states & (1 << (axis * 2 + 1));
            
            // If moving towards limit that's active
            if (MotorController::getInstance().isMoving(axis)) {
                bool moving_positive = MotorController::getInstance()
                    .getDirection(axis);
                
                if ((moving_positive && max_switch) || 
                    (!moving_positive && min_switch)) {
                    
                    // Stop motor immediately
                    MotorController::getInstance().emergencyStop(axis);
                    
                    // Set error
                    ErrorManager::getInstance().setMotorError(
                        axis, MotorError::LIMIT_SWITCH_HIT
                    );
                }
            }
        }
    }
};
```

## Error Manager Central Coordinator

```cpp
class ErrorManager {
private:
    // Error states
    CriticalError active_critical_error_ = CriticalError::NONE;
    std::array<MotorError, 6> motor_errors_;
    std::queue<CommError> comm_error_log_;
    
    // Error callbacks
    std::vector<std::function<void(CriticalError)>> critical_callbacks_;
    std::vector<std::function<void(uint8_t, MotorError)>> motor_callbacks_;
    
    // Singleton
    ErrorManager() = default;
    
public:
    static ErrorManager& getInstance() {
        static ErrorManager instance;
        return instance;
    }
    
    void triggerCriticalError(CriticalError error) {
        active_critical_error_ = error;
        
        // Immediate actions
        switch (error) {
            case CriticalError::EMERGENCY_STOP:
                // Stop all RMT channels
                for (int i = 0; i < 4; i++) {
                    rmt_tx_stop(rmt_channels_[i]);
                }
                // Stop all MCPWM
                mcpwm_timer_stop(mcpwm_timers_[0]);
                mcpwm_timer_stop(mcpwm_timers_[1]);
                
                // Try to disable all motors
                for (int i = 0; i < 6; i++) {
                    gpio_set_level(enable_pins_[i], 0);
                }
                break;
                
            case CriticalError::I2C_COMMUNICATION_LOST:
                // Can't use I2C, so just stop motion
                stopAllMotion();
                break;
                
            default:
                stopAllMotion();
                break;
        }
        
        // Log to NVS
        logCriticalError(error);
        
        // Notify callbacks
        for (auto& cb : critical_callbacks_) {
            cb(error);
        }
        
        // Set error LED pattern
        setErrorLED(error);
    }
    
    void setMotorError(uint8_t axis, MotorError error) {
        motor_errors_[axis] = error;
        
        // Motor-specific actions
        switch (error) {
            case MotorError::LIMIT_SWITCH_HIT:
                // Already stopped by detector
                break;
                
            case MotorError::FOLLOWING_ERROR:
                // Stop motor and mark as needing recalibration
                MotorController::getInstance().disable(axis);
                break;
                
            default:
                break;
        }
        
        // Notify callbacks
        for (auto& cb : motor_callbacks_) {
            cb(axis, error);
        }
    }
    
    esp_err_t clearError(uint8_t axis) {
        if (active_critical_error_ != CriticalError::NONE) {
            return ESP_ERR_INVALID_STATE;  // Need system reset
        }
        
        motor_errors_[axis] = MotorError::NONE;
        return ESP_OK;
    }
    
    esp_err_t reset() {
        if (active_critical_error_ == CriticalError::EMERGENCY_STOP) {
            // Check if E-stop is released
            if (isEmergencyStopPressed()) {
                return ESP_ERR_INVALID_STATE;
            }
        }
        
        // Clear all errors
        active_critical_error_ = CriticalError::NONE;
        motor_errors_.fill(MotorError::NONE);
        
        // Reinitialize hardware
        return reinitializeHardware();
    }
    
    // Status reporting
    bool hasErrors() const {
        return active_critical_error_ != CriticalError::NONE;
    }
    
    void getErrorStatus(char* buffer, size_t len) {
        if (active_critical_error_ != CriticalError::NONE) {
            snprintf(buffer, len, "CRITICAL: %s", 
                getCriticalErrorString(active_critical_error_));
            return;
        }
        
        // Check motor errors
        for (int i = 0; i < 6; i++) {
            if (motor_errors_[i] != MotorError::NONE) {
                snprintf(buffer, len, "AXIS %d: %s", i,
                    getMotorErrorString(motor_errors_[i]));
                return;
            }
        }
        
        strcpy(buffer, "No errors");
    }
};
```

## Recovery Procedures

### 1. Automatic Recovery Attempts

```cpp
class AutoRecovery {
public:
    void attemptRecovery(uint8_t axis, MotorError error) {
        switch (error) {
            case MotorError::CALIBRATION_FAILED:
                // Retry homing with slower speed
                retryHoming(axis);
                break;
                
            case MotorError::FOLLOWING_ERROR:
                // Re-home to restore position
                if (rehomeAxis(axis) == ESP_OK) {
                    ErrorManager::getInstance().clearError(axis);
                }
                break;
                
            default:
                // No automatic recovery
                break;
        }
    }
    
private:
    esp_err_t retryHoming(uint8_t axis, uint8_t attempt = 1) {
        if (attempt > 3) return ESP_FAIL;
        
        // Reduce speed with each attempt
        float speed = BASE_HOMING_SPEED / attempt;
        
        return MotorController::getInstance()
            .homeAxis(axis, speed);
    }
};
```

### 2. Manual Recovery Commands

```
RST              - Clear non-critical errors
RST ERRORS       - Force clear all motor errors  
RST EMERGENCY    - Clear E-stop (if released)
HOME <axis>      - Re-establish position reference
TEST <axis>      - Verify axis functionality
```

## Error Reporting

### 1. LED Patterns

```cpp
enum class LEDPattern {
    SOLID_GREEN,      // Normal operation
    SLOW_BLINK_YELLOW, // Warning
    FAST_BLINK_RED,   // Error
    SOLID_RED,        // Critical error
    ALTERNATING,      // Specific axis error
};
```

### 2. OLED Display

```
[ERROR] E-STOP ACTIVE
Press RST after release

[WARN] Axis 2: LIMIT HIT
Clear: MOVR 2 -5
```

### 3. Command Response

```
ERROR E006 Emergency stop active
ERROR E008 Motor fault: Axis 2 limit switch
ERROR E009 Communication error: I2C timeout
```

## Testing Error Handling

```cpp
TEST_CASE("Emergency stop response time") {
    // Trigger E-stop during motion
    motor.moveToPosition(100);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    triggerEmergencyStop();
    
    // Verify motion stopped within 5ms
    uint32_t stop_time = measureStopTime();
    TEST_ASSERT_LESS_THAN(5000, stop_time);  // microseconds
}

TEST_CASE("I2C failure handling") {
    // Disconnect I2C during operation
    disconnectI2C();
    
    // Verify system enters safe state
    TEST_ASSERT_TRUE(ErrorManager::getInstance().hasErrors());
    TEST_ASSERT_EQUAL(0, getAllMotorEnableStates());
}
```