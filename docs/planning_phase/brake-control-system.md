# Servo Brake Control System

## Overview

The brake control system manages electromagnetic spring-applied brakes for servo motors (axes X, Y, Z, A, B). These brakes are fail-safe devices that engage mechanically when power is removed, ensuring position holding during power loss or emergency conditions.

## Brake Characteristics

### Hardware Specifications
- **Type**: Spring-applied, power-to-release electromagnetic brakes
- **Voltage**: 24V DC (typical)
- **Release Time**: 50-100ms (mechanical disengagement)
- **Engagement Time**: 20-50ms (spring application)
- **Control**: Via second 74HC595 shift register (bits 0-4)

### Safety Design
- **Fail-Safe**: Brakes engage on power loss
- **Independent Control**: Each axis brake controlled separately
- **Status Monitoring**: Brake state readable via I/O expander

## Brake Control Strategies

### 1. BRAKE_ON_DISABLE (Default)
Most common strategy balancing safety and performance.

```cpp
// Behavior Timeline
Motor Enable (EN X 1):
  └─> Release brake (50ms delay)
      └─> Motor holds position electrically
          └─> Normal operation (no brake cycling)

Motor Disable (EN X 0):
  └─> Engage brake immediately
      └─> Wait for engagement (50ms)
          └─> Motor power can be removed safely
```

**Use Cases**:
- Standard pick-and-place operations
- Continuous motion applications
- When servo has sufficient holding torque

**Advantages**:
- No mechanical wear during operation
- Fast motion response
- Quiet operation

### 2. BRAKE_ON_ESTOP
Minimal brake usage for maximum performance.

```cpp
// Normal Operation States
Enabled + Moving:     Brake OFF, Motor powered
Enabled + Stopped:    Brake OFF, Motor holds electrically
Disabled:             Brake OFF, Motor freewheels

// Emergency States
E-Stop Triggered:     Brake ON immediately
Fault Detected:       Brake ON immediately
Power Loss:          Brake ON (mechanical spring)
```

**Use Cases**:
- High-speed continuous operations
- Low-load horizontal axes
- When position loss on disable is acceptable

**Configuration Example**:
```cpp
// Z-axis might need different strategy due to gravity
axis_config[AXIS_X].brake_mode = BRAKE_ON_ESTOP;  // Horizontal
axis_config[AXIS_Z].brake_mode = BRAKE_ON_DISABLE; // Vertical
```

### 3. BRAKE_ON_IDLE
Engages brake after configurable idle period.

```cpp
// State Machine
Moving:
  └─> Brake OFF, Reset idle timer

Stopped:
  └─> Start idle timer
      └─> If (idle_time > threshold):
          └─> Engage brake
              └─> Motor remains enabled

Next Move Command:
  └─> Release brake (50ms)
      └─> Execute motion
```

**Timing Configuration**:
```cpp
struct IdleBrakeConfig {
    uint32_t idle_threshold_ms;    // Default: 30000 (30 seconds)
    uint32_t release_delay_ms;     // Default: 100 (brake release time)
    bool keep_motor_enabled;       // Default: true
};
```

**Use Cases**:
- Vertical axes holding against gravity
- Long periods between moves
- Power consumption optimization

**Implementation**:
```cpp
void updateIdleTimer(uint8_t axis) {
    if (axis_state[axis].is_moving) {
        idle_timers[axis] = 0;
        if (brake_engaged[axis]) {
            releaseBrake(axis);
        }
    } else {
        idle_timers[axis] += TASK_PERIOD_MS;
        if (idle_timers[axis] > config.idle_threshold_ms) {
            if (!brake_engaged[axis]) {
                engageBrake(axis);
            }
        }
    }
}
```

### 4. BRAKE_NEVER
Manual control only - brake state controlled explicitly by commands.

```cpp
// Only these commands affect brake:
BRAKE X HOLD      // Engage brake
BRAKE X RELEASE   // Release brake
BRAKE X STATUS    // Query brake state

// These commands DO NOT affect brake:
EN X 0/1          // Motor enable/disable
STOP X            // Stop motion
E-STOP            // Emergency stop (motor stops but brake unchanged)
```

**Use Cases**:
- Testing and commissioning
- Special maintenance procedures
- Custom brake control sequences

**Safety Consideration**: E-stop may not engage brakes in this mode!

## Command Interface

### Brake Control Commands

```bash
# Set brake strategy
BRAKE X AUTO              # Use configured strategy (default)
BRAKE X NEVER             # Manual control only
BRAKE X ALWAYS            # Immediate brake on any stop
BRAKE X IDLE 30000        # Brake after 30 seconds idle

# Manual brake control
BRAKE X HOLD              # Force brake engagement
BRAKE X RELEASE           # Force brake release
BRAKE ALL HOLD            # Engage all brakes (maintenance)
BRAKE ALL RELEASE         # Release all brakes

# Status query
BRAKE X STATUS            # Get brake state and strategy
BRAKE ALL STATUS          # Get all brake states
```

### Response Format
```
OK BRAKE:X STATE:RELEASED STRATEGY:ON_DISABLE IDLE:0
OK BRAKE:ALL X:REL Y:REL Z:HOLD A:REL B:REL
```

## Implementation Details

### Hardware Interface
```cpp
class BrakeController {
private:
    ShiftRegister* brake_shift_reg;  // Second 74HC595
    uint8_t brake_states;            // Current brake states
    SemaphoreHandle_t spi_mutex;     // Shared SPI bus protection
    
    // Brake control bits in shift register
    enum BrakeBits {
        BRAKE_X = 0,  // Bit 0
        BRAKE_Y = 1,  // Bit 1
        BRAKE_Z = 2,  // Bit 2
        BRAKE_A = 3,  // Bit 3
        BRAKE_B = 4,  // Bit 4
    };
    
    // Combined shift register update (DIR/EN + Brakes)
    void updateShiftRegisters(uint16_t dir_en_bits) {
        xSemaphoreTake(spi_mutex, portMAX_DELAY);
        
        // Prepare 24-bit data: [8-bit brake][16-bit dir/en]
        uint32_t combined_data = (brake_states << 16) | dir_en_bits;
        
        // Single atomic SPI transaction
        gpio_set_level(SR_LATCH, 0);
        spi_transaction_t trans = {
            .length = 24,
            .tx_buffer = &combined_data,
        };
        spi_device_transmit(spi_handle, &trans);
        gpio_set_level(SR_LATCH, 1);  // Latch outputs
        
        xSemaphoreGive(spi_mutex);
    }
    
public:
    void engageBrake(uint8_t axis) {
        if (axis > AXIS_B) return;  // C,D steppers have no brakes
        
        // Clear bit to engage (active low for fail-safe)
        brake_states &= ~(1 << axis);
        updateShiftRegisters(current_dir_en_states);
        
        // Log event
        ESP_LOGI(TAG, "Brake engaged: Axis %c", 'X' + axis);
        
        // Publish event
        EventManager::publish({
            .type = EVENT_BRAKE_ENGAGED,
            .axis = axis,
            .timestamp = esp_timer_get_time()
        });
    }
    
    void releaseBrake(uint8_t axis) {
        if (axis > AXIS_B) return;
        
        // Set bit to release (power applied)
        brake_states |= (1 << axis);
        updateShiftRegisters(current_dir_en_states);
        
        // Wait for mechanical release
        vTaskDelay(pdMS_TO_TICKS(config.release_delay_ms));
        
        ESP_LOGI(TAG, "Brake released: Axis %c", 'X' + axis);
    }
};
```

### State Management
```cpp
struct AxisBrakeState {
    BrakeStrategy strategy;
    bool brake_engaged;
    bool motor_enabled;
    uint32_t idle_timer_ms;
    uint32_t idle_threshold_ms;
};

void updateBrakeLogic(uint8_t axis) {
    auto& state = brake_state[axis];
    
    switch (state.strategy) {
        case BRAKE_ON_DISABLE:
            if (!state.motor_enabled && !state.brake_engaged) {
                engageBrake(axis);
            } else if (state.motor_enabled && state.brake_engaged) {
                releaseBrake(axis);
            }
            break;
            
        case BRAKE_ON_ESTOP:
            if (system_state.emergency_stop && !state.brake_engaged) {
                engageBrake(axis);
            } else if (!system_state.emergency_stop && state.brake_engaged) {
                releaseBrake(axis);
            }
            break;
            
        case BRAKE_ON_IDLE:
            updateIdleTimer(axis);
            break;
            
        case BRAKE_NEVER:
            // Manual control only
            break;
    }
}
```

### Motion Sequencing with Brakes

```cpp
esp_err_t startMotion(uint8_t axis, float target) {
    // Check if brake needs release
    if (brake_state[axis].brake_engaged) {
        if (brake_state[axis].strategy == BRAKE_NEVER) {
            // Manual mode - don't auto-release
            return ESP_ERR_INVALID_STATE;
        }
        
        // Release brake and wait
        releaseBrake(axis);
    }
    
    // Clear idle timer
    brake_state[axis].idle_timer_ms = 0;
    
    // Start motion
    return motor[axis]->moveAbsolute(target);
}
```

### Emergency Stop Handling

```cpp
void handleEmergencyStop() {
    // Stop all motion immediately
    for (int axis = 0; axis < NUM_AXES; axis++) {
        motor[axis]->emergencyStop();
    }
    
    // Apply brakes based on strategy
    for (int axis = 0; axis < NUM_SERVO_AXES; axis++) {
        switch (brake_state[axis].strategy) {
            case BRAKE_ON_ESTOP:
            case BRAKE_ON_DISABLE:
            case BRAKE_ON_IDLE:
                engageBrake(axis);
                break;
                
            case BRAKE_NEVER:
                // Manual mode - don't auto-engage
                ESP_LOGW(TAG, "E-stop: Axis %c brake not engaged (MANUAL mode)", 
                        'X' + axis);
                break;
        }
    }
}
```

### Power Loss Protection

```cpp
// Called from power monitor interrupt
void handlePowerLoss() {
    // Shift register outputs go to 0 on power loss
    // This ensures brakes engage (active low)
    // Spring force provides mechanical backup
    
    // Save critical position data to RTC memory
    for (int axis = 0; axis < NUM_AXES; axis++) {
        rtc_position_data[axis] = motor[axis]->getCurrentPosition();
    }
    
    // Set flag for power loss recovery
    rtc_power_loss_flag = true;
}

// Called on startup
void checkPowerRecovery() {
    if (rtc_power_loss_flag) {
        ESP_LOGW(TAG, "Power loss detected - brakes were engaged");
        
        // All brakes are engaged mechanically
        for (int axis = 0; axis < NUM_SERVO_AXES; axis++) {
            brake_state[axis].brake_engaged = true;
        }
        
        // Report saved positions
        for (int axis = 0; axis < NUM_AXES; axis++) {
            ESP_LOGI(TAG, "Axis %c last position: %.2f", 
                    'X' + axis, rtc_position_data[axis]);
        }
        
        rtc_power_loss_flag = false;
    }
}
```

## Configuration Storage

Brake configurations are stored in NVS for each axis:

```cpp
// NVS Keys
"brake_0_mode"      // BrakeStrategy enum
"brake_0_idle"      // Idle timeout in ms
"brake_1_mode"      // For Y axis
...

// Save configuration
void saveBrakeConfig(uint8_t axis) {
    char key[16];
    snprintf(key, sizeof(key), "brake_%d_mode", axis);
    nvs_set_u8(nvs_handle, key, (uint8_t)brake_state[axis].strategy);
    
    snprintf(key, sizeof(key), "brake_%d_idle", axis);
    nvs_set_u32(nvs_handle, key, brake_state[axis].idle_threshold_ms);
}
```

## Safety Considerations

### Critical Safety Rules

1. **Power Loss**: Brakes MUST engage on power loss (shift register outputs go low)
2. **E-Stop**: Should engage brakes unless in BRAKE_NEVER mode
3. **Initialization**: Start with all brakes engaged until explicitly released
4. **SPI Integrity**: Verify shift register updates for safety-critical changes
5. **Interlocks**: Don't allow motion if brake release fails
6. **Fail-Safe Wiring**: Active LOW for brake control (0 = engaged, 1 = released)

### Fault Detection

```cpp
bool verifyBrakeRelease(uint8_t axis) {
    releaseBrake(axis);
    
    // Check current feedback if available
    float brake_current = measureBrakeCurrent(axis);
    if (brake_current < MIN_BRAKE_CURRENT) {
        // Brake may be stuck or power failure
        ESP_LOGE(TAG, "Brake release failed: Axis %c", 'X' + axis);
        engageBrake(axis);  // Re-engage for safety
        return false;
    }
    
    return true;
}
```

## Testing Requirements

### Functional Tests
1. Each brake engages/releases independently
2. Release time measurement (should be <100ms)
3. Strategy switching during operation
4. Idle timeout accuracy
5. E-stop brake engagement

### Safety Tests
1. Power loss simulation - verify mechanical engagement
2. Brake failure detection (if current monitoring available)
3. Motion interlock when brake engaged
4. Recovery from brake faults

### Performance Tests
1. Brake cycling impact on motion timing
2. Power consumption in different modes
3. Mechanical wear testing (continuous cycling)
4. Acoustic noise levels

## Future Enhancements

1. **Current Monitoring**: Detect brake coil failures
2. **Temperature Compensation**: Adjust timing for temperature
3. **Predictive Maintenance**: Track brake engagement cycles
4. **Smart Idle**: Learn usage patterns for optimal idle timing
5. **Energy Recovery**: Use brake engagement to generate position feedback