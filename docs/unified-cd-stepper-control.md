# C and D Stepper Motor Control

## Overview
Independent control system for stepper motors C and D with unified position/velocity interfaces:
- **C axis (Picker Jaw)**: MCPWM0 Timer 1 + PCNT1 - hardware position tracking with floating switch for object measurement
- **D axis (Picker Retractor)**: LEDC - software position calculation for discrete positions
- **Unified Interface**: Both motors respond to identical position/velocity commands for ROS2 compatibility

## Architecture

```
Independent Operation:
┌─────────────────────┐
│ MCPWM0 Timer 1 +    │──→ C (Picker Jaw)
│ PCNT1               │    - Precise position control
│ (Hardware position) │    - Floating switch measurement
└─────────────────────┘

┌─────────────────────┐
│ LEDC Channel 0      │──→ D (Picker Retractor)
│ GPIO14              │    - Discrete positions
│ (Software position) │    - Calculated states
└─────────────────────┘
```

## Implementation

### Base Stepper Interface
```cpp
class StepperAxis : public MotorBase {
public:
    // Standard motor interface for ROS2 compatibility
    virtual void moveAbsolute(float position) = 0;
    virtual void moveRelative(float distance) = 0;
    virtual void setVelocity(float velocity) = 0;
    virtual float getCurrentPosition() = 0;
    virtual float getTargetPosition() = 0;
    virtual float getCurrentVelocity() = 0;
    virtual bool isMoving() = 0;
    virtual void stop() = 0;
    
    // Calibration
    virtual void homeToMinLimit() = 0;
    virtual void homeToMaxLimit() = 0;
};
```

### C Axis - Picker Jaw with Measurement
```cpp
class PickerJawAxis : public StepperAxis {
private:
    // MCPWM hardware
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t mcpwm_oper;
    
    // PCNT hardware position counter
    pcnt_unit_handle_t pcnt_unit;
    pcnt_channel_handle_t pcnt_channel;
    
    // Measurement features
    float last_object_width = 0.0f;
    float grip_start_position = 0.0f;
    bool is_gripping = false;
    
    // Event callback
    std::function<void(float)> on_object_measured;
    
public:
    esp_err_t init() override {
        // Configure MCPWM Timer 1
        mcpwm_timer_config_t timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 10000000,  // 10MHz, 100ns resolution
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = 10000,  // Initial period
        };
        mcpwm_new_timer(&timer_config, &timer);
        
        // Configure PCNT for position tracking with internal routing
        pcnt_unit_config_t unit_config = {
            .high_limit = INT16_MAX,
            .low_limit = INT16_MIN,
            .flags.accum_count = true,  // Enable 32-bit counter
        };
        pcnt_new_unit(&unit_config, &pcnt_unit);
        
        // Configure PCNT channel with internal routing from MCPWM
        pcnt_chan_config_t chan_config = {
            .edge_gpio_num = -1,      // No physical GPIO - internal routing
            .level_gpio_num = -1,     // Not used
            .flags = {
                .io_loop_back = true  // Enable internal MCPWM→PCNT routing
            }
        };
        pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_channel);
        
        // Count on rising edge
        pcnt_channel_set_edge_action(pcnt_channel, 
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // Rising edge
            PCNT_CHANNEL_EDGE_ACTION_HOLD);      // Falling edge
            
        // Enable PCNT
        pcnt_unit_enable(pcnt_unit);
        pcnt_unit_start(pcnt_unit);
        
        return ESP_OK;
    }
    
    void moveAbsolute(float position) override {
        target_position = constrain(position, 0.0f, max_position);
        float distance = target_position - getCurrentPosition();
        
        if (fabsf(distance) < 0.01f) return;
        
        // Check if this is a gripping motion
        if (distance < 0 && !is_gripping) {
            // Starting to close jaw
            is_gripping = true;
            grip_start_position = getCurrentPosition();
        }
        
        // Set direction and enable
        bool forward = (distance > 0);
        shiftReg.setDirection(AXIS_C, forward);
        shiftReg.setEnable(AXIS_C, true);
        
        // Configure pulse frequency based on velocity
        uint32_t frequency = (uint32_t)(fabsf(current_velocity) * steps_per_mm);
        uint32_t period = 10000000 / frequency;  // 10MHz timer
        mcpwm_timer_set_period(timer, period);
        
        // Start motion
        mcpwm_timer_start(timer);
    }
    
    float getCurrentPosition() override {
        int32_t count;
        pcnt_unit_get_count(pcnt_unit, &count);
        return home_position + ((float)count / steps_per_mm);
    }
    
    void onLimitSwitch(uint8_t switch_id, bool state) override {
        if (switch_id == LIMIT_MAX && state && is_gripping) {
            // Floating switch triggered - object detected
            float current_pos = getCurrentPosition();
            last_object_width = grip_start_position - current_pos;
            
            // Stop motion (object gripped)
            stop();
            
            // Publish measurement event
            if (on_object_measured) {
                on_object_measured(last_object_width);
            }
            
            EventManager::publish({
                .type = EVENT_OBJECT_DETECTED,
                .axis = AXIS_C,
                .data.measurement = last_object_width,
                .timestamp = esp_timer_get_time()
            });
            
            is_gripping = false;
        } else if (switch_id == LIMIT_MIN && state) {
            // Hard limit reached
            stop();
            home_position = 0.0f;
            pcnt_unit_clear_count(pcnt_unit);
        }
    }
    
    float getLastObjectWidth() const {
        return last_object_width;
    }
    
    void setMeasurementCallback(std::function<void(float)> callback) {
        on_object_measured = callback;
    }
};
```

### D Axis - Picker Retractor
```cpp
class RetractorAxis : public StepperAxis {
private:
    // LEDC hardware
    ledc_timer_config_t timer_config;
    ledc_channel_config_t channel_config;
    
    // Position calculation
    float calculated_position = 0.0f;
    float velocity_setpoint = 0.0f;
    uint64_t motion_start_time = 0;
    uint64_t last_update_time = 0;
    bool is_moving = false;
    
    // Position update task
    TaskHandle_t position_task;
    
public:
    esp_err_t init() override {
        // Configure LEDC timer
        timer_config = {
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_1_BIT,  // 1-bit for 50% duty
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 1000,  // Initial frequency
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&timer_config);
        
        // Configure LEDC channel
        channel_config = {
            .gpio_num = GPIO_D_STEP,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,  // Initially off
            .hpoint = 0,
        };
        ledc_channel_config(&channel_config);
        
        // Create position calculation task
        xTaskCreate(positionUpdateTask, "d_pos_calc", 2048, 
                    this, 10, &position_task);
        
        return ESP_OK;
    }
    
    void moveAbsolute(float position) override {
        // D axis typically moves between two positions
        target_position = (position > 50.0f) ? max_position : 0.0f;
        
        float distance = target_position - calculated_position;
        if (fabsf(distance) < 0.1f) return;
        
        // Set direction and enable
        bool forward = (distance > 0);
        shiftReg.setDirection(AXIS_D, forward);
        shiftReg.setEnable(AXIS_D, true);
        velocity_setpoint = forward ? fabsf(default_velocity) : -fabsf(default_velocity);
        
        // Set pulse frequency
        uint32_t frequency = (uint32_t)(fabsf(velocity_setpoint) * steps_per_mm);
        ledc_set_freq(LEDC_HIGH_SPEED_MODE, timer_config.timer_num, frequency);
        
        // Start pulses (50% duty cycle)
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel_config.channel, 1);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel_config.channel);
        
        // Initialize motion tracking
        motion_start_time = esp_timer_get_time();
        last_update_time = motion_start_time;
        is_moving = true;
    }
    
    float getCurrentPosition() override {
        if (is_moving) {
            // Calculate position based on time and velocity
            uint64_t now = esp_timer_get_time();
            float dt = (now - last_update_time) / 1000000.0f;
            calculated_position += velocity_setpoint * dt;
            last_update_time = now;
        }
        return calculated_position;
    }
    
    void stop() override {
        // Stop pulse generation
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel_config.channel, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel_config.channel);
        
        // Disable motor
        shiftReg.setEnable(AXIS_D, false);
        
        is_moving = false;
        
        // Final position update
        getCurrentPosition();
    }
    
    void onLimitSwitch(uint8_t switch_id, bool state) override {
        if (state) {
            stop();
            
            // Recalibrate position based on limit switch
            if (switch_id == LIMIT_MIN) {
                calculated_position = 0.0f;
            } else if (switch_id == LIMIT_MAX) {
                calculated_position = max_position;
            }
        }
    }
    
    static void positionUpdateTask(void* param) {
        RetractorAxis* axis = (RetractorAxis*)param;
        
        while (1) {
            if (axis->is_moving) {
                float current_pos = axis->getCurrentPosition();
                
                // Check if target reached
                bool at_target = false;
                if (axis->velocity_setpoint > 0) {
                    at_target = (current_pos >= axis->target_position);
                } else {
                    at_target = (current_pos <= axis->target_position);
                }
                
                if (at_target) {
                    axis->stop();
                    axis->calculated_position = axis->target_position;
                }
            }
            
            vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz update
        }
    }
};
```

## Command Interface

Both C and D axes accept identical commands for uniform control:

```cpp
// Position Commands
MOVE C 30.0   // Move jaw to 30mm (grip position)
MOVE D 100.0  // Extend retractor fully
MOVE D 0.0    // Retract fully

// Velocity Commands  
VEL C 25.0    // Set jaw velocity to 25mm/s
VEL D 50.0    // Set retractor velocity to 50mm/s

// Status Commands
STAT C        // Returns position, velocity, and last object width
STAT D        // Returns calculated position and velocity

// Homing
HOME C        // Home jaw to minimum position
HOME D        // Home retractor to minimum position
```

## Event System

```cpp
// Object measurement event (C axis only)
EVENT: OBJECT_DETECTED
{
    "axis": "C",
    "width": 12.5,      // mm
    "timestamp": 1234567890
}

// Standard events (both axes)
EVENT: LIMIT_REACHED
{
    "axis": "D",
    "limit": "MAX",
    "timestamp": 1234567890
}

EVENT: POSITION_REACHED
{
    "axis": "C", 
    "position": 30.0,
    "timestamp": 1234567890
}
```

## ROS2 Interface Compatibility

Both axes provide standard hardware interfaces:

```cpp
// Position State Interface (published at 50-100Hz)
hardware_interface::StateInterface("picker_jaw/position", &c_position);
hardware_interface::StateInterface("picker_retractor/position", &d_position);

// Velocity State Interface  
hardware_interface::StateInterface("picker_jaw/velocity", &c_velocity);
hardware_interface::StateInterface("picker_retractor/velocity", &d_velocity);

// Command Interfaces
hardware_interface::CommandInterface("picker_jaw/position", &c_cmd_position);
hardware_interface::CommandInterface("picker_retractor/position", &d_cmd_position);

// Additional Sensor Interface (C axis)
hardware_interface::StateInterface("picker_jaw/object_width", &c_object_width);
```

## Calibration and Homing

### C Axis (Picker Jaw)
1. Move to MIN limit switch (fully open)
2. Clear PCNT counter
3. Set position to 0.0mm
4. Ready for operation

### D Axis (Picker Retractor)
1. Move to MIN limit switch (fully retracted)
2. Reset calculated position to 0.0mm
3. Ready for operation
4. Position recalibrated on every limit switch hit

## Position Accuracy

- **C Axis**: ±1 step accuracy (hardware PCNT counter with internal routing)
- **D Axis**: ±0.5mm typical (software calculation, recalibrated each cycle)

## Internal Routing Benefits

The C axis uses ESP32-S3's internal signal routing to connect MCPWM output directly to PCNT input:

1. **No External Wiring**: MCPWM→PCNT connection is internal, eliminating PCB traces
2. **Improved Reliability**: No external signal path means no EMI/noise issues  
3. **GPIO Savings**: Frees up GPIO pins for other uses
4. **Zero Latency**: Direct internal connection ensures accurate pulse counting

### Alternative Implementation (for older ESP-IDF)
If using ESP-IDF < 5.0 without `io_loop_back` support:

```cpp
// Manual internal routing using GPIO Matrix
void setupInternalRouting() {
    // Get signal indices
    uint8_t mcpwm_out_sig = MCPWM0_OUT1A_IDX;  // MCPWM Timer1 output A
    uint8_t pcnt_in_sig = PCNT_SIG_CH1_IN0_IDX; // PCNT Unit1 input
    
    // Route MCPWM output directly to PCNT input
    gpio_matrix_in(mcpwm_out_sig, pcnt_in_sig, false);
}
```

Both axes maintain sufficient accuracy for their respective functions while providing uniform interfaces for higher-level control systems.