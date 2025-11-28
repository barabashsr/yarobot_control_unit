# Trapezoidal Motion Profile Implementation

## Overview
Implementation of trapezoidal velocity profiles for smooth motor control. Supports both RMT (hardware-accelerated) and MCPWM (timer-based) approaches.

## Profile Phases

```
Velocity
   ^
   |     ___________
   |    /|         |\
   |   / |         | \
   |  /  |         |  \
   | /   |         |   \
   |/____|_________|____\___> Time
     Accel  Const   Decel
```

1. **Acceleration Phase**: Linear velocity increase from 0 to max velocity
2. **Constant Velocity Phase**: Maintain max velocity (may be zero for short moves)
3. **Deceleration Phase**: Linear velocity decrease from max velocity to 0

## Mathematical Model

### Key Variables
- `d`: Total distance (pulses)
- `v_max`: Maximum velocity (pulses/sec)
- `a`: Acceleration/deceleration (pulses/sec²)

### Profile Calculations

```cpp
// Time to reach max velocity
t_accel = v_max / a;

// Distance covered during acceleration
d_accel = 0.5 * a * t_accel * t_accel;

// Check if we can reach max velocity
if (2 * d_accel > abs(d)) {
    // Triangle profile - never reach v_max
    t_accel = sqrt(abs(d) / a);
    d_accel = abs(d) / 2;
    v_peak = a * t_accel;  // Peak velocity < v_max
} else {
    // Full trapezoidal profile
    d_const = abs(d) - 2 * d_accel;
    t_const = d_const / v_max;
}
```

## RMT Implementation (Axes 0-3)

### Profile Generator

```cpp
class TrapezoidalProfileGenerator {
private:
    struct ProfileParams {
        float total_distance;
        float max_velocity;
        float acceleration;
        float accel_distance;
        float const_distance;
        float decel_distance;
        bool is_triangle;
    };
    
    ProfileParams calculateProfile(float distance, float velocity, float acceleration) {
        ProfileParams params;
        params.total_distance = fabs(distance);
        params.max_velocity = velocity;
        params.acceleration = acceleration;
        
        // Calculate acceleration phase
        float t_accel = velocity / acceleration;
        params.accel_distance = 0.5f * acceleration * t_accel * t_accel;
        
        // Check profile type
        if (2 * params.accel_distance > params.total_distance) {
            // Triangle profile
            params.is_triangle = true;
            params.accel_distance = params.total_distance / 2;
            params.const_distance = 0;
            params.decel_distance = params.total_distance / 2;
            
            // Recalculate actual peak velocity
            t_accel = sqrt(params.total_distance / acceleration);
            params.max_velocity = acceleration * t_accel;
        } else {
            // Trapezoidal profile
            params.is_triangle = false;
            params.decel_distance = params.accel_distance;
            params.const_distance = params.total_distance - 2 * params.accel_distance;
        }
        
        return params;
    }
    
public:
    size_t generateRMTSymbols(
        rmt_symbol_word_t* symbols,
        size_t max_symbols,
        float distance,
        float velocity,
        float acceleration
    ) {
        ProfileParams params = calculateProfile(distance, velocity, acceleration);
        size_t symbol_count = 0;
        
        // Generate acceleration phase
        symbol_count += generateAccelPhase(
            &symbols[symbol_count],
            max_symbols - symbol_count,
            0,                      // start velocity
            params.max_velocity,    // end velocity
            acceleration,
            params.accel_distance
        );
        
        // Generate constant phase (if any)
        if (params.const_distance > 0) {
            symbol_count += generateConstPhase(
                &symbols[symbol_count],
                max_symbols - symbol_count,
                params.max_velocity,
                params.const_distance
            );
        }
        
        // Generate deceleration phase
        symbol_count += generateAccelPhase(
            &symbols[symbol_count],
            max_symbols - symbol_count,
            params.max_velocity,    // start velocity
            0,                      // end velocity
            -acceleration,          // negative acceleration
            params.decel_distance
        );
        
        return symbol_count;
    }
    
private:
    size_t generateAccelPhase(
        rmt_symbol_word_t* symbols,
        size_t max_symbols,
        float start_vel,
        float end_vel,
        float accel,
        float distance
    ) {
        size_t count = 0;
        float current_vel = start_vel;
        float position = 0;
        
        // RMT clock is typically 80MHz
        const uint32_t RMT_CLK_FREQ = 80000000;
        const uint16_t MIN_DURATION = 50;  // Minimum pulse width in RMT ticks
        
        while (position < distance && count < max_symbols) {
            // Time between pulses
            float period = (current_vel > 0) ? (1.0f / current_vel) : 1.0f;
            
            // Convert to RMT ticks (nanoseconds to ticks)
            uint32_t duration_ticks = (uint32_t)(period * RMT_CLK_FREQ / 2);
            
            // Clamp to valid RMT duration (15-bit max)
            if (duration_ticks > 0x7FFF) duration_ticks = 0x7FFF;
            if (duration_ticks < MIN_DURATION) duration_ticks = MIN_DURATION;
            
            // Create RMT symbol (50% duty cycle)
            symbols[count].duration0 = duration_ticks;  // HIGH duration
            symbols[count].level0 = 1;
            symbols[count].duration1 = duration_ticks;  // LOW duration
            symbols[count].level1 = 0;
            
            // Update for next pulse
            float dt = period;
            current_vel += accel * dt;
            if (accel > 0 && current_vel > end_vel) current_vel = end_vel;
            if (accel < 0 && current_vel < end_vel) current_vel = end_vel;
            
            position += 1.0f;  // One pulse
            count++;
        }
        
        return count;
    }
    
    size_t generateConstPhase(
        rmt_symbol_word_t* symbols,
        size_t max_symbols,
        float velocity,
        float distance
    ) {
        size_t count = 0;
        float period = 1.0f / velocity;
        const uint32_t RMT_CLK_FREQ = 80000000;
        
        uint32_t duration_ticks = (uint32_t)(period * RMT_CLK_FREQ / 2);
        if (duration_ticks > 0x7FFF) duration_ticks = 0x7FFF;
        
        size_t pulse_count = (size_t)distance;
        
        // Generate constant velocity pulses
        while (count < pulse_count && count < max_symbols) {
            symbols[count].duration0 = duration_ticks;
            symbols[count].level0 = 1;
            symbols[count].duration1 = duration_ticks;
            symbols[count].level1 = 0;
            count++;
        }
        
        return count;
    }
};
```

### RMT Motor Integration

```cpp
esp_err_t RMTMotor::moveToPosition(float position, float velocity, float acceleration) {
    // Calculate move parameters
    float distance = position - current_position_;
    if (fabs(distance) < 0.001f) return ESP_OK;  // Already at position
    
    // Set direction
    gpio_set_level(dir_pin_, distance > 0 ? 1 : 0);
    
    // Use configured defaults if not specified
    if (velocity <= 0) velocity = max_velocity_;
    if (acceleration <= 0) acceleration = max_acceleration_;
    
    // Convert to pulses
    float distance_pulses = distance * pulses_per_unit_;
    float velocity_pulses = velocity * pulses_per_unit_;
    float accel_pulses = acceleration * pulses_per_unit_;
    
    // Generate profile
    TrapezoidalProfileGenerator generator;
    size_t symbol_count = generator.generateRMTSymbols(
        motion_profile_buffer_,
        MAX_RMT_SYMBOLS,
        distance_pulses,
        velocity_pulses,
        accel_pulses
    );
    
    if (symbol_count == 0) {
        ESP_LOGE(TAG, "Failed to generate motion profile");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Mark end of transmission
    motion_profile_buffer_[symbol_count].duration0 = 0;
    motion_profile_buffer_[symbol_count].duration1 = 0;
    
    // Update state
    target_position_ = position;
    is_moving_ = true;
    
    // Start RMT transmission
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,  // No loop
        .flags = {
            .eot_level = 0,
        }
    };
    
    ESP_ERROR_CHECK(rmt_transmit(
        rmt_channel_,
        pulse_encoder_,
        motion_profile_buffer_,
        symbol_count * sizeof(rmt_symbol_word_t),
        &tx_config
    ));
    
    return ESP_OK;
}
```

## MCPWM Implementation (Axes 4-5)

### Timer-Based Profile Controller

```cpp
class MCPWMProfileController {
private:
    // Update rate for velocity changes (Hz)
    static constexpr uint32_t PROFILE_UPDATE_RATE = 1000;
    static constexpr float UPDATE_PERIOD = 1.0f / PROFILE_UPDATE_RATE;
    
    struct MotionState {
        // Profile parameters
        float target_position;
        float max_velocity;
        float acceleration;
        
        // Runtime state
        float current_position;
        float current_velocity;
        int32_t pulses_remaining;
        int32_t pulses_to_decel;
        
        enum Phase {
            IDLE,
            ACCELERATING,
            CONSTANT,
            DECELERATING
        } phase;
        
        // Direction
        bool forward;
    } state_;
    
public:
    void startMotion(float distance, float max_velocity, float acceleration) {
        state_.target_position = distance;
        state_.max_velocity = max_velocity;
        state_.acceleration = acceleration;
        state_.current_position = 0;
        state_.current_velocity = 0;
        state_.pulses_remaining = abs((int32_t)distance);
        state_.forward = distance > 0;
        state_.phase = MotionState::ACCELERATING;
        
        // Pre-calculate deceleration point
        float decel_distance = (max_velocity * max_velocity) / (2 * acceleration);
        state_.pulses_to_decel = (int32_t)decel_distance;
    }
    
    // Called from timer interrupt at PROFILE_UPDATE_RATE
    bool IRAM_ATTR updateProfile(uint32_t* new_frequency) {
        if (state_.phase == MotionState::IDLE) {
            return false;  // Motion complete
        }
        
        switch (state_.phase) {
            case MotionState::ACCELERATING:
                // Increase velocity
                state_.current_velocity += state_.acceleration * UPDATE_PERIOD;
                
                // Check if reached max velocity
                if (state_.current_velocity >= state_.max_velocity) {
                    state_.current_velocity = state_.max_velocity;
                    state_.phase = MotionState::CONSTANT;
                }
                
                // Check if need to start decelerating
                if (state_.pulses_remaining <= state_.pulses_to_decel) {
                    state_.phase = MotionState::DECELERATING;
                }
                break;
                
            case MotionState::CONSTANT:
                // Check if time to decelerate
                float current_decel_distance = 
                    (state_.current_velocity * state_.current_velocity) / 
                    (2 * state_.acceleration);
                    
                if (state_.pulses_remaining <= current_decel_distance) {
                    state_.phase = MotionState::DECELERATING;
                }
                break;
                
            case MotionState::DECELERATING:
                // Decrease velocity
                state_.current_velocity -= state_.acceleration * UPDATE_PERIOD;
                
                // Check if stopped
                if (state_.current_velocity <= 0 || state_.pulses_remaining <= 0) {
                    state_.current_velocity = 0;
                    state_.phase = MotionState::IDLE;
                    *new_frequency = 0;
                    return false;  // Motion complete
                }
                break;
        }
        
        // Calculate new frequency
        *new_frequency = (uint32_t)state_.current_velocity;
        
        // Update position estimate
        float pulses_this_update = state_.current_velocity * UPDATE_PERIOD;
        state_.current_position += pulses_this_update;
        state_.pulses_remaining -= (int32_t)pulses_this_update;
        
        return true;  // Continue motion
    }
};
```

### MCPWM Motor Integration

```cpp
class MCPWMMotor : public MotorInterface {
private:
    MCPWMProfileController profile_controller_;
    esp_timer_handle_t profile_timer_;
    
    // Timer callback
    static void IRAM_ATTR profile_timer_callback(void* arg) {
        MCPWMMotor* motor = (MCPWMMotor*)arg;
        uint32_t new_frequency = 0;
        
        if (motor->profile_controller_.updateProfile(&new_frequency)) {
            // Update MCPWM frequency
            if (new_frequency > 0) {
                mcpwm_timer_set_period(motor->timer_, 
                    APB_CLK_FREQ / new_frequency);
            }
        } else {
            // Motion complete
            mcpwm_timer_stop(motor->timer_);
            motor->is_moving_ = false;
        }
    }
    
public:
    esp_err_t moveToPosition(float position, float velocity, float acceleration) override {
        float distance = position - current_position_;
        if (fabs(distance) < 0.001f) return ESP_OK;
        
        // Set direction
        gpio_set_level(dir_pin_, distance > 0 ? 1 : 0);
        
        // Convert to pulses
        float distance_pulses = distance * pulses_per_unit_;
        float velocity_pulses = velocity * pulses_per_unit_;
        float accel_pulses = acceleration * pulses_per_unit_;
        
        // Configure profile controller
        profile_controller_.startMotion(
            distance_pulses,
            velocity_pulses,
            accel_pulses
        );
        
        // Update state
        target_position_ = position;
        is_moving_ = true;
        
        // Start MCPWM
        mcpwm_timer_start(timer_);
        
        return ESP_OK;
    }
};
```

## Usage Example

```cpp
// Initialize motors
RMTMotor::Config servo_config = {
    .step_pin = GPIO_NUM_1,
    .dir_pin = GPIO_NUM_6,
    .enable_pin = GPIO_NUM_10,  // Via I2C expander
    .pos_complete_pin = GPIO_NUM_14,
    .z_signal_pin = GPIO_NUM_18,
    .rmt_channel = 0
};

auto servo = std::make_unique<RMTMotor>(servo_config);
servo->setPulsesPerUnit(400);      // 400 pulses/mm
servo->setLimits(-100, 100);       // ±100mm travel
servo->setVelocity(200, 1000);     // 200mm/s, 1000mm/s²

// Execute motion
servo->enable();
servo->moveToPosition(50.0);       // Move to 50mm at defaults
servo->moveToPosition(25.0, 100, 500); // Move to 25mm at 100mm/s, 500mm/s²
```

## Performance Characteristics

### RMT Implementation
- **CPU Usage**: Near zero during motion (hardware handles everything)
- **Profile Accuracy**: Exact pulse timing (12.5ns resolution at 80MHz)
- **Memory Usage**: ~4KB for profile buffer (1024 symbols)
- **Max Frequency**: 1-2 MHz pulse rate achievable

### MCPWM Implementation
- **CPU Usage**: Timer interrupt at 1kHz (minimal load)
- **Profile Accuracy**: ±1ms velocity updates
- **Memory Usage**: Minimal (profile state only)
- **Max Frequency**: ~200kHz typical

## Testing Strategy

1. **Unit Tests**
   - Profile calculation accuracy
   - Edge cases (zero distance, very short moves)
   - Parameter validation

2. **Integration Tests**
   - Actual motion vs calculated profile
   - Position accuracy at end of move
   - Multi-axis synchronization

3. **Performance Tests**
   - Maximum achievable speeds
   - CPU load during motion
   - Jitter measurement

## Future Enhancements (Post-MVP)

1. **S-Curve Profiles**: Add jerk limitation for smoother motion
2. **Custom Profiles**: Load arbitrary velocity curves
3. **Look-ahead**: Blend moves for continuous motion
4. **Adaptive Profiles**: Adjust based on load feedback
5. **Profile Visualization**: Real-time plotting via debug interface