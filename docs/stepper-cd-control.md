# Stepper C/D Control Architecture

## Overview
Control architecture for C and D stepper motors with different requirements:
- C: Full position and velocity control (switch between discrete positions)
- D: Velocity-only or simplified position control

## Peripheral Options for ESP32-S3

### Available Peripherals for Step/Dir Control
1. **RMT** (4 channels) - Already used by X, Z, A, B
2. **MCPWM** (2 units × 3 timers) - Timer 0 for Y, Timer 1 available
3. **LEDC** (8 channels) - Simple PWM, good for velocity
4. **GPTimer** (4 timers) - Software step generation
5. **I2S** - Complex, overkill for steppers

## Proposed Architecture

### Option 1: Shared MCPWM with Full Control for Both
```
MCPWM0 Timer 1 + PCNT1 → Dynamically switched between C and D
- Pro: Both motors get full position control
- Con: Cannot move simultaneously
```

### Option 2: Split Control (Recommended)
```
C: MCPWM0 Timer 1 + PCNT1 (Full position + velocity control)
D: LEDC Channel 0 (Velocity control only)
```

### Option 3: Software Timer for D
```
C: MCPWM0 Timer 1 + PCNT1 (Full control)
D: GPTimer 0 + Software stepping (Velocity with basic position)
```

## Implementation: Option 2 (Split Control)

### C Motor - Full Position Control (MCPWM + PCNT)
```cpp
class StepperC : public MotorBase {
private:
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator_handle;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
    pcnt_unit_handle_t pcnt_unit;
    pcnt_channel_handle_t pcnt_channel;
    
    // Discrete positions for C
    struct Position {
        const char* name;
        float position_mm;
        float velocity_mm_s;
    };
    
    static constexpr Position positions[] = {
        {"HOME", 0.0f, 10.0f},
        {"POS1", 25.0f, 20.0f},
        {"POS2", 50.0f, 20.0f},
        {"POS3", 75.0f, 20.0f},
        {"MAX", 100.0f, 10.0f}
    };
    
    uint8_t current_position_index = 0;
    
public:
    esp_err_t init() override {
        // Configure MCPWM for precise stepping
        mcpwm_timer_config_t timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 10000000,  // 10MHz
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = 10000,  // Variable based on speed
        };
        mcpwm_new_timer(&timer_config, &timer);
        
        // Configure operator
        mcpwm_operator_config_t operator_config = {
            .group_id = 0,
        };
        mcpwm_new_operator(&operator_config, &operator_handle);
        mcpwm_operator_connect_timer(operator_handle, timer);
        
        // Configure PWM generator for 50% duty cycle
        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = GPIO_CD_STEP,
        };
        mcpwm_new_generator(operator_handle, &generator_config, &generator);
        
        // Set up actions for 50% duty square wave
        mcpwm_gen_timer_event_action_t event_action = {
            .direction = MCPWM_TIMER_DIRECTION_UP,
            .event = MCPWM_TIMER_EVENT_EMPTY,
            .action = MCPWM_GEN_ACTION_TOGGLE,
        };
        mcpwm_generator_set_actions_on_timer_event(generator, event_action);
        
        // Configure PCNT for position tracking
        pcnt_unit_config_t unit_config = {
            .high_limit = 100000,
            .low_limit = -100000,
        };
        pcnt_new_unit(&unit_config, &pcnt_unit);
        
        pcnt_chan_config_t chan_config = {
            .edge_gpio_num = GPIO_CD_STEP,
            .level_gpio_num = -1,
        };
        pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_channel);
        
        // Count on rising edge
        pcnt_channel_set_edge_action(pcnt_channel, 
                                    PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                    PCNT_CHANNEL_EDGE_ACTION_HOLD);
        
        pcnt_unit_enable(pcnt_unit);
        pcnt_unit_start(pcnt_unit);
        
        return ESP_OK;
    }
    
    void moveToPosition(uint8_t position_index) {
        if (position_index >= sizeof(positions)/sizeof(positions[0])) {
            return;
        }
        
        Position target = positions[position_index];
        float current_pos_mm = getCurrentPosition();
        float distance = target.position_mm - current_pos_mm;
        
        // Set direction
        shiftReg.setDirection(MOTOR_C, distance > 0);
        
        // Configure velocity
        setVelocity(target.velocity_mm_s);
        
        // Enable motor
        shiftReg.setEnable(MOTOR_C, true);
        
        // Start motion
        mcpwm_timer_start(timer);
        
        current_position_index = position_index;
    }
    
    void stopOnLimitSwitch() {
        mcpwm_timer_stop(timer);
        shiftReg.setEnable(MOTOR_C, false);
        
        // Check which limit was hit
        if (motor_state[MOTOR_C].at_min_limit) {
            current_position_index = 0;  // HOME
            pcnt_unit_clear_count(pcnt_unit);
        } else if (motor_state[MOTOR_C].at_max_limit) {
            current_position_index = 4;  // MAX
            // Set counter to max position
            int32_t max_counts = positions[4].position_mm * steps_per_mm;
            pcnt_unit_set_count(pcnt_unit, max_counts);
        }
    }
    
    float getCurrentPosition() override {
        int32_t count;
        pcnt_unit_get_count(pcnt_unit, &count);
        return (float)count / steps_per_mm;
    }
};
```

### D Motor - Velocity Control (LEDC)
```cpp
class StepperD : public MotorBase {
private:
    ledc_channel_t channel = LEDC_CHANNEL_0;
    ledc_timer_t timer = LEDC_TIMER_0;
    
    // D motor states (ON/OFF positions)
    enum DPosition {
        D_OFF = 0,
        D_ON = 1
    };
    
    DPosition current_state = D_OFF;
    float travel_distance_mm = 50.0f;  // Distance between ON/OFF
    
public:
    esp_err_t init() override {
        // Configure LEDC timer
        ledc_timer_config_t timer_config = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_1_BIT,  // 1-bit for 50% duty
            .timer_num = timer,
            .freq_hz = 1000,  // Start at 1kHz
            .clk_cfg = LEDC_AUTO_CLK
        };
        ledc_timer_config(&timer_config);
        
        // Configure LEDC channel
        ledc_channel_config_t channel_config = {
            .gpio_num = GPIO_D_STEP,  // Need separate GPIO for D
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channel,
            .timer_sel = timer,
            .duty = 1,  // 50% duty (with 1-bit resolution)
            .hpoint = 0
        };
        ledc_channel_config(&channel_config);
        
        return ESP_OK;
    }
    
    void moveToState(DPosition target_state) {
        if (target_state == current_state) return;
        
        // Set direction
        bool forward = (target_state == D_ON);
        shiftReg.setDirection(MOTOR_D, forward);
        shiftReg.setEnable(MOTOR_D, true);
        
        // Set velocity (fixed for D)
        setVelocity(25.0f);  // 25mm/s
        
        // Start motion
        ledc_set_freq(LEDC_LOW_SPEED_MODE, timer, 
                     (uint32_t)(25.0f * steps_per_mm));
        
        // Motion will stop on limit switch interrupt
        current_state = target_state;
    }
    
    void setVelocity(float velocity_mm_s) override {
        uint32_t frequency = (uint32_t)(fabsf(velocity_mm_s) * steps_per_mm);
        
        // LEDC frequency limits
        if (frequency < 1) frequency = 1;
        if (frequency > 40000) frequency = 40000;
        
        ledc_set_freq(LEDC_LOW_SPEED_MODE, timer, frequency);
    }
    
    void stop() override {
        // Stop by setting frequency to 0
        ledc_set_freq(LEDC_LOW_SPEED_MODE, timer, 0);
        shiftReg.setEnable(MOTOR_D, false);
    }
    
    void stopOnLimitSwitch() {
        stop();
        
        // Update state based on which limit
        if (motor_state[MOTOR_D].at_min_limit) {
            current_state = D_OFF;
        } else if (motor_state[MOTOR_D].at_max_limit) {
            current_state = D_ON;
        }
    }
    
    const char* getStateName() {
        return current_state == D_OFF ? "OFF" : "ON";
    }
};
```

### Alternative: D Motor with GPTimer (Better Position Tracking)
```cpp
class StepperD_GPTimer : public MotorBase {
private:
    gptimer_handle_t timer;
    volatile int32_t step_count = 0;
    volatile int32_t target_steps = 0;
    volatile bool motion_complete = false;
    
    static bool IRAM_ATTR timer_cb(gptimer_handle_t timer, 
                                   const gptimer_alarm_event_data_t *edata,
                                   void *user_data) {
        StepperD_GPTimer* motor = (StepperD_GPTimer*)user_data;
        
        // Toggle step pin
        static bool step_state = false;
        gpio_set_level(GPIO_D_STEP, step_state);
        step_state = !step_state;
        
        // Count steps (on rising edge)
        if (step_state) {
            motor->step_count++;
            
            // Check if target reached
            if (motor->step_count >= motor->target_steps) {
                motor->motion_complete = true;
                gptimer_stop(timer);
                return false;  // Don't reload
            }
        }
        
        return true;  // Reload timer
    }
    
public:
    esp_err_t init() override {
        // Create timer
        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000,  // 1MHz, 1us resolution
        };
        gptimer_new_timer(&timer_config, &timer);
        
        // Set up alarm (for step frequency)
        gptimer_alarm_config_t alarm_config = {
            .reload_count = 0,
            .alarm_count = 1000,  // 1kHz default
            .flags.auto_reload_on_alarm = true,
        };
        gptimer_set_alarm_action(timer, &alarm_config);
        
        // Register callback
        gptimer_event_callbacks_t cbs = {
            .on_alarm = timer_cb,
        };
        gptimer_register_event_callbacks(timer, &cbs, this);
        
        gptimer_enable(timer);
        
        return ESP_OK;
    }
    
    void moveToPosition(float target_mm) {
        float distance = target_mm - getCurrentPosition();
        target_steps = abs((int32_t)(distance * steps_per_mm));
        
        // Set direction
        shiftReg.setDirection(MOTOR_D, distance > 0);
        shiftReg.setEnable(MOTOR_D, true);
        
        // Reset counters
        step_count = 0;
        motion_complete = false;
        
        // Start timer
        gptimer_start(timer);
    }
    
    void setVelocity(float velocity_mm_s) override {
        uint32_t frequency = (uint32_t)(fabsf(velocity_mm_s) * steps_per_mm);
        uint64_t alarm_count = 1000000 / (frequency * 2);  // For toggle
        
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = alarm_count,
        };
        gptimer_set_alarm_action(timer, &alarm_config);
    }
};
```

## GPIO Allocation Update

Since D needs its own step signal for independent control:

| Signal | GPIO | Function | Notes |
|--------|------|----------|-------|
| D_STEP | GPIO14 | LEDC_CH0 or GPTimer | D motor step signal |
| CD_STEP | GPIO15 | MCPWM0_1A | C motor step signal (renamed) |

## Command Interface for C/D

```cpp
// Move C to predefined position
// MOVC <position_index>
Response cmdMoveC(uint8_t position_index) {
    if (position_index >= 5) {
        return Response::error("Invalid C position (0-4)");
    }
    
    motorC.moveToPosition(position_index);
    return Response::ok(String("C moving to ") + 
                       motorC.getPositionName(position_index));
}

// Move D to ON/OFF state  
// MOVD <state>  (0=OFF, 1=ON)
Response cmdMoveD(uint8_t state) {
    if (state > 1) {
        return Response::error("Invalid D state (0=OFF, 1=ON)");
    }
    
    motorD.moveToState(state ? D_ON : D_OFF);
    return Response::ok(String("D moving to ") + (state ? "ON" : "OFF"));
}

// Get C/D status
// STATCD
Response cmdStatCD() {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
             "C: %s (%.1fmm) D: %s",
             motorC.getCurrentPositionName(),
             motorC.getCurrentPosition(),
             motorD.getStateName());
    return Response::ok(buffer);
}
```

## Advantages of Split Control

1. **Independent Operation**: C and D can move simultaneously if needed
2. **Optimized Control**: C gets full position control, D gets simple velocity
3. **Resource Efficient**: LEDC is lightweight for simple D control
4. **Flexible**: Can upgrade D to GPTimer if position tracking needed
5. **Clear Separation**: Easier to debug and maintain

## Motion Coordination

```cpp
// Example: Coordinated C/D movement
void executeCDSequence() {
    // Move D to OFF position first
    motorD.moveToState(D_OFF);
    waitForMotorStop(MOTOR_D);
    
    // Move C to position 2
    motorC.moveToPosition(2);
    waitForMotorStop(MOTOR_C);
    
    // Move D to ON
    motorD.moveToState(D_ON);
    waitForMotorStop(MOTOR_D);
}
```