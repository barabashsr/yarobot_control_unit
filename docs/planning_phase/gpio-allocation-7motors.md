# GPIO Allocation for 8-Axis System

> **⚠️ SUPERSEDED** - This planning document has been superseded by the finalized GPIO assignments.
> For current authoritative GPIO assignments, see `/docs/gpio-assignment.md`.
>
> **Key change (2025-12-06)**: MCP0 interrupts moved from GPIO3/46 (strapping pins) to GPIO47/48 to avoid boot mode conflicts.

> **TODO - ARCHITECTURE SESSION**: This document contains GPIO assignments that need final verification during the dedicated GPIO allocation architecture session. All conflicts are marked with TODO.

## Overview
GPIO allocation for 5 servo motors (X,Y,Z,A,B), 2 stepper motors (C,D), and 1 discrete axis (E) using ESP32-S3 DevKit with shift registers for DIR/EN control.

## Peripheral Assignment Strategy

### Fixed Assignments:
- **X (Servo)** → RMT Channel 0 + DMA (railway X axis)
- **Y (Servo)** → MCPWM0 Timer 0 + PCNT0 (gripper Y axis, dedicated)
- **Z (Servo)** → RMT Channel 1 + DMA (selector Z axis)
- **A (Servo)** → RMT Channel 2 + DMA (picker Z axis)
- **B (Servo)** → RMT Channel 3 + DMA (picker Y axis)
- **C (Stepper)** → MCPWM0 Timer 1 + PCNT1 (picker jaw with floating switch)
- **D (Stepper)** → LEDC (picker retractor, discrete positions)
- **E (Discrete)** → MCP23017 I2C expander outputs (linear drive, two-state)

## GPIO Pin Allocation

### Motor Control Signals (Pulse Outputs)
| Signal | GPIO | Peripheral | Notes |
|--------|------|------------|-------|
| X_STEP | GPIO2 | RMT_CH0 | X-axis servo pulses |
| Y_STEP | GPIO4 | MCPWM0_0A | Y-axis servo (internally routed to PCNT0) |
| Z_STEP | GPIO5 | RMT_CH1 | Z-axis servo pulses |
| A_STEP | GPIO6 | RMT_CH2 | A-axis servo pulses |
| B_STEP | GPIO7 | RMT_CH3 | B-axis servo pulses |
| C_STEP | GPIO15 | MCPWM0_1A | C-axis picker jaw (internally routed to PCNT1) |
| D_STEP | GPIO14 | LEDC_CH0 | D-axis retractor pulses |

### Direction and Enable (Via Shift Registers)
| Signal | GPIO | Function | Notes |
|--------|------|----------|-------|
| SR_MOSI | GPIO11 | SPI2_MOSI | Shift register data |
| SR_SCLK | GPIO12 | SPI2_CLK | Shift register clock |
| SR_CS | GPIO10 | SPI2_CS | Shift register latch |
| SR_OE | GPIO9 | GPIO | Output enable (optional) |

### Position Feedback Signals
| Signal | GPIO | Function | Notes |
|--------|------|----------|-------|
| X_POS_COMPLETE | GPIO35 | Input | X servo position reached |
| Y_POS_COMPLETE | GPIO36 | Input | Y servo position reached |
| Z_POS_COMPLETE | GPIO37 | Input | Z servo position reached |
| A_POS_COMPLETE | GPIO38 | Input | A servo position reached |
| B_POS_COMPLETE | GPIO39 | Input | B servo position reached |

### Z-Signal Inputs (Index Pulse)
| Signal | GPIO | Function | Notes |
|--------|------|----------|-------|
| X_Z_SIGNAL | GPIO40 | Input | X servo index pulse |
| Y_Z_SIGNAL | GPIO41 | Input | Y servo index pulse |
| Z_Z_SIGNAL | GPIO42 | Input | Z servo index pulse |
| A_Z_SIGNAL | GPIO48 | Input | A servo index pulse |
| B_Z_SIGNAL | GPIO47 | Input | B servo index pulse |

### Pulse Counting (PCNT) - Internal Routing
| Signal | GPIO | Peripheral | Notes |
|--------|------|------------|-------|
| Y_COUNT | Internal | PCNT0 | Internally routed from MCPWM0_0A |
| C_COUNT | Internal | PCNT1 | Internally routed from MCPWM0_1A |

**Note**: MCPWM outputs are internally connected to PCNT inputs via ESP32-S3 GPIO Matrix, eliminating the need for external wiring. This saves GPIO pins and improves reliability.

### I2C Bus (Limit Switches & I/O)
| Signal | GPIO | Function | Notes |
|--------|------|----------|-------|
| I2C_SDA | GPIO8 | I2C0_SDA | I2C data |
| I2C_SCL | GPIO18 | I2C0_SCL | I2C clock |
| INT0 | GPIO3 | Input | Interrupt from expander 0 |
| INT1 | GPIO46 | Input | Interrupt from expander 1 |

### Safety Signals
| Signal | GPIO | Function | Notes |
|--------|------|----------|-------|
| E_STOP | **TODO** | Input | Emergency stop (direct) - **TODO: Resolve GPIO conflict in architecture session** |

> **TODO - GPIO CONFLICT**: E-Stop pin assignment needs verification. freertos-task-architecture.md references GPIO12, but GPIO12 is used for SR_SCLK. GPIO16 was previously assigned but needs confirmation.

### USB Communication
| Signal | GPIO | Function | Notes |
|--------|------|----------|-------|
| USB_D- | GPIO19 | USB_OTG | USB data negative |
| USB_D+ | GPIO20 | USB_OTG | USB data positive |

## I2C Expander Allocation

### Expander 0 (Address 0x20) - Limit Switches
```
P00: X_LIMIT_MIN
P01: X_LIMIT_MAX
P02: Y_LIMIT_MIN
P03: Y_LIMIT_MAX
P04: Z_LIMIT_MIN
P05: Z_LIMIT_MAX
P06: A_LIMIT_MIN
P07: A_LIMIT_MAX
P10: B_LIMIT_MIN
P11: B_LIMIT_MAX
P12: C_LIMIT_MIN
P13: C_LIMIT_MAX (floating switch)
P14: D_LIMIT_MIN
P15: D_LIMIT_MAX
P16: E_LIMIT_MIN
P17: E_LIMIT_MAX
```

### Expander 1 (Address 0x21) - General Purpose I/O & E Axis Control

> **TODO - ARCHITECTURE SESSION**: E axis pin assignments on MCP23017 #2 need final confirmation. yaml-configuration-system.md shows GPB6/GPB7, but this conflicts with some GP output allocations.

```
P00-P07: General Purpose Inputs (GPA0-7)
P10-P15: General Purpose Outputs (GPB0-5)
P16: E_ENABLE (E-axis enable output) - TODO: Confirm GPB6
P17: E_DIR (E-axis direction output) - TODO: Confirm GPB7
```

## Shift Register Bit Mapping

### 24-bit Shift Register Chain (3x TPIC6B595N)
```
Register #1 (U1) - Motor DIR/EN (X,Y,Z,A):
Bit  0: X_DIR   (X-axis direction)
Bit  1: X_EN    (X-axis enable)
Bit  2: Y_DIR   (Y-axis direction)
Bit  3: Y_EN    (Y-axis enable)
Bit  4: Z_DIR   (Z-axis direction)
Bit  5: Z_EN    (Z-axis enable)
Bit  6: A_DIR   (A-axis direction)
Bit  7: A_EN    (A-axis enable)

Register #2 (U2) - Motor DIR/EN (B,C,D):
Bit  8: B_DIR   (B-axis direction)
Bit  9: B_EN    (B-axis enable)
Bit 10: C_DIR   (C-axis direction)
Bit 11: C_EN    (C-axis enable)
Bit 12: D_DIR   (D-axis direction)
Bit 13: D_EN    (D-axis enable)
Bit 14: SPARE_1
Bit 15: SPARE_2

Register #3 (U3) - Brake Control (via optocoupler → relay):
Bit 16: X_BRAKE_RLY (X-axis brake relay)
Bit 17: Y_BRAKE_RLY (Y-axis brake relay)
Bit 18: Z_BRAKE_RLY (Z-axis brake relay)
Bit 19: A_BRAKE_RLY (A-axis brake relay)
Bit 20: B_BRAKE_RLY (B-axis brake relay)
Bit 21: SPARE_RLY1
Bit 22: SPARE_RLY2
Bit 23: SPARE_RLY3

Note: Brake logic is active-low (0 = brake engaged, 1 = brake released)
      Fail-safe operation on power loss.
```

## C and D Independent Control

### C-Axis Picker Jaw (MCPWM + PCNT)
```cpp
class PickerJawAxis : public Axis {
private:
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    pcnt_unit_handle_t pcnt_unit;
    float last_object_width = 0.0f;
    bool floating_switch_active = false;
    
public:
    void onLimitSwitch(uint8_t switch_id, bool state) override {
        if (switch_id == LIMIT_MAX && state && is_closing) {
            // Floating switch triggered during closing
            floating_switch_active = true;
            last_object_width = getPosition() - home_position;
            
            // Publish event
            EventManager::publish(EVENT_OBJECT_DETECTED, {
                .axis = AXIS_C,
                .width = last_object_width,
                .timestamp = esp_timer_get_time()
            });
        }
    }
    
    float getLastObjectWidth() const { return last_object_width; }
};
```

### D-Axis Picker Retractor (LEDC)
```cpp
class RetractorAxis : public Axis {
private:
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    pcnt_unit_handle_t pcnt_unit;
    uint8_t current_motor;  // MOTOR_C or MOTOR_D
    SemaphoreHandle_t mutex;
    
public:
    void selectMotor(uint8_t motor) {
        if (motor != MOTOR_C && motor != MOTOR_D) return;
        
        xSemaphoreTake(mutex, portMAX_DELAY);
        
        // Stop current motion if any
        mcpwm_timer_stop(timer);
        
        // Update direction for selected motor
        current_motor = motor;
        
        // Reset position counter
        pcnt_unit_clear_count(pcnt_unit);
        
        xSemaphoreGive(mutex);
    }
    
    void startMotion(float velocity) {
        // Configure timer for requested velocity
        uint32_t frequency = fabsf(velocity * steps_per_mm);
        
        mcpwm_timer_config_t timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 10000000,  // 10MHz
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = 10000000 / frequency,
        };
        
        mcpwm_timer_set_period(timer, timer_config.period_ticks);
        
        // Set direction via shift register
        bool direction = (velocity > 0);
        shiftReg.setDirection(current_motor, direction);
        shiftReg.setEnable(current_motor, true);
        
        // Start motion
        mcpwm_timer_start(timer);
    }
    
    void stopOnSwitch() {
        // Called from limit switch interrupt
        mcpwm_timer_stop(timer);
        shiftReg.setEnable(current_motor, false);
    }
};
```

## GPIO Summary

### Total GPIO Usage:
- Motor step signals: 7 GPIOs (X,Y,Z,A,B,C,D - E uses I2C expander)
- Shift register control: 4 GPIOs
- Position feedback: 5 GPIOs
- Z-signal inputs: 5 GPIOs
- I2C bus: 4 GPIOs
- E-stop: 1 GPIO (TODO: confirm assignment)
- **Total: ~26 GPIOs used**

### Available for future use:
- GPIO 13, 17, 21, 22, 23, 26, 27, 32, 33, 34

> **Note**: E axis (linear drive) does not use direct GPIO - it is controlled via MCP23017 I2C expander pins.

## Initialization Code

```cpp
void initializeGPIOs() {
    // 1. Configure shift register GPIOs first (motors disabled)
    shiftRegController.init();
    
    // 2. Configure motor step outputs (except D which uses LEDC)
    gpio_config_t motor_conf = {
        .pin_bit_mask = (1ULL << GPIO_X_STEP) | (1ULL << GPIO_Y_STEP) | 
                       (1ULL << GPIO_Z_STEP) | (1ULL << GPIO_A_STEP) |
                       (1ULL << GPIO_B_STEP) | (1ULL << GPIO_C_STEP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&motor_conf);
    
    // D_STEP (GPIO14) is configured by LEDC internally
    
    // 3. Configure position feedback inputs
    gpio_config_t feedback_conf = {
        .pin_bit_mask = (1ULL << GPIO_X_POS_COMPLETE) | 
                       (1ULL << GPIO_Y_POS_COMPLETE) |
                       (1ULL << GPIO_Z_POS_COMPLETE) | 
                       (1ULL << GPIO_A_POS_COMPLETE) |
                       (1ULL << GPIO_B_POS_COMPLETE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&feedback_conf);
    
    // 4. Configure Z-signal inputs  
    gpio_config_t zsignal_conf = {
        .pin_bit_mask = (1ULL << GPIO_X_Z_SIGNAL) | 
                       (1ULL << GPIO_Y_Z_SIGNAL) |
                       (1ULL << GPIO_Z_Z_SIGNAL) | 
                       (1ULL << GPIO_A_Z_SIGNAL) |
                       (1ULL << GPIO_B_Z_SIGNAL),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,  // Interrupt on index pulse
    };
    gpio_config(&zsignal_conf);
    
    // 5. Configure E-stop (highest priority)
    gpio_config_t estop_conf = {
        .pin_bit_mask = (1ULL << GPIO_E_STOP),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&estop_conf);
    
    // 6. Configure I2C interrupt pins
    gpio_config_t i2c_int_conf = {
        .pin_bit_mask = (1ULL << GPIO_INT0) | (1ULL << GPIO_INT1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&i2c_int_conf);
}
```