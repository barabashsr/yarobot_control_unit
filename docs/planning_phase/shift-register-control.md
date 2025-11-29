# Shift Register Control for Direction and Enable Pins

## Overview
Using TPIC6B595N serial shift registers to control motor direction, enable, and brake signals with native 24V open-drain outputs, saving GPIO pins while maintaining fast, reliable control.

## Shift Register Selection

### TPIC6B595N - 8-bit Serial-In, Parallel-Out (24V Open-Drain)
- **Outputs**: 8 open-drain outputs per chip (150mA continuous each)
- **Interface**: SPI-compatible (3-wire)
- **Speed**: Up to 25MHz clock
- **Cascadable**: Yes, for multiple chips
- **Output voltage**: Up to 50V (VDD clamp)
- **Logic input**: 3.3V compatible (TTL levels)
- **Latching**: Yes, prevents glitches during shifts

**Why TPIC6B595N over 74HC595:**
- Native 24V open-drain outputs - no level shifters needed
- Higher current capability (150mA vs 35mA)
- Direct interface to industrial motor drivers
- Fail-safe with pull-up resistors (outputs float high when IC unpowered)

### Pin Requirements
```
ESP32-S3 → Shift Register:
- MOSI  → SER (Serial data input)
- SCLK  → SRCLK (Shift register clock) 
- CS    → RCLK (Storage register clock/latch)
- (Optional) OE → Output Enable (active low)
```

## Control Signal Mapping

### 24-bit Chain: 3x TPIC6B595N (DIR/EN + Brakes)
```
Shift Register 1 (U1) - Motor DIR/EN (X,Y,Z,A):
Bit 0: X_DIR  (Servo)
Bit 1: X_EN   (Servo)
Bit 2: Y_DIR  (Servo)
Bit 3: Y_EN   (Servo)
Bit 4: Z_DIR  (Servo)
Bit 5: Z_EN   (Servo)
Bit 6: A_DIR  (Servo)
Bit 7: A_EN   (Servo)

Shift Register 2 (U2) - Motor DIR/EN (B,C,D):
Bit 0: B_DIR  (Servo)
Bit 1: B_EN   (Servo)
Bit 2: C_DIR  (Stepper)
Bit 3: C_EN   (Stepper)
Bit 4: D_DIR  (Stepper)
Bit 5: D_EN   (Stepper)
Bit 6: SPARE_1
Bit 7: SPARE_2

Shift Register 3 (U3) - Brake Control:
Bit 0: X_BRAKE_RLY (Servo brake relay)
Bit 1: Y_BRAKE_RLY (Servo brake relay)
Bit 2: Z_BRAKE_RLY (Servo brake relay)
Bit 3: A_BRAKE_RLY (Servo brake relay)
Bit 4: B_BRAKE_RLY (Servo brake relay)
Bit 5: SPARE_RLY1
Bit 6: SPARE_RLY2
Bit 7: SPARE_RLY3

Note: Brake logic is active-low (0 = brake engaged, 1 = brake released)
      This ensures fail-safe operation on power loss.
```

## Hardware Design

### Cascaded Configuration (3x TPIC6B595N)
```
         ┌──────────┐      ┌──────────┐      ┌──────────┐
ESP32 ───┤   U1     ├─────►┤   U2     ├─────►┤   U3     ├────► NC
MOSI ────┤SER    Q7'├──────┤SER    Q7'├──────┤SER    Q7'├
SCLK ────┤SRCLK     ├──┬───┤SRCLK     ├──┬───┤SRCLK     ├
CS ──────┤RCLK      ├  └───┤RCLK      ├  └───┤RCLK      ├
         │          ├      │          ├      │          ├
         │ DRAIN0-7 ├      │ DRAIN0-7 ├      │ DRAIN0-7 ├
         └────┬─────┘      └────┬─────┘      └────┬─────┘
              │                 │                 │
         X,Y,Z,A           B,C,D             Brake Relays
         DIR/EN            DIR/EN            (5 axes)
```

### Output Configuration (Open-Drain with Pull-ups)
```
                    24V
                     │
                   [10kΩ] Pull-up resistor
                     │
TPIC6B595N Output ───┼───────► Motor Driver Input (24V logic)
(Open Drain)         │
                   [0.1µF] Noise filter (optional)
                     │
                    GND

Logic: Output ON (sinking) = Signal LOW
       Output OFF (open) = Signal HIGH (via pull-up)
```

## Software Implementation

### SPI Configuration
```cpp
#define SHIFT_REG_MOSI    GPIO_NUM_11
#define SHIFT_REG_SCLK    GPIO_NUM_12  
#define SHIFT_REG_CS      GPIO_NUM_10
#define SHIFT_REG_OE      GPIO_NUM_9    // Optional

class ShiftRegisterController {
private:
    spi_device_handle_t spi_handle;
    uint32_t current_state;  // Shadow register for 24 bits (3x TPIC6B595N)
    SemaphoreHandle_t mutex;
    
public:
    esp_err_t init() {
        // Configure SPI bus
        spi_bus_config_t buscfg = {
            .mosi_io_num = SHIFT_REG_MOSI,
            .miso_io_num = -1,  // Not used
            .sclk_io_num = SHIFT_REG_SCLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 16
        };
        
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));
        
        // Configure SPI device
        spi_device_interface_config_t devcfg = {
            .mode = 0,  // SPI mode 0
            .clock_speed_hz = 10 * 1000 * 1000,  // 10MHz
            .spics_io_num = SHIFT_REG_CS,
            .queue_size = 1,
            .pre_cb = NULL,
            .post_cb = NULL,
            .flags = SPI_DEVICE_NO_DUMMY
        };
        
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));
        
        // Configure output enable if used
        if (SHIFT_REG_OE >= 0) {
            gpio_set_direction(SHIFT_REG_OE, GPIO_MODE_OUTPUT);
            gpio_set_level(SHIFT_REG_OE, 0);  // Enable outputs
        }
        
        // Create mutex for thread safety
        mutex = xSemaphoreCreateMutex();
        
        // Initialize all outputs to safe state (motors disabled, brakes engaged)
        // Bits 0-15: DIR/EN = 0 (all disabled)
        // Bits 16-23: Brakes = 0 (all engaged, active-low logic)
        current_state = 0x000000;
        return update();
    }
```

### Fast Update Method
```cpp
    esp_err_t update() {
        // Send 24 bits for three cascaded TPIC6B595N shift registers
        uint8_t tx_data[3];
        tx_data[0] = (current_state >> 16) & 0xFF;  // U3 data (brakes)
        tx_data[1] = (current_state >> 8) & 0xFF;   // U2 data (B,C,D DIR/EN)
        tx_data[2] = current_state & 0xFF;          // U1 data (X,Y,Z,A DIR/EN)

        spi_transaction_t trans = {
            .length = 24,
            .tx_buffer = tx_data,
            .rx_buffer = NULL
        };

        return spi_device_transmit(spi_handle, &trans);
    }
```

### Motor Control Methods
```cpp
    // Set direction for a motor
    void setDirection(uint8_t motor, bool direction) {
        if (motor >= 7) return;
        
        xSemaphoreTake(mutex, portMAX_DELAY);
        
        uint8_t bit_pos = motor * 2;  // DIR is even bits
        if (direction) {
            current_state |= (1 << bit_pos);
        } else {
            current_state &= ~(1 << bit_pos);
        }
        
        update();
        xSemaphoreGive(mutex);
    }
    
    // Enable/disable motor
    void setEnable(uint8_t motor, bool enable) {
        if (motor >= 7) return;
        
        xSemaphoreTake(mutex, portMAX_DELAY);
        
        uint8_t bit_pos = motor * 2 + 1;  // EN is odd bits
        if (enable) {
            current_state |= (1 << bit_pos);
        } else {
            current_state &= ~(1 << bit_pos);
        }
        
        update();
        xSemaphoreGive(mutex);
    }
```

### Batch Updates
```cpp
    // Update multiple motors at once (atomic operation)
    void batchUpdate(uint8_t motor_mask, bool* directions, bool* enables) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        
        for (uint8_t motor = 0; motor < 7; motor++) {
            if (motor_mask & (1 << motor)) {
                uint8_t dir_bit = motor * 2;
                uint8_t en_bit = motor * 2 + 1;
                
                // Update direction
                if (directions[motor]) {
                    current_state |= (1 << dir_bit);
                } else {
                    current_state &= ~(1 << dir_bit);
                }
                
                // Update enable
                if (enables[motor]) {
                    current_state |= (1 << en_bit);
                } else {
                    current_state &= ~(1 << en_bit);
                }
            }
        }
        
        update();
        xSemaphoreGive(mutex);
    }
```

### Emergency Stop Integration
```cpp
    // Called from ISR - disable all motors and engage all brakes immediately
    void IRAM_ATTR emergencyDisableAll() {
        // Clear all enable bits (odd bits in lower 16 bits)
        // Clear all brake bits (bits 16-20) to engage brakes (active-low)
        uint32_t emergency_state = current_state & 0x005555;  // Keep DIR, clear EN and brakes

        // Direct SPI register access for speed
        uint8_t tx_data[3];
        tx_data[0] = (emergency_state >> 16) & 0xFF;  // Brakes engaged (0)
        tx_data[1] = (emergency_state >> 8) & 0xFF;
        tx_data[2] = emergency_state & 0xFF;

        // Bit-bang SPI for guaranteed timing in ISR
        gpio_set_level(SHIFT_REG_CS, 0);

        for (int i = 0; i < 24; i++) {
            gpio_set_level(SHIFT_REG_MOSI,
                          (tx_data[i/8] & (0x80 >> (i%8))) ? 1 : 0);
            gpio_set_level(SHIFT_REG_SCLK, 1);
            gpio_set_level(SHIFT_REG_SCLK, 0);
        }

        gpio_set_level(SHIFT_REG_CS, 1);  // Latch outputs
    }

    // Brake control methods (bits 16-20)
    void engageBrake(uint8_t axis) {
        if (axis > 4) return;  // Only X,Y,Z,A,B have brakes
        xSemaphoreTake(mutex, portMAX_DELAY);
        current_state &= ~(1 << (16 + axis));  // Clear bit = engage (active-low)
        update();
        xSemaphoreGive(mutex);
    }

    void releaseBrake(uint8_t axis) {
        if (axis > 4) return;
        xSemaphoreTake(mutex, portMAX_DELAY);
        current_state |= (1 << (16 + axis));   // Set bit = release
        update();
        xSemaphoreGive(mutex);
    }
};
```

## Motor Channel Mapping

```cpp
enum MotorChannel {
    MOTOR_X = 0,  // Servo - High speed
    MOTOR_Y = 1,  // Servo - Slow speed (MCPWM)
    MOTOR_Z = 2,  // Servo - High speed
    MOTOR_A = 3,  // Servo - High speed
    MOTOR_B = 4,  // Servo - High speed
    MOTOR_C = 5,  // Stepper - ON/OFF
    MOTOR_D = 6   // Stepper - ON/OFF
};

// Human-readable names
const char* motor_names[] = {
    "X", "Y", "Z", "A", "B", "C", "D"
};
```

## Timing Considerations

### Direction Setup Time
```cpp
void prepareMove(uint8_t motor, bool direction) {
    // Set direction first
    shiftReg.setDirection(motor, direction);
    
    // Wait for driver direction setup time
    // Most drivers need 5-20µs
    esp_rom_delay_us(20);
    
    // Now safe to enable and send pulses
    shiftReg.setEnable(motor, true);
}
```

### Update Latency
- SPI clock: 10MHz = 100ns per bit
- 16 bits transfer: 1.6µs
- Latch time: ~10ns
- **Total update time: < 2µs**

This is fast enough for motor control where direction changes happen at motion start/end.

## Power-On Safety

```cpp
void initializeShiftRegisters() {
    // Before SPI init, ensure safe GPIO states
    gpio_set_direction(SHIFT_REG_OE, GPIO_MODE_OUTPUT);
    gpio_set_level(SHIFT_REG_OE, 1);  // Disable all outputs
    
    // Initialize SPI and shift registers
    shiftReg.init();
    
    // Set safe state (all motors disabled)
    for (int i = 0; i < 7; i++) {
        shiftReg.setDirection(i, 0);
        shiftReg.setEnable(i, 0);
    }
    
    // Enable outputs
    gpio_set_level(SHIFT_REG_OE, 0);
}
```

## Advantages of This Approach

1. **GPIO Savings**: Only 3-4 GPIOs control 24 signals (3x TPIC6B595N)
2. **Atomic Updates**: All signals change simultaneously
3. **Native 24V**: No level shifters needed for motor drivers
4. **Expandability**: Easy to add more shift registers
5. **Fail-Safe**: Open-drain with pull-ups ensures safe state on power loss
6. **Reliable**: Latched outputs prevent glitches

## Integration with Motor Control

```cpp
class MotorBase {
protected:
    uint8_t motor_id;
    ShiftRegisterController* shift_reg;
    
public:
    void setDirection(bool forward) {
        shift_reg->setDirection(motor_id, forward);
    }
    
    void enable() {
        // Set direction first, then enable
        esp_rom_delay_us(20);  // Direction setup time
        shift_reg->setEnable(motor_id, true);
    }
    
    void disable() {
        shift_reg->setEnable(motor_id, false);
    }
};
```