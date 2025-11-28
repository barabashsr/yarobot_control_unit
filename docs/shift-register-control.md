# Shift Register Control for Direction and Enable Pins

## Overview
Using serial shift registers (74HC595 or similar) to control motor direction and enable signals, saving GPIO pins while maintaining fast, reliable control.

## Shift Register Selection

### 74HC595 - 8-bit Serial-In, Parallel-Out
- **Outputs**: 8 parallel outputs per chip
- **Interface**: SPI-compatible (3-wire)
- **Speed**: Up to 100MHz clock
- **Cascadable**: Yes, for multiple chips
- **Output current**: 35mA per pin
- **Latching**: Yes, prevents glitches during shifts

### Pin Requirements
```
ESP32-S3 → Shift Register:
- MOSI  → SER (Serial data input)
- SCLK  → SRCLK (Shift register clock) 
- CS    → RCLK (Storage register clock/latch)
- (Optional) OE → Output Enable (active low)
```

## Control Signal Mapping

### For 7 Motors (5 Servos + 2 Steppers)
```
Shift Register 1 (U1):
Bit 0: X_DIR  (Servo)
Bit 1: X_EN   (Servo)
Bit 2: Y_DIR  (Servo)
Bit 3: Y_EN   (Servo)
Bit 4: Z_DIR  (Servo)
Bit 5: Z_EN   (Servo)
Bit 6: A_DIR  (Servo)
Bit 7: A_EN   (Servo)

Shift Register 2 (U2):
Bit 0: B_DIR  (Servo)
Bit 1: B_EN   (Servo)
Bit 2: C_DIR  (Stepper)
Bit 3: C_EN   (Stepper)
Bit 4: D_DIR  (Stepper)
Bit 5: D_EN   (Stepper)
Bit 6: SPARE_1
Bit 7: SPARE_2
```

## Hardware Design

### Cascaded Configuration
```
         ┌──────────┐      ┌──────────┐
ESP32 ───┤   U1     ├─────►┤   U2     ├────►
MOSI ────┤SER    Q7'├──────┤SER    Q7'├─ NC
SCLK ────┤SRCLK     ├──┬───┤SRCLK     ├
CS ──────┤RCLK      ├  └───┤RCLK      ├
         │          ├      │          ├
         │ Q0-Q7    ├      │ Q0-Q7    ├
         └────┬─────┘      └────┬─────┘
              │                 │
         Motor 0-3         Motor 4-6
         DIR/EN pins       DIR/EN pins
```

### Output Protection
```
74HC595 Output → 220Ω → Motor Driver Input
                  │
                  ├── 10kΩ → GND (Pull-down)
                  │
                  └── 0.1µF → GND (Noise filter)
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
    uint16_t current_state;  // Shadow register for 16 bits
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
        
        // Initialize all outputs to safe state (disabled)
        current_state = 0x0000;  // All enables OFF
        return update();
    }
```

### Fast Update Method
```cpp
    esp_err_t update() {
        // Send 16 bits for two cascaded shift registers
        uint8_t tx_data[2];
        tx_data[0] = (current_state >> 8) & 0xFF;  // U2 data
        tx_data[1] = current_state & 0xFF;         // U1 data
        
        spi_transaction_t trans = {
            .length = 16,
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
    // Called from ISR - disable all motors immediately
    void IRAM_ATTR emergencyDisableAll() {
        // Clear all enable bits (odd bits)
        uint16_t emergency_state = current_state & 0x5555;  // Keep DIR, clear EN
        
        // Direct SPI register access for speed
        uint8_t tx_data[2];
        tx_data[0] = (emergency_state >> 8) & 0xFF;
        tx_data[1] = emergency_state & 0xFF;
        
        // Bit-bang SPI for guaranteed timing in ISR
        gpio_set_level(SHIFT_REG_CS, 0);
        
        for (int i = 0; i < 16; i++) {
            gpio_set_level(SHIFT_REG_MOSI, 
                          (tx_data[i/8] & (0x80 >> (i%8))) ? 1 : 0);
            gpio_set_level(SHIFT_REG_SCLK, 1);
            gpio_set_level(SHIFT_REG_SCLK, 0);
        }
        
        gpio_set_level(SHIFT_REG_CS, 1);  // Latch outputs
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

1. **GPIO Savings**: Only 3-4 GPIOs control 14 signals
2. **Atomic Updates**: All signals change simultaneously
3. **EMI Reduction**: Slower edge rates than direct GPIO
4. **Expandability**: Easy to add more shift registers
5. **Cost Effective**: 74HC595 chips are inexpensive
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