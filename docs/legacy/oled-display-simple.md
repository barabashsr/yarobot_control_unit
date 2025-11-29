# Simple OLED Display for Tuning

## Overview
Minimal OLED display showing only essential information for motor tuning and debugging.

## Display Layout (128x64, 8 lines)

```
0:  25.5 > 100.0
1: 180.0 = 180.0
2:  -5.2 >  50.0  
3:DISABLED
4:DISABLED
5:DISABLED
E-STOP ACTIVE
I2C:OK  USB:OK
```

## Simple Implementation

```cpp
// Basic display update - no animations, no fancy graphics
void updateDisplay() {
    char line[22];
    
    // Clear buffer
    memset(display_buffer, 0, sizeof(display_buffer));
    
    // Show each axis status
    for (int i = 0; i < 6; i++) {
        if (!motors[i]->isEnabled()) {
            snprintf(line, sizeof(line), "%d:DISABLED", i);
        } else {
            float current = motors[i]->getCurrentPosition();
            float target = motors[i]->getTargetPosition();
            char status = '?';
            
            if (fabs(current - target) < 0.1f) {
                status = '=';  // At position
            } else if (motors[i]->isMoving()) {
                status = '>';  // Moving
            } else {
                status = '!';  // Stopped but not at target
            }
            
            // Simple format: axis: current status target
            snprintf(line, sizeof(line), "%d:%6.1f %c %6.1f", 
                    i, current, status, target);
        }
        
        oled_write_line(i, line);
    }
    
    // Line 6: Error/status
    if (ErrorManager::getInstance().hasErrors()) {
        oled_write_line(6, "ERROR ACTIVE");
    } else if (isEmergencyStopped()) {
        oled_write_line(6, "E-STOP ACTIVE");
    } else {
        oled_write_line(6, "");
    }
    
    // Line 7: System status
    snprintf(line, sizeof(line), "I2C:%s USB:%s", 
            i2c_healthy ? "OK" : "ERR",
            usb_connected ? "OK" : "NO");
    oled_write_line(7, line);
    
    // Send to display
    oled_update();
}

// Simple task - update every 200ms
void displayTask(void* param) {
    while (1) {
        updateDisplay();
        vTaskDelay(pdMS_TO_TICKS(200));  // 5Hz is plenty
    }
}
```

## Minimal OLED Driver

```cpp
// Just the essentials
#define OLED_ADDR  0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

static uint8_t oled_buffer[1024];  // 128x64/8

esp_err_t oled_init() {
    // Minimal init sequence for SSD1306
    uint8_t init_cmds[] = {
        0xAE,       // Display off
        0x20, 0x00, // Memory mode
        0x21, 0, 127, // Column address
        0x22, 0, 7,   // Page address
        0xC8,       // COM scan direction
        0xA1,       // Segment remap
        0x81, 0xCF, // Contrast
        0xA4,       // Display RAM
        0xA6,       // Normal display
        0xD5, 0x80, // Clock
        0x8D, 0x14, // Charge pump
        0xAF        // Display on
    };
    
    for (int i = 0; i < sizeof(init_cmds); i++) {
        i2c_write_byte(OLED_ADDR, 0x00, init_cmds[i]);
    }
    
    return ESP_OK;
}

void oled_write_line(uint8_t line, const char* text) {
    // Simple 8x8 font
    for (int i = 0; i < 21 && text[i]; i++) {
        int offset = (line * 128) + (i * 6);
        draw_char(offset, text[i]);
    }
}

void oled_update() {
    // Send buffer to display
    i2c_write_byte(OLED_ADDR, 0x00, 0x21); // Column
    i2c_write_byte(OLED_ADDR, 0x00, 0);
    i2c_write_byte(OLED_ADDR, 0x00, 127);
    
    i2c_write_byte(OLED_ADDR, 0x00, 0x22); // Page  
    i2c_write_byte(OLED_ADDR, 0x00, 0);
    i2c_write_byte(OLED_ADDR, 0x00, 7);
    
    // Send data
    i2c_write_block(OLED_ADDR, 0x40, oled_buffer, 1024);
}
```

## What We Show

1. **Current position** - For tuning moves
2. **Target position** - Where we're going  
3. **Status character**:
   - `=` At target
   - `>` Moving
   - `!` Error/stuck
4. **System errors** - E-stop, I2C fail
5. **Connection status** - USB and I2C health

## What We DON'T Show

- Animations
- Progress bars
- Velocity
- Fancy graphics
- Multiple display modes
- Menus

## Benefits

1. **Low CPU usage** - 5Hz update rate
2. **Simple code** - Easy to debug
3. **Essential info only** - Position and errors
4. **No complexity** - Just text, no graphics

This is perfect for motor tuning where you just need to see:
- Is it moving?
- Where is it?
- Where should it be?
- Any errors?