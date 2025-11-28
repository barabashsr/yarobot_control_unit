# OLED Display Interface Design (Simplified)

## Overview
Minimal display implementation for 128x64 OLED (SSD1306) showing essential motor tuning information. Focus on debugging and position feedback only.

## Display Layout

### Normal Operation Mode (8 lines × 21 chars)
```
┌─────────────────────────┐
│0:C: 25.5 T:100.0 ═════► │  Line 0: Axis 0 status
│1:C:180.0 T:180.0 ✓      │  Line 1: Axis 1 status  
│2:C:  0.0 T: 50.0 ════►  │  Line 2: Axis 2 status
│3:C:-45.2 T:-45.2 ✓      │  Line 3: Axis 3 status
│4:DISABLED               │  Line 4: Axis 4 status
│5:DISABLED               │  Line 5: Axis 5 status
│─────────────────────────│  Line 6: Separator
│USB:OK  I2C:OK  RUN:5:32 │  Line 7: System status
└─────────────────────────┘
```

### Format Details

#### Axis Status Line Format
```
<axis>:C:<current> T:<target> <state>

States:
═════►  - Moving forward (animated)
◄═════  - Moving backward (animated)
✓       - At position (position_complete = true)
•       - Stopped but not at target
!       - Error/Limit hit
```

#### Compact Format (when all axes shown)
```
0: 25.5→100.0 ►
1:180.0=180.0 ✓
2:LIMIT MIN
```

## Display Modes

### 1. Overview Mode (Default)
Shows all 6 axes with compact status

```cpp
typedef struct {
    uint8_t mode;
    uint8_t selected_axis;    // For detailed view
    uint8_t error_display_time;
    bool show_debug;
} DisplayState_t;

void renderOverviewMode(DisplayState_t* state) {
    oled_clear();
    
    // Render each axis
    for (int i = 0; i < 6; i++) {
        char line[22];
        
        if (!motors[i]->isEnabled()) {
            snprintf(line, sizeof(line), "%d:DISABLED", i);
        } else {
            float current = motors[i]->getCurrentPosition();
            float target = motors[i]->getTargetPosition();
            
            // Determine state symbol
            char symbol = '•';  // Stopped
            if (motors[i]->isMoving()) {
                symbol = (current < target) ? '>' : '<';
            } else if (motors[i]->isPositionComplete()) {
                symbol = '=';  // Use = instead of ✓ for ASCII
            }
            
            snprintf(line, sizeof(line), "%d:%5.1f→%5.1f %c",
                    i, current, target, symbol);
        }
        
        oled_write_line(i, line);
    }
    
    // System status line
    renderStatusLine(7);
}
```

### 2. Detailed Axis Mode
Shows single axis with more information

```
┌─────────────────────────┐
│     AXIS 0 DETAIL       │
│Pos:    25.50 mm         │
│Target: 100.00 mm        │
│Vel:    125.5 mm/s       │
│Remaining: 74.50 mm      │
│State: MOVING            │
│Limits: -100.0 to 100.0  │
│USB:OK I2C:OK  T:25.3°C  │
└─────────────────────────┘
```

### 3. Error Display Mode
Automatically shows when error occurs

```
┌─────────────────────────┐
│  ⚠ EMERGENCY STOP ⚠     │
│                         │
│  Press RST button       │
│  after releasing        │
│  E-stop                 │
│                         │
│─────────────────────────│
│Last: Axis 2 @ 45.2mm    │
└─────────────────────────┘
```

### 4. Calibration Mode
Shows during homing/calibration

```
┌─────────────────────────┐
│  CALIBRATING AXIS 0     │
│                         │
│  Phase: Finding home    │
│  ████████░░░░░░ 50%     │
│                         │
│  Speed: 10.0 mm/s       │
│─────────────────────────│
│[STOP] to abort          │
└─────────────────────────┘
```

## Animation System

### Progress Indicators

```cpp
// Animation characters for motion
const char* PROGRESS_CHARS[] = {
    "       ",  // Empty
    "█      ",
    "██     ", 
    "███    ",
    "████   ",
    "█████  ",
    "██████ ",
    "███████"
};

const char* REVERSE_PROGRESS[] = {
    "       ",
    "      █",
    "     ██",
    "    ███",
    "   ████",
    "  █████",
    " ██████",
    "███████"
};

typedef struct {
    uint8_t animation_frame;
    uint32_t last_update_ms;
} AnimationState_t;

void updateMotionAnimation(uint8_t axis, AnimationState_t* anim) {
    if (!motors[axis]->isMoving()) return;
    
    uint32_t now = esp_timer_get_time() / 1000;
    if (now - anim->last_update_ms > 100) {  // 10Hz animation
        anim->animation_frame = (anim->animation_frame + 1) % 8;
        anim->last_update_ms = now;
    }
}
```

### Blinking for Alerts

```cpp
void renderAlert(const char* message, bool blink) {
    static bool visible = true;
    static uint32_t last_blink = 0;
    
    if (blink) {
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - last_blink > 500) {  // 1Hz blink
            visible = !visible;
            last_blink = now;
        }
    }
    
    if (visible) {
        oled_write_line_centered(3, message);
    }
}
```

## Display Update Strategy

### Update Task

```cpp
void displayUpdateTask(void* pvParameters) {
    DisplayState_t state = {
        .mode = DISPLAY_MODE_OVERVIEW,
        .selected_axis = 0,
        .error_display_time = 0,
        .show_debug = false
    };
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t update_period = pdMS_TO_TICKS(100);  // 10Hz
    
    AnimationState_t animations[6] = {0};
    
    while (1) {
        vTaskDelayUntil(&last_wake_time, update_period);
        
        // Check for mode changes
        updateDisplayMode(&state);
        
        // Render based on current mode
        switch (state.mode) {
            case DISPLAY_MODE_OVERVIEW:
                renderOverviewMode(&state);
                break;
                
            case DISPLAY_MODE_DETAIL:
                renderDetailMode(&state);
                break;
                
            case DISPLAY_MODE_ERROR:
                renderErrorMode(&state);
                break;
                
            case DISPLAY_MODE_CALIBRATION:
                renderCalibrationMode(&state);
                break;
                
            case DISPLAY_MODE_MENU:
                renderMenuMode(&state);
                break;
        }
        
        // Update animations
        for (int i = 0; i < 6; i++) {
            updateMotionAnimation(i, &animations[i]);
        }
        
        // Push to display
        oled_update();
    }
}
```

### Smart Update Logic

```cpp
void updateDisplayMode(DisplayState_t* state) {
    // Check for errors - highest priority
    if (ErrorManager::getInstance().hasErrors()) {
        state->mode = DISPLAY_MODE_ERROR;
        state->error_display_time = 50;  // 5 seconds
        return;
    }
    
    // Countdown error display
    if (state->error_display_time > 0) {
        state->error_display_time--;
        if (state->error_display_time == 0) {
            state->mode = DISPLAY_MODE_OVERVIEW;
        }
        return;
    }
    
    // Check for calibration
    if (isAnyAxisCalibrating()) {
        state->mode = DISPLAY_MODE_CALIBRATION;
        state->selected_axis = getCalibatingAxis();
        return;
    }
    
    // Check for user input (button press)
    if (button_pressed) {
        cycleDisplayMode(state);
    }
}
```

## OLED Driver Interface

### Low-Level Driver

```cpp
// SSD1306 128x64 I2C driver
typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    uint8_t buffer[128 * 64 / 8];  // 1KB frame buffer
    bool needs_update;
} OLED_Driver_t;

static OLED_Driver_t oled = {
    .i2c_port = I2C_NUM_0,
    .i2c_addr = 0x3C,  // Common SSD1306 address
};

// Initialize OLED
esp_err_t oled_init() {
    // Reset sequence
    static const uint8_t init_cmds[] = {
        0xAE,  // Display off
        0xD5, 0x80,  // Set clock
        0xA8, 0x3F,  // Multiplex ratio
        0xD3, 0x00,  // Display offset
        0x40,  // Start line
        0x8D, 0x14,  // Charge pump
        0x20, 0x00,  // Memory mode
        0xA1,  // Segment remap
        0xC8,  // COM scan direction
        0xDA, 0x12,  // COM pins
        0x81, 0xCF,  // Contrast
        0xD9, 0xF1,  // Precharge
        0xDB, 0x40,  // VCOM detect
        0xA4,  // Display RAM
        0xA6,  // Normal display
        0xAF   // Display on
    };
    
    // Send init sequence
    for (int i = 0; i < sizeof(init_cmds); i++) {
        oled_write_cmd(init_cmds[i]);
    }
    
    oled_clear();
    return ESP_OK;
}
```

### Text Rendering

```cpp
// 5x7 font in program memory
static const uint8_t FONT_5X7[] PROGMEM = {
    // ASCII 32-127
    0x00, 0x00, 0x00, 0x00, 0x00,  // Space
    0x00, 0x00, 0x5F, 0x00, 0x00,  // !
    // ... rest of font data
};

void oled_write_char(uint8_t x, uint8_t y, char c) {
    if (c < 32 || c > 127) return;
    
    const uint8_t* glyph = &FONT_5X7[(c - 32) * 5];
    
    for (int i = 0; i < 5; i++) {
        uint8_t column = pgm_read_byte(&glyph[i]);
        
        for (int j = 0; j < 8; j++) {
            if (column & (1 << j)) {
                oled_set_pixel(x + i, y + j, 1);
            }
        }
    }
}

void oled_write_string(uint8_t x, uint8_t y, const char* str) {
    while (*str) {
        oled_write_char(x, y, *str++);
        x += 6;  // 5 pixels + 1 space
    }
}
```

### Graphics Primitives

```cpp
// Progress bar
void oled_draw_progress(uint8_t x, uint8_t y, uint8_t width, 
                       uint8_t height, uint8_t percent) {
    // Draw border
    oled_draw_rect(x, y, width, height);
    
    // Fill based on percentage
    uint8_t fill_width = (width - 2) * percent / 100;
    oled_fill_rect(x + 1, y + 1, fill_width, height - 2);
}

// Status icons
void oled_draw_icon(uint8_t x, uint8_t y, IconType icon) {
    const uint8_t* icon_data;
    
    switch (icon) {
        case ICON_USB:
            icon_data = USB_ICON_8X8;
            break;
        case ICON_WARNING:
            icon_data = WARNING_ICON_8X8;
            break;
        case ICON_CHECK:
            icon_data = CHECK_ICON_8X8;
            break;
    }
    
    oled_draw_bitmap(x, y, 8, 8, icon_data);
}
```

## User Input Integration

### Button Handling for Display

```cpp
// Single button for display control
void handleDisplayButton() {
    static uint32_t press_start = 0;
    static bool long_press_handled = false;
    
    if (gpio_get_level(BUTTON_GPIO) == 0) {  // Pressed
        if (press_start == 0) {
            press_start = esp_timer_get_time() / 1000;
        }
        
        uint32_t press_duration = (esp_timer_get_time() / 1000) - press_start;
        
        if (press_duration > 2000 && !long_press_handled) {
            // Long press - toggle debug mode
            display_state.show_debug = !display_state.show_debug;
            long_press_handled = true;
        }
    } else {  // Released
        uint32_t press_duration = (esp_timer_get_time() / 1000) - press_start;
        
        if (press_duration < 500 && press_duration > 50) {
            // Short press - cycle display mode
            cycleDisplayMode(&display_state);
        }
        
        press_start = 0;
        long_press_handled = false;
    }
}
```

## Optimization Strategies

### Partial Updates

```cpp
// Track dirty regions for efficient updates
typedef struct {
    bool dirty[8];  // One per line
    uint8_t dirty_chars[8][21];  // Track individual character changes
} DirtyTracker_t;

void oled_update_line(uint8_t line, const char* text) {
    // Only update changed characters
    for (int i = 0; i < 21 && text[i]; i++) {
        if (dirty_tracker.dirty_chars[line][i] != text[i]) {
            oled_write_char(i * 6, line * 8, text[i]);
            dirty_tracker.dirty_chars[line][i] = text[i];
        }
    }
}
```

### Frame Buffer Management

```cpp
// Double buffering for flicker-free updates
uint8_t frame_buffer[2][1024];
uint8_t active_buffer = 0;

void oled_flip() {
    // Send inactive buffer to display
    oled_send_buffer(frame_buffer[active_buffer ^ 1]);
    
    // Swap buffers
    active_buffer ^= 1;
    
    // Copy to new buffer for incremental updates
    memcpy(frame_buffer[active_buffer], 
           frame_buffer[active_buffer ^ 1], 1024);
}
```

## Error Display Priorities

```cpp
typedef enum {
    ERROR_PRIORITY_INFO = 0,
    ERROR_PRIORITY_WARNING,
    ERROR_PRIORITY_ERROR,
    ERROR_PRIORITY_CRITICAL
} ErrorPriority_t;

typedef struct {
    char message[32];
    ErrorPriority_t priority;
    uint32_t timestamp;
} ErrorDisplay_t;

// Priority-based error display
void displayError(const ErrorDisplay_t* error) {
    switch (error->priority) {
        case ERROR_PRIORITY_CRITICAL:
            // Full screen, blinking
            oled_clear();
            oled_write_line_centered(2, "!!! CRITICAL !!!");
            oled_write_line_centered(3, error->message);
            oled_write_line_centered(5, "SYSTEM HALTED");
            break;
            
        case ERROR_PRIORITY_ERROR:
            // Top line, static
            oled_write_line(0, error->message);
            break;
            
        case ERROR_PRIORITY_WARNING:
            // Bottom line, temporary
            oled_write_line(7, error->message);
            break;
    }
}
```

## Testing Display Output

```cpp
TEST_CASE("Display all characters") {
    oled_clear();
    
    // Display full character set
    for (int i = 0; i < 96; i++) {
        int x = (i % 21) * 6;
        int y = (i / 21) * 8;
        oled_write_char(x, y, 32 + i);
    }
    
    oled_update();
    vTaskDelay(pdMS_TO_TICKS(2000));
}

TEST_CASE("Animation smoothness") {
    for (int pos = 0; pos < 100; pos++) {
        oled_clear();
        oled_draw_progress(10, 28, 108, 8, pos);
        oled_update();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```