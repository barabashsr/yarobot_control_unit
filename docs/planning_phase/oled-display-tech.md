# OLED Display Technical Interface

## Overview
128x64 SSD1306 OLED display for debugging and technical information. Not intended for daily operation - primarily for setup, troubleshooting, and status verification.

## Hardware Configuration
- **Display**: 128x64 monochrome OLED
- **Controller**: SSD1306
- **Interface**: Dedicated I2C bus (I2C1)
  - SDA: GPIO19
  - SCL: GPIO20  
  - Address: 0x3C
- **Update Rate**: 2 Hz (simple status display)
- **Font**: 6x8 for 7 readable lines

## Display Layout

All 7 lines dedicated to operational information:

### Normal Operation (movements priority)
```
railway:125.5→150.0 M
selector: 50.0→75.0 M
lift: 75.3      
jaw: 30.0       W12
─────────────────────


```
Shows only moving axes and those with flags.

### Event Display (2 second hold)
```
railway:125.5      
selector: 50.0     
─────────────────────
EVENT: railway DONE
EVENT: jaw OBJECT
EVENT: selector LIM+

```
Recent events shown for 2 seconds after occurrence.

### Error Priority Display
```
!!! E-STOP ACTIVE !!!
─────────────────────
AXES DISABLED



```

Or for operational errors:
```
railway:125.5    E2
selector: 50.0   
─────────────────────
ERROR: I2C 0x20 FAIL
ERROR: railway LIMIT


```

### Display States and Priority

1. **E-STOP** - Override entire display
2. **Errors** - Show axes + error messages below separator
3. **Events** - Show for 2 seconds after event occurs
4. **Movement** - Default view showing active axes

### State Transitions
- Movement → Event: When position reached or limit hit
- Event → Movement: After 2 second display time
- Any → Error: Immediately on error
- Error → Previous: When error cleared

### Line Usage
- **Lines 1-4**: Active axes (moving or with flags)
- **Line 5**: Separator (when needed)
- **Lines 6-7**: Events or errors

### Compact Flags
- `M` = Moving
- `E1` = At MIN limit
- `E2` = At MAX limit  
- `W##` = Width measurement (C axis)
- Numbers only, no units (assumed mm)

## Implementation Structure

```c
// Simplified display state
typedef struct {
    bool has_estop;
    bool has_errors;
    uint8_t error_code;
    char error_msg[22];
    system_mode_t system_mode;  // OPERATION, CONFIG, ERROR
} oled_state_t;

// Display buffer - 7 lines
typedef struct {
    char lines[7][22];  // 21 chars + null per line
} oled_buffer_t;

// Separate I2C bus configuration
i2c_config_t oled_i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = GPIO_NUM_19,
    .scl_io_num = GPIO_NUM_20,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000,  // 400kHz
};
```

## Update Strategy

### Simple Update Logic
```c
typedef enum {
    DISPLAY_MOVEMENT,
    DISPLAY_EVENT,
    DISPLAY_ERROR,
    DISPLAY_ESTOP
} display_state_t;

void oled_task(void* param) {
    // Initialize separate I2C bus for display
    i2c_param_config(I2C_NUM_1, &oled_i2c_config);
    i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
    ssd1306_init(I2C_NUM_1, 0x3C);
    
    display_state_t state = DISPLAY_MOVEMENT;
    TickType_t event_show_until = 0;
    
    while (1) {
        oled_buffer_t buffer = {0};
        TickType_t now = xTaskGetTickCount();
        
        // State management
        if (system_has_estop()) {
            state = DISPLAY_ESTOP;
        } else if (system_has_errors()) {
            state = DISPLAY_ERROR;
        } else if (event_queue_has_new() && state != DISPLAY_EVENT) {
            state = DISPLAY_EVENT;
            event_show_until = now + pdMS_TO_TICKS(2000);
        } else if (state == DISPLAY_EVENT && now > event_show_until) {
            state = DISPLAY_MOVEMENT;
        }
        
        // Render based on state
        switch (state) {
            case DISPLAY_ESTOP:
                strcpy(buffer.lines[0], "!!! E-STOP ACTIVE !!!");
                strcpy(buffer.lines[1], "─────────────────────");
                strcpy(buffer.lines[2], "AXES DISABLED");
                break;
                
            case DISPLAY_ERROR:
                render_axes(&buffer, 0, 3);  // Lines 0-3 for axes
                strcpy(buffer.lines[4], "─────────────────────");
                render_errors(&buffer, 5, 6); // Lines 5-6 for errors
                break;
                
            case DISPLAY_EVENT:
                render_axes(&buffer, 0, 3);  // Lines 0-3 for axes
                strcpy(buffer.lines[4], "─────────────────────");
                render_events(&buffer, 5, 6); // Lines 5-6 for events
                break;
                
            case DISPLAY_MOVEMENT:
            default:
                render_axes(&buffer, 0, 6);  // All 7 lines for axes
                break;
        }
        
        // Update display
        ssd1306_clear();
        for (int i = 0; i < 7; i++) {
            if (buffer.lines[i][0] != '\0') {
                ssd1306_draw_string(0, i * 8, buffer.lines[i]);
            }
        }
        ssd1306_refresh();
        
        vTaskDelay(pdMS_TO_TICKS(500)); // 2Hz update
    }
}

bool should_show_axis(int axis_num) {
    axis_t* axis = &axes[axis_num];
    return axis->is_moving || 
           axis->at_limit_min || 
           axis->at_limit_max ||
           (axis->id == AXIS_C && axis->object_width > 0) ||
           (axis->id == AXIS_E);  // Always show E axis state
}
```

## Display Formatting

### Axis Position Format
```c
// Format: "X:125.5→150.0 M"
void format_axis_line(char* buf, axis_t* axis) {
    if (axis->is_moving) {
        snprintf(buf, 22, "%s:%6.1f→%-6.1f %s",
                axis->alias,
                axis->current_pos,
                axis->target_pos,
                get_axis_flags(axis));
    } else {
        snprintf(buf, 22, "%s:%6.1f        %s",
                axis->alias,
                axis->current_pos,
                get_axis_flags(axis));
    }
}

// Flags: M=moving, E1/E2=limits, W##=width
const char* get_axis_flags(axis_t* axis) {
    static char flags[4];
    flags[0] = '\0';
    
    if (axis->is_moving) return "M";
    if (axis->at_limit_min) return "E1";
    if (axis->at_limit_max) return "E2";
    if (axis->id == AXIS_C && axis->object_width > 0) {
        snprintf(flags, 4, "W%d", (int)axis->object_width);
        return flags;
    }
    return "";
}
```

### Error Priority
Errors displayed by severity:
1. E-STOP active
2. I2C communication failures  
3. Axis limit violations
4. Position errors (Z-signal, InPos timeout)
5. Configuration errors

## Event Types Displayed
```
EVENT: railway DONE    # Position reached
EVENT: selector LIM+   # Limit switch hit
EVENT: jaw OBJECT     # Object detected
EVENT: lift CAL       # Z-signal calibration
EVENT: E-STOP CLR     # E-stop cleared
```

## Font Selection
- **Single font**: 6x8 monospace for everything
- 21 characters per line
- 7 lines, all usable for data
- High contrast for readability

## Configuration
```yaml
display:
  type: "SSD1306"
  i2c_bus: 1       # Separate from main I2C bus
  sda_pin: 19
  scl_pin: 20
  address: 0x3C
  enabled: true
  brightness: 128  # 0-255
  rotation: 0      # 0 or 180 degrees
```

## Separate I2C Bus Benefits
1. **Isolation**: Display failures don't affect critical I/O expanders
2. **Performance**: No bus contention with limit switches
3. **Debugging**: Can disconnect display without affecting operation
4. **Reliability**: Display I2C issues won't trigger E-stop

## Error Handling
- If display I2C fails, system continues normally
- No retries - just skip update cycle
- Display task at lowest priority

## Integration Notes
1. Display is completely optional
2. Task only started if display detected at boot
3. 2Hz update rate keeps bus traffic minimal
4. All formatting done with stack buffers