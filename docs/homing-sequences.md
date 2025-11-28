# Homing Sequences

## Overview
Homing establishes absolute position references by moving axes to known mechanical positions (limit switches). The YaRobot implements a safety-first homing sequence with ROS2 integration support.

## Homing State Machine

```c
typedef enum {
    HOME_STATE_IDLE,
    HOME_STATE_INIT,
    HOME_STATE_CLEAR_Z,        // Move Z up first for clearance
    HOME_STATE_FAST_SEARCH,    // Fast move to find switch
    HOME_STATE_BACKOFF,        // Back away from switch
    HOME_STATE_SLOW_SEARCH,    // Slow approach for accuracy
    HOME_STATE_SET_POSITION,   // Set zero reference
    HOME_STATE_FIND_INDEX,     // Find Z-signal for absolute reference
    HOME_STATE_MOVE_OFFSET,    // Move to home offset position
    HOME_STATE_COMPLETE,
    HOME_STATE_ERROR
} homing_state_t;

typedef struct {
    homing_state_t state;
    uint8_t axis_mask;         // Axes to home (bit mask)
    uint8_t current_axis;      // Currently homing axis
    float fast_velocity;       // Fast search speed
    float slow_velocity;       // Slow search speed  
    float backoff_distance;    // Distance to back off switch
    float home_offset;         // Final position after homing
    TickType_t timeout;        // Max time per phase
    TickType_t start_time;     // Phase start time
} homing_context_t;
```

## Homing Sequence Order

### Safety-First Sequence
1. **Z axes first** (A, Z) - Clear workspace vertically
2. **Y axes** (Y, B) - Retract grippers/tools
3. **X axis** - Final positioning axis
4. **C, D** - Tool axes (if configured)
5. **E** - Linear drive to retracted position

```c
// Default homing order (configurable in YAML)
const uint8_t homing_order[] = {
    AXIS_A,  // Picker Z first
    AXIS_Z,  // Selector Z
    AXIS_Y,  // Selector Y
    AXIS_B,  // Picker Y  
    AXIS_X,  // Railway X
    AXIS_C,  // Jaw (optional)
    AXIS_D,  // Retractor (optional)
    AXIS_E   // Linear drive
};
```

## Homing Implementation

### Main Homing Function
```c
esp_err_t home_all_axes(uint8_t axis_mask) {
    homing_context_t ctx = {
        .state = HOME_STATE_INIT,
        .axis_mask = axis_mask,
        .fast_velocity = 0,  // Set per axis
        .slow_velocity = 0,  // Set per axis
        .backoff_distance = 5.0,
        .home_offset = 0.0,
        .timeout = pdMS_TO_TICKS(30000)  // 30s per axis
    };
    
    // Disable all axes first
    motion_disable_all();
    
    // Execute homing sequence
    for (int i = 0; i < NUM_AXES; i++) {
        if (axis_mask & (1 << homing_order[i])) {
            esp_err_t err = home_single_axis(homing_order[i], &ctx);
            if (err != ESP_OK) {
                // Stop on first error
                motion_emergency_stop();
                return err;
            }
        }
    }
    
    return ESP_OK;
}
```

### Single Axis Homing State Machine
```c
esp_err_t home_single_axis(uint8_t axis_num, homing_context_t* ctx) {
    axis_t* axis = &axes[axis_num];
    
    // Load axis-specific parameters
    ctx->fast_velocity = axis->config.homing.fast_velocity;
    ctx->slow_velocity = axis->config.homing.slow_velocity;
    ctx->backoff_distance = axis->config.homing.backoff_distance;
    ctx->home_offset = axis->config.homing.home_offset;
    
    // State machine
    ctx->state = HOME_STATE_FAST_SEARCH;
    ctx->start_time = xTaskGetTickCount();
    
    while (ctx->state != HOME_STATE_COMPLETE && 
           ctx->state != HOME_STATE_ERROR) {
        
        // Check timeout
        if ((xTaskGetTickCount() - ctx->start_time) > ctx->timeout) {
            ESP_LOGE(TAG, "Homing timeout on %s", axis->alias);
            ctx->state = HOME_STATE_ERROR;
            break;
        }
        
        switch (ctx->state) {
            case HOME_STATE_FAST_SEARCH:
                // Move toward limit at fast speed
                if (axis->config.homing.direction == HOME_NEGATIVE) {
                    axis_jog(axis_num, -ctx->fast_velocity);
                } else {
                    axis_jog(axis_num, ctx->fast_velocity);
                }
                
                // Wait for limit switch
                if (wait_for_limit(axis_num, ctx->timeout)) {
                    axis_stop(axis_num);
                    ctx->state = HOME_STATE_BACKOFF;
                    ctx->start_time = xTaskGetTickCount();
                }
                break;
                
            case HOME_STATE_BACKOFF:
                // Back away from switch
                float backoff_target = axis->current_position + 
                    (axis->config.homing.direction == HOME_NEGATIVE ? 
                     ctx->backoff_distance : -ctx->backoff_distance);
                
                axis_move_to(axis_num, backoff_target, ctx->slow_velocity);
                
                if (wait_for_motion_complete(axis_num, ctx->timeout)) {
                    ctx->state = HOME_STATE_SLOW_SEARCH;
                    ctx->start_time = xTaskGetTickCount();
                }
                break;
                
            case HOME_STATE_SLOW_SEARCH:
                // Slow approach to switch
                if (axis->config.homing.direction == HOME_NEGATIVE) {
                    axis_jog(axis_num, -ctx->slow_velocity);
                } else {
                    axis_jog(axis_num, ctx->slow_velocity);
                }
                
                // Wait for limit with higher precision
                if (wait_for_limit_precise(axis_num, ctx->timeout)) {
                    axis_stop(axis_num);
                    ctx->state = HOME_STATE_SET_POSITION;
                }
                break;
                
            case HOME_STATE_SET_POSITION:
                // Set this as zero position
                axis_set_position(axis_num, 0.0);
                
                // If axis has Z-signal, wait for index pulse
                if (axis->config.feedback.z_signal.enabled) {
                    ctx->state = HOME_STATE_FIND_INDEX;
                } else {
                    // No Z-signal, proceed to offset
                    if (fabs(ctx->home_offset) > 0.1) {
                        ctx->state = HOME_STATE_MOVE_OFFSET;
                    } else {
                        ctx->state = HOME_STATE_COMPLETE;
                    }
                }
                break;
                
            case HOME_STATE_FIND_INDEX:
                // Move slowly to find Z-signal index pulse
                axis_jog(axis_num, ctx->slow_velocity);
                
                // Wait for Z-signal with one full rotation max
                if (wait_for_z_signal(axis_num, ctx->timeout)) {
                    axis_stop(axis_num);
                    // Z-signal provides absolute reference
                    axis_set_position(axis_num, 0.0);
                    
                    if (fabs(ctx->home_offset) > 0.1) {
                        ctx->state = HOME_STATE_MOVE_OFFSET;
                    } else {
                        ctx->state = HOME_STATE_COMPLETE;
                    }
                } else if (axis_z_signal_timeout(axis_num)) {
                    // No Z-signal found in expected distance
                    ESP_LOGW(TAG, "No Z-signal found on %s, using switch only", 
                             axis->alias);
                    // Continue without Z-signal
                    if (fabs(ctx->home_offset) > 0.1) {
                        ctx->state = HOME_STATE_MOVE_OFFSET;
                    } else {
                        ctx->state = HOME_STATE_COMPLETE;
                    }
                }
                break;
                
            case HOME_STATE_MOVE_OFFSET:
                // Move to home offset position
                axis_move_to(axis_num, ctx->home_offset, ctx->slow_velocity);
                
                if (wait_for_motion_complete(axis_num, ctx->timeout)) {
                    ctx->state = HOME_STATE_COMPLETE;
                }
                break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Generate event
    event_t event = {
        .type = (ctx->state == HOME_STATE_COMPLETE) ? 
                EVENT_HOMING_COMPLETE : EVENT_HOMING_FAILED,
        .source_id = axis_num,
        .timestamp = xTaskGetTickCount()
    };
    strncpy(event.source_name, axis->alias, 15);
    xQueueSend(g_event_queue, &event, 0);
    
    return (ctx->state == HOME_STATE_COMPLETE) ? ESP_OK : ESP_FAIL;
}
```

## Z-Signal Handling During Homing

```c
bool wait_for_z_signal(uint8_t axis_num, TickType_t timeout) {
    axis_t* axis = &axes[axis_num];
    
    // Only for axes with Z-signal enabled
    if (!axis->config.feedback.z_signal.enabled) {
        return false;
    }
    
    // Clear any previous Z-signal flag
    axis->z_signal_detected = false;
    
    // Record starting position
    float start_position = axis->current_position;
    float max_travel = axis->config.feedback.counts_per_revolution / 
                      axis->config.steps_per_unit;
    
    TickType_t start = xTaskGetTickCount();
    
    while ((xTaskGetTickCount() - start) < timeout) {
        // Check if Z-signal detected (set by ISR)
        if (axis->z_signal_detected) {
            return true;
        }
        
        // Check if we've moved more than one revolution
        if (fabs(axis->current_position - start_position) > max_travel * 1.2) {
            // Traveled too far without finding index
            return false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    return false;
}

bool axis_z_signal_timeout(uint8_t axis_num) {
    axis_t* axis = &axes[axis_num];
    // Check if we moved more than expected without finding index
    return axis->z_signal_search_timeout;
}
```

## Limit Switch Handling During Homing

```c
bool wait_for_limit(uint8_t axis_num, TickType_t timeout) {
    axis_t* axis = &axes[axis_num];
    TickType_t start = xTaskGetTickCount();
    
    while ((xTaskGetTickCount() - start) < timeout) {
        // Check appropriate limit based on homing direction
        bool limit_active = (axis->config.homing.direction == HOME_NEGATIVE) ?
                           axis->at_limit_min : axis->at_limit_max;
        
        if (limit_active) {
            // Debounce
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // Verify still active
            limit_active = (axis->config.homing.direction == HOME_NEGATIVE) ?
                          axis->at_limit_min : axis->at_limit_max;
            
            if (limit_active) {
                return true;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    return false;
}
```

## Homing Modes

### Mode 1: Limit Switch + Z-Signal (Most Accurate)
- Find limit switch using two-speed approach
- Move off switch to find Z-signal index pulse
- Set absolute zero at index pulse
- Move to home offset position

### Mode 2: Limit Switch Only (Standard)
- Find limit switch using two-speed approach  
- Set position to zero at switch activation
- Move to home offset position
- Less accurate but works with any servo

### Mode 3: No Homing (Open Loop)
- For axes without position feedback
- Assumes position from last known state
- Requires manual positioning on power-up

## YAML Configuration

```yaml
axes:
  X:
    homing:
      enabled: true
      required: true  # Must home before operation
      direction: "negative"  # Home toward min limit
      fast_velocity: 50.0    # mm/s
      slow_velocity: 5.0     # mm/s
      backoff_distance: 5.0  # mm
      home_offset: 10.0      # Final position after homing
      timeout_ms: 30000      # Per-phase timeout
      
  C:
    homing:
      enabled: false  # Jaw doesn't need homing
      
  E:
    homing:
      enabled: true
      direction: "negative"  # Retract to home
      fast_velocity: 100.0   # Full speed for discrete axis
      slow_velocity: 100.0   # Same speed (on/off control)
```

## ROS2 Integration

### Hardware Interface Homing
```c
// Called during on_configure() phase
CallbackReturn YaRobotHardware::on_configure(
    const rclcpp_lifecycle::State & previous_state) 
{
    RCLCPP_INFO(logger_, "Configuring hardware interface");
    
    // Initialize communication
    if (!init_serial_port()) {
        return CallbackReturn::ERROR;
    }
    
    // Check if homing required
    bool homing_required = false;
    for (auto& axis : axes_) {
        if (axis.config.homing.required) {
            homing_required = true;
            break;
        }
    }
    
    if (homing_required) {
        RCLCPP_INFO(logger_, "Starting homing sequence");
        
        // Send homing command to controller
        send_command("HOME ALL");
        
        // Wait for completion with timeout
        auto start = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(120);
        
        while (!is_homing_complete()) {
            if ((std::chrono::steady_clock::now() - start) > timeout) {
                RCLCPP_ERROR(logger_, "Homing timeout");
                return CallbackReturn::ERROR;
            }
            
            // Process events from controller
            process_controller_events();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        RCLCPP_INFO(logger_, "Homing complete");
    }
    
    return CallbackReturn::SUCCESS;
}

// State interface for homing status
void YaRobotHardware::export_state_interfaces() {
    for (auto& axis : axes_) {
        // Standard interfaces
        state_interfaces.emplace_back(
            axis.name, "position", &axis.position);
        state_interfaces.emplace_back(
            axis.name, "velocity", &axis.velocity);
            
        // Homing status interface
        state_interfaces.emplace_back(
            axis.name, "is_homed", &axis.is_homed);
    }
}
```

### Homing Commands

```bash
# Home all configured axes
HOME ALL
> OK HOME STARTED
> EVENT HOMING_COMPLETE lift
> EVENT HOMING_COMPLETE picker_z
> EVENT HOMING_COMPLETE selector
> EVENT HOMING_COMPLETE railway
> OK HOME COMPLETE

# Home specific axis
HOME railway
> OK HOME railway STARTED
> EVENT HOMING_COMPLETE railway

# Query homing status
HOME STATUS
> HOMED: railway selector lift picker_z
> NOT_HOMED: jaw retractor
> DISABLED: linear

# Abort homing
HOME ABORT
> OK HOME ABORTED
```

## Safety Considerations

1. **Motion Interlock**: No motion commands accepted until homing complete (if required)
2. **Soft Limits**: Activated only after successful homing
3. **E-Stop During Homing**: Immediately aborts sequence
4. **Power Loss**: Requires re-homing on power recovery
5. **Switch Failure**: Timeout prevents infinite motion

## Error Recovery

```c
typedef enum {
    HOME_ERROR_NONE = 0,
    HOME_ERROR_TIMEOUT,
    HOME_ERROR_NO_SWITCH,
    HOME_ERROR_STUCK_SWITCH,
    HOME_ERROR_MOTION_FAULT,
    HOME_ERROR_ABORTED
} home_error_t;

void handle_homing_error(uint8_t axis_num, home_error_t error) {
    axis_t* axis = &axes[axis_num];
    
    // Stop all motion
    axis_stop(axis_num);
    
    // Log error
    ESP_LOGE(TAG, "Homing error on %s: %s", 
             axis->alias, get_home_error_string(error));
    
    // Mark axis as not homed
    axis->is_homed = false;
    
    // Disable axis if safety critical
    if (axis->config.homing.required) {
        axis_disable(axis_num);
    }
}
```

## Testing and Validation

1. **Simulate Stuck Switch**: Verify timeout handling
2. **E-Stop During Homing**: Confirm immediate abort
3. **Power Cycle**: Verify homing required after power loss
4. **ROS2 Integration**: Verify on_configure() waits for completion
5. **Multi-Axis**: Verify correct sequence order