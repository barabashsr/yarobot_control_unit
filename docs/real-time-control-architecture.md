# Universal Real-Time Control Architecture

## Overview

The yarobot_control_unit is designed as a universal real-time control device that provides deterministic motor control with flexible communication interfaces. The architecture prioritizes real-time performance while maintaining compatibility with higher-level frameworks like ROS2.

## Core Design Principles

### 1. Hardware Abstraction
- **Uniform Interface**: All axes expose identical position/velocity interfaces regardless of underlying hardware
- **Peripheral Transparency**: Higher layers don't need to know if using RMT, MCPWM, or LEDC
- **Capability Discovery**: Runtime detection of axis features (encoder feedback, position complete, etc.)

### 2. Real-Time Determinism
- **FreeRTOS Task Priorities**: Critical tasks run at highest priorities
  - Safety Monitor: Priority 24
  - Motion Control: Priority 25  
  - Communication: Priority 20/19
- **Interrupt-Driven I/O**: Hardware events handled immediately
- **Predictable Timing**: Pulse generation using hardware peripherals

### 3. Communication Architecture

#### Command-Response Pattern
```
Request:  MOVE X 100.0
Response: OK MOVING
Event:    EVENT POSITION_REACHED X POS:100.0 TIME:1234567890
```

#### Three Message Types
1. **Commands**: Synchronous requests with responses
2. **Events**: Asynchronous notifications  
3. **Streams**: Periodic data publishing

### 4. Layered Architecture

```
┌─────────────────────────────────────────┐
│         Application Layer               │
│    (ROS2, G-code, Custom Protocol)     │
├─────────────────────────────────────────┤
│      Communication Interface            │
│  (Serial Commands, Events, Streams)     │
├─────────────────────────────────────────┤
│         Axis Control Layer              │
│  (Uniform Position/Velocity Interface)  │
├─────────────────────────────────────────┤
│       Motor Control Layer               │
│   (Hardware-Specific Implementation)    │
├─────────────────────────────────────────┤
│      Hardware Abstraction Layer         │
│  (RMT, MCPWM, LEDC, GPIO, I2C)        │
├─────────────────────────────────────────┤
│         ESP32-S3 Hardware               │
└─────────────────────────────────────────┘
```

## Real-Time Features

### Motion Control
- **Hardware Pulse Generation**: Offloaded to peripherals (RMT, MCPWM)
- **DMA Transfers**: Minimal CPU involvement for high-frequency pulses
- **Hardware Position Counting**: PCNT peripherals track position independently
- **Trapezoidal Profiles**: Smooth acceleration/deceleration curves

### Safety Systems
- **Hardware E-Stop**: Direct GPIO interrupt with highest priority
- **Limit Switches**: Hardware interrupts for immediate response
- **I2C Watchdog**: Detect expander communication failures
- **Motion Interlocks**: Prevent unsafe simultaneous movements

### Position Feedback
- **Hardware Counters**: PCNT units for pulse counting
- **Z-Signal Processing**: Index pulse for absolute positioning
- **Software Estimation**: Calculated positions for LEDC axes
- **Calibration Recovery**: Automatic position sync on limit switches

## Universal Interface Design

### Position Control
All axes support identical position commands regardless of implementation:
```cpp
// Servo with encoder
motors[AXIS_X]->moveAbsolute(100.0);  // RMT + encoder feedback

// Servo without encoder  
motors[AXIS_Y]->moveAbsolute(50.0);   // MCPWM + pulse counting

// Stepper with PCNT
motors[AXIS_C]->moveAbsolute(30.0);   // MCPWM + hardware counter

// Stepper with LEDC
motors[AXIS_D]->moveAbsolute(100.0);  // LEDC + software position
```

### Velocity Control
Uniform velocity interface with automatic profile generation:
```cpp
// All axes accept velocity commands
motors[axis]->setVelocity(25.0);      // mm/s or deg/s
motors[axis]->setAcceleration(100.0); // mm/s² or deg/s²
```

### State Reporting
Consistent state interface for all axes:
```cpp
struct AxisState {
    float position;          // Current position
    float velocity;          // Current velocity  
    float target_position;   // Target position
    bool is_moving;          // Motion status
    bool is_enabled;         // Enable status
    uint32_t error_flags;    // Error conditions
};
```

## Testing Strategy

### Phase 1: Serial Interface Development
- Direct USB CDC commands
- Manual testing and validation
- Hardware debugging and tuning
- Safety system verification

### Phase 2: Mock ROS2 Interface
- Simulate ros2_control hardware interface
- Validate state publishing rates
- Test command/event mapping
- Performance profiling

### Phase 3: Full ROS2 Integration
- Implement ros2_control hardware interface
- Joint state publisher (50-100Hz)
- Command interfaces (position/velocity)
- Service interfaces (homing, calibration)

## Performance Specifications

### Timing Requirements
- **Pulse Generation**: Up to 100kHz per axis
- **Position Updates**: 1000Hz (1ms resolution)
- **Command Response**: <10ms typical
- **Event Latency**: <5ms from hardware trigger
- **Stream Publishing**: Configurable 10-1000Hz

### Resource Usage
- **CPU Load**: <50% during 7-axis motion
- **Memory**: 256KB heap, 64KB stack total
- **Flash**: <1MB including configuration
- **Power**: 3.3V @ 500mA typical

## Future Extensibility

### Planned Enhancements
1. **Ethernet Interface**: For deterministic networking
2. **Modbus RTU**: Industrial protocol support
3. **Encoder Feedback**: Closed-loop servo control
4. **S-Curve Profiles**: Smoother motion trajectories
5. **G-Code Layer**: CNC compatibility

### Modular Design Benefits
- Add new peripherals without changing upper layers
- Support different motor types with same interface
- Scale to more axes using I2C expansion
- Integrate new sensors via subscription system

## Implementation Best Practices

### Task Design
```cpp
// High-priority real-time task
void motionControlTask(void* param) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // Deterministic 1ms loop
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
        
        // Update all active motions
        for (auto& axis : activeAxes) {
            axis->updateMotionProfile();
        }
    }
}
```

### Event Publishing
```cpp
// Non-blocking event system
void publishEvent(const Event& event) {
    if (xQueueSend(eventQueue, &event, 0) != pdTRUE) {
        // Handle overflow - increment counter
        eventOverflowCount++;
    }
}
```

### Resource Protection
```cpp
// Mutex for shared resources
SemaphoreHandle_t i2cMutex = xSemaphoreCreateMutex();

void accessI2C() {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Perform I2C operation
        i2c_master_cmd_begin(...);
        xSemaphoreGive(i2cMutex);
    } else {
        // Handle timeout
        reportError(ERROR_I2C_BUSY);
    }
}
```

## Conclusion

This architecture provides a solid foundation for real-time motor control while maintaining flexibility for different communication protocols and motor types. The uniform interface design ensures that higher-level systems can control any axis without knowledge of the underlying hardware implementation.