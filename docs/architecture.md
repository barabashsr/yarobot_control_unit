# Architecture

## Executive Summary

The YaRobot Control Unit architecture is designed around a real-time FreeRTOS-based firmware running on ESP32-S3, providing a unified command interface for controlling 8 heterogeneous motor axes. The architecture prioritizes deterministic timing, safety-first design, and clean abstraction layers that hide motor-type complexity from the host interface.

Key architectural principles:
- **Dual-core separation**: Core 0 handles communication and safety, Core 1 handles motion control
- **Hardware-first timing**: RMT+DMA, MCPWM, and LEDC peripherals for jitter-free pulse generation
- **Fail-safe design**: Brakes engage on power loss, E-stop bypasses software entirely
- **Unified abstraction**: All motor types expose identical command interface

## Project Initialization

**First implementation step:**

```bash
# Create ESP-IDF project
idf.py create-project yarobot_control_unit
cd yarobot_control_unit

# Set target to ESP32-S3
idf.py set-target esp32s3

# Add ESP-IDF component dependencies
idf.py add-dependency "espressif/mcp23017^0.1.1"

# Configure for N16R8 variant (16MB Flash, 8MB PSRAM)
idf.py menuconfig
```

**Key sdkconfig settings to configure:**
- `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y`
- `CONFIG_SPIRAM_MODE_OCT=y` (Octal PSRAM)
- `CONFIG_FREERTOS_HZ=1000` (1ms tick for timing precision)
- `CONFIG_ESP_CONSOLE_USB_CDC=y` (USB CDC for serial)
- `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`

**ESP-IDF provides these decisions:**
- FreeRTOS (IDF SMP variant for dual-core)
- CMake-based build system
- Core peripheral drivers (RMT, MCPWM, LEDC, PCNT, SPI, I2C)
- USB stack with CDC support

## Decision Summary

| Category | Decision | Version | Verified | Affects FRs | Rationale |
| -------- | -------- | ------- | -------- | ----------- | --------- |
| Platform | ESP32-S3-DevKitC-1 N16R8 | - | 2025-11-29 | All | 16MB Flash, 8MB PSRAM, dual-core, native USB |
| Framework | ESP-IDF | v5.4 (LTS) | 2025-11-29 | All | FreeRTOS SMP, mature peripheral drivers |
| Component Structure | Layered Architecture | - | - | All | Clear HAL/Driver/Control separation, testable |
| Motor Abstraction | Hybrid (Inheritance + Composition) | - | - | FR1-10 | Injectable pulse generators, mockable for tests |
| Shift Registers | 5x TPIC6B595N (40-bit chain) | - | 2025-11-29 | FR3-5, FR14 | Native 24V, fail-safe brakes, ALARM_CLR, GP outputs |
| I2C Expanders | 2x MCP23017 (on I2C0) | espressif/mcp23017 v0.1.1 | 2025-11-29 | FR11, FR35-37 | Limit switches, ALARM_INPUT, InPos (inputs only) |
| OLED Display | SSD1306 128x64 (on I2C1) | esp_lcd (ESP-IDF native) | 2025-11-29 | FR47-47c | Dedicated bus isolates from critical I/O |

> **Version Verification Note:** All versioned components verified current as of dates shown. ESP-IDF v5.4 is a Long-Term Support release. Re-verify versions before starting implementation if more than 30 days have passed.

## Behavioral Decisions Summary

The following behavioral decisions define how the system responds to operator commands and state transitions. These were resolved during architecture review to eliminate ambiguity.

| # | Category | Decision | Rationale |
|---|----------|----------|-----------|
| 1 | Startup | Motors **disabled by default** at power-on | Safety: prevents unexpected motion, requires explicit EN command |
| 2 | Startup | Send **EVENT BOOT** notification | Host knows when ready: `EVENT BOOT V1.0.0 AXES:8 STATE:IDLE` |
| 3 | Config | **Block all motion** during CONFIG mode | CFGSTART fails if any axis moving; prevents config/motion race conditions |
| 4 | Config | **Full rollback** on YAML parse failure | Parse & validate entire config first, then apply atomically; no partial state |
| 5 | Config | **YAML only** for config changes | No SETCFG command; all changes via CFGSTART/CFGDATA/CFGEND flow |
| 6 | Motion | **Blend to new target** on mid-motion MOVE | Recalculate profile on-the-fly for smooth transition to new target |
| 7 | Motion | **Multi-axis sync is post-MVP** | Single-axis commands for MVP; coordinated motion as future enhancement |
| 8 | Motion | **Command queue is post-MVP** | Single-command model for MVP; master waits for EVT MOTION_COMPLETE |
| 9 | Velocity | **Smooth acceleration** on VEL change | Ramp from current velocity to new target using configured acceleration |
| 10 | Velocity | **Blend to target** on VEL→MOVE | Smooth trajectory from current velocity/position to new target |
| 11 | Velocity | **Parameter overrides config** for acceleration | VEL accel param overrides config.max_acceleration for this move only |
| 12 | Stop | **Drain DMA buffer** on STOP | Complete current buffer for position accuracy; decel profile honored |
| 13 | Stop | **Blend allowed** after STOP | New MOVE can blend from decelerating state; no wait required |
| 14 | Emergency | **Position preserved** after STOP EMERGENCY | pulse_count valid; may have small error from aborted decel |
| 15 | E-Stop | **Position unknown** after hardware E-STOP | All axes marked UNHOMED; require full re-home after E-stop release |
| 16 | E-Stop | **STOP EMERGENCY = E-STOP behavior** | Both enter STATE_ESTOP_ACTIVE; both require RST command to clear |
| 17 | Brakes | **Current SPI timing acceptable** | ~50µs software brake path OK; trust motor holding torque during gap |
| 18 | Feed | **Feed override is post-MVP** | Master calculates scaled velocities; no FEED command for MVP |
| 19 | Profile | **Trapezoidal for MVP**, S-curve later | Simple accel-cruise-decel; S-curve as future enhancement |
| 20 | Config | **Error on YAML overflow** | Return ERROR E030 if config exceeds 8KB buffer; require shorter config |

## Project Structure

```
yarobot_control_unit/
├── CMakeLists.txt                 # Top-level project CMake
├── sdkconfig                      # ESP-IDF configuration
├── sdkconfig.defaults             # Default config values
├── partitions.csv                 # Flash partition table
│
├── main/                          # Application entry point
│   ├── CMakeLists.txt
│   ├── main.cpp                   # app_main() entry
│   └── Kconfig.projbuild          # Project-specific menu options
│
├── components/
│   ├── hal/                       # Hardware Abstraction Layer
│   │   ├── gpio_hal/
│   │   ├── i2c_hal/
│   │   └── spi_hal/
│   │
│   ├── drivers/                   # Device drivers
│   │   ├── mcp23017/              # I2C GPIO expander (espressif/mcp23017 component)
│   │   ├── tpic6b595/             # Shift register chain (custom SPI driver)
│   │   └── oled/                  # OLED display (esp_lcd_panel_ssd1306 from ESP-IDF)
│   │
│   ├── pulse_gen/                 # Pulse generation interfaces
│   │   ├── include/
│   │   │   └── pulse_generator.h  # IPulseGenerator interface
│   │   ├── rmt_pulse_gen.cpp      # RMT+DMA implementation
│   │   ├── mcpwm_pulse_gen.cpp    # MCPWM implementation
│   │   └── ledc_pulse_gen.cpp     # LEDC implementation
│   │
│   ├── position/                  # Position tracking
│   │   ├── include/
│   │   │   └── position_tracker.h # IPositionTracker interface
│   │   ├── pcnt_tracker.cpp       # Hardware counter
│   │   └── time_tracker.cpp       # Time-based calculation
│   │
│   ├── motor/                     # Motor abstraction
│   │   ├── include/
│   │   │   ├── motor_base.h
│   │   │   ├── servo_motor.h
│   │   │   ├── stepper_motor.h
│   │   │   └── discrete_axis.h
│   │   └── src/
│   │
│   ├── control/                   # Control logic
│   │   ├── motion_controller/     # Coordinates all axes
│   │   ├── safety_monitor/        # E-stop, limits, faults
│   │   └── command_executor/      # Command processing
│   │
│   ├── interface/                 # External interfaces
│   │   ├── usb_cdc/               # USB serial interface
│   │   └── command_parser/        # Text command parsing
│   │
│   ├── config/                    # Configuration management
│   │   ├── nvs_manager/           # NVS read/write
│   │   └── yaml_parser/           # YAML config handling
│   │
│   └── events/                    # Event system
│       └── event_manager/         # Publish/subscribe events
│
├── test/                          # Unit tests (host-based)
│   └── components/
│
└── docs/                          # Documentation
    ├── architecture.md
    └── planning_phase/
```

## FR Category to Architecture Mapping

| FR Category | Architecture Component | Notes |
|-------------|----------------------|-------|
| Motor Control (FR1-10) | `components/motor/`, `components/pulse_gen/` | 8 axes via IMotor interface |
| Safety (FR11-18) | `components/control/safety_monitor/` | Core 0, highest priority |
| Communication (FR19-26) | `components/interface/usb_cdc/`, `command_parser/` | USB CDC, text protocol |
| Configuration (FR27-34) | `components/config/nvs_manager/`, `yaml_parser/` | NVS + serial YAML transfer |
| I/O (FR35-42) | `components/drivers/mcp23017/` | 3x I2C expanders |
| Monitoring (FR43-50) | `components/control/`, `components/drivers/oled/` | OLED + events |
| Modes (FR51-55) | `components/control/command_executor/` | State machine |
| Errors (FR56-61) | `components/control/safety_monitor/`, `events/` | Centralized error manager |

## Technology Stack Details

### Core Technologies

| Component | Technology | Version | Purpose |
|-----------|------------|---------|---------|
| MCU | ESP32-S3-DevKitC-1 N16R8 | - | 16MB Flash, 8MB PSRAM, 240MHz dual-core |
| Framework | ESP-IDF | 5.4 | FreeRTOS SMP, drivers, build system |
| RTOS | FreeRTOS | IDF bundled | Task management, queues, mutexes |
| USB | TinyUSB (IDF component) | - | USB CDC serial interface |
| NVS | ESP-IDF NVS | - | Persistent configuration storage |
| Build | CMake + idf.py | - | Component-based build |

### Peripheral Usage

| Peripheral | Assignment | Notes |
|------------|------------|-------|
| RMT | 4 channels (X,Z,A,B) | DMA-driven pulse generation |
| MCPWM | 2 timers (Y,C) | PWM + internal PCNT routing |
| LEDC | 1 channel (D) | Discrete stepper pulses |
| PCNT | 2 units (Y,C) | Hardware pulse counting |
| SPI2 | Shift registers | 40-bit DIR/EN/Brake/ALARM_CLR/GP chain |
| I2C0 | MCP23017 x2 | Limit switches, ALARM_INPUT, InPos (inputs only) |
| I2C1 | SSD1306 OLED | Status display (isolated bus) |

### Integration Points

**Host Interface (USB CDC):**
- Text commands at 115200+ baud
- Async event notifications
- YAML configuration transfer

**External Hardware:**
- STEP/DIR/EN to servo/stepper drivers (24V logic via shift registers)
- 14 limit switches via I2C expanders (MCP23017 #0)
- 5 brake relays via shift registers
- ALARM_INPUT via I2C expander (MCP23017 #1 Port A)
- ALARM_CLEAR via shift registers (4th bit per axis)
- InPos feedback via I2C expander (MCP23017 #1 Port B)
- 8 general purpose outputs via shift registers (SR4)
- E-stop via direct GPIO

**Future Integration:**
- ROS2 hardware_interface (post-MVP)
- Python/C++ host libraries

### Post-MVP Features

The following features are explicitly deferred to post-MVP development:

| Feature | Description | Workaround for MVP |
|---------|-------------|-------------------|
| **Multi-axis sync** | Coordinated motion start (`MOVE X 0.1 Y 0.2 SYNC`) | Master times commands manually; ~1ms jitter |
| **Command queue** | Buffer multiple moves per axis | Master waits for EVT MOTION_COMPLETE between commands |
| **Feed override** | Runtime velocity scaling (`FEED X 120`) | Master calculates scaled velocities, sends new VEL command |
| **S-curve profiles** | Jerk-limited smooth motion | Trapezoidal profiles used; acceptable for most applications |
| **ROS2 integration** | hardware_interface plugin | Use USB CDC text protocol directly |
| **Backlash compensation** | Automatic backlash handling | Manual compensation in host application |

## Implementation Patterns

These patterns ensure consistent implementation across all components:

### Streaming Double-Buffer Pulse Generation

> **⚠️ ARCHITECTURE CONSTRAINT - APPLIES TO ALL MOTION**
>
> **All pulse generation uses streaming double-buffer architecture.** This is non-negotiable.
>
> - Short moves, long moves, and continuous jogging use the **same streaming infrastructure**
> - No special-case code paths for "small" vs "large" motions
> - Buffer swap happens via DMA callback - zero CPU involvement during transmission
> - Motion length is **unlimited** - profile generator streams pulses on-demand

**Why Double-Buffer Everywhere:**

1. **Consistency** - One code path to test, debug, and maintain
2. **Unlimited motion** - No buffer size limits on travel distance
3. **Smooth continuous motion** - VEL (jog) command works seamlessly
4. **Predictable timing** - DMA handles transmission, CPU handles profile math
5. **Clean abort** - Stop command just stops refilling buffers

#### Buffer Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                     RMT DMA Double-Buffer                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌──────────────┐         ┌──────────────┐                        │
│   │  Buffer A    │         │  Buffer B    │                        │
│   │  (512 sym)   │         │  (512 sym)   │                        │
│   └──────┬───────┘         └──────┬───────┘                        │
│          │                        │                                 │
│          ▼                        ▼                                 │
│   ┌──────────────────────────────────────────┐                     │
│   │            RMT DMA Engine                 │                     │
│   │  (transmits while CPU fills other buf)   │                     │
│   └──────────────────┬───────────────────────┘                     │
│                      │                                              │
│                      ▼                                              │
│              ┌──────────────┐                                       │
│              │  GPIO Output │ ────► STEP pulse to motor driver     │
│              └──────────────┘                                       │
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │                    Profile Generator                         │  │
│   │  ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐     │  │
│   │  │  Accel  │ → │ Const V │ → │  Decel  │ → │  Done   │     │  │
│   │  │  Phase  │   │  Phase  │   │  Phase  │   │  State  │     │  │
│   │  └─────────┘   └─────────┘   └─────────┘   └─────────┘     │  │
│   │                                                              │  │
│   │  Generates next batch of RMT symbols on TX-done callback    │  │
│   └─────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

#### Streaming State Machine

```
                    ┌─────────────┐
                    │    IDLE     │
                    └──────┬──────┘
                           │ start_motion()
                           ▼
                    ┌─────────────┐
                    │   PRIMING   │  ← Fill both buffers before TX start
                    └──────┬──────┘
                           │ buffers ready
                           ▼
                    ┌─────────────┐
            ┌──────►│  STREAMING  │◄─────┐
            │       └──────┬──────┘      │
            │              │             │
    TX-done callback       │             │ more pulses needed
    (swap buffers)         │             │
            │              ▼             │
            │       ┌─────────────┐      │
            └───────┤  REFILLING  ├──────┘
                    └──────┬──────┘
                           │ profile complete OR stop()
                           ▼
                    ┌─────────────┐
                    │  DRAINING   │  ← Transmit remaining buffer, no refill
                    └──────┬──────┘
                           │ TX complete
                           ▼
                    ┌─────────────┐
                    │    IDLE     │
                    └─────────────┘
```

#### IPulseGenerator Interface (Streaming)

```cpp
// Streaming pulse generator interface - ALL implementations use this
class IPulseGenerator {
public:
    virtual ~IPulseGenerator() = default;

    // === Motion Control ===

    // Start a position move (finite pulses with trapezoidal profile)
    virtual esp_err_t startMove(
        int32_t pulses,           // Target pulse count (signed for direction)
        float max_velocity,       // pulses/sec
        float acceleration        // pulses/sec²
    ) = 0;

    // Start continuous motion (velocity mode - infinite until stop)
    virtual esp_err_t startVelocity(
        float velocity,           // pulses/sec (signed for direction)
        float acceleration        // ramp rate to target velocity
    ) = 0;

    // Controlled stop (decelerate to zero)
    // NOTE: DMA buffer is drained before stop completes (position accuracy preserved)
    // New MOVE commands can blend from decelerating state (no wait required)
    virtual esp_err_t stop(float deceleration) = 0;

    // Emergency stop (immediate halt, no decel ramp)
    // NOTE: Position preserved but may have small error from aborted buffer
    virtual void stopImmediate() = 0;

    // === Status ===
    virtual bool isRunning() const = 0;
    virtual int64_t getPulseCount() const = 0;      // Pulses generated so far
    virtual float getCurrentVelocity() const = 0;   // Instantaneous velocity

    // === Callbacks ===
    using MotionCompleteCallback = std::function<void(int64_t total_pulses)>;
    virtual void setCompletionCallback(MotionCompleteCallback cb) = 0;
};
```

#### RMT Implementation Details

```cpp
// RMT-specific streaming implementation
class RmtPulseGenerator : public IPulseGenerator {
private:
    // Double buffer configuration
    static constexpr size_t BUFFER_SYMBOLS = 512;  // Per buffer
    rmt_symbol_word_t buffer_a_[BUFFER_SYMBOLS];
    rmt_symbol_word_t buffer_b_[BUFFER_SYMBOLS];

    // Buffer state
    enum class BufferState { IDLE, TRANSMITTING, READY };
    struct {
        rmt_symbol_word_t* data;
        size_t count;
        BufferState state;
    } buffers_[2];
    uint8_t active_buffer_;      // Currently being transmitted by DMA
    uint8_t fill_buffer_;        // Currently being filled by CPU

    // Profile state
    enum class ProfilePhase { ACCEL, CONST, DECEL, DONE };
    struct {
        ProfilePhase phase;
        float current_velocity;
        float target_velocity;
        float acceleration;
        int64_t pulses_remaining;
        int64_t pulses_generated;
        bool is_continuous;      // true for VEL command
    } profile_;

    // RMT handles
    rmt_channel_handle_t channel_;
    rmt_encoder_handle_t encoder_;

    // Hardware constants
    static constexpr uint32_t RMT_RESOLUTION_HZ = 80000000;  // 80MHz
    static constexpr uint16_t MIN_SYMBOL_TICKS = 40;         // 500ns minimum
    static constexpr uint16_t MAX_SYMBOL_TICKS = 0x7FFF;     // 15-bit max

public:
    // Called by RMT TX-done ISR
    static bool IRAM_ATTR onTxDone(
        rmt_channel_handle_t channel,
        const rmt_tx_done_event_data_t* event,
        void* user_ctx
    ) {
        auto* self = static_cast<RmtPulseGenerator*>(user_ctx);
        BaseType_t woken = pdFALSE;

        // Signal motion task to refill the just-completed buffer
        xTaskNotifyFromISR(self->motion_task_, NOTIFY_BUFFER_DONE,
                          eSetBits, &woken);

        // Swap to next buffer if ready
        if (self->buffers_[self->fill_buffer_].state == BufferState::READY) {
            self->active_buffer_ = self->fill_buffer_;
            self->fill_buffer_ = 1 - self->fill_buffer_;
            // Continue transmission with next buffer
            rmt_transmit(channel, self->encoder_,
                        self->buffers_[self->active_buffer_].data,
                        self->buffers_[self->active_buffer_].count * sizeof(rmt_symbol_word_t),
                        &self->tx_config_);
        }

        return woken == pdTRUE;
    }

private:
    // Fill one buffer with next batch of profile pulses
    size_t fillBuffer(rmt_symbol_word_t* buffer, size_t max_symbols) {
        size_t count = 0;

        while (count < max_symbols && profile_.phase != ProfilePhase::DONE) {
            // Calculate period for current velocity
            float period_sec = 1.0f / fabsf(profile_.current_velocity);
            uint32_t half_period_ticks = (uint32_t)(period_sec * RMT_RESOLUTION_HZ / 2);

            // Clamp to valid range
            half_period_ticks = std::clamp(half_period_ticks,
                                           (uint32_t)MIN_SYMBOL_TICKS,
                                           (uint32_t)MAX_SYMBOL_TICKS);

            // Create RMT symbol (one STEP pulse)
            buffer[count].duration0 = half_period_ticks;
            buffer[count].level0 = 1;
            buffer[count].duration1 = half_period_ticks;
            buffer[count].level1 = 0;
            count++;

            // Update profile state
            profile_.pulses_generated++;
            if (!profile_.is_continuous) {
                profile_.pulses_remaining--;
            }

            // Update velocity based on current phase
            updateProfilePhase();
        }

        return count;
    }

    void updateProfilePhase() {
        switch (profile_.phase) {
            case ProfilePhase::ACCEL:
                profile_.current_velocity += profile_.acceleration / profile_.current_velocity;
                if (profile_.current_velocity >= profile_.target_velocity) {
                    profile_.current_velocity = profile_.target_velocity;
                    profile_.phase = ProfilePhase::CONST;
                }
                // Check if we need to start decel (for position moves)
                if (!profile_.is_continuous && needsDecelNow()) {
                    profile_.phase = ProfilePhase::DECEL;
                }
                break;

            case ProfilePhase::CONST:
                // Velocity stays constant
                if (!profile_.is_continuous && needsDecelNow()) {
                    profile_.phase = ProfilePhase::DECEL;
                }
                break;

            case ProfilePhase::DECEL:
                profile_.current_velocity -= profile_.acceleration / profile_.current_velocity;
                if (profile_.current_velocity <= 0 || profile_.pulses_remaining <= 0) {
                    profile_.phase = ProfilePhase::DONE;
                }
                break;

            case ProfilePhase::DONE:
                break;
        }
    }

    bool needsDecelNow() const {
        // Calculate pulses needed to decelerate from current velocity to zero
        // d = v² / (2a)
        float decel_pulses = (profile_.current_velocity * profile_.current_velocity)
                           / (2.0f * profile_.acceleration);
        return profile_.pulses_remaining <= (int64_t)decel_pulses;
    }
};
```

#### Buffer Configuration Constants

Add to `config_limits.h`:

```c
// ============================================================================
// PULSE GENERATION BUFFER SIZES
// ============================================================================
// Double-buffer streaming for unlimited motion length
#define LIMIT_RMT_BUFFER_SYMBOLS    512     // Symbols per buffer (×2 for double)
#define LIMIT_RMT_BUFFER_COUNT      2       // Always 2 for double-buffering
#define LIMIT_MCPWM_BUFFER_PULSES   256     // MCPWM pulse batch size
#define LIMIT_LEDC_BUFFER_PULSES    128     // LEDC pulse batch size

// Timing constraints
#define LIMIT_MIN_PULSE_PERIOD_NS   500     // 2MHz max pulse rate
#define LIMIT_MAX_PULSE_PERIOD_NS   1000000000  // 1Hz min pulse rate (1 second)
```

#### Motion Examples

**Short Move (10 pulses):**
```
1. startMove(10, 1000, 5000) called
2. PRIMING: Fill buffer A (10 symbols), buffer B empty
3. STREAMING: Transmit buffer A
4. DRAINING: No refill needed - motion < 1 buffer
5. TX-done: Profile complete, fire callback
```

**Long Move (100,000 pulses):**
```
1. startMove(100000, 50000, 10000) called
2. PRIMING: Fill buffer A (512 sym), fill buffer B (512 sym)
3. STREAMING: Transmit A, refill A when done, swap to B
4. Continue until pulses_remaining < 512
5. DRAINING: Final partial buffer, no refill
6. TX-done: Profile complete, fire callback
```

**Continuous Jog (VEL command):**
```
1. startVelocity(10000, 5000) called
2. PRIMING: Fill buffer A, fill buffer B (both at accel ramp)
3. STREAMING: Endless loop - refill buffers with constant-velocity pulses
4. stop(5000) called → switch to DECEL phase
5. DRAINING: Decel ramp completes
6. TX-done: Motion stopped, fire callback
```

### SI Units Convention (ROS2 Compatible)

> **⚠️ ARCHITECTURE CONSTRAINT - APPLIES TO ALL INTERFACES**
>
> **All external interfaces use SI units.** This is non-negotiable.
>
> | Quantity | Unit | Symbol | Notes |
> |----------|------|--------|-------|
> | Linear position | meters | m | Not mm, not inches |
> | Angular position | radians | rad | Not degrees |
> | Linear velocity | meters/second | m/s | |
> | Angular velocity | radians/second | rad/s | |
> | Linear acceleration | meters/second² | m/s² | |
> | Angular acceleration | radians/second² | rad/s² | |
> | Time | seconds | s | |
>
> **ROS2 Compatibility:** These units match REP-103 (Standard Units of Measure) for seamless future integration with ros2_control hardware_interface.

#### Unit Domains

The system has three unit domains with clear boundaries:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         EXTERNAL DOMAIN                                 │
│                     (Host Interface - SI Units)                         │
│                                                                         │
│   Commands:    MOVE X 0.150 0.050      (150mm at 50mm/s)               │
│                VEL Y 1.571             (90°/s in radians)              │
│   Responses:   POS X 0.150             (position in meters)            │
│   Config:      limits: [-0.500, 0.500] (±500mm in meters)              │
│                                                                         │
│   Units: meters, radians, seconds                                       │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             │ IMotor interface (SI units)
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         MOTOR DOMAIN                                    │
│                    (Unit Conversion Layer)                              │
│                                                                         │
│   AxisConfig (two fundamental parameters):                              │
│     pulses_per_rev = 10000       (driver PA14 setting or gear ratio)   │
│     units_per_rev = 0.005        (5mm ball screw = 0.005 m/rev)        │
│                                                                         │
│   Derived:                                                              │
│     pulses_per_unit = pulses_per_rev / units_per_rev                   │
│                     = 10000 / 0.005 = 2,000,000 pulses/meter           │
│                                                                         │
│   Conversion (linear axis example):                                     │
│     position_m = 0.150                                                  │
│     pulses = position_m * pulses_per_unit = 300000                     │
│     velocity_pps = velocity_mps * pulses_per_unit                      │
│                                                                         │
│   Z-signal sync: 1 Z-pulse = 1 rev = pulses_per_rev = units_per_rev   │
│   Position tracking in SI units (meters or radians)                     │
│   Soft limit checking in SI units                                       │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             │ IPulseGenerator interface (pulses)
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         PULSE DOMAIN                                    │
│                    (Hardware Abstraction)                               │
│                                                                         │
│   startMove(150000, 50000.0f, accel_pps2)                              │
│                                                                         │
│   Units: pulses, pulses/second, pulses/second²                         │
│   No awareness of physical units - pure pulse counting                  │
└─────────────────────────────────────────────────────────────────────────┘
```

#### Axis Configuration Structure

```cpp
// Per-axis configuration - stored in NVS, loaded at startup
struct AxisConfig {
    // === Unit Conversion (two fundamental parameters) ===
    uint32_t pulses_per_rev;      // Command pulses for 1 motor revolution (driver PA14 or gear ratio)
    float units_per_rev;          // Physical travel per revolution: meters (linear) or radians (rotary)
    bool is_rotary;               // true = radians, false = meters

    // === Derived (calculated at load time) ===
    // pulses_per_unit = pulses_per_rev / units_per_rev

    // === Soft Limits (SI units) ===
    float limit_min;              // meters or radians
    float limit_max;              // meters or radians

    // === Motion Parameters (SI units) ===
    float max_velocity;           // m/s or rad/s
    float max_acceleration;       // m/s² or rad/s²

    // === Compensation ===
    float backlash;               // meters or radians
    float home_offset;            // meters or radians

    // === Z-Signal Sync (servo axes only) ===
    bool z_signal_enabled;        // Enable Z-signal position sync
    int32_t z_drift_threshold;    // Max allowed drift in pulses before alarm (0 = auto-correct only)

    // === Identity ===
    char alias[LIMIT_ALIAS_MAX_LENGTH + 1];  // Human-readable name

    // === Computed at runtime (not stored) ===
    float pulses_per_unit() const {
        return static_cast<float>(pulses_per_rev) / units_per_rev;
    }
};

// Compile-time validation
static_assert(sizeof(AxisConfig) <= 80, "AxisConfig must fit in NVS blob");
```

#### Unit Conversion Implementation

```cpp
class ServoMotor : public IMotor {
private:
    AxisConfig config_;
    std::unique_ptr<IPulseGenerator> pulse_gen_;

    // === POSITION AUTHORITY ===
    // pulse_count_ is the SINGLE SOURCE OF TRUTH for position.
    // current_position_ is DERIVED from pulse_count_ / pulses_per_unit.
    // All position modifications go through pulse_count_ first.
    std::atomic<int64_t> pulse_count_{0};  // AUTHORITATIVE - thread-safe for ISR access
    float current_position_;               // DERIVED - SI units (meters or radians)

    // Z-signal synchronization state
    std::atomic<int32_t> z_signal_count_{0};  // Number of Z-signals seen since home
    int32_t z_drift_accumulated_;              // Accumulated drift corrections
    std::atomic<int32_t> z_pending_correction_{0};  // Deferred correction (applied when IDLE)

    // Convert SI units to pulses (uses derived pulses_per_unit)
    int32_t toPulses(float si_units) const {
        return static_cast<int32_t>(si_units * config_.pulses_per_unit());
    }

    // Convert pulses to SI units
    float toSIUnits(int64_t pulses) const {
        return static_cast<float>(pulses) / config_.pulses_per_unit();
    }

    // Convert velocity: SI units/sec → pulses/sec
    float toVelocityPPS(float velocity_si) const {
        return fabsf(velocity_si) * config_.pulses_per_unit();
    }

    // Convert acceleration: SI units/sec² → pulses/sec²
    float toAccelPPS2(float accel_si) const {
        return accel_si * config_.pulses_per_unit();
    }

public:
    esp_err_t moveAbsolute(float position, float velocity) override {
        // Validate against soft limits (SI units)
        if (position < config_.limit_min || position > config_.limit_max) {
            return ESP_ERR_INVALID_ARG;
        }
        if (velocity > config_.max_velocity) {
            velocity = config_.max_velocity;
        }

        // Calculate delta in SI units
        float delta = position - current_position_;

        // Apply backlash compensation if direction changed
        // ... (backlash logic)

        // Convert to pulse domain
        int32_t pulses = toPulses(delta);
        float velocity_pps = toVelocityPPS(velocity);
        float accel_pps2 = toAccelPPS2(config_.max_acceleration);

        // Delegate to pulse generator (no unit awareness)
        return pulse_gen_->startMove(pulses, velocity_pps, accel_pps2);
    }

    float getPosition() const override {
        // Return position in SI units
        return current_position_;
    }

    // Z-signal ISR callback - called from GPIO interrupt
    // DEFERRED CORRECTION: Drift is queued and applied when motion completes (IDLE state)
    // This prevents mid-trajectory position jumps that could cause mechanical issues.
    void IRAM_ATTR onZSignal() {
        if (!config_.z_signal_enabled) return;

        z_signal_count_.fetch_add(1, std::memory_order_relaxed);
        int32_t count = z_signal_count_.load(std::memory_order_relaxed);

        // Expected pulse count at this Z-signal
        int64_t expected = static_cast<int64_t>(count) * config_.pulses_per_rev;
        int64_t actual = pulse_count_.load(std::memory_order_relaxed);
        int32_t drift = static_cast<int32_t>(actual - expected);

        if (drift != 0) {
            // DEFERRED: Accumulate correction, don't apply immediately
            z_pending_correction_.fetch_add(drift, std::memory_order_relaxed);

            // Notify host of detected drift (not yet corrected)
            BaseType_t woken = pdFALSE;
            Event evt = {
                .type = EVT_ZSYNC_DETECTED,
                .axis = axis_index_,
                .data.drift = drift
            };
            EventManager::publishFromISR(evt, &woken);

            // Check if drift exceeds threshold (alarm even before correction)
            if (config_.z_drift_threshold > 0 &&
                abs(drift) > config_.z_drift_threshold) {
                Event alarm_evt = {
                    .type = EVT_ALARM_TRIGGERED,
                    .axis = axis_index_,
                    .data.code = ERR_ZSYNC_DRIFT
                };
                EventManager::publishFromISR(alarm_evt, &woken);
            }

            portYIELD_FROM_ISR(woken);
        }
    }

    // Called when motion completes (from motion task context, not ISR)
    // Applies any accumulated Z-signal corrections
    void applyDeferredZCorrection() {
        int32_t correction = z_pending_correction_.exchange(0, std::memory_order_acq_rel);
        if (correction != 0) {
            // Apply correction to authoritative pulse count
            pulse_count_.fetch_sub(correction, std::memory_order_relaxed);
            z_drift_accumulated_ += correction;

            // Update derived position
            current_position_ = toSIUnits(pulse_count_.load(std::memory_order_relaxed));

            // Notify host that correction was applied
            EventManager::publish(Event{
                .type = EVT_ZSYNC_CORRECTED,
                .axis = axis_index_,
                .data.drift = correction
            });
        }
    }

    float getVelocity() const override {
        // Convert current pulse rate to SI units
        float velocity_pps = pulse_gen_->getCurrentVelocity();
        return velocity_pps / config_.pulses_per_unit;
    }
};
```

#### Typical Axis Configurations

| Axis | Type | pulses_per_rev | units_per_rev | Derived pulses/unit | Z-signal |
|------|------|----------------|---------------|---------------------|----------|
| X (Railway) | Linear | 10000 | 0.005 m (5mm screw) | 2,000,000 | Yes |
| Y (Gripper) | Rotary | 10000 | 6.283 rad (direct) | 1,592 | Yes |
| Z (Vertical) | Linear | 10000 | 0.010 m (10mm screw) | 1,000,000 | Yes |
| A, B (Picker) | Linear | 10000 | 0.005 m (5mm screw) | 2,000,000 | Yes |
| C (Stepper) | Rotary | 6400 | 6.283 rad (direct) | 1,019 | No |
| D (Stepper) | Linear | 6400 | 0.008 m (8mm screw) | 800,000 | No |
| E (Discrete) | Binary | 1 | 1 | 1 | No |

> **Note:** `pulses_per_rev` matches servo driver PA14 setting or stepper microstep config.
> `units_per_rev` is mechanical: ball screw lead (m) or 2π for direct-drive rotary.

#### YAML Configuration (SI Units)

```yaml
axes:
  X:
    alias: "RAILWAY"
    type: linear                    # meters
    pulses_per_rev: 10000           # Driver PA14 = 10000 pulses/revolution
    units_per_rev: 0.005            # 5mm ball screw lead = 0.005 m/rev
    # Derived: pulses_per_unit = 10000 / 0.005 = 2,000,000 pulses/meter
    limits: [-0.500, 0.500]         # ±500mm in meters
    max_velocity: 0.200             # 200mm/s in m/s
    max_acceleration: 1.0           # 1 m/s²
    backlash: 0.00005               # 50µm in meters
    home_offset: 0.0
    z_signal:
      enabled: true                 # Enable Z-signal position sync
      drift_threshold: 100          # Alarm if drift > 100 pulses (0 = no alarm, just correct)

  Y:
    alias: "GRIPPER"
    type: rotary                    # radians
    pulses_per_rev: 10000           # Driver PA14 = 10000 pulses/revolution
    units_per_rev: 6.283185         # 2π rad/rev (direct drive, no gearbox)
    # Derived: pulses_per_unit = 10000 / 6.283185 = 1,592 pulses/radian
    limits: [0.0, 6.283185]         # 0 to 2π radians (360°)
    max_velocity: 6.283185          # 2π rad/s (360°/s)
    max_acceleration: 31.415927     # 10π rad/s² (1800°/s²)
    backlash: 0.0
    home_offset: 0.0
    z_signal:
      enabled: true
      drift_threshold: 50

  C:
    alias: "JAW"
    type: rotary                    # radians
    pulses_per_rev: 6400            # Stepper: 200 steps × 32 microsteps
    units_per_rev: 6.283185         # 2π rad/rev (direct drive)
    limits: [-3.14159, 3.14159]     # ±180° in radians
    max_velocity: 3.14159           # π rad/s (180°/s)
    max_acceleration: 15.708        # 5π rad/s²
    backlash: 0.0
    home_offset: 0.0
    # No z_signal section = disabled (steppers don't have Z-signal)

  # ... other axes follow same schema
```

#### Command Examples (SI Units)

```
# Linear axis (X) - all values in meters
MOVE X 0.150 0.050        # Move to 150mm at 50mm/s
MOVR X -0.025 0.100       # Move -25mm relative at 100mm/s
VEL X 0.030               # Jog at 30mm/s (uses max_acceleration from config)
POS X                     # Response: OK 0.150000

# Rotary axis (Y) - all values in radians
MOVE Y 1.570796 3.14159   # Move to π/2 rad (90°) at π rad/s (180°/s)
VEL Y -0.523599           # Jog at -30°/s (in radians: -π/6)
POS Y                     # Response: OK 1.570796

# Setting limits (SI units)
SETL X -0.500 0.500       # Set limits to ±500mm (in meters)
SETV X 0.300              # Set max velocity to 300mm/s (in m/s)
```

### Homing Workflow

> **⚠️ ARCHITECTURE CONSTRAINT - HOMING ESTABLISHES POSITION REFERENCE**
>
> **All servo axes require homing before accurate position tracking.** Z-signal synchronization depends on homing being completed first.
>
> - Homing establishes the Z-signal count reference (`z_signal_count_ = 0`)
> - Without homing, Z-signal corrections are invalid
> - Stepper axes also require homing but use limit switch only (no Z-signal)

#### Homing Sequence (Servo Axes with Z-Signal)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     HOMING STATE MACHINE                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   ┌──────────────┐                                                      │
│   │   UNHOMED    │  ← Initial state after power-on                      │
│   └──────┬───────┘                                                      │
│          │ HOME command received                                        │
│          ▼                                                              │
│   ┌──────────────┐                                                      │
│   │ SEEK_LIMIT   │  ← Move toward MIN limit at homing_velocity          │
│   └──────┬───────┘                                                      │
│          │ Limit switch activated                                       │
│          ▼                                                              │
│   ┌──────────────┐                                                      │
│   │   BACKOFF    │  ← Back off from limit by homing_backoff distance    │
│   └──────┬───────┘                                                      │
│          │ Backoff complete                                             │
│          ▼                                                              │
│   ┌──────────────┐                                                      │
│   │ SEEK_ZSIGNAL │  ← Move slowly toward limit, waiting for Z-signal    │
│   └──────┬───────┘                                                      │
│          │ Z-signal detected (first index pulse)                        │
│          ▼                                                              │
│   ┌──────────────┐                                                      │
│   │  SET_HOME    │  ← Reset: pulse_count_=0, z_signal_count_=0          │
│   └──────┬───────┘    current_position_=0, apply home_offset            │
│          │                                                              │
│          ▼                                                              │
│   ┌──────────────┐                                                      │
│   │    HOMED     │  ← Ready for normal operation                        │
│   └──────────────┘    Fire EVT_HOMING_COMPLETE                          │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

#### Homing Configuration

```cpp
// Additional AxisConfig fields for homing
struct AxisConfig {
    // ... existing fields ...

    // === Homing Parameters ===
    float homing_velocity;        // Velocity for limit seek (m/s or rad/s)
    float homing_velocity_slow;   // Velocity for Z-signal seek (slower)
    float homing_backoff;         // Distance to back off from limit (m or rad)
    bool homing_direction;        // true = toward MIN limit, false = toward MAX
};
```

#### Homing Implementation

```cpp
class ServoMotor : public IMotor {
public:
    // Initiate homing sequence
    esp_err_t startHoming() {
        if (isMoving()) return ESP_ERR_INVALID_STATE;

        homing_state_ = HomingState::SEEK_LIMIT;

        // Move toward limit at homing velocity
        float direction = config_.homing_direction ? -1.0f : 1.0f;
        float velocity = direction * config_.homing_velocity;

        // Use velocity mode - limit switch will stop us
        return moveVelocity(velocity);
    }

    // Called when limit switch triggers during homing
    void onLimitDuringHoming(bool is_min_limit) {
        if (homing_state_ != HomingState::SEEK_LIMIT) return;

        stopImmediate();
        homing_state_ = HomingState::BACKOFF;

        // Back off from limit
        float backoff = config_.homing_direction ?
            config_.homing_backoff : -config_.homing_backoff;
        moveRelative(backoff, config_.homing_velocity);
    }

    // Called when backoff move completes
    void onBackoffComplete() {
        homing_state_ = HomingState::SEEK_ZSIGNAL;

        // Move slowly toward limit, watching for Z-signal
        float direction = config_.homing_direction ? -1.0f : 1.0f;
        moveVelocity(direction * config_.homing_velocity_slow);
    }

    // Called when Z-signal detected during SEEK_ZSIGNAL phase
    void onZSignalDuringHoming() {
        if (homing_state_ != HomingState::SEEK_ZSIGNAL) return;

        stopImmediate();

        // === ESTABLISH HOME REFERENCE ===
        pulse_count_.store(0, std::memory_order_release);
        z_signal_count_.store(0, std::memory_order_release);
        z_pending_correction_.store(0, std::memory_order_release);
        z_drift_accumulated_ = 0;
        current_position_ = config_.home_offset;

        homing_state_ = HomingState::HOMED;

        EventManager::publish(Event{
            .type = EVT_HOMING_COMPLETE,
            .axis = axis_index_,
            .data.position = current_position_
        });
    }

private:
    enum class HomingState { UNHOMED, SEEK_LIMIT, BACKOFF, SEEK_ZSIGNAL, HOMED };
    HomingState homing_state_ = HomingState::UNHOMED;
};
```

#### Homing Sequence (Stepper Axes - No Z-Signal)

Steppers use a simpler homing sequence since they lack encoder index signals:

```
1. Move toward MIN limit at homing_velocity
2. Limit switch activated → stop immediately
3. Back off from limit by homing_backoff distance
4. Set position: pulse_count_=0, pcnt_count_=0, current_position_=home_offset
5. Fire EVT_HOMING_COMPLETE
```

> **Note:** Stepper axes should be homed at the start of each work cycle to prevent cumulative position drift from missed steps.

### Velocity Mode Control

> **⚠️ ARCHITECTURE CONSTRAINT - VELOCITY MODE PARAMETERS**
>
> **Velocity mode (`VEL` command) uses `max_acceleration` from config by default.**
> Optional acceleration parameter in command overrides config for this move only.
>
> - `VEL X 0.050` — uses config.max_acceleration
> - `VEL X 0.050 2.0` — uses 2.0 m/s² for this move only
> - Velocity changes during active VEL mode ramp smoothly (no discontinuity)

#### Velocity Mode Implementation

```cpp
class ServoMotor : public IMotor {
public:
    // Velocity mode - jog at specified velocity
    // Acceleration from parameter if provided, otherwise from config
    esp_err_t moveVelocity(float velocity, float acceleration = 0) override {
        // Clamp velocity to configured maximum
        if (fabsf(velocity) > config_.max_velocity) {
            velocity = copysignf(config_.max_velocity, velocity);
        }

        // Use provided acceleration or fall back to config
        float accel = (acceleration > 0) ? acceleration : config_.max_acceleration;

        // Store direction for position tracking
        velocity_direction_ = (velocity >= 0) ? 1 : -1;

        // Convert to pulse domain
        float velocity_pps = toVelocityPPS(velocity);
        float accel_pps2 = toAccelPPS2(accel);

        // Start continuous motion (or transition smoothly if already moving)
        return pulse_gen_->startVelocity(velocity_pps, accel_pps2);
    }
};
```

#### Soft Limit Handling in Velocity Mode

Velocity mode respects soft limits by decelerating to stop exactly at the limit boundary:

```cpp
class ServoMotor : public IMotor {
private:
    // Called periodically during velocity mode motion (from motion task)
    void checkSoftLimitsInVelocityMode() {
        if (!pulse_gen_->isRunning()) return;

        float pos = getPosition();
        float vel = getVelocity();
        float accel = config_.max_acceleration;

        // Calculate stopping distance: d = v² / (2a)
        float stopping_distance = (vel * vel) / (2.0f * accel);

        // Check if we need to start decelerating
        bool approaching_min = (vel < 0) &&
            (pos - stopping_distance <= config_.limit_min);
        bool approaching_max = (vel > 0) &&
            (pos + stopping_distance >= config_.limit_max);

        if (approaching_min || approaching_max) {
            // Begin controlled deceleration to stop at limit
            pulse_gen_->stop(toAccelPPS2(accel));

            EventManager::publish(Event{
                .type = EVT_SOFT_LIMIT_APPROACH,
                .axis = axis_index_,
                .data.position = approaching_min ? config_.limit_min : config_.limit_max
            });
        }
    }
};
```

### PCNT Verification for Steppers

> **⚠️ ARCHITECTURE CONSTRAINT - PCNT MISMATCH HANDLING**
>
> **PCNT (Pulse Counter) verifies commanded vs actual pulses for stepper axes.**
> Mismatch detection logs a warning but does NOT stop motion.
> Steppers are homed every work cycle, so drift is reset regularly.

#### PCNT Verification Implementation

```cpp
class StepperMotor : public IMotor {
private:
    pcnt_unit_handle_t pcnt_unit_;
    std::atomic<int64_t> commanded_pulses_{0};  // What we commanded
    int pcnt_overflow_count_ = 0;               // Track PCNT overflows

public:
    // Called after motion completes
    void verifyPositionWithPCNT() {
        int pcnt_count;
        pcnt_unit_get_count(pcnt_unit_, &pcnt_count);

        // Calculate total PCNT pulses (handling overflow)
        int64_t total_pcnt = (int64_t)pcnt_overflow_count_ * PCNT_HIGH_LIMIT + pcnt_count;
        int64_t commanded = commanded_pulses_.load(std::memory_order_relaxed);

        int64_t mismatch = total_pcnt - commanded;

        if (mismatch != 0) {
            // Log warning but continue - don't stop motion
            ESP_LOGW(TAG, "Axis %d PCNT mismatch: commanded=%lld, actual=%lld, diff=%lld",
                     axis_index_, commanded, total_pcnt, mismatch);

            EventManager::publish(Event{
                .type = EVT_PCNT_MISMATCH,
                .axis = axis_index_,
                .data.mismatch = (int32_t)mismatch
            });

            // DO NOT auto-correct - trust commanded position
            // User should re-home if concerned about accuracy
        }
    }

    // Called during homing
    void resetPCNT() {
        pcnt_unit_clear_count(pcnt_unit_);
        pcnt_overflow_count_ = 0;
        commanded_pulses_.store(0, std::memory_order_release);
    }
};
```

### Motion Completion Signaling

> **⚠️ ARCHITECTURE CONSTRAINT - DUAL COMPLETION MECHANISM**
>
> **Motion completion is available via both callback AND polling.**
> - **Callback**: Fires from motion task context when profile completes
> - **Polling**: `isMoving()` returns false when motion is done
>
> Use callbacks for event-driven architectures, polling for simpler synchronous code.

#### Motion Completion Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                   MOTION COMPLETION FLOW                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   IPulseGenerator                    ServoMotor                         │
│   ┌────────────────┐                ┌────────────────┐                  │
│   │ Profile runs   │                │                │                  │
│   │ DECEL → DONE   │                │                │                  │
│   └───────┬────────┘                │                │                  │
│           │                         │                │                  │
│           │ TX-done ISR             │                │                  │
│           ▼                         │                │                  │
│   ┌────────────────┐                │                │                  │
│   │ Notify motion  │───────────────►│ Motion task   │                  │
│   │ task           │   Task Notify  │ wakes up      │                  │
│   └────────────────┘                └───────┬────────┘                  │
│                                             │                           │
│                                             ▼                           │
│                                     ┌────────────────┐                  │
│                                     │ Apply deferred │                  │
│                                     │ Z-correction   │                  │
│                                     └───────┬────────┘                  │
│                                             │                           │
│                                             ▼                           │
│                                     ┌────────────────┐                  │
│                                     │ Update position│                  │
│                                     │ from pulse_gen │                  │
│                                     └───────┬────────┘                  │
│                                             │                           │
│                                             ▼                           │
│                                     ┌────────────────┐                  │
│                                     │ Fire callback  │                  │
│                                     │ (if registered)│                  │
│                                     └───────┬────────┘                  │
│                                             │                           │
│                                             ▼                           │
│                                     ┌────────────────┐                  │
│                                     │ Publish        │                  │
│                                     │ EVT_MOTION_    │                  │
│                                     │ COMPLETE       │                  │
│                                     └────────────────┘                  │
│                                                                         │
│   Polling: isMoving() returns pulse_gen_->isRunning()                   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

#### Implementation

```cpp
class ServoMotor : public IMotor {
public:
    using MotionCompleteCallback = std::function<void(uint8_t axis, float position)>;

    void setMotionCompleteCallback(MotionCompleteCallback cb) {
        motion_complete_cb_ = cb;
    }

    bool isMoving() const override {
        return pulse_gen_->isRunning();
    }

private:
    MotionCompleteCallback motion_complete_cb_;

    // Called from motion task when pulse generator signals completion
    void onMotionComplete() {
        // Apply any deferred Z-signal corrections
        applyDeferredZCorrection();

        // Update position from pulse generator
        int64_t pulses = pulse_gen_->getPulseCount();
        pulse_count_.store(pulses, std::memory_order_release);
        current_position_ = toSIUnits(pulses);

        // Fire callback if registered
        if (motion_complete_cb_) {
            motion_complete_cb_(axis_index_, current_position_);
        }

        // Publish event for async subscribers
        EventManager::publish(Event{
            .type = EVT_MOTION_COMPLETE,
            .axis = axis_index_,
            .data.position = current_position_
        });
    }
};
```

### Motor Control Pattern

```cpp
// All motor types implement this interface
// ALL PARAMETERS IN SI UNITS (meters, radians, seconds)
class IMotor {
public:
    // Position moves - position in meters (linear) or radians (rotary)
    virtual esp_err_t moveAbsolute(float position, float velocity) = 0;
    virtual esp_err_t moveRelative(float delta, float velocity) = 0;

    // Velocity mode - velocity in m/s or rad/s
    virtual esp_err_t moveVelocity(float velocity) = 0;

    // Stop commands
    virtual esp_err_t stop() = 0;              // Controlled deceleration
    virtual esp_err_t stopImmediate() = 0;     // Emergency stop

    // Status - returns SI units
    virtual float getPosition() const = 0;     // meters or radians
    virtual float getVelocity() const = 0;     // m/s or rad/s
    virtual bool isMoving() const = 0;

    // Enable/disable
    virtual void enable(bool en) = 0;

    // Configuration
    virtual const AxisConfig& getConfig() const = 0;
    virtual esp_err_t setConfig(const AxisConfig& config) = 0;
};

// Composition: Motor uses injected pulse generator and position tracker
class ServoMotor : public IMotor {
private:
    AxisConfig config_;
    std::unique_ptr<IPulseGenerator> pulse_gen_;
    std::unique_ptr<IPositionTracker> tracker_;
    ShiftRegisterController* shift_reg_;
    uint8_t axis_id_;
    float current_position_;  // SI units
    // ...
};
```

### Motion Blending Behavior

> **⚠️ ARCHITECTURE CONSTRAINT - MID-MOTION TARGET CHANGES**
>
> **New MOVE commands during active motion blend to the new target.** This is non-negotiable.
>
> - No "axis busy" errors - motion is always interruptible
> - Profile generator recalculates trajectory on-the-fly
> - Current velocity preserved for smooth transition
> - Same behavior for VEL→MOVE transitions

#### Blending State Machine

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    MOTION BLENDING SCENARIOS                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   SCENARIO 1: MOVE during MOVE (target change)                          │
│   ─────────────────────────────────────────────                         │
│   Current: Moving to 100mm at 50mm/s, currently at 60mm                 │
│   New cmd: MOVE X 150                                                   │
│   Result:  Recalculate profile from (pos=60, vel=current) to 150mm      │
│            No stop, smooth transition to new target                     │
│                                                                         │
│   SCENARIO 2: VEL during VEL (velocity change)                          │
│   ─────────────────────────────────────────────                         │
│   Current: Jogging at 30mm/s                                            │
│   New cmd: VEL X 60                                                     │
│   Result:  Accelerate from 30mm/s to 60mm/s using max_acceleration      │
│            Smooth ramp, no discontinuity                                │
│                                                                         │
│   SCENARIO 3: MOVE during VEL (mode transition)                         │
│   ─────────────────────────────────────────────                         │
│   Current: Jogging at 40mm/s, currently at 80mm                         │
│   New cmd: MOVE X 120                                                   │
│   Result:  Calculate trajectory from (pos=80, vel=40) to target=120     │
│            May require decel if overshooting, or continued accel        │
│                                                                         │
│   SCENARIO 4: MOVE during STOP (decel override)                         │
│   ─────────────────────────────────────────────                         │
│   Current: Decelerating after STOP command                              │
│   New cmd: MOVE X 200                                                   │
│   Result:  Blend from current (pos, vel) to new target                  │
│            No need to wait for full stop                                │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

#### Blend Implementation

```cpp
class ServoMotor : public IMotor {
public:
    esp_err_t moveAbsolute(float position, float velocity) override {
        // Validate against soft limits
        if (position < config_.limit_min || position > config_.limit_max) {
            return ESP_ERR_INVALID_ARG;
        }

        // Get current state (works whether moving or stopped)
        float current_pos = getPosition();
        float current_vel = pulse_gen_->isRunning() ?
            toSIUnits(pulse_gen_->getCurrentVelocity()) : 0.0f;

        // Calculate new trajectory from current state to target
        TrajectoryParams traj = calculateBlendTrajectory(
            current_pos, current_vel,  // Start state
            position, velocity,         // Target state
            config_.max_acceleration    // Constraint
        );

        // If currently moving, this replaces the active profile
        // Profile generator handles the transition seamlessly
        return startTrajectory(traj);
    }

private:
    TrajectoryParams calculateBlendTrajectory(
        float start_pos, float start_vel,
        float target_pos, float target_vel,
        float max_accel
    ) {
        float delta = target_pos - start_pos;
        float direction = (delta >= 0) ? 1.0f : -1.0f;

        // Check if we need to reverse direction
        bool same_direction = (start_vel * direction) >= 0;

        if (!same_direction && fabsf(start_vel) > 0.001f) {
            // Moving wrong way - must decel to zero first, then accel to target
            // This is the most complex case
            return calculateReversalTrajectory(start_pos, start_vel,
                                               target_pos, target_vel, max_accel);
        }

        // Same direction or stopped - standard trapezoidal blend
        return calculateTrapezoidalTrajectory(start_pos, fabsf(start_vel),
                                              target_pos, target_vel, max_accel);
    }
};
```

### Command Handler Pattern

```cpp
// Each command type has a dedicated handler
typedef esp_err_t (*CommandHandler)(const ParsedCommand* cmd, Response* resp);

static const CommandEntry command_table[] = {
    { CMD_MOVE,  handle_move,  STATE_READY | STATE_MOVING },
    { CMD_STOP,  handle_stop,  STATE_ANY },
    { CMD_EN,    handle_enable, STATE_READY },
    // ...
};

// Dispatcher validates state before calling handler
esp_err_t dispatch_command(const ParsedCommand* cmd, Response* resp) {
    for (const auto& entry : command_table) {
        if (strcmp(cmd->verb, entry.verb) == 0) {
            if (!(current_state & entry.allowed_states)) {
                return ERR_MODE_BLOCKED;
            }
            return entry.handler(cmd, resp);
        }
    }
    return ERR_INVALID_COMMAND;
}
```

### Event Publication Pattern

```cpp
// Central event manager for decoupled communication
typedef struct {
    EventType type;
    uint8_t axis;
    union {
        float position;
        float width;
        uint8_t error_code;
    } data;
    int64_t timestamp;
} Event;

// Publishers (motion, safety, I/O)
EventManager::publish(Event{ .type = EVT_MOTION_COMPLETE, .axis = 2, .data.position = 100.0 });

// Subscribers (USB TX, OLED, logging)
EventManager::subscribe(EVT_MOTION_COMPLETE, [](const Event& e) {
    send_event_response("DONE %c %.2f", 'X' + e.axis, e.data.position);
});
```

### Background Task Pattern

> **⚠️ ARCHITECTURE CONSTRAINT - APPLIES TO ALL NON-CRITICAL ASYNC WORK**
>
> **All non-time-critical operations MUST be deferred to background tasks.** This is non-negotiable.
>
> - NVS writes (persist configuration)
> - OLED display updates
> - Logging to serial
> - Event notifications to host
> - Diagnostic data collection
>
> **Never perform blocking I/O in ISRs or high-priority tasks.**

#### Background Work Queue Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                     BACKGROUND WORK SYSTEM                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌──────────────┐    ┌──────────────┐    ┌──────────────┐        │
│   │  ISR Context │    │ High-Priority│    │ Normal Tasks │        │
│   │  (E-stop,    │    │ Tasks        │    │ (Command     │        │
│   │   Limits)    │    │ (Safety,     │    │  Executor)   │        │
│   │              │    │  Motion)     │    │              │        │
│   └──────┬───────┘    └──────┬───────┘    └──────┬───────┘        │
│          │                   │                   │                 │
│          │ xQueueSendFromISR │ xQueueSend        │ xQueueSend     │
│          │                   │                   │                 │
│          ▼                   ▼                   ▼                 │
│   ┌─────────────────────────────────────────────────────────────┐ │
│   │                   BACKGROUND WORK QUEUE                      │ │
│   │              (FreeRTOS Queue, depth: 32)                     │ │
│   └──────────────────────────┬──────────────────────────────────┘ │
│                              │                                     │
│                              ▼                                     │
│   ┌─────────────────────────────────────────────────────────────┐ │
│   │              BACKGROUND WORKER TASK                          │ │
│   │         (Core 0, Priority 5 - Low Priority)                  │ │
│   │                                                              │ │
│   │   Processes work items sequentially:                         │ │
│   │   • NVS writes (batched if possible)                         │ │
│   │   • OLED frame buffer updates                                │ │
│   │   • USB TX response queueing                                 │ │
│   │   • Log message formatting                                   │ │
│   └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

#### Work Item Definition

```cpp
// Work item types for background processing
typedef enum {
    WORK_NVS_WRITE,         // Persist configuration to NVS
    WORK_NVS_COMMIT,        // Commit NVS namespace
    WORK_OLED_UPDATE,       // Refresh OLED display
    WORK_LOG_MESSAGE,       // Format and output log message
    WORK_USB_RESPONSE,      // Queue response to USB TX
    WORK_DIAG_COLLECT,      // Collect diagnostic data
} WorkType;

// Work item structure (fits in queue)
typedef struct {
    WorkType type;
    union {
        struct {
            char key[16];
            uint32_t value;
        } nvs_u32;
        struct {
            char key[16];
            float value;
        } nvs_float;
        struct {
            uint8_t line;
            char text[22];  // OLED_CHARS_PER_LINE + 1
        } oled;
        struct {
            esp_log_level_t level;
            char tag[16];
            char message[64];
        } log;
        struct {
            char response[LIMIT_RESPONSE_MAX_LENGTH];
        } usb;
    } data;
} WorkItem;

// Queue handle (global, initialized at startup)
static QueueHandle_t background_work_queue = NULL;
```

#### Background Worker Task

```cpp
// Background worker task - processes non-critical async work
void background_worker_task(void* param) {
    WorkItem item;

    while (1) {
        // Block until work available (no timeout - idle when empty)
        if (xQueueReceive(background_work_queue, &item, portMAX_DELAY) == pdTRUE) {
            switch (item.type) {
                case WORK_NVS_WRITE:
                    // NVS operations can block - that's OK here
                    nvs_set_u32(nvs_handle, item.data.nvs_u32.key,
                                item.data.nvs_u32.value);
                    break;

                case WORK_NVS_COMMIT:
                    nvs_commit(nvs_handle);
                    break;

                case WORK_OLED_UPDATE:
                    oled_draw_text(item.data.oled.line, item.data.oled.text);
                    break;

                case WORK_LOG_MESSAGE:
                    ESP_LOG_LEVEL(item.data.log.level, item.data.log.tag,
                                  "%s", item.data.log.message);
                    break;

                case WORK_USB_RESPONSE:
                    usb_cdc_write(item.data.usb.response,
                                  strlen(item.data.usb.response));
                    break;

                case WORK_DIAG_COLLECT:
                    collect_diagnostic_snapshot();
                    break;
            }
        }
    }
}

// Initialize background work system
esp_err_t background_worker_init(void) {
    background_work_queue = xQueueCreate(
        LIMIT_BACKGROUND_QUEUE_DEPTH,  // 32 items
        sizeof(WorkItem)
    );

    if (background_work_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Create worker task on Core 0 (alongside safety, opposite from motion)
    BaseType_t ret = xTaskCreatePinnedToCore(
        background_worker_task,
        "bg_worker",
        STACK_BACKGROUND_TASK,  // 4096 words
        NULL,
        5,                       // Low priority (safety=24, motion=20)
        &background_task_handle,
        0                        // Core 0
    );

    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}
```

#### Helper Functions for Submitting Work

```cpp
// Submit work from normal task context
esp_err_t background_submit(const WorkItem* item) {
    if (xQueueSend(background_work_queue, item, pdMS_TO_TICKS(10)) != pdTRUE) {
        // Queue full - this is a warning, not fatal
        ESP_LOGW(TAG, "Background queue full, item dropped");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

// Submit work from ISR context (non-blocking)
BaseType_t background_submit_from_isr(const WorkItem* item, BaseType_t* woken) {
    return xQueueSendFromISR(background_work_queue, item, woken);
}

// Convenience: Queue NVS write
esp_err_t background_nvs_write_u32(const char* key, uint32_t value) {
    WorkItem item = {
        .type = WORK_NVS_WRITE,
        .data.nvs_u32 = { .value = value }
    };
    strncpy(item.data.nvs_u32.key, key, sizeof(item.data.nvs_u32.key) - 1);
    return background_submit(&item);
}

// Convenience: Queue OLED update
esp_err_t background_oled_update(uint8_t line, const char* text) {
    WorkItem item = {
        .type = WORK_OLED_UPDATE,
        .data.oled = { .line = line }
    };
    strncpy(item.data.oled.text, text, sizeof(item.data.oled.text) - 1);
    return background_submit(&item);
}
```

#### Configuration Constants

Add to `config_limits.h`:

```c
// ============================================================================
// BACKGROUND WORK QUEUE
// ============================================================================
#define LIMIT_BACKGROUND_QUEUE_DEPTH    32      // Work items in queue
#define STACK_BACKGROUND_TASK           4096    // Stack size (words)
```

#### Usage Examples

```cpp
// From command executor (normal task) - persist configuration change
void handle_setv_command(uint8_t axis, float velocity) {
    axis_config[axis].max_velocity = velocity;

    // Queue NVS write - don't block command processing
    char key[16];
    snprintf(key, sizeof(key), "ax%d_vel", axis);
    background_nvs_write_float(key, velocity);

    send_response("OK");
}

// From motion task - update OLED display
void on_motion_complete(uint8_t axis, float position) {
    char text[22];
    snprintf(text, sizeof(text), "%c: %.3f DONE", 'X' + axis, position);
    background_oled_update(axis % 4, text);
}

// From ISR - queue error notification (never block in ISR!)
void IRAM_ATTR limit_switch_isr(void* arg) {
    uint8_t axis = (uint8_t)(uintptr_t)arg;
    BaseType_t woken = pdFALSE;

    WorkItem item = {
        .type = WORK_USB_RESPONSE,
    };
    snprintf(item.data.usb.response, sizeof(item.data.usb.response),
             "EVENT LIMIT %c\n", 'X' + axis);

    background_submit_from_isr(&item, &woken);
    portYIELD_FROM_ISR(woken);
}
```

### ISR Error Notification Pattern

> **⚠️ ARCHITECTURE CONSTRAINT - APPLIES TO ALL ISR ERROR HANDLING**
>
> **ISRs must NEVER block.** Error handling in ISR context follows this pattern:
>
> 1. Take immediate safety action (stop motors, engage brakes)
> 2. Set atomic flag or notify task
> 3. Exit ISR immediately
> 4. Task handles logging, notifications, recovery

#### Error Notification Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    ISR ERROR NOTIFICATION FLOW                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ISR CONTEXT (< 5µs)                                               │
│   ┌──────────────────────────────────────────────────────────────┐ │
│   │  1. Immediate Safety Action                                   │ │
│   │     • Stop pulse generation (direct register write)           │ │
│   │     • Force outputs safe (GPIO.out_w1tc)                     │ │
│   │                                                               │ │
│   │  2. Notify (non-blocking)                                     │ │
│   │     • xTaskNotifyFromISR() - set error bits                  │ │
│   │     • xQueueSendFromISR() - queue error details              │ │
│   │                                                               │ │
│   │  3. Yield if higher-priority task woken                       │ │
│   │     • portYIELD_FROM_ISR(woken)                              │ │
│   └──────────────────────────────────────────────────────────────┘ │
│                              │                                      │
│                              ▼                                      │
│   TASK CONTEXT (Safety Task, Priority 24)                          │
│   ┌──────────────────────────────────────────────────────────────┐ │
│   │  4. Process Error                                             │ │
│   │     • Read error queue for details                           │ │
│   │     • Update system state machine                            │ │
│   │     • Engage brakes (via shift register)                     │ │
│   │                                                               │ │
│   │  5. Notify External (via Background Worker)                   │ │
│   │     • Queue USB error response                               │ │
│   │     • Queue OLED error display                               │ │
│   │     • Queue log message                                       │ │
│   └──────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

#### Error Types and Notification Bits

```cpp
// Error notification bits (for xTaskNotify)
#define NOTIFY_ERROR_ESTOP          (1 << 0)    // E-stop activated
#define NOTIFY_ERROR_LIMIT          (1 << 1)    // Limit switch triggered
#define NOTIFY_ERROR_ALARM          (1 << 2)    // Driver alarm detected
#define NOTIFY_ERROR_WATCHDOG       (1 << 3)    // Motion watchdog timeout
#define NOTIFY_ERROR_I2C            (1 << 4)    // I2C communication failure
#define NOTIFY_ERROR_OVERCURRENT    (1 << 5)    // Overcurrent detected

// Detailed error information (queued to safety task)
typedef struct {
    uint32_t timestamp;         // esp_timer_get_time() / 1000
    uint8_t error_type;         // NOTIFY_ERROR_* type
    uint8_t axis;               // Affected axis (0-7, or 0xFF for system)
    uint8_t severity;           // 0=warning, 1=error, 2=critical
    union {
        struct {
            bool min_triggered;
            bool max_triggered;
        } limit;
        struct {
            uint8_t alarm_code;
        } driver;
        struct {
            int32_t drift_pulses;
        } position;
    } detail;
} ErrorInfo;

// Error queue for detailed error information
static QueueHandle_t error_info_queue = NULL;
```

#### ISR Implementation Pattern

```cpp
// E-stop ISR - maximum urgency, minimum code
void IRAM_ATTR estop_isr(void* arg) {
    // 1. IMMEDIATE: Stop all motion (direct register access, no function calls)
    RMT.conf_ch[0].conf1.tx_start = 0;
    RMT.conf_ch[1].conf1.tx_start = 0;
    RMT.conf_ch[2].conf1.tx_start = 0;
    RMT.conf_ch[3].conf1.tx_start = 0;
    MCPWM0.timer[0].mode.start = 0;
    MCPWM0.timer[1].mode.start = 0;
    GPIO.out_w1tc = STEP_OUTPUT_MASK;  // Force STEP pins low

    // 2. NOTIFY: Signal safety task
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(safety_task_handle, NOTIFY_ERROR_ESTOP, eSetBits, &woken);

    // 3. YIELD: Let safety task run immediately
    portYIELD_FROM_ISR(woken);
}

// Limit switch ISR - stop specific axis, queue details
void IRAM_ATTR limit_switch_isr(void* arg) {
    uint8_t axis = (uint8_t)(uintptr_t)arg;
    BaseType_t woken = pdFALSE;

    // 1. IMMEDIATE: Stop this axis only
    stop_axis_pulse_generation(axis);  // Must be IRAM_ATTR, register-only

    // 2. QUEUE: Send error details
    ErrorInfo info = {
        .timestamp = (uint32_t)(esp_timer_get_time() / 1000),
        .error_type = NOTIFY_ERROR_LIMIT,
        .axis = axis,
        .severity = 1,  // Error
        .detail.limit = {
            .min_triggered = gpio_get_level(limit_min_gpio[axis]) == 0,
            .max_triggered = gpio_get_level(limit_max_gpio[axis]) == 0,
        }
    };
    xQueueSendFromISR(error_info_queue, &info, &woken);

    // 3. NOTIFY: Signal safety task
    xTaskNotifyFromISR(safety_task_handle, NOTIFY_ERROR_LIMIT, eSetBits, &woken);

    portYIELD_FROM_ISR(woken);
}

// Helper: Stop single axis pulse generation (IRAM_ATTR, register-only)
static void IRAM_ATTR stop_axis_pulse_generation(uint8_t axis) {
    switch (axis) {
        case 0: RMT.conf_ch[RMT_CHANNEL_X].conf1.tx_start = 0; break;  // X
        case 1: MCPWM0.timer[MCPWM_TIMER_Y].mode.start = 0; break;     // Y
        case 2: RMT.conf_ch[RMT_CHANNEL_Z].conf1.tx_start = 0; break;  // Z
        case 3: RMT.conf_ch[RMT_CHANNEL_A].conf1.tx_start = 0; break;  // A
        case 4: RMT.conf_ch[RMT_CHANNEL_B].conf1.tx_start = 0; break;  // B
        case 5: MCPWM0.timer[MCPWM_TIMER_C].mode.start = 0; break;     // C
        case 6: ledc_stop(LEDC_MODE, LEDC_CHANNEL_D, 0); break;        // D
        // E-axis is discrete, no pulse generation
    }
}
```

#### Safety Task Error Processing

```cpp
void safety_monitor_task(void* param) {
    ErrorInfo error;
    uint32_t notify_bits;

    while (1) {
        // Wait for error notification or periodic check
        if (xTaskNotifyWait(0, ULONG_MAX, &notify_bits,
                           pdMS_TO_TICKS(TIMING_SAFETY_POLL_MS))) {

            // Process each error type
            if (notify_bits & NOTIFY_ERROR_ESTOP) {
                handle_estop_error();
            }

            if (notify_bits & NOTIFY_ERROR_LIMIT) {
                // Drain error queue for details
                while (xQueueReceive(error_info_queue, &error, 0) == pdTRUE) {
                    handle_limit_error(&error);
                }
            }

            if (notify_bits & NOTIFY_ERROR_ALARM) {
                while (xQueueReceive(error_info_queue, &error, 0) == pdTRUE) {
                    handle_alarm_error(&error);
                }
            }
        }

        // Periodic safety checks
        monitor_watchdogs();
        check_i2c_health();
    }
}

// Handle limit switch error (task context - can do I/O)
static void handle_limit_error(const ErrorInfo* error) {
    uint8_t axis = error->axis;

    // Update axis state
    axis_state[axis].error_code = ERR_POSITION_LIMIT;
    axis_state[axis].can_move_positive = !error->detail.limit.max_triggered;
    axis_state[axis].can_move_negative = !error->detail.limit.min_triggered;

    // Engage brake (via shift register - SPI, OK in task context)
    sr_engage_brake(axis);
    sr_update();

    // Queue notifications to background worker (non-blocking)
    WorkItem work;

    // USB notification
    work.type = WORK_USB_RESPONSE;
    snprintf(work.data.usb.response, sizeof(work.data.usb.response),
             "EVENT LIMIT %c %s%s\n",
             'X' + axis,
             error->detail.limit.min_triggered ? "MIN" : "",
             error->detail.limit.max_triggered ? "MAX" : "");
    background_submit(&work);

    // OLED notification
    work.type = WORK_OLED_UPDATE;
    work.data.oled.line = OLED_EVENT_LINES;
    snprintf(work.data.oled.text, sizeof(work.data.oled.text),
             "LIMIT %c %s", 'X' + axis,
             error->detail.limit.min_triggered ? "MIN" : "MAX");
    background_submit(&work);

    // Log (goes to background worker)
    ESP_LOGW(TAG, "Limit triggered: axis=%c min=%d max=%d",
             'X' + axis,
             error->detail.limit.min_triggered,
             error->detail.limit.max_triggered);
}
```

#### Error Recovery Pattern

```cpp
// Clear error and allow motion (after operator intervention)
esp_err_t clear_axis_error(uint8_t axis) {
    if (axis >= LIMIT_NUM_AXES) return ESP_ERR_INVALID_ARG;

    // Check if safe to clear
    if (system_state == STATE_ESTOP_ACTIVE) {
        return ESP_ERR_INVALID_STATE;  // Must clear E-stop first
    }

    // Check current limit state
    if (axis_state[axis].error_code == ERR_POSITION_LIMIT) {
        bool min_active = !gpio_get_level(limit_min_gpio[axis]);
        bool max_active = !gpio_get_level(limit_max_gpio[axis]);

        if (min_active || max_active) {
            // Still on limit - allow motion away only
            axis_state[axis].can_move_positive = !max_active;
            axis_state[axis].can_move_negative = !min_active;
            ESP_LOGW(TAG, "Axis %c still on limit, restricted motion", 'X' + axis);
        } else {
            // Off limit - full motion allowed
            axis_state[axis].can_move_positive = true;
            axis_state[axis].can_move_negative = true;
        }
    }

    // Clear error state
    axis_state[axis].error_code = 0;

    // Notify
    WorkItem work = { .type = WORK_USB_RESPONSE };
    snprintf(work.data.usb.response, sizeof(work.data.usb.response),
             "EVENT CLR %c\n", 'X' + axis);
    background_submit(&work);

    return ESP_OK;
}
```

## Consistency Rules

### Naming Conventions

**Files:**
```
components/motor/src/servo_motor.cpp    # snake_case for files
components/motor/include/servo_motor.h  # Header matches source
```

**Types and Classes:**
```cpp
class ServoMotor;              // PascalCase for classes
typedef struct axis_config_t;  // snake_case_t for C structs
enum class BrakeStrategy;      // PascalCase for enums
```

**Functions and Variables:**
```cpp
void handleMotionComplete();                   // camelCase for functions
float current_position;                        // snake_case for variables
static const int MAX_AXES = LIMIT_NUM_AXES;    // UPPER_CASE for constants
```

**GPIO and Config Defines:**
```cpp
// Defined in config_gpio.h - see gpio-assignment.md for actual pin numbers
#define GPIO_X_STEP         ...        // GPIO_ prefix for pin names
#define TIMING_DIR_SETUP_US ...        // TIMING_ prefix, value in µs
#define ERR_POSITION_LIMIT  "E005"     // ERR_ prefix for error codes
```

**Axis Naming:**
- Physical: X, Y, Z, A, B, C, D, E (letters)
- Index: 0-7 (array indices)
- Alias: User-defined strings (e.g., "RAILWAY", "GRIPPER")

### Code Organization

**Component Structure:**
```
components/motor/
├── CMakeLists.txt           # Component build config
├── Kconfig                  # Optional menuconfig
├── include/
│   ├── motor.h              # Public API only
│   └── motor_types.h        # Shared type definitions
└── src/
    ├── motor_base.cpp       # Base class implementation
    ├── servo_motor.cpp      # Servo-specific
    ├── stepper_motor.cpp    # Stepper-specific
    └── motor_private.h      # Internal declarations
```

**Header Guards:**
```cpp
#ifndef COMPONENTS_MOTOR_INCLUDE_MOTOR_H_
#define COMPONENTS_MOTOR_INCLUDE_MOTOR_H_
// ...
#endif  // COMPONENTS_MOTOR_INCLUDE_MOTOR_H_
```

### Error Handling

**Return Codes:**
```cpp
// Use ESP-IDF error codes for internal APIs
esp_err_t move_to_position(uint8_t axis, float pos) {
    if (axis >= LIMIT_NUM_AXES) return ESP_ERR_INVALID_ARG;
    if (!is_enabled(axis)) return ESP_ERR_INVALID_STATE;
    if (pos < limits[axis].min) return ESP_ERR_INVALID_SIZE;
    // ...
    return ESP_OK;
}
```

**Error Response Format:**
```
ERROR <code> <message>
ERROR E005 Position limit exceeded
ERROR E006 Emergency stop active
```

**Error Propagation:**
```cpp
// Check and propagate errors
esp_err_t err = move_to_position(axis, target);
if (err != ESP_OK) {
    send_error_response(map_esp_err_to_code(err));
    return err;
}
```

### Logging Strategy

**Log Levels (ESP-IDF standard):**
```cpp
ESP_LOGE(TAG, "Critical error: %s", msg);     // Always shown
ESP_LOGW(TAG, "Warning: axis %d limit", axis); // Operational issues
ESP_LOGI(TAG, "Motion complete: %.2f", pos);   // Normal events
ESP_LOGD(TAG, "Pulse count: %d", count);       // Development
ESP_LOGV(TAG, "ISR entry");                    // Trace (verbose)
```

**Tag Naming:**
```cpp
static const char* TAG = "motor";     // Component name
static const char* TAG = "safety";    // Functional area
static const char* TAG = "usb_cdc";   // Specific module
```

**Runtime Log Control:**
```cpp
// LOG command adjusts level
// LOG NONE | ERROR | WARN | INFO | DEBUG
esp_log_level_set("*", ESP_LOG_INFO);        // Default
esp_log_level_set("motor", ESP_LOG_DEBUG);   // Specific component
```

**Performance Logging:**
```cpp
// Use ESP_EARLY_LOG* for ISR-safe logging (limited)
// Never log from ISR in production - use queued events
void IRAM_ATTR some_isr(void* arg) {
    // DON'T: ESP_LOGI(TAG, "ISR triggered");
    // DO: Queue event for task logging
    xQueueSendFromISR(log_queue, &event, &woken);
}
```

## Compile-Time Configuration

All hardware assignments, timing constants, buffer sizes, and system parameters are defined in header files. **No magic numbers in source code.**

### MANDATORY: Header-Only Configuration Requirement

> **⚠️ ARCHITECTURE CONSTRAINT - APPLIES TO ALL EPICS AND STORIES**
>
> **Every configurable value MUST be defined in a header file.** This is non-negotiable.
>
> - **GPIO pins**: Use `GPIO_X_STEP`, never `GPIO_NUM_2`
> - **Timing values**: Use `TIMING_DIR_SETUP_US`, never `20`
> - **Buffer sizes**: Use `LIMIT_CMD_MAX_LENGTH`, never `256`
> - **Command strings**: Use `CMD_MOVE`, never `"MOVE"`
> - **Error codes**: Use `ERR_INVALID_AXIS`, never `"E002"`
> - **YAML keys**: Use `YAML_KEY_VELOCITY`, never `"velocity"`
> - **I2C addresses**: Use `I2C_ADDR_MCP23017_0`, never `0x20`
>
> **Validation Rule**: Code review MUST reject any:
> - Literal GPIO numbers in driver code
> - Hardcoded timing values
> - Magic numbers for buffer sizes or limits
> - String literals for commands, errors, or YAML keys
>
> **This requirement MUST be included in every epic and story context file.**

### Core Concepts

**1. Single Source of Truth**
Each configuration category lives in exactly one header file. GPIO pins are defined only in `config_gpio.h`, never scattered across driver files. When you need to change a pin, you change it in one place.

**2. Separation of Concerns**
Configuration files are organized by domain:
- **Hardware mapping** (`config_gpio.h`, `config_i2c.h`) — What physical pin connects to what
- **Peripheral assignment** (`config_peripherals.h`) — Which ESP32 peripheral handles which function
- **Behavior tuning** (`config_timing.h`, `config_limits.h`) — How the system behaves
- **Protocol definition** (`config_commands.h`) — The external interface
- **Default values** (`config_defaults.h`) — Initial runtime parameters

**3. Compile-Time vs Runtime Configuration**
| Compile-Time (headers) | Runtime (NVS/YAML) |
|------------------------|-------------------|
| GPIO assignments | Axis units, limits |
| Peripheral channels | Velocity, acceleration |
| Buffer sizes | Backlash compensation |
| Command syntax | Axis aliases |
| Error codes | System behavior |
| Feature flags | User preferences |

Compile-time = hardware/firmware structure (requires rebuild).
Runtime = operational parameters (changeable via commands).

**4. Self-Documenting Code**
Configuration defines serve as inline documentation:
```c
// Instead of hardcoded pin numbers:
gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);  // What is this pin for?

// Always use named constants from config headers:
gpio_set_direction(GPIO_X_STEP, GPIO_MODE_OUTPUT);  // Clear: X-axis step output
```

**5. Build-Time Validation**
Grouped defines enable compile-time checks:
```c
// config_limits.h
#define LIMIT_NUM_AXES          8
#define LIMIT_NUM_SERVOS        5
#define LIMIT_NUM_STEPPERS      2
#define LIMIT_NUM_DISCRETE      1

// In code:
static_assert(LIMIT_NUM_SERVOS + LIMIT_NUM_STEPPERS + LIMIT_NUM_DISCRETE == LIMIT_NUM_AXES,
              "Axis count mismatch!");
```

**6. Easy Hardware Variants**
To create a variant with different GPIO assignments:
1. Copy `config_gpio.h` to `config_gpio_variant.h`
2. Modify pin assignments
3. Include variant header in `config.h`
4. Rebuild

No source code changes needed — only configuration.

**7. Feature Flags**
Optional features compile in/out cleanly:
```c
#define FEATURE_OLED_DISPLAY    0   // Set to 1 to enable

// In code:
#if FEATURE_OLED_DISPLAY
    initOledDisplay();
    xTaskCreate(displayUpdateTask, "display", STACK_DISPLAY_TASK, ...);
#endif
```

### Configuration Header Structure (Complete Tree)

```
components/config/include/
│
├── config.h                          # Master include
│   ├── FIRMWARE_NAME                 # "YAROBOT_CONTROL_UNIT"
│   ├── FIRMWARE_VERSION_*            # Major, Minor, Patch, String
│   └── FEATURE_*                     # Feature flags (OLED, Z_SIGNAL, STREAMING)
│
├── config_gpio.h                     # GPIO pin assignments (grouped by AXIS)
│   │
│   ├── Servo Axes (STEP + Z_SIGNAL paired on same board row)
│   │   ├── X Axis (row 4)
│   │   │   ├── GPIO_X_STEP           # J1-4 (RMT CH0)
│   │   │   └── GPIO_X_Z_SIGNAL       # J3-4 (index pulse)
│   │   ├── Y Axis (row 5)
│   │   │   ├── GPIO_Y_STEP           # J1-5 (MCPWM T0)
│   │   │   └── GPIO_Y_Z_SIGNAL       # J3-5 (index pulse)
│   │   ├── Z Axis (row 6)
│   │   │   ├── GPIO_Z_STEP           # J1-6 (RMT CH1)
│   │   │   └── GPIO_Z_Z_SIGNAL       # J3-6 (index pulse)
│   │   ├── A Axis (row 7)
│   │   │   ├── GPIO_A_STEP           # J1-7 (RMT CH2)
│   │   │   └── GPIO_A_Z_SIGNAL       # J3-7 (index pulse)
│   │   └── B Axis (row 8)
│   │       ├── GPIO_B_STEP           # J1-8 (RMT CH3)
│   │       └── GPIO_B_Z_SIGNAL       # J3-8 (index pulse)
│   │
│   ├── Stepper Axes (no Z-signal)
│   │   ├── GPIO_C_STEP               # J1-9 (MCPWM T1)
│   │   └── GPIO_D_STEP               # J1-10 (LEDC CH0)
│   │
│   ├── E Axis                        # Discrete - DIR/EN/BRAKE/ALARM_CLR via shift register (bits 28-31)
│   │
│   ├── InPos Signals                 # NOT GPIO! Via MCP23017 #1 (0x21) Port B
│   │
│   ├── Shift Register (SPI2) - J1 pins 15-18
│   │   ├── GPIO_SR_OE                # Output enable (active-low)
│   │   ├── GPIO_SR_CS                # Latch
│   │   ├── GPIO_SR_MOSI              # Data
│   │   └── GPIO_SR_SCLK              # Clock
│   │
│   ├── I2C Bus 0 (MCP23017s) - J1 pins 11-12
│   │   ├── GPIO_I2C_SCL              # Clock (400kHz)
│   │   └── GPIO_I2C_SDA              # Data
│   │
│   ├── MCP23017 Interrupts (grouped: INTA left side, INTB right side)
│   │   ├── GPIO_MCP0_INTA            # MCP #0 Port A (X-A limits) - J1-13
│   │   ├── GPIO_MCP1_INTA            # MCP #1 Port A (ALARM_INPUT) - J1-14
│   │   ├── GPIO_MCP0_INTB            # MCP #0 Port B (B-E limits) - J3-9
│   │   └── GPIO_MCP1_INTB            # MCP #1 Port B (InPos signals) - J3-10
│   │
│   ├── Safety
│   │   └── GPIO_E_STOP               # Emergency stop input (J1-19)
│   │
│   ├── OLED I2C Bus 1 (isolated)
│   │   ├── GPIO_OLED_SDA             # J1-20
│   │   └── GPIO_OLED_SCL             # J3-18
│   │
│   └── USB (Fixed - do not change)
│       ├── GPIO_USB_DN               # Native USB D-
│       └── GPIO_USB_DP               # Native USB D+
│
├── config_peripherals.h              # ESP32 peripheral assignments
│   ├── RMT Channels
│   │   ├── RMT_CHANNEL_X             # 0
│   │   ├── RMT_CHANNEL_Z             # 1
│   │   ├── RMT_CHANNEL_A             # 2
│   │   └── RMT_CHANNEL_B             # 3
│   │
│   ├── MCPWM
│   │   ├── MCPWM_GROUP_ID            # 0
│   │   ├── MCPWM_TIMER_Y             # 0
│   │   └── MCPWM_TIMER_C             # 1
│   │
│   ├── PCNT (Pulse Counter)
│   │   ├── PCNT_UNIT_Y               # 0
│   │   └── PCNT_UNIT_C               # 1
│   │
│   ├── LEDC
│   │   ├── LEDC_TIMER                # LEDC_TIMER_0
│   │   ├── LEDC_CHANNEL_D            # LEDC_CHANNEL_0
│   │   └── LEDC_MODE                 # LEDC_LOW_SPEED_MODE
│   │
│   └── SPI
│       └── SPI_HOST_SR               # SPI2_HOST
│
├── config_timing.h                   # Timing constants (all in ms/µs)
│   ├── Motion Timing
│   │   ├── TIMING_DIR_SETUP_US       # 20    - Direction setup
│   │   ├── TIMING_ENABLE_DELAY_US    # 50    - Enable to motion
│   │   ├── TIMING_BRAKE_ENGAGE_MS    # 50    - Brake engage time
│   │   └── TIMING_BRAKE_RELEASE_MS   # 30    - Brake release time
│   │
│   ├── Communication Timing
│   │   ├── TIMING_CMD_RESPONSE_MS    # 10    - Max response time
│   │   └── TIMING_USB_RX_TIMEOUT_MS  # 100   - USB receive timeout
│   │
│   ├── I2C Timing
│   │   ├── TIMING_I2C_POLL_MS        # 5     - Polling interval
│   │   ├── TIMING_I2C_TIMEOUT_MS     # 50    - Transaction timeout
│   │   └── TIMING_I2C_RETRY_COUNT    # 3     - Retry attempts
│   │
│   ├── Safety Timing
│   │   ├── TIMING_ESTOP_DEBOUNCE_MS  # 5     - E-stop debounce
│   │   ├── TIMING_LIMIT_DEBOUNCE_MS  # 10    - Limit switch debounce
│   │   ├── TIMING_IDLE_TIMEOUT_S     # 300   - Motor idle timeout
│   │   └── TIMING_SAFETY_POLL_MS     # 10    - Safety task poll interval
│   │
│   └── Task Periods
│       ├── PERIOD_I2C_MONITOR_MS     # 100   - I2C health check
│       ├── PERIOD_DISPLAY_UPDATE_MS  # 100   - OLED update
│       └── PERIOD_IDLE_CHECK_MS      # 1000  - Idle timeout check
│
├── config_limits.h                   # Buffer sizes, queue depths
│   ├── Buffer Sizes
│   │   ├── LIMIT_CMD_MAX_LENGTH      # 256   - Command line max
│   │   ├── LIMIT_RESPONSE_MAX_LENGTH # 256   - Response max
│   │   ├── LIMIT_YAML_BUFFER_SIZE    # 8192  - YAML buffer (8KB)
│   │   └── LIMIT_ALIAS_MAX_LENGTH    # 16    - Axis alias max
│   │
│   ├── Queue Depths
│   │   ├── LIMIT_COMMAND_QUEUE_DEPTH # 32
│   │   ├── LIMIT_RESPONSE_QUEUE_DEPTH# 32
│   │   ├── LIMIT_SAFETY_QUEUE_DEPTH  # 64
│   │   ├── LIMIT_EVENT_QUEUE_DEPTH   # 32
│   │   ├── LIMIT_BACKGROUND_QUEUE_DEPTH # 32  - Background work items
│   │   └── LIMIT_ERROR_INFO_QUEUE_DEPTH # 16  - ISR error details
│   │
│   ├── Axis Limits
│   │   ├── LIMIT_NUM_AXES            # 8     - Total axes
│   │   ├── LIMIT_NUM_SERVOS          # 5     - Servo count
│   │   ├── LIMIT_NUM_STEPPERS        # 2     - Stepper count
│   │   ├── LIMIT_NUM_DISCRETE        # 1     - Discrete count
│   │   └── LIMIT_CONFIG_SLOTS        # 10    - NVS slots
│   │
│   ├── Motion Limits
│   │   ├── LIMIT_MAX_PULSE_FREQ_HZ   # 100000 (100kHz)
│   │   ├── LIMIT_MIN_PULSE_FREQ_HZ   # 1
│   │   └── LIMIT_MOTION_QUEUE_DEPTH  # 4
│   │
│   └── Stack Sizes (words)
│       ├── STACK_SAFETY_TASK         # 4096
│       ├── STACK_USB_RX_TASK         # 2048
│       ├── STACK_USB_TX_TASK         # 2048
│       ├── STACK_CMD_EXECUTOR_TASK   # 8192
│       ├── STACK_I2C_MONITOR_TASK    # 2048
│       ├── STACK_MOTION_TASK         # 2048
│       ├── STACK_DISPLAY_TASK        # 4096
│       ├── STACK_IDLE_MONITOR_TASK   # 2048
│       └── STACK_BACKGROUND_TASK     # 4096  - Background worker
│
├── config_commands.h                 # Protocol strings & error codes
│   ├── Command Strings
│   │   ├── CMD_MOVE, CMD_MOVR, CMD_VEL, CMD_STOP
│   │   ├── CMD_POS, CMD_STAT, CMD_INFO
│   │   ├── CMD_EN, CMD_BRAKE, CMD_HOME, CMD_ZERO, CMD_CALB
│   │   ├── CMD_GETWIDTH, CMD_SETU, CMD_SETL, CMD_SETV, CMD_SETB, CMD_ALIAS
│   │   ├── CMD_DIN, CMD_DOUT, CMD_MODE, CMD_CLR
│   │   ├── CMD_SAVE, CMD_LOAD, CMD_RST, CMD_ECHO
│   │   ├── CMD_TEST, CMD_LOG, CMD_DIAG, CMD_STREAM
│   │   └── CMD_CFGSTART, CMD_CFGDATA, CMD_CFGEND, CMD_CFGEXPORT
│   │
│   ├── Response Prefixes
│   │   ├── RESP_OK                   # "OK"
│   │   ├── RESP_ERROR                # "ERROR"
│   │   ├── RESP_EVENT                # "EVENT"
│   │   ├── RESP_YAML                 # "YAML:"
│   │   └── RESP_STREAM               # "STRM"
│   │
│   ├── Error Codes
│   │   ├── ERR_INVALID_COMMAND       # "E001"
│   │   ├── ERR_INVALID_AXIS          # "E002"
│   │   ├── ERR_INVALID_PARAMETER     # "E003"
│   │   ├── ERR_AXIS_NOT_ENABLED      # "E004"
│   │   ├── ERR_POSITION_LIMIT        # "E005"
│   │   ├── ERR_EMERGENCY_STOP        # "E006"
│   │   ├── ERR_CALIBRATION_REQUIRED  # "E007"
│   │   ├── ERR_MOTOR_FAULT           # "E008"
│   │   ├── ERR_COMMUNICATION         # "E009"
│   │   ├── ERR_CONFIGURATION         # "E010"
│   │   ├── ERR_EVENT_OVERFLOW        # "E011"
│   │   ├── ERR_MODE_BLOCKED          # "E012"
│   │   └── ERR_MOTION_ACTIVE         # "E013"
│   │
│   └── Error Messages
│       └── MSG_* (corresponding message strings)
│
├── config_i2c.h                      # I2C addresses & pin mappings
│   ├── Bus Configuration
│   │   ├── I2C_PORT                  # I2C_NUM_0
│   │   └── I2C_FREQ_HZ               # 400000 (400kHz)
│   │
│   ├── Device Addresses
│   │   ├── I2C_ADDR_MCP23017_0       # 0x20 - Limit switches (inputs only)
│   │   ├── I2C_ADDR_MCP23017_1       # 0x21 - ALARM_INPUT + InPos (inputs only)
│   │   └── I2C_ADDR_SSD1306          # 0x3C - OLED display
│   │
│   ├── MCP23017 #0 Pin Mappings (Limit Switches)
│   │   ├── MCP0_X_LIMIT_MIN/MAX      # 0,1   - X axis
│   │   ├── MCP0_Y_LIMIT_MIN/MAX      # 2,3   - Y axis
│   │   ├── MCP0_Z_LIMIT_MIN/MAX      # 4,5   - Z axis
│   │   ├── MCP0_A_LIMIT_MIN/MAX      # 6,7   - A axis
│   │   ├── MCP0_B_LIMIT_MIN/MAX      # 8,9   - B axis
│   │   ├── MCP0_C_LIMIT_MIN/MAX      # 10,11 - C axis (floating switch)
│   │   ├── MCP0_D_LIMIT_MIN/MAX      # 12,13 - D axis
│   │   └── MCP0_E_LIMIT_MIN/MAX      # 14,15 - E axis
│   │
│   └── MCP23017 #1 Pin Mappings (ALARM_INPUT + InPos)
│       ├── MCP1_X/Y/Z/A/B/C/D_ALARM_INPUT  # 0-6  - ALARM_INPUT (Port A)
│       ├── MCP1_GP_IN_0              # 7     - Spare input (Port A)
│       ├── MCP1_X/Y/Z/A/B_INPOS      # 8-12  - InPos signals (Port B)
│       └── MCP1_GP_IN_1..3           # 13-15 - Spare inputs (Port B)
│
├── config_defaults.h                 # Default runtime parameters (SI UNITS)
│   ├── Generic Defaults (Linear Axis)
│   │   ├── DEFAULT_PULSES_PER_UNIT   # 1000000.0f (1 million pulses/meter = 1µm)
│   │   ├── DEFAULT_IS_ROTARY         # false
│   │   ├── DEFAULT_LIMIT_MIN         # -1.0f (meters)
│   │   ├── DEFAULT_LIMIT_MAX         # 1.0f (meters)
│   │   ├── DEFAULT_MAX_VELOCITY      # 0.1f (m/s = 100mm/s)
│   │   ├── DEFAULT_MAX_ACCELERATION  # 1.0f (m/s²)
│   │   ├── DEFAULT_BACKLASH          # 0.0f (meters)
│   │   └── DEFAULT_HOME_OFFSET       # 0.0f (meters)
│   │
│   ├── Rotary Axis Defaults
│   │   ├── DEFAULT_ROTARY_PULSES_PER_UNIT  # 10000.0f (pulses/radian)
│   │   ├── DEFAULT_ROTARY_LIMIT_MIN  # 0.0f (radians)
│   │   ├── DEFAULT_ROTARY_LIMIT_MAX  # 6.283185f (2π radians)
│   │   ├── DEFAULT_ROTARY_MAX_VEL    # 3.14159f (π rad/s = 180°/s)
│   │   └── DEFAULT_ROTARY_MAX_ACCEL  # 10.0f (rad/s²)
│   │
│   └── E-Axis Specific (discrete)
│       ├── E_AXIS_PULSES_PER_UNIT    # 1.0f (binary: 0 or 1)
│       ├── E_AXIS_MAX_VELOCITY       # 1.0f
│       ├── E_AXIS_LIMIT_MIN          # 0.0f
│       └── E_AXIS_LIMIT_MAX          # 1.0f
│
├── config_oled.h                     # OLED display configuration
│   ├── OLED I2C Bus (dedicated, separate from main I2C)
│   │   ├── I2C_OLED_PORT             # I2C_NUM_1
│   │   ├── I2C_OLED_FREQ_HZ          # 400000 (400kHz)
│   │   ├── GPIO_OLED_SDA             # (defined in config_gpio.h)
│   │   └── GPIO_OLED_SCL             # (defined in config_gpio.h)
│   │
│   └── Display Parameters
│       ├── OLED_ADDRESS              # 0x3C
│       ├── OLED_WIDTH                # 128
│       ├── OLED_HEIGHT               # 64
│       └── OLED_UPDATE_HZ            # 2 (2Hz update rate)
│
├── config_sr.h                       # Shift register bit positions (5x TPIC6B595N = 40 bits)
│   ├── Per-Axis Bit Positions (4 bits per axis: DIR, EN, BRAKE, ALARM_CLR)
│   │   ├── SR_X_DIR, SR_X_EN, SR_X_BRAKE, SR_X_ALARM_CLR     # Bits 0-3
│   │   ├── SR_Y_DIR, SR_Y_EN, SR_Y_BRAKE, SR_Y_ALARM_CLR     # Bits 4-7
│   │   ├── SR_Z_DIR, SR_Z_EN, SR_Z_BRAKE, SR_Z_ALARM_CLR     # Bits 8-11
│   │   ├── SR_A_DIR, SR_A_EN, SR_A_BRAKE, SR_A_ALARM_CLR     # Bits 12-15
│   │   ├── SR_B_DIR, SR_B_EN, SR_B_BRAKE, SR_B_ALARM_CLR     # Bits 16-19
│   │   ├── SR_C_DIR, SR_C_EN, SR_C_BRAKE, SR_C_ALARM_CLR     # Bits 20-23
│   │   ├── SR_D_DIR, SR_D_EN, SR_D_BRAKE, SR_D_ALARM_CLR     # Bits 24-27
│   │   └── SR_E_DIR, SR_E_EN, SR_E_BRAKE, SR_E_ALARM_CLR     # Bits 28-31
│   │
│   ├── General Purpose Outputs (SR4)
│   │   └── SR_GP_OUT_0..7            # Bits 32-39
│   │
│   ├── Helper Macros
│   │   ├── SR_DIR_BIT(axis)          # Get DIR bit for axis (axis * 4 + 0)
│   │   ├── SR_EN_BIT(axis)           # Get EN bit for axis (axis * 4 + 1)
│   │   ├── SR_BRAKE_BIT(axis)        # Get BRAKE bit for axis (axis * 4 + 2)
│   │   ├── SR_ALARM_CLR_BIT(axis)    # Get ALARM_CLR bit for axis (axis * 4 + 3)
│   │   ├── SR_SET_BIT(data, bit)     # Set bit in data (uint64_t)
│   │   └── SR_CLR_BIT(data, bit)     # Clear bit in data (uint64_t)
│   │
│   └── Fail-Safe Defaults
│       └── SR_SAFE_STATE             # 0x0000000000ULL (all brakes engaged)
│
└── config_yaml_schema.h              # YAML configuration schema & validation
    ├── Schema Version
    │   └── YAML_SCHEMA_VERSION       # 1
    │
    ├── Root Keys
    │   ├── YAML_KEY_VERSION          # "version"
    │   ├── YAML_KEY_AXES             # "axes"
    │   └── YAML_KEY_SYSTEM           # "system"
    │
    ├── Axis Names (section keys)
    │   └── YAML_AXIS_X..E            # "X", "Y", "Z", "A", "B", "C", "D", "E"
    │
    ├── Per-Axis Parameter Keys
    │   ├── YAML_KEY_ALIAS            # "alias"
    │   ├── YAML_KEY_TYPE             # "type" (linear/rotary)
    │   ├── YAML_KEY_PULSES_PER_REV   # "pulses_per_rev" (driver PA14 or microsteps)
    │   ├── YAML_KEY_UNITS_PER_REV    # "units_per_rev" (meters or radians per revolution)
    │   ├── YAML_KEY_LIMITS           # "limits"
    │   ├── YAML_KEY_MAX_VELOCITY     # "max_velocity"
    │   ├── YAML_KEY_MAX_ACCELERATION # "max_acceleration"
    │   ├── YAML_KEY_BACKLASH         # "backlash"
    │   ├── YAML_KEY_HOME_OFFSET      # "home_offset"
    │   ├── YAML_KEY_HOMING           # "homing" (nested: velocity, velocity_slow, backoff, direction)
    │   └── YAML_KEY_ZSIGNAL          # "z_signal" (nested: enabled, drift_threshold)
    │
    ├── System Parameter Keys
    │   ├── YAML_KEY_DEBOUNCE_MS      # "debounce_ms"
    │   ├── YAML_KEY_IDLE_TIMEOUT_S   # "idle_timeout_s"
    │   └── YAML_KEY_LOG_LEVEL        # "log_level"
    │
    ├── NVS Key Mappings
    │   ├── NVS_KEY_PREFIX_AXIS       # "ax" (ax0_, ax1_, etc.)
    │   ├── NVS_KEY_SUFFIX_PPR        # "_ppr" (pulses_per_rev)
    │   ├── NVS_KEY_SUFFIX_UPR        # "_upr" (units_per_rev)
    │   ├── NVS_KEY_SUFFIX_*          # _rot, _min, _max, _vel, _acc, etc.
    │   ├── NVS_KEY_SUFFIX_ZSIG_EN    # "_zsen" (z_signal_enabled)
    │   ├── NVS_KEY_SUFFIX_ZSIG_THR   # "_zsth" (z_drift_threshold)
    │   └── NVS_KEY_SYS_*             # sys_version, sys_slot, sys_debounce
    │
    └── Validation Macros
        ├── YAML_PULSES_PER_UNIT      # Derived from ppr/upr
        ├── YAML_MAX_VELOCITY         # Computed from LIMIT_MAX_PULSE_FREQ_HZ / ppu
        ├── YAML_MIN_VELOCITY         # Computed from LIMIT_MIN_PULSE_FREQ_HZ / ppu
        ├── YAML_VALIDATE_PPR         # pulses_per_rev validation
        ├── YAML_VALIDATE_UPR         # units_per_rev validation
        └── YAML_VALIDATE_*           # Runtime validation helpers
```

### config_gpio.h — Pin Assignments

> **IMPORTANT:** For complete GPIO assignment details including physical board layout,
> shift register bit mapping, and MCP23017 pin assignments, see:
> **[GPIO Assignment Document](./gpio-assignment.md)**

```c
#ifndef CONFIG_GPIO_H
#define CONFIG_GPIO_H

// ============================================================================
// MOTOR STEP PULSE OUTPUTS - GROUPED BY AXIS
// ============================================================================
// All STEP outputs on LEFT side (J1), paired with Z_SIGNAL on RIGHT side (J3)
// Same row number = same physical position on board for clean cabling

// Servo axes (X, Y, Z, A, B) - STEP on left, Z_SIGNAL on right (same row)
#define GPIO_X_STEP         GPIO_NUM_4      // X-axis servo (RMT CH0) - J1-4 (row 4)
#define GPIO_Y_STEP         GPIO_NUM_5      // Y-axis servo (MCPWM T0) - J1-5 (row 5)
#define GPIO_Z_STEP         GPIO_NUM_6      // Z-axis servo (RMT CH1) - J1-6 (row 6)
#define GPIO_A_STEP         GPIO_NUM_7      // A-axis servo (RMT CH2) - J1-7 (row 7)
#define GPIO_B_STEP         GPIO_NUM_15     // B-axis servo (RMT CH3) - J1-8 (row 8)

// Stepper axes (C, D) - no Z-signal
#define GPIO_C_STEP         GPIO_NUM_16     // C-axis stepper (MCPWM T1) - J1-9
#define GPIO_D_STEP         GPIO_NUM_17     // D-axis stepper (LEDC CH0) - J1-10

// E-axis: DIR/EN via shift register (bits 21-22), no STEP GPIO needed (discrete actuator)

// ============================================================================
// SERVO Z-SIGNAL (INDEX) INPUTS - PAIRED WITH STEP OUTPUTS
// ============================================================================
// All Z_SIGNAL inputs on RIGHT side (J3), same row as corresponding STEP on left
// Hardware interrupts required for precise encoder index capture

#define GPIO_X_Z_SIGNAL     GPIO_NUM_1      // X servo index pulse - J3-4 (row 4)
#define GPIO_Y_Z_SIGNAL     GPIO_NUM_2      // Y servo index pulse - J3-5 (row 5)
#define GPIO_Z_Z_SIGNAL     GPIO_NUM_42     // Z servo index pulse - J3-6 (row 6)
#define GPIO_A_Z_SIGNAL     GPIO_NUM_41     // A servo index pulse - J3-7 (row 7)
#define GPIO_B_Z_SIGNAL     GPIO_NUM_40     // B servo index pulse - J3-8 (row 8)

// ============================================================================
// INPOS SIGNALS - VIA I2C EXPANDER (NOT GPIO!)
// ============================================================================
// InPos (Position Complete) signals are read via MCP23017 #1 (0x21) Port B
// They don't require fast interrupt response - I2C polling is sufficient
// See config_i2c.h for MCP23017 #1 pin assignments

// ============================================================================
// SHIFT REGISTER CONTROL (SPI2/HSPI)
// ============================================================================
// Grouped on left side (J1 pins 15-18)
#define GPIO_SR_OE          GPIO_NUM_9      // Output enable (active-low) - J1-15
#define GPIO_SR_CS          GPIO_NUM_10     // Shift register latch - J1-16
#define GPIO_SR_MOSI        GPIO_NUM_11     // Shift register data - J1-17
#define GPIO_SR_SCLK        GPIO_NUM_12     // Shift register clock - J1-18

// ============================================================================
// I2C BUS 0 (MCP23017 Expanders)
// ============================================================================
// I2C lines grouped on left side (J1 pins 11-12)
#define GPIO_I2C_SCL        GPIO_NUM_18     // I2C clock - J1-11
#define GPIO_I2C_SDA        GPIO_NUM_8      // I2C data - J1-12

// ============================================================================
// MCP23017 INTERRUPT LINES (4 GPIOs for 2 MCPs × 2 interrupts each)
// ============================================================================
// Grouped by interrupt type: INTA on left side (J1), INTB on right side (J3)
// Both MCPs are input-only, so all 4 interrupt lines are active

// INTA interrupts - grouped on LEFT side (J1-13, J1-14)
#define GPIO_MCP0_INTA      GPIO_NUM_3      // MCP #0 Port A (X-A limits) - J1-13
#define GPIO_MCP1_INTA      GPIO_NUM_46     // MCP #1 Port A (ALARM_INPUT) - J1-14

// INTB interrupts - grouped on RIGHT side (J3-9, J3-10)
#define GPIO_MCP0_INTB      GPIO_NUM_39     // MCP #0 Port B (B-E limits) - J3-9
#define GPIO_MCP1_INTB      GPIO_NUM_38     // MCP #1 Port B (InPos signals) - J3-10

// ============================================================================
// SAFETY SIGNALS
// ============================================================================
#define GPIO_E_STOP         GPIO_NUM_13     // Emergency stop input - J1-19

// ============================================================================
// OLED I2C BUS (Dedicated bus, isolated from main I2C)
// ============================================================================
#define GPIO_OLED_SDA       GPIO_NUM_14     // OLED I2C data (I2C_NUM_1) - J1-20
#define GPIO_OLED_SCL       GPIO_NUM_21     // OLED I2C clock (I2C_NUM_1) - J3-18

// ============================================================================
// USB (Fixed by hardware - DO NOT CHANGE)
// ============================================================================
#define GPIO_USB_DN         GPIO_NUM_19     // USB D- - J3-20
#define GPIO_USB_DP         GPIO_NUM_20     // USB D+ - J3-19

#endif // CONFIG_GPIO_H
```

### config_peripherals.h — Peripheral Assignments

```c
#ifndef CONFIG_PERIPHERALS_H
#define CONFIG_PERIPHERALS_H

// ============================================================================
// RMT CHANNEL ASSIGNMENTS
// ============================================================================
#define RMT_CHANNEL_X       0               // X-axis RMT channel
#define RMT_CHANNEL_Z       1               // Z-axis RMT channel
#define RMT_CHANNEL_A       2               // A-axis RMT channel
#define RMT_CHANNEL_B       3               // B-axis RMT channel

// ============================================================================
// MCPWM ASSIGNMENTS
// ============================================================================
#define MCPWM_GROUP_ID      0               // MCPWM group
#define MCPWM_TIMER_Y       0               // Y-axis timer
#define MCPWM_TIMER_C       1               // C-axis timer

// ============================================================================
// PCNT (Pulse Counter) ASSIGNMENTS
// ============================================================================
#define PCNT_UNIT_Y         0               // Y-axis pulse counter
#define PCNT_UNIT_C         1               // C-axis pulse counter

// ============================================================================
// LEDC ASSIGNMENTS
// ============================================================================
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_CHANNEL_D      LEDC_CHANNEL_0  // D-axis LEDC channel
#define LEDC_MODE           LEDC_LOW_SPEED_MODE

// ============================================================================
// SPI ASSIGNMENTS
// ============================================================================
#define SPI_HOST_SR         SPI2_HOST       // Shift register SPI host

#endif // CONFIG_PERIPHERALS_H
```

### config_timing.h — Timing Constants

```c
#ifndef CONFIG_TIMING_H
#define CONFIG_TIMING_H

// ============================================================================
// MOTION TIMING
// ============================================================================
#define TIMING_DIR_SETUP_US         20      // Direction setup time (µs)
#define TIMING_ENABLE_DELAY_US      50      // Enable to motion delay (µs)
#define TIMING_BRAKE_ENGAGE_MS      50      // Brake engagement time (ms)
#define TIMING_BRAKE_RELEASE_MS     30      // Brake release time (ms)

// ============================================================================
// COMMUNICATION TIMING
// ============================================================================
#define TIMING_CMD_RESPONSE_MS      10      // Max command response time (ms)
#define TIMING_USB_RX_TIMEOUT_MS    100     // USB receive timeout (ms)

// ============================================================================
// I2C TIMING
// ============================================================================
#define TIMING_I2C_POLL_MS          5       // I2C polling interval (ms)
#define TIMING_I2C_TIMEOUT_MS       50      // I2C transaction timeout (ms)
#define TIMING_I2C_RETRY_COUNT      3       // I2C retry attempts

// ============================================================================
// SAFETY TIMING
// ============================================================================
#define TIMING_ESTOP_DEBOUNCE_MS    5       // E-stop debounce time (ms)
#define TIMING_LIMIT_DEBOUNCE_MS    10      // Limit switch debounce (ms)
#define TIMING_IDLE_TIMEOUT_S       300     // Motor idle timeout (seconds, 0=disabled)
#define TIMING_SAFETY_POLL_MS       10      // Safety task poll interval (ms)

// ============================================================================
// TASK PERIODS
// ============================================================================
#define PERIOD_I2C_MONITOR_MS       100     // I2C health check period (ms)
#define PERIOD_DISPLAY_UPDATE_MS    100     // OLED update period (ms)
#define PERIOD_IDLE_CHECK_MS        1000    // Idle timeout check period (ms)

#endif // CONFIG_TIMING_H
```

### config_limits.h — Buffer Sizes and Limits

```c
#ifndef CONFIG_LIMITS_H
#define CONFIG_LIMITS_H

// ============================================================================
// BUFFER SIZES
// ============================================================================
#define LIMIT_CMD_MAX_LENGTH        256     // Max command line length
#define LIMIT_RESPONSE_MAX_LENGTH   256     // Max response line length
#define LIMIT_YAML_BUFFER_SIZE      8192    // YAML receive buffer (8KB) - ERROR E030 if exceeded
#define LIMIT_ALIAS_MAX_LENGTH      16      // Max axis alias length

// ============================================================================
// QUEUE DEPTHS
// ============================================================================
#define LIMIT_COMMAND_QUEUE_DEPTH   32      // Command queue size
#define LIMIT_RESPONSE_QUEUE_DEPTH  32      // Response queue size
#define LIMIT_SAFETY_QUEUE_DEPTH    64      // Safety event queue size
#define LIMIT_EVENT_QUEUE_DEPTH     32      // Event queue size
#define LIMIT_BACKGROUND_QUEUE_DEPTH 32     // Background work queue size
#define LIMIT_ERROR_INFO_QUEUE_DEPTH 16     // ISR error detail queue size

// ============================================================================
// AXIS LIMITS
// ============================================================================
#define LIMIT_NUM_AXES              8       // Total number of axes (X-E)
#define LIMIT_NUM_SERVOS            5       // Servo axes (X,Y,Z,A,B)
#define LIMIT_NUM_STEPPERS          2       // Stepper axes (C,D)
#define LIMIT_NUM_DISCRETE          1       // Discrete axes (E)
#define LIMIT_CONFIG_SLOTS          10      // NVS configuration slots (0-9)

// ============================================================================
// MOTION LIMITS
// ============================================================================
#define LIMIT_MAX_PULSE_FREQ_HZ     100000  // Maximum pulse frequency (100kHz)
#define LIMIT_MIN_PULSE_FREQ_HZ     1       // Minimum pulse frequency (1Hz)
#define LIMIT_MOTION_QUEUE_DEPTH    4       // Motion commands per axis

// ============================================================================
// STACK SIZES (words, not bytes)
// ============================================================================
#define STACK_SAFETY_TASK           4096
#define STACK_USB_RX_TASK           2048
#define STACK_USB_TX_TASK           2048
#define STACK_CMD_EXECUTOR_TASK     8192
#define STACK_I2C_MONITOR_TASK      2048
#define STACK_MOTION_TASK           2048
#define STACK_DISPLAY_TASK          4096
#define STACK_IDLE_MONITOR_TASK     2048
#define STACK_BACKGROUND_TASK       4096    // Background worker task

#endif // CONFIG_LIMITS_H
```

### config_commands.h — Command Strings and Error Codes

```c
#ifndef CONFIG_COMMANDS_H
#define CONFIG_COMMANDS_H

// ============================================================================
// COMMAND STRINGS
// ============================================================================
#define CMD_MOVE            "MOVE"
#define CMD_MOVR            "MOVR"
#define CMD_VEL             "VEL"
#define CMD_STOP            "STOP"
#define CMD_POS             "POS"
#define CMD_STAT            "STAT"
#define CMD_INFO            "INFO"
#define CMD_EN              "EN"
#define CMD_BRAKE           "BRAKE"
#define CMD_HOME            "HOME"
#define CMD_ZERO            "ZERO"
#define CMD_CALB            "CALB"
#define CMD_GETWIDTH        "GETWIDTH"
#define CMD_SETU            "SETU"
#define CMD_SETL            "SETL"
#define CMD_SETV            "SETV"
#define CMD_SETB            "SETB"
#define CMD_ALIAS           "ALIAS"
#define CMD_DIN             "DIN"
#define CMD_DOUT            "DOUT"
#define CMD_MODE            "MODE"
#define CMD_SAVE            "SAVE"
#define CMD_LOAD            "LOAD"
#define CMD_RST             "RST"
#define CMD_ECHO            "ECHO"
#define CMD_TEST            "TEST"
#define CMD_LOG             "LOG"
#define CMD_DIAG            "DIAG"
#define CMD_STREAM          "STREAM"
#define CMD_CLR             "CLR"           // Clear alarm/fault
#define CMD_CFGSTART        "CFGSTART"
#define CMD_CFGDATA         "CFGDATA"
#define CMD_CFGEND          "CFGEND"
#define CMD_CFGEXPORT       "CFGEXPORT"

// ============================================================================
// RESPONSE PREFIXES
// ============================================================================
#define RESP_OK             "OK"
#define RESP_ERROR          "ERROR"
#define RESP_EVENT          "EVENT"
#define RESP_YAML           "YAML:"
#define RESP_STREAM         "STRM"

// ============================================================================
// ERROR CODES
// ============================================================================
#define ERR_INVALID_COMMAND         "E001"
#define ERR_INVALID_AXIS            "E002"
#define ERR_INVALID_PARAMETER       "E003"
#define ERR_AXIS_NOT_ENABLED        "E004"
#define ERR_POSITION_LIMIT          "E005"
#define ERR_EMERGENCY_STOP          "E006"
#define ERR_CALIBRATION_REQUIRED    "E007"
#define ERR_MOTOR_FAULT             "E008"
#define ERR_COMMUNICATION           "E009"
#define ERR_CONFIGURATION           "E010"
#define ERR_EVENT_OVERFLOW          "E011"
#define ERR_MODE_BLOCKED            "E012"
#define ERR_MOTION_ACTIVE           "E013"
#define ERR_DRIVER_ALARM            "E014"
#define ERR_ALARM_CLEAR_FAILED      "E015"
#define ERR_ZSYNC_DRIFT             "E016"
#define ERR_CONFIG_AXES_MOVING      "E017"  // CFGSTART rejected: axes moving
#define ERR_CONFIG_PARSE_FAILED     "E018"  // YAML parse error
#define ERR_CONFIG_VALIDATION       "E019"  // YAML validation failed
#define ERR_CONFIG_BUFFER_OVERFLOW  "E030"  // YAML buffer exceeded 8KB

// ============================================================================
// ERROR MESSAGES
// ============================================================================
#define MSG_INVALID_COMMAND         "Invalid command"
#define MSG_INVALID_AXIS            "Invalid axis"
#define MSG_INVALID_PARAMETER       "Invalid parameter"
#define MSG_AXIS_NOT_ENABLED        "Axis not enabled"
#define MSG_POSITION_LIMIT          "Position limit exceeded"
#define MSG_EMERGENCY_STOP          "Emergency stop active"
#define MSG_CALIBRATION_REQUIRED    "Calibration required"
#define MSG_MOTOR_FAULT             "Motor fault"
#define MSG_COMMUNICATION           "Communication error"
#define MSG_CONFIGURATION           "Configuration error"
#define MSG_EVENT_OVERFLOW          "Event buffer overflow"
#define MSG_MODE_BLOCKED            "Command blocked in current mode"
#define MSG_MOTION_ACTIVE           "Motion active - stop first"
#define MSG_DRIVER_ALARM            "Driver alarm active"
#define MSG_ALARM_CLEAR_FAILED      "Alarm clear failed"
#define MSG_ZSYNC_DRIFT             "Z-signal drift exceeded threshold"
#define MSG_CONFIG_AXES_MOVING      "Stop all axes before configuration"
#define MSG_CONFIG_PARSE_FAILED     "YAML parse error"
#define MSG_CONFIG_VALIDATION       "Configuration validation failed"
#define MSG_CONFIG_BUFFER_OVERFLOW  "Config exceeds 8KB buffer limit"

// ============================================================================
// EVENT TYPES (for async notifications)
// ============================================================================
#define EVT_BOOT                    "BOOT"      // System ready: "EVENT BOOT V1.0.0 AXES:8 STATE:IDLE"
#define EVT_MOTION_COMPLETE         "DONE"
#define EVT_LIMIT_TRIGGERED         "LIMIT"
#define EVT_ESTOP_ACTIVATED         "ESTOP"
#define EVT_ALARM_TRIGGERED         "ALARM"
#define EVT_ALARM_CLEARED           "ALARMCLR"
#define EVT_HOMING_COMPLETE         "HOMED"
#define EVT_POSITION_UPDATE         "POS"
#define EVT_ZSYNC_DETECTED          "ZSYNC"     // Z-signal drift detected (not yet corrected)
#define EVT_ZSYNC_CORRECTED         "ZSYNCD"    // Z-signal correction applied (after motion complete)
#define EVT_SOFT_LIMIT_APPROACH     "SLIMIT"    // Approaching soft limit in velocity mode
#define EVT_PCNT_MISMATCH           "PCNTM"     // Stepper PCNT mismatch detected (warning only)

#endif // CONFIG_COMMANDS_H
```

### config_i2c.h — I2C Device Addresses

```c
#ifndef CONFIG_I2C_H
#define CONFIG_I2C_H

// ============================================================================
// I2C BUS CONFIGURATION
// ============================================================================
#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         400000          // 400kHz Fast Mode

// ============================================================================
// MCP23017 I/O EXPANDER ADDRESSES (2 devices, inputs only)
// ============================================================================
#define I2C_ADDR_MCP23017_0 0x20            // Limit switches (all 8 axes)
#define I2C_ADDR_MCP23017_1 0x21            // ALARM_INPUT + InPos (inputs only)

// ============================================================================
// SSD1306 OLED DISPLAY (on dedicated I2C_NUM_1 bus)
// OLED is isolated from main I2C to prevent display issues affecting I/O
// ============================================================================
// Note: OLED I2C configuration is in config_oled.h (separate bus)

// ============================================================================
// MCP23017 PIN MAPPINGS - EXPANDER 0 (Limit Switches)
// ============================================================================
#define MCP0_X_LIMIT_MIN    0               // GPA0
#define MCP0_X_LIMIT_MAX    1               // GPA1
#define MCP0_Y_LIMIT_MIN    2               // GPA2
#define MCP0_Y_LIMIT_MAX    3               // GPA3
#define MCP0_Z_LIMIT_MIN    4               // GPA4
#define MCP0_Z_LIMIT_MAX    5               // GPA5
#define MCP0_A_LIMIT_MIN    6               // GPA6
#define MCP0_A_LIMIT_MAX    7               // GPA7
#define MCP0_B_LIMIT_MIN    8               // GPB0
#define MCP0_B_LIMIT_MAX    9               // GPB1
#define MCP0_C_LIMIT_MIN    10              // GPB2
#define MCP0_C_LIMIT_MAX    11              // GPB3 (floating switch)
#define MCP0_D_LIMIT_MIN    12              // GPB4
#define MCP0_D_LIMIT_MAX    13              // GPB5
#define MCP0_E_LIMIT_MIN    14              // GPB6
#define MCP0_E_LIMIT_MAX    15              // GPB7

// ============================================================================
// MCP23017 PIN MAPPINGS - EXPANDER 1 (ALARM_INPUT + InPos)
// ============================================================================
// Port A (GPA*) - ALARM_INPUT Signals (Inputs) - 7 axes + 1 spare
#define MCP1_X_ALARM_INPUT  0               // GPA0 - X driver alarm input
#define MCP1_Y_ALARM_INPUT  1               // GPA1 - Y driver alarm input
#define MCP1_Z_ALARM_INPUT  2               // GPA2 - Z driver alarm input
#define MCP1_A_ALARM_INPUT  3               // GPA3 - A driver alarm input
#define MCP1_B_ALARM_INPUT  4               // GPA4 - B driver alarm input
#define MCP1_C_ALARM_INPUT  5               // GPA5 - C stepper alarm input
#define MCP1_D_ALARM_INPUT  6               // GPA6 - D stepper alarm input
#define MCP1_GP_IN_0        7               // GPA7 - Spare input

// Port B (GPB*) - InPos Signals + Spare Inputs
#define MCP1_X_INPOS        8               // GPB0 - X servo in-position
#define MCP1_Y_INPOS        9               // GPB1 - Y servo in-position
#define MCP1_Z_INPOS        10              // GPB2 - Z servo in-position
#define MCP1_A_INPOS        11              // GPB3 - A servo in-position
#define MCP1_B_INPOS        12              // GPB4 - B servo in-position
#define MCP1_GP_IN_1        13              // GPB5 - Spare input
#define MCP1_GP_IN_2        14              // GPB6 - Spare input
#define MCP1_GP_IN_3        15              // GPB7 - Spare input

// Note: ALARM_CLEAR outputs are on shift register (bits 3, 7, 11, 15, 19, 23, 27, 31)
// General purpose outputs are on shift register SR4 (bits 32-39)

#endif // CONFIG_I2C_H
```

### config_sr.h — Shift Register Bit Positions

```c
#ifndef CONFIG_SR_H
#define CONFIG_SR_H

// ============================================================================
// SHIFT REGISTER BIT POSITIONS
// ============================================================================
// Organization: 4 bits per axis [DIR, EN, BRAKE, ALARM_CLR] + 8 GP outputs
// Chain: MOSI → SR0 → SR1 → SR2 → SR3 → SR4 (5x TPIC6B595N, 40-bit)
// ============================================================================

// ============================================================================
// SR0-SR3: MOTOR CONTROL - 4 bits per axis [DIR, EN, BRAKE, ALARM_CLR]
// ============================================================================

// X-axis (Servo) - SR0 bits 0-3
#define SR_X_DIR            0       // SR0.Q0 - Direction
#define SR_X_EN             1       // SR0.Q1 - Enable
#define SR_X_BRAKE          2       // SR0.Q2 - Brake release
#define SR_X_ALARM_CLR      3       // SR0.Q3 - Alarm clear

// Y-axis (Servo) - SR0 bits 4-7
#define SR_Y_DIR            4       // SR0.Q4
#define SR_Y_EN             5       // SR0.Q5
#define SR_Y_BRAKE          6       // SR0.Q6
#define SR_Y_ALARM_CLR      7       // SR0.Q7

// Z-axis (Servo) - SR1 bits 8-11
#define SR_Z_DIR            8       // SR1.Q0
#define SR_Z_EN             9       // SR1.Q1
#define SR_Z_BRAKE          10      // SR1.Q2
#define SR_Z_ALARM_CLR      11      // SR1.Q3

// A-axis (Servo) - SR1 bits 12-15
#define SR_A_DIR            12      // SR1.Q4
#define SR_A_EN             13      // SR1.Q5
#define SR_A_BRAKE          14      // SR1.Q6
#define SR_A_ALARM_CLR      15      // SR1.Q7

// B-axis (Servo) - SR2 bits 16-19
#define SR_B_DIR            16      // SR2.Q0
#define SR_B_EN             17      // SR2.Q1
#define SR_B_BRAKE          18      // SR2.Q2
#define SR_B_ALARM_CLR      19      // SR2.Q3

// C-axis (Stepper) - SR2 bits 20-23
#define SR_C_DIR            20      // SR2.Q4
#define SR_C_EN             21      // SR2.Q5
#define SR_C_BRAKE          22      // SR2.Q6 (not connected - stepper)
#define SR_C_ALARM_CLR      23      // SR2.Q7

// D-axis (Stepper) - SR3 bits 24-27
#define SR_D_DIR            24      // SR3.Q0
#define SR_D_EN             25      // SR3.Q1
#define SR_D_BRAKE          26      // SR3.Q2 (not connected - stepper)
#define SR_D_ALARM_CLR      27      // SR3.Q3

// E-axis (Discrete) - SR3 bits 28-31
#define SR_E_DIR            28      // SR3.Q4
#define SR_E_EN             29      // SR3.Q5
#define SR_E_BRAKE          30      // SR3.Q6 (not used - discrete)
#define SR_E_ALARM_CLR      31      // SR3.Q7 (not used - discrete)

// ============================================================================
// SR4: GENERAL PURPOSE OUTPUTS (8 bits)
// ============================================================================
#define SR_GP_OUT_0         32      // SR4.Q0 - General purpose output
#define SR_GP_OUT_1         33      // SR4.Q1
#define SR_GP_OUT_2         34      // SR4.Q2
#define SR_GP_OUT_3         35      // SR4.Q3
#define SR_GP_OUT_4         36      // SR4.Q4
#define SR_GP_OUT_5         37      // SR4.Q5
#define SR_GP_OUT_6         38      // SR4.Q6
#define SR_GP_OUT_7         39      // SR4.Q7

// ============================================================================
// HELPER MACROS - 4 bits per axis
// ============================================================================
#define SR_DIR_BIT(axis)       ((axis) * 4 + 0)
#define SR_EN_BIT(axis)        ((axis) * 4 + 1)
#define SR_BRAKE_BIT(axis)     ((axis) * 4 + 2)
#define SR_ALARM_CLR_BIT(axis) ((axis) * 4 + 3)

// Bit manipulation (64-bit for 40-bit chain)
#define SR_SET_BIT(data, bit)   ((data) | (1ULL << (bit)))
#define SR_CLR_BIT(data, bit)   ((data) & ~(1ULL << (bit)))
#define SR_GET_BIT(data, bit)   (((data) >> (bit)) & 1)

// ============================================================================
// FAIL-SAFE DEFAULTS
// ============================================================================
// On power-up or reset, all outputs LOW = all brakes engaged
#define SR_SAFE_STATE       0x0000000000ULL // All 40 bits 0 = fail-safe

#endif // CONFIG_SR_H
```

### config_defaults.h — Default Axis Parameters (SI Units)

```c
#ifndef CONFIG_DEFAULTS_H
#define CONFIG_DEFAULTS_H

// ============================================================================
// SI UNITS: All values in meters, radians, seconds
// ROS2 REP-103 compliant for future ros2_control integration
// ============================================================================

// ============================================================================
// MATH CONSTANTS
// ============================================================================
#define CONST_PI                    3.14159265f
#define CONST_2PI                   6.28318531f
#define CONST_DEG_TO_RAD            0.01745329f     // π/180
#define CONST_RAD_TO_DEG            57.2957795f     // 180/π

// ============================================================================
// DEFAULT LINEAR AXIS CONFIGURATION (meters)
// ============================================================================
#define DEFAULT_PULSES_PER_UNIT     1000000.0f  // 1M pulses/meter = 1µm resolution
#define DEFAULT_IS_ROTARY           false
#define DEFAULT_LIMIT_MIN           -1.0f       // -1 meter
#define DEFAULT_LIMIT_MAX           1.0f        // +1 meter
#define DEFAULT_MAX_VELOCITY        0.1f        // 0.1 m/s = 100 mm/s
#define DEFAULT_MAX_ACCELERATION    1.0f        // 1 m/s²
#define DEFAULT_BACKLASH            0.0f        // meters
#define DEFAULT_HOME_OFFSET         0.0f        // meters

// ============================================================================
// DEFAULT ROTARY AXIS CONFIGURATION (radians)
// ============================================================================
#define DEFAULT_ROTARY_PULSES_PER_UNIT  10000.0f    // pulses/radian
#define DEFAULT_ROTARY_LIMIT_MIN        0.0f        // radians
#define DEFAULT_ROTARY_LIMIT_MAX        CONST_2PI   // 2π rad (360°)
#define DEFAULT_ROTARY_MAX_VEL          CONST_PI    // π rad/s (180°/s)
#define DEFAULT_ROTARY_MAX_ACCEL        10.0f       // rad/s²

// ============================================================================
// E-AXIS (DISCRETE ACTUATOR) - binary positions
// ============================================================================
#define E_AXIS_PULSES_PER_UNIT      1.0f        // 1 pulse = 1 unit (binary)
#define E_AXIS_IS_ROTARY            false
#define E_AXIS_LIMIT_MIN            0.0f        // retracted
#define E_AXIS_LIMIT_MAX            1.0f        // extended
#define E_AXIS_MAX_VELOCITY         1.0f        // units/sec (actuation speed)
#define E_AXIS_MAX_ACCELERATION     10.0f       // units/sec²

#endif // CONFIG_DEFAULTS_H
```

### config_oled.h — OLED Display Configuration

```c
#ifndef CONFIG_OLED_H
#define CONFIG_OLED_H

// ============================================================================
// OLED I2C BUS CONFIGURATION (Dedicated bus, isolated from main I2C)
// ============================================================================
#define I2C_OLED_PORT       I2C_NUM_1       // Dedicated I2C bus for OLED
#define I2C_OLED_FREQ_HZ    400000          // 400kHz Fast Mode

// GPIO pins defined in config_gpio.h:
// GPIO_OLED_SDA (J1-20)
// GPIO_OLED_SCL (J3-18)

// ============================================================================
// DISPLAY PARAMETERS
// ============================================================================
#define OLED_ADDRESS        0x3C            // SSD1306 I2C address
#define OLED_WIDTH          128             // Display width in pixels
#define OLED_HEIGHT         64              // Display height in pixels
#define OLED_UPDATE_HZ      2               // Update rate (2Hz = 500ms)

// ============================================================================
// DISPLAY LAYOUT
// ============================================================================
#define OLED_FONT_WIDTH     6               // Font character width
#define OLED_FONT_HEIGHT    8               // Font character height
#define OLED_CHARS_PER_LINE 21              // Characters per line (128/6)
#define OLED_NUM_LINES      7               // Total lines (64/8, all usable)

// Line allocations
#define OLED_AXIS_LINES     4               // Lines 0-3 for axis status
#define OLED_SEPARATOR_LINE 4               // Line 4 for separator
#define OLED_EVENT_LINES    2               // Lines 5-6 for events/errors

// ============================================================================
// TIMING
// ============================================================================
#define OLED_EVENT_DISPLAY_MS   2000        // Event display duration (2 sec)

#endif // CONFIG_OLED_H
```

### config_yaml_schema.h — YAML Configuration Schema

```c
#ifndef CONFIG_YAML_SCHEMA_H
#define CONFIG_YAML_SCHEMA_H

#include "config_limits.h"
#include "config_defaults.h"

// ============================================================================
// YAML SCHEMA VERSION
// Increment when schema structure changes (not just values)
// ============================================================================
#define YAML_SCHEMA_VERSION         1

// ============================================================================
// ROOT KEYS
// ============================================================================
#define YAML_KEY_VERSION            "version"
#define YAML_KEY_AXES               "axes"
#define YAML_KEY_SYSTEM             "system"

// ============================================================================
// AXIS NAMES (used as YAML section keys under "axes:")
// ============================================================================
#define YAML_AXIS_X                 "X"
#define YAML_AXIS_Y                 "Y"
#define YAML_AXIS_Z                 "Z"
#define YAML_AXIS_A                 "A"
#define YAML_AXIS_B                 "B"
#define YAML_AXIS_C                 "C"
#define YAML_AXIS_D                 "D"
#define YAML_AXIS_E                 "E"

// Axis name array for iteration
static const char* const YAML_AXIS_NAMES[LIMIT_NUM_AXES] = {
    YAML_AXIS_X, YAML_AXIS_Y, YAML_AXIS_Z, YAML_AXIS_A,
    YAML_AXIS_B, YAML_AXIS_C, YAML_AXIS_D, YAML_AXIS_E
};

// ============================================================================
// PER-AXIS PARAMETER KEYS (SI Units: meters, radians, seconds)
// ============================================================================
#define YAML_KEY_ALIAS              "alias"
#define YAML_KEY_TYPE               "type"              // "linear" or "rotary"
#define YAML_KEY_PULSES_PER_REV     "pulses_per_rev"    // pulses per motor revolution (driver PA14 or gear ratio)
#define YAML_KEY_UNITS_PER_REV      "units_per_rev"     // meters or radians per motor revolution
#define YAML_KEY_LIMITS             "limits"            // [min, max] in meters or radians
#define YAML_KEY_MAX_VELOCITY       "max_velocity"      // m/s or rad/s
#define YAML_KEY_MAX_ACCELERATION   "max_acceleration"  // m/s² or rad/s²
#define YAML_KEY_BACKLASH           "backlash"          // meters or radians
#define YAML_KEY_HOME_OFFSET        "home_offset"       // meters or radians

// Homing configuration (nested under "homing" key)
#define YAML_KEY_HOMING             "homing"            // Homing config section
#define YAML_KEY_HOMING_VELOCITY    "velocity"          // Velocity for limit seek (m/s or rad/s)
#define YAML_KEY_HOMING_VELOCITY_SLOW "velocity_slow"   // Velocity for Z-signal seek (slower)
#define YAML_KEY_HOMING_BACKOFF     "backoff"           // Distance to back off from limit (m or rad)
#define YAML_KEY_HOMING_DIRECTION   "direction"         // "min" = toward MIN limit, "max" = toward MAX

// Z-signal configuration (nested under "z_signal" key)
#define YAML_KEY_ZSIGNAL            "z_signal"          // Z-signal sync config section
#define YAML_KEY_ZSIGNAL_ENABLED    "enabled"           // Enable Z-signal position sync (bool)
#define YAML_KEY_ZSIGNAL_THRESHOLD  "drift_threshold"   // Alarm threshold in pulses (0 = no alarm)

// Axis type values
#define YAML_VAL_TYPE_LINEAR        "linear"    // meters
#define YAML_VAL_TYPE_ROTARY        "rotary"    // radians

// ============================================================================
// SYSTEM PARAMETER KEYS
// ============================================================================
#define YAML_KEY_DEBOUNCE_MS        "debounce_ms"
#define YAML_KEY_IDLE_TIMEOUT_S     "idle_timeout_s"
#define YAML_KEY_LOG_LEVEL          "log_level"

// ============================================================================
// LOG LEVEL VALUES (must match ESP-IDF esp_log_level_t)
// ============================================================================
#define YAML_VAL_LOG_NONE           "NONE"
#define YAML_VAL_LOG_ERROR          "ERROR"
#define YAML_VAL_LOG_WARN           "WARN"
#define YAML_VAL_LOG_INFO           "INFO"
#define YAML_VAL_LOG_DEBUG          "DEBUG"

// ============================================================================
// NVS KEY MAPPINGS
// Maps YAML structure to flat NVS key-value storage
// ============================================================================
#define NVS_NAMESPACE               "yarobot"
#define NVS_KEY_PREFIX_AXIS         "ax"       // ax0_, ax1_, etc.
#define NVS_KEY_SUFFIX_PPR          "_ppr"     // pulses_per_rev (uint32)
#define NVS_KEY_SUFFIX_UPR          "_upr"     // units_per_rev (float)
#define NVS_KEY_SUFFIX_ROTARY       "_rot"     // is_rotary (bool)
#define NVS_KEY_SUFFIX_MIN          "_min"
#define NVS_KEY_SUFFIX_MAX          "_max"
#define NVS_KEY_SUFFIX_VEL          "_vel"
#define NVS_KEY_SUFFIX_ACC          "_acc"
#define NVS_KEY_SUFFIX_BACKLASH     "_blash"
#define NVS_KEY_SUFFIX_ALIAS        "_alias"
#define NVS_KEY_SUFFIX_HOME_OFF     "_home_off"
#define NVS_KEY_SUFFIX_ZSIG_EN      "_zsen"    // z_signal_enabled (bool)
#define NVS_KEY_SUFFIX_ZSIG_THR     "_zsth"    // z_drift_threshold (int32)

// System NVS keys
#define NVS_KEY_SYS_VERSION         "sys_version"
#define NVS_KEY_SYS_SLOT            "sys_slot"
#define NVS_KEY_SYS_DEBOUNCE        "sys_debounce"

// ============================================================================
// VALIDATION MACROS
// Compute limits from compile-time constants for runtime validation
// ============================================================================

// Compute pulses_per_unit from configuration parameters
#define YAML_PULSES_PER_UNIT(pulses_per_rev, units_per_rev) \
    ((float)(pulses_per_rev) / (units_per_rev))

// Maximum velocity in user units
// velocity_max = max_pulse_freq / pulses_per_unit
#define YAML_MAX_VELOCITY(pulses_per_rev, units_per_rev) \
    ((float)LIMIT_MAX_PULSE_FREQ_HZ / YAML_PULSES_PER_UNIT(pulses_per_rev, units_per_rev))

// Minimum velocity (must generate at least 1 pulse/sec)
#define YAML_MIN_VELOCITY(pulses_per_rev, units_per_rev) \
    ((float)LIMIT_MIN_PULSE_FREQ_HZ / YAML_PULSES_PER_UNIT(pulses_per_rev, units_per_rev))

// Validate velocity against computed limits
#define YAML_VALIDATE_VELOCITY(vel, pulses_per_rev, units_per_rev) \
    ((vel) >= YAML_MIN_VELOCITY(pulses_per_rev, units_per_rev) && \
     (vel) <= YAML_MAX_VELOCITY(pulses_per_rev, units_per_rev))

// Validate pulses_per_rev (must be positive, reasonable range)
#define YAML_VALIDATE_PPR(ppr) \
    ((ppr) > 0 && (ppr) <= 1000000)

// Validate units_per_rev (must be positive)
#define YAML_VALIDATE_UPR(upr) \
    ((upr) > 0.0f)

// Validate alias length
#define YAML_VALIDATE_ALIAS_LEN(alias) \
    (strlen(alias) <= LIMIT_ALIAS_MAX_LENGTH)

// Validate axis index
#define YAML_VALIDATE_AXIS(idx) \
    ((idx) >= 0 && (idx) < LIMIT_NUM_AXES)

// Validate position limits (min < max)
#define YAML_VALIDATE_LIMITS(min, max) \
    ((min) < (max))

// Validate Z-signal drift threshold (0 or positive)
#define YAML_VALIDATE_ZSIG_THRESHOLD(thr) \
    ((thr) >= 0)

// ============================================================================
// HELPER: Build NVS key string
// Usage: char key[16]; NVS_AXIS_KEY(key, 0, NVS_KEY_SUFFIX_UNITS); // "ax0_units"
// ============================================================================
#define NVS_AXIS_KEY(buf, axis_idx, suffix) \
    snprintf((buf), sizeof(buf), "%s%d%s", NVS_KEY_PREFIX_AXIS, (axis_idx), (suffix))

#endif // CONFIG_YAML_SCHEMA_H
```

### config.h — Master Include

```c
#ifndef CONFIG_H
#define CONFIG_H

// Master configuration header - includes all configuration files

#include "config_gpio.h"
#include "config_peripherals.h"
#include "config_timing.h"
#include "config_limits.h"
#include "config_commands.h"
#include "config_i2c.h"
#include "config_sr.h"              // Shift register bit positions
#include "config_defaults.h"
#include "config_oled.h"
#include "config_yaml_schema.h"

// ============================================================================
// VERSION INFORMATION
// ============================================================================
#define FIRMWARE_NAME           "YAROBOT_CONTROL_UNIT"
#define FIRMWARE_VERSION_MAJOR  1
#define FIRMWARE_VERSION_MINOR  0
#define FIRMWARE_VERSION_PATCH  0
#define FIRMWARE_VERSION_STRING "1.0.0"

// ============================================================================
// FEATURE FLAGS
// ============================================================================
#define FEATURE_OLED_DISPLAY    1           // MVP: OLED display enabled
#define FEATURE_Z_SIGNAL        1           // Set to 1 to enable Z-signal homing
#define FEATURE_STREAMING       1           // Set to 1 to enable position streaming

#endif // CONFIG_H
```

**Usage in Code:**

```cpp
// Example: Using configuration in motor driver
#include "config.h"

void initMotorX() {
    // GPIO comes from config
    gpio_set_direction(GPIO_X_STEP, GPIO_MODE_OUTPUT);

    // Timing comes from config
    esp_rom_delay_us(TIMING_DIR_SETUP_US);

    // Limits from config
    if (queue_depth > LIMIT_MOTION_QUEUE_DEPTH) {
        return ESP_ERR_NO_MEM;
    }
}

// Example: Using command strings
if (strcmp(cmd, CMD_MOVE) == 0) {
    // Handle MOVE command
} else if (strcmp(cmd, CMD_STOP) == 0) {
    // Handle STOP command
}

// Example: Error response
sendResponse(RESP_ERROR " " ERR_AXIS_NOT_ENABLED " " MSG_AXIS_NOT_ENABLED);
// Output: "ERROR E004 Axis not enabled"

// Example: Using YAML schema keys in parser
#include "config_yaml_schema.h"

esp_err_t parse_axis_config(yaml_node_t* axis_node, uint8_t axis_idx) {
    float velocity = yaml_get_float(axis_node, YAML_KEY_VELOCITY);
    float units = yaml_get_float(axis_node, YAML_KEY_UNITS_PER_PULSE);

    // Validate against compile-time limits
    if (!YAML_VALIDATE_VELOCITY(velocity, units)) {
        ESP_LOGE(TAG, "Velocity %.2f exceeds max %.2f for axis %d",
                 velocity, YAML_MAX_VELOCITY(units), axis_idx);
        return ESP_ERR_INVALID_ARG;
    }

    // Build NVS key and store
    char nvs_key[16];
    NVS_AXIS_KEY(nvs_key, axis_idx, NVS_KEY_SUFFIX_VEL);
    nvs_set_float(nvs_handle, nvs_key, velocity);

    return ESP_OK;
}
```

## Data Architecture

### NVS (Non-Volatile Storage) Organization

ESP32-S3 NVS is used for persistent configuration storage.

**Namespace:** `yarobot`

```
Per-Axis Keys (X=0 to E=7):
├── ax{n}_ppr        (u32)     Pulses per revolution (driver PA14 or microstep config)
├── ax{n}_upr        (float)   Units per revolution (meters or radians)
├── ax{n}_rot        (u8)      Axis type: 0=linear(meters), 1=rotary(radians)
├── ax{n}_min        (float)   Position limit minimum (meters or radians)
├── ax{n}_max        (float)   Position limit maximum (meters or radians)
├── ax{n}_vel        (float)   Max velocity (m/s or rad/s)
├── ax{n}_acc        (float)   Max acceleration (m/s² or rad/s²)
├── ax{n}_blash      (float)   Backlash compensation (meters or radians)
├── ax{n}_alias      (str:16)  Human-readable alias
├── ax{n}_home_off   (float)   Home position offset (meters or radians)
├── ax{n}_zsen       (u8)      Z-signal enabled: 0=disabled, 1=enabled
└── ax{n}_zsth       (i32)     Z-signal drift threshold (pulses, 0=no alarm)

System Keys:
├── sys_version      (u32)     Configuration version number
├── sys_slot         (u8)      Active configuration slot (0-9)
└── sys_debounce     (u16)     Input debounce time (ms)

Configuration Slots (0-9):
└── slot{n}_blob     (blob)    Serialized configuration snapshot
```

### YAML Configuration Schema

Configuration transferred via serial using CFGSTART/CFGDATA/CFGEND commands.

```yaml
# yarobot_config.yaml
# All units follow SI convention (ROS2 REP-103 compatible):
#   Linear: meters (m), m/s, m/s²
#   Rotary: radians (rad), rad/s, rad/s²
#
# Position conversion: pulses = position / units_per_rev * pulses_per_rev
# Z-signal sync: 1 Z-pulse = 1 revolution = pulses_per_rev = units_per_rev
version: 1

axes:
  X:
    alias: "RAILWAY"
    type: linear                # meters (SI base unit)
    pulses_per_rev: 10000       # Driver PA14 setting (pulses per motor revolution)
    units_per_rev: 0.005        # 5mm ball screw lead = 0.005 m/rev
    # Derived: pulses_per_unit = 10000 / 0.005 = 2,000,000 pulses/meter
    limits: [-0.5, 0.5]         # ±500mm in meters
    max_velocity: 0.2           # 200mm/s in m/s
    max_acceleration: 1.0       # 1000mm/s² in m/s²
    backlash: 0.00005           # 50µm in meters (post-MVP)
    home_offset: 0.0
    homing:                     # Homing configuration
      velocity: 0.05            # 50mm/s for limit seek
      velocity_slow: 0.01       # 10mm/s for Z-signal seek
      backoff: 0.005            # 5mm backoff from limit
      direction: min            # Home toward MIN limit
    z_signal:                   # Z-signal position synchronization (servo axes only)
      enabled: true             # Enable periodic position sync via encoder index
      drift_threshold: 100      # Alarm if drift > 100 pulses (0 = correct silently)

  Y:
    alias: "GRIPPER"
    type: rotary                # radians (SI base unit)
    pulses_per_rev: 10000       # Driver PA14 setting
    units_per_rev: 6.283185     # 2π rad/rev (direct drive, no gearbox)
    # Derived: pulses_per_unit = 10000 / 6.283185 = 1,592 pulses/radian
    limits: [0.0, 6.283185]     # 0-360° in radians
    max_velocity: 12.566        # 720°/s in rad/s
    max_acceleration: 62.83     # 3600°/s² in rad/s²
    backlash: 0.0
    home_offset: 0.0
    homing:
      velocity: 1.047           # 60°/s for limit seek
      velocity_slow: 0.175      # 10°/s for Z-signal seek
      backoff: 0.087            # 5° backoff from limit
      direction: min
    z_signal:
      enabled: true
      drift_threshold: 50

  C:
    alias: "JAW"
    type: rotary
    pulses_per_rev: 6400        # Stepper: 200 steps × 32 microsteps
    units_per_rev: 6.283185     # 2π rad/rev (direct drive)
    limits: [-3.14159, 3.14159]
    max_velocity: 3.14159
    max_acceleration: 15.708
    backlash: 0.0
    home_offset: 0.0
    homing:                     # Steppers use limit-only homing (no Z-signal)
      velocity: 0.524           # 30°/s for limit seek
      backoff: 0.087            # 5° backoff
      direction: min
    # No z_signal section = disabled (steppers don't have encoder index)

  # ... Z, A, B, D, E axes follow same schema

system:
  debounce_ms: 10               # input debounce time
  idle_timeout_s: 300           # motor idle timeout (0 = disabled)
  log_level: "INFO"             # NONE, ERROR, WARN, INFO, DEBUG
```

### Data Flow

> **⚠️ ARCHITECTURE CONSTRAINT - CONFIG MODE REQUIREMENTS**
>
> **CONFIG mode requires all axes STOPPED.** This is non-negotiable.
>
> - CFGSTART command fails with ERROR if any axis is moving
> - Configuration is parsed and validated entirely before any changes are applied
> - On validation failure, NO changes are applied (full rollback)
> - YAML buffer overflow (>8KB) returns ERROR E030 and clears buffer
> - Host must ensure all motion is complete before initiating config transfer

```
Host (Python/C++)
     │
     ├─ CFGSTART ──────────► Firmware enters CONFIG mode
     │                       └─► ERROR if any axis moving
     ├─ CFGDATA line1 ─────► Buffer accumulates YAML
     ├─ CFGDATA line2 ─────► ERROR E030 if buffer exceeds 8KB
     ├─ ...
     ├─ CFGEND APPLY ──────► Parse YAML, validate ALL, then apply atomically
     │                       └─► On validation error: NO changes, full rollback
     │                       └─► SAVE ──► Persist to NVS
     │
     └─ CFGEXPORT ─────────► Firmware outputs current config as YAML
```

## Safety Architecture

### System State Machine

```
                    ┌─────────────────┐
                    │   POWER_ON      │
                    └────────┬────────┘
                             │ Init complete
                             │ Send EVENT BOOT
                    ┌────────v────────┐
                    │     IDLE        │◄──────────────────┐
                    │ (Motors DISABLED│                   │
                    │  Brakes engaged)│                   │
                    └────────┬────────┘                   │
                             │ EN command                 │ RST command
                    ┌────────v────────┐                   │ (E-stop released)
              ┌─────┤    READY        ├───────┐           │
              │     │ (Brakes can be  │       │           │
              │     │  released)      │       │           │
              │     └────────┬────────┘       │           │
              │              │ Motion cmd     │           │
              │     ┌────────v────────┐       │           │
              │     │    MOVING       │       │           │
              │     │ (Active motion) │       │           │
              │     └────────┬────────┘       │           │
              │              │ Limit/Fault    │           │
              │     ┌────────v────────┐       │           │
              │     │     ERROR       │───────┤           │
              │     │ (Axis stopped)  │       │           │
              │     └─────────────────┘       │           │
              │                               │           │
              │     ┌─────────────────┐       │           │
              └────►│  ESTOP_ACTIVE   │◄──────┘           │
                    │ (All stopped,   ├───────────────────┘
                    │  Brakes engaged)│
                    └─────────────────┘
                          │
                    E-stop held │
                          v
                    ┌─────────────────┐
                    │ CRITICAL_FAULT  │
                    │ (Requires power │
                    │  cycle)         │
                    └─────────────────┘
```

### E-Stop Handling (Highest Priority)

> **⚠️ ARCHITECTURE CONSTRAINT - E-STOP POSITION HANDLING**
>
> **After hardware E-stop, all axes are marked UNHOMED.** This is non-negotiable.
>
> - Position is considered unknown after E-stop release
> - All axes require re-homing before position moves are allowed
> - VEL (jog) commands allowed for manual recovery
> - STOP EMERGENCY (software) also triggers STATE_ESTOP_ACTIVE with same recovery path
> - This is a safety measure: mechanical settling during E-stop may cause position drift

E-Stop uses direct GPIO interrupt for fastest response:

```c
// ISR context - executes in <5µs
static void IRAM_ATTR emergency_stop_isr(void* arg) {
    // 1. Stop all pulse generation immediately via direct register access
    RMT.conf_ch[RMT_CHANNEL_X].conf1.tx_start = 0;
    RMT.conf_ch[RMT_CHANNEL_Z].conf1.tx_start = 0;
    RMT.conf_ch[RMT_CHANNEL_A].conf1.tx_start = 0;
    RMT.conf_ch[RMT_CHANNEL_B].conf1.tx_start = 0;
    MCPWM0.timer[MCPWM_TIMER_Y].mode.start = 0;
    MCPWM0.timer[MCPWM_TIMER_C].mode.start = 0;

    // 2. Force STEP outputs low
    GPIO.out_w1tc = STEP_OUTPUT_MASK;

    // 3. Notify safety task (deferred brake handling)
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(safety_task_handle, NOTIFY_ESTOP, eSetBits, &woken);
    portYIELD_FROM_ISR(woken);
}
```

### Brake Control Strategies

Per-axis configurable brake behavior:

| Strategy | When Engaged | Use Case |
|----------|--------------|----------|
| `BRAKE_ON_DISABLE` | Motor disabled | Default - normal operation |
| `BRAKE_ON_ESTOP` | E-stop only | High-speed horizontal axes |
| `BRAKE_ON_IDLE` | After idle timeout | Vertical axes, power saving |
| `BRAKE_NEVER` | Manual only | Testing/maintenance |

```c
// Brake control via shift register (active-low, fail-safe)
// Bit 0 = engaged (spring applied), Bit 1 = released (power applied)
typedef struct {
    BrakeStrategy strategy;
    uint32_t idle_threshold_ms;  // For BRAKE_ON_IDLE
    bool is_engaged;
    uint32_t idle_timer;
} brake_state_t;

brake_state_t brake_states[LIMIT_NUM_SERVOS];  // X,Y,Z,A,B only (servos)
```

### Limit Switch Handling

14 limit switches on MCP23017 expanders with interrupt-driven detection:

```c
// Limit switch response (task context, <5ms latency)
void process_limit_switch(uint8_t axis, bool min_triggered, bool max_triggered) {
    float velocity = get_current_velocity(axis);

    // Only stop if moving toward triggered limit
    if ((min_triggered && velocity < 0) || (max_triggered && velocity > 0)) {
        stop_axis_immediate(axis);
        set_axis_error(axis, ERR_LIMIT_SWITCH);

        // Still allow motion away from limit
        axis_state[axis].can_move_positive = !max_triggered;
        axis_state[axis].can_move_negative = !min_triggered;
    }
}
```

### Driver Alarm Handling

Motor drivers (servo and stepper) may enter alarm states (overcurrent, overheat, position error, etc.). The system provides dedicated alarm detection and clearing for all motor axes:

**Alarm Detection (ALARM_INPUT signals) — via MCP23017 #1:**
- ALARM_INPUT signals on MCP23017 #1 Port A (GPA0-GPA6) for axes X-D
- Monitored via GPIO_MCP1_INTA interrupt
- When driver enters alarm state, corresponding ALARM_INPUT goes active
- Generates `EVENT ALARM <axis>` async notification to host

**Alarm Clearing (ALARM_CLR outputs) — via Shift Registers (SPI):**
- ALARM_CLR outputs on shift register bits (SR_X_ALARM_CLR through SR_D_ALARM_CLR)
- Fast SPI-based control (~1µs per update vs ~100µs for I2C)
- Pulse these outputs to attempt alarm reset
- Pulse polarity and duration are driver-dependent (consult driver docs)
- Generates `EVENT ALARMCLR <axis>` on successful clear

**CLR Command:**
```
CLR <axis>              # Clear alarm for specific axis (X, Y, Z, A, B, C, D)
CLR ALL                 # Clear alarms for all axes
```

**Response:**
```
OK                      # Alarm cleared successfully
ERROR E014 Driver alarm active    # Alarm still present after clear attempt
ERROR E015 Alarm clear failed     # Clear operation failed
```

```c
// Alarm clear via Shift Registers (SPI) - fast control
esp_err_t clear_driver_alarm(uint8_t axis) {
    if (axis >= LIMIT_NUM_MOTOR_AXES) return ESP_ERR_INVALID_ARG;  // 0-6 for X-D

    // Check if alarm is actually active (read from MCP23017)
    uint8_t alarm_input_pin = MCP1_X_ALARM_INPUT + axis;
    if (!mcp23017_get_pin(&mcp1, alarm_input_pin)) {
        return ESP_OK;  // No alarm to clear
    }

    // Pulse the ALARM_CLR output via shift register (duration/polarity driver-specific)
    uint8_t alarm_clr_bit = SR_ALARM_CLR_BIT(axis);
    sr_set_bit(alarm_clr_bit);                    // Assert via SPI
    sr_update();                                  // Latch to outputs
    vTaskDelay(pdMS_TO_TICKS(100));               // Hold 100ms (adjust per driver)
    sr_clear_bit(alarm_clr_bit);                  // Release via SPI
    sr_update();                                  // Latch to outputs

    // Check if alarm cleared (read from MCP23017)
    vTaskDelay(pdMS_TO_TICKS(50));
    if (mcp23017_get_pin(&mcp1, alarm_input_pin)) {
        // Alarm still active
        return ESP_ERR_INVALID_STATE;
    }

    // Publish alarm cleared event
    EventManager::publish(Event{
        .type = EVT_ALARM_CLEARED,
        .axis = axis,
        .timestamp = esp_timer_get_time()
    });

    return ESP_OK;
}

// Alarm detection ISR handler
void IRAM_ATTR mcp1_inta_isr(void* arg) {
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(safety_task_handle, NOTIFY_ALARM, eSetBits, &woken);
    portYIELD_FROM_ISR(woken);
}

// Process alarm interrupt in safety task
void process_alarm_inputs(void) {
    uint8_t port_a = mcp23017_read_port(&mcp1, MCP23017_PORT_A);

    for (uint8_t axis = 0; axis < LIMIT_NUM_MOTOR_AXES; axis++) {
        bool alarm_active = (port_a >> axis) & 0x01;

        if (alarm_active && !axis_state[axis].alarm_active) {
            // New alarm detected
            axis_state[axis].alarm_active = true;
            motor_stop(axis);  // Stop axis motion

            EventManager::publish(Event{
                .type = EVT_ALARM_TRIGGERED,
                .axis = axis,
                .timestamp = esp_timer_get_time()
            });
        }
    }
}
```

### Error Categories

| Category | Examples | Response | Recovery |
|----------|----------|----------|----------|
| **Critical** | E-stop, I2C lost, runaway | Stop all, engage brakes | Power cycle or RST |
| **Axis** | Limit hit, driver alarm | Stop axis only | CLR command |
| **Warning** | USB timeout, retry success | Log, continue | Auto-clear |

**Alarm vs Fault Distinction:**
- **Driver Alarm (E014)**: Detected via ALARM_INPUT, recoverable with CLR command
- **Motor Fault (E008)**: Internal communication/hardware failure, may require power cycle

### Safety Task (Core 0, Priority 24 - Highest)

```c
void safety_monitor_task(void* param) {
    while (1) {
        uint32_t notify;

        // Wait for safety events or periodic check
        if (xTaskNotifyWait(0, ULONG_MAX, &notify, pdMS_TO_TICKS(TIMING_SAFETY_POLL_MS))) {
            if (notify & NOTIFY_ESTOP) handle_estop();
            if (notify & NOTIFY_LIMIT) process_limit_switches();
            if (notify & NOTIFY_ALARM) process_alarm_inputs();
        }

        // Periodic checks at TIMING_SAFETY_POLL_MS interval
        monitor_estop_gpio();
        check_driver_alarms();
        update_brake_timers();
        check_motion_timeouts();
    }
}
```

### Power Loss Protection

Fail-safe design ensures brakes engage on power loss:

1. **Shift register outputs go LOW** on power loss (TPIC6B595N)
2. **Brake logic is active-low** (0 = engaged, 1 = released)
3. **Spring-applied brakes** mechanically engage without power
4. **RTC memory stores positions** before brownout (if detected)

```c
// Brownout detection (if available)
void power_monitor_isr(void) {
    // Quick save to RTC memory
    for (int i = 0; i < LIMIT_NUM_AXES; i++) {
        rtc_position[i] = current_position[i];
    }
    rtc_power_loss_flag = true;
}
```

## API Contracts

### Command Protocol

All commands are newline-terminated ASCII text:

```
<VERB> [<AXIS>] [<PARAM1>] [<PARAM2>] ...
```

**Response Format:**
```
OK [<data>]                    # Success
ERROR <code> <message>         # Error
EVENT <type> <axis> [<data>]   # Async notification
```

**Command Reference:** See `docs/planning_phase/command-interface.md` for complete specification.

### Internal APIs

**Motor Interface:**
```cpp
esp_err_t motor_move_absolute(uint8_t axis, float position, float velocity);
esp_err_t motor_move_relative(uint8_t axis, float delta, float velocity);
esp_err_t motor_stop(uint8_t axis);
esp_err_t motor_enable(uint8_t axis, bool enable);
float motor_get_position(uint8_t axis);
bool motor_is_moving(uint8_t axis);
```

**Shift Register Interface:**
```cpp
void sr_set_direction(uint8_t axis, bool forward);
void sr_set_enable(uint8_t axis, bool enable);
void sr_engage_brake(uint8_t axis);
void sr_release_brake(uint8_t axis);
void sr_emergency_disable_all(void);  // ISR-safe
```

## Security Architecture

**Scope:** Internal tool, no network connectivity, physical access required.

**Measures:**
- No authentication (USB physical access = full control)
- Input validation prevents buffer overflows and invalid parameters
- NaN/Infinity checks on all float inputs
- Axis bounds checking before array access
- No code execution from USB - command parsing only

**Not Applicable:**
- Network security (no network)
- Encryption (local USB only)
- User roles (single operator)

## Performance Considerations

### Critical Timing Requirements

| Requirement | Target | Strategy |
|-------------|--------|----------|
| E-stop response | <5µs | Direct ISR register access |
| Limit switch response | <5ms | I2C interrupt + polling |
| Command latency | <10ms | Queue-based processing |
| Pulse accuracy | ±1% | Hardware peripherals (RMT/MCPWM) |
| Motion update rate | 1000Hz | FreeRTOS tick at 1ms |

### Memory Budget

| Region | Size | Usage |
|--------|------|-------|
| Flash | 16MB | Firmware (~1MB), NVS (64KB), OTA partition |
| PSRAM | 8MB | Available for buffers, future expansion |
| SRAM | 512KB | Task stacks, queues, runtime data |

### Task Stack Allocation

See `config_limits.h` for stack sizes. Total ~32KB for all tasks.

## Deployment Architecture

### Flash Partition Table

```
# Name,   Type, SubType, Offset,   Size,  Flags
nvs,      data, nvs,     0x9000,   0x6000
phy_init, data, phy,     0xf000,   0x1000
factory,  app,  factory, 0x10000,  0x300000
```

### Firmware Update Process

```bash
# Build
idf.py build

# Flash via USB
idf.py -p /dev/ttyACM0 flash

# Monitor
idf.py -p /dev/ttyACM0 monitor
```

### Configuration Deployment

```bash
# Export current config
echo "CFGEXPORT" > /dev/ttyACM0 && cat /dev/ttyACM0 > config_backup.yaml

# Deploy new config
echo "CFGSTART" > /dev/ttyACM0
cat new_config.yaml | while read line; do echo "CFGDATA $line" > /dev/ttyACM0; done
echo "CFGEND APPLY" > /dev/ttyACM0
echo "SAVE 0" > /dev/ttyACM0
```

## Development Environment

### Prerequisites

- ESP-IDF v5.4
- Python 3.8+ (for idf.py, esptool)
- CMake 3.16+
- USB-C cable for flashing and debugging

### Setup Commands

```bash
# Install ESP-IDF (if not already)
mkdir -p ~/esp
cd ~/esp
git clone -b v5.4 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
source export.sh

# Clone project
cd ~/projects
git clone <repository_url> yarobot_control_unit
cd yarobot_control_unit

# Configure
idf.py set-target esp32s3
idf.py menuconfig  # Verify Flash size, PSRAM, USB CDC settings

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/ttyACM0 flash monitor
```

### Debugging

```bash
# OpenOCD for JTAG debugging
idf.py openocd

# GDB attachment
idf.py gdb

# Core dump analysis (if enabled)
idf.py coredump-info
```

## Architecture Decision Records (ADRs)

### ADR-001: ESP32-S3 Platform Selection
**Context:** Need MCU with multiple PWM channels, USB CDC, sufficient GPIO.
**Decision:** ESP32-S3-DevKitC-1 N16R8 (16MB Flash, 8MB PSRAM).
**Rationale:** Native USB (no FTDI), 4 RMT channels, 2 MCPWM groups, ample GPIO, FreeRTOS SMP.

### ADR-002: Layered Component Architecture
**Context:** Need modular design for testability and maintainability.
**Decision:** HAL → Drivers → Control → Interface layering.
**Rationale:** Clear dependencies, unit testable, follows ESP-IDF conventions.

### ADR-003: TPIC6B595N for Shift Registers
**Context:** Need to control DIR/EN/Brake signals at 24V logic levels.
**Decision:** 3x TPIC6B595N (24-bit chain) instead of 74HC595.
**Rationale:** Native 24V open-drain outputs, no level shifters, fail-safe brakes.

### ADR-004: Separate I2C Bus for OLED
**Context:** OLED display sharing I2C with critical limit switches is risky.
**Decision:** OLED on I2C_NUM_1 (GPIO_OLED_SDA/GPIO_OLED_SCL), MCP23017s on I2C_NUM_0 (GPIO_I2C_SDA/GPIO_I2C_SCL).
**Rationale:** Isolation prevents display issues from affecting safety-critical I/O.

### ADR-005: Text Command Protocol
**Context:** Need simple, debuggable host interface.
**Decision:** Human-readable text commands over USB CDC.
**Rationale:** Works with any serial terminal, easy debugging, no special tooling needed.

### ADR-006: Streaming Double-Buffer Pulse Generation
**Context:** Need to support both short point-to-point moves and unlimited-length continuous motion (jogging) without code path divergence. Single pre-computed buffers limit maximum travel distance and require different handling for VEL commands.
**Decision:** All pulse generation (RMT, MCPWM, LEDC) uses streaming double-buffer architecture with DMA callback-driven buffer swap.
**Rationale:**
- **One code path** - Short moves, long moves, and continuous jog all use identical streaming infrastructure
- **Unlimited motion length** - Profile generator streams pulses on-demand; no buffer size limits travel
- **Zero-copy DMA** - CPU fills buffer B while DMA transmits buffer A; no timing jitter
- **Clean abort** - Stop command switches profile to DECEL phase; buffers drain naturally
- **Testability** - Single streaming state machine easier to unit test than conditional paths
**Trade-offs:** Slightly more complex initial implementation; ~4KB RAM per RMT channel for double buffers (acceptable given 8MB PSRAM).

### ADR-007: SI Units Convention (ROS2 REP-103 Compatible)
**Context:** Need consistent unit system across all axes (linear and rotary), host API, configuration, and internal calculations. Must support future ros2_control integration without unit conversion at integration boundary.
**Decision:** All external interfaces use SI base units: meters (linear position), radians (angular position), seconds (time). Derived units follow: m/s, m/s², rad/s, rad/s².
**Rationale:**
- **ROS2 Compatibility** - REP-103 compliance enables direct ros2_control integration
- **Consistency** - Same unit system for linear (X,Y,Z,C,D) and rotary (A,B,E) axes
- **Precision** - Float32 provides sub-micrometer resolution at meter scale
- **Physical intuition** - SI units are unambiguous; no mm vs inch confusion
- **Three-domain separation** - External (SI) → Motor (conversion layer) → Pulse (hardware) keeps each domain clean
**Trade-offs:** Configuration values are small decimals (0.1 m instead of 100 mm), but inline comments in YAML provide human-readable equivalents.

---

_Generated by BMAD Decision Architecture Workflow v1.0_
_Date: 2025-11-29_
_For: Sergey_
