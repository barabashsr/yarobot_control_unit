# Motor Control Class Architecture

## Overview
Object-oriented architecture for ESP32-S3 motor control using ESP-IDF framework. Designed for 6-axis control (4 servo drives with RMT, 2 steppers with MCPWM).

## Base Classes

### MotorInterface (Abstract Base)

```cpp
class MotorInterface {
public:
    // Core motion control
    virtual esp_err_t moveToPosition(float position, float velocity, float acceleration) = 0;
    virtual esp_err_t moveVelocity(float velocity, float acceleration) = 0;
    virtual esp_err_t stop(bool emergency = false) = 0;
    
    // Position management  
    virtual float getCurrentPosition() const = 0;
    virtual float getTargetPosition() const = 0;
    virtual bool isMoving() const = 0;
    virtual bool isAtTarget() const = 0;
    
    // Configuration
    virtual esp_err_t setLimits(float minPos, float maxPos) = 0;
    virtual esp_err_t setPulsesPerUnit(float ppu) = 0;
    virtual esp_err_t setBacklash(float backlash) = 0;
    
    // Calibration
    virtual esp_err_t calibrateHome(CalibrationConfig& config) = 0;
    virtual esp_err_t verifyPosition() = 0;
    
    // Enable/Disable
    virtual esp_err_t enable() = 0;
    virtual esp_err_t disable() = 0;
    virtual bool isEnabled() const = 0;
    
protected:
    gpio_num_t dir_pin_;
    gpio_num_t enable_pin_;
    
    float current_position_ = 0.0f;
    float target_position_ = 0.0f;
    float pulses_per_unit_ = 1000.0f;  // Default: 1000 pulses/mm
    
    float min_limit_ = -1000.0f;
    float max_limit_ = 1000.0f;
    float backlash_compensation_ = 0.0f;
    
    bool is_enabled_ = false;
    bool is_moving_ = false;
};
```

### Axis Class

```cpp
class Axis {
public:
    Axis(std::unique_ptr<MotorInterface> motor,
         float angular_ratio = 1.0f,
         float linear_ratio = 0.0f);
    
    // High-level control
    esp_err_t moveToPosition(float position, float velocity, float acceleration);
    esp_err_t moveLinear(float distance, float velocity, float acceleration);
    esp_err_t moveAngular(float degrees, float velocity, float acceleration);
    
    // Sensor subscription
    void subscribeToSensor(BaseSensor* sensor, std::function<void(SensorEvent)> callback);
    void unsubscribeFromSensor(BaseSensor* sensor);
    
    // Position queries
    float getLinearPosition() const;
    float getAngularPosition() const;
    float getPulsePosition() const;
    
    // Calibration
    esp_err_t calibrate(CalibrationStrategy* strategy);
    
private:
    std::unique_ptr<MotorInterface> motor_;
    float angular_transmission_ratio_;
    float linear_transmission_ratio_;
    
    std::vector<std::pair<BaseSensor*, std::function<void(SensorEvent)>>> sensor_callbacks_;
    
    // Thread safety
    SemaphoreHandle_t mutex_;
};
```

## Concrete Motor Implementations

### RMTMotor (High Precision Servo Control)

```cpp
class RMTMotor : public MotorInterface {
public:
    struct Config {
        gpio_num_t step_pin;
        gpio_num_t dir_pin;
        gpio_num_t enable_pin;
        gpio_num_t pos_complete_pin;
        gpio_num_t z_signal_pin;
        uint8_t rmt_channel;  // 0-3
    };
    
    explicit RMTMotor(const Config& config);
    ~RMTMotor();
    
    // Motion profile management
    esp_err_t loadMotionProfile(const MotionProfile& profile);
    esp_err_t setAccelerationCurve(AccelerationCurve curve);
    
    // Z-signal handling
    esp_err_t enableZSignalTracking();
    int32_t getZSignalCount() const { return z_pulse_count_; }
    bool hasZSignalError() const;
    
    // Position complete monitoring
    bool isPositionComplete() const;
    esp_err_t waitForPositionComplete(uint32_t timeout_ms);
    
private:
    // RMT specific
    rmt_channel_handle_t rmt_channel_;
    rmt_encoder_handle_t pulse_encoder_;
    
    // GPIO pins
    gpio_num_t pos_complete_pin_;
    gpio_num_t z_signal_pin_;
    
    // Z-signal tracking
    volatile int32_t z_pulse_count_ = 0;
    volatile int32_t pulses_since_z_ = 0;
    volatile bool z_signal_error_ = false;
    
    // Motion profile buffer
    std::vector<rmt_symbol_word_t> motion_profile_buffer_;
    
    // ISR handlers
    static void IRAM_ATTR z_signal_isr(void* arg);
    static void IRAM_ATTR pos_complete_isr(void* arg);
    
    // RMT configuration
    esp_err_t configureRMT();
    esp_err_t generatePulseSequence(float distance, float velocity, float accel);
};
```

### MCPWMMotor (Stepper Control)

```cpp
class MCPWMMotor : public MotorInterface {
public:
    struct Config {
        gpio_num_t step_pin;
        gpio_num_t dir_pin;
        gpio_num_t enable_pin;
        mcpwm_unit_t mcpwm_unit;  // 0 or 1
    };
    
    explicit MCPWMMotor(const Config& config);
    ~MCPWMMotor();
    
    // Pulse counting
    int32_t getPulseCount() const;
    esp_err_t resetPulseCount();
    
private:
    // MCPWM handles
    mcpwm_timer_handle_t timer_;
    mcpwm_oper_handle_t operator_;
    mcpwm_cmpr_handle_t comparator_;
    mcpwm_gen_handle_t generator_;
    
    // Pulse counter
    pcnt_unit_handle_t pcnt_unit_;
    
    // Configuration
    esp_err_t configureMCPWM();
    esp_err_t configurePCNT();
    esp_err_t updateFrequency(uint32_t frequency);
};
```

## Sensor Classes

### BaseSensor (Abstract)

```cpp
class BaseSensor {
public:
    enum class EventType {
        RISING_EDGE,
        FALLING_EDGE,
        VALUE_CHANGE,
        THRESHOLD_EXCEEDED
    };
    
    struct SensorEvent {
        EventType type;
        float value;
        uint32_t timestamp;
    };
    
    virtual esp_err_t init() = 0;
    virtual float getValue() const = 0;
    virtual bool getState() const = 0;
    
    // Event registration
    void registerCallback(std::function<void(SensorEvent)> callback);
    
protected:
    std::vector<std::function<void(SensorEvent)>> callbacks_;
    
    void notifyListeners(const SensorEvent& event);
};
```

### EndSwitch

```cpp
class EndSwitch : public BaseSensor {
public:
    EndSwitch(I2CExpander* expander, uint8_t pin, bool normally_open = true);
    
    esp_err_t init() override;
    bool getState() const override;
    float getValue() const override { return getState() ? 1.0f : 0.0f; }
    
    // Debouncing
    void setDebounceTime(uint32_t ms) { debounce_ms_ = ms; }
    
private:
    I2CExpander* expander_;
    uint8_t pin_;
    bool normally_open_;
    uint32_t debounce_ms_ = 10;
    
    // State tracking for edge detection
    bool last_state_ = false;
    uint32_t last_change_time_ = 0;
};
```

## Motion Profiles

### MotionProfile

```cpp
class MotionProfile {
public:
    enum class Type {
        TRAPEZOIDAL,
        S_CURVE,
        CUSTOM
    };
    
    struct Segment {
        float duration_ms;
        float start_velocity;
        float end_velocity;
        float acceleration;
    };
    
    // Profile generation
    static MotionProfile generateTrapezoidal(float distance, float max_vel, float accel);
    static MotionProfile generateSCurve(float distance, float max_vel, float accel, float jerk);
    
    // Access segments
    const std::vector<Segment>& getSegments() const { return segments_; }
    float getTotalTime() const { return total_time_ms_; }
    float getTotalDistance() const { return total_distance_; }
    
private:
    Type type_;
    std::vector<Segment> segments_;
    float total_time_ms_;
    float total_distance_;
};
```

## Calibration Strategies

### CalibrationStrategy (Abstract)

```cpp
class CalibrationStrategy {
public:
    virtual esp_err_t execute(Axis* axis) = 0;
    virtual const char* getName() const = 0;
    
protected:
    // Common calibration utilities
    esp_err_t moveUntilSwitch(Axis* axis, EndSwitch* switch_sensor, float velocity);
    esp_err_t findZSignal(RMTMotor* motor, float search_distance);
};
```

### HomeWithZSignal (Servo Calibration)

```cpp
class HomeWithZSignal : public CalibrationStrategy {
public:
    HomeWithZSignal(EndSwitch* min_switch, float search_velocity = 10.0f);
    
    esp_err_t execute(Axis* axis) override;
    const char* getName() const override { return "Home with Z-Signal"; }
    
private:
    EndSwitch* min_switch_;
    float search_velocity_;
    
    // Steps: Find switch → Back off → Find Z-signal → Set zero
};
```

## System Integration

### MotorController (System Manager)

```cpp
class MotorController {
public:
    static MotorController& getInstance();
    
    // Axis management
    esp_err_t addAxis(uint8_t id, std::unique_ptr<Axis> axis);
    Axis* getAxis(uint8_t id);
    
    // System control
    esp_err_t emergencyStop();
    esp_err_t enableAll();
    esp_err_t disableAll();
    
    // Coordinated motion
    esp_err_t moveMultipleAxes(const std::vector<AxisMove>& moves);
    
    // Configuration
    esp_err_t loadConfig(const char* filename);
    esp_err_t saveConfig(const char* filename);
    
private:
    MotorController() = default;
    
    std::map<uint8_t, std::unique_ptr<Axis>> axes_;
    
    // Thread for motion coordination
    TaskHandle_t motion_task_;
    QueueHandle_t command_queue_;
    
    // Safety
    volatile bool emergency_stop_active_ = false;
};
```

## Usage Example

```cpp
// System initialization
void initMotorSystem() {
    auto& controller = MotorController::getInstance();
    
    // Create servo motor with RMT
    RMTMotor::Config servo_config{
        .step_pin = GPIO_NUM_1,
        .dir_pin = GPIO_NUM_6,
        .enable_pin = GPIO_NUM_10,
        .pos_complete_pin = GPIO_NUM_14,
        .z_signal_pin = GPIO_NUM_18,
        .rmt_channel = 0
    };
    
    auto servo = std::make_unique<RMTMotor>(servo_config);
    
    // Create axis with 5:1 reduction, 2mm/rev lead screw
    auto axis = std::make_unique<Axis>(
        std::move(servo),
        5.0f,     // angular ratio
        2.0f/5.0f // linear ratio (2mm per motor rev / 5:1)
    );
    
    // Add end switches
    auto min_switch = new EndSwitch(i2c_expander1, 0);
    axis->subscribeToSensor(min_switch, [](BaseSensor::SensorEvent event) {
        if (event.type == BaseSensor::EventType::RISING_EDGE) {
            ESP_LOGI(TAG, "Min limit reached");
        }
    });
    
    // Register axis
    controller.addAxis(0, std::move(axis));
    
    // Calibrate
    HomeWithZSignal calibration(min_switch);
    controller.getAxis(0)->calibrate(&calibration);
}
```

## Thread Safety

All public methods use FreeRTOS mutexes for thread safety. Motion commands are queued and processed by a dedicated high-priority task. Interrupt handlers use minimal processing with deferred handling in tasks.