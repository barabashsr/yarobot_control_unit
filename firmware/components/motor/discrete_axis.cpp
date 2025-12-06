/**
 * @file discrete_axis.cpp
 * @brief Discrete axis implementation for E axis (pneumatic cylinder/solenoid)
 * @author YaRobot Team
 * @date 2025
 *
 * Implements IMotor for discrete actuators with binary positions (0=retracted, 1=extended).
 * Motion is time-based using TimeTracker and esp_timer for completion detection.
 */

#include "discrete_axis.h"
#include "config_timing.h"
#include "config_defaults.h"
#include "config_sr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>
#include <algorithm>

static const char* TAG = "discrete_axis";

// Position tolerance for "already at target" check
static constexpr float POSITION_TOLERANCE = 0.01f;

// Microseconds per millisecond for timer conversion
static constexpr int64_t US_PER_MS = 1000;

// Position midpoint for binary threshold determination
static constexpr float POSITION_MIDPOINT = (E_AXIS_LIMIT_MIN + E_AXIS_LIMIT_MAX) / 2.0f;

// Binary position values for TimeTracker (0 = retracted, 1 = extended)
static constexpr int64_t BINARY_POSITION_RETRACTED = 0;
static constexpr int64_t BINARY_POSITION_EXTENDED = 1;

// ============================================================================
// Constructor / Destructor
// ============================================================================

DiscreteAxis::DiscreteAxis(TimeTracker* time_tracker,
                           uint8_t axis_id,
                           const AxisConfig& config)
    : time_tracker_(time_tracker)
    , axis_id_(axis_id)
    , config_(config)
    , state_(AXIS_STATE_DISABLED)
    , target_position_(0.0f)
    , current_position_(0.0f)
    , motion_timer_(nullptr)
    , initialized_(false)
    , motion_complete_cb_(nullptr)
{
}

DiscreteAxis::~DiscreteAxis()
{
    // Clean up timer if it exists
    if (motion_timer_ != nullptr) {
        esp_timer_stop(motion_timer_);
        esp_timer_delete(motion_timer_);
        motion_timer_ = nullptr;
    }
}

// ============================================================================
// Initialization (Task 2)
// ============================================================================

esp_err_t DiscreteAxis::init()
{
    if (initialized_) {
        ESP_LOGW(TAG, "Axis %d already initialized", axis_id_);
        return ESP_ERR_INVALID_STATE;
    }

    // Validate dependencies
    if (time_tracker_ == nullptr) {
        ESP_LOGE(TAG, "Axis %d: time tracker is null", axis_id_);
        return ESP_ERR_INVALID_ARG;
    }

    // Validate configuration
    if (config_.limit_min >= config_.limit_max) {
        ESP_LOGE(TAG, "Axis %d: invalid limits (min >= max)", axis_id_);
        return ESP_ERR_INVALID_ARG;
    }

    if (config_.max_velocity <= 0) {
        ESP_LOGE(TAG, "Axis %d: invalid max_velocity", axis_id_);
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize time tracker
    esp_err_t err = time_tracker_->init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d: failed to init time tracker", axis_id_);
        return err;
    }

    // Create esp_timer for motion completion detection
    esp_timer_create_args_t timer_args = {
        .callback = motionTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "discrete_motion",
        .skip_unhandled_events = false
    };

    err = esp_timer_create(&timer_args, &motion_timer_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d: failed to create motion timer", axis_id_);
        return err;
    }

    // Initialize state
    current_position_.store(E_AXIS_LIMIT_MIN, std::memory_order_release);
    target_position_.store(E_AXIS_LIMIT_MIN, std::memory_order_release);
    state_.store(AXIS_STATE_UNHOMED, std::memory_order_release);
    initialized_ = true;

    ESP_LOGI(TAG, "Axis %d initialized (discrete actuator, travel time %d ms)",
             axis_id_, TIMING_E_AXIS_TRAVEL_MS);

    return ESP_OK;
}

// ============================================================================
// Motion Commands (Tasks 3-5)
// ============================================================================

esp_err_t DiscreteAxis::moveAbsolute(float position, float velocity)
{
    (void)velocity;  // Discrete actuator has fixed speed

    // Check state (AC13)
    AxisState current_state = state_.load(std::memory_order_acquire);
    if (current_state == AXIS_STATE_DISABLED) {
        ESP_LOGW(TAG, "Axis %d: moveAbsolute rejected - axis disabled", axis_id_);
        return ESP_ERR_INVALID_STATE;
    }

    // Validate position against limits (AC12)
    if (position < E_AXIS_LIMIT_MIN || position > E_AXIS_LIMIT_MAX) {
        ESP_LOGW(TAG, "Axis %d: position %.2f out of limits [%.2f, %.2f]",
                 axis_id_, position, E_AXIS_LIMIT_MIN, E_AXIS_LIMIT_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    // Get current position
    float current_pos = current_position_.load(std::memory_order_acquire);

    // Check if already at target position (AC4)
    if (std::fabs(position - current_pos) < POSITION_TOLERANCE) {
        ESP_LOGD(TAG, "Axis %d: already at target position %.2f", axis_id_, position);
        // Invoke callback immediately for already-at-position case
        if (motion_complete_cb_) {
            motion_complete_cb_(axis_id_, current_pos);
        }
        return ESP_OK;
    }

    // Determine direction: forward = extend (0→1), reverse = retract (1→0)
    bool forward = (position > current_pos);

    // Set direction via shift register (AC2, AC3)
    esp_err_t err = sr_set_direction(axis_id_, forward);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d: failed to set direction", axis_id_);
        return err;
    }

    err = sr_update();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d: failed to update shift register", axis_id_);
        return err;
    }

    // Set time tracker direction and start motion
    time_tracker_->setDirection(forward);
    time_tracker_->startMotion();

    // Store target position
    target_position_.store(position, std::memory_order_release);

    // Update state to MOVING (AC2)
    state_.store(AXIS_STATE_MOVING, std::memory_order_release);

    // Start motion completion timer (convert ms to us)
    uint64_t timeout_us = static_cast<uint64_t>(TIMING_E_AXIS_TRAVEL_MS) * US_PER_MS;
    err = esp_timer_start_once(motion_timer_, timeout_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d: failed to start motion timer", axis_id_);
        state_.store(AXIS_STATE_IDLE, std::memory_order_release);
        return err;
    }

    ESP_LOGD(TAG, "Axis %d: moveAbsolute to %.2f (%s)",
             axis_id_, position, forward ? "extend" : "retract");

    return ESP_OK;
}

esp_err_t DiscreteAxis::moveRelative(float delta, float velocity)
{
    // Delegate to moveAbsolute
    float current_pos = current_position_.load(std::memory_order_acquire);
    float target = current_pos + delta;
    return moveAbsolute(target, velocity);
}

esp_err_t DiscreteAxis::moveVelocity(float velocity)
{
    (void)velocity;

    // Discrete actuators do not support velocity mode (AC5)
    ESP_LOGW(TAG, "Axis %d: DiscreteAxis does not support velocity mode", axis_id_);
    return ESP_ERR_NOT_SUPPORTED;
}

// ============================================================================
// Stop Commands (Task 6)
// ============================================================================

esp_err_t DiscreteAxis::stop()
{
    // Check if moving
    if (!isMoving()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Cancel motion timer (AC6)
    esp_timer_stop(motion_timer_);

    // Get interpolated position from time tracker
    int64_t pos_int = time_tracker_->getPosition();
    float pos = (pos_int > 0) ? E_AXIS_LIMIT_MAX : E_AXIS_LIMIT_MIN;
    current_position_.store(pos, std::memory_order_release);

    // Transition to IDLE
    state_.store(AXIS_STATE_IDLE, std::memory_order_release);

    // Invoke callback with interpolated position
    if (motion_complete_cb_) {
        motion_complete_cb_(axis_id_, pos);
    }

    ESP_LOGD(TAG, "Axis %d: stop executed, position %.2f", axis_id_, pos);

    return ESP_OK;
}

void DiscreteAxis::stopImmediate()
{
    // Cancel motion timer (AC7)
    if (motion_timer_ != nullptr) {
        esp_timer_stop(motion_timer_);
    }

    // Clear EN via shift register
    sr_set_enable(axis_id_, false);
    sr_update();

    // Transition to IDLE
    AxisState current = state_.load(std::memory_order_acquire);
    if (current == AXIS_STATE_MOVING) {
        state_.store(AXIS_STATE_IDLE, std::memory_order_release);
    }

    ESP_LOGD(TAG, "Axis %d: stopImmediate executed", axis_id_);
}

// ============================================================================
// Status Methods (Task 7)
// ============================================================================

float DiscreteAxis::getPosition() const
{
    if (!initialized_) {
        return E_AXIS_LIMIT_MIN;
    }

    // If moving, return interpolated position from time tracker (AC8)
    if (isMoving()) {
        int64_t pos_int = time_tracker_->getPosition();
        // TimeTracker returns 0 or 1, convert to float
        return (pos_int > 0) ? E_AXIS_LIMIT_MAX : E_AXIS_LIMIT_MIN;
    }

    return current_position_.load(std::memory_order_acquire);
}

float DiscreteAxis::getVelocity() const
{
    // Return fixed velocity if moving, 0.0 if idle (AC9)
    if (isMoving()) {
        return E_AXIS_MAX_VELOCITY;
    }
    return 0.0f;
}

bool DiscreteAxis::isMoving() const
{
    return state_.load(std::memory_order_acquire) == AXIS_STATE_MOVING;
}

bool DiscreteAxis::isEnabled() const
{
    return state_.load(std::memory_order_acquire) != AXIS_STATE_DISABLED;
}

AxisState DiscreteAxis::getState() const
{
    return state_.load(std::memory_order_acquire);
}

// ============================================================================
// Enable/Disable (Task 8)
// ============================================================================

esp_err_t DiscreteAxis::enable(bool en)
{
    esp_err_t err;

    if (en) {
        // Enable motor (AC10)
        err = sr_set_enable(axis_id_, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Axis %d: failed to set enable", axis_id_);
            return err;
        }

        err = sr_update();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Axis %d: failed to update shift register", axis_id_);
            return err;
        }

        // Wait enable delay
        waitEnableDelay();

        // Transition state
        AxisState current = state_.load(std::memory_order_acquire);
        if (current == AXIS_STATE_DISABLED) {
            state_.store(AXIS_STATE_IDLE, std::memory_order_release);
        }

        ESP_LOGI(TAG, "Axis %d enabled", axis_id_);
    } else {
        // Disable motor (AC11)
        // Stop any motion first
        if (isMoving()) {
            stopImmediate();
        }

        err = sr_set_enable(axis_id_, false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Axis %d: failed to clear enable", axis_id_);
            return err;
        }

        err = sr_update();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Axis %d: failed to update shift register", axis_id_);
            return err;
        }

        state_.store(AXIS_STATE_DISABLED, std::memory_order_release);

        ESP_LOGI(TAG, "Axis %d disabled", axis_id_);
    }

    return ESP_OK;
}

// ============================================================================
// Configuration Methods (Task 10)
// ============================================================================

const AxisConfig& DiscreteAxis::getConfig() const
{
    return config_;
}

esp_err_t DiscreteAxis::setConfig(const AxisConfig& config)
{
    // Validate new config
    if (config.limit_min >= config.limit_max) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config.max_velocity <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Can't change config while moving
    if (isMoving()) {
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;
    ESP_LOGD(TAG, "Axis %d config updated", axis_id_);

    return ESP_OK;
}

void DiscreteAxis::setMotionCompleteCallback(MotionCompleteCallback cb)
{
    motion_complete_cb_ = cb;
}

// ============================================================================
// Motion Completion (Task 9)
// ============================================================================

void DiscreteAxis::motionTimerCallback(void* arg)
{
    DiscreteAxis* self = static_cast<DiscreteAxis*>(arg);
    if (self != nullptr) {
        self->onMotionComplete();
    }
}

void DiscreteAxis::onMotionComplete()
{
    // Get target position
    float target = target_position_.load(std::memory_order_acquire);

    // Update current position to target (AC2, AC3)
    current_position_.store(target, std::memory_order_release);

    // Reset time tracker to final position (extended if above midpoint, retracted otherwise)
    time_tracker_->reset(target > POSITION_MIDPOINT ? BINARY_POSITION_EXTENDED : BINARY_POSITION_RETRACTED);

    // Transition to IDLE
    state_.store(AXIS_STATE_IDLE, std::memory_order_release);

    // Invoke user callback (EVT_MOTION_COMPLETE equivalent)
    if (motion_complete_cb_) {
        motion_complete_cb_(axis_id_, target);
    }

    ESP_LOGD(TAG, "Axis %d: motion complete at position %.2f", axis_id_, target);
}

void DiscreteAxis::waitEnableDelay()
{
    // Use esp_rom_delay_us for microsecond delay
    esp_rom_delay_us(TIMING_ENABLE_DELAY_US);
}
