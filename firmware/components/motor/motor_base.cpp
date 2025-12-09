/**
 * @file motor_base.cpp
 * @brief Abstract base class implementation for motor control
 * @author YaRobot Team
 * @date 2025
 *
 * @note Contains ~90% of motor implementation shared by ServoMotor and StepperMotor.
 *       Type-specific behavior is delegated to virtual hooks.
 */

#include "motor_base.h"
#include "config_timing.h"
#include "config_defaults.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <algorithm>
#include <cinttypes>

static const char* TAG = "motor_base";

MotorBase::MotorBase(IPulseGenerator* pulse_gen,
                     IPositionTracker* position_tracker,
                     uint8_t axis_id,
                     const AxisConfig& config)
    : pulse_gen_(pulse_gen)
    , position_tracker_(position_tracker)
    , axis_id_(axis_id)
    , config_(config)
    , pulse_count_(0)
    , state_(AXIS_STATE_DISABLED)
    , initialized_(false)
    , motion_complete_cb_(nullptr)
{
}

esp_err_t MotorBase::init()
{
    if (initialized_) {
        ESP_LOGW(TAG, "Axis %d already initialized", axis_id_);
        return ESP_ERR_INVALID_STATE;
    }

    // Validate dependencies
    if (pulse_gen_ == nullptr) {
        ESP_LOGE(TAG, "Axis %d: pulse generator is null", axis_id_);
        return ESP_ERR_INVALID_ARG;
    }

    if (position_tracker_ == nullptr) {
        ESP_LOGE(TAG, "Axis %d: position tracker is null", axis_id_);
        return ESP_ERR_INVALID_ARG;
    }

    // Validate configuration
    if (config_.pulses_per_rev <= 0 || config_.units_per_rev <= 0) {
        ESP_LOGE(TAG, "Axis %d: invalid pulses_per_rev or units_per_rev", axis_id_);
        return ESP_ERR_INVALID_ARG;
    }

    if (config_.limit_min >= config_.limit_max) {
        ESP_LOGE(TAG, "Axis %d: invalid limits (min >= max)", axis_id_);
        return ESP_ERR_INVALID_ARG;
    }

    if (config_.max_velocity <= 0) {
        ESP_LOGE(TAG, "Axis %d: invalid max_velocity", axis_id_);
        return ESP_ERR_INVALID_ARG;
    }

    if (config_.max_acceleration <= 0) {
        ESP_LOGE(TAG, "Axis %d: invalid max_acceleration", axis_id_);
        return ESP_ERR_INVALID_ARG;
    }

    // Connect position tracker to pulse generator
    pulse_gen_->setPositionTracker(position_tracker_);

    // Register our completion callback with pulse generator
    pulse_gen_->setCompletionCallback(
        [this](int64_t total_pulses) {
            this->onMotionComplete(total_pulses);
        });

    // Reset position tracker to zero
    esp_err_t err = position_tracker_->reset(0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d: failed to reset position tracker", axis_id_);
        return err;
    }

    pulse_count_.store(0, std::memory_order_release);
    state_.store(getInitialState(), std::memory_order_release);
    initialized_ = true;

    ESP_LOGI(TAG, "Axis %d initialized (%.3f pulses/unit)",
             axis_id_, config_.getPulsesPerUnit());

    return ESP_OK;
}

esp_err_t MotorBase::moveAbsolute(float position, float velocity)
{
    ESP_LOGW(TAG, "DEBUG Axis %d: moveAbsolute(pos=%.4f, vel=%.4f)", axis_id_, position, velocity);

    // Check state
    AxisState current_state = state_.load(std::memory_order_acquire);
    if (current_state == AXIS_STATE_DISABLED) {
        ESP_LOGW(TAG, "Axis %d: moveAbsolute rejected - axis disabled (state=%d)", axis_id_, current_state);
        return ESP_ERR_INVALID_STATE;
    }

    // Validate position against soft limits
    if (position < config_.limit_min || position > config_.limit_max) {
        ESP_LOGW(TAG, "Axis %d: position %.4f out of limits [%.4f, %.4f]",
                 axis_id_, position, config_.limit_min, config_.limit_max);
        return ESP_ERR_INVALID_ARG;
    }

    // Clamp velocity to max
    velocity = std::fabs(velocity);
    velocity = std::min(velocity, config_.max_velocity);
    ESP_LOGW(TAG, "DEBUG Axis %d: clamped velocity=%.4f (max=%.4f)", axis_id_, velocity, config_.max_velocity);

    // Get current position in SI units
    float current_pos = getPosition();
    ESP_LOGW(TAG, "DEBUG Axis %d: current_pos=%.4f, target_pos=%.4f", axis_id_, current_pos, position);

    // Calculate delta and direction
    float delta = position - current_pos;
    bool forward = (delta >= 0);

    // Handle zero-distance move
    if (std::fabs(delta) < (1.0f / config_.getPulsesPerUnit())) {
        ESP_LOGW(TAG, "DEBUG Axis %d: target position already reached (delta=%.6f)", axis_id_, delta);
        return ESP_OK;
    }

    // Set direction via shift register
    ESP_LOGW(TAG, "DEBUG Axis %d: setting direction=%s", axis_id_, forward ? "FWD" : "REV");
    esp_err_t err = setDirection(forward);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d: failed to set direction: %s", axis_id_, esp_err_to_name(err));
        return err;
    }

    // Wait direction setup time
    waitDirectionSetup();

    // Convert to pulse domain
    float pulses_per_unit = config_.getPulsesPerUnit();
    int32_t pulses = static_cast<int32_t>(std::fabs(delta) * pulses_per_unit);
    float freq_hz = velocityToFrequency(velocity);
    float accel_pulses = accelerationToPulses(config_.max_acceleration);

    ESP_LOGW(TAG, "DEBUG Axis %d: pulses_per_unit=%.2f, pulses=%ld, freq_hz=%.1f, accel=%.1f",
             axis_id_, pulses_per_unit, (long)pulses, freq_hz, accel_pulses);

    // Set position tracker direction
    position_tracker_->setDirection(forward);

    // Start motion (blending handled by pulse generator)
    ESP_LOGW(TAG, "DEBUG Axis %d: calling pulse_gen_->startMove(pulses=%ld, freq=%.1f, accel=%.1f)",
             axis_id_, (long)(forward ? pulses : -pulses), freq_hz, accel_pulses);
    err = pulse_gen_->startMove(forward ? pulses : -pulses, freq_hz, accel_pulses);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d: startMove failed: %s", axis_id_, esp_err_to_name(err));
        return err;
    }

    // Update state
    state_.store(AXIS_STATE_MOVING, std::memory_order_release);

    ESP_LOGW(TAG, "DEBUG Axis %d: moveAbsolute completed, state=MOVING", axis_id_);

    return ESP_OK;
}

esp_err_t MotorBase::moveRelative(float delta, float velocity)
{
    // Delegate to moveAbsolute
    float target = getPosition() + delta;
    return moveAbsolute(target, velocity);
}

esp_err_t MotorBase::moveVelocity(float velocity)
{
    ESP_LOGW(TAG, "DEBUG Axis %d: moveVelocity(vel=%.4f)", axis_id_, velocity);

    // Check state
    AxisState current_state = state_.load(std::memory_order_acquire);
    if (current_state == AXIS_STATE_DISABLED) {
        ESP_LOGW(TAG, "Axis %d: moveVelocity rejected - axis disabled (state=%d)", axis_id_, current_state);
        return ESP_ERR_INVALID_STATE;
    }

    // Clamp velocity to +-max_velocity
    float sign = (velocity >= 0) ? 1.0f : -1.0f;
    velocity = std::min(std::fabs(velocity), config_.max_velocity) * sign;
    ESP_LOGW(TAG, "DEBUG Axis %d: clamped velocity=%.4f (max=%.4f)", axis_id_, velocity, config_.max_velocity);

    bool forward = (velocity >= 0);

    // Set direction via shift register
    ESP_LOGW(TAG, "DEBUG Axis %d: setting direction=%s", axis_id_, forward ? "FWD" : "REV");
    esp_err_t err = setDirection(forward);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d: failed to set direction for velocity mode: %s", axis_id_, esp_err_to_name(err));
        return err;
    }

    // Wait direction setup time
    waitDirectionSetup();

    // Convert to pulse domain
    float freq_hz = velocityToFrequency(std::fabs(velocity));
    float accel_pulses = accelerationToPulses(config_.max_acceleration);

    ESP_LOGW(TAG, "DEBUG Axis %d: freq_hz=%.1f, accel=%.1f", axis_id_, freq_hz, accel_pulses);

    // Set position tracker direction
    position_tracker_->setDirection(forward);

    // Start velocity mode
    ESP_LOGW(TAG, "DEBUG Axis %d: calling pulse_gen_->startVelocity(freq=%.1f, accel=%.1f)",
             axis_id_, forward ? freq_hz : -freq_hz, accel_pulses);
    err = pulse_gen_->startVelocity(forward ? freq_hz : -freq_hz, accel_pulses);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d: startVelocity failed: %s", axis_id_, esp_err_to_name(err));
        return err;
    }

    // Update state
    state_.store(AXIS_STATE_MOVING, std::memory_order_release);

    ESP_LOGW(TAG, "DEBUG Axis %d: moveVelocity completed, state=MOVING", axis_id_);

    return ESP_OK;
}

esp_err_t MotorBase::stop()
{
    // Check if moving
    if (!isMoving()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Stop with deceleration
    float accel_pulses = accelerationToPulses(config_.max_acceleration);
    esp_err_t err = pulse_gen_->stop(accel_pulses);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d: stop failed", axis_id_);
        return err;
    }

    // State transition happens in onMotionComplete callback
    ESP_LOGD(TAG, "Axis %d: stop initiated with deceleration", axis_id_);

    return ESP_OK;
}

void MotorBase::stopImmediate()
{
    // Emergency stop - no deceleration
    pulse_gen_->stopImmediate();

    // Immediately transition to IDLE
    AxisState current = state_.load(std::memory_order_acquire);
    if (current == AXIS_STATE_MOVING) {
        state_.store(AXIS_STATE_IDLE, std::memory_order_release);
    }

    // Sync position from tracker (subclass-specific)
    syncPositionFromTracker();

    ESP_LOGD(TAG, "Axis %d: stopImmediate executed", axis_id_);
}

float MotorBase::getPosition() const
{
    // During motion, query position tracker for real-time position
    // This ensures POS commands return live position during MOVE/VEL
    if (isMoving() && position_tracker_) {
        int64_t live_pulses = position_tracker_->getPosition();
        return static_cast<float>(live_pulses) / config_.getPulsesPerUnit();
    }

    // When idle, use cached pulse_count_ (synced from tracker at completion)
    int64_t pulses = pulse_count_.load(std::memory_order_acquire);
    return static_cast<float>(pulses) / config_.getPulsesPerUnit();
}

float MotorBase::getVelocity() const
{
    if (!isMoving()) {
        return 0.0f;
    }

    float freq_hz = pulse_gen_->getCurrentVelocity();
    return freq_hz / config_.getPulsesPerUnit();
}

bool MotorBase::isMoving() const
{
    return state_.load(std::memory_order_acquire) == AXIS_STATE_MOVING;
}

bool MotorBase::isEnabled() const
{
    return state_.load(std::memory_order_acquire) != AXIS_STATE_DISABLED;
}

esp_err_t MotorBase::enable(bool en)
{
    esp_err_t err;

    if (en) {
        // Enable motor
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

        // Call subclass hook
        onEnableHook(true);

        // Transition state
        AxisState current = state_.load(std::memory_order_acquire);
        if (current == AXIS_STATE_DISABLED) {
            // Go to UNHOMED after enable (position unknown)
            state_.store(AXIS_STATE_UNHOMED, std::memory_order_release);
        }

        ESP_LOGI(TAG, "Axis %d enabled", axis_id_);
    } else {
        // Disable motor
        // Stop any motion first
        if (isMoving()) {
            stopImmediate();
        }

        // Call subclass hook before disabling
        onEnableHook(false);

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

AxisState MotorBase::getState() const
{
    return state_.load(std::memory_order_acquire);
}

const AxisConfig& MotorBase::getConfig() const
{
    return config_;
}

esp_err_t MotorBase::setConfig(const AxisConfig& config)
{
    // Validate new config
    if (config.pulses_per_rev <= 0 || config.units_per_rev <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config.limit_min >= config.limit_max) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config.max_velocity <= 0 || config.max_acceleration <= 0) {
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

void MotorBase::setMotionCompleteCallback(MotionCompleteCallback cb)
{
    motion_complete_cb_ = cb;
}

void MotorBase::onMotionComplete(int64_t total_pulses)
{
    ESP_LOGW(TAG, "DEBUG Axis %d: onMotionComplete(total_pulses=%lld)", axis_id_, (long long)total_pulses);

    // Debug: position before sync
    int64_t pulses_before = pulse_count_.load(std::memory_order_acquire);
    int64_t tracker_pos = position_tracker_->getPosition();
    ESP_LOGW(TAG, "DEBUG Axis %d: before sync: pulse_count=%lld, tracker_pos=%lld",
             axis_id_, (long long)pulses_before, (long long)tracker_pos);

    // Sync position (subclass-specific strategy)
    syncPositionFromTracker();

    // Debug: position after sync
    int64_t pulses_after = pulse_count_.load(std::memory_order_acquire);
    ESP_LOGW(TAG, "DEBUG Axis %d: after sync: pulse_count=%lld", axis_id_, (long long)pulses_after);

    // Transition to IDLE
    state_.store(AXIS_STATE_IDLE, std::memory_order_release);

    // Invoke user callback if registered
    if (motion_complete_cb_) {
        float position_si = getPosition();
        motion_complete_cb_(axis_id_, position_si);
    }

    ESP_LOGW(TAG, "Axis %d: motion complete at position %.4f",
             axis_id_, getPosition());
}

float MotorBase::velocityToFrequency(float velocity_si) const
{
    // velocity (m/s) * (pulses/m) = pulses/s = Hz
    return velocity_si * config_.getPulsesPerUnit();
}

float MotorBase::accelerationToPulses(float accel_si) const
{
    // accel (m/s^2) * (pulses/m) = pulses/s^2
    return accel_si * config_.getPulsesPerUnit();
}

esp_err_t MotorBase::setDirection(bool forward)
{
    esp_err_t err = sr_set_direction(axis_id_, forward);
    if (err != ESP_OK) {
        return err;
    }

    return sr_update();
}

void MotorBase::waitDirectionSetup()
{
    // Use esp_rom_delay_us for microsecond delay
    esp_rom_delay_us(TIMING_DIR_SETUP_US);
}

void MotorBase::waitEnableDelay()
{
    // Use esp_rom_delay_us for microsecond delay
    esp_rom_delay_us(TIMING_ENABLE_DELAY_US);
}
