/**
 * @file motion_controller.cpp
 * @brief Motion controller implementation
 * @author YaRobot Team
 * @date 2025
 */

#include "motion_controller.h"
#include "event_manager.h"
#include "response_formatter.h"
#include "config_defaults.h"
#include "config_axes.h"
#include "config_limits.h"

#include "esp_log.h"
#include "esp_timer.h"
#include <cctype>

static const char* TAG = "MOTION_CTRL";

/* ==========================================================================
 * Singleton Instance
 * ========================================================================== */

/** @brief Global singleton instance */
static MotionController s_motion_controller;

MotionController* getMotionController()
{
    return &s_motion_controller;
}

/* ==========================================================================
 * MotionController Implementation
 * ========================================================================== */

MotionController::MotionController()
    : initialized_(false)
{
    // Initialize all motor pointers to nullptr
    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        motors_[i] = nullptr;
    }
}

esp_err_t MotionController::init(IMotor* motors[LIMIT_NUM_AXES])
{
    if (initialized_) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (motors == nullptr) {
        ESP_LOGE(TAG, "Motors array is null");
        return ESP_ERR_INVALID_ARG;
    }

    // Validate and store all motor pointers
    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        if (motors[i] == nullptr) {
            ESP_LOGE(TAG, "Motor %d is null", i);
            return ESP_ERR_INVALID_ARG;
        }
        motors_[i] = motors[i];
    }

    // Register motion complete callback on each motor
    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        // Capture axis index in lambda
        const uint8_t axis = i;
        motors_[i]->setMotionCompleteCallback(
            [axis](uint8_t /*unused*/, float position) {
                onMotionComplete(axis, position);
            }
        );
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Initialized with %d axes", LIMIT_NUM_AXES);
    return ESP_OK;
}

IMotor* MotionController::getMotor(uint8_t axis_id) const
{
    if (!initialized_) {
        return nullptr;
    }

    if (axis_id >= LIMIT_NUM_AXES) {
        return nullptr;
    }

    return motors_[axis_id];
}

IMotor* MotionController::getMotor(char axis_char) const
{
    if (!initialized_) {
        return nullptr;
    }

    // Convert to uppercase for case-insensitive comparison
    char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(axis_char)));

    // Map character to axis ID using config_axes.h constants
    uint8_t axis_id;
    switch (upper) {
        case AXIS_CHAR_X: axis_id = AXIS_X; break;
        case AXIS_CHAR_Y: axis_id = AXIS_Y; break;
        case AXIS_CHAR_Z: axis_id = AXIS_Z; break;
        case AXIS_CHAR_A: axis_id = AXIS_A; break;
        case AXIS_CHAR_B: axis_id = AXIS_B; break;
        case AXIS_CHAR_C: axis_id = AXIS_C; break;
        case AXIS_CHAR_D: axis_id = AXIS_D; break;
        case AXIS_CHAR_E: axis_id = AXIS_E; break;
        default:
            return nullptr;  // Invalid axis character
    }

    return motors_[axis_id];
}

esp_err_t MotionController::moveAbsolute(uint8_t axis, float position, float velocity)
{
    if (!initialized_) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate axis ID (AC12)
    if (axis >= LIMIT_NUM_AXES) {
        ESP_LOGE(TAG, "Invalid axis: %d", axis);
        return ESP_ERR_INVALID_ARG;
    }

    IMotor* motor = motors_[axis];
    if (motor == nullptr) {
        ESP_LOGE(TAG, "Motor %d is null", axis);
        return ESP_ERR_INVALID_ARG;
    }

    // Check if motor is enabled (AC6)
    if (!motor->isEnabled()) {
        ESP_LOGD(TAG, "Axis %d not enabled", axis);
        return ESP_ERR_INVALID_STATE;
    }

    // Use default velocity if not specified (AC10)
    float move_velocity = velocity;
    if (move_velocity <= 0.0f) {
        move_velocity = DEFAULT_MAX_VELOCITY;
    }

    // Position limit check is handled by motor->moveAbsolute() (AC7)
    // Motor returns ESP_ERR_INVALID_ARG if position exceeds limits

    // Delegate to motor (AC1, AC2, AC9 - motion blending handled by motor)
    esp_err_t ret = motor->moveAbsolute(position, move_velocity);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "moveAbsolute failed for axis %d: 0x%x", axis, ret);
    }

    return ret;
}

esp_err_t MotionController::moveRelative(uint8_t axis, float delta, float velocity)
{
    if (!initialized_) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate axis ID
    if (axis >= LIMIT_NUM_AXES) {
        ESP_LOGE(TAG, "Invalid axis: %d", axis);
        return ESP_ERR_INVALID_ARG;
    }

    IMotor* motor = motors_[axis];
    if (motor == nullptr) {
        ESP_LOGE(TAG, "Motor %d is null", axis);
        return ESP_ERR_INVALID_ARG;
    }

    // Get current position and calculate target (AC3, AC4)
    float current = motor->getPosition();
    float target = current + delta;

    // Delegate to moveAbsolute
    return moveAbsolute(axis, target, velocity);
}

void MotionController::onMotionComplete(uint8_t axis, float position)
{
    ESP_LOGD(TAG, "Motion complete: axis=%d pos=%.3f", axis, position);

    // Publish EVT_MOTION_COMPLETE event (AC13)
    Event event = {
        .type = EVTTYPE_MOTION_COMPLETE,
        .axis = axis,
        .data = { .position = position },
        .timestamp = esp_timer_get_time()
    };

    esp_err_t ret = event_publish(&event);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish motion complete event: 0x%x", ret);
    }
}
