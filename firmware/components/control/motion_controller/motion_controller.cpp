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
#include "config_commands.h"
#include "safety_monitor.h"
#include "floating_switch.h"  // Story 4-9: C axis width measurement
#include "usb_cdc.h"          // For EVENT DONE messages to host

#include "esp_log.h"
#include "esp_timer.h"
#include <cctype>
#include <cmath>

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

    // Story 4-3: Check if axis is faulted (both limits active)
    if (safety_monitor_is_axis_faulted(axis)) {
        ESP_LOGW(TAG, "Axis %d is faulted (both limits active)", axis);
        return ESP_ERR_INVALID_STATE;
    }

    // Story 4-3: Check direction blocking by active limit switches
    float current = motor->getPosition();
    float delta = position - current;
    int8_t direction = 0;
    if (std::fabs(delta) > 0.0001f) {  // Non-zero movement
        direction = (delta > 0) ? 1 : -1;
        if (safety_monitor_is_direction_blocked(axis, direction)) {
            ESP_LOGW(TAG, "Axis %d direction %+d blocked by limit", axis, direction);
            return ESP_ERR_INVALID_STATE;
        }
    }

    // Story 4-9: Notify floating switch handler when C axis starts moving
    // This enables width reset on new closing motion (AC6)
    if (axis == AXIS_C && direction != 0) {
        floating_switch_on_motion_start(direction);
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

esp_err_t MotionController::setAxisEnabled(uint8_t axis, bool enable)
{
    if (!initialized_) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (axis >= LIMIT_NUM_AXES) {
        ESP_LOGE(TAG, "Invalid axis: %d", axis);
        return ESP_ERR_INVALID_ARG;
    }

    IMotor* motor = motors_[axis];
    if (motor == nullptr) {
        ESP_LOGE(TAG, "Motor %d is null", axis);
        return ESP_ERR_INVALID_ARG;
    }

    return motor->enable(enable);
}

esp_err_t MotionController::getAxisPosition(uint8_t axis, float* position)
{
    if (!initialized_) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (axis >= LIMIT_NUM_AXES || position == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    IMotor* motor = motors_[axis];
    if (motor == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    *position = motor->getPosition();
    return ESP_OK;
}

esp_err_t MotionController::getAllAxisPositions(float positions[LIMIT_NUM_AXES])
{
    if (!initialized_) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (positions == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        if (motors_[i] != nullptr) {
            positions[i] = motors_[i]->getPosition();
        } else {
            positions[i] = 0.0f;
        }
    }

    return ESP_OK;
}

esp_err_t MotionController::moveAxisVelocity(uint8_t axis, float velocity)
{
    if (!initialized_) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (axis >= LIMIT_NUM_AXES) {
        ESP_LOGE(TAG, "Invalid axis: %d", axis);
        return ESP_ERR_INVALID_ARG;
    }

    IMotor* motor = motors_[axis];
    if (motor == nullptr) {
        ESP_LOGE(TAG, "Motor %d is null", axis);
        return ESP_ERR_INVALID_ARG;
    }

    if (!motor->isEnabled()) {
        ESP_LOGD(TAG, "Axis %d not enabled", axis);
        return ESP_ERR_INVALID_STATE;
    }

    // Story 4-3: Check if axis is faulted (both limits active)
    if (safety_monitor_is_axis_faulted(axis)) {
        ESP_LOGW(TAG, "Axis %d is faulted (both limits active)", axis);
        return ESP_ERR_INVALID_STATE;
    }

    // Story 4-3: Check direction blocking by active limit switches
    // Velocity sign determines direction: positive = MAX direction, negative = MIN direction
    int8_t direction = 0;
    if (std::fabs(velocity) > 0.0001f) {  // Non-zero velocity
        direction = (velocity > 0) ? 1 : -1;
        if (safety_monitor_is_direction_blocked(axis, direction)) {
            ESP_LOGW(TAG, "Axis %d velocity direction %+d blocked by limit", axis, direction);
            return ESP_ERR_INVALID_STATE;
        }
    }

    // Story 4-9: Notify floating switch handler when C axis starts moving
    // This enables width reset on new closing motion (AC6)
    if (axis == AXIS_C && direction != 0) {
        floating_switch_on_motion_start(direction);
    }

    return motor->moveVelocity(velocity);
}

esp_err_t MotionController::stopAxis(uint8_t axis)
{
    if (!initialized_) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (axis >= LIMIT_NUM_AXES) {
        ESP_LOGE(TAG, "Invalid axis: %d", axis);
        return ESP_ERR_INVALID_ARG;
    }

    IMotor* motor = motors_[axis];
    if (motor == nullptr) {
        ESP_LOGE(TAG, "Motor %d is null", axis);
        return ESP_ERR_INVALID_ARG;
    }

    // Stop returns ESP_ERR_INVALID_STATE if not moving, which is OK
    esp_err_t ret = motor->stop();
    if (ret == ESP_ERR_INVALID_STATE) {
        return ESP_OK;  // Not moving is not an error for STOP
    }
    return ret;
}

esp_err_t MotionController::stopAllAxes()
{
    if (!initialized_) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = ESP_OK;

    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        IMotor* motor = motors_[i];
        if (motor == nullptr) {
            continue;
        }

        if (motor->isMoving()) {
            esp_err_t ret = motor->stop();
            if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "Failed to stop axis %d: 0x%x", i, ret);
                result = ret;  // Track last error but continue
            }
        }
    }

    return result;
}

void MotionController::onMotionComplete(uint8_t axis, float position)
{
    ESP_LOGD(TAG, "Motion complete: axis=%d pos=%.3f", axis, position);

    // Send EVENT DONE via USB CDC for host-side motion tracking
    static const char* axis_names[] = {"X", "Y", "Z", "A", "B", "C", "D", "E"};
    if (axis < LIMIT_NUM_AXES) {
        char event_msg[64];
        snprintf(event_msg, sizeof(event_msg), "EVENT DONE %s %.4f",
                 axis_names[axis], position);
        usb_cdc_send_line(event_msg);
    }

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

/* ==========================================================================
 * C Wrapper Functions (Story 4-3)
 * ========================================================================== */

extern "C" {

esp_err_t motion_controller_stop_axis(uint8_t axis)
{
    MotionController* mc = getMotionController();
    if (mc == nullptr || !mc->isInitialized()) {
        return ESP_ERR_INVALID_STATE;
    }
    return mc->stopAxis(axis);
}

bool motion_controller_is_initialized(void)
{
    MotionController* mc = getMotionController();
    return (mc != nullptr && mc->isInitialized());
}

esp_err_t motion_controller_get_position(uint8_t axis, float* position)
{
    MotionController* mc = getMotionController();
    if (mc == nullptr || !mc->isInitialized()) {
        return ESP_ERR_INVALID_STATE;
    }
    return mc->getAxisPosition(axis, position);
}

esp_err_t motion_controller_stop_all_axes(void)
{
    MotionController* mc = getMotionController();
    if (mc == nullptr || !mc->isInitialized()) {
        return ESP_ERR_INVALID_STATE;
    }
    return mc->stopAllAxes();
}

} // extern "C"
