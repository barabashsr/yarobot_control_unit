/**
 * @file motion_controller.h
 * @brief Motion controller for coordinating motor movements
 * @author YaRobot Team
 * @date 2025
 *
 * @note Central coordinator for all motor motion. Provides axis lookup,
 *       movement commands, and motion completion event handling.
 *
 * Thread Safety:
 * - All methods are thread-safe
 * - Motion completion callbacks may be invoked from ISR context
 */

#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <cstdint>
#include "esp_err.h"
#include "i_motor.h"
#include "config_limits.h"
#include "config_axes.h"

/**
 * @brief Motion controller class for coordinating motor movements
 *
 * The MotionController is the central coordinator for all motor motion.
 * It maintains an array of motor pointers for all axes and provides
 * a unified interface for movement commands.
 *
 * Key responsibilities:
 * - Motor registry: stores IMotor* pointers for all LIMIT_NUM_AXES axes
 * - Axis lookup: maps axis characters to motor pointers
 * - Motion delegation: validates parameters and delegates to motors
 * - Event coordination: registers callbacks on motors, publishes events
 *
 * Usage:
 * @code
 * MotionController controller;
 * IMotor* motors[LIMIT_NUM_AXES] = { &servoX, &servoY, ... };
 * controller.init(motors);
 *
 * // Move X axis to position 100mm at 50mm/s
 * controller.moveAbsolute(AXIS_X, 0.1f, 0.05f);
 * @endcode
 */
class MotionController {
public:
    /**
     * @brief Default constructor
     *
     * Initializes all motor pointers to nullptr.
     */
    MotionController();

    /**
     * @brief Initialize the motion controller with motor array
     *
     * Stores motor pointers and registers motion complete callbacks on each.
     * All motor pointers must be non-null.
     *
     * @param[in] motors Array of LIMIT_NUM_AXES motor pointers
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if motors is nullptr or any motor is nullptr
     * @return ESP_ERR_INVALID_STATE if already initialized
     */
    esp_err_t init(IMotor* motors[LIMIT_NUM_AXES]);

    /**
     * @brief Get motor by axis ID
     *
     * @param[in] axis_id Axis index (0 to LIMIT_NUM_AXES-1)
     *
     * @return Pointer to IMotor, or nullptr if axis_id is invalid or not initialized
     */
    IMotor* getMotor(uint8_t axis_id) const;

    /**
     * @brief Get motor by axis character
     *
     * Maps axis character to motor pointer using config_axes.h constants.
     *
     * @param[in] axis_char Axis character ('X', 'Y', 'Z', 'A', 'B', 'C', 'D', 'E')
     *
     * @return Pointer to IMotor, or nullptr if axis_char is invalid
     *
     * @note Case-insensitive: 'x' and 'X' both work
     */
    IMotor* getMotor(char axis_char) const;

    /**
     * @brief Move axis to absolute position
     *
     * Validates parameters and delegates to motor->moveAbsolute().
     * If called during motion, blends to new target (AC9).
     *
     * @param[in] axis Axis index (0 to LIMIT_NUM_AXES-1)
     * @param[in] position Target position in SI units (meters or radians)
     * @param[in] velocity Maximum velocity (m/s or rad/s), or <= 0 for default
     *
     * @return ESP_OK if move started successfully
     * @return ESP_ERR_INVALID_ARG if axis is invalid or position exceeds limits (AC7)
     * @return ESP_ERR_INVALID_STATE if axis is disabled (AC6)
     *
     * @note If velocity <= 0, uses DEFAULT_MAX_VELOCITY (AC10)
     */
    esp_err_t moveAbsolute(uint8_t axis, float position, float velocity);

    /**
     * @brief Move axis relative to current position
     *
     * Calculates target = current_position + delta and calls moveAbsolute().
     *
     * @param[in] axis Axis index (0 to LIMIT_NUM_AXES-1)
     * @param[in] delta Distance to move in SI units (positive or negative)
     * @param[in] velocity Maximum velocity (m/s or rad/s), or <= 0 for default
     *
     * @return ESP_OK if move started successfully
     * @return ESP_ERR_INVALID_ARG if target position exceeds limits
     * @return ESP_ERR_INVALID_STATE if axis is disabled
     */
    esp_err_t moveRelative(uint8_t axis, float delta, float velocity);

    /**
     * @brief Check if controller is initialized
     *
     * @return true if init() was called successfully
     */
    bool isInitialized() const { return initialized_; }

private:
    /**
     * @brief Motion complete callback handler
     *
     * Static callback registered on each motor. Publishes EVT_MOTION_COMPLETE
     * event via event_manager.
     *
     * @param[in] axis Axis ID that completed motion
     * @param[in] position Final position in SI units
     */
    static void onMotionComplete(uint8_t axis, float position);

    /** @brief Array of motor pointers, one per axis */
    IMotor* motors_[LIMIT_NUM_AXES];

    /** @brief Initialization flag */
    bool initialized_;
};

/**
 * @brief Get the global MotionController instance
 *
 * Returns a pointer to the singleton motion controller instance.
 * The instance is created on first call.
 *
 * @return Pointer to the global MotionController
 */
MotionController* getMotionController();

#endif // MOTION_CONTROLLER_H
