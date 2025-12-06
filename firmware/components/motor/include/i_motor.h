/**
 * @file i_motor.h
 * @brief Abstract motor interface for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note This interface provides a unified API for all motor types (servo, stepper,
 *       discrete actuator). All external interfaces use SI units (meters, radians,
 *       seconds). Internal pulse domain conversion happens in implementations.
 *
 * @note Header-only interface - no implementation file.
 */

#ifndef I_MOTOR_H
#define I_MOTOR_H

#include <cstdint>
#include <functional>
#include "esp_err.h"
#include "motor_types.h"

/**
 * @brief Abstract interface for motor control
 *
 * Implementations:
 * - ServoMotor: For servo axes (X, Y, Z, A, B) using pulse generators
 * - StepperMotor: For stepper axes (C, D) with open-loop control
 * - DiscreteMotor: For discrete actuators (E) with binary positions
 *
 * All position values are in SI units:
 * - Linear axes: meters (m)
 * - Rotary axes: radians (rad)
 * - Velocity: m/s or rad/s
 * - Acceleration: m/s^2 or rad/s^2
 *
 * Thread Safety:
 * - All methods except setConfig() are thread-safe
 * - State and position use atomic operations
 * - Callbacks may be invoked from ISR context
 */
class IMotor {
public:
    virtual ~IMotor() = default;

    /**
     * @brief Callback type for motion completion notification
     *
     * @param axis Axis ID that completed motion
     * @param position Final position in SI units (meters or radians)
     *
     * @note May be called from ISR context - keep handler short and non-blocking
     */
    using MotionCompleteCallback = std::function<void(uint8_t axis, float position)>;

    /**
     * @brief Initialize the motor
     *
     * Validates all dependencies, configures hardware, and sets initial state
     * to AXIS_STATE_UNHOMED (or AXIS_STATE_DISABLED if init fails).
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if configuration is invalid
     * @return ESP_ERR_INVALID_STATE if already initialized
     * @return ESP_ERR_NO_MEM if memory allocation fails
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Move to absolute position
     *
     * Moves the axis to the specified position using a trapezoidal velocity
     * profile. If called during motion, blends to the new target (no "axis busy"
     * error).
     *
     * @param position Target position in SI units (meters or radians)
     * @param velocity Maximum velocity for this move (m/s or rad/s)
     *
     * @return ESP_OK if move started successfully
     * @return ESP_ERR_INVALID_ARG if position exceeds soft limits (AC14)
     * @return ESP_ERR_INVALID_STATE if axis is disabled (AC15)
     *
     * @note Position is validated against config.limit_min and config.limit_max
     * @note Velocity is clamped to config.max_velocity
     * @note Direction is set via shift register with TIMING_DIR_SETUP_US delay
     */
    virtual esp_err_t moveAbsolute(float position, float velocity) = 0;

    /**
     * @brief Move relative to current position
     *
     * Equivalent to moveAbsolute(getPosition() + delta, velocity).
     *
     * @param delta Distance to move in SI units (positive or negative)
     * @param velocity Maximum velocity for this move (m/s or rad/s)
     *
     * @return ESP_OK if move started successfully
     * @return ESP_ERR_INVALID_ARG if target position exceeds soft limits
     * @return ESP_ERR_INVALID_STATE if axis is disabled
     */
    virtual esp_err_t moveRelative(float delta, float velocity) = 0;

    /**
     * @brief Start continuous velocity mode
     *
     * Moves the axis continuously at the specified velocity until stop() is
     * called or a limit is reached. Velocity sign determines direction.
     *
     * @param velocity Target velocity in SI units (m/s or rad/s, signed)
     *
     * @return ESP_OK if velocity mode started
     * @return ESP_ERR_INVALID_STATE if axis is disabled
     *
     * @note Velocity is clamped to ±config.max_velocity
     * @note Soft limit checking during velocity mode is handled by motion controller
     */
    virtual esp_err_t moveVelocity(float velocity) = 0;

    /**
     * @brief Stop motion with controlled deceleration
     *
     * Decelerates to zero velocity using config.max_acceleration.
     * State transitions to AXIS_STATE_IDLE when stopped.
     * Fires motion complete callback when fully stopped.
     *
     * @return ESP_OK if stop initiated
     * @return ESP_ERR_INVALID_STATE if not moving
     */
    virtual esp_err_t stop() = 0;

    /**
     * @brief Emergency stop - halt immediately
     *
     * Stops pulse generation immediately without deceleration ramp.
     * State transitions to AXIS_STATE_IDLE immediately.
     * Does NOT fire motion complete callback (motion was aborted).
     *
     * @note Thread-safe and ISR-safe
     */
    virtual void stopImmediate() = 0;

    /**
     * @brief Get current position
     *
     * Returns the current position in SI units, derived from the internal
     * pulse count: position = pulse_count / pulses_per_unit
     *
     * @return Current position in meters (linear) or radians (rotary)
     *
     * @note Thread-safe (uses atomic pulse count)
     */
    virtual float getPosition() const = 0;

    /**
     * @brief Get current velocity
     *
     * Returns the instantaneous velocity in SI units.
     *
     * @return Current velocity in m/s (linear) or rad/s (rotary)
     *
     * @note Returns 0 if not moving
     */
    virtual float getVelocity() const = 0;

    /**
     * @brief Check if axis is currently moving
     *
     * @return true if state is AXIS_STATE_MOVING, false otherwise
     */
    virtual bool isMoving() const = 0;

    /**
     * @brief Check if axis is enabled
     *
     * @return true if state is not AXIS_STATE_DISABLED, false otherwise
     */
    virtual bool isEnabled() const = 0;

    /**
     * @brief Enable or disable the motor
     *
     * When enabled (en=true):
     * - Sets shift register EN bit
     * - Waits TIMING_ENABLE_DELAY_US before allowing motion
     * - Transitions to AXIS_STATE_IDLE (or AXIS_STATE_UNHOMED if not homed)
     *
     * When disabled (en=false):
     * - Stops any active motion immediately
     * - Clears shift register EN bit
     * - Transitions to AXIS_STATE_DISABLED
     *
     * @param en true to enable, false to disable
     *
     * @return ESP_OK on success
     * @return ESP_ERR_TIMEOUT if shift register update fails
     */
    virtual esp_err_t enable(bool en) = 0;

    /**
     * @brief Get current axis state
     *
     * @return Current AxisState value
     *
     * @note Thread-safe (uses atomic state)
     */
    virtual AxisState getState() const = 0;

    /**
     * @brief Get current axis configuration
     *
     * @return Const reference to current AxisConfig
     */
    virtual const AxisConfig& getConfig() const = 0;

    /**
     * @brief Update axis configuration
     *
     * Updates the axis configuration. Some changes may only take effect
     * when the axis is idle (e.g., pulses_per_unit).
     *
     * @param config New configuration values
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if configuration values are invalid
     * @return ESP_ERR_INVALID_STATE if axis is moving and config requires idle
     */
    virtual esp_err_t setConfig(const AxisConfig& config) = 0;

    /**
     * @brief Set motion completion callback
     *
     * Registers a callback to be invoked when motion completes (either
     * reaching target position or controlled stop). The callback receives
     * the axis ID and final position.
     *
     * @param cb Callback function, or nullptr to clear
     *
     * @note Callback may be invoked from ISR context
     * @note Previous callback is replaced if already set
     */
    virtual void setMotionCompleteCallback(MotionCompleteCallback cb) = 0;
};

#endif // I_MOTOR_H
