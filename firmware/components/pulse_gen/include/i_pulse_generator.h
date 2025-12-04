/**
 * @file i_pulse_generator.h
 * @brief Abstract interface for pulse generation used by motor control
 * @author YaRobot Team
 * @date 2025
 *
 * @note This interface provides a unified API for different pulse generation
 *       technologies (RMT, MCPWM, LEDC). All implementations use the streaming
 *       double-buffer architecture for consistent behavior across motion types.
 */

#ifndef I_PULSE_GENERATOR_H
#define I_PULSE_GENERATOR_H

#include <cstdint>
#include <functional>
#include "esp_err.h"

/**
 * @brief Abstract interface for pulse generation
 *
 * Implementations:
 * - RmtPulseGenerator: For X, Z, A, B servo axes using RMT peripheral
 * - McpwmPulseGenerator: For Y, C axes using MCPWM with PCNT feedback
 * - LedcPulseGenerator: For D axis stepper using LEDC timer
 *
 * All motion parameters are in pulse domain (pulses, pulses/second, pulses/second^2).
 * SI unit conversion happens in the motor abstraction layer.
 */
class IPulseGenerator {
public:
    virtual ~IPulseGenerator() = default;

    /**
     * @brief Initialize the pulse generator
     * @return ESP_OK on success, error code on failure
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Start a position move with specified parameters
     *
     * Generates exactly `pulses` pulses using a trapezoidal velocity profile
     * with the specified maximum velocity and acceleration. The sign of pulses
     * indicates direction (positive = forward, negative = reverse).
     *
     * @param pulses Target pulse count (signed, negative for reverse)
     * @param max_velocity Maximum velocity in pulses/second (must be positive)
     * @param acceleration Acceleration in pulses/second^2 (must be positive)
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if parameters are out of range
     * @return ESP_ERR_INVALID_STATE if already running and blending not supported
     */
    virtual esp_err_t startMove(int32_t pulses, float max_velocity, float acceleration) = 0;

    /**
     * @brief Start continuous velocity mode
     *
     * Generates pulses continuously at the target velocity until stop() is called.
     * Accelerates from current velocity (or zero) to target velocity.
     *
     * @param velocity Target velocity in pulses/second (signed, negative for reverse)
     * @param acceleration Acceleration rate in pulses/second^2 (must be positive)
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if parameters are out of range
     */
    virtual esp_err_t startVelocity(float velocity, float acceleration) = 0;

    /**
     * @brief Stop motion with controlled deceleration
     *
     * Decelerates from current velocity to zero using the specified rate.
     * Fires completion callback when stopped.
     *
     * @param deceleration Deceleration rate in pulses/second^2 (must be positive)
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_STATE if not running
     */
    virtual esp_err_t stop(float deceleration) = 0;

    /**
     * @brief Emergency stop - halt immediately without deceleration
     *
     * Stops pulse generation within 10µs. Does NOT fire completion callback
     * (motion was aborted, not completed).
     */
    virtual void stopImmediate() = 0;

    /**
     * @brief Check if motion is currently in progress
     * @return true if generating pulses, false if idle
     */
    virtual bool isRunning() const = 0;

    /**
     * @brief Get the total pulse count since last startMove
     *
     * Returns the cumulative number of pulses generated since the last
     * startMove() call. Updated atomically from ISR context.
     *
     * @return Total pulses generated (absolute value, direction tracked separately)
     */
    virtual int64_t getPulseCount() const = 0;

    /**
     * @brief Get current instantaneous velocity
     * @return Current pulse frequency in pulses/second (Hz)
     */
    virtual float getCurrentVelocity() const = 0;

    /**
     * @brief Callback type for motion completion notification
     * @param total_pulses Total number of pulses generated during the motion
     */
    using MotionCompleteCallback = std::function<void(int64_t total_pulses)>;

    /**
     * @brief Set callback for motion completion
     *
     * The callback is invoked when motion completes normally (all pulses sent
     * or velocity stop completed). It is NOT called on stopImmediate().
     *
     * @param cb Callback function, or nullptr to clear
     */
    virtual void setCompletionCallback(MotionCompleteCallback cb) = 0;
};

#endif // I_PULSE_GENERATOR_H
