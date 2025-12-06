/**
 * @file motor_base.h
 * @brief Abstract base class for motor implementations
 * @author YaRobot Team
 * @date 2025
 *
 * @note MotorBase provides ~90% of IMotor implementation shared by all motor types.
 *       Subclasses (ServoMotor, StepperMotor) override virtual hooks for type-specific behavior.
 *
 * @note Header-only configuration: All timing values from config_timing.h,
 *       all peripheral references from config_peripherals.h.
 */

#ifndef MOTOR_BASE_H
#define MOTOR_BASE_H

#include <atomic>
#include <cstdint>
#include "esp_err.h"
#include "i_motor.h"
#include "i_pulse_generator.h"
#include "i_position_tracker.h"

// Forward declaration for C linkage shift register functions
extern "C" {
    #include "tpic6b595.h"
}

/**
 * @brief Abstract base class for motor implementations
 *
 * Provides shared implementation for:
 * - Motion control: moveAbsolute, moveRelative, moveVelocity, stop, stopImmediate
 * - State management: enable, getState, isMoving, isEnabled
 * - Position/velocity: getPosition, getVelocity
 * - Configuration: getConfig, setConfig, setMotionCompleteCallback
 *
 * Subclasses must implement virtual hooks:
 * - onEnableHook(): Type-specific behavior on enable/disable (e.g., brake control)
 * - syncPositionFromTracker(): Type-specific position synchronization strategy
 * - getInitialState(): Initial state after init (default: AXIS_STATE_UNHOMED)
 *
 * Architecture Constraints:
 * - All external interfaces use SI units (m, rad, s)
 * - All internal motion uses pulse domain
 * - Thread-safe: atomic pulse_count_ and state_
 * - No hardcoded values - all config from headers
 */
class MotorBase : public IMotor {
public:
    /**
     * @brief Construct a MotorBase
     *
     * @param pulse_gen Pulse generator for motion execution
     * @param position_tracker Position tracker for feedback
     * @param axis_id Axis identifier (matches sr_axis_t)
     * @param config Axis configuration with limits, velocity, etc.
     *
     * @note Dependencies are injected, not owned - caller manages lifetime
     */
    MotorBase(IPulseGenerator* pulse_gen,
              IPositionTracker* position_tracker,
              uint8_t axis_id,
              const AxisConfig& config);

    virtual ~MotorBase() = default;

    // Non-copyable, non-movable (contains atomics and pointers)
    MotorBase(const MotorBase&) = delete;
    MotorBase& operator=(const MotorBase&) = delete;
    MotorBase(MotorBase&&) = delete;
    MotorBase& operator=(MotorBase&&) = delete;

    // IMotor interface implementation (shared by all motor types)
    esp_err_t init() override;
    esp_err_t moveAbsolute(float position, float velocity) override;
    esp_err_t moveRelative(float delta, float velocity) override;
    esp_err_t moveVelocity(float velocity) override;
    esp_err_t stop() override;
    void stopImmediate() override;
    float getPosition() const override;
    float getVelocity() const override;
    bool isMoving() const override;
    bool isEnabled() const override;
    esp_err_t enable(bool en) override;
    AxisState getState() const override;
    const AxisConfig& getConfig() const override;
    esp_err_t setConfig(const AxisConfig& config) override;
    void setMotionCompleteCallback(MotionCompleteCallback cb) override;

protected:
    /**
     * @defgroup motor_hooks Virtual Hooks for Subclass Customization
     * @brief Override these in derived classes for type-specific behavior
     * @{
     */

    /**
     * @brief Hook called on enable/disable
     *
     * Called after enable bit is set/cleared, before state transition.
     * - ServoMotor: Prepare for brake control (Epic 4)
     * - StepperMotor: No-op (holding torque, no brake)
     *
     * @param enabled true if enabling, false if disabling
     */
    virtual void onEnableHook(bool enabled) = 0;

    /**
     * @brief Hook to synchronize pulse_count_ with position tracker
     *
     * Called on motion completion. Different sync strategies:
     * - ServoMotor: pulse_count_ is authority (internal counting)
     * - StepperMotor: PCNT is authority (sync pulse_count_ FROM tracker)
     */
    virtual void syncPositionFromTracker() = 0;

    /**
     * @brief Get initial axis state after successful init
     *
     * @return AxisState to set after init (default: AXIS_STATE_UNHOMED)
     */
    virtual AxisState getInitialState() { return AXIS_STATE_UNHOMED; }

    /** @} */ // end motor_hooks

    /**
     * @brief Handle motion completion from pulse generator
     *
     * Called by pulse generator when motion completes. Updates state,
     * syncs position via virtual hook, and invokes user callback.
     *
     * @param total_pulses Total pulses generated during motion
     */
    void onMotionComplete(int64_t total_pulses);

    /**
     * @defgroup motor_helpers Protected Helper Methods
     * @brief Shared utility methods for subclasses
     * @{
     */

    /**
     * @brief Convert SI velocity to pulse frequency
     *
     * @param velocity_si Velocity in m/s or rad/s
     * @return Frequency in pulses/second (Hz)
     */
    float velocityToFrequency(float velocity_si) const;

    /**
     * @brief Convert SI acceleration to pulse acceleration
     *
     * @param accel_si Acceleration in m/s^2 or rad/s^2
     * @return Acceleration in pulses/s^2
     */
    float accelerationToPulses(float accel_si) const;

    /**
     * @brief Set motor direction via shift register
     *
     * @param forward true for positive direction, false for negative
     * @return ESP_OK on success
     */
    esp_err_t setDirection(bool forward);

    /**
     * @brief Wait for direction setup time
     *
     * Delays TIMING_DIR_SETUP_US before starting pulses.
     */
    void waitDirectionSetup();

    /**
     * @brief Wait for enable delay time
     *
     * Delays TIMING_ENABLE_DELAY_US after enabling before allowing motion.
     */
    void waitEnableDelay();

    /** @} */ // end motor_helpers

    // Dependencies (injected, not owned)
    IPulseGenerator* pulse_gen_;         ///< Pulse generator for motion
    IPositionTracker* position_tracker_; ///< Position feedback source

    // Axis identification
    uint8_t axis_id_;                    ///< Axis ID (0-7 for X-E)

    // Configuration
    AxisConfig config_;                  ///< Axis configuration

    // Position tracking
    std::atomic<int64_t> pulse_count_;   ///< Current position in pulses

    // State machine
    std::atomic<AxisState> state_;       ///< Current axis state

    // Initialization flag
    bool initialized_;                   ///< true after successful init()

    // User callback
    MotionCompleteCallback motion_complete_cb_; ///< User motion complete callback
};

#endif // MOTOR_BASE_H
