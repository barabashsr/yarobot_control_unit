/**
 * @file discrete_axis.h
 * @brief Discrete axis implementation for E axis (pneumatic cylinder/solenoid)
 * @author YaRobot Team
 * @date 2025
 *
 * @note DiscreteAxis implements IMotor for the E axis discrete actuator.
 *       Unlike servo/stepper motors, discrete actuators have only two positions:
 *       0.0 (retracted) and 1.0 (extended). Motion is time-based, not pulse-based.
 *
 * @note Uses TimeTracker for position interpolation and shift register for
 *       DIR/EN control. Does NOT generate STEP pulses.
 */

#ifndef DISCRETE_AXIS_H
#define DISCRETE_AXIS_H

#include <atomic>
#include <cstdint>
#include "esp_err.h"
#include "esp_timer.h"
#include "i_motor.h"
#include "time_tracker.h"

// Forward declaration for C linkage shift register functions
extern "C" {
    #include "tpic6b595.h"
}

/**
 * @brief Discrete axis implementation for binary-position actuators
 *
 * Implements the IMotor interface for discrete actuators (E axis).
 * Key differences from ServoMotor/StepperMotor:
 * - Position is binary: 0.0 (retracted) or 1.0 (extended)
 * - Motion is time-based (no pulse generation)
 * - moveVelocity() is NOT supported (returns ESP_ERR_NOT_SUPPORTED)
 * - Uses esp_timer for motion completion detection
 *
 * Architecture Constraints:
 * - All external interfaces use SI units (position 0.0-1.0)
 * - Thread-safe: atomic state management
 * - DIR/EN control via shift register (SR_E_DIR, SR_E_EN)
 * - Travel time from config (TIMING_E_AXIS_TRAVEL_MS)
 *
 * State Machine:
 * - DISABLED: Motor disabled, no motion possible
 * - UNHOMED: Enabled but position unknown (initial state after init)
 * - IDLE: Enabled, not moving, ready for commands
 * - MOVING: Active motion in progress (extending or retracting)
 * - ERROR: Error condition, requires recovery
 */
class DiscreteAxis : public IMotor {
public:
    /**
     * @brief Construct a DiscreteAxis
     *
     * @param time_tracker TimeTracker for time-based position tracking
     * @param axis_id Axis identifier (7 for E axis, matches SR_AXIS_E)
     * @param config Axis configuration with limits, velocity, etc.
     *
     * @note TimeTracker is injected, not owned - caller manages lifetime
     * @note Shift register functions are called via C API (sr_set_direction, etc.)
     */
    DiscreteAxis(TimeTracker* time_tracker,
                 uint8_t axis_id,
                 const AxisConfig& config);

    ~DiscreteAxis() override;

    // Non-copyable, non-movable (contains atomics and timer handle)
    DiscreteAxis(const DiscreteAxis&) = delete;
    DiscreteAxis& operator=(const DiscreteAxis&) = delete;
    DiscreteAxis(DiscreteAxis&&) = delete;
    DiscreteAxis& operator=(DiscreteAxis&&) = delete;

    // IMotor interface implementation
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

private:
    /**
     * @brief Static callback for esp_timer motion completion
     *
     * Called when TIMING_E_AXIS_TRAVEL_MS elapses after motion start.
     * Transitions state to IDLE and invokes user callback.
     *
     * @param arg Pointer to DiscreteAxis instance
     */
    static void motionTimerCallback(void* arg);

    /**
     * @brief Handle motion completion
     *
     * Called by timer callback. Updates state, position, and invokes
     * user callback if registered.
     */
    void onMotionComplete();

    /**
     * @brief Wait for enable delay time
     *
     * Delays TIMING_ENABLE_DELAY_US after enabling before allowing motion.
     */
    void waitEnableDelay();

    // Dependencies (injected, not owned)
    TimeTracker* time_tracker_;          ///< Time-based position tracker

    // Axis identification
    uint8_t axis_id_;                    ///< Axis ID (7 for E)

    // Configuration
    AxisConfig config_;                  ///< Axis configuration

    // State machine
    std::atomic<AxisState> state_;       ///< Current axis state

    // Motion tracking
    std::atomic<float> target_position_; ///< Target position (0.0 or 1.0)
    std::atomic<float> current_position_;///< Current position (0.0 to 1.0)

    // Timer for motion completion
    esp_timer_handle_t motion_timer_;    ///< ESP timer for completion detection

    // Initialization flag
    bool initialized_;                   ///< true after successful init()

    // User callback
    MotionCompleteCallback motion_complete_cb_; ///< User motion complete callback
};

#endif // DISCRETE_AXIS_H
