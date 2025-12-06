/**
 * @file servo_motor.h
 * @brief Servo motor implementation for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note ServoMotor implements IMotor for servo axes (X, Y, Z, A, B).
 *       Inherits common functionality from MotorBase and provides
 *       servo-specific behavior via virtual hook overrides.
 */

#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "motor_base.h"

/**
 * @brief Servo motor implementation using pulse generators
 *
 * Inherits from MotorBase for shared motor control logic.
 * Provides servo-specific behavior:
 * - pulse_count_ is the SINGLE SOURCE OF TRUTH for position
 * - Brake control preparation via onEnableHook() (Epic 4)
 * - Direction tracking for backlash compensation
 *
 * Uses:
 * - IPulseGenerator for motion execution (RMT for X/Z/A/B, MCPWM for Y)
 * - IPositionTracker for position feedback (Software for X/Z/A/B, PCNT for Y)
 * - Shift register for direction and enable control
 */
class ServoMotor : public MotorBase {
public:
    /**
     * @brief Construct a ServoMotor
     *
     * @param pulse_gen Pulse generator for motion execution (RMT or MCPWM)
     * @param position_tracker Position tracker for feedback (Software or PCNT)
     * @param axis_id Axis identifier (0-4 for X-B, matches sr_axis_t)
     * @param config Axis configuration with limits, velocity, etc.
     *
     * @note Dependencies are injected, not owned - caller manages lifetime
     * @note Shift register functions are called via C API (sr_set_direction, etc.)
     */
    ServoMotor(IPulseGenerator* pulse_gen,
               IPositionTracker* position_tracker,
               uint8_t axis_id,
               const AxisConfig& config);

    ~ServoMotor() override = default;

protected:
    /**
     * @brief Handle enable/disable for servo-specific behavior
     *
     * Prepares for brake control (Epic 4). Currently no-op, but
     * will handle brake release on enable and brake engage on disable.
     *
     * @param enabled true if enabling, false if disabling
     */
    void onEnableHook(bool enabled) override;

    /**
     * @brief Sync position from tracker (servo strategy)
     *
     * For servo motors, pulse_count_ is the authority. This syncs
     * pulse_count_ from the position tracker after motion completion.
     * The tracker follows the commanded pulses (internal counting).
     */
    void syncPositionFromTracker() override;

private:
    /// Direction tracking for backlash compensation
    bool last_direction_forward_;
};

#endif // SERVO_MOTOR_H
