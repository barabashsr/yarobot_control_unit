/**
 * @file stepper_motor.h
 * @brief Stepper motor implementation for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note StepperMotor implements IMotor for stepper axes (C, D).
 *       Inherits common functionality from MotorBase and provides
 *       stepper-specific behavior via virtual hook overrides.
 *
 * @note Key difference from ServoMotor: PCNT hardware counter is the
 *       authoritative position source. pulse_count_ is synchronized
 *       FROM the tracker (not the other way around).
 */

#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "motor_base.h"

/**
 * @brief Stepper motor implementation using pulse generators
 *
 * Inherits from MotorBase for shared motor control logic.
 * Provides stepper-specific behavior:
 * - PCNT hardware counter is authoritative position source
 * - No brake control (holding torque sufficient)
 * - Power loss = position lost (starts in AXIS_STATE_UNHOMED)
 *
 * Hardware mapping:
 * - C axis: McpwmPulseGenerator (MCPWM_TIMER_C), PcntTracker (PCNT_UNIT_C)
 * - D axis: LedcPulseGenerator (LEDC_CHANNEL_D), PcntTracker (PCNT_UNIT_D via GPIO loopback)
 *
 * Uses:
 * - IPulseGenerator for motion execution (MCPWM for C, LEDC for D)
 * - IPositionTracker (PcntTracker) for hardware position counting
 * - Shift register for direction and enable control
 */
class StepperMotor : public MotorBase {
public:
    /**
     * @brief Construct a StepperMotor
     *
     * @param pulse_gen Pulse generator for motion execution (MCPWM or LEDC)
     * @param position_tracker Position tracker (PcntTracker for hardware counting)
     * @param axis_id Axis identifier (5 for C, 6 for D, matches sr_axis_t)
     * @param config Axis configuration with limits, velocity, etc.
     *
     * @note Dependencies are injected, not owned - caller manages lifetime
     * @note PCNT hardware counter is authoritative for stepper position
     */
    StepperMotor(IPulseGenerator* pulse_gen,
                 IPositionTracker* position_tracker,
                 uint8_t axis_id,
                 const AxisConfig& config);

    ~StepperMotor() override = default;

protected:
    /**
     * @brief Handle enable/disable for stepper-specific behavior
     *
     * For steppers, this is a no-op. Steppers have holding torque
     * and don't require brake control like servos.
     *
     * @param enabled true if enabling, false if disabling (ignored)
     */
    void onEnableHook(bool enabled) override;

    /**
     * @brief Sync position from tracker (stepper strategy)
     *
     * For stepper motors, the PCNT hardware counter is authoritative.
     * This syncs pulse_count_ FROM the position tracker (PCNT).
     * This is the opposite of ServoMotor where pulse_count_ is authority.
     */
    void syncPositionFromTracker() override;

    /**
     * @brief Get initial axis state
     *
     * Steppers always start in UNHOMED state because position is lost
     * on power cycle (no absolute encoder feedback).
     *
     * @return AXIS_STATE_UNHOMED
     */
    AxisState getInitialState() override;
};

#endif // STEPPER_MOTOR_H
