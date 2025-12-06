/**
 * @file stepper_motor.cpp
 * @brief Stepper motor implementation for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note StepperMotor inherits from MotorBase and provides only
 *       stepper-specific behavior via virtual hook overrides.
 *       All common motor logic is in MotorBase.
 *
 * @note Key difference from ServoMotor: PCNT hardware counter is the
 *       authoritative position source, not the internal pulse_count_.
 */

#include "stepper_motor.h"
#include "esp_log.h"

static const char* TAG = "stepper_motor";

StepperMotor::StepperMotor(IPulseGenerator* pulse_gen,
                           IPositionTracker* position_tracker,
                           uint8_t axis_id,
                           const AxisConfig& config)
    : MotorBase(pulse_gen, position_tracker, axis_id, config)
{
}

void StepperMotor::onEnableHook(bool enabled)
{
    // Stepper-specific enable behavior: NO-OP
    // Steppers have holding torque and don't need brake control.
    // Just log for debugging.
    ESP_LOGD(TAG, "Axis %d: onEnableHook(%s) - no-op for stepper",
             axis_id_, enabled ? "true" : "false");

    (void)enabled;
}

void StepperMotor::syncPositionFromTracker()
{
    // For stepper motors, PCNT hardware counter is authoritative.
    // Sync pulse_count_ FROM the position tracker (PCNT).
    // This is the key difference from ServoMotor.
    int64_t pcnt_position = position_tracker_->getPosition();
    pulse_count_.store(pcnt_position, std::memory_order_release);

    ESP_LOGD(TAG, "Axis %d: synced position from PCNT: %lld pulses",
             axis_id_, (long long)pcnt_position);
}

AxisState StepperMotor::getInitialState()
{
    // Steppers always start UNHOMED because position is lost on power cycle.
    // No absolute encoder feedback - homing is required.
    return AXIS_STATE_UNHOMED;
}
