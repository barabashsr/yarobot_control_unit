/**
 * @file servo_motor.cpp
 * @brief Servo motor implementation for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note ServoMotor inherits from MotorBase and provides only
 *       servo-specific behavior via virtual hook overrides.
 *       All common motor logic is in MotorBase.
 */

#include "servo_motor.h"
#include "esp_log.h"

static const char* TAG = "servo_motor";

ServoMotor::ServoMotor(IPulseGenerator* pulse_gen,
                       IPositionTracker* position_tracker,
                       uint8_t axis_id,
                       const AxisConfig& config)
    : MotorBase(pulse_gen, position_tracker, axis_id, config)
    , last_direction_forward_(true)
{
}

void ServoMotor::onEnableHook(bool enabled)
{
    // Servo-specific enable behavior
    // Epic 4 will add brake control here:
    // - enable(true): release brake (set SR_x_BRAKE bit)
    // - enable(false): engage brake (clear SR_x_BRAKE bit)
    //
    // For now, just log for debugging
    ESP_LOGD(TAG, "Axis %d: onEnableHook(%s)", axis_id_, enabled ? "true" : "false");

    // No brake control in this story - will be added in Epic 4
    (void)enabled;
}

void ServoMotor::syncPositionFromTracker()
{
    // For servo motors, the internal pulse_count_ is authority
    // Sync pulse_count_ from position tracker (which tracks commanded pulses)
    int64_t pos = position_tracker_->getPosition();
    pulse_count_.store(pos, std::memory_order_release);
}
