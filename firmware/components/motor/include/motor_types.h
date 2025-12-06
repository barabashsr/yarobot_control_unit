/**
 * @file motor_types.h
 * @brief Motor type definitions for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note Defines AxisState enum and AxisConfig struct used by IMotor interface.
 *       All configuration values use SI units (meters, radians, seconds).
 */

#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <cstdint>
#include "config_limits.h"
#include "config_defaults.h"

/**
 * @defgroup motor_types Motor Type Definitions
 * @brief Enums and structs for motor abstraction layer
 * @{
 */

/**
 * @brief Axis state enumeration
 *
 * Represents the current operational state of a motor axis.
 * State transitions are managed by the motor implementation.
 */
typedef enum {
    AXIS_STATE_DISABLED,    ///< Motor disabled, no motion possible
    AXIS_STATE_IDLE,        ///< Enabled, not moving, ready for commands
    AXIS_STATE_MOVING,      ///< Active motion in progress
    AXIS_STATE_ERROR,       ///< Error condition, requires recovery
    AXIS_STATE_UNHOMED      ///< Power-on state, position unknown
} AxisState;

/**
 * @brief Axis configuration structure
 *
 * Contains all per-axis configuration parameters. Values are in SI units
 * (meters for linear axes, radians for rotary axes, seconds for time).
 * Default values are provided by config_defaults.h constants.
 */
struct AxisConfig {
    /**
     * @brief Pulses per motor revolution
     *
     * Number of step/encoder pulses for one complete motor shaft revolution.
     * Used with units_per_rev to calculate pulses_per_unit.
     */
    float pulses_per_rev;

    /**
     * @brief Units of travel per motor revolution
     *
     * For linear axes: meters per revolution (e.g., lead screw pitch)
     * For rotary axes: radians per revolution (typically 2*PI)
     */
    float units_per_rev;

    /**
     * @brief Axis type: true for rotary, false for linear
     */
    bool is_rotary;

    /**
     * @brief Minimum position soft limit (meters or radians)
     *
     * Motion commands are rejected if target position is below this limit.
     */
    float limit_min;

    /**
     * @brief Maximum position soft limit (meters or radians)
     *
     * Motion commands are rejected if target position exceeds this limit.
     */
    float limit_max;

    /**
     * @brief Maximum velocity (m/s or rad/s)
     *
     * Commanded velocities are clamped to this value.
     */
    float max_velocity;

    /**
     * @brief Maximum acceleration (m/s^2 or rad/s^2)
     *
     * Used for trapezoidal profile generation.
     */
    float max_acceleration;

    /**
     * @brief Backlash compensation value (meters or radians)
     *
     * Applied when direction changes to compensate for mechanical play.
     */
    float backlash;

    /**
     * @brief Home position offset (meters or radians)
     *
     * Offset applied after homing to set the zero position.
     */
    float home_offset;

    /**
     * @brief Axis alias for human-readable identification
     *
     * Null-terminated string, max LIMIT_ALIAS_MAX_LENGTH characters.
     */
    char alias[LIMIT_ALIAS_MAX_LENGTH + 1];

    /**
     * @brief Calculate pulses per SI unit
     *
     * @return pulses_per_rev / units_per_rev (pulses per meter or radian)
     */
    float getPulsesPerUnit() const {
        return pulses_per_rev / units_per_rev;
    }

    /**
     * @brief Create default linear axis configuration
     *
     * @return AxisConfig with DEFAULT_* values for linear axis
     */
    static AxisConfig createDefaultLinear() {
        AxisConfig cfg{};
        cfg.pulses_per_rev = DEFAULT_PULSES_PER_UNIT;  // Using as direct pulses/unit
        cfg.units_per_rev = 1.0f;  // 1:1 mapping when using pulses_per_unit directly
        cfg.is_rotary = DEFAULT_IS_ROTARY;
        cfg.limit_min = DEFAULT_LIMIT_MIN;
        cfg.limit_max = DEFAULT_LIMIT_MAX;
        cfg.max_velocity = DEFAULT_MAX_VELOCITY;
        cfg.max_acceleration = DEFAULT_MAX_ACCELERATION;
        cfg.backlash = DEFAULT_BACKLASH;
        cfg.home_offset = DEFAULT_HOME_OFFSET;
        cfg.alias[0] = '\0';
        return cfg;
    }

    /**
     * @brief Create default rotary axis configuration
     *
     * @return AxisConfig with DEFAULT_ROTARY_* values for rotary axis
     */
    static AxisConfig createDefaultRotary() {
        AxisConfig cfg{};
        cfg.pulses_per_rev = DEFAULT_ROTARY_PULSES_PER_UNIT * CONST_2PI;  // Convert to pulses/rev
        cfg.units_per_rev = CONST_2PI;  // One revolution = 2*PI radians
        cfg.is_rotary = true;
        cfg.limit_min = DEFAULT_ROTARY_LIMIT_MIN;
        cfg.limit_max = DEFAULT_ROTARY_LIMIT_MAX;
        cfg.max_velocity = DEFAULT_ROTARY_MAX_VEL;
        cfg.max_acceleration = DEFAULT_ROTARY_MAX_ACCEL;
        cfg.backlash = DEFAULT_BACKLASH;
        cfg.home_offset = DEFAULT_HOME_OFFSET;
        cfg.alias[0] = '\0';
        return cfg;
    }
};

/** @} */ // end motor_types

#endif // MOTOR_TYPES_H
