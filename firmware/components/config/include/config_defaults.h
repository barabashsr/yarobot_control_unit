/**
 * @file config_defaults.h
 * @brief Default axis parameters and mathematical constants
 * @author YaRobot Team
 * @date 2025
 *
 * @note All values in SI units (meters, radians, seconds) per ROS2 REP-103.
 *       These are compile-time defaults that can be overridden via YAML config.
 */

#ifndef CONFIG_DEFAULTS_H
#define CONFIG_DEFAULTS_H

/**
 * @defgroup config_defaults Default Parameters
 * @brief Compile-time default values for axis configuration
 * @{
 */

/**
 * @defgroup math_constants Mathematical Constants
 * @brief Commonly used mathematical values
 * @{
 */

/** @brief Pi (3.14159265...) */
#define CONST_PI                    3.14159265f

/** @brief 2*Pi (6.28318531...) */
#define CONST_2PI                   6.28318531f

/** @brief Degrees to radians conversion factor (Pi/180) */
#define CONST_DEG_TO_RAD            0.01745329f

/** @brief Radians to degrees conversion factor (180/Pi) */
#define CONST_RAD_TO_DEG            57.2957795f

/** @} */ // end math_constants

/**
 * @defgroup linear_defaults Default Linear Axis Configuration
 * @brief Default parameters for linear (translation) axes in meters
 *
 * These defaults provide 1 micrometer resolution with reasonable
 * velocity and acceleration limits.
 * @{
 */

/**
 * @brief Default pulses per unit (meter)
 *
 * 1,000,000 pulses/meter = 1 micrometer resolution.
 * Adjust based on motor steps/rev, gear ratio, and lead screw pitch.
 */
#define DEFAULT_PULSES_PER_UNIT     1000000.0f

/** @brief Default axis type (false = linear, true = rotary) */
#define DEFAULT_IS_ROTARY           false

/** @brief Default minimum position limit (meters) */
#define DEFAULT_LIMIT_MIN           -1.0f

/** @brief Default maximum position limit (meters) */
#define DEFAULT_LIMIT_MAX           1.0f

/**
 * @brief Default maximum velocity (m/s)
 *
 * 0.1 m/s = 100 mm/s - a conservative default.
 */
#define DEFAULT_MAX_VELOCITY        0.1f

/** @brief Default maximum acceleration (m/s^2) */
#define DEFAULT_MAX_ACCELERATION    1.0f

/** @brief Default backlash compensation (meters) */
#define DEFAULT_BACKLASH            0.0f

/** @brief Default home position offset (meters) */
#define DEFAULT_HOME_OFFSET         0.0f

/** @} */ // end linear_defaults

/**
 * @defgroup rotary_defaults Default Rotary Axis Configuration
 * @brief Default parameters for rotary (angular) axes in radians
 *
 * Rotary axes use radians for position and rad/s for velocity.
 * @{
 */

/**
 * @brief Default rotary pulses per unit (pulses/radian)
 *
 * 10,000 pulses/radian provides good angular resolution.
 */
#define DEFAULT_ROTARY_PULSES_PER_UNIT  10000.0f

/** @brief Default rotary minimum position limit (radians) */
#define DEFAULT_ROTARY_LIMIT_MIN        0.0f

/**
 * @brief Default rotary maximum position limit (radians)
 *
 * 2*Pi radians = 360 degrees (full rotation).
 */
#define DEFAULT_ROTARY_LIMIT_MAX        CONST_2PI

/**
 * @brief Default rotary maximum velocity (rad/s)
 *
 * Pi rad/s = 180 degrees/second.
 */
#define DEFAULT_ROTARY_MAX_VEL          CONST_PI

/** @brief Default rotary maximum acceleration (rad/s^2) */
#define DEFAULT_ROTARY_MAX_ACCEL        10.0f

/** @} */ // end rotary_defaults

/**
 * @defgroup e_axis_defaults E-Axis (Discrete Actuator) Configuration
 * @brief Configuration for binary position discrete axis
 *
 * The E-axis operates as a discrete actuator (e.g., pneumatic cylinder)
 * with only two positions: 0 (retracted) and 1 (extended).
 * @{
 */

/**
 * @brief E-axis pulses per unit
 *
 * 1.0 means position units directly map to actuator state.
 * Position 0 = retracted, position 1 = extended.
 */
#define E_AXIS_PULSES_PER_UNIT      1.0f

/** @brief E-axis minimum position (retracted) */
#define E_AXIS_LIMIT_MIN            0.0f

/** @brief E-axis maximum position (extended) */
#define E_AXIS_LIMIT_MAX            1.0f

/**
 * @brief E-axis "velocity" (transition time in units/sec)
 *
 * For discrete actuators, this represents the transition speed.
 * A value of 1.0 means full travel in 1 second (if applicable).
 */
#define E_AXIS_MAX_VELOCITY         1.0f

/**
 * @brief E-axis acceleration (not applicable for discrete)
 *
 * Discrete actuators don't accelerate, but a value is needed
 * for interface consistency.
 */
#define E_AXIS_MAX_ACCELERATION     10.0f

/** @} */ // end e_axis_defaults

/**
 * @defgroup homing_defaults Default Homing Parameters
 * @brief Default values for homing sequences
 * @{
 */

/**
 * @brief Default homing velocity (m/s or rad/s)
 *
 * Slow speed for approaching limit switch during homing.
 */
#define DEFAULT_HOMING_VELOCITY         0.01f

/**
 * @brief Default slow homing velocity for Z-signal seek (m/s or rad/s)
 *
 * Very slow speed for precise Z-signal capture.
 */
#define DEFAULT_HOMING_VELOCITY_SLOW    0.005f

/**
 * @brief Default backoff distance (m or rad)
 *
 * Distance to move away from limit before seeking Z-signal.
 */
#define DEFAULT_HOMING_BACKOFF          0.005f

/** @} */ // end homing_defaults

/**
 * @defgroup zsignal_defaults Default Z-Signal Parameters
 * @brief Default values for encoder index (Z-signal) synchronization
 * @{
 */

/** @brief Default Z-signal drift threshold in pulses (0 = no alarm) */
#define DEFAULT_ZSIG_DRIFT_THRESHOLD    0

/** @brief Z-signal enabled by default for servo axes */
#define DEFAULT_ZSIG_ENABLED            true

/** @} */ // end zsignal_defaults

/** @} */ // end config_defaults

#endif // CONFIG_DEFAULTS_H
