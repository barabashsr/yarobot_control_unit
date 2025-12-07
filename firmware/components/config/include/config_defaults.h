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
 * @defgroup linear_defaults Default Axis Configuration
 * @brief Default parameters for all axes in degrees
 *
 * Configuration for servo/stepper axes with 200 pulses/rev drivers.
 * Units are degrees for human-readable position values.
 * @{
 */

/**
 * @brief Default pulses per revolution
 *
 * 200 pulses/rev matches common servo driver PA14 settings.
 */
#define DEFAULT_PULSES_PER_REV      200.0f

/**
 * @brief Default units per revolution (degrees)
 *
 * 360 degrees per revolution for human-readable positions.
 */
#define DEFAULT_UNITS_PER_REV       360.0f

/** @brief Default axis type (false = linear, true = rotary) */
#define DEFAULT_IS_ROTARY           false

/** @brief Default minimum position limit (degrees) - 100 revolutions negative */
#define DEFAULT_LIMIT_MIN           -360000.0f

/** @brief Default maximum position limit (degrees) - 100 revolutions positive */
#define DEFAULT_LIMIT_MAX           360000.0f

/**
 * @brief Default maximum velocity (deg/s)
 *
 * 1080 deg/s = 3 rev/s = 180 RPM = 600 Hz pulse output
 * (with 200 pulses/rev: 1080 * 200/360 = 600 Hz)
 */
#define DEFAULT_MAX_VELOCITY        1080.0f

/** @brief Default maximum acceleration (deg/s^2) - reaches max velocity in 0.1s */
#define DEFAULT_MAX_ACCELERATION    10800.0f

/** @brief Default backlash compensation (meters) */
#define DEFAULT_BACKLASH            0.0f

/** @brief Default home position offset (meters) */
#define DEFAULT_HOME_OFFSET         0.0f

/** @} */ // end linear_defaults

/**
 * @defgroup rotary_defaults Default Rotary Axis Configuration
 * @brief Default parameters for rotary axes (same as linear for unified config)
 *
 * All axes use degrees for position. Rotary axes share the same defaults.
 * @{
 */

/** @brief Default rotary pulses per revolution (same as linear) */
#define DEFAULT_ROTARY_PULSES_PER_REV   DEFAULT_PULSES_PER_REV

/** @brief Default rotary units per revolution (degrees) */
#define DEFAULT_ROTARY_UNITS_PER_REV    DEFAULT_UNITS_PER_REV

/** @brief Default rotary minimum position limit (degrees) */
#define DEFAULT_ROTARY_LIMIT_MIN        DEFAULT_LIMIT_MIN

/** @brief Default rotary maximum position limit (degrees) */
#define DEFAULT_ROTARY_LIMIT_MAX        DEFAULT_LIMIT_MAX

/** @brief Default rotary maximum velocity (deg/s) */
#define DEFAULT_ROTARY_MAX_VEL          DEFAULT_MAX_VELOCITY

/** @brief Default rotary maximum acceleration (deg/s^2) */
#define DEFAULT_ROTARY_MAX_ACCEL        DEFAULT_MAX_ACCELERATION

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

/**
 * @defgroup format_defaults Position Display Format Constants
 * @brief Formatting for position output in POS command
 * @{
 */

/**
 * @brief Number of decimal places for position output
 *
 * 6 decimal places provides micrometer (linear) or microradian (rotary) precision.
 * Used in POS command response formatting.
 */
#define DEFAULT_POSITION_DECIMALS       6

/**
 * @brief Printf format string for single position value
 */
#define DEFAULT_POSITION_FMT            "%0.6f"

/** @} */ // end format_defaults

/** @} */ // end config_defaults

#endif // CONFIG_DEFAULTS_H
