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

// ============================================================================
// PER-AXIS HARDWARE TEST CONFIGURATION (2025-12-18) - ENABLED
// Hardware test mode with mm units
// ============================================================================
#define X_AXIS_PULSES_PER_REV       10000.0f
#define X_AXIS_UNITS_PER_REV        350.0f   // mm

#define Y_AXIS_PULSES_PER_REV       10000.0f
#define Y_AXIS_UNITS_PER_REV        48.0f    // mm

#define Z_AXIS_PULSES_PER_REV       312.0f
#define Z_AXIS_UNITS_PER_REV        5.0f     // mm

#define A_AXIS_PULSES_PER_REV       625.0f
#define A_AXIS_UNITS_PER_REV        3.8f     // mm

#define B_AXIS_PULSES_PER_REV       10000.0f
#define B_AXIS_UNITS_PER_REV        72.0f    // mm

#define C_AXIS_PULSES_PER_REV       3200.0f
#define C_AXIS_UNITS_PER_REV        48.0f    // mm

#define D_AXIS_PULSES_PER_REV       3200.0f
#define D_AXIS_UNITS_PER_REV        48.0f    // mm
// ============================================================================

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
 * @defgroup limit_defaults Limit Switch Defaults
 * @brief Default values for limit switch configuration
 * @{
 */

/**
 * @brief Default limit switch polarity
 *
 * 0 = NO (normally open): switch closes circuit when triggered
 * 1 = NC (normally closed): switch opens circuit when triggered
 */
#define DEFAULT_LIMIT_POLARITY          0

/** @} */ // end limit_defaults

/**
 * @defgroup brake_strategy Brake Control Strategy
 * @brief Per-axis brake engagement strategy configuration
 * @{
 */

/**
 * @brief Brake control strategy enumeration
 *
 * Defines when brakes engage/release for each axis. Servo axes (X, Y, Z, A, B)
 * have electromagnetic spring-applied brakes. Stepper axes (C, D) and discrete
 * axis (E) have no brake hardware.
 *
 * @note Active-low brake outputs: SR bit = 0 means brake ENGAGED (spring applied),
 *       SR bit = 1 means brake RELEASED (electromagnet powered).
 */
typedef enum {
    /**
     * @brief Engage brake when axis is disabled (default for vertical axes)
     *
     * On EN X 0: Engage brake -> wait TIMING_BRAKE_ENGAGE_MS -> clear enable bit
     * On EN X 1: Set enable bit -> wait TIMING_BRAKE_RELEASE_MS -> release brake
     */
    BRAKE_ON_DISABLE = 0,

    /**
     * @brief Engage brake only during E-stop (default for horizontal axes)
     *
     * Normal disable (EN X 0) does NOT engage brake.
     * E-stop engages brake via sr_emergency_disable_all().
     */
    BRAKE_ON_ESTOP = 1,

    /**
     * @brief Engage brake after idle timeout
     *
     * Axis remains enabled but brake engages after TIMING_IDLE_TIMEOUT_S.
     * Brake releases automatically before motion commands.
     * Publishes EVENT BRAKE <axis> ENGAGED when idle brake engages.
     */
    BRAKE_ON_IDLE = 2,

    /**
     * @brief Manual brake control via BRAKE command only
     *
     * Brake state only changes via explicit BRAKE <axis> <0|1> command.
     * Other axes reject BRAKE command with ERR_BRAKE_AUTO.
     */
    BRAKE_MANUAL = 3
} BrakeStrategy;

/**
 * @brief Default brake strategy for Z axis (vertical - needs holding)
 */
#define DEFAULT_BRAKE_STRATEGY_Z        BRAKE_ON_DISABLE

/**
 * @brief Default brake strategy for horizontal servo axes (X, Y, A, B)
 */
#define DEFAULT_BRAKE_STRATEGY_HORIZ    BRAKE_ON_ESTOP

/** @} */ // end brake_strategy

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
