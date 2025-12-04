/**
 * @file config_yaml_schema.h
 * @brief YAML configuration schema keys, NVS mappings, and validation macros
 * @author YaRobot Team
 * @date 2025
 *
 * @note Defines all string keys used in YAML configuration files,
 *       NVS storage key formats, and validation helper macros.
 */

#ifndef CONFIG_YAML_SCHEMA_H
#define CONFIG_YAML_SCHEMA_H

#include <stdio.h>
#include <string.h>
#include "config_limits.h"
#include "config_defaults.h"

/**
 * @defgroup config_yaml_schema YAML Schema Configuration
 * @brief YAML key definitions, NVS mappings, and validation
 * @{
 */

/**
 * @defgroup yaml_version Schema Version
 * @brief YAML configuration format version tracking
 * @{
 */

/** @brief Current YAML schema version */
#define YAML_SCHEMA_VERSION     "1.0"

/** @} */ // end yaml_version

/**
 * @defgroup yaml_root Root YAML Keys
 * @brief Top-level section keys in configuration files
 * @{
 */

/** @brief Schema version key */
#define YAML_KEY_VERSION            "version"

/** @brief Axes configuration section */
#define YAML_KEY_AXES               "axes"

/** @brief System configuration section */
#define YAML_KEY_SYSTEM             "system"

/** @} */ // end yaml_root

/**
 * @defgroup yaml_axis_names Axis Name Keys
 * @brief YAML section keys for each axis (used under "axes:")
 * @{
 */

/** @brief X-axis section key */
#define YAML_AXIS_X     "X"
/** @brief Y-axis section key */
#define YAML_AXIS_Y     "Y"
/** @brief Z-axis section key */
#define YAML_AXIS_Z     "Z"
/** @brief A-axis section key */
#define YAML_AXIS_A     "A"
/** @brief B-axis section key */
#define YAML_AXIS_B     "B"
/** @brief C-axis section key */
#define YAML_AXIS_C     "C"
/** @brief D-axis section key */
#define YAML_AXIS_D     "D"
/** @brief E-axis section key */
#define YAML_AXIS_E     "E"

/**
 * @brief Array of axis name strings for iteration
 *
 * Usage: YAML_AXIS_NAMES[axis_idx] gives the axis letter string.
 */
#define YAML_AXIS_NAMES { "X", "Y", "Z", "A", "B", "C", "D", "E" }

/** @} */ // end yaml_axis_names

/**
 * @defgroup yaml_axis_params Per-Axis Parameter Keys
 * @brief Keys used within each axis configuration section
 * @{
 */

/** @brief Axis alias (user-friendly name) */
#define YAML_KEY_ALIAS              "alias"

/** @brief Axis type ("linear" or "rotary") */
#define YAML_KEY_TYPE               "type"

/** @brief Pulses per motor revolution (driver setting or gear ratio) */
#define YAML_KEY_PULSES_PER_REV     "pulses_per_rev"

/** @brief Physical units per motor revolution (meters or radians) */
#define YAML_KEY_UNITS_PER_REV      "units_per_rev"

/** @brief Position limits [min, max] */
#define YAML_KEY_LIMITS             "limits"

/** @brief Maximum velocity (m/s or rad/s) */
#define YAML_KEY_MAX_VELOCITY       "max_velocity"

/** @brief Maximum acceleration (m/s^2 or rad/s^2) */
#define YAML_KEY_MAX_ACCELERATION   "max_acceleration"

/** @brief Backlash compensation value */
#define YAML_KEY_BACKLASH           "backlash"

/** @brief Home position offset from limit switch */
#define YAML_KEY_HOME_OFFSET        "home_offset"

/** @} */ // end yaml_axis_params

/**
 * @defgroup yaml_homing Homing Configuration Keys
 * @brief Keys for homing sequence configuration (nested under "homing")
 * @{
 */

/** @brief Homing configuration section */
#define YAML_KEY_HOMING             "homing"

/** @brief Homing velocity for limit seek */
#define YAML_KEY_HOMING_VELOCITY    "velocity"

/** @brief Slow velocity for Z-signal seek */
#define YAML_KEY_HOMING_VELOCITY_SLOW "velocity_slow"

/** @brief Backoff distance from limit */
#define YAML_KEY_HOMING_BACKOFF     "backoff"

/** @brief Homing direction ("min" or "max") */
#define YAML_KEY_HOMING_DIRECTION   "direction"

/** @} */ // end yaml_homing

/**
 * @defgroup yaml_zsignal Z-Signal Configuration Keys
 * @brief Keys for encoder index pulse configuration (nested under "z_signal")
 * @{
 */

/** @brief Z-signal configuration section */
#define YAML_KEY_ZSIGNAL            "z_signal"

/** @brief Z-signal synchronization enabled */
#define YAML_KEY_ZSIGNAL_ENABLED    "enabled"

/** @brief Z-signal drift alarm threshold (pulses) */
#define YAML_KEY_ZSIGNAL_THRESHOLD  "drift_threshold"

/** @brief Z-signal fallback behavior ("auto", "confirm", "fail") */
#define YAML_KEY_ZSIGNAL_FALLBACK   "fallback"

/** @} */ // end yaml_zsignal

/**
 * @defgroup yaml_switches End Switch Configuration Keys
 * @brief Keys for limit switch configuration (nested under "switches")
 * @{
 */

/** @brief Switch configuration section */
#define YAML_KEY_SWITCHES           "switches"

/** @brief MIN limit switch configuration */
#define YAML_KEY_SWITCH_MIN         "min"

/** @brief MAX limit switch configuration */
#define YAML_KEY_SWITCH_MAX         "max"

/** @brief Switch mode ("NONE", "HARD_STOP", "RESTRICT", "EVENT_ONLY") */
#define YAML_KEY_SWITCH_MODE        "mode"

/** @brief Per-switch debounce override (ms) */
#define YAML_KEY_SWITCH_DEBOUNCE    "debounce_ms"

/** @} */ // end yaml_switches

/**
 * @defgroup yaml_safety Safety Configuration Keys
 * @brief Keys for calibration and safety parameters
 * @{
 */

/** @brief Maximum mechanical travel for calibration timeout */
#define YAML_KEY_MECHANICAL_MAX     "mechanical_max_travel"

/** @brief Soft limit buffer distance */
#define YAML_KEY_SOFT_LIMIT_BUFFER  "soft_limit_buffer"

/** @} */ // end yaml_safety

/**
 * @defgroup yaml_type_values Axis Type Values
 * @brief String values for YAML_KEY_TYPE
 * @{
 */

/** @brief Linear axis type (meters) */
#define YAML_VAL_TYPE_LINEAR        "linear"

/** @brief Rotary axis type (radians) */
#define YAML_VAL_TYPE_ROTARY        "rotary"

/** @} */ // end yaml_type_values

/**
 * @defgroup yaml_system System Parameter Keys
 * @brief Keys for system-wide configuration
 * @{
 */

/** @brief Legacy debounce key (use default_switch_debounce_ms) */
#define YAML_KEY_DEBOUNCE_MS                "debounce_ms"

/** @brief System-wide default switch debounce (ms) */
#define YAML_KEY_DEFAULT_SWITCH_DEBOUNCE    "default_switch_debounce_ms"

/** @brief Idle timeout before brake engage (seconds) */
#define YAML_KEY_IDLE_TIMEOUT_S             "idle_timeout_s"

/** @brief Logging level */
#define YAML_KEY_LOG_LEVEL                  "log_level"

/** @} */ // end yaml_system

/**
 * @defgroup yaml_log_levels Log Level Values
 * @brief String values for YAML_KEY_LOG_LEVEL (matches ESP-IDF levels)
 * @{
 */

/** @brief No logging output */
#define YAML_LOG_NONE       "NONE"
/** @brief Error messages only */
#define YAML_LOG_ERROR      "ERROR"
/** @brief Warnings and errors */
#define YAML_LOG_WARN       "WARN"
/** @brief Informational messages */
#define YAML_LOG_INFO       "INFO"
/** @brief Debug output */
#define YAML_LOG_DEBUG      "DEBUG"
/** @brief Verbose debug output */
#define YAML_LOG_VERBOSE    "VERBOSE"

/** @} */ // end yaml_log_levels

/**
 * @defgroup nvs_keys NVS Storage Key Mappings
 * @brief Non-Volatile Storage key format definitions
 *
 * NVS keys have a 15-character maximum length.
 * Format: prefix + axis_index + suffix (e.g., "ax0_vel" for axis 0 velocity)
 * @{
 */

/** @brief NVS namespace for all yarobot configuration */
#define NVS_NAMESPACE               "yarobot"

/** @brief Axis key prefix ("ax" + index) */
#define NVS_KEY_PREFIX_AXIS         "ax"

/** @brief Pulses per revolution suffix */
#define NVS_KEY_SUFFIX_PPR          "_ppr"

/** @brief Units per revolution suffix */
#define NVS_KEY_SUFFIX_UPR          "_upr"

/** @brief Is rotary axis suffix */
#define NVS_KEY_SUFFIX_ROTARY       "_rot"

/** @brief Minimum limit suffix */
#define NVS_KEY_SUFFIX_MIN          "_min"

/** @brief Maximum limit suffix */
#define NVS_KEY_SUFFIX_MAX          "_max"

/** @brief Maximum velocity suffix */
#define NVS_KEY_SUFFIX_VEL          "_vel"

/** @brief Maximum acceleration suffix */
#define NVS_KEY_SUFFIX_ACC          "_acc"

/** @brief Backlash compensation suffix */
#define NVS_KEY_SUFFIX_BACKLASH     "_blash"

/** @brief Axis alias suffix */
#define NVS_KEY_SUFFIX_ALIAS        "_alias"

/** @brief Home offset suffix */
#define NVS_KEY_SUFFIX_HOME_OFF     "_home_off"

/** @brief Z-signal enabled suffix */
#define NVS_KEY_SUFFIX_ZSIG_EN      "_zsen"

/** @brief Z-signal drift threshold suffix */
#define NVS_KEY_SUFFIX_ZSIG_THR     "_zsth"

/** @} */ // end nvs_keys

/**
 * @defgroup nvs_sys_keys System NVS Keys
 * @brief Non-Volatile Storage keys for system settings
 * @{
 */

/** @brief Configuration schema version */
#define NVS_KEY_SYS_VERSION         "sys_version"

/** @brief Active configuration slot */
#define NVS_KEY_SYS_SLOT            "sys_slot"

/** @brief System-wide debounce setting */
#define NVS_KEY_SYS_DEBOUNCE        "sys_debounce"

/** @} */ // end nvs_sys_keys

/**
 * @defgroup yaml_validation Validation Macros
 * @brief Compile-time and runtime validation helpers
 * @{
 */

/**
 * @brief Calculate pulses per unit from pulses/rev and units/rev
 * @param ppr Pulses per revolution
 * @param upr Units per revolution (meters or radians)
 * @return Pulses per unit (pulses/meter or pulses/radian)
 */
#define YAML_PULSES_PER_UNIT(ppr, upr)  ((float)(ppr) / (upr))

/**
 * @brief Calculate maximum velocity from pulse rate limit
 * @param ppr Pulses per revolution
 * @param upr Units per revolution
 * @return Maximum velocity in units/second based on LIMIT_MAX_PULSE_FREQ_HZ
 */
#define YAML_MAX_VELOCITY(ppr, upr) \
    ((float)LIMIT_MAX_PULSE_FREQ_HZ / YAML_PULSES_PER_UNIT(ppr, upr))

/**
 * @brief Calculate minimum velocity from pulse rate limit
 * @param ppr Pulses per revolution
 * @param upr Units per revolution
 * @return Minimum velocity in units/second based on LIMIT_MIN_PULSE_FREQ_HZ
 */
#define YAML_MIN_VELOCITY(ppr, upr) \
    ((float)LIMIT_MIN_PULSE_FREQ_HZ / YAML_PULSES_PER_UNIT(ppr, upr))

/**
 * @brief Validate velocity against pulse rate limits
 * @param vel Velocity to validate (m/s or rad/s)
 * @param ppr Pulses per revolution
 * @param upr Units per revolution
 * @return true if velocity is within valid range
 */
#define YAML_VALIDATE_VELOCITY(vel, ppr, upr) \
    ((vel) >= YAML_MIN_VELOCITY(ppr, upr) && \
     (vel) <= YAML_MAX_VELOCITY(ppr, upr))

/**
 * @brief Validate pulses per revolution
 * @param ppr Pulses per revolution value
 * @return true if value is valid (positive and <= 1,000,000)
 */
#define YAML_VALIDATE_PPR(ppr) \
    ((ppr) > 0 && (ppr) <= 1000000)

/**
 * @brief Validate units per revolution
 * @param upr Units per revolution value
 * @return true if value is positive
 */
#define YAML_VALIDATE_UPR(upr) \
    ((upr) > 0.0f)

/**
 * @brief Validate alias string length
 * @param alias Pointer to alias string
 * @return true if length is within LIMIT_ALIAS_MAX_LENGTH
 */
#define YAML_VALIDATE_ALIAS_LEN(alias) \
    (strlen(alias) <= LIMIT_ALIAS_MAX_LENGTH)

/**
 * @brief Validate axis index
 * @param idx Axis index (0-7)
 * @return true if index is valid
 */
#define YAML_VALIDATE_AXIS(idx) \
    ((idx) >= 0 && (idx) < LIMIT_NUM_AXES)

/**
 * @brief Validate position limits (min must be less than max)
 * @param min Minimum limit value
 * @param max Maximum limit value
 * @return true if min < max
 */
#define YAML_VALIDATE_LIMITS(min, max) \
    ((min) < (max))

/**
 * @brief Validate Z-signal drift threshold
 * @param thr Threshold value in pulses
 * @return true if threshold is non-negative
 */
#define YAML_VALIDATE_ZSIG_THRESHOLD(thr) \
    ((thr) >= 0)

/** @} */ // end yaml_validation

/**
 * @defgroup nvs_key_builder NVS Key Builder Macro
 * @brief Helper for constructing axis-specific NVS keys
 * @{
 */

/**
 * @brief Build an axis-specific NVS key
 *
 * Constructs a key string in the format "axN_suffix" where N is the axis index.
 *
 * @param buf Character buffer to write key into (must be at least 16 bytes)
 * @param axis_idx Axis index (0-7)
 * @param suffix Key suffix (e.g., NVS_KEY_SUFFIX_VEL)
 *
 * @code
 * char key[16];
 * NVS_AXIS_KEY(key, 0, NVS_KEY_SUFFIX_VEL);  // key = "ax0_vel"
 * @endcode
 */
#define NVS_AXIS_KEY(buf, axis_idx, suffix) \
    snprintf((buf), sizeof(buf), "%s%d%s", NVS_KEY_PREFIX_AXIS, (axis_idx), (suffix))

/** @} */ // end nvs_key_builder

/** @} */ // end config_yaml_schema

#endif // CONFIG_YAML_SCHEMA_H
