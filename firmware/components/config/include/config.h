/**
 * @file config.h
 * @brief Master configuration header for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note This is the main include file for all configuration constants.
 *       Include this file to access all configuration headers.
 *
 * @code
 * #include "config.h"
 * // Now you have access to all GPIO, timing, limits, commands, etc.
 * @endcode
 */

#ifndef CONFIG_H
#define CONFIG_H

/**
 * @defgroup config Master Configuration
 * @brief Central configuration header including all sub-headers
 * @{
 */

/**
 * @defgroup firmware_version Firmware Version
 * @brief Version information for the control unit firmware
 * @{
 */

/** @brief Firmware product name */
#define FIRMWARE_NAME               "YAROBOT_CONTROL_UNIT"

/** @brief Major version number */
#define FIRMWARE_VERSION_MAJOR      1

/** @brief Minor version number */
#define FIRMWARE_VERSION_MINOR      0

/** @brief Patch version number */
#define FIRMWARE_VERSION_PATCH      0

/** @brief Version string for display and logging */
#define FIRMWARE_VERSION_STRING     "1.0.0"

/** @} */ // end firmware_version

/**
 * @defgroup feature_flags Feature Flags
 * @brief Compile-time feature enable/disable switches
 *
 * Set to 1 to enable, 0 to disable at compile time.
 * @{
 */

/**
 * @brief Enable OLED display support
 *
 * When enabled, the OLED display task is created and status is shown.
 */
#define FEATURE_OLED_DISPLAY        1

/**
 * @brief Enable Z-signal (encoder index) support
 *
 * When enabled, servo axes can use Z-signal for position synchronization.
 */
#define FEATURE_Z_SIGNAL            1

/**
 * @brief Enable streaming position mode
 *
 * When enabled, continuous position streaming via USB is available.
 */
#define FEATURE_STREAMING           1

/** @} */ // end feature_flags

/*
 * Include all configuration sub-headers
 */

/** @brief GPIO pin assignments */
#include "config_gpio.h"

/** @brief ESP32 peripheral assignments (RMT, MCPWM, PCNT, LEDC, SPI) */
#include "config_peripherals.h"

/** @brief Timing constants (ms, µs) */
#include "config_timing.h"

/** @brief Buffer sizes, queue depths, axis counts, stack sizes */
#include "config_limits.h"

/** @brief Command strings, response prefixes, error codes, event types */
#include "config_commands.h"

/** @brief I2C addresses and MCP23017 pin mappings */
#include "config_i2c.h"

/** @brief Shift register bit positions and helper macros */
#include "config_sr.h"

/** @brief Default axis parameters (SI units) */
#include "config_defaults.h"

/** @brief OLED display configuration */
#include "config_oled.h"

/** @brief YAML schema keys, NVS mappings, validation macros */
#include "config_yaml_schema.h"

/** @} */ // end config

#endif // CONFIG_H
