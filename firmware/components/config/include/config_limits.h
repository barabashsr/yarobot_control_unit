/**
 * @file config_limits.h
 * @brief Buffer sizes, queue depths, axis counts, and stack sizes
 * @author YaRobot Team
 * @date 2025
 *
 * @note All numeric limits for the control system are defined here.
 *       Includes compile-time validation via static_assert.
 */

#ifndef CONFIG_LIMITS_H
#define CONFIG_LIMITS_H

#include <assert.h>

/**
 * @defgroup config_limits System Limits and Sizes
 * @brief Buffer sizes, queue depths, axis counts, and memory allocation limits
 * @{
 */

/**
 * @defgroup limits_buffer Buffer Sizes
 * @brief Maximum sizes for command/response buffers
 * @{
 */

/** @brief Maximum command line length (bytes) */
#define LIMIT_CMD_MAX_LENGTH        256

/** @brief Maximum response line length (bytes) */
#define LIMIT_RESPONSE_MAX_LENGTH   256

/** @brief YAML configuration buffer size (bytes) - ERROR E030 if exceeded */
#define LIMIT_YAML_BUFFER_SIZE      8192

/** @brief Maximum axis alias length (characters) */
#define LIMIT_ALIAS_MAX_LENGTH      16

/** @} */ // end limits_buffer

/**
 * @defgroup limits_queue Queue Depths
 * @brief FreeRTOS queue capacities
 * @{
 */

/** @brief Command queue depth (pending commands) */
#define LIMIT_COMMAND_QUEUE_DEPTH       32

/** @brief Response queue depth (pending responses) */
#define LIMIT_RESPONSE_QUEUE_DEPTH      32

/** @brief Safety event queue depth */
#define LIMIT_SAFETY_QUEUE_DEPTH        64

/** @brief Event notification queue depth */
#define LIMIT_EVENT_QUEUE_DEPTH         32

/** @brief Maximum subscribers per event type */
#define LIMIT_EVENT_SUBSCRIBERS         8

/** @brief Background work queue depth */
#define LIMIT_BACKGROUND_QUEUE_DEPTH    32

/** @brief Error info queue depth */
#define LIMIT_ERROR_INFO_QUEUE_DEPTH    16

/** @brief Motion command queue depth per axis */
#define LIMIT_MOTION_QUEUE_DEPTH        4

/** @} */ // end limits_queue

/**
 * @defgroup limits_axis Axis Configuration
 * @brief Number of axes by type
 *
 * @note static_assert validates: SERVOS + STEPPERS + DISCRETE == TOTAL
 * @{
 */

/** @brief Total number of axes (X, Y, Z, A, B, C, D, E) */
#define LIMIT_NUM_AXES              8

/** @brief Number of servo axes (X, Y, Z, A, B) */
#define LIMIT_NUM_SERVOS            5

/** @brief Number of stepper axes (C, D) */
#define LIMIT_NUM_STEPPERS          2

/** @brief Number of discrete axes (E) */
#define LIMIT_NUM_DISCRETE          1

/** @brief Number of motor axes (X-D, excludes discrete E) */
#define LIMIT_NUM_MOTOR_AXES        7

/** @brief Number of NVS configuration slots */
#define LIMIT_CONFIG_SLOTS          10

/** @} */ // end limits_axis

/**
 * @defgroup limits_motion Motion Limits
 * @brief Pulse frequency limits for motor drivers
 *
 * @note LIMIT_MAX_PULSE_FREQ_HZ is the absolute hardware capability (500 kHz).
 *       DEFAULT_MAX_PULSE_FREQ_HZ is the safe per-axis default (200 kHz).
 *       Per-axis max velocity can be configured between MIN and MAX.
 * @{
 */

/**
 * @brief Absolute maximum pulse frequency (Hz) - hardware capability
 *
 * This is the maximum frequency the RMT peripheral can generate with 50% duty
 * cycle at 80 MHz resolution (160 ticks/period = 2µs period = 500 kHz).
 * Do not exceed this value.
 */
#define LIMIT_MAX_PULSE_FREQ_HZ     500000

/**
 * @brief Default per-axis maximum pulse frequency (Hz)
 *
 * This is the recommended safe default for servo axes. Individual axes can
 * be configured up to LIMIT_MAX_PULSE_FREQ_HZ if the mechanical system allows.
 */
#define DEFAULT_MAX_PULSE_FREQ_HZ   200000

/** @brief Minimum pulse frequency (Hz) */
#define LIMIT_MIN_PULSE_FREQ_HZ     1

/**
 * @brief RMT peripheral resolution (Hz)
 *
 * 80 MHz clock provides 12.5ns resolution per tick.
 * At 500 kHz: period = 2µs = 160 ticks (80 high + 80 low for 50% duty)
 * At 200 kHz: period = 5µs = 400 ticks (200 high + 200 low for 50% duty)
 */
#define LIMIT_RMT_RESOLUTION_HZ     80000000

/** @} */ // end limits_motion

/**
 * @defgroup limits_stack FreeRTOS Task Stack Sizes
 * @brief Stack allocation for each task (words, not bytes)
 *
 * @note ESP32-S3 uses 4-byte words, so 2048 words = 8192 bytes
 * @{
 */

/** @brief Safety monitor task stack size (bytes) */
#define STACK_SAFETY_TASK           8192

/** @brief USB receive task stack size (bytes) */
#define STACK_USB_RX_TASK           8192

/** @brief USB transmit task stack size (bytes) */
#define STACK_USB_TX_TASK           8192

/** @brief Command executor task stack size (bytes) */
#define STACK_CMD_EXECUTOR_TASK     16384

/** @brief I2C monitor task stack size (bytes) */
#define STACK_I2C_MONITOR_TASK      8192

/** @brief Motion control task stack size (bytes) */
#define STACK_MOTION_TASK           8192

/** @brief OLED display task stack size (bytes) */
#define STACK_DISPLAY_TASK          8192

/** @brief Idle monitor task stack size (bytes) */
#define STACK_IDLE_MONITOR_TASK     8192

/** @brief Background worker task stack size (bytes) */
#define STACK_BACKGROUND_TASK       8192

/** @brief Event processor task stack size (bytes) */
#define STACK_EVENT_TASK            4096

/** @} */ // end limits_stack

/**
 * @defgroup limits_validation Compile-Time Validation
 * @brief Static assertions to catch configuration errors at build time
 * @{
 */

/**
 * @brief Validate axis count consistency
 *
 * Ensures that servo + stepper + discrete axes equals total axes.
 * Build will fail if this invariant is violated.
 */
static_assert(LIMIT_NUM_SERVOS + LIMIT_NUM_STEPPERS + LIMIT_NUM_DISCRETE == LIMIT_NUM_AXES,
              "Axis count mismatch: servo + stepper + discrete must equal total axes");

/**
 * @brief Validate pulse frequency limits
 *
 * Ensures that default max frequency does not exceed absolute hardware limit.
 * Build will fail if DEFAULT_MAX > LIMIT_MAX.
 */
static_assert(DEFAULT_MAX_PULSE_FREQ_HZ <= LIMIT_MAX_PULSE_FREQ_HZ,
              "DEFAULT_MAX_PULSE_FREQ_HZ must not exceed LIMIT_MAX_PULSE_FREQ_HZ");

/**
 * @brief Validate minimum frequency is positive
 */
static_assert(LIMIT_MIN_PULSE_FREQ_HZ >= 1,
              "LIMIT_MIN_PULSE_FREQ_HZ must be at least 1 Hz");

/** @} */ // end limits_validation

/** @} */ // end config_limits

#endif // CONFIG_LIMITS_H
