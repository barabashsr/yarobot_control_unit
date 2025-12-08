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

/**
 * @brief Minimum pulse frequency (Hz)
 *
 * Set to 100 Hz for responsive motor startup.
 * With MCPWM_RESOLUTION_HZ = 100000: period = 100000/100 = 1000 ticks
 * Lower values cause long delays during acceleration ramp-up.
 */
#define LIMIT_MIN_PULSE_FREQ_HZ     100

/**
 * @brief Maximum stop latency for immediate stop (microseconds)
 *
 * stopImmediate() must complete within this time.
 * 100µs provides adequate margin for LEDC and RMT stop operations.
 */
#define LIMIT_STOP_LATENCY_US       100

/**
 * @brief LEDC maximum frequency (Hz) at configured resolution
 *
 * At 10-bit resolution (1024 levels) with 80 MHz APB clock:
 * max_freq = APB_CLK / 2^resolution = 80000000 / 1024 ≈ 78125 Hz
 * Conservative limit set below theoretical max for stability.
 */
#define LIMIT_LEDC_MAX_FREQ_HZ      75000

/**
 * @brief LEDC minimum frequency (Hz) at configured resolution
 *
 * At 10-bit resolution with low-speed timer, minimum frequency is limited
 * by the maximum clock divider. With APB_CLK = 80 MHz and 18-bit divider:
 * min_freq ≈ 80000000 / (2^18 * 2^10) ≈ 0.3 Hz (theoretical)
 *
 * In practice, very low frequencies cause timer instability. 100 Hz provides
 * a safe minimum for D-axis stepper positioning while allowing slow moves.
 */
#define LIMIT_LEDC_MIN_FREQ_HZ      100

/**
 * @brief RMT clock resolution (Hz)
 *
 * FastAccelStepper uses 16 MHz providing 62.5ns per tick.
 * At 500 kHz output: period = 32 ticks (adequate precision).
 * At 200 kHz output: period = 80 ticks (good precision).
 *
 * @note Changed from 80 MHz to match FastAccelStepper proven configuration.
 */
#define LIMIT_RMT_RESOLUTION_HZ         16000000

/**
 * @brief RMT memory block size (symbols per channel)
 *
 * ESP32-S3 has SOC_RMT_MEM_WORDS_PER_CHANNEL = 48 symbols per channel.
 * With callback encoder (no DMA), we use exactly one channel's worth.
 *
 * @note Must equal SOC_RMT_MEM_WORDS_PER_CHANNEL for ESP32-S3.
 */
#define LIMIT_RMT_MEM_BLOCK_SYMBOLS     48

/**
 * @brief Symbols per encoder callback (half of memory block)
 *
 * RMT double-buffers internally. Encoder fills PART_SIZE symbols per call.
 * At 500 kHz: 24 symbols = 48µs worth of pulses.
 *
 * @note FastAccelStepper pattern: PART_SIZE = RMT_SIZE >> 1
 */
#define LIMIT_RMT_PART_SIZE             (LIMIT_RMT_MEM_BLOCK_SYMBOLS >> 1)

/**
 * @brief Command queue depth (entries)
 *
 * Ring buffer holding motion commands. FastAccelStepper uses 32.
 * With 10ms forward planning at 500 kHz = ~20 commands needed.
 * 32 entries provides sufficient buffer with margin.
 *
 * @note Must be power of 2 for efficient modulo via bitmask.
 */
#define LIMIT_RMT_QUEUE_LEN             32

/**
 * @brief Queue length bitmask for efficient modulo
 */
#define LIMIT_RMT_QUEUE_LEN_MASK        (LIMIT_RMT_QUEUE_LEN - 1)

/**
 * @brief Minimum command period in RMT ticks
 *
 * FastAccelStepper: MIN_CMD_TICKS = TICKS_PER_S / 5000 = 3200 ticks.
 * This limits maximum step frequency to ~5 kHz per command.
 * Actual max frequency achieved by batching multiple steps per command.
 *
 * At 16 MHz: 3200 ticks = 200µs minimum command duration.
 */
#define LIMIT_RMT_MIN_CMD_TICKS         3200

/**
 * @brief Maximum steps per single command
 *
 * Limits how many steps can be batched in one queue entry.
 * At high frequencies, more steps per command = lower ISR rate.
 * At low frequencies, fewer steps per command = lower latency.
 *
 * 255 = uint8_t max (FastAccelStepper compatible)
 */
#define LIMIT_RMT_MAX_STEPS_PER_CMD     255

/**
 * @brief Forward planning time in RMT ticks
 *
 * How far ahead to fill the command queue.
 * Set to 10ms (160,000 ticks at 16MHz) to meet <10ms blend response latency.
 *
 * Trade-off: shorter planning = faster blend response but higher risk of
 * queue underrun at very high step rates. 10ms provides 5000 steps buffer
 * at 500kHz max frequency - sufficient margin.
 *
 * @note Reduced from FastAccelStepper's 20ms to meet PRD <10ms requirement.
 */
#define LIMIT_RMT_FORWARD_PLANNING_TICKS (LIMIT_RMT_RESOLUTION_HZ / 100)

/**
 * @brief Direction pin setup time (µs)
 *
 * Time between direction pin change and first step pulse.
 * Motor driver dependent. Typical: 5-20µs.
 */
#define LIMIT_RMT_DIR_SETUP_US          20

/**
 * @brief Minimum commands to keep in queue
 *
 * At low frequencies (250Hz), each command takes ~4ms.
 * Keep at least 10 commands queued to prevent underrun.
 */
#define LIMIT_RMT_MIN_QUEUE_CMDS        10

/**
 * @brief PCNT high limit for overflow detection
 *
 * ESP32-S3 PCNT is 16-bit signed (-32768 to +32767).
 * Watch points at ±32767 catch overflow before wrap-around.
 * ISR accumulates overflow count to extend to int64_t.
 */
#define LIMIT_PCNT_HIGH_LIMIT       32767

/**
 * @brief PCNT low limit for overflow detection
 *
 * Negative overflow watch point for reverse counting.
 */
#define LIMIT_PCNT_LOW_LIMIT        (-32767)

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
