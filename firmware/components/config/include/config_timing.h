/**
 * @file config_timing.h
 * @brief Timing constants for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note All timing values are in milliseconds (ms) or microseconds (µs) as noted.
 *       Values are chosen for reliable operation with servo/stepper motor drivers.
 */

#ifndef CONFIG_TIMING_H
#define CONFIG_TIMING_H

/**
 * @defgroup config_timing Timing Constants
 * @brief All timing-related configuration values
 * @{
 */

/**
 * @defgroup timing_motion Motion Control Timing
 * @brief Timing for motor direction and enable signals
 * @{
 */

/** @brief Direction signal setup time before step pulse (µs) */
#define TIMING_DIR_SETUP_US         20

/** @brief Enable signal delay after asserting (µs) */
#define TIMING_ENABLE_DELAY_US      50

/** @brief Time to engage brake after motion stop (ms) */
#define TIMING_BRAKE_ENGAGE_MS      50

/** @brief Time to release brake before motion start (ms) */
#define TIMING_BRAKE_RELEASE_MS     30

/**
 * @brief E-axis discrete actuator travel time (ms)
 *
 * Time for E-axis pneumatic cylinder or solenoid to complete
 * full travel from retracted (0) to extended (1) position.
 * Used by TimeTracker for position interpolation.
 */
#define TIMING_E_AXIS_TRAVEL_MS     1000

/** @} */ // end timing_motion

/**
 * @defgroup timing_comm Communication Timing
 * @brief USB and command processing timeouts
 * @{
 */

/** @brief Maximum command response time (ms) */
#define TIMING_CMD_RESPONSE_MS      10

/** @brief USB receive timeout (ms) */
#define TIMING_USB_RX_TIMEOUT_MS    100

/** @} */ // end timing_comm

/**
 * @defgroup timing_i2c I2C Bus Timing
 * @brief I2C communication timeouts and polling intervals
 * @{
 */

/** @brief I2C polling interval for input changes (ms) */
#define TIMING_I2C_POLL_MS          5

/** @brief I2C transaction timeout (ms) */
#define TIMING_I2C_TIMEOUT_MS       50

/** @brief Number of I2C retries on failure */
#define TIMING_I2C_RETRY_COUNT      3

/** @} */ // end timing_i2c

/**
 * @defgroup timing_safety Safety System Timing
 * @brief Debounce and safety-related timing
 * @{
 */

/** @brief E-stop switch debounce time (ms) */
#define TIMING_ESTOP_DEBOUNCE_MS    5

/** @brief Limit switch debounce time (ms) */
#define TIMING_LIMIT_DEBOUNCE_MS    10

/** @brief Idle timeout before brake engage (seconds) */
#define TIMING_IDLE_TIMEOUT_S       300

/** @brief Safety monitoring poll interval (ms) */
#define TIMING_SAFETY_POLL_MS       10

/** @} */ // end timing_safety

/**
 * @defgroup timing_position Position Tracking Timing
 * @brief Position update intervals for software-counted axes
 * @{
 */

/**
 * @brief LEDC (D-axis) position update interval (ms)
 *
 * How often the LEDC pulse generator updates the SoftwareTracker
 * with the current pulse count during motion. Lower values give
 * more responsive position queries but increase timer overhead.
 *
 * RMT axes update on DMA buffer completion (hardware-driven).
 * PCNT axes count in hardware (always real-time).
 */
#define TIMING_LEDC_POSITION_UPDATE_MS  20

/** @} */ // end timing_position

/**
 * @defgroup timing_periods Task Periods
 * @brief FreeRTOS task execution periods
 * @{
 */

/** @brief I2C monitor task period (ms) */
#define PERIOD_I2C_MONITOR_MS       100

/** @brief OLED display update period (ms) */
#define PERIOD_DISPLAY_UPDATE_MS    100

/** @brief Idle timeout check period (ms) */
#define PERIOD_IDLE_CHECK_MS        1000

/**
 * @brief Motion task update period (ms)
 *
 * How often each motion_task calls motor_system_update() for monitoring.
 * Actual pulse streaming is handled by pulse generator internal tasks.
 * This is for state monitoring and diagnostics only.
 */
#define PERIOD_MOTION_UPDATE_MS     10

/**
 * @brief RMT ramp generator task period (ms)
 *
 * How often the ramp generator refills the command queue.
 * FastAccelStepper default: 4ms. Must be < forward planning time.
 *
 * @note Event-driven with fallback periodic wake-up.
 */
#define TIMING_RMT_RAMP_TASK_PERIOD_MS  4

/**
 * @brief RMT ramp generator task stack size (bytes)
 *
 * FPU operations in ramp generator require larger stack.
 */
#define TIMING_RMT_RAMP_TASK_STACK      4096

/**
 * @brief Position update latency target (ms)
 *
 * Maximum time between position updates during motion.
 * PRD requirement: <10ms for real-time position feedback.
 * Used to calculate steps per command.
 */
#define TIMING_POSITION_UPDATE_LATENCY_MS  10

/**
 * @brief RMT stop latency maximum (µs)
 *
 * Maximum time for stopImmediate() to halt pulses.
 * With queue-based system: time to drain current command.
 * Worst case: PART_SIZE symbols at min frequency.
 */
#define TIMING_RMT_STOP_LATENCY_US      100

/** @} */ // end timing_periods

/** @} */ // end config_timing

#endif // CONFIG_TIMING_H
