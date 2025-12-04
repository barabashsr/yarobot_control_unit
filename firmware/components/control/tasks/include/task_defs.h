/**
 * @file task_defs.h
 * @brief FreeRTOS task function declarations
 * @author YaRobot Team
 * @date 2025
 *
 * @note Declares all FreeRTOS task entry points for the control system.
 *       Core 0: safety_monitor, usb_rx, usb_tx, cmd_executor, i2c_monitor, idle_monitor
 *       Core 1: motion (x8), display
 */

#ifndef TASK_DEFS_H
#define TASK_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup tasks FreeRTOS Task Functions
 * @brief Task entry points for the control system
 * @{
 */

/**
 * @brief Safety monitor task - highest priority on Core 0
 *
 * Monitors E-stop, fault detection, and safety interlocks.
 * Priority: 24 (highest), Core: 0
 *
 * @param arg Unused (NULL)
 */
void safety_monitor_task(void* arg);

/**
 * @brief USB receive task - handles incoming USB CDC data
 *
 * Priority: 10, Core: 0
 *
 * @param arg Unused (NULL)
 */
void usb_rx_task(void* arg);

/**
 * @brief USB transmit task - sends outgoing USB CDC data
 *
 * Priority: 10, Core: 0
 *
 * @param arg Unused (NULL)
 */
void usb_tx_task(void* arg);

/**
 * @brief Command executor task - processes parsed commands
 *
 * Priority: 12, Core: 0
 *
 * @param arg Unused (NULL)
 */
void cmd_executor_task(void* arg);

/**
 * @brief I2C monitor task - polls MCP23017 expanders
 *
 * Priority: 8, Core: 0
 *
 * @param arg Unused (NULL)
 */
void i2c_monitor_task(void* arg);

/**
 * @brief Idle monitor task - diagnostics and logging
 *
 * Priority: 4 (lowest), Core: 0
 *
 * @param arg Unused (NULL)
 */
void idle_monitor_task(void* arg);

/**
 * @brief Motion control task - real-time pulse streaming
 *
 * One instance per axis (8 total). Priority: 15, Core: 1
 *
 * @param arg Axis index (0-7, cast from intptr_t)
 */
void motion_task(void* arg);

/**
 * @brief Display task - OLED updates
 *
 * Priority: 5, Core: 1
 *
 * @param arg Unused (NULL)
 */
void display_task(void* arg);

/** @} */ // end tasks

#ifdef __cplusplus
}
#endif

#endif // TASK_DEFS_H
