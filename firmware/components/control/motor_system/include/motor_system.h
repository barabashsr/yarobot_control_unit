/**
 * @file motor_system.h
 * @brief Motor system initialization and management
 * @author YaRobot Team
 * @date 2025
 *
 * @note Central integration module that instantiates all motor control components:
 *       - Pulse generators (RMT, MCPWM, LEDC)
 *       - Position trackers (PCNT, Software, Time)
 *       - Motor objects (ServoMotor, StepperMotor, DiscreteAxis)
 *       - MotionController initialization
 *       - Command handler registration (MOVE, MOVR)
 *
 * @note All objects are statically allocated to avoid heap fragmentation.
 *       See Story 3-9b for architecture details.
 */

#ifndef MOTOR_SYSTEM_H
#define MOTOR_SYSTEM_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the complete motor subsystem
 *
 * Performs the following initialization sequence:
 * 1. Instantiate all pulse generators (RMT x4, MCPWM x2, LEDC x1)
 * 2. Instantiate all position trackers (Software x4, PCNT x3, Time x1)
 * 3. Create axis configurations with default values
 * 4. Instantiate motor objects (ServoMotor x5, StepperMotor x2, DiscreteAxis x1)
 * 5. Wire pulse generators to position trackers
 * 6. Initialize MotionController with motor array
 * 7. Register command handlers (MOVE, MOVR)
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if already initialized
 * @return ESP_ERR_NO_MEM if resource allocation fails
 * @return Other error codes from component initialization
 *
 * @note Must be called before any motion commands are processed
 * @note Thread-safe: uses internal initialization flag
 */
esp_err_t motor_system_init(void);

/**
 * @brief Check if motor system is initialized
 *
 * @return true if motor_system_init() completed successfully
 */
bool motor_system_is_initialized(void);

/**
 * @brief Real-time motion update for an axis
 *
 * Called from motion_task() for each axis. Handles:
 * - Pulse generator buffer refill (for streaming generators)
 * - Profile updates during motion
 * - Motion state monitoring
 *
 * @param axis_id Axis index (0 to LIMIT_NUM_AXES-1)
 *
 * @note Called at high frequency from Core 1 tasks
 * @note Safe to call before motor_system_init() (no-op if not initialized)
 */
void motor_system_update(uint8_t axis_id);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_SYSTEM_H
