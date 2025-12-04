/**
 * @file command_executor.h
 * @brief Command dispatcher and executor public API
 * @author YaRobot Team
 * @date 2025
 *
 * @note Dispatches parsed commands to appropriate handlers based on verb.
 *       Provides state-aware command routing with bitmask validation.
 *       Thread-safe: multiple tasks can call dispatch_command() safely.
 */

#ifndef COMMAND_EXECUTOR_H
#define COMMAND_EXECUTOR_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup command_executor Command Executor
 * @brief Route commands to handlers with state validation
 * @{
 */

/**
 * @defgroup system_state System State
 * @brief System operating state enumeration
 * @{
 */

/**
 * @brief System operating state
 *
 * Each state is a power of 2 to allow bitmask combinations.
 * Commands specify allowed_states as OR of valid states.
 */
typedef enum {
    /** @brief Initial state after boot - motors disabled */
    STATE_IDLE   = 0x01,

    /** @brief Normal operation - motion allowed */
    STATE_READY  = 0x02,

    /** @brief Configuration mode - motion blocked */
    STATE_CONFIG = 0x04,

    /** @brief Emergency stop active - only status/RST allowed */
    STATE_ESTOP  = 0x08,

    /** @brief Axis error state - requires clearing */
    STATE_ERROR  = 0x10,

    /** @brief Allowed in any state */
    STATE_ANY    = 0xFF
} SystemState;

/** @} */ // end system_state

/**
 * @defgroup command_types Command Handler Types
 * @brief Types for command handlers and registration
 * @{
 */

/**
 * @brief Command handler function pointer
 *
 * Handlers receive the parsed command and write response to provided buffer.
 * Handlers are stateless - all context passed as parameters.
 *
 * @param[in] cmd Pointer to parsed command structure
 * @param[out] response Buffer to write response string
 * @param[in] resp_len Size of response buffer
 *
 * @return ESP_OK on success
 * @return ESP_FAIL or specific error code on failure
 */
typedef esp_err_t (*CommandHandler)(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Command table entry
 *
 * Maps a verb to its handler and allowed states.
 */
typedef struct {
    /** @brief Command verb (CMD_* constant from config_commands.h) */
    const char* verb;

    /** @brief Handler function pointer */
    CommandHandler handler;

    /** @brief Bitmask of valid SystemState values */
    uint32_t allowed_states;
} CommandEntry;

/** @} */ // end command_types

/**
 * @defgroup executor_api Executor API
 * @brief Command executor initialization and dispatch
 * @{
 */

/**
 * @brief Initialize the command executor
 *
 * Initializes command table, mutex, and registers built-in commands.
 * Must be called once before using dispatch_command() or register.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if mutex creation fails
 */
esp_err_t cmd_executor_init(void);

/**
 * @brief Register a command handler
 *
 * Adds a command entry to the command table. Thread-safe.
 *
 * @param[in] entry Pointer to command entry (copied internally)
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if entry is NULL or verb is empty
 * @return ESP_ERR_NO_MEM if command table is full
 */
esp_err_t cmd_executor_register(const CommandEntry* entry);

/**
 * @brief Dispatch a parsed command to its handler
 *
 * Looks up the command verb in the table, validates state, and invokes handler.
 * Thread-safe: multiple tasks can call concurrently.
 *
 * @param[in] cmd Pointer to parsed command
 * @param[out] response Buffer to write response string
 * @param[in] resp_len Size of response buffer
 *
 * @return ESP_OK on success (handler executed successfully)
 * @return ESP_ERR_NOT_FOUND if verb not in command table
 * @return ESP_ERR_INVALID_STATE if current state not in allowed_states
 * @return ESP_FAIL if handler returns error
 *
 * @note On error, response buffer contains formatted error string.
 * @note Response buffer always contains valid response on return.
 */
esp_err_t dispatch_command(const ParsedCommand* cmd, char* response, size_t resp_len);

/** @} */ // end executor_api

/**
 * @defgroup state_api State Management API
 * @brief System state query and modification
 * @{
 */

/**
 * @brief Get current system state
 *
 * @return Current SystemState value
 */
SystemState get_system_state(void);

/**
 * @brief Set system state
 *
 * Atomically updates the system state.
 *
 * @param[in] new_state New state to set
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if new_state is not a valid single state
 */
esp_err_t set_system_state(SystemState new_state);

/**
 * @brief Check if current state matches allowed mask
 *
 * @param[in] allowed_mask Bitmask of allowed states
 *
 * @return true if (current_state & allowed_mask) != 0
 * @return false otherwise
 */
bool is_state_allowed(uint32_t allowed_mask);

/** @} */ // end state_api

/** @} */ // end command_executor

#ifdef __cplusplus
}
#endif

#endif // COMMAND_EXECUTOR_H
