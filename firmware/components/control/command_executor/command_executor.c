/**
 * @file command_executor.c
 * @brief Command dispatcher and executor implementation
 * @author YaRobot Team
 * @date 2025
 */

#include "command_executor.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_limits.h"

#include <string.h>
#include <strings.h>  // For strcasecmp
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char* TAG = "CMD_EXEC";

/* ==========================================================================
 * Configuration Constants
 * ========================================================================== */

/** @brief Maximum number of registered commands */
#define MAX_COMMANDS    32

/* ==========================================================================
 * Static Variables
 * ========================================================================== */

/** @brief Current system state - volatile for atomic reads */
static volatile SystemState s_current_state = STATE_IDLE;

/** @brief Command table */
static CommandEntry s_command_table[MAX_COMMANDS];

/** @brief Number of registered commands */
static size_t s_command_count = 0;

/** @brief Mutex for command table registration */
static SemaphoreHandle_t s_table_mutex = NULL;

/** @brief Initialization flag */
static bool s_initialized = false;

/* ==========================================================================
 * Forward Declarations - Stub Handlers
 * ========================================================================== */

static esp_err_t handle_echo(const ParsedCommand* cmd, char* response, size_t resp_len);
static esp_err_t handle_info(const ParsedCommand* cmd, char* response, size_t resp_len);
static esp_err_t handle_stat(const ParsedCommand* cmd, char* response, size_t resp_len);
static esp_err_t handle_mode(const ParsedCommand* cmd, char* response, size_t resp_len);

/* ==========================================================================
 * Built-in Command Table
 * ========================================================================== */

/** @brief Built-in commands registered at init */
static const CommandEntry s_builtin_commands[] = {
    { CMD_ECHO, handle_echo, STATE_ANY },
    { CMD_INFO, handle_info, STATE_ANY },
    { CMD_STAT, handle_stat, STATE_ANY },
    { CMD_MODE, handle_mode, STATE_ANY },
};

#define BUILTIN_COUNT (sizeof(s_builtin_commands) / sizeof(s_builtin_commands[0]))

/* ==========================================================================
 * State Management Implementation
 * ========================================================================== */

SystemState get_system_state(void)
{
    return s_current_state;
}

esp_err_t set_system_state(SystemState new_state)
{
    // Validate that new_state is a single valid state (power of 2) or STATE_ANY
    if (new_state == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if it's a single bit or STATE_ANY
    if (new_state != STATE_ANY && (new_state & (new_state - 1)) != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Atomic write (single word on ESP32)
    s_current_state = new_state;
    ESP_LOGD(TAG, "State changed to 0x%02X", new_state);

    return ESP_OK;
}

bool is_state_allowed(uint32_t allowed_mask)
{
    return (s_current_state & allowed_mask) != 0;
}

/* ==========================================================================
 * Command Table Management
 * ========================================================================== */

/**
 * @brief Find command entry by verb
 *
 * @param[in] verb Command verb to search for
 *
 * @return Pointer to CommandEntry if found, NULL otherwise
 */
static const CommandEntry* find_command(const char* verb)
{
    for (size_t i = 0; i < s_command_count; i++) {
        if (strcasecmp(verb, s_command_table[i].verb) == 0) {
            return &s_command_table[i];
        }
    }
    return NULL;
}

esp_err_t cmd_executor_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Create mutex for table access
    s_table_mutex = xSemaphoreCreateMutex();
    if (s_table_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize command table with built-in commands
    s_command_count = 0;
    for (size_t i = 0; i < BUILTIN_COUNT; i++) {
        if (s_command_count >= MAX_COMMANDS) {
            ESP_LOGE(TAG, "Command table full during init");
            break;
        }
        s_command_table[s_command_count] = s_builtin_commands[i];
        s_command_count++;
    }

    // Set initial state
    s_current_state = STATE_IDLE;

    s_initialized = true;
    ESP_LOGI(TAG, "Initialized with %d built-in commands", (int)BUILTIN_COUNT);

    return ESP_OK;
}

esp_err_t cmd_executor_register(const CommandEntry* entry)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (entry == NULL || entry->verb == NULL || entry->verb[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    if (entry->handler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    if (xSemaphoreTake(s_table_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Check if already registered
    if (find_command(entry->verb) != NULL) {
        ESP_LOGW(TAG, "Command '%s' already registered", entry->verb);
        ret = ESP_ERR_INVALID_STATE;
        goto done;
    }

    // Check capacity
    if (s_command_count >= MAX_COMMANDS) {
        ESP_LOGE(TAG, "Command table full");
        ret = ESP_ERR_NO_MEM;
        goto done;
    }

    // Add entry
    s_command_table[s_command_count] = *entry;
    s_command_count++;
    ESP_LOGD(TAG, "Registered command '%s'", entry->verb);

done:
    xSemaphoreGive(s_table_mutex);
    return ret;
}

/* ==========================================================================
 * Command Dispatch
 * ========================================================================== */

esp_err_t dispatch_command(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
        return ESP_ERR_INVALID_STATE;
    }

    if (cmd == NULL || response == NULL || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Skip empty commands
    if (cmd->verb[0] == '\0') {
        format_ok(response, resp_len);
        return ESP_OK;
    }

    // Log command (FR26: command history for debugging)
    ESP_LOGD(TAG, "verb=%s axis=%c params=%d",
             cmd->verb,
             cmd->axis ? cmd->axis : '-',
             cmd->param_count);

    // Find command in table
    const CommandEntry* entry = find_command(cmd->verb);
    if (entry == NULL) {
        ESP_LOGD(TAG, "Unknown command: %s", cmd->verb);
        format_error(response, resp_len, ERR_INVALID_COMMAND, MSG_INVALID_COMMAND);
        return ESP_ERR_NOT_FOUND;
    }

    // Check state permission
    if (!is_state_allowed(entry->allowed_states)) {
        ESP_LOGD(TAG, "Command blocked in state 0x%02X (allowed: 0x%02X)",
                 s_current_state, (unsigned)entry->allowed_states);
        format_error(response, resp_len, ERR_MODE_BLOCKED, MSG_MODE_BLOCKED);
        return ESP_ERR_INVALID_STATE;
    }

    // Invoke handler
    esp_err_t result = entry->handler(cmd, response, resp_len);
    if (result != ESP_OK) {
        ESP_LOGD(TAG, "Handler returned error: 0x%x", result);
        // Response buffer should already contain error from handler
        // If not, provide a generic error
        if (response[0] == '\0') {
            format_error(response, resp_len, ERR_COMMUNICATION, MSG_COMMUNICATION);
        }
    }

    return result;
}

/* ==========================================================================
 * Stub Command Handlers
 * ========================================================================== */

/**
 * @brief Handle ECHO command
 *
 * Returns "OK [input text]" for communication testing.
 */
static esp_err_t handle_echo(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    if (cmd->has_str_param) {
        return format_ok_data(response, resp_len, "%s", cmd->str_param);
    }
    return format_ok(response, resp_len);
}

/**
 * @brief Handle INFO command
 *
 * Returns system information: name, version.
 * Stub implementation - full version info in Story 2-5.
 */
static esp_err_t handle_info(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    (void)cmd;  // Unused in stub
    return format_ok_data(response, resp_len, "YAROBOT_CONTROL_UNIT 1.0.0 AXES:8");
}

/**
 * @brief Handle STAT command
 *
 * Returns system/axis status.
 * Stub implementation - full status in Story 2-5.
 */
static esp_err_t handle_stat(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // If axis specified, return axis status
    if (cmd->axis != '\0') {
        return format_ok_data(response, resp_len, "%c IDLE POS:0.000", cmd->axis);
    }

    // Otherwise return system status
    const char* state_str = "IDLE";
    switch (s_current_state) {
        case STATE_IDLE:   state_str = "IDLE";   break;
        case STATE_READY:  state_str = "READY";  break;
        case STATE_CONFIG: state_str = "CONFIG"; break;
        case STATE_ESTOP:  state_str = "ESTOP";  break;
        case STATE_ERROR:  state_str = "ERROR";  break;
        default:           state_str = "UNKNOWN"; break;
    }

    return format_ok_data(response, resp_len, "STATE:%s", state_str);
}

/**
 * @brief Handle MODE command
 *
 * Query or set operating mode.
 * Stub implementation - full mode transitions in Story 2-6.
 */
static esp_err_t handle_mode(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Query mode if no parameter
    if (!cmd->has_str_param && cmd->param_count == 0) {
        const char* mode_str = "IDLE";
        switch (s_current_state) {
            case STATE_IDLE:   mode_str = "IDLE";   break;
            case STATE_READY:  mode_str = "READY";  break;
            case STATE_CONFIG: mode_str = "CONFIG"; break;
            case STATE_ESTOP:  mode_str = "ESTOP";  break;
            case STATE_ERROR:  mode_str = "ERROR";  break;
            default:           mode_str = "UNKNOWN"; break;
        }
        return format_ok_data(response, resp_len, "%s", mode_str);
    }

    // Set mode based on string parameter
    if (cmd->has_str_param) {
        SystemState new_state = STATE_IDLE;

        if (strcasecmp(cmd->str_param, "IDLE") == 0) {
            new_state = STATE_IDLE;
        } else if (strcasecmp(cmd->str_param, "READY") == 0) {
            new_state = STATE_READY;
        } else if (strcasecmp(cmd->str_param, "CONFIG") == 0) {
            new_state = STATE_CONFIG;
        } else {
            format_error(response, resp_len, ERR_INVALID_PARAMETER, MSG_INVALID_PARAMETER);
            return ESP_ERR_INVALID_ARG;
        }

        esp_err_t ret = set_system_state(new_state);
        if (ret != ESP_OK) {
            format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
            return ret;
        }

        return format_ok_data(response, resp_len, "%s", cmd->str_param);
    }

    // No valid parameter
    format_error(response, resp_len, ERR_INVALID_PARAMETER, MSG_INVALID_PARAMETER);
    return ESP_ERR_INVALID_ARG;
}
