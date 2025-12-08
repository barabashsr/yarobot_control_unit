/**
 * @file command_executor.c
 * @brief Command dispatcher and executor implementation
 * @author YaRobot Team
 * @date 2025
 */

#include "command_executor.h"
#include "command_parser.h"      // For is_valid_axis()
#include "response_formatter.h"
#include "event_manager.h"
#include "config.h"              // For FIRMWARE_NAME, FIRMWARE_VERSION_STRING
#include "config_commands.h"
#include "config_limits.h"

#include <string.h>
#include <strings.h>  // For strcasecmp
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"  // For esp_timer_get_time()

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

    // Initialize event manager (Story 2-7)
    // Event manager initializes its own task and publishes EVT_BOOT
    esp_err_t event_ret = event_manager_init();
    if (event_ret != ESP_OK) {
        ESP_LOGW(TAG, "Event manager init failed: 0x%x (non-fatal)", event_ret);
        // Non-fatal: system can operate without events
    }

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
    if (cmd->param_count > 0) {
        ESP_LOGI(TAG, "CMD: %s %c %.4f",
                 cmd->verb,
                 cmd->axis ? cmd->axis : '-',
                 cmd->params[0]);
    } else {
        ESP_LOGI(TAG, "CMD: %s %c",
                 cmd->verb,
                 cmd->axis ? cmd->axis : '-');
    }

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
 * Returns system information: firmware name and version.
 * Response format: "OK YAROBOT_CONTROL_UNIT 1.0.0"
 *
 * @note Uses FIRMWARE_NAME and FIRMWARE_VERSION_STRING from config.h
 *       per AC3, AC10.
 */
static esp_err_t handle_info(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    (void)cmd;  // No parameters used
    return format_ok_data(response, resp_len, "%s %s",
                          FIRMWARE_NAME, FIRMWARE_VERSION_STRING);
}

/**
 * @brief Handle STAT command
 *
 * Returns system status or axis-specific status.
 *
 * System status format (AC4, AC7):
 *   "OK MODE:<mode> ESTOP:<0|1> AXES:<n> UPTIME:<ms>"
 *
 * Axis status format (AC5, AC6, AC8):
 *   "OK <axis> POS:<pos> EN:<0|1> MOV:<0|1> ERR:<0|1> LIM:<hex>"
 *
 * @note ESTOP and axis status return placeholder values until
 *       Epic 3 (motor control) and Epic 4 (safety systems).
 */
static esp_err_t handle_stat(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // If axis specified, return axis status
    if (cmd->axis != '\0') {
        // Validate axis is valid (X-E)
        if (!is_valid_axis(cmd->axis)) {
            return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
        }

        // Placeholder values until motor control implemented (AC8)
        float pos = 0.0f;
        int en = 0;      // not enabled
        int mov = 0;     // not moving
        int err = 0;     // no error
        int lim = 0x00;  // no limits active (bit0=min, bit1=max)

        return format_ok_data(response, resp_len,
                              "%c POS:%.3f EN:%d MOV:%d ERR:%d LIM:%02X",
                              cmd->axis, pos, en, mov, err, lim);
    }

    // System status
    const char* mode_str = "IDLE";
    switch (s_current_state) {
        case STATE_IDLE:   mode_str = "IDLE";    break;
        case STATE_READY:  mode_str = "READY";   break;
        case STATE_CONFIG: mode_str = "CONFIG";  break;
        case STATE_ESTOP:  mode_str = "ESTOP";   break;
        case STATE_ERROR:  mode_str = "ERROR";   break;
        default:           mode_str = "UNKNOWN"; break;
    }

    // E-stop placeholder until Epic 4
    int estop = 0;

    // Uptime in milliseconds from boot (AC7)
    int64_t uptime_ms = esp_timer_get_time() / 1000;

    return format_ok_data(response, resp_len,
                          "MODE:%s ESTOP:%d AXES:%d UPTIME:%lld",
                          mode_str, estop, LIMIT_NUM_AXES, uptime_ms);
}

/**
 * @brief Convert SystemState to mode name string
 *
 * @param[in] state System state to convert
 * @return Mode name string (MODE_NAME_* constant)
 */
static const char* state_to_mode_string(SystemState state)
{
    switch (state) {
        case STATE_IDLE:   return MODE_NAME_IDLE;
        case STATE_READY:  return MODE_NAME_READY;
        case STATE_CONFIG: return MODE_NAME_CONFIG;
        case STATE_ESTOP:  return MODE_NAME_ESTOP;
        case STATE_ERROR:  return MODE_NAME_ERROR;
        default:           return "UNKNOWN";
    }
}

/**
 * @brief Publish mode change event
 *
 * Creates and publishes a MODE_CHANGED event via the event manager.
 * The event is delivered to all subscribers including USB output.
 *
 * @param[in] new_state The new system state
 */
static void publish_mode_event(SystemState new_state)
{
    Event event = {
        .type = EVTTYPE_MODE_CHANGED,
        .axis = 0xFF,  // System-wide event
        .data.mode_name = state_to_mode_string(new_state),
        .timestamp = esp_timer_get_time()
    };
    event_publish(&event);
    ESP_LOGD(TAG, "Published MODE event: %s", state_to_mode_string(new_state));
}

/**
 * @brief Handle MODE command
 *
 * Query or set operating mode.
 *
 * Query (no args):
 *   Returns "OK <current_mode>" (AC1)
 *
 * Set (with mode name):
 *   - MODE READY from IDLE: OK READY (AC2)
 *   - MODE CONFIG from READY: OK CONFIG (AC3)
 *   - MODE READY from CONFIG: OK READY (AC4)
 *   - MODE READY from ESTOP: ERROR E006 (AC6)
 *   - MODE READY from ERROR: ERROR E031 (AC10)
 *   - Invalid mode name: ERROR E001 (AC9)
 *
 * Events:
 *   - Successful mode change publishes EVENT MODE <new_mode> (AC7)
 */
static esp_err_t handle_mode(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Query mode if no parameter (AC1)
    if (!cmd->has_str_param && cmd->param_count == 0) {
        return format_ok_data(response, resp_len, "%s", state_to_mode_string(s_current_state));
    }

    // Set mode based on string parameter
    if (cmd->has_str_param) {
        SystemState new_state = STATE_IDLE;
        SystemState current = s_current_state;

        // Parse target mode (AC9: invalid mode returns E001 Unknown command)
        if (strcasecmp(cmd->str_param, MODE_NAME_IDLE) == 0) {
            new_state = STATE_IDLE;
        } else if (strcasecmp(cmd->str_param, MODE_NAME_READY) == 0) {
            new_state = STATE_READY;
        } else if (strcasecmp(cmd->str_param, MODE_NAME_CONFIG) == 0) {
            new_state = STATE_CONFIG;
        } else {
            // AC9: Unknown mode name returns E001 (treated as unknown command)
            format_error(response, resp_len, ERR_INVALID_COMMAND, MSG_INVALID_COMMAND);
            return ESP_ERR_NOT_FOUND;
        }

        // AC6: ESTOP mode blocks all mode transitions
        if (current == STATE_ESTOP) {
            format_error(response, resp_len, ERR_EMERGENCY_STOP, MSG_EMERGENCY_STOP);
            return ESP_ERR_INVALID_STATE;
        }

        // AC10: ERROR mode blocks all mode transitions (requires RST)
        if (current == STATE_ERROR) {
            format_error(response, resp_len, ERR_SYSTEM_ERROR, MSG_SYSTEM_ERROR);
            return ESP_ERR_INVALID_STATE;
        }

        // Validate transition is allowed
        // Valid transitions:
        // - IDLE -> READY (AC2)
        // - READY -> CONFIG (AC3)
        // - CONFIG -> READY (AC4)
        // - Any state -> same state (no-op, still OK)
        bool valid_transition = false;
        if (current == new_state) {
            // Same state - always valid (no actual change)
            valid_transition = true;
        } else if (current == STATE_IDLE && new_state == STATE_READY) {
            valid_transition = true;  // AC2
        } else if (current == STATE_READY && new_state == STATE_CONFIG) {
            valid_transition = true;  // AC3
        } else if (current == STATE_CONFIG && new_state == STATE_READY) {
            valid_transition = true;  // AC4
        } else if (current == STATE_READY && new_state == STATE_IDLE) {
            valid_transition = true;  // Allow READY -> IDLE
        } else if (current == STATE_CONFIG && new_state == STATE_IDLE) {
            valid_transition = true;  // Allow CONFIG -> IDLE
        }

        if (!valid_transition) {
            format_error(response, resp_len, ERR_INVALID_PARAMETER, MSG_INVALID_PARAMETER);
            return ESP_ERR_INVALID_ARG;
        }

        // Execute state transition (AC8: atomic via set_system_state)
        esp_err_t ret = set_system_state(new_state);
        if (ret != ESP_OK) {
            format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
            return ret;
        }

        // AC7: Publish event if state actually changed
        if (current != new_state) {
            publish_mode_event(new_state);
        }

        return format_ok_data(response, resp_len, "%s", state_to_mode_string(new_state));
    }

    // No valid parameter
    format_error(response, resp_len, ERR_INVALID_PARAMETER, MSG_INVALID_PARAMETER);
    return ESP_ERR_INVALID_ARG;
}
