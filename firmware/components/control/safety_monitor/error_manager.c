/**
 * @file error_manager.c
 * @brief Error Manager implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-10: Error Tracking & Recovery
 *       AC1: Error counts tracked per category (I2C, TIMEOUT, LIMIT, ESTOP, POSLOS)
 *       AC2: Per-axis error counts tracked
 *       AC3: ERROR events published with context
 *       AC4: CLRERR command clears error state
 *       AC7: ERRCNT CLR resets all counts
 *       AC8: STAT shows ERROR state
 */

#include "error_manager.h"
#include "config_commands.h"
#include "config_axes.h"
#include "event_manager.h"
#include "response_formatter.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <string.h>

static const char* TAG = "error_mgr";

/**
 * @brief Category name strings for ERRCNT response
 *
 * Order must match ErrorCategory enum
 */
const char* const ERROR_CATEGORY_NAMES[] = {
    "I2C",      // ERR_CAT_I2C
    "TIMEOUT",  // ERR_CAT_TIMEOUT
    "LIMIT",    // ERR_CAT_LIMIT
    "ESTOP",    // ERR_CAT_ESTOP
    "POSLOS"    // ERR_CAT_POSLOS
};

/**
 * @brief Axis name strings for logging and events
 */
static const char* const AXIS_NAMES[] = {"X", "Y", "Z", "A", "B", "C", "D", "E"};

/**
 * @brief Axis error state structure
 */
typedef struct {
    bool in_error;                      /**< Axis is in ERROR state */
    char error_code[ERROR_CODE_MAX_LEN]; /**< Current error code */
} AxisErrorState;

/**
 * @brief Error manager state
 */
static struct {
    bool initialized;                           /**< Initialization flag */
    SemaphoreHandle_t mutex;                    /**< Protects state access */

    // Global error counts per category
    uint32_t global_counts[ERR_CAT_COUNT];      /**< Cumulative counts per category */

    // Per-axis error counts
    uint32_t axis_counts[LIMIT_NUM_AXES][ERR_CAT_COUNT]; /**< Per-axis counts */

    // Axis ERROR states
    AxisErrorState axis_errors[LIMIT_NUM_AXES]; /**< Per-axis error state */

    // Test mode
    bool test_mode;                             /**< Test mode flag */

} s_errmgr = {
    .initialized = false,
    .mutex = NULL,
    .test_mode = false,
};

// ----------------------------------------------------------------------------
// Private Helper Functions
// ----------------------------------------------------------------------------

/**
 * @brief Take mutex with timeout
 */
static inline bool take_mutex(void)
{
    return xSemaphoreTake(s_errmgr.mutex, pdMS_TO_TICKS(100)) == pdTRUE;
}

/**
 * @brief Give mutex
 */
static inline void give_mutex(void)
{
    xSemaphoreGive(s_errmgr.mutex);
}

/**
 * @brief Check if axis index is valid
 */
static inline bool is_valid_axis(uint8_t axis)
{
    return axis < LIMIT_NUM_AXES;
}

/**
 * @brief Check if category is valid
 */
static inline bool is_valid_category(ErrorCategory category)
{
    return category < ERR_CAT_COUNT;
}

// ----------------------------------------------------------------------------
// Initialization API
// ----------------------------------------------------------------------------

esp_err_t error_manager_init(void)
{
    if (s_errmgr.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create mutex
    s_errmgr.mutex = xSemaphoreCreateMutex();
    if (s_errmgr.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Zero all global counters
    memset(s_errmgr.global_counts, 0, sizeof(s_errmgr.global_counts));

    // Zero all per-axis counters
    memset(s_errmgr.axis_counts, 0, sizeof(s_errmgr.axis_counts));

    // Clear all axis error states
    for (int i = 0; i < LIMIT_NUM_AXES; i++) {
        s_errmgr.axis_errors[i].in_error = false;
        s_errmgr.axis_errors[i].error_code[0] = '\0';
    }

    s_errmgr.test_mode = false;
    s_errmgr.initialized = true;

    ESP_LOGI(TAG, "Error manager initialized - all counters zeroed");

    return ESP_OK;
}

bool error_manager_is_initialized(void)
{
    return s_errmgr.initialized;
}

esp_err_t error_manager_deinit(void)
{
    if (!s_errmgr.initialized) {
        return ESP_OK;
    }

    // Delete mutex
    if (s_errmgr.mutex != NULL) {
        vSemaphoreDelete(s_errmgr.mutex);
        s_errmgr.mutex = NULL;
    }

    // Reset state
    memset(s_errmgr.global_counts, 0, sizeof(s_errmgr.global_counts));
    memset(s_errmgr.axis_counts, 0, sizeof(s_errmgr.axis_counts));
    s_errmgr.test_mode = false;

    s_errmgr.initialized = false;

    ESP_LOGI(TAG, "Error manager deinitialized");
    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Error Count Functions
// ----------------------------------------------------------------------------

esp_err_t error_manager_increment(ErrorCategory category, uint8_t axis, bool publish_event)
{
    if (!s_errmgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!is_valid_category(category)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    // Increment global counter
    s_errmgr.global_counts[category]++;

    // Increment per-axis counter if axis specified
    if (is_valid_axis(axis)) {
        s_errmgr.axis_counts[axis][category]++;
        ESP_LOGW(TAG, "Error %s on axis %s (count: %lu)",
                 ERROR_CATEGORY_NAMES[category],
                 AXIS_NAMES[axis],
                 (unsigned long)s_errmgr.axis_counts[axis][category]);
    } else {
        ESP_LOGW(TAG, "System error %s (count: %lu)",
                 ERROR_CATEGORY_NAMES[category],
                 (unsigned long)s_errmgr.global_counts[category]);
    }

    give_mutex();

    // Publish event if requested
    if (publish_event) {
        const char* error_code = NULL;
        switch (category) {
            case ERR_CAT_I2C:
                error_code = ERR_I2C_FAILURE;
                break;
            case ERR_CAT_LIMIT:
                error_code = ERR_LIMIT_ACTIVE;
                break;
            case ERR_CAT_ESTOP:
                error_code = ERR_ESTOP_ACTIVE;
                break;
            case ERR_CAT_POSLOS:
                error_code = ERR_POSLOS;
                break;
            case ERR_CAT_TIMEOUT:
                error_code = ERR_MOTOR_FAULT;  // Use motor fault for timeout
                break;
            default:
                error_code = ERR_SYSTEM_ERROR;
                break;
        }
        error_manager_publish_error(category, axis, error_code);
    }

    return ESP_OK;
}

esp_err_t error_manager_get_counts(uint32_t* counts)
{
    if (counts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_errmgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(counts, s_errmgr.global_counts, sizeof(s_errmgr.global_counts));

    give_mutex();

    return ESP_OK;
}

esp_err_t error_manager_get_axis_counts(uint8_t axis, uint32_t* counts)
{
    if (counts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_valid_axis(axis)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_errmgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(counts, s_errmgr.axis_counts[axis], sizeof(s_errmgr.axis_counts[axis]));

    give_mutex();

    return ESP_OK;
}

esp_err_t error_manager_clear_counts(void)
{
    if (!s_errmgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    // Clear all global counters
    memset(s_errmgr.global_counts, 0, sizeof(s_errmgr.global_counts));

    // Clear all per-axis counters
    memset(s_errmgr.axis_counts, 0, sizeof(s_errmgr.axis_counts));

    give_mutex();

    ESP_LOGI(TAG, "All error counters cleared");

    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Axis Error State Functions
// ----------------------------------------------------------------------------

esp_err_t error_manager_set_axis_error(uint8_t axis, const char* error_code)
{
    if (!s_errmgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!is_valid_axis(axis)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (error_code == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    s_errmgr.axis_errors[axis].in_error = true;
    strncpy(s_errmgr.axis_errors[axis].error_code, error_code, ERROR_CODE_MAX_LEN - 1);
    s_errmgr.axis_errors[axis].error_code[ERROR_CODE_MAX_LEN - 1] = '\0';

    give_mutex();

    ESP_LOGW(TAG, "Axis %s set to ERROR state: %s", AXIS_NAMES[axis], error_code);

    return ESP_OK;
}

esp_err_t error_manager_clear_axis_error(uint8_t axis)
{
    if (!s_errmgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!is_valid_axis(axis)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    bool was_in_error = s_errmgr.axis_errors[axis].in_error;
    s_errmgr.axis_errors[axis].in_error = false;
    s_errmgr.axis_errors[axis].error_code[0] = '\0';

    give_mutex();

    if (was_in_error) {
        ESP_LOGI(TAG, "Axis %s ERROR state cleared", AXIS_NAMES[axis]);
    }

    return ESP_OK;
}

esp_err_t error_manager_clear_all_axis_errors(void)
{
    if (!s_errmgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    for (int i = 0; i < LIMIT_NUM_AXES; i++) {
        s_errmgr.axis_errors[i].in_error = false;
        s_errmgr.axis_errors[i].error_code[0] = '\0';
    }

    give_mutex();

    ESP_LOGI(TAG, "All axis ERROR states cleared");

    return ESP_OK;
}

bool error_manager_is_axis_in_error(uint8_t axis)
{
    if (!is_valid_axis(axis)) {
        return false;
    }

    if (!s_errmgr.initialized) {
        return false;  // Fail-safe: not in error if not initialized
    }

    // Atomic read, no mutex needed
    return s_errmgr.axis_errors[axis].in_error;
}

esp_err_t error_manager_get_axis_error_code(uint8_t axis, char* error_code, size_t buf_len)
{
    if (error_code == NULL || buf_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_valid_axis(axis)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_errmgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_errmgr.axis_errors[axis].in_error) {
        give_mutex();
        return ESP_ERR_NOT_FOUND;
    }

    strncpy(error_code, s_errmgr.axis_errors[axis].error_code, buf_len - 1);
    error_code[buf_len - 1] = '\0';

    give_mutex();

    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Error Event Functions
// ----------------------------------------------------------------------------

esp_err_t error_manager_publish_error(ErrorCategory category, uint8_t axis, const char* error_code)
{
    if (error_code == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    Event evt = {
        .type = EVTTYPE_ERROR,
        .axis = axis,  // 0xFF for SYSTEM
        .data.error_code = 0,  // Not used directly, we format manually
        .timestamp = esp_timer_get_time()
    };

    esp_err_t err = event_publish(&evt);
    if (err != ESP_OK) {
        if (axis < LIMIT_NUM_AXES) {
            ESP_LOGW(TAG, "Failed to publish ERROR event for axis %s: %s",
                     AXIS_NAMES[axis], esp_err_to_name(err));
        } else {
            ESP_LOGW(TAG, "Failed to publish ERROR event for SYSTEM: %s",
                     esp_err_to_name(err));
        }
        return err;
    }

    if (axis < LIMIT_NUM_AXES) {
        ESP_LOGI(TAG, "Published EVENT ERROR %s %s", AXIS_NAMES[axis], error_code);
    } else {
        ESP_LOGI(TAG, "Published EVENT ERROR SYSTEM %s", error_code);
    }

    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Test/Debug Functions
// ----------------------------------------------------------------------------

void error_manager_inject_test_count(ErrorCategory category, uint32_t count)
{
    if (!is_valid_category(category)) {
        return;
    }

    s_errmgr.test_mode = true;
    s_errmgr.global_counts[category] = count;
    ESP_LOGI(TAG, "TEST: %s count set to %lu", ERROR_CATEGORY_NAMES[category], (unsigned long)count);
}

void error_manager_inject_test_error(uint8_t axis, bool in_error, const char* error_code)
{
    if (!is_valid_axis(axis)) {
        return;
    }

    s_errmgr.test_mode = true;
    s_errmgr.axis_errors[axis].in_error = in_error;
    if (in_error && error_code != NULL) {
        strncpy(s_errmgr.axis_errors[axis].error_code, error_code, ERROR_CODE_MAX_LEN - 1);
        s_errmgr.axis_errors[axis].error_code[ERROR_CODE_MAX_LEN - 1] = '\0';
    } else {
        s_errmgr.axis_errors[axis].error_code[0] = '\0';
    }

    ESP_LOGI(TAG, "TEST: Axis %s ERROR set to %s", AXIS_NAMES[axis], in_error ? "true" : "false");
}
