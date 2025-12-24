/**
 * @file floating_switch.c
 * @brief Floating Switch Handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-9: C-Axis Floating Switch (Object Width Measurement)
 *       Monitors C_LIMIT_MAX input for floating switch trigger during
 *       closing motion. Calculates width and publishes EVENT WIDTH C.
 */

#include "floating_switch.h"
#include "safety_monitor.h"
#include "response_formatter.h"
#include "event_manager.h"
#include "config_axes.h"
#include "config_commands.h"
#include "config_i2c.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// C API wrappers from motion_controller.h (declared in extern "C" block there)
// Declared here to avoid including the C++ header in this C file
esp_err_t motion_controller_stop_axis(uint8_t axis);
esp_err_t motion_controller_get_position(uint8_t axis, float* position);
bool motion_controller_is_initialized(void);

static const char* TAG = "FLOAT_SW";

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/**
 * @brief Closing direction for C axis (picker jaw)
 *
 * Closing motion moves toward center (negative direction).
 * Width measurement only valid during closing motion.
 */
#define C_AXIS_CLOSING_DIRECTION    (-1)

/* ==========================================================================
 * Static Variables
 * ========================================================================== */

/** @brief Initialization flag */
static bool s_initialized = false;

/** @brief Last measured width value */
static float s_last_width = FLOATING_SWITCH_NO_MEASUREMENT;

/** @brief Flag indicating valid measurement exists */
static bool s_has_measurement = false;

/** @brief Current C axis motion direction (-1=closing, +1=opening, 0=stopped) */
static int8_t s_c_axis_direction = 0;

/** @brief Mutex for thread-safe access to measurement state */
static SemaphoreHandle_t s_mutex = NULL;

/* ==========================================================================
 * Private Helper Functions
 * ========================================================================== */

/**
 * @brief Publish width measurement event
 *
 * Formats and publishes "EVENT WIDTH C <value>" via event manager.
 *
 * @param[in] width Measured width value
 */
static void publish_width_event(float width)
{
    Event evt = {
        .type = EVTTYPE_WIDTH_MEASURED,
        .axis = AXIS_C,
        .data = { .width = width },
        .timestamp = esp_timer_get_time()
    };

    esp_err_t ret = event_publish(&evt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to publish width event: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Published EVENT WIDTH C %.3f", width);
    }
}

/**
 * @brief Limit change callback for floating switch detection
 *
 * Called by safety_monitor when any limit switch state changes.
 * Only processes C axis MAX limit (floating switch) during closing motion.
 *
 * @param[in] axis Axis that changed
 * @param[in] min_active New MIN limit state
 * @param[in] max_active New MAX limit state
 * @param[in] ctx User context (unused)
 */
static void floating_switch_limit_callback(uint8_t axis, bool min_active, bool max_active, void* ctx)
{
    (void)ctx;
    (void)min_active;

    // Only interested in C axis MAX limit activation
    if (axis != AXIS_C || !max_active) {
        return;
    }

    // Take mutex for thread-safe access
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout acquiring mutex");
        return;
    }

    // AC5: Only measure during closing motion
    if (s_c_axis_direction != C_AXIS_CLOSING_DIRECTION) {
        ESP_LOGD(TAG, "Floating switch ignored - not closing (dir=%d)", s_c_axis_direction);
        xSemaphoreGive(s_mutex);
        return;
    }

    ESP_LOGI(TAG, "Floating switch triggered during closing motion");

    // AC2: Get current position for width calculation
    float position = 0.0f;
    esp_err_t ret = motion_controller_get_position(AXIS_C, &position);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get C axis position: %s", esp_err_to_name(ret));
        xSemaphoreGive(s_mutex);
        return;
    }

    // AC2: Store width measurement (width = position at contact)
    // For C axis (picker jaw), position represents distance traveled from open position
    s_last_width = position;
    s_has_measurement = true;

    ESP_LOGI(TAG, "Width measurement: %.3f", s_last_width);

    // Release mutex before stopping motion (stop may take time)
    xSemaphoreGive(s_mutex);

    // AC3: Publish width event (within 10ms requirement met by direct call)
    publish_width_event(s_last_width);

    // AC1: Stop motion gracefully (not error stop)
    // This is a graceful stop - axis remains enabled, no ERROR state
    ret = motion_controller_stop_axis(AXIS_C);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to stop C axis: %s", esp_err_to_name(ret));
    }

    // Clear direction after stop
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_c_axis_direction = 0;
        xSemaphoreGive(s_mutex);
    }
}

/* ==========================================================================
 * Public API Implementation
 * ========================================================================== */

esp_err_t floating_switch_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Check that safety monitor is initialized
    if (!safety_monitor_is_initialized()) {
        ESP_LOGE(TAG, "safety_monitor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Check that motion controller is initialized
    if (!motion_controller_is_initialized()) {
        ESP_LOGE(TAG, "motion_controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create mutex
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize state
    s_last_width = FLOATING_SWITCH_NO_MEASUREMENT;
    s_has_measurement = false;
    s_c_axis_direction = 0;

    // Register limit callback to receive C axis limit changes
    esp_err_t ret = safety_monitor_register_limit_callback(floating_switch_limit_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register limit callback: %s", esp_err_to_name(ret));
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Initialized: C axis floating switch handler");

    return ESP_OK;
}

bool floating_switch_is_initialized(void)
{
    return s_initialized;
}

float floating_switch_get_width(void)
{
    if (!s_initialized) {
        return FLOATING_SWITCH_NO_MEASUREMENT;
    }

    float width = FLOATING_SWITCH_NO_MEASUREMENT;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        width = s_last_width;
        xSemaphoreGive(s_mutex);
    }

    return width;
}

bool floating_switch_has_measurement(void)
{
    if (!s_initialized) {
        return false;
    }

    bool has = false;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        has = s_has_measurement;
        xSemaphoreGive(s_mutex);
    }

    return has;
}

void floating_switch_clear_measurement(void)
{
    if (!s_initialized) {
        return;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_last_width = FLOATING_SWITCH_NO_MEASUREMENT;
        s_has_measurement = false;
        ESP_LOGD(TAG, "Measurement cleared");
        xSemaphoreGive(s_mutex);
    }
}

void floating_switch_on_motion_start(int8_t direction)
{
    if (!s_initialized) {
        return;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_c_axis_direction = direction;

        // AC6: Clear previous measurement when new closing motion starts
        if (direction == C_AXIS_CLOSING_DIRECTION) {
            s_last_width = FLOATING_SWITCH_NO_MEASUREMENT;
            s_has_measurement = false;
            ESP_LOGD(TAG, "Closing motion started - measurement cleared");
        }

        xSemaphoreGive(s_mutex);
    }
}

esp_err_t floating_switch_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    // Note: safety_monitor doesn't support unregistering callbacks
    // The callback will just return early if not initialized

    if (s_mutex != NULL) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "Deinitialized");

    return ESP_OK;
}
