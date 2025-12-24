/**
 * @file position_loss.c
 * @brief Position Loss Detector implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-6: Position Loss Detection
 *       AC1: Power cycle sets all servo axes POSLOS, all axes UNHOMED
 *       AC2: E-stop sets servo axes POSLOS (may have coasted)
 *       AC3: EVENT POSLOS published on position loss
 *       AC4: POSOK X clears single axis POSLOS
 *       AC5: POSOK clears all axes POSLOS
 *       AC8: Homing complete clears POSLOS
 *       AC9: Steppers use UNHOMED only (no POSLOS)
 */

#include "position_loss.h"
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

static const char* TAG = "poslos";

/**
 * @brief Axis name strings for logging
 */
static const char* const AXIS_NAMES[] = {"X", "Y", "Z", "A", "B", "C", "D", "E"};

/**
 * @brief Publish position loss event for an axis
 *
 * @param axis Axis index (0-4 for servo axes)
 */
static void publish_poslos_event(uint8_t axis)
{
    if (axis >= POSLOS_NUM_SERVO_AXES) {
        return;
    }

    Event evt = {
        .type = EVTTYPE_POSLOS,
        .axis = axis,
        .data.error_code = 0,  // Not an error, just informational
        .timestamp = esp_timer_get_time()
    };

    esp_err_t err = event_publish(&evt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish POSLOS event for axis %s: %s",
                 AXIS_NAMES[axis], esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Published EVENT POSLOS %s", AXIS_NAMES[axis]);
    }
}

/**
 * @brief Position loss detector state
 */
static struct {
    bool initialized;                           /**< Initialization flag */
    SemaphoreHandle_t mutex;                    /**< Protects state access */

    // POSLOS flags for servo axes (5 bits: X,Y,Z,A,B)
    bool poslos_flags[POSLOS_NUM_SERVO_AXES];   /**< Position lost flags */

    // HOMED flags for all axes (8 bits: X-E)
    bool homed_flags[POSLOS_NUM_ALL_AXES];      /**< Homing complete flags */

    // Test mode
    bool test_mode;                             /**< Test mode flag */
    bool test_poslos[POSLOS_NUM_SERVO_AXES];    /**< Injected test state */

} s_poslos = {
    .initialized = false,
    .mutex = NULL,
    .test_mode = false,
};

// ----------------------------------------------------------------------------
// Public API Implementation
// ----------------------------------------------------------------------------

esp_err_t position_loss_init(void)
{
    if (s_poslos.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create mutex
    s_poslos.mutex = xSemaphoreCreateMutex();
    if (s_poslos.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // AC1: Set POSLOS for all servo axes on boot
    for (int i = 0; i < POSLOS_NUM_SERVO_AXES; i++) {
        s_poslos.poslos_flags[i] = true;
        s_poslos.test_poslos[i] = true;
    }

    // AC1: Set all axes UNHOMED on boot
    for (int i = 0; i < POSLOS_NUM_ALL_AXES; i++) {
        s_poslos.homed_flags[i] = false;
    }

    s_poslos.test_mode = false;
    s_poslos.initialized = true;

    ESP_LOGI(TAG, "Position loss detector initialized - all servos POSLOS");

    // AC3: Publish POSLOS events for all servo axes
    for (int i = 0; i < POSLOS_NUM_SERVO_AXES; i++) {
        publish_poslos_event((uint8_t)i);
    }

    return ESP_OK;
}

bool position_loss_is_initialized(void)
{
    return s_poslos.initialized;
}

esp_err_t position_loss_set(uint8_t axis, bool lost)
{
    if (!s_poslos.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Only servo axes have POSLOS
    if (axis >= POSLOS_NUM_SERVO_AXES) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_poslos.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool changed = (s_poslos.poslos_flags[axis] != lost);
    s_poslos.poslos_flags[axis] = lost;

    xSemaphoreGive(s_poslos.mutex);

    if (changed) {
        if (lost) {
            ESP_LOGW(TAG, "Axis %s position lost", AXIS_NAMES[axis]);
            // Story 4-10: Increment POSLOS error counter
            error_manager_increment(ERR_CAT_POSLOS, axis, false);
            publish_poslos_event(axis);
        } else {
            ESP_LOGI(TAG, "Axis %s position acknowledged", AXIS_NAMES[axis]);
        }
    }

    return ESP_OK;
}

bool position_loss_get(uint8_t axis)
{
    // Stepper and discrete axes never have POSLOS
    if (axis >= POSLOS_NUM_SERVO_AXES) {
        return false;
    }

    if (!s_poslos.initialized) {
        return true;  // Fail-safe: report POSLOS if not initialized
    }

    if (s_poslos.test_mode) {
        return s_poslos.test_poslos[axis];
    }

    // No need for mutex for atomic read
    return s_poslos.poslos_flags[axis];
}

uint8_t position_loss_get_all(void)
{
    if (!s_poslos.initialized) {
        return 0x1F;  // All servo axes POSLOS
    }

    uint8_t bitmap = 0;
    for (int i = 0; i < POSLOS_NUM_SERVO_AXES; i++) {
        if (position_loss_get((uint8_t)i)) {
            bitmap |= (1 << i);
        }
    }
    return bitmap;
}

void position_loss_on_estop(void)
{
    if (!s_poslos.initialized) {
        ESP_LOGW(TAG, "on_estop called but not initialized");
        return;
    }

    ESP_LOGW(TAG, "E-stop: marking all servo axes POSLOS");

    // AC2: Set POSLOS for all servo axes (servos may have coasted)
    if (xSemaphoreTake(s_poslos.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < POSLOS_NUM_SERVO_AXES; i++) {
            if (!s_poslos.poslos_flags[i]) {
                s_poslos.poslos_flags[i] = true;
            }
        }
        xSemaphoreGive(s_poslos.mutex);
    }

    // AC3: Publish POSLOS events for all servo axes
    for (int i = 0; i < POSLOS_NUM_SERVO_AXES; i++) {
        publish_poslos_event((uint8_t)i);
    }

    // AC2: Steppers NOT marked POSLOS (they stop immediately, no coasting)
    // C,D remain at their current POSLOS state (which should be false)
}

void position_loss_on_homing_complete(uint8_t axis)
{
    if (!s_poslos.initialized) {
        return;
    }

    if (axis >= POSLOS_NUM_ALL_AXES) {
        return;
    }

    if (xSemaphoreTake(s_poslos.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // AC8: Clear POSLOS flag (for servo axes)
        if (axis < POSLOS_NUM_SERVO_AXES) {
            s_poslos.poslos_flags[axis] = false;
        }

        // Set HOMED flag for any axis
        s_poslos.homed_flags[axis] = true;

        xSemaphoreGive(s_poslos.mutex);
    }

    ESP_LOGI(TAG, "Axis %s homing complete - POSLOS cleared, HOMED set", AXIS_NAMES[axis]);
}

esp_err_t position_loss_acknowledge(uint8_t axis)
{
    if (!s_poslos.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // AC4: Only servo axes have POSLOS
    if (axis >= POSLOS_NUM_SERVO_AXES) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_poslos.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool was_poslos = s_poslos.poslos_flags[axis];
    s_poslos.poslos_flags[axis] = false;

    xSemaphoreGive(s_poslos.mutex);

    if (was_poslos) {
        ESP_LOGI(TAG, "POSOK: Axis %s position acknowledged by user", AXIS_NAMES[axis]);
    }

    return ESP_OK;
}

esp_err_t position_loss_acknowledge_all(void)
{
    if (!s_poslos.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_poslos.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // AC5: Clear all servo axes POSLOS
    for (int i = 0; i < POSLOS_NUM_SERVO_AXES; i++) {
        s_poslos.poslos_flags[i] = false;
    }

    xSemaphoreGive(s_poslos.mutex);

    ESP_LOGI(TAG, "POSOK: All servo axes position acknowledged by user");

    return ESP_OK;
}

bool position_loss_is_servo_axis(uint8_t axis)
{
    return axis < POSLOS_NUM_SERVO_AXES;
}

bool position_loss_is_homed(uint8_t axis)
{
    if (axis >= POSLOS_NUM_ALL_AXES) {
        return false;
    }

    if (!s_poslos.initialized) {
        return false;  // Not homed if not initialized
    }

    return s_poslos.homed_flags[axis];
}

void position_loss_mark_all_unhomed(void)
{
    if (!s_poslos.initialized) {
        return;
    }

    if (xSemaphoreTake(s_poslos.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < POSLOS_NUM_ALL_AXES; i++) {
            s_poslos.homed_flags[i] = false;
        }
        xSemaphoreGive(s_poslos.mutex);
    }

    ESP_LOGW(TAG, "All axes marked UNHOMED");
}

// ----------------------------------------------------------------------------
// Test/Debug Functions
// ----------------------------------------------------------------------------

void position_loss_inject_test_state(uint8_t axis, bool lost)
{
    if (axis >= POSLOS_NUM_SERVO_AXES) {
        return;
    }

    s_poslos.test_mode = true;
    s_poslos.test_poslos[axis] = lost;
    ESP_LOGI(TAG, "TEST: Axis %s POSLOS set to %s", AXIS_NAMES[axis], lost ? "LOST" : "OK");
}

esp_err_t position_loss_deinit(void)
{
    if (!s_poslos.initialized) {
        return ESP_OK;
    }

    // Delete mutex
    if (s_poslos.mutex != NULL) {
        vSemaphoreDelete(s_poslos.mutex);
        s_poslos.mutex = NULL;
    }

    // Reset state
    memset(s_poslos.poslos_flags, 0, sizeof(s_poslos.poslos_flags));
    memset(s_poslos.homed_flags, 0, sizeof(s_poslos.homed_flags));
    s_poslos.test_mode = false;

    s_poslos.initialized = false;

    ESP_LOGI(TAG, "Position loss detector deinitialized");
    return ESP_OK;
}
