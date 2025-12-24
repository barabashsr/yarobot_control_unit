/**
 * @file brake_controller.c
 * @brief Brake Controller implementation - Per-axis brake management
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-5: Brake Control System
 *       AC1: BRAKE_ON_DISABLE - engage before motor disable, release after enable
 *       AC2: BRAKE_ON_ESTOP - engage only on E-stop via sr_emergency_disable_all()
 *       AC3: BRAKE_ON_IDLE - engage after idle timeout, release before motion
 *       AC4: BRAKE_MANUAL - only BRAKE command can control
 *       AC5: Timing correctness with proper delays
 */

#include "brake_controller.h"
#include "tpic6b595.h"
#include "config_timing.h"
#include "config_commands.h"
#include "event_manager.h"
#include "response_formatter.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <string.h>

static const char* TAG = "brake";

/**
 * @brief Axis name strings for logging
 */
static const char* const AXIS_NAMES[] = {"X", "Y", "Z", "A", "B"};

/**
 * @brief Publish brake event
 */
static void publish_brake_event(uint8_t axis, bool engaged)
{
    Event evt = {
        .type = EVTTYPE_BRAKE_CHANGED,
        .axis = axis,
        .data.brake_engaged = engaged,
        .timestamp = esp_timer_get_time()
    };
    esp_err_t err = event_publish(&evt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish brake event: %s", esp_err_to_name(err));
    }
}

/**
 * @brief Brake controller state
 */
static struct {
    bool initialized;                           /**< Initialization flag */
    SemaphoreHandle_t mutex;                    /**< Protects state access */

    // Brake state per axis (true = engaged, false = released)
    bool brake_engaged[BRAKE_NUM_AXES];         /**< Current brake state */

    // Strategy per axis
    BrakeStrategy strategy[BRAKE_NUM_AXES];     /**< Brake strategy per axis */

    // Idle timers for BRAKE_ON_IDLE strategy
    esp_timer_handle_t idle_timer[BRAKE_NUM_AXES];  /**< Idle timeout timers */
    bool idle_timer_active[BRAKE_NUM_AXES];         /**< Timer active flag */

    // Test mode
    bool test_mode;                             /**< Test mode flag */
    bool test_state[BRAKE_NUM_AXES];            /**< Injected test state */
} s_brake = {
    .initialized = false,
    .mutex = NULL,
    .test_mode = false,
};

/**
 * @brief Idle timer callback
 *
 * Called when idle timeout expires for a BRAKE_ON_IDLE axis.
 * Engages brake and publishes EVENT.
 */
static void idle_timer_callback(void* arg)
{
    uint8_t axis = (uint8_t)(uintptr_t)arg;

    if (axis >= BRAKE_NUM_AXES) {
        return;
    }

    if (xSemaphoreTake(s_brake.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Mutex timeout in idle callback for axis %d", axis);
        return;
    }

    // Only engage if strategy is still BRAKE_ON_IDLE and timer was active
    if (s_brake.strategy[axis] == BRAKE_ON_IDLE && s_brake.idle_timer_active[axis]) {
        s_brake.idle_timer_active[axis] = false;

        // Engage brake
        if (!s_brake.brake_engaged[axis]) {
            s_brake.brake_engaged[axis] = true;
            sr_set_brake(axis, false);  // false = engage (active low)
            sr_update();

            ESP_LOGI(TAG, "Axis %s brake engaged (idle timeout)", AXIS_NAMES[axis]);

            // Publish EVENT BRAKE <axis> ENGAGED
            publish_brake_event(axis, true);
        }
    }

    xSemaphoreGive(s_brake.mutex);
}

/**
 * @brief Start idle timer for an axis
 */
static void start_idle_timer(uint8_t axis)
{
    if (axis >= BRAKE_NUM_AXES || s_brake.idle_timer[axis] == NULL) {
        return;
    }

    // Stop existing timer if running
    esp_timer_stop(s_brake.idle_timer[axis]);

    // Start timer with idle timeout
    uint64_t timeout_us = (uint64_t)TIMING_IDLE_TIMEOUT_S * 1000000ULL;
    esp_err_t err = esp_timer_start_once(s_brake.idle_timer[axis], timeout_us);
    if (err == ESP_OK) {
        s_brake.idle_timer_active[axis] = true;
        ESP_LOGD(TAG, "Axis %s idle timer started (%ds)", AXIS_NAMES[axis], TIMING_IDLE_TIMEOUT_S);
    } else {
        ESP_LOGW(TAG, "Failed to start idle timer for axis %d: %s", axis, esp_err_to_name(err));
    }
}

/**
 * @brief Stop idle timer for an axis
 */
static void stop_idle_timer(uint8_t axis)
{
    if (axis >= BRAKE_NUM_AXES || s_brake.idle_timer[axis] == NULL) {
        return;
    }

    esp_timer_stop(s_brake.idle_timer[axis]);
    s_brake.idle_timer_active[axis] = false;
}

// ----------------------------------------------------------------------------
// Public API Implementation
// ----------------------------------------------------------------------------

esp_err_t brake_controller_init(void)
{
    if (s_brake.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Create mutex
    s_brake.mutex = xSemaphoreCreateMutex();
    if (s_brake.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize all brakes as engaged (safe state)
    for (int i = 0; i < BRAKE_NUM_AXES; i++) {
        s_brake.brake_engaged[i] = true;

        // TEMPORARY: Set all axes to BRAKE_MANUAL for hardware testing
        // TODO: Restore automatic brake strategies after testing complete
        s_brake.strategy[i] = BRAKE_MANUAL;

        // Original default strategy: Z = BRAKE_ON_DISABLE, others = BRAKE_ON_ESTOP
        // if (i == BRAKE_AXIS_Z) {
        //     s_brake.strategy[i] = DEFAULT_BRAKE_STRATEGY_Z;
        // } else {
        //     s_brake.strategy[i] = DEFAULT_BRAKE_STRATEGY_HORIZ;
        // }

        s_brake.idle_timer[i] = NULL;
        s_brake.idle_timer_active[i] = false;
        s_brake.test_state[i] = true;  // Engaged
    }

    // Create idle timers for all axes (used by BRAKE_ON_IDLE)
    esp_timer_create_args_t timer_args = {
        .callback = idle_timer_callback,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "brake_idle",
    };

    for (int i = 0; i < BRAKE_NUM_AXES; i++) {
        timer_args.arg = (void*)(uintptr_t)i;
        esp_err_t err = esp_timer_create(&timer_args, &s_brake.idle_timer[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create idle timer for axis %d: %s", i, esp_err_to_name(err));
            // Continue anyway - timer will just be NULL
        }
    }

    s_brake.test_mode = false;
    s_brake.initialized = true;

    ESP_LOGI(TAG, "Brake controller initialized");
    return ESP_OK;
}

bool brake_controller_is_initialized(void)
{
    return s_brake.initialized;
}

esp_err_t brake_set_state(uint8_t axis, bool engage)
{
    if (!s_brake.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (axis >= BRAKE_NUM_AXES) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_brake.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool changed = (s_brake.brake_engaged[axis] != engage);

    if (changed) {
        s_brake.brake_engaged[axis] = engage;

        // Update shift register (active low: false = engage, true = release)
        sr_set_brake(axis, !engage);

        ESP_LOGI(TAG, "Axis %s brake %s", AXIS_NAMES[axis], engage ? "engaged" : "released");
    }

    // Reset/start idle timer for BRAKE_ON_IDLE
    if (s_brake.strategy[axis] == BRAKE_ON_IDLE) {
        if (engage) {
            stop_idle_timer(axis);
        } else {
            start_idle_timer(axis);
        }
    }

    xSemaphoreGive(s_brake.mutex);

    return ESP_OK;
}

esp_err_t brake_set_state_with_timing(uint8_t axis, bool engage)
{
    esp_err_t err = brake_set_state(axis, engage);
    if (err != ESP_OK) {
        return err;
    }

    // Latch to hardware
    sr_update();

    // Apply timing delay
    if (engage) {
        vTaskDelay(pdMS_TO_TICKS(TIMING_BRAKE_ENGAGE_MS));
    } else {
        vTaskDelay(pdMS_TO_TICKS(TIMING_BRAKE_RELEASE_MS));
    }

    // Publish event after timing complete
    publish_brake_event(axis, engage);

    return ESP_OK;
}

bool brake_get_state(uint8_t axis)
{
    if (!s_brake.initialized || axis >= BRAKE_NUM_AXES) {
        return true;  // Fail-safe: report engaged
    }

    if (s_brake.test_mode) {
        return s_brake.test_state[axis];
    }

    // No need for mutex for atomic read
    return s_brake.brake_engaged[axis];
}

uint8_t brake_get_all_states(void)
{
    if (!s_brake.initialized) {
        return 0;  // All engaged
    }

    uint8_t states = 0;
    for (int i = 0; i < BRAKE_NUM_AXES; i++) {
        if (!brake_get_state(i)) {  // Not engaged = released = bit set
            states |= (1 << i);
        }
    }
    return states;
}

esp_err_t brake_set_strategy(uint8_t axis, BrakeStrategy strategy)
{
    if (axis >= BRAKE_NUM_AXES) {
        return ESP_ERR_INVALID_ARG;
    }

    if (strategy > BRAKE_MANUAL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_brake.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    BrakeStrategy old_strategy = s_brake.strategy[axis];
    s_brake.strategy[axis] = strategy;

    // Handle strategy transition
    if (old_strategy == BRAKE_ON_IDLE && strategy != BRAKE_ON_IDLE) {
        stop_idle_timer(axis);
    } else if (strategy == BRAKE_ON_IDLE && !s_brake.brake_engaged[axis]) {
        // Start idle timer if brake is released
        start_idle_timer(axis);
    }

    ESP_LOGI(TAG, "Axis %s brake strategy set to %d", AXIS_NAMES[axis], strategy);

    xSemaphoreGive(s_brake.mutex);

    return ESP_OK;
}

BrakeStrategy brake_get_strategy(uint8_t axis)
{
    if (axis >= BRAKE_NUM_AXES) {
        return BRAKE_ON_ESTOP;  // Safe default
    }

    return s_brake.strategy[axis];
}

bool brake_has_hardware(uint8_t axis)
{
    return axis < BRAKE_FIRST_NO_BRAKE;
}

// ----------------------------------------------------------------------------
// Motion Integration
// ----------------------------------------------------------------------------

esp_err_t brake_on_axis_enable(uint8_t axis)
{
    if (!s_brake.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (axis >= BRAKE_NUM_AXES) {
        return ESP_ERR_INVALID_ARG;
    }

    BrakeStrategy strategy = s_brake.strategy[axis];

    // For BRAKE_ON_DISABLE: motor enable first, then release brake after delay
    // Motor enable is caller's responsibility - we just handle brake release
    if (strategy == BRAKE_ON_DISABLE) {
        // Wait for motor to stabilize
        vTaskDelay(pdMS_TO_TICKS(TIMING_BRAKE_RELEASE_MS));

        // Release brake
        brake_set_state(axis, false);
        sr_update();

        publish_brake_event(axis, false);
        return ESP_OK;
    }

    // For BRAKE_ON_IDLE: reset idle timer and release brake if needed
    if (strategy == BRAKE_ON_IDLE) {
        if (s_brake.brake_engaged[axis]) {
            // Release brake before motion
            brake_set_state(axis, false);
            sr_update();
            vTaskDelay(pdMS_TO_TICKS(TIMING_BRAKE_RELEASE_MS));
            publish_brake_event(axis, false);
        }
        stop_idle_timer(axis);
        return ESP_OK;
    }

    // For other strategies, no action needed on enable
    return ESP_OK;
}

esp_err_t brake_on_axis_disable(uint8_t axis)
{
    if (!s_brake.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (axis >= BRAKE_NUM_AXES) {
        return ESP_ERR_INVALID_ARG;
    }

    BrakeStrategy strategy = s_brake.strategy[axis];

    // For BRAKE_ON_DISABLE: engage brake first, wait, then motor disable
    if (strategy == BRAKE_ON_DISABLE) {
        // Engage brake
        brake_set_state(axis, true);
        sr_update();

        // Wait for brake to fully engage before motor disable
        vTaskDelay(pdMS_TO_TICKS(TIMING_BRAKE_ENGAGE_MS));

        publish_brake_event(axis, true);
        return ESP_OK;
    }

    // For BRAKE_ON_ESTOP: do NOT engage brake on normal disable
    if (strategy == BRAKE_ON_ESTOP) {
        // No action - brake only engages on E-stop
        return ESP_OK;
    }

    // For BRAKE_ON_IDLE: start idle timer (brake engages after timeout)
    if (strategy == BRAKE_ON_IDLE) {
        start_idle_timer(axis);
        return ESP_OK;
    }

    // For BRAKE_MANUAL: no automatic action
    return ESP_OK;
}

esp_err_t brake_on_motion_start(uint8_t axis)
{
    if (!s_brake.initialized || axis >= BRAKE_NUM_AXES) {
        return ESP_ERR_INVALID_ARG;
    }

    // Stop idle timer (motion is starting)
    stop_idle_timer(axis);

    // If brake is engaged (from idle timeout), release it
    if (s_brake.brake_engaged[axis] && s_brake.strategy[axis] == BRAKE_ON_IDLE) {
        brake_set_state(axis, false);
        sr_update();
        vTaskDelay(pdMS_TO_TICKS(TIMING_BRAKE_RELEASE_MS));
        publish_brake_event(axis, false);
    }

    return ESP_OK;
}

void brake_on_motion_stop(uint8_t axis)
{
    if (!s_brake.initialized || axis >= BRAKE_NUM_AXES) {
        return;
    }

    // For BRAKE_ON_IDLE: start the idle timer
    if (s_brake.strategy[axis] == BRAKE_ON_IDLE) {
        start_idle_timer(axis);
    }
}

// ----------------------------------------------------------------------------
// Test/Debug
// ----------------------------------------------------------------------------

void brake_inject_test_state(uint8_t axis, bool engaged)
{
    if (axis >= BRAKE_NUM_AXES) {
        return;
    }

    s_brake.test_mode = true;
    s_brake.test_state[axis] = engaged;
}

void brake_force_idle_timeout(uint8_t axis)
{
    if (!s_brake.initialized || axis >= BRAKE_NUM_AXES) {
        return;
    }

    // Directly call the callback with NULL mutex protection
    idle_timer_callback((void*)(uintptr_t)axis);
}

esp_err_t brake_controller_deinit(void)
{
    if (!s_brake.initialized) {
        return ESP_OK;
    }

    // Stop and delete all timers
    for (int i = 0; i < BRAKE_NUM_AXES; i++) {
        if (s_brake.idle_timer[i] != NULL) {
            esp_timer_stop(s_brake.idle_timer[i]);
            esp_timer_delete(s_brake.idle_timer[i]);
            s_brake.idle_timer[i] = NULL;
        }
    }

    // Delete mutex
    if (s_brake.mutex != NULL) {
        vSemaphoreDelete(s_brake.mutex);
        s_brake.mutex = NULL;
    }

    s_brake.initialized = false;

    ESP_LOGI(TAG, "Brake controller deinitialized");
    return ESP_OK;
}
