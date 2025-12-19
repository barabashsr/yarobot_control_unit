/**
 * @file safety_monitor.c
 * @brief Safety Monitor implementation - Limit switch monitoring and E-stop
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-2: Limit Switch Monitoring
 *       AC1: All 14 limit switches readable via LIM command
 *       AC2: Single axis limit query
 *       AC3: Interrupt triggers within 1ms
 *       AC4: Debounce filtering
 *       AC5: Polarity inversion configurable
 *
 * @note Story 4-4: Emergency Stop System
 *       AC1: E-stop response within 1ms via ISR
 *       AC2: All brakes engage on E-stop
 *       AC3: ESTOP event published
 *       AC4: Motion commands rejected in ESTOP mode
 *       AC5: RST clears E-stop only if button released
 *       AC6: All axes marked UNHOMED on E-stop
 *       AC7: Hardware interrupt on GPIO_E_STOP falling edge
 */

#include "safety_monitor.h"
#include "mcp23017_wrapper.h"
#include "config_i2c.h"
#include "config_timing.h"
#include "config_limits.h"
#include "config_commands.h"
#include "config_axes.h"
#include "config_gpio.h"
#include "event_manager.h"
#include "response_formatter.h"
#include "tpic6b595.h"
#include "command_executor.h"

// C wrapper functions from motion_controller (defined in motion_controller.cpp)
extern esp_err_t motion_controller_stop_axis(uint8_t axis);
extern esp_err_t motion_controller_stop_all_axes(void);
extern bool motion_controller_is_initialized(void);
extern esp_err_t motion_controller_get_position(uint8_t axis, float* position);

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <string.h>

static const char* TAG = "safety";

/** @brief Maximum number of limit change callbacks */
#define MAX_LIMIT_CALLBACKS 4

/**
 * @brief Debounce state for a single switch
 */
typedef struct {
    bool last_raw_state;            /**< Last raw (pre-debounce) state */
    bool debounced_state;           /**< Stable debounced state */
    int64_t last_change_time_us;    /**< Timestamp of last raw state change */
} debounce_state_t;

/**
 * @brief Limit callback entry
 */
typedef struct {
    safety_limit_callback_t callback;
    void* ctx;
} limit_callback_entry_t;

/**
 * @brief Safety monitor state
 */
static struct {
    bool initialized;                           /**< Initialization flag */
    bool test_mode;                             /**< Test mode - disables hardware reads */
    TaskHandle_t task_handle;                   /**< safety_monitor_task handle */
    SemaphoreHandle_t mutex;                    /**< Protects cache access */

    // Limit state cache (AC1, AC2)
    uint16_t raw_state;                         /**< Raw MCP23017 reading */
    uint16_t limit_state;                       /**< Processed state (after polarity/debounce) */
    uint16_t prev_limit_state;                  /**< Previous state for change detection */

    // Polarity configuration (AC5)
    uint8_t polarity[SAFETY_NUM_AXES * 2];      /**< Polarity per switch (0=NO, 1=NC) */

    // Debounce state (AC4)
    debounce_state_t debounce[SAFETY_NUM_AXES * 2];  /**< Debounce state per switch */

    // Story 4-3: EndSwitchMode per switch
    uint8_t endswitch_mode[SAFETY_NUM_AXES * 2]; /**< EndSwitchMode per switch */

    // Story 4-3: Fault state per axis (both limits active)
    bool fault_state[SAFETY_NUM_AXES];          /**< true if axis is faulted */

    // Story 4-3: Blocked direction per axis
    // +1 = positive (MAX) direction blocked, -1 = negative (MIN) direction blocked, 0 = none
    int8_t blocked_direction[SAFETY_NUM_AXES];  /**< Blocked direction per axis */

    // Story 4-3: Limit change callbacks
    limit_callback_entry_t callbacks[MAX_LIMIT_CALLBACKS];
    uint8_t num_callbacks;

    // Story 4-4: E-stop state
    bool estop_active;                          /**< true if E-stop is active */
    bool axes_homed[SAFETY_NUM_AXES];           /**< Homing status per axis */

    // Story 4-4: E-stop test mode
    bool estop_test_mode;                       /**< true = use simulated button state */
    bool estop_test_button_pressed;             /**< Simulated button state */

} s_monitor = {
    .initialized = false,
    .test_mode = false,
    .task_handle = NULL,
    .mutex = NULL,
    .raw_state = 0,
    .limit_state = 0,
    .prev_limit_state = 0,
    .num_callbacks = 0,
    .estop_active = false,
};

/**
 * @brief Convert axis index to limit switch bit positions
 *
 * @param axis Axis index (0-7)
 * @param min_bit Output: bit position for MIN switch
 * @param max_bit Output: bit position for MAX switch
 */
static void axis_to_bits(uint8_t axis, uint8_t* min_bit, uint8_t* max_bit)
{
    // Pin mapping from config_i2c.h:
    // Port A (bits 0-7): X(0,1), Y(2,3), Z(4,5), A(6,7)
    // Port B (bits 8-15): B(8,9), C(10,11), D(12,13), E(14,15)
    *min_bit = axis * 2;
    *max_bit = axis * 2 + 1;
}

/**
 * @brief Apply debounce filtering to a single switch
 *
 * @param switch_idx Switch index (0-15)
 * @param raw_active Current raw state from MCP23017
 * @param now_us Current timestamp in microseconds
 * @return Debounced state (true = switch active)
 */
static bool apply_debounce(uint8_t switch_idx, bool raw_active, int64_t now_us)
{
    debounce_state_t* db = &s_monitor.debounce[switch_idx];

    // Check if raw state changed
    if (raw_active != db->last_raw_state) {
        db->last_raw_state = raw_active;
        db->last_change_time_us = now_us;
    }

    // Check if debounce period has elapsed
    int64_t elapsed_us = now_us - db->last_change_time_us;
    if (elapsed_us >= (TIMING_LIMIT_DEBOUNCE_MS * 1000)) {
        // Stable for debounce period - update debounced state
        db->debounced_state = db->last_raw_state;
    }

    return db->debounced_state;
}

/**
 * @brief Apply polarity inversion to a switch reading
 *
 * @param switch_idx Switch index (0-15)
 * @param raw_state Raw state (true = pin low = switch closed)
 * @return Logical state after polarity inversion
 */
static bool apply_polarity(uint8_t switch_idx, bool raw_state)
{
    // MCP23017 with pull-ups: pin reads LOW when switch closes circuit to GND
    // For NO switches: LOW = active (switch closed)
    // For NC switches: HIGH = active (switch opened from normally closed state)

    if (s_monitor.polarity[switch_idx] == LIMIT_POLARITY_NC) {
        // NC switch: invert the reading
        return !raw_state;
    }
    // NO switch: use raw reading (LOW = active)
    return raw_state;
}

/**
 * @brief Process raw MCP23017 reading into limit state
 *
 * @param raw_value 16-bit raw reading from MCP23017 #0
 * @return Processed limit state (2 bits per axis)
 */
static uint16_t process_raw_state(uint16_t raw_value)
{
    int64_t now_us = esp_timer_get_time();
    uint16_t result = 0;

    for (uint8_t axis = 0; axis < SAFETY_NUM_AXES; axis++) {
        // E axis has no physical switches - skip
        if (axis == SAFETY_AXIS_E) {
            continue;
        }

        uint8_t min_bit, max_bit;
        axis_to_bits(axis, &min_bit, &max_bit);

        // Get raw states (MCP23017: pin LOW when switch closed)
        bool raw_min = ((raw_value >> min_bit) & 0x01) == 0;
        bool raw_max = ((raw_value >> max_bit) & 0x01) == 0;

        // Apply polarity inversion
        bool logical_min = apply_polarity(min_bit, raw_min);
        bool logical_max = apply_polarity(max_bit, raw_max);

        // Apply debounce filtering
        bool debounced_min = apply_debounce(min_bit, logical_min, now_us);
        bool debounced_max = apply_debounce(max_bit, logical_max, now_us);

        // Set result bits
        if (debounced_min) {
            result |= (1 << min_bit);
        }
        if (debounced_max) {
            result |= (1 << max_bit);
        }
    }

    return result;
}

/**
 * @brief Publish a limit triggered event
 *
 * @param axis Axis index
 * @param is_max true for MAX limit, false for MIN limit
 */
static void publish_limit_event(uint8_t axis, bool is_max)
{
    Event evt = {
        .type = EVTTYPE_LIMIT_TRIGGERED,
        .axis = axis,
        .data.limit_state = is_max ? LIMIT_STATE_MAX : LIMIT_STATE_MIN,
        .timestamp = esp_timer_get_time()
    };

    esp_err_t err = event_publish(&evt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish LIMIT event: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Published EVENT LIMIT axis=%d %s", axis, is_max ? "MAX" : "MIN");
    }
}

/**
 * @brief Publish a motion complete (DONE) event
 *
 * @param axis Axis index
 * @param position Final position
 */
static void publish_done_event(uint8_t axis, float position)
{
    Event evt = {
        .type = EVTTYPE_MOTION_COMPLETE,
        .axis = axis,
        .data.position = position,
        .timestamp = esp_timer_get_time()
    };

    esp_err_t err = event_publish(&evt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish DONE event: %s", esp_err_to_name(err));
    }
}

/**
 * @brief Update blocked direction for an axis based on limit state
 *
 * @param axis Axis index
 * @param min_active MIN limit active
 * @param max_active MAX limit active
 */
static void update_blocked_direction(uint8_t axis, bool min_active, bool max_active)
{
    if (axis >= SAFETY_NUM_AXES) return;

    uint8_t min_bit, max_bit;
    axis_to_bits(axis, &min_bit, &max_bit);

    uint8_t min_mode = s_monitor.endswitch_mode[min_bit];
    uint8_t max_mode = s_monitor.endswitch_mode[max_bit];

    // Determine blocked direction based on active limits and their modes
    int8_t blocked = 0;

    if (min_active && (min_mode == END_SWITCH_MODE_HARD_STOP || min_mode == END_SWITCH_MODE_RESTRICT)) {
        // MIN limit blocks negative direction
        blocked = -1;
    }
    if (max_active && (max_mode == END_SWITCH_MODE_HARD_STOP || max_mode == END_SWITCH_MODE_RESTRICT)) {
        // MAX limit blocks positive direction
        // If both are active, we'll handle via fault state
        if (blocked == -1) {
            blocked = 0;  // Both blocked - handled by fault
        } else {
            blocked = +1;
        }
    }

    s_monitor.blocked_direction[axis] = blocked;
}

/**
 * @brief Handle limit switch state change
 *
 * Per ADR-022: Always stop first on limit trigger.
 * Per ADR-023: Use controlled deceleration for all stops.
 * Per ADR-024: Both limits active = FAULT state.
 *
 * @param axis Axis that changed
 * @param prev_min Previous MIN state
 * @param prev_max Previous MAX state
 * @param new_min New MIN state
 * @param new_max New MAX state
 */
static void handle_limit_change(uint8_t axis, bool prev_min, bool prev_max, bool new_min, bool new_max)
{
    if (axis >= SAFETY_NUM_AXES) return;

    uint8_t min_bit, max_bit;
    axis_to_bits(axis, &min_bit, &max_bit);

    uint8_t min_mode = s_monitor.endswitch_mode[min_bit];
    uint8_t max_mode = s_monitor.endswitch_mode[max_bit];

    bool min_activated = new_min && !prev_min;
    bool max_activated = new_max && !prev_max;

    // ADR-024: Check for both-limits fault condition
    if (new_min && new_max) {
        if (!s_monitor.fault_state[axis]) {
            s_monitor.fault_state[axis] = true;
            ESP_LOGE(TAG, "FAULT: Both limits active on axis %d", axis);

            // Publish fault event (use EVTTYPE_ERROR for now, proper fault event type could be added)
            Event evt = {
                .type = EVTTYPE_ERROR,
                .axis = axis,
                .data.error_code = 0x24,  // ADR-024 code
                .timestamp = esp_timer_get_time()
            };
            event_publish(&evt);

            // Stop motion immediately
            safety_monitor_stop_axis_motion(axis);
        }
        // Block all directions via fault state, not blocked_direction
        s_monitor.blocked_direction[axis] = 0;
        return;
    }

    // Clear fault if both limits are no longer active
    if (s_monitor.fault_state[axis] && (!new_min || !new_max)) {
        // Fault condition resolved (at least one limit cleared)
        // Note: fault_state cleared via safety_monitor_clear_fault()
    }

    // ADR-022: Always stop first on limit trigger
    // Handle MIN limit activation
    if (min_activated && min_mode != END_SWITCH_MODE_NONE) {
        ESP_LOGI(TAG, "MIN limit activated on axis %d, mode=%d", axis, min_mode);

        // Publish limit event for all modes except NONE
        publish_limit_event(axis, false);

        // Stop motion for HARD_STOP and RESTRICT modes
        if (min_mode == END_SWITCH_MODE_HARD_STOP || min_mode == END_SWITCH_MODE_RESTRICT) {
            esp_err_t err = safety_monitor_stop_axis_motion(axis);
            if (err == ESP_OK) {
                // Get final position and publish DONE event
                if (motion_controller_is_initialized()) {
                    float pos = 0.0f;
                    motion_controller_get_position(axis, &pos);
                    publish_done_event(axis, pos);
                }
            }
        }
    }

    // Handle MAX limit activation
    if (max_activated && max_mode != END_SWITCH_MODE_NONE) {
        ESP_LOGI(TAG, "MAX limit activated on axis %d, mode=%d", axis, max_mode);

        // Publish limit event for all modes except NONE
        publish_limit_event(axis, true);

        // Stop motion for HARD_STOP and RESTRICT modes
        if (max_mode == END_SWITCH_MODE_HARD_STOP || max_mode == END_SWITCH_MODE_RESTRICT) {
            esp_err_t err = safety_monitor_stop_axis_motion(axis);
            if (err == ESP_OK) {
                // Get final position and publish DONE event
                if (motion_controller_is_initialized()) {
                    float pos = 0.0f;
                    motion_controller_get_position(axis, &pos);
                    publish_done_event(axis, pos);
                }
            }
        }
    }

    // Update blocked direction
    update_blocked_direction(axis, new_min, new_max);

    // Invoke registered callbacks
    for (uint8_t i = 0; i < s_monitor.num_callbacks; i++) {
        if (s_monitor.callbacks[i].callback != NULL) {
            s_monitor.callbacks[i].callback(axis, new_min, new_max, s_monitor.callbacks[i].ctx);
        }
    }
}

/**
 * @brief Process limit state changes for all axes
 *
 * Compares new state with previous state and calls handle_limit_change for each axis that changed.
 *
 * @param new_state New limit state bitmap
 */
static void process_limit_changes(uint16_t new_state)
{
    uint16_t prev_state = s_monitor.prev_limit_state;

    // Check each axis for changes
    for (uint8_t axis = 0; axis < SAFETY_NUM_AXES; axis++) {
        if (axis == SAFETY_AXIS_E) continue;  // E axis has no limits

        uint8_t min_bit, max_bit;
        axis_to_bits(axis, &min_bit, &max_bit);

        bool prev_min = (prev_state >> min_bit) & 0x01;
        bool prev_max = (prev_state >> max_bit) & 0x01;
        bool new_min = (new_state >> min_bit) & 0x01;
        bool new_max = (new_state >> max_bit) & 0x01;

        // Handle change if either MIN or MAX changed
        if (prev_min != new_min || prev_max != new_max) {
            handle_limit_change(axis, prev_min, prev_max, new_min, new_max);
        }
    }

    // Update previous state
    s_monitor.prev_limit_state = new_state;
}

// ============================================================================
// Story 4-4: Emergency Stop System
// ============================================================================

/**
 * @brief E-stop GPIO ISR handler (AC1, AC7)
 *
 * Executes immediately on GPIO_E_STOP falling edge.
 * CRITICAL: Must complete within TIMING_ESTOP_RESPONSE_MS (1ms).
 *
 * Actions (ISR-safe, no blocking):
 * 1. Call sr_emergency_disable_all() - clears enables, engages brakes, tristates OE
 * 2. Notify safety_monitor_task via xTaskNotifyFromISR()
 *
 * @param arg Unused
 */
static void IRAM_ATTR estop_isr_handler(void* arg)
{
    (void)arg;

    // AC1, AC2: Immediately disable all motors and engage brakes
    // sr_emergency_disable_all() is ISR-safe, completes in <100µs
    sr_emergency_disable_all();

    // GPIO_SR_OE is already set HIGH by sr_emergency_disable_all()
    // This tristates all shift register outputs for extra safety

    // Notify safety_monitor_task for event publishing and mode change
    if (s_monitor.task_handle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(
            s_monitor.task_handle,
            NOTIFY_ESTOP,
            eSetBits,
            &xHigherPriorityTaskWoken
        );
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Initialize E-stop GPIO interrupt (AC7)
 *
 * Configures GPIO_E_STOP as input with pull-up, falling edge interrupt.
 *
 * @return esp_err_t
 *         - ESP_OK: E-stop GPIO configured successfully
 *         - ESP_FAIL: GPIO configuration failed
 */
static esp_err_t estop_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_E_STOP),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // Falling edge = button press
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO_E_STOP: %s", esp_err_to_name(err));
        return err;
    }

    // Install GPIO ISR service (if not already installed)
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means already installed, which is OK
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(err));
        return err;
    }

    // Add ISR handler
    err = gpio_isr_handler_add(GPIO_E_STOP, estop_isr_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add E-stop ISR handler: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "E-stop GPIO configured on GPIO%d (falling edge)", GPIO_E_STOP);
    return ESP_OK;
}

/**
 * @brief Handle E-stop activation in task context (AC3, AC6)
 *
 * Called from safety_monitor_task when NOTIFY_ESTOP received.
 * Performs non-ISR-safe operations:
 * - Set system mode to STATE_ESTOP
 * - Mark all axes as UNHOMED
 * - Publish EVENT ESTOP ACTIVE
 */
static void handle_estop_activation(void)
{
    // Prevent duplicate handling
    if (s_monitor.estop_active) {
        return;
    }

    s_monitor.estop_active = true;

    // AC3: Set system mode to ESTOP
    set_system_state(STATE_ESTOP);
    ESP_LOGE(TAG, "EMERGENCY STOP ACTIVATED");

    // CRITICAL: Stop all pulse generators to halt motor motion
    // sr_emergency_disable_all() only disables drivers, not pulse output
    if (motion_controller_is_initialized()) {
        esp_err_t err = motion_controller_stop_all_axes();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to stop all axes: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "All pulse generators stopped");
        }
    }

    // AC6: Mark all axes as UNHOMED (position may be lost)
    safety_monitor_mark_all_unhomed();

    // AC3: Publish ESTOP event
    Event evt = {
        .type = EVTTYPE_ESTOP_CHANGED,
        .axis = 0xFF,  // System-wide event
        .data.estop_active = true,
        .timestamp = esp_timer_get_time()
    };

    esp_err_t err = event_publish(&evt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish ESTOP event: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Published EVENT ESTOP ACTIVE");
    }
}

/**
 * @brief Safety monitor task (AC3)
 *
 * Waits for MCP23017 interrupt notifications and updates limit state cache.
 * Also handles E-stop notifications (Story 4-4).
 * Priority 24 (highest) on Core 0.
 *
 * @param arg Unused
 */
void safety_monitor_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "safety_monitor_task started on core %d", xPortGetCoreID());

    // Initial cache update
    safety_monitor_update_cache();

    for (;;) {
        // Wait for interrupt notification (AC3: within 1ms)
        uint32_t notification_value = 0;
        BaseType_t result = xTaskNotifyWait(
            0,                      // Don't clear on entry
            MCP_NOTIFY_LIMIT | NOTIFY_ESTOP,  // Clear limit and estop bits on exit
            &notification_value,
            pdMS_TO_TICKS(TIMING_SAFETY_POLL_MS)  // Fallback poll interval
        );

        // Story 4-4: Handle E-stop notification first (highest priority)
        if (result == pdTRUE && (notification_value & NOTIFY_ESTOP)) {
            handle_estop_activation();
            // Continue processing - don't skip limit handling if also notified
        }

        // Skip hardware updates in test mode
        if (s_monitor.test_mode) {
            continue;
        }

        if (result == pdTRUE && (notification_value & MCP_NOTIFY_LIMIT)) {
            // Interrupt notification received - read INTCAP to clear interrupt
            uint16_t capture;
            esp_err_t err = mcp23017_wrapper_get_int_capture(MCP_DEVICE_0, &capture);
            if (err == ESP_OK) {
                // INTCAP contains the pin states at interrupt time
                uint16_t new_state = 0;
                if (xSemaphoreTake(s_monitor.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    s_monitor.raw_state = capture;
                    new_state = process_raw_state(capture);
                    s_monitor.limit_state = new_state;
                    xSemaphoreGive(s_monitor.mutex);
                    ESP_LOGD(TAG, "Interrupt: raw=0x%04X processed=0x%04X",
                             capture, new_state);
                }
                // Story 4-3: Process limit changes for motion stop
                process_limit_changes(new_state);
            } else {
                ESP_LOGW(TAG, "Failed to read INTCAP: %s", esp_err_to_name(err));
            }
        } else {
            // Timeout - do a polling read as fallback
            uint16_t prev_state = s_monitor.limit_state;
            safety_monitor_update_cache();
            uint16_t new_state = s_monitor.limit_state;
            // Story 4-3: Process limit changes for motion stop
            if (prev_state != new_state) {
                process_limit_changes(new_state);
            }
        }
    }
}

esp_err_t safety_monitor_init(void)
{
    if (s_monitor.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Check MCP23017 wrapper is ready
    if (!mcp23017_wrapper_is_initialized()) {
        ESP_LOGE(TAG, "MCP23017 wrapper not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create mutex
    s_monitor.mutex = xSemaphoreCreateMutex();
    if (s_monitor.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize polarity to NO (normally open) for all switches
    memset(s_monitor.polarity, LIMIT_POLARITY_NO, sizeof(s_monitor.polarity));

    // Initialize debounce state
    memset(s_monitor.debounce, 0, sizeof(s_monitor.debounce));

    // Story 4-3: Initialize EndSwitchMode to HARD_STOP (default per ADR-021)
    for (size_t i = 0; i < sizeof(s_monitor.endswitch_mode); i++) {
        s_monitor.endswitch_mode[i] = END_SWITCH_MODE_DEFAULT;
    }

    // Story 4-3: Initialize fault state and blocked directions
    memset(s_monitor.fault_state, 0, sizeof(s_monitor.fault_state));
    memset(s_monitor.blocked_direction, 0, sizeof(s_monitor.blocked_direction));

    // Story 4-3: Initialize callbacks
    memset(s_monitor.callbacks, 0, sizeof(s_monitor.callbacks));
    s_monitor.num_callbacks = 0;

    // Story 4-4: Initialize E-stop state
    s_monitor.estop_active = false;
    memset(s_monitor.axes_homed, 0, sizeof(s_monitor.axes_homed));

    // Initialize previous limit state
    s_monitor.prev_limit_state = 0;

    // Register for interrupt notifications
    // Note: task handle set after task creation
    // For now, we'll create the task and then register it

    // Create safety monitor task - priority 24 (highest) on Core 0
    BaseType_t ret = xTaskCreatePinnedToCore(
        safety_monitor_task,
        "safety_mon",
        4096,           // Stack size
        NULL,           // Task argument
        24,             // Priority (highest)
        &s_monitor.task_handle,
        0               // Core 0
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        vSemaphoreDelete(s_monitor.mutex);
        s_monitor.mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Register task for notifications
    esp_err_t err = mcp23017_wrapper_register_notify_task(s_monitor.task_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register notify task: %s", esp_err_to_name(err));
        // Continue anyway - polling will work
    }

    // Configure MCP0 interrupts
    err = mcp23017_wrapper_config_mcp0_interrupts();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to configure MCP0 interrupts: %s", esp_err_to_name(err));
        // Continue anyway - polling will work
    }

    // Install ISR handlers
    err = mcp23017_wrapper_install_isr();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means already installed, which is OK
        ESP_LOGW(TAG, "Failed to install ISR: %s", esp_err_to_name(err));
        // Continue anyway - polling will work
    }

    // Story 4-4: Initialize E-stop GPIO interrupt
    err = estop_gpio_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to init E-stop GPIO: %s", esp_err_to_name(err));
        // Continue anyway - E-stop may still work via polling
    }

    s_monitor.initialized = true;
    ESP_LOGI(TAG, "Safety monitor initialized (with E-stop)");

    return ESP_OK;
}

esp_err_t safety_monitor_deinit(void)
{
    if (!s_monitor.initialized) {
        return ESP_OK;
    }

    // Stop task
    if (s_monitor.task_handle != NULL) {
        vTaskDelete(s_monitor.task_handle);
        s_monitor.task_handle = NULL;
    }

    // Delete mutex
    if (s_monitor.mutex != NULL) {
        vSemaphoreDelete(s_monitor.mutex);
        s_monitor.mutex = NULL;
    }

    s_monitor.initialized = false;
    ESP_LOGI(TAG, "Safety monitor deinitialized");

    return ESP_OK;
}

uint16_t safety_monitor_get_limit_state(void)
{
    uint16_t state = 0;

    if (s_monitor.mutex != NULL) {
        if (xSemaphoreTake(s_monitor.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            state = s_monitor.limit_state;
            xSemaphoreGive(s_monitor.mutex);
        }
    }

    return state;
}

esp_err_t safety_monitor_get_axis_limits(uint8_t axis, bool* min_active, bool* max_active)
{
    if (axis >= SAFETY_NUM_AXES || min_active == NULL || max_active == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // E axis has no physical limit switches
    if (axis == SAFETY_AXIS_E) {
        *min_active = false;
        *max_active = false;
        return ESP_OK;
    }

    uint16_t state = safety_monitor_get_limit_state();

    uint8_t min_bit, max_bit;
    axis_to_bits(axis, &min_bit, &max_bit);

    *min_active = (state >> min_bit) & 0x01;
    *max_active = (state >> max_bit) & 0x01;

    return ESP_OK;
}

esp_err_t safety_monitor_update_cache(void)
{
    if (!s_monitor.initialized && s_monitor.mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read current state from MCP23017 #0
    uint16_t raw_value;
    esp_err_t err = mcp23017_wrapper_read_all(MCP_DEVICE_0, &raw_value);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read MCP0: %s", esp_err_to_name(err));
        return err;
    }

    // Update cache and disable test mode
    if (xSemaphoreTake(s_monitor.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_monitor.test_mode = false;  // Exit test mode on hardware read
        s_monitor.raw_state = raw_value;
        s_monitor.limit_state = process_raw_state(raw_value);
        xSemaphoreGive(s_monitor.mutex);
    }

    return ESP_OK;
}

bool safety_monitor_is_initialized(void)
{
    return s_monitor.initialized;
}

esp_err_t safety_monitor_set_polarity(uint8_t axis, uint8_t min_polarity, uint8_t max_polarity)
{
    if (axis >= SAFETY_NUM_AXES) {
        return ESP_ERR_INVALID_ARG;
    }

    if (min_polarity > LIMIT_POLARITY_NC || max_polarity > LIMIT_POLARITY_NC) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t min_bit, max_bit;
    axis_to_bits(axis, &min_bit, &max_bit);

    s_monitor.polarity[min_bit] = min_polarity;
    s_monitor.polarity[max_bit] = max_polarity;

    // Update cache with new polarity
    if (s_monitor.initialized) {
        safety_monitor_update_cache();
    }

    return ESP_OK;
}

esp_err_t safety_monitor_get_polarity(uint8_t axis, uint8_t* min_polarity, uint8_t* max_polarity)
{
    if (axis >= SAFETY_NUM_AXES || min_polarity == NULL || max_polarity == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t min_bit, max_bit;
    axis_to_bits(axis, &min_bit, &max_bit);

    *min_polarity = s_monitor.polarity[min_bit];
    *max_polarity = s_monitor.polarity[max_bit];

    return ESP_OK;
}

void safety_monitor_inject_test_state(uint16_t simulated_state)
{
    if (s_monitor.mutex != NULL) {
        if (xSemaphoreTake(s_monitor.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_monitor.test_mode = true;  // Enable test mode to prevent overwrite
            s_monitor.limit_state = simulated_state;
            ESP_LOGI(TAG, "Injected test state: 0x%04X (test mode ON)", simulated_state);
            xSemaphoreGive(s_monitor.mutex);
        }
    }
}

// ============================================================================
// Story 4-3: Limit Switch Motion Stop API Implementation
// ============================================================================

esp_err_t safety_monitor_set_endswitch_mode(uint8_t axis, bool is_max, uint8_t mode)
{
    if (axis >= SAFETY_NUM_AXES) {
        return ESP_ERR_INVALID_ARG;
    }

    if (mode > END_SWITCH_MODE_EVENT_ONLY) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t min_bit, max_bit;
    axis_to_bits(axis, &min_bit, &max_bit);

    uint8_t switch_idx = is_max ? max_bit : min_bit;
    s_monitor.endswitch_mode[switch_idx] = mode;

    ESP_LOGD(TAG, "Set endswitch mode: axis=%d %s mode=%d",
             axis, is_max ? "MAX" : "MIN", mode);

    return ESP_OK;
}

esp_err_t safety_monitor_get_endswitch_mode(uint8_t axis, bool is_max, uint8_t* mode)
{
    if (axis >= SAFETY_NUM_AXES || mode == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t min_bit, max_bit;
    axis_to_bits(axis, &min_bit, &max_bit);

    uint8_t switch_idx = is_max ? max_bit : min_bit;
    *mode = s_monitor.endswitch_mode[switch_idx];

    return ESP_OK;
}

esp_err_t safety_monitor_stop_axis_motion(uint8_t axis)
{
    if (axis >= SAFETY_NUM_AXES) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if motion controller is available
    if (!motion_controller_is_initialized()) {
        ESP_LOGW(TAG, "Motion controller not available for axis stop");
        return ESP_ERR_INVALID_STATE;
    }

    // ADR-023: Use controlled deceleration via stopAxis()
    esp_err_t err = motion_controller_stop_axis(axis);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to stop axis %d: %s", axis, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Stopped axis %d motion due to limit", axis);
    return ESP_OK;
}

bool safety_monitor_is_direction_blocked(uint8_t axis, int8_t direction)
{
    if (axis >= SAFETY_NUM_AXES || direction == 0) {
        return false;
    }

    // If axis is faulted, all motion is blocked
    if (s_monitor.fault_state[axis]) {
        return true;
    }

    int8_t blocked = s_monitor.blocked_direction[axis];

    // Check if requested direction matches blocked direction
    // blocked = +1 means positive direction blocked (MAX limit active)
    // blocked = -1 means negative direction blocked (MIN limit active)
    if (direction > 0 && blocked > 0) {
        return true;  // Positive direction blocked
    }
    if (direction < 0 && blocked < 0) {
        return true;  // Negative direction blocked
    }

    return false;
}

bool safety_monitor_is_axis_faulted(uint8_t axis)
{
    if (axis >= SAFETY_NUM_AXES) {
        return false;
    }
    return s_monitor.fault_state[axis];
}

esp_err_t safety_monitor_clear_fault(uint8_t axis)
{
    if (axis >= SAFETY_NUM_AXES) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_monitor.fault_state[axis]) {
        // No fault to clear
        return ESP_OK;
    }

    // Check if fault condition is still present (both limits active)
    bool min_active, max_active;
    esp_err_t err = safety_monitor_get_axis_limits(axis, &min_active, &max_active);
    if (err != ESP_OK) {
        return err;
    }

    if (min_active && max_active) {
        ESP_LOGW(TAG, "Cannot clear fault on axis %d: both limits still active", axis);
        return ESP_ERR_INVALID_STATE;
    }

    // Clear fault state
    s_monitor.fault_state[axis] = false;

    // Update blocked direction based on current limit state
    update_blocked_direction(axis, min_active, max_active);

    ESP_LOGI(TAG, "Cleared fault on axis %d", axis);
    return ESP_OK;
}

esp_err_t safety_monitor_register_limit_callback(safety_limit_callback_t callback, void* ctx)
{
    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_monitor.num_callbacks >= MAX_LIMIT_CALLBACKS) {
        return ESP_ERR_NO_MEM;
    }

    s_monitor.callbacks[s_monitor.num_callbacks].callback = callback;
    s_monitor.callbacks[s_monitor.num_callbacks].ctx = ctx;
    s_monitor.num_callbacks++;

    return ESP_OK;
}

// ============================================================================
// Story 4-4: E-stop Public API Implementation
// ============================================================================

bool safety_monitor_is_estop_active(void)
{
    return s_monitor.estop_active;
}

bool safety_monitor_is_estop_button_pressed(void)
{
    // Test mode: return simulated state
    if (s_monitor.estop_test_mode) {
        return s_monitor.estop_test_button_pressed;
    }
    // GPIO_E_STOP is active-low: LOW = pressed, HIGH = released
    return gpio_get_level(GPIO_E_STOP) == 0;
}

void safety_monitor_mark_all_unhomed(void)
{
    for (uint8_t i = 0; i < SAFETY_NUM_AXES; i++) {
        s_monitor.axes_homed[i] = false;
    }
    ESP_LOGW(TAG, "All axes marked as UNHOMED (position may be lost)");
}

esp_err_t safety_monitor_reset_estop(void)
{
    // AC5: Only reset if currently in E-stop mode
    if (!s_monitor.estop_active) {
        ESP_LOGD(TAG, "RST called but not in E-stop mode");
        return ESP_ERR_INVALID_STATE;
    }

    // AC5: Check if E-stop button is released (GPIO high)
    if (safety_monitor_is_estop_button_pressed()) {
        ESP_LOGW(TAG, "Cannot reset E-stop: button still pressed");
        return ESP_ERR_INVALID_STATE;
    }

    // Clear E-stop state
    s_monitor.estop_active = false;

    // Transition system mode to IDLE
    set_system_state(STATE_IDLE);

    // Re-enable shift register outputs
    esp_err_t err = sr_enable_outputs();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to re-enable SR outputs: %s", esp_err_to_name(err));
        // Continue anyway - state cleared
    }

    // Publish ESTOP INACTIVE event
    Event evt = {
        .type = EVTTYPE_ESTOP_CHANGED,
        .axis = 0xFF,  // System-wide event
        .data.estop_active = false,
        .timestamp = esp_timer_get_time()
    };

    err = event_publish(&evt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish ESTOP INACTIVE event: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Published EVENT ESTOP INACTIVE");
    }

    ESP_LOGI(TAG, "E-stop cleared, system returned to IDLE mode");
    return ESP_OK;
}

// ============================================================================
// Story 4-4: E-stop Test Injection Functions
// ============================================================================

void safety_monitor_inject_estop(void)
{
    ESP_LOGW(TAG, "TEST: Injecting E-stop activation");

    // Simulate ISR actions (but from task context)
    sr_emergency_disable_all();

    // Notify task (or handle directly if called from same context)
    if (s_monitor.task_handle != NULL) {
        xTaskNotify(s_monitor.task_handle, NOTIFY_ESTOP, eSetBits);
    } else {
        // Task not running - handle directly
        handle_estop_activation();
    }
}

void safety_monitor_set_test_button_state(bool pressed)
{
    s_monitor.estop_test_button_pressed = pressed;
    ESP_LOGI(TAG, "TEST: E-stop button state set to %s", pressed ? "PRESSED" : "RELEASED");
}

void safety_monitor_set_estop_test_mode(bool enable)
{
    s_monitor.estop_test_mode = enable;
    if (enable) {
        s_monitor.estop_test_button_pressed = false;  // Default: released
    }
    ESP_LOGI(TAG, "TEST: E-stop test mode %s", enable ? "ENABLED" : "DISABLED");
}
