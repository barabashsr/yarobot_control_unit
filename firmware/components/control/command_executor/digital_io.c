/**
 * @file digital_io.c
 * @brief Digital I/O Manager implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-7: General-Purpose Digital I/O
 *       Manages 4 digital inputs (DIN0-3) via MCP23017 #1
 *       and 8 digital outputs (DOUT0-7) via shift register chain.
 */

#include "digital_io.h"
#include "config_i2c.h"
#include "config_sr.h"
#include "config_timing.h"
#include "config_commands.h"
#include "mcp23017_wrapper.h"
#include "tpic6b595.h"
#include "event_manager.h"
#include "response_formatter.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <strings.h>  // For strcasecmp

static const char* TAG = "DIO";

/* ==========================================================================
 * Pin Mappings (from config_i2c.h)
 * ==========================================================================
 * DIN0 = MCP1_GP_IN_0 = GPA7 (pin 7 within Port A)
 * DIN1 = MCP1_GP_IN_1 = GPB5 (pin 13, which is bit 5 within Port B)
 * DIN2 = MCP1_GP_IN_2 = GPB6 (pin 14, which is bit 6 within Port B)
 * DIN3 = MCP1_GP_IN_3 = GPB7 (pin 15, which is bit 7 within Port B)
 *
 * DOUT0-7 = SR_GP_OUT_0..7 = bits 32-39 in shift register chain
 * ========================================================================== */

/** @brief Bit position of DIN0 within MCP1 Port A */
#define DIN0_BIT_IN_PORTA   7   // GPA7

/** @brief Bit positions of DIN1-3 within MCP1 Port B */
#define DIN1_BIT_IN_PORTB   5   // GPB5
#define DIN2_BIT_IN_PORTB   6   // GPB6
#define DIN3_BIT_IN_PORTB   7   // GPB7

/* ==========================================================================
 * Debounce State Structure
 * ========================================================================== */

/**
 * @brief Per-input debounce state
 */
typedef struct {
    bool raw_value;             /**< Last raw value from hardware */
    bool pending_value;         /**< Value being debounced */
    bool debounced_value;       /**< Stable debounced output */
    int64_t last_change_time;   /**< Timestamp of last raw change (us) */
} debounce_state_t;

/* ==========================================================================
 * Static Variables
 * ========================================================================== */

/** @brief Initialization flag */
static bool s_initialized = false;

/** @brief Mutex for thread-safe access */
static SemaphoreHandle_t s_mutex = NULL;

/** @brief Debounce state for each input (0-3) */
static debounce_state_t s_debounce[DIO_NUM_INPUTS];

/** @brief Event mode bitmap (bit N = DIN N event enabled) */
static uint8_t s_event_mode = 0;

/** @brief Cached output state bitmap */
static uint8_t s_output_state = 0;

/** @brief Alias table */
static dio_alias_t s_aliases[DIO_MAX_ALIASES];
static size_t s_alias_count = 0;

/* ==========================================================================
 * Default Aliases (for Epic 5, these would be loaded from config)
 * ========================================================================== */

// Note: Full alias loading from YAML config is Epic 5
// For now, aliases are empty - users can hardcode test aliases here
// Example: { "SENSOR1", DIO_TYPE_INPUT, 0 }
#define DEFAULT_ALIAS_COUNT 0

/* ==========================================================================
 * Helper Functions
 * ========================================================================== */

/**
 * @brief Read raw input values from MCP23017 #1
 *
 * @param[out] raw_inputs 4-bit bitmap with DIN3:DIN2:DIN1:DIN0
 * @return esp_err_t
 */
static esp_err_t read_raw_inputs(uint8_t* raw_inputs)
{
    uint8_t port_a, port_b;
    esp_err_t ret;

    // Read Port A (contains DIN0 = GPA7)
    ret = mcp23017_wrapper_read_port(MCP_DEVICE_1, MCP_PORT_A, &port_a);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MCP1 Port A: %s", esp_err_to_name(ret));
        return ret;
    }

    // Read Port B (contains DIN1-3 = GPB5-7)
    ret = mcp23017_wrapper_read_port(MCP_DEVICE_1, MCP_PORT_B, &port_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MCP1 Port B: %s", esp_err_to_name(ret));
        return ret;
    }

    // Debug: Log raw port values (use LOGI for visibility during testing)
    ESP_LOGI(TAG, "MCP1 raw: PortA=0x%02X PortB=0x%02X", port_a, port_b);

    // Extract individual input bits and pack into 4-bit result
    // Inputs are active-low with pull-ups, so invert
    bool din0 = !((port_a >> DIN0_BIT_IN_PORTA) & 1);
    bool din1 = !((port_b >> DIN1_BIT_IN_PORTB) & 1);
    bool din2 = !((port_b >> DIN2_BIT_IN_PORTB) & 1);
    bool din3 = !((port_b >> DIN3_BIT_IN_PORTB) & 1);

    ESP_LOGI(TAG, "DIN: d0=%d d1=%d d2=%d d3=%d (raw: %d %d %d %d)",
             din0, din1, din2, din3,
             (port_a >> DIN0_BIT_IN_PORTA) & 1,
             (port_b >> DIN1_BIT_IN_PORTB) & 1,
             (port_b >> DIN2_BIT_IN_PORTB) & 1,
             (port_b >> DIN3_BIT_IN_PORTB) & 1);

    *raw_inputs = (din3 << 3) | (din2 << 2) | (din1 << 1) | din0;
    return ESP_OK;
}

/**
 * @brief Publish DIN change event
 */
static void publish_din_event(uint8_t pin, bool value)
{
    Event event = {
        .type = EVTTYPE_DIN_CHANGED,
        .axis = 0xFF,  // Not axis-specific
        .data.din = {
            .pin = pin,
            .value = value,
            .alias = digital_io_get_alias(DIO_TYPE_INPUT, pin)
        },
        .timestamp = esp_timer_get_time()
    };
    event_publish(&event);
    ESP_LOGD(TAG, "Published DIN event: pin=%d value=%d", pin, value ? 1 : 0);
}

/* ==========================================================================
 * Public API Implementation
 * ========================================================================== */

esp_err_t digital_io_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Create mutex
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize debounce state
    for (int i = 0; i < DIO_NUM_INPUTS; i++) {
        s_debounce[i].raw_value = false;
        s_debounce[i].pending_value = false;
        s_debounce[i].debounced_value = false;
        s_debounce[i].last_change_time = 0;
    }

    // Initialize output state from shift register
    uint64_t sr_state = sr_get_state();
    s_output_state = 0;
    for (int i = 0; i < DIO_NUM_OUTPUTS; i++) {
        if (SR_GET_BIT(sr_state, SR_GP_OUT_0 + i)) {
            s_output_state |= (1 << i);
        }
    }

    // Load default aliases (stub for Epic 5 - alias loading from config)
    s_alias_count = 0;
#if DEFAULT_ALIAS_COUNT > 0
    for (size_t i = 0; i < DEFAULT_ALIAS_COUNT && s_alias_count < DIO_MAX_ALIASES; i++) {
        s_aliases[s_alias_count++] = s_default_aliases[i];
    }
#endif

    // Initialize event mode (all disabled by default)
    s_event_mode = 0;

    // Do initial read of inputs
    uint8_t raw_inputs;
    if (read_raw_inputs(&raw_inputs) == ESP_OK) {
        for (int i = 0; i < DIO_NUM_INPUTS; i++) {
            bool value = (raw_inputs >> i) & 1;
            s_debounce[i].raw_value = value;
            s_debounce[i].pending_value = value;
            s_debounce[i].debounced_value = value;
        }
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Initialized: %d inputs, %d outputs, %d aliases",
             DIO_NUM_INPUTS, DIO_NUM_OUTPUTS, (int)s_alias_count);

    return ESP_OK;
}

bool digital_io_is_initialized(void)
{
    return s_initialized;
}

esp_err_t digital_io_read_input(uint8_t pin, bool* value)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pin >= DIO_NUM_INPUTS || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read fresh from hardware (not cached debounced value)
    uint8_t raw_inputs;
    esp_err_t ret = read_raw_inputs(&raw_inputs);
    if (ret != ESP_OK) {
        return ret;
    }

    *value = (raw_inputs >> pin) & 1;
    return ESP_OK;
}

uint8_t digital_io_read_all_inputs(void)
{
    if (!s_initialized) {
        return 0;
    }

    // Read fresh from hardware
    uint8_t raw_inputs = 0;
    if (read_raw_inputs(&raw_inputs) != ESP_OK) {
        return 0;
    }
    return raw_inputs;
}

esp_err_t digital_io_set_output(uint8_t pin, bool level)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pin >= DIO_NUM_OUTPUTS) {
        return ESP_ERR_INVALID_ARG;
    }

    // Set the GP output via shift register
    esp_err_t ret = sr_set_gp_output(pin, level);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GP output %d: %s", pin, esp_err_to_name(ret));
        return ret;
    }

    // Latch to hardware
    ret = sr_update();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update SR: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update cached state
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (level) {
            s_output_state |= (1 << pin);
        } else {
            s_output_state &= ~(1 << pin);
        }
        xSemaphoreGive(s_mutex);
    }

    ESP_LOGD(TAG, "Set DOUT%d = %d", pin, level ? 1 : 0);
    return ESP_OK;
}

uint8_t digital_io_get_all_outputs(void)
{
    if (!s_initialized) {
        return 0;
    }

    // Read current state from shift register shadow
    uint64_t sr_state = sr_get_state();
    uint8_t result = 0;
    for (int i = 0; i < DIO_NUM_OUTPUTS; i++) {
        if (SR_GET_BIT(sr_state, SR_GP_OUT_0 + i)) {
            result |= (1 << i);
        }
    }
    return result;
}

esp_err_t digital_io_resolve_alias(const char* name, dio_type_t* type, uint8_t* pin)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (name == NULL || type == NULL || pin == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_ERR_NOT_FOUND;
    for (size_t i = 0; i < s_alias_count; i++) {
        if (strcasecmp(name, s_aliases[i].name) == 0) {
            *type = s_aliases[i].type;
            *pin = s_aliases[i].pin;
            ret = ESP_OK;
            break;
        }
    }

    xSemaphoreGive(s_mutex);
    return ret;
}

const char* digital_io_get_alias(dio_type_t type, uint8_t pin)
{
    if (!s_initialized) {
        return NULL;
    }

    const char* result = NULL;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (size_t i = 0; i < s_alias_count; i++) {
            if (s_aliases[i].type == type && s_aliases[i].pin == pin) {
                result = s_aliases[i].name;
                break;
            }
        }
        xSemaphoreGive(s_mutex);
    }
    return result;
}

esp_err_t digital_io_update_debounce(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read raw inputs
    uint8_t raw_inputs;
    esp_err_t ret = read_raw_inputs(&raw_inputs);
    if (ret != ESP_OK) {
        return ret;
    }

    int64_t now = esp_timer_get_time();
    int64_t debounce_time_us = TIMING_LIMIT_DEBOUNCE_MS * 1000;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (int i = 0; i < DIO_NUM_INPUTS; i++) {
        bool new_raw = (raw_inputs >> i) & 1;

        // Check if raw value changed
        if (new_raw != s_debounce[i].raw_value) {
            s_debounce[i].raw_value = new_raw;
            s_debounce[i].pending_value = new_raw;
            s_debounce[i].last_change_time = now;
        }

        // Check if debounce period has elapsed for pending value
        if (s_debounce[i].pending_value != s_debounce[i].debounced_value) {
            if ((now - s_debounce[i].last_change_time) >= debounce_time_us) {
                bool old_value = s_debounce[i].debounced_value;
                s_debounce[i].debounced_value = s_debounce[i].pending_value;

                // Publish event if event mode enabled
                if ((s_event_mode & (1 << i)) && old_value != s_debounce[i].debounced_value) {
                    // Release mutex before publishing event
                    xSemaphoreGive(s_mutex);
                    publish_din_event(i, s_debounce[i].debounced_value);
                    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                        return ESP_ERR_TIMEOUT;
                    }
                }
            }
        }
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t digital_io_set_event_mode(uint8_t pin, bool enabled)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pin >= DIO_NUM_INPUTS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (enabled) {
        s_event_mode |= (1 << pin);
    } else {
        s_event_mode &= ~(1 << pin);
    }

    xSemaphoreGive(s_mutex);
    ESP_LOGD(TAG, "Event mode for DIN%d: %s", pin, enabled ? "enabled" : "disabled");
    return ESP_OK;
}

bool digital_io_get_event_mode(uint8_t pin)
{
    if (!s_initialized || pin >= DIO_NUM_INPUTS) {
        return false;
    }
    return (s_event_mode & (1 << pin)) != 0;
}
