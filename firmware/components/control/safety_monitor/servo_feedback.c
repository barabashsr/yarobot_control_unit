/**
 * @file servo_feedback.c
 * @brief Servo Feedback Reader implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-8: Servo Feedback Processing (Basic InPos)
 *       Reads InPos signals from MCP23017 #1 Port B (GPB0-4).
 */

#include "servo_feedback.h"
#include "config_i2c.h"
#include "config_axes.h"
#include "mcp23017_wrapper.h"

#include "esp_log.h"

static const char* TAG = "SERVO_FB";

/* ==========================================================================
 * InPos Pin Mapping (from config_i2c.h)
 * ==========================================================================
 * MCP1_X_INPOS = 8  (GPB0, bit 0 within Port B)
 * MCP1_Y_INPOS = 9  (GPB1, bit 1 within Port B)
 * MCP1_Z_INPOS = 10 (GPB2, bit 2 within Port B)
 * MCP1_A_INPOS = 11 (GPB3, bit 3 within Port B)
 * MCP1_B_INPOS = 12 (GPB4, bit 4 within Port B)
 *
 * InPos signals are active HIGH from servo drivers.
 * ========================================================================== */

/** @brief Bit positions of InPos signals within MCP1 Port B */
#define INPOS_X_BIT     0   // GPB0
#define INPOS_Y_BIT     1   // GPB1
#define INPOS_Z_BIT     2   // GPB2
#define INPOS_A_BIT     3   // GPB3
#define INPOS_B_BIT     4   // GPB4

/** @brief Mask for all InPos bits in Port B (bits 0-4) */
#define INPOS_MASK      0x1F

/* ==========================================================================
 * Static Variables
 * ========================================================================== */

/** @brief Initialization flag */
static bool s_initialized = false;

/* ==========================================================================
 * Helper Functions
 * ========================================================================== */

/**
 * @brief Map axis index to InPos bit position within Port B
 *
 * @param axis Axis index (AXIS_X through AXIS_B)
 * @return Bit position (0-4) or -1 if not a servo axis
 */
static int axis_to_inpos_bit(uint8_t axis)
{
    switch (axis) {
        case AXIS_X: return INPOS_X_BIT;
        case AXIS_Y: return INPOS_Y_BIT;
        case AXIS_Z: return INPOS_Z_BIT;
        case AXIS_A: return INPOS_A_BIT;
        case AXIS_B: return INPOS_B_BIT;
        default: return -1;
    }
}

/* ==========================================================================
 * Public API Implementation
 * ========================================================================== */

esp_err_t servo_feedback_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Check that MCP23017 wrapper is initialized
    if (!mcp23017_wrapper_is_initialized()) {
        ESP_LOGE(TAG, "mcp23017_wrapper not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // No additional initialization needed - MCP23017 #1 Port B is already
    // configured as input with pull-ups by mcp23017_wrapper_init()

    s_initialized = true;
    ESP_LOGI(TAG, "Initialized: %d servo axes with InPos feedback", SERVO_FEEDBACK_NUM_AXES);

    return ESP_OK;
}

bool servo_feedback_is_initialized(void)
{
    return s_initialized;
}

esp_err_t servo_feedback_get_inpos(uint8_t axis, bool *inpos)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (inpos == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if axis has servo feedback
    if (!AXIS_HAS_SERVO_FEEDBACK(axis)) {
        ESP_LOGD(TAG, "Axis %d has no servo feedback", axis);
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Get bit position for this axis
    int bit = axis_to_inpos_bit(axis);
    if (bit < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read MCP1 Port B
    uint8_t port_b;
    esp_err_t ret = mcp23017_wrapper_read_port(MCP_DEVICE_1, MCP_PORT_B, &port_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MCP1 Port B: %s", esp_err_to_name(ret));
        return ret;
    }

    // Extract InPos bit (active HIGH from servo driver)
    *inpos = (port_b >> bit) & 1;

    ESP_LOGD(TAG, "Axis %d InPos: %d (port_b=0x%02X)", axis, *inpos ? 1 : 0, port_b);
    return ESP_OK;
}

esp_err_t servo_feedback_get_all_inpos(uint8_t *inpos_bitmap)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (inpos_bitmap == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read MCP1 Port B
    uint8_t port_b;
    esp_err_t ret = mcp23017_wrapper_read_port(MCP_DEVICE_1, MCP_PORT_B, &port_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MCP1 Port B: %s", esp_err_to_name(ret));
        return ret;
    }

    // Extract InPos bits (bits 0-4, active HIGH)
    *inpos_bitmap = port_b & INPOS_MASK;

    ESP_LOGD(TAG, "All InPos: 0x%02X (port_b=0x%02X)", *inpos_bitmap, port_b);
    return ESP_OK;
}

esp_err_t servo_feedback_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "Deinitialized");

    return ESP_OK;
}
