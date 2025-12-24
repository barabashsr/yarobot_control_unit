/**
 * @file test_servo_feedback.c
 * @brief Unit tests for Servo Feedback Reader - InPos signal monitoring
 * @author YaRobot Team
 * @date 2025
 *
 * Tests acceptance criteria from Story 4-8:
 *   AC1: servo_feedback_init() succeeds after mcp23017_wrapper_init()
 *   AC2: INPOS reads all 5 servo axes, returns X:1 Y:0 Z:1 A:1 B:0
 *   AC3: INPOS X reads single axis, returns OK X 1
 *   AC5: INPOS C returns N/A for stepper axes
 *   AC6: Invalid axis returns error
 *
 * @note Hardware-dependent tests require MCP23017 I2C expander connected.
 *       Use TEST_IGNORE_MESSAGE for graceful skip when hardware unavailable.
 */

#include "unity.h"
#include "servo_feedback.h"
#include "config_axes.h"
#include "mcp23017_wrapper.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "test_servo_fb";

/* ==========================================================================
 * Test Setup/Teardown
 * ========================================================================== */

static void ensure_clean_state(void)
{
    /* Deinit servo feedback if initialized */
    if (servo_feedback_is_initialized()) {
        servo_feedback_deinit();
    }
}

static bool ensure_hardware_ready(void)
{
    /* Check if MCP23017 wrapper is initialized */
    if (!mcp23017_wrapper_is_initialized()) {
        return false;
    }
    return true;
}

/* ==========================================================================
 * Initialization Tests (AC1)
 * ========================================================================== */

/**
 * AC1: Init fails if mcp23017_wrapper not initialized
 */
TEST_CASE("init fails without mcp23017_wrapper", "[servo_feedback]")
{
    ensure_clean_state();

    /* Skip if MCP already initialized (can't easily deinit in test) */
    if (mcp23017_wrapper_is_initialized()) {
        TEST_IGNORE_MESSAGE("MCP23017 already initialized - cannot test failure case");
    }

    esp_err_t ret = servo_feedback_init();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
    TEST_ASSERT_FALSE(servo_feedback_is_initialized());
}

/**
 * AC1: Init succeeds when mcp23017_wrapper initialized
 */
TEST_CASE("init succeeds with mcp23017_wrapper", "[servo_feedback]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("MCP23017 not initialized - skipping hardware test");
    }

    esp_err_t ret = servo_feedback_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_TRUE(servo_feedback_is_initialized());

    ensure_clean_state();
}

/**
 * Double init returns ESP_OK (already initialized)
 */
TEST_CASE("double init returns ESP_OK", "[servo_feedback]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("MCP23017 not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, servo_feedback_init());
    TEST_ASSERT_EQUAL(ESP_OK, servo_feedback_init());  /* Should succeed - already init */

    ensure_clean_state();
}

/* ==========================================================================
 * API Parameter Validation Tests
 * ========================================================================== */

/**
 * AC6: get_inpos returns invalid arg for NULL pointer
 */
TEST_CASE("get_inpos returns invalid arg for null pointer", "[servo_feedback]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("MCP23017 not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, servo_feedback_init());

    esp_err_t ret = servo_feedback_get_inpos(AXIS_X, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    ensure_clean_state();
}

/**
 * AC6: get_all_inpos returns invalid arg for NULL pointer
 */
TEST_CASE("get_all_inpos returns invalid arg for null pointer", "[servo_feedback]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("MCP23017 not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, servo_feedback_init());

    esp_err_t ret = servo_feedback_get_all_inpos(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    ensure_clean_state();
}

/**
 * AC5: get_inpos returns NOT_SUPPORTED for stepper axis C
 */
TEST_CASE("get_inpos returns not supported for axis C", "[servo_feedback]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("MCP23017 not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, servo_feedback_init());

    bool inpos;
    esp_err_t ret = servo_feedback_get_inpos(AXIS_C, &inpos);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, ret);

    ensure_clean_state();
}

/**
 * AC5: get_inpos returns NOT_SUPPORTED for stepper axis D
 */
TEST_CASE("get_inpos returns not supported for axis D", "[servo_feedback]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("MCP23017 not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, servo_feedback_init());

    bool inpos;
    esp_err_t ret = servo_feedback_get_inpos(AXIS_D, &inpos);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, ret);

    ensure_clean_state();
}

/**
 * AC5: get_inpos returns NOT_SUPPORTED for discrete axis E
 */
TEST_CASE("get_inpos returns not supported for axis E", "[servo_feedback]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("MCP23017 not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, servo_feedback_init());

    bool inpos;
    esp_err_t ret = servo_feedback_get_inpos(AXIS_E, &inpos);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, ret);

    ensure_clean_state();
}

/* ==========================================================================
 * AXIS_HAS_SERVO_FEEDBACK Macro Tests
 * ========================================================================== */

/**
 * Verify AXIS_HAS_SERVO_FEEDBACK macro returns correct values
 */
TEST_CASE("AXIS_HAS_SERVO_FEEDBACK macro correctness", "[servo_feedback]")
{
    /* Servo axes should return true */
    TEST_ASSERT_TRUE(AXIS_HAS_SERVO_FEEDBACK(AXIS_X));
    TEST_ASSERT_TRUE(AXIS_HAS_SERVO_FEEDBACK(AXIS_Y));
    TEST_ASSERT_TRUE(AXIS_HAS_SERVO_FEEDBACK(AXIS_Z));
    TEST_ASSERT_TRUE(AXIS_HAS_SERVO_FEEDBACK(AXIS_A));
    TEST_ASSERT_TRUE(AXIS_HAS_SERVO_FEEDBACK(AXIS_B));

    /* Stepper/discrete axes should return false */
    TEST_ASSERT_FALSE(AXIS_HAS_SERVO_FEEDBACK(AXIS_C));
    TEST_ASSERT_FALSE(AXIS_HAS_SERVO_FEEDBACK(AXIS_D));
    TEST_ASSERT_FALSE(AXIS_HAS_SERVO_FEEDBACK(AXIS_E));
}

/* ==========================================================================
 * Hardware Read Tests (require MCP23017 hardware)
 * ========================================================================== */

/**
 * AC3: get_inpos reads servo axis X successfully
 */
TEST_CASE("get_inpos reads axis X", "[servo_feedback][hardware]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("MCP23017 not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, servo_feedback_init());

    bool inpos;
    esp_err_t ret = servo_feedback_get_inpos(AXIS_X, &inpos);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    /* Value can be 0 or 1 depending on hardware state */
    ESP_LOGI(TAG, "Axis X InPos: %d", inpos ? 1 : 0);

    ensure_clean_state();
}

/**
 * AC2: get_all_inpos reads all 5 servo axes
 */
TEST_CASE("get_all_inpos reads all servo axes", "[servo_feedback][hardware]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("MCP23017 not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, servo_feedback_init());

    uint8_t inpos_bitmap;
    esp_err_t ret = servo_feedback_get_all_inpos(&inpos_bitmap);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Validate bitmap is 5 bits (0x00-0x1F) */
    TEST_ASSERT_LESS_OR_EQUAL(0x1F, inpos_bitmap);

    ESP_LOGI(TAG, "All InPos bitmap: 0x%02X (X=%d Y=%d Z=%d A=%d B=%d)",
             inpos_bitmap,
             (inpos_bitmap >> 0) & 1,
             (inpos_bitmap >> 1) & 1,
             (inpos_bitmap >> 2) & 1,
             (inpos_bitmap >> 3) & 1,
             (inpos_bitmap >> 4) & 1);

    ensure_clean_state();
}

/* ==========================================================================
 * State Validation Tests
 * ========================================================================== */

/**
 * API returns INVALID_STATE when not initialized
 */
TEST_CASE("api returns invalid state when not initialized", "[servo_feedback]")
{
    ensure_clean_state();

    bool inpos;
    uint8_t bitmap;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, servo_feedback_get_inpos(AXIS_X, &inpos));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, servo_feedback_get_all_inpos(&bitmap));
}

/**
 * Deinit is safe when not initialized
 */
TEST_CASE("deinit is safe when not initialized", "[servo_feedback]")
{
    ensure_clean_state();

    TEST_ASSERT_FALSE(servo_feedback_is_initialized());
    TEST_ASSERT_EQUAL(ESP_OK, servo_feedback_deinit());  /* Should succeed */
}
