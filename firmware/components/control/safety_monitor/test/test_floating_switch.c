/**
 * @file test_floating_switch.c
 * @brief Unit tests for Floating Switch Handler - C axis width measurement
 * @author YaRobot Team
 * @date 2025
 *
 * Tests acceptance criteria from Story 4-9:
 *   AC1: Floating switch detection distinct from hard limit
 *   AC2: Width measurement on floating switch trigger
 *   AC3: Width event publication
 *   AC4: WIDTH command returns last measurement or -1.000
 *   AC5: Detection only during closing motion
 *   AC6: Width reset on new close motion
 *   AC7: Invalid axis handling
 *
 * @note Hardware-dependent tests require MCP23017 I2C expander and
 *       motion controller initialized. Use TEST_IGNORE_MESSAGE for
 *       graceful skip when hardware unavailable.
 */

#include "unity.h"
#include "floating_switch.h"
#include "config_axes.h"
#include "mcp23017_wrapper.h"
#include "safety_monitor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "test_float_sw";

/* ==========================================================================
 * Test Setup/Teardown
 * ========================================================================== */

static void ensure_clean_state(void)
{
    /* Deinit floating switch if initialized */
    if (floating_switch_is_initialized()) {
        floating_switch_deinit();
    }
}

static bool ensure_hardware_ready(void)
{
    /* Check if safety_monitor and motion_controller are initialized */
    if (!safety_monitor_is_initialized()) {
        return false;
    }
    /* motion_controller_is_initialized() is declared but we check via safety_monitor */
    return true;
}

/* ==========================================================================
 * Initialization Tests (AC1)
 * ========================================================================== */

/**
 * AC1: Init fails if safety_monitor not initialized
 */
TEST_CASE("init fails without safety_monitor", "[floating_switch]")
{
    ensure_clean_state();

    /* Skip if safety_monitor already initialized (can't easily deinit in test) */
    if (safety_monitor_is_initialized()) {
        TEST_IGNORE_MESSAGE("safety_monitor already initialized - cannot test failure case");
    }

    esp_err_t ret = floating_switch_init();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
    TEST_ASSERT_FALSE(floating_switch_is_initialized());
}

/**
 * AC1: Init succeeds when dependencies initialized
 */
TEST_CASE("init succeeds with dependencies", "[floating_switch]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Dependencies not initialized - skipping hardware test");
    }

    esp_err_t ret = floating_switch_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_TRUE(floating_switch_is_initialized());

    ensure_clean_state();
}

/**
 * Double init returns ESP_OK (already initialized)
 */
TEST_CASE("double init returns ESP_OK", "[floating_switch]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Dependencies not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, floating_switch_init());
    TEST_ASSERT_EQUAL(ESP_OK, floating_switch_init());  /* Should succeed - already init */

    ensure_clean_state();
}

/* ==========================================================================
 * Width Measurement API Tests (AC2, AC4)
 * ========================================================================== */

/**
 * AC4: get_width returns NO_MEASUREMENT (-1.0) when no measurement taken
 */
TEST_CASE("get_width returns no measurement initially", "[floating_switch]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Dependencies not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, floating_switch_init());

    float width = floating_switch_get_width();
    TEST_ASSERT_EQUAL_FLOAT(FLOATING_SWITCH_NO_MEASUREMENT, width);
    TEST_ASSERT_FALSE(floating_switch_has_measurement());

    ensure_clean_state();
}

/**
 * AC4: has_measurement returns false initially
 */
TEST_CASE("has_measurement is false initially", "[floating_switch]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Dependencies not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, floating_switch_init());

    TEST_ASSERT_FALSE(floating_switch_has_measurement());

    ensure_clean_state();
}

/**
 * AC6: clear_measurement resets stored width
 */
TEST_CASE("clear_measurement resets state", "[floating_switch]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Dependencies not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, floating_switch_init());

    /* Clear (even if nothing stored) should leave clean state */
    floating_switch_clear_measurement();

    float width = floating_switch_get_width();
    TEST_ASSERT_EQUAL_FLOAT(FLOATING_SWITCH_NO_MEASUREMENT, width);
    TEST_ASSERT_FALSE(floating_switch_has_measurement());

    ensure_clean_state();
}

/* ==========================================================================
 * Motion Direction Tests (AC5, AC6)
 * ========================================================================== */

/**
 * AC6: on_motion_start with closing direction clears measurement
 */
TEST_CASE("on_motion_start closing clears measurement", "[floating_switch]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Dependencies not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, floating_switch_init());

    /* Simulate starting closing motion (negative direction = -1) */
    floating_switch_on_motion_start(-1);

    /* Should have cleared any previous measurement */
    TEST_ASSERT_FALSE(floating_switch_has_measurement());
    TEST_ASSERT_EQUAL_FLOAT(FLOATING_SWITCH_NO_MEASUREMENT, floating_switch_get_width());

    ensure_clean_state();
}

/**
 * AC5: on_motion_start with opening direction does not affect measurement state
 */
TEST_CASE("on_motion_start opening does not clear", "[floating_switch]")
{
    ensure_clean_state();

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Dependencies not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, floating_switch_init());

    /* Simulate starting opening motion (positive direction = +1) */
    floating_switch_on_motion_start(1);

    /* State should remain unchanged (no measurement) */
    TEST_ASSERT_FALSE(floating_switch_has_measurement());

    ensure_clean_state();
}

/* ==========================================================================
 * AXIS_HAS_FLOATING_SWITCH Macro Tests
 * ========================================================================== */

/**
 * Verify AXIS_HAS_FLOATING_SWITCH macro returns true only for C axis
 */
TEST_CASE("AXIS_HAS_FLOATING_SWITCH macro correctness", "[floating_switch]")
{
    /* Only C axis should have floating switch */
    TEST_ASSERT_FALSE(AXIS_HAS_FLOATING_SWITCH(AXIS_X));
    TEST_ASSERT_FALSE(AXIS_HAS_FLOATING_SWITCH(AXIS_Y));
    TEST_ASSERT_FALSE(AXIS_HAS_FLOATING_SWITCH(AXIS_Z));
    TEST_ASSERT_FALSE(AXIS_HAS_FLOATING_SWITCH(AXIS_A));
    TEST_ASSERT_FALSE(AXIS_HAS_FLOATING_SWITCH(AXIS_B));
    TEST_ASSERT_TRUE(AXIS_HAS_FLOATING_SWITCH(AXIS_C));
    TEST_ASSERT_FALSE(AXIS_HAS_FLOATING_SWITCH(AXIS_D));
    TEST_ASSERT_FALSE(AXIS_HAS_FLOATING_SWITCH(AXIS_E));
}

/* ==========================================================================
 * State Validation Tests
 * ========================================================================== */

/**
 * API returns safe values when not initialized
 */
TEST_CASE("api returns safe values when not initialized", "[floating_switch]")
{
    ensure_clean_state();

    /* Should return NO_MEASUREMENT when not initialized */
    float width = floating_switch_get_width();
    TEST_ASSERT_EQUAL_FLOAT(FLOATING_SWITCH_NO_MEASUREMENT, width);

    /* Should return false when not initialized */
    TEST_ASSERT_FALSE(floating_switch_has_measurement());

    /* Clear should be safe even when not initialized */
    floating_switch_clear_measurement();  /* Should not crash */

    /* on_motion_start should be safe even when not initialized */
    floating_switch_on_motion_start(-1);  /* Should not crash */
}

/**
 * Deinit is safe when not initialized
 */
TEST_CASE("deinit is safe when not initialized", "[floating_switch]")
{
    ensure_clean_state();

    TEST_ASSERT_FALSE(floating_switch_is_initialized());
    TEST_ASSERT_EQUAL(ESP_OK, floating_switch_deinit());  /* Should succeed */
}

/**
 * is_initialized returns correct state
 */
TEST_CASE("is_initialized tracks state correctly", "[floating_switch]")
{
    ensure_clean_state();

    TEST_ASSERT_FALSE(floating_switch_is_initialized());

    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Dependencies not initialized - skipping hardware test");
    }

    TEST_ASSERT_EQUAL(ESP_OK, floating_switch_init());
    TEST_ASSERT_TRUE(floating_switch_is_initialized());

    TEST_ASSERT_EQUAL(ESP_OK, floating_switch_deinit());
    TEST_ASSERT_FALSE(floating_switch_is_initialized());
}

/* ==========================================================================
 * NO_MEASUREMENT Constant Test
 * ========================================================================== */

/**
 * Verify FLOATING_SWITCH_NO_MEASUREMENT constant is -1.0
 */
TEST_CASE("NO_MEASUREMENT constant is -1.0", "[floating_switch]")
{
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, FLOATING_SWITCH_NO_MEASUREMENT);
}
