/**
 * @file test_safety_monitor.c
 * @brief Unit tests for Safety Monitor - Limit Switch Monitoring
 * @author YaRobot Team
 * @date 2025
 *
 * Tests acceptance criteria from Story 4-2:
 *   AC1: All 14 limit switches readable via LIM command
 *   AC2: Single axis limit query
 *   AC3: Interrupt triggers within 1ms (integration test)
 *   AC4: Debounce filtering prevents false triggers
 *   AC5: Polarity inversion configurable per switch
 *
 * @note Hardware-dependent tests require MCP23017 #0 connected.
 *       Use TEST_IGNORE_MESSAGE for graceful skip when hardware unavailable.
 */

#include "unity.h"
#include "safety_monitor.h"
#include "mcp23017_wrapper.h"
#include "config_i2c.h"
#include "config_timing.h"
#include "config_commands.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

/* ==========================================================================
 * Test Setup/Teardown
 * ========================================================================== */

static void ensure_clean_state(void)
{
    /* Deinit safety monitor if initialized */
    if (safety_monitor_is_initialized()) {
        safety_monitor_deinit();
    }
    /* Deinit MCP23017 wrapper if initialized */
    if (mcp23017_wrapper_is_initialized()) {
        mcp23017_wrapper_deinit();
    }
}

static bool ensure_hardware_ready(void)
{
    ensure_clean_state();

    /* Initialize MCP23017 wrapper */
    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        return false;
    }

    /* Initialize safety monitor */
    ret = safety_monitor_init();
    if (ret != ESP_OK) {
        mcp23017_wrapper_deinit();
        return false;
    }

    return true;
}

/* ==========================================================================
 * AC1: All 14 Limit Switches Readable
 * ========================================================================== */

/**
 * AC1: get_limit_state returns 16-bit value
 */
TEST_CASE("AC1: get_limit_state returns 16-bit value", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Get limit state - should return valid 16-bit value */
    uint16_t state = safety_monitor_get_limit_state();

    /* Value is valid (can be any combination of bits) */
    /* Just verify function returns without crash */
    (void)state;

    /* Cleanup */
    ensure_clean_state();
}

/**
 * AC1: E axis bits are always 0 (no physical switches)
 */
TEST_CASE("AC1: E axis has no limits (bits always 0)", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Get E axis limits */
    bool min_active, max_active;
    esp_err_t ret = safety_monitor_get_axis_limits(SAFETY_AXIS_E, &min_active, &max_active);

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FALSE(min_active);
    TEST_ASSERT_FALSE(max_active);

    /* Cleanup */
    ensure_clean_state();
}

/* ==========================================================================
 * AC2: Single Axis Limit Query
 * ========================================================================== */

/**
 * AC2: get_axis_limits returns valid data for valid axis
 */
TEST_CASE("AC2: get_axis_limits returns OK for valid axis", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    bool min_active, max_active;

    /* Test all axes 0-7 */
    for (uint8_t axis = 0; axis < SAFETY_NUM_AXES; axis++) {
        esp_err_t ret = safety_monitor_get_axis_limits(axis, &min_active, &max_active);
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "get_axis_limits failed for valid axis");
    }

    /* Cleanup */
    ensure_clean_state();
}

/**
 * AC2: get_axis_limits returns error for invalid axis
 */
TEST_CASE("AC2: get_axis_limits returns error for invalid axis", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    bool min_active, max_active;

    /* Invalid axis (>= SAFETY_NUM_AXES) */
    esp_err_t ret = safety_monitor_get_axis_limits(SAFETY_NUM_AXES, &min_active, &max_active);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    ret = safety_monitor_get_axis_limits(255, &min_active, &max_active);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * AC2: get_axis_limits returns error for NULL pointers
 */
TEST_CASE("AC2: get_axis_limits returns error for NULL pointers", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    bool dummy;

    /* NULL min_active */
    esp_err_t ret = safety_monitor_get_axis_limits(0, NULL, &dummy);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    /* NULL max_active */
    ret = safety_monitor_get_axis_limits(0, &dummy, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    /* Both NULL */
    ret = safety_monitor_get_axis_limits(0, NULL, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    /* Cleanup */
    ensure_clean_state();
}

/* ==========================================================================
 * AC4: Debounce Filtering
 * ========================================================================== */

/**
 * AC4: Debounce timing constant is defined
 */
TEST_CASE("AC4: Debounce timing constant is 10ms", "[safety_monitor]")
{
    TEST_ASSERT_EQUAL(10, TIMING_LIMIT_DEBOUNCE_MS);
}

/* ==========================================================================
 * AC5: Polarity Inversion
 * ========================================================================== */

/**
 * AC5: set_polarity accepts valid values
 */
TEST_CASE("AC5: set_polarity accepts valid values", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Set NO polarity */
    esp_err_t ret = safety_monitor_set_polarity(0, LIMIT_POLARITY_NO, LIMIT_POLARITY_NO);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Set NC polarity */
    ret = safety_monitor_set_polarity(0, LIMIT_POLARITY_NC, LIMIT_POLARITY_NC);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Mixed polarity */
    ret = safety_monitor_set_polarity(0, LIMIT_POLARITY_NO, LIMIT_POLARITY_NC);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * AC5: set_polarity rejects invalid axis
 */
TEST_CASE("AC5: set_polarity rejects invalid axis", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    esp_err_t ret = safety_monitor_set_polarity(SAFETY_NUM_AXES, LIMIT_POLARITY_NO, LIMIT_POLARITY_NO);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * AC5: set_polarity rejects invalid polarity values
 */
TEST_CASE("AC5: set_polarity rejects invalid polarity", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Invalid polarity value (> 1) */
    esp_err_t ret = safety_monitor_set_polarity(0, 2, LIMIT_POLARITY_NO);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    ret = safety_monitor_set_polarity(0, LIMIT_POLARITY_NO, 2);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * AC5: get_polarity returns set values
 */
TEST_CASE("AC5: get_polarity returns set values", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    uint8_t min_pol, max_pol;

    /* Set specific polarity */
    esp_err_t ret = safety_monitor_set_polarity(0, LIMIT_POLARITY_NC, LIMIT_POLARITY_NO);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Read back */
    ret = safety_monitor_get_polarity(0, &min_pol, &max_pol);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(LIMIT_POLARITY_NC, min_pol);
    TEST_ASSERT_EQUAL(LIMIT_POLARITY_NO, max_pol);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * AC5: get_polarity rejects NULL pointers
 */
TEST_CASE("AC5: get_polarity rejects NULL pointers", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    uint8_t dummy;

    esp_err_t ret = safety_monitor_get_polarity(0, NULL, &dummy);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    ret = safety_monitor_get_polarity(0, &dummy, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    /* Cleanup */
    ensure_clean_state();
}

/* ==========================================================================
 * Initialization Tests
 * ========================================================================== */

/**
 * Init returns ESP_ERR_INVALID_STATE when MCP23017 not ready
 */
TEST_CASE("init fails when MCP23017 not initialized", "[safety_monitor]")
{
    ensure_clean_state();

    /* Don't init MCP23017 first */
    esp_err_t ret = safety_monitor_init();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
    TEST_ASSERT_FALSE(safety_monitor_is_initialized());
}

/**
 * Init succeeds when MCP23017 is ready
 */
TEST_CASE("init succeeds when MCP23017 ready", "[safety_monitor][hardware]")
{
    ensure_clean_state();

    /* Init MCP23017 first */
    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Now init safety monitor */
    ret = safety_monitor_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_TRUE(safety_monitor_is_initialized());

    /* Cleanup */
    ensure_clean_state();
}

/**
 * Double init returns ESP_ERR_INVALID_STATE
 */
TEST_CASE("double init returns ESP_ERR_INVALID_STATE", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Second init should fail */
    esp_err_t ret = safety_monitor_init();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * Deinit is safe when not initialized
 */
TEST_CASE("deinit is safe when not initialized", "[safety_monitor]")
{
    ensure_clean_state();

    /* Should not crash */
    esp_err_t ret = safety_monitor_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

/**
 * is_initialized returns correct state
 */
TEST_CASE("is_initialized returns correct state", "[safety_monitor][hardware]")
{
    ensure_clean_state();

    /* Should be false initially */
    TEST_ASSERT_FALSE(safety_monitor_is_initialized());

    /* Init MCP23017 */
    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Init safety monitor */
    ret = safety_monitor_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_TRUE(safety_monitor_is_initialized());

    /* Deinit */
    safety_monitor_deinit();
    TEST_ASSERT_FALSE(safety_monitor_is_initialized());

    /* Cleanup */
    ensure_clean_state();
}

/* ==========================================================================
 * Cache Update Tests
 * ========================================================================== */

/**
 * update_cache reads from hardware
 */
TEST_CASE("update_cache reads from hardware", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Force update */
    esp_err_t ret = safety_monitor_update_cache();
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Value should be readable */
    uint16_t state = safety_monitor_get_limit_state();
    (void)state; /* Value depends on hardware state */

    /* Cleanup */
    ensure_clean_state();
}

/* ==========================================================================
 * Story 4-3: Limit Switch Motion Stop Tests
 * ========================================================================== */

/**
 * Story 4-3 Task 2: EndSwitchMode set/get works correctly
 */
TEST_CASE("4-3: set_endswitch_mode accepts valid modes", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Test all valid modes for MIN switch */
    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_set_endswitch_mode(0, false, END_SWITCH_MODE_NONE));
    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_set_endswitch_mode(0, false, END_SWITCH_MODE_HARD_STOP));
    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_set_endswitch_mode(0, false, END_SWITCH_MODE_RESTRICT));
    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_set_endswitch_mode(0, false, END_SWITCH_MODE_EVENT_ONLY));

    /* Test all valid modes for MAX switch */
    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_set_endswitch_mode(0, true, END_SWITCH_MODE_NONE));
    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_set_endswitch_mode(0, true, END_SWITCH_MODE_HARD_STOP));

    /* Cleanup */
    ensure_clean_state();
}

/**
 * Story 4-3 Task 2: EndSwitchMode rejects invalid values
 */
TEST_CASE("4-3: set_endswitch_mode rejects invalid values", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Invalid axis */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, safety_monitor_set_endswitch_mode(SAFETY_NUM_AXES, false, END_SWITCH_MODE_HARD_STOP));

    /* Invalid mode */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, safety_monitor_set_endswitch_mode(0, false, 99));

    /* Cleanup */
    ensure_clean_state();
}

/**
 * Story 4-3 Task 2: get_endswitch_mode returns set values
 */
TEST_CASE("4-3: get_endswitch_mode returns set values", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    uint8_t mode;

    /* Set MIN to RESTRICT, MAX to EVENT_ONLY */
    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_set_endswitch_mode(0, false, END_SWITCH_MODE_RESTRICT));
    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_set_endswitch_mode(0, true, END_SWITCH_MODE_EVENT_ONLY));

    /* Read back MIN */
    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_get_endswitch_mode(0, false, &mode));
    TEST_ASSERT_EQUAL(END_SWITCH_MODE_RESTRICT, mode);

    /* Read back MAX */
    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_get_endswitch_mode(0, true, &mode));
    TEST_ASSERT_EQUAL(END_SWITCH_MODE_EVENT_ONLY, mode);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * Story 4-3 Task 3: Direction blocking logic - no limits active
 */
TEST_CASE("4-3: direction not blocked when no limits active", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Inject state with no limits active */
    safety_monitor_inject_test_state(0x0000);

    /* Neither direction should be blocked */
    TEST_ASSERT_FALSE(safety_monitor_is_direction_blocked(0, +1));
    TEST_ASSERT_FALSE(safety_monitor_is_direction_blocked(0, -1));

    /* Cleanup */
    ensure_clean_state();
}

/**
 * Story 4-3 Task 3: Direction blocking - invalid input handling
 */
TEST_CASE("4-3: is_direction_blocked handles invalid input", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Invalid axis returns false */
    TEST_ASSERT_FALSE(safety_monitor_is_direction_blocked(SAFETY_NUM_AXES, +1));

    /* Zero direction returns false */
    TEST_ASSERT_FALSE(safety_monitor_is_direction_blocked(0, 0));

    /* Cleanup */
    ensure_clean_state();
}

/**
 * Story 4-3 Task 6: Fault detection - axis not faulted initially
 */
TEST_CASE("4-3: is_axis_faulted returns false initially", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* All axes should start not faulted */
    for (uint8_t axis = 0; axis < SAFETY_NUM_AXES; axis++) {
        TEST_ASSERT_FALSE(safety_monitor_is_axis_faulted(axis));
    }

    /* Invalid axis returns false */
    TEST_ASSERT_FALSE(safety_monitor_is_axis_faulted(SAFETY_NUM_AXES));

    /* Cleanup */
    ensure_clean_state();
}

/**
 * Story 4-3 Task 6: Clear fault rejects invalid axis
 */
TEST_CASE("4-3: clear_fault rejects invalid axis", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, safety_monitor_clear_fault(SAFETY_NUM_AXES));

    /* Cleanup */
    ensure_clean_state();
}

/**
 * Story 4-3 Task 6: Clear fault on non-faulted axis succeeds
 */
TEST_CASE("4-3: clear_fault on non-faulted axis succeeds", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Clear fault when not faulted should succeed */
    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_clear_fault(0));

    /* Cleanup */
    ensure_clean_state();
}

/**
 * Story 4-3: stop_axis_motion rejects invalid axis
 */
TEST_CASE("4-3: stop_axis_motion rejects invalid axis", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, safety_monitor_stop_axis_motion(SAFETY_NUM_AXES));

    /* Cleanup */
    ensure_clean_state();
}

/**
 * Story 4-3: Default EndSwitchMode is HARD_STOP
 */
TEST_CASE("4-3: default EndSwitchMode is HARD_STOP", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    uint8_t mode;

    /* Check default mode for axis 0 MIN and MAX */
    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_get_endswitch_mode(0, false, &mode));
    TEST_ASSERT_EQUAL(END_SWITCH_MODE_DEFAULT, mode);

    TEST_ASSERT_EQUAL(ESP_OK, safety_monitor_get_endswitch_mode(0, true, &mode));
    TEST_ASSERT_EQUAL(END_SWITCH_MODE_DEFAULT, mode);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * Story 4-3: Limit callback registration
 */
TEST_CASE("4-3: limit callback registration", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* NULL callback should fail */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, safety_monitor_register_limit_callback(NULL, NULL));

    /* Cleanup */
    ensure_clean_state();
}

/* ==========================================================================
 * Story 4-4: Emergency Stop System Tests
 * ========================================================================== */

/**
 * 4-4 AC1: E-stop inactive by default
 */
TEST_CASE("4-4: E-stop inactive by default", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* E-stop should be inactive after init */
    TEST_ASSERT_FALSE(safety_monitor_is_estop_active());

    /* Cleanup */
    ensure_clean_state();
}

/**
 * 4-4 AC3: inject_estop triggers E-stop mode
 */
TEST_CASE("4-4: inject_estop triggers E-stop mode", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* E-stop should be inactive initially */
    TEST_ASSERT_FALSE(safety_monitor_is_estop_active());

    /* Inject E-stop */
    safety_monitor_inject_estop();

    /* Allow task to process notification */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* E-stop should now be active */
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Cleanup - reset E-stop */
    safety_monitor_set_estop_test_mode(true);
    safety_monitor_set_test_button_state(false);  /* Button released */
    safety_monitor_reset_estop();
    safety_monitor_set_estop_test_mode(false);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * 4-4 AC5: reset_estop fails when button pressed
 */
TEST_CASE("4-4: reset_estop fails when button pressed", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Enable test mode */
    safety_monitor_set_estop_test_mode(true);

    /* Inject E-stop */
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Set button as pressed */
    safety_monitor_set_test_button_state(true);

    /* Reset should fail */
    esp_err_t ret = safety_monitor_reset_estop();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Now release button */
    safety_monitor_set_test_button_state(false);

    /* Reset should succeed */
    ret = safety_monitor_reset_estop();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FALSE(safety_monitor_is_estop_active());

    /* Disable test mode */
    safety_monitor_set_estop_test_mode(false);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * 4-4 AC5: reset_estop succeeds when button released
 */
TEST_CASE("4-4: reset_estop succeeds when button released", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Enable test mode */
    safety_monitor_set_estop_test_mode(true);
    safety_monitor_set_test_button_state(false);  /* Button released */

    /* Inject E-stop */
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Reset should succeed */
    esp_err_t ret = safety_monitor_reset_estop();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FALSE(safety_monitor_is_estop_active());

    /* Disable test mode */
    safety_monitor_set_estop_test_mode(false);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * 4-4: test_button_state simulation works
 */
TEST_CASE("4-4: test_button_state simulation works", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Enable test mode */
    safety_monitor_set_estop_test_mode(true);

    /* Set button pressed */
    safety_monitor_set_test_button_state(true);
    TEST_ASSERT_TRUE(safety_monitor_is_estop_button_pressed());

    /* Set button released */
    safety_monitor_set_test_button_state(false);
    TEST_ASSERT_FALSE(safety_monitor_is_estop_button_pressed());

    /* Disable test mode */
    safety_monitor_set_estop_test_mode(false);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * 4-4: reset_estop fails when not in E-stop mode
 */
TEST_CASE("4-4: reset_estop fails when not in estop mode", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* E-stop should be inactive */
    TEST_ASSERT_FALSE(safety_monitor_is_estop_active());

    /* Reset should fail - not in E-stop mode */
    esp_err_t ret = safety_monitor_reset_estop();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * 4-4 AC6: E-stop marks all axes unhomed
 */
TEST_CASE("4-4: estop marks all axes unhomed", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Note: This test verifies the function is callable.
     * Full verification requires checking motor homed state,
     * which is tested in integration tests. */

    /* Enable test mode */
    safety_monitor_set_estop_test_mode(true);
    safety_monitor_set_test_button_state(false);

    /* Inject E-stop (this internally calls mark_all_unhomed) */
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(50));

    /* E-stop should be active */
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Clean up */
    safety_monitor_reset_estop();
    safety_monitor_set_estop_test_mode(false);

    /* Cleanup */
    ensure_clean_state();
}

/**
 * 4-4: Multiple E-stop injections don't cause issues
 */
TEST_CASE("4-4: multiple estop injections are idempotent", "[safety_monitor][hardware]")
{
    if (!ensure_hardware_ready()) {
        TEST_IGNORE_MESSAGE("Skipping - MCP23017 hardware not connected");
    }

    /* Enable test mode */
    safety_monitor_set_estop_test_mode(true);
    safety_monitor_set_test_button_state(false);

    /* Inject E-stop multiple times */
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(20));
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(20));
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(20));

    /* E-stop should be active */
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Single reset should clear it */
    esp_err_t ret = safety_monitor_reset_estop();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FALSE(safety_monitor_is_estop_active());

    /* Disable test mode */
    safety_monitor_set_estop_test_mode(false);

    /* Cleanup */
    ensure_clean_state();
}
