/**
 * @file test_brake_controller.c
 * @brief Unit tests for Brake Controller - Per-axis brake management
 * @author YaRobot Team
 * @date 2025
 *
 * Tests acceptance criteria from Story 4-5:
 *   AC1: BRAKE_ON_DISABLE - engage before motor disable, release after enable
 *   AC2: BRAKE_ON_ESTOP - engage only on E-stop via sr_emergency_disable_all()
 *   AC3: BRAKE_ON_IDLE - engage after idle timeout, release before motion
 *   AC4: BRAKE_MANUAL - only BRAKE command can control
 *   AC5: Timing correctness with proper delays
 *
 * @note Hardware-dependent tests require TPIC6B595 shift register connected.
 *       Use TEST_IGNORE_MESSAGE for graceful skip when hardware unavailable.
 */

#include "unity.h"
#include "brake_controller.h"
#include "config_defaults.h"
#include "config_timing.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

/* ==========================================================================
 * Test Setup/Teardown
 * ========================================================================== */

static void ensure_clean_state(void)
{
    /* Deinit brake controller if initialized */
    if (brake_controller_is_initialized()) {
        brake_controller_deinit();
    }
}

static bool ensure_brake_ready(void)
{
    ensure_clean_state();

    /* Initialize brake controller */
    esp_err_t ret = brake_controller_init();
    return (ret == ESP_OK);
}

/* ==========================================================================
 * Initialization Tests
 * ========================================================================== */

/**
 * Init succeeds on first call
 */
TEST_CASE("init succeeds on first call", "[brake_controller]")
{
    ensure_clean_state();

    esp_err_t ret = brake_controller_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_TRUE(brake_controller_is_initialized());

    ensure_clean_state();
}

/**
 * Double init returns ESP_ERR_INVALID_STATE
 */
TEST_CASE("double init returns ESP_ERR_INVALID_STATE", "[brake_controller]")
{
    ensure_clean_state();

    TEST_ASSERT_EQUAL(ESP_OK, brake_controller_init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, brake_controller_init());

    ensure_clean_state();
}

/**
 * Deinit is safe when not initialized
 */
TEST_CASE("deinit is safe when not initialized", "[brake_controller]")
{
    ensure_clean_state();

    /* Should not crash */
    esp_err_t ret = brake_controller_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

/**
 * is_initialized returns correct state
 */
TEST_CASE("is_initialized returns correct state", "[brake_controller]")
{
    ensure_clean_state();

    /* Should be false initially */
    TEST_ASSERT_FALSE(brake_controller_is_initialized());

    /* Init */
    TEST_ASSERT_EQUAL(ESP_OK, brake_controller_init());
    TEST_ASSERT_TRUE(brake_controller_is_initialized());

    /* Deinit */
    brake_controller_deinit();
    TEST_ASSERT_FALSE(brake_controller_is_initialized());
}

/* ==========================================================================
 * Axis Validation Tests
 * ========================================================================== */

/**
 * brake_has_hardware returns true for X, Y, Z, A, B (0-4)
 */
TEST_CASE("brake_has_hardware returns true for servo axes", "[brake_controller]")
{
    /* X, Y, Z, A, B have brakes */
    TEST_ASSERT_TRUE(brake_has_hardware(BRAKE_AXIS_X));
    TEST_ASSERT_TRUE(brake_has_hardware(BRAKE_AXIS_Y));
    TEST_ASSERT_TRUE(brake_has_hardware(BRAKE_AXIS_Z));
    TEST_ASSERT_TRUE(brake_has_hardware(BRAKE_AXIS_A));
    TEST_ASSERT_TRUE(brake_has_hardware(BRAKE_AXIS_B));
}

/**
 * brake_has_hardware returns false for C, D, E (5-7)
 */
TEST_CASE("brake_has_hardware returns false for stepper/discrete axes", "[brake_controller]")
{
    /* C, D, E have no brakes */
    TEST_ASSERT_FALSE(brake_has_hardware(5));  /* C */
    TEST_ASSERT_FALSE(brake_has_hardware(6));  /* D */
    TEST_ASSERT_FALSE(brake_has_hardware(7));  /* E */
    TEST_ASSERT_FALSE(brake_has_hardware(8));  /* Invalid */
    TEST_ASSERT_FALSE(brake_has_hardware(255)); /* Invalid */
}

/* ==========================================================================
 * Default State Tests
 * ========================================================================== */

/**
 * All brakes start engaged (safe state)
 */
TEST_CASE("all brakes start engaged after init", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* All brakes should be engaged (safe state) */
    for (uint8_t i = 0; i < BRAKE_NUM_AXES; i++) {
        TEST_ASSERT_TRUE_MESSAGE(brake_get_state(i), "Brake should be engaged after init");
    }

    ensure_clean_state();
}

/**
 * Z axis defaults to BRAKE_ON_DISABLE strategy
 */
TEST_CASE("Z axis defaults to BRAKE_ON_DISABLE", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    TEST_ASSERT_EQUAL(BRAKE_ON_DISABLE, brake_get_strategy(BRAKE_AXIS_Z));

    ensure_clean_state();
}

/**
 * Horizontal axes default to BRAKE_ON_ESTOP strategy
 */
TEST_CASE("horizontal axes default to BRAKE_ON_ESTOP", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    TEST_ASSERT_EQUAL(BRAKE_ON_ESTOP, brake_get_strategy(BRAKE_AXIS_X));
    TEST_ASSERT_EQUAL(BRAKE_ON_ESTOP, brake_get_strategy(BRAKE_AXIS_Y));
    TEST_ASSERT_EQUAL(BRAKE_ON_ESTOP, brake_get_strategy(BRAKE_AXIS_A));
    TEST_ASSERT_EQUAL(BRAKE_ON_ESTOP, brake_get_strategy(BRAKE_AXIS_B));

    ensure_clean_state();
}

/* ==========================================================================
 * Strategy Management Tests
 * ========================================================================== */

/**
 * set_strategy accepts all valid strategies
 */
TEST_CASE("set_strategy accepts all valid strategies", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Test all strategies */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_ON_DISABLE));
    TEST_ASSERT_EQUAL(BRAKE_ON_DISABLE, brake_get_strategy(BRAKE_AXIS_X));

    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_ON_ESTOP));
    TEST_ASSERT_EQUAL(BRAKE_ON_ESTOP, brake_get_strategy(BRAKE_AXIS_X));

    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_ON_IDLE));
    TEST_ASSERT_EQUAL(BRAKE_ON_IDLE, brake_get_strategy(BRAKE_AXIS_X));

    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_MANUAL));
    TEST_ASSERT_EQUAL(BRAKE_MANUAL, brake_get_strategy(BRAKE_AXIS_X));

    ensure_clean_state();
}

/**
 * set_strategy rejects invalid axis
 */
TEST_CASE("set_strategy rejects invalid axis", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, brake_set_strategy(BRAKE_NUM_AXES, BRAKE_ON_ESTOP));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, brake_set_strategy(255, BRAKE_ON_ESTOP));

    ensure_clean_state();
}

/**
 * set_strategy rejects invalid strategy value
 */
TEST_CASE("set_strategy rejects invalid strategy", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, brake_set_strategy(BRAKE_AXIS_X, 99));

    ensure_clean_state();
}

/**
 * get_strategy returns safe default for invalid axis
 */
TEST_CASE("get_strategy returns BRAKE_ON_ESTOP for invalid axis", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    TEST_ASSERT_EQUAL(BRAKE_ON_ESTOP, brake_get_strategy(BRAKE_NUM_AXES));
    TEST_ASSERT_EQUAL(BRAKE_ON_ESTOP, brake_get_strategy(255));

    ensure_clean_state();
}

/* ==========================================================================
 * Brake State Control Tests
 * ========================================================================== */

/**
 * brake_set_state accepts valid axis
 */
TEST_CASE("brake_set_state accepts valid axis", "[brake_controller][hardware]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* All valid axes should work */
    for (uint8_t i = 0; i < BRAKE_NUM_AXES; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, brake_set_state(i, true));
        TEST_ASSERT_EQUAL(ESP_OK, brake_set_state(i, false));
    }

    ensure_clean_state();
}

/**
 * brake_set_state rejects invalid axis
 */
TEST_CASE("brake_set_state rejects invalid axis", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, brake_set_state(BRAKE_NUM_AXES, true));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, brake_set_state(255, true));

    ensure_clean_state();
}

/**
 * brake_set_state fails when not initialized
 */
TEST_CASE("brake_set_state fails when not initialized", "[brake_controller]")
{
    ensure_clean_state();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, brake_set_state(0, true));
}

/**
 * brake_get_state tracks changes
 */
TEST_CASE("brake_get_state tracks state changes", "[brake_controller][hardware]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Start engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));

    /* Release */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_state(BRAKE_AXIS_X, false));
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    /* Engage */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_state(BRAKE_AXIS_X, true));
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));

    ensure_clean_state();
}

/**
 * brake_get_state returns engaged for invalid axis (fail-safe)
 */
TEST_CASE("brake_get_state returns engaged for invalid axis", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Invalid axis returns true (engaged = safe) */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_NUM_AXES));
    TEST_ASSERT_TRUE(brake_get_state(255));

    ensure_clean_state();
}

/**
 * brake_get_all_states returns correct bitmask
 */
TEST_CASE("brake_get_all_states returns correct bitmask", "[brake_controller][hardware]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* All engaged initially = bitmask 0x00 (bit = released) */
    uint8_t states = brake_get_all_states();
    TEST_ASSERT_EQUAL(0x00, states);

    /* Release X and Z */
    brake_set_state(BRAKE_AXIS_X, false);
    brake_set_state(BRAKE_AXIS_Z, false);

    states = brake_get_all_states();
    TEST_ASSERT_BITS(0x01, 0x01, states);  /* X released */
    TEST_ASSERT_BITS(0x04, 0x04, states);  /* Z released */
    TEST_ASSERT_BITS(0x02, 0x00, states);  /* Y engaged */

    ensure_clean_state();
}

/* ==========================================================================
 * Timing Constants Tests
 * ========================================================================== */

/**
 * Timing constants are defined with expected values
 */
TEST_CASE("timing constants are defined", "[brake_controller]")
{
    /* Verify timing constants exist and have reasonable values */
    TEST_ASSERT_EQUAL(50, TIMING_BRAKE_ENGAGE_MS);
    TEST_ASSERT_EQUAL(30, TIMING_BRAKE_RELEASE_MS);
    TEST_ASSERT_EQUAL(300, TIMING_IDLE_TIMEOUT_S);
}

/* ==========================================================================
 * AC1: BRAKE_ON_DISABLE Tests
 * ========================================================================== */

/**
 * AC1: brake_on_axis_disable engages brake for BRAKE_ON_DISABLE strategy
 */
TEST_CASE("AC1: brake_on_axis_disable engages brake for BRAKE_ON_DISABLE", "[brake_controller][hardware]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Z axis defaults to BRAKE_ON_DISABLE */
    TEST_ASSERT_EQUAL(BRAKE_ON_DISABLE, brake_get_strategy(BRAKE_AXIS_Z));

    /* First release brake (simulate enabled state) */
    brake_set_state(BRAKE_AXIS_Z, false);
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_Z));

    /* Call disable sequence - should engage brake */
    esp_err_t ret = brake_on_axis_disable(BRAKE_AXIS_Z);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Brake should now be engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_Z));

    ensure_clean_state();
}

/**
 * AC1: brake_on_axis_enable releases brake for BRAKE_ON_DISABLE strategy
 */
TEST_CASE("AC1: brake_on_axis_enable releases brake for BRAKE_ON_DISABLE", "[brake_controller][hardware]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Z axis defaults to BRAKE_ON_DISABLE */
    /* Brake starts engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_Z));

    /* Call enable sequence - should release brake */
    esp_err_t ret = brake_on_axis_enable(BRAKE_AXIS_Z);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Brake should now be released */
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_Z));

    ensure_clean_state();
}

/* ==========================================================================
 * AC2: BRAKE_ON_ESTOP Tests
 * ========================================================================== */

/**
 * AC2: brake_on_axis_disable does NOT engage brake for BRAKE_ON_ESTOP
 */
TEST_CASE("AC2: brake_on_axis_disable skips BRAKE_ON_ESTOP axes", "[brake_controller][hardware]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* X axis defaults to BRAKE_ON_ESTOP */
    TEST_ASSERT_EQUAL(BRAKE_ON_ESTOP, brake_get_strategy(BRAKE_AXIS_X));

    /* First release brake */
    brake_set_state(BRAKE_AXIS_X, false);
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    /* Call disable sequence - should NOT engage brake */
    esp_err_t ret = brake_on_axis_disable(BRAKE_AXIS_X);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Brake should still be released (E-stop only) */
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    ensure_clean_state();
}

/* ==========================================================================
 * AC3: BRAKE_ON_IDLE Tests
 * ========================================================================== */

/**
 * AC3: brake_on_motion_start releases brake for BRAKE_ON_IDLE
 */
TEST_CASE("AC3: brake_on_motion_start releases brake for BRAKE_ON_IDLE", "[brake_controller][hardware]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Set X to BRAKE_ON_IDLE */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_ON_IDLE));

    /* Brake starts engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));

    /* Start motion - should release brake */
    esp_err_t ret = brake_on_motion_start(BRAKE_AXIS_X);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Brake should be released */
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    ensure_clean_state();
}

/**
 * AC3: idle timeout engages brake via force function
 */
TEST_CASE("AC3: force_idle_timeout engages brake", "[brake_controller][hardware]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Set X to BRAKE_ON_IDLE */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_ON_IDLE));

    /* Release brake and mark as needing idle timer */
    brake_set_state(BRAKE_AXIS_X, false);
    brake_on_motion_stop(BRAKE_AXIS_X);  /* This starts the timer */

    /* Force immediate timeout */
    brake_force_idle_timeout(BRAKE_AXIS_X);

    /* Wait a bit for callback to execute */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Brake should be engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));

    ensure_clean_state();
}

/**
 * AC3: motion_stop starts idle timer for BRAKE_ON_IDLE
 */
TEST_CASE("AC3: brake_on_motion_stop is callable", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Set X to BRAKE_ON_IDLE */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_ON_IDLE));

    /* Release brake */
    brake_set_state(BRAKE_AXIS_X, false);

    /* motion_stop should not crash */
    brake_on_motion_stop(BRAKE_AXIS_X);

    /* Brake should still be released (timer just started) */
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    ensure_clean_state();
}

/* ==========================================================================
 * AC4: BRAKE_MANUAL Tests
 * ========================================================================== */

/**
 * AC4: BRAKE_MANUAL - disable sequence does not affect brake
 */
TEST_CASE("AC4: BRAKE_MANUAL - auto sequences do not affect brake", "[brake_controller][hardware]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Set X to BRAKE_MANUAL */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_MANUAL));

    /* Release brake */
    brake_set_state(BRAKE_AXIS_X, false);
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    /* Disable should not engage brake */
    brake_on_axis_disable(BRAKE_AXIS_X);
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    /* motion_stop should not engage brake */
    brake_on_motion_stop(BRAKE_AXIS_X);
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    ensure_clean_state();
}

/* ==========================================================================
 * Test Mode / Injection Tests
 * ========================================================================== */

/**
 * Test mode injection works
 */
TEST_CASE("test mode injection overrides actual state", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Normal state is engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));

    /* Inject released state */
    brake_inject_test_state(BRAKE_AXIS_X, false);

    /* get_state should return injected value */
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    /* Inject engaged state */
    brake_inject_test_state(BRAKE_AXIS_X, true);
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));

    ensure_clean_state();
}

/* ==========================================================================
 * Error Handling Tests
 * ========================================================================== */

/**
 * Motion functions reject invalid axis
 */
TEST_CASE("motion functions reject invalid axis", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, brake_on_axis_enable(BRAKE_NUM_AXES));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, brake_on_axis_disable(BRAKE_NUM_AXES));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, brake_on_motion_start(BRAKE_NUM_AXES));

    /* brake_on_motion_stop returns void, should not crash */
    brake_on_motion_stop(BRAKE_NUM_AXES);

    ensure_clean_state();
}

/**
 * Functions fail gracefully when not initialized
 */
TEST_CASE("functions fail when not initialized", "[brake_controller]")
{
    ensure_clean_state();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, brake_set_state(0, true));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, brake_on_axis_enable(0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, brake_on_axis_disable(0));

    /* These return safe defaults when not initialized */
    TEST_ASSERT_TRUE(brake_get_state(0));  /* Returns engaged */
    TEST_ASSERT_EQUAL(0, brake_get_all_states());  /* All engaged */
}

/* ==========================================================================
 * Strategy Transition Tests
 * ========================================================================== */

/**
 * Changing from BRAKE_ON_IDLE stops idle timer
 */
TEST_CASE("changing strategy from BRAKE_ON_IDLE stops timer", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Set X to BRAKE_ON_IDLE */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_ON_IDLE));

    /* Release brake and start idle timer */
    brake_set_state(BRAKE_AXIS_X, false);
    brake_on_motion_stop(BRAKE_AXIS_X);

    /* Change strategy to BRAKE_MANUAL */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_MANUAL));

    /* Force timeout - should NOT engage since strategy changed */
    brake_force_idle_timeout(BRAKE_AXIS_X);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Brake should still be released */
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    ensure_clean_state();
}

/* ==========================================================================
 * Concurrent Access Safety Tests
 * ========================================================================== */

/**
 * Rapid state changes don't corrupt state
 */
TEST_CASE("rapid state changes are safe", "[brake_controller][hardware]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Rapidly toggle brake state */
    for (int i = 0; i < 50; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, brake_set_state(BRAKE_AXIS_X, false));
        TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

        TEST_ASSERT_EQUAL(ESP_OK, brake_set_state(BRAKE_AXIS_X, true));
        TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));
    }

    ensure_clean_state();
}

/**
 * Rapid strategy changes don't corrupt state
 */
TEST_CASE("rapid strategy changes are safe", "[brake_controller]")
{
    if (!ensure_brake_ready()) {
        TEST_FAIL_MESSAGE("Failed to initialize brake controller");
    }

    /* Rapidly change strategy */
    for (int i = 0; i < 50; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_ON_DISABLE));
        TEST_ASSERT_EQUAL(BRAKE_ON_DISABLE, brake_get_strategy(BRAKE_AXIS_X));

        TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_ON_IDLE));
        TEST_ASSERT_EQUAL(BRAKE_ON_IDLE, brake_get_strategy(BRAKE_AXIS_X));

        TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_MANUAL));
        TEST_ASSERT_EQUAL(BRAKE_MANUAL, brake_get_strategy(BRAKE_AXIS_X));
    }

    ensure_clean_state();
}
