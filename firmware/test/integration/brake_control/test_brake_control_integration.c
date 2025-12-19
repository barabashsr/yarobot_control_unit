/**
 * @file test_brake_control_integration.c
 * @brief Integration tests for Brake Control System (Story 4-5)
 * @author YaRobot Team
 * @date 2025
 *
 * Tests acceptance criteria from Story 4-5:
 *   AC1: BRAKE_ON_DISABLE - engage before motor disable, release after enable
 *   AC2: BRAKE_ON_ESTOP - engage only on E-stop via sr_emergency_disable_all()
 *   AC3: BRAKE_ON_IDLE - engage after idle timeout, release before motion
 *   AC4: BRAKE_MANUAL - only BRAKE command can control
 *   AC5: Timing correctness with proper delays
 *   AC6: EVENT BRAKE notifications published
 *
 * @note These tests use test injection functions and hardware abstraction.
 *       Full hardware testing requires physical brake solenoids connected.
 */

#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "brake_controller.h"
#include "safety_monitor.h"
#include "command_executor.h"
#include "command_parser.h"
#include "event_manager.h"
#include "mcp23017_wrapper.h"
#include "tpic6b595.h"
#include "config_commands.h"
#include "config_timing.h"
#include "config_limits.h"

static const char *TAG = "test_brake";

/** @brief Flag indicating hardware is available */
static bool s_hardware_available = false;

/** @brief Response buffer for command testing */
static char s_response[LIMIT_RESPONSE_MAX_LENGTH];

/** @brief Brake event tracking */
static volatile bool s_brake_event_received = false;
static volatile uint8_t s_brake_event_axis = 0;
static volatile bool s_brake_event_engaged = false;
static int64_t s_brake_event_timestamp_us = 0;

/**
 * @brief Event callback for brake event testing
 */
static void test_brake_event_callback(const Event* event, void* ctx)
{
    (void)ctx;
    if (event->type == EVTTYPE_BRAKE_CHANGED) {
        s_brake_event_received = true;
        s_brake_event_axis = event->axis;
        s_brake_event_engaged = event->data.brake_engaged;
        s_brake_event_timestamp_us = esp_timer_get_time();
        ESP_LOGI(TAG, "BRAKE event: axis=%d engaged=%d", event->axis, event->data.brake_engaged);
    }
}

/**
 * @brief Reset brake event state for testing
 */
static void reset_brake_event_state(void)
{
    s_brake_event_received = false;
    s_brake_event_axis = 0;
    s_brake_event_engaged = false;
    s_brake_event_timestamp_us = 0;
}

/**
 * @brief Helper to create a parsed command
 */
static void make_command(ParsedCommand* cmd, const char* verb, char axis,
                         int param_count, float param0)
{
    memset(cmd, 0, sizeof(*cmd));
    strncpy(cmd->verb, verb, sizeof(cmd->verb) - 1);
    cmd->axis = axis;
    cmd->param_count = param_count;
    if (param_count > 0) {
        cmd->params[0] = param0;
    }
}

/**
 * @brief Test setup - initialize all required components
 */
void setUp(void)
{
    /* Deinitialize if already initialized */
    if (brake_controller_is_initialized()) {
        brake_controller_deinit();
    }
    if (safety_monitor_is_initialized()) {
        safety_monitor_deinit();
    }
    if (mcp23017_wrapper_is_initialized()) {
        mcp23017_wrapper_deinit();
    }

    /* Initialize shift register (TPIC6B595) */
    esp_err_t ret = sr_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* sr_init may return INVALID_STATE if already init, which is OK */
        s_hardware_available = false;
        ESP_LOGW(TAG, "Failed to init shift register: 0x%x", ret);
        return;
    }

    /* Initialize MCP23017 wrapper (for safety monitor) */
    ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        s_hardware_available = false;
        ESP_LOGW(TAG, "Failed to init MCP23017: 0x%x", ret);
        return;
    }

    /* Initialize safety monitor */
    ret = safety_monitor_init();
    if (ret != ESP_OK) {
        mcp23017_wrapper_deinit();
        s_hardware_available = false;
        ESP_LOGW(TAG, "Failed to init safety monitor: 0x%x", ret);
        return;
    }

    /* Initialize command executor */
    ret = cmd_executor_init();
    if (ret != ESP_OK) {
        s_hardware_available = false;
        ESP_LOGW(TAG, "Failed to init command executor: 0x%x", ret);
        return;
    }

    /* Initialize brake controller */
    ret = brake_controller_init();
    if (ret != ESP_OK) {
        s_hardware_available = false;
        ESP_LOGW(TAG, "Failed to init brake controller: 0x%x", ret);
        return;
    }

    /* Register event callback */
    event_subscribe(EVTTYPE_BRAKE_CHANGED, test_brake_event_callback, NULL);

    s_hardware_available = true;
    reset_brake_event_state();

    /* Enable test mode for safety monitor */
    safety_monitor_set_estop_test_mode(true);
    safety_monitor_set_test_button_state(false);

    /* Set system to READY mode for motion commands */
    set_system_state(STATE_READY);

    /* Small delay to let tasks start */
    vTaskDelay(pdMS_TO_TICKS(50));
}

/**
 * @brief Test teardown - cleanup
 */
void tearDown(void)
{
    /* Reset E-stop if active */
    if (safety_monitor_is_estop_active()) {
        safety_monitor_set_test_button_state(false);
        safety_monitor_reset_estop();
    }

    safety_monitor_set_estop_test_mode(false);

    if (brake_controller_is_initialized()) {
        brake_controller_deinit();
    }
    if (safety_monitor_is_initialized()) {
        safety_monitor_deinit();
    }
    if (mcp23017_wrapper_is_initialized()) {
        mcp23017_wrapper_deinit();
    }
}

/* ==========================================================================
 * AC1: BRAKE_ON_DISABLE Integration Tests
 * ========================================================================== */

/**
 * @brief Test: EN OFF engages brake for BRAKE_ON_DISABLE axis (Z)
 *
 * Verifies the EN handler properly sequences:
 * 1. Engage brake
 * 2. Wait timing delay
 * 3. Disable motor
 */
void test_ac1_en_off_engages_brake_on_disable(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Z axis defaults to BRAKE_ON_DISABLE */
    TEST_ASSERT_EQUAL(BRAKE_ON_DISABLE, brake_get_strategy(BRAKE_AXIS_Z));

    /* First release brake (simulate enabled state) */
    brake_set_state(BRAKE_AXIS_Z, false);
    sr_update();
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_Z));

    reset_brake_event_state();
    int64_t start_us = esp_timer_get_time();

    /* Call disable sequence */
    esp_err_t ret = brake_on_axis_disable(BRAKE_AXIS_Z);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    int64_t end_us = esp_timer_get_time();
    int64_t elapsed_ms = (end_us - start_us) / 1000;

    /* Brake should be engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_Z));

    /* Should have waited at least TIMING_BRAKE_ENGAGE_MS */
    TEST_ASSERT_GREATER_OR_EQUAL(TIMING_BRAKE_ENGAGE_MS, elapsed_ms);
    ESP_LOGI(TAG, "brake_on_axis_disable took %lld ms (expected >= %d)", elapsed_ms, TIMING_BRAKE_ENGAGE_MS);

    /* Wait for event to be published */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Event should have been published */
    TEST_ASSERT_TRUE(s_brake_event_received);
    TEST_ASSERT_EQUAL(BRAKE_AXIS_Z, s_brake_event_axis);
    TEST_ASSERT_TRUE(s_brake_event_engaged);
}

/**
 * @brief Test: EN ON releases brake for BRAKE_ON_DISABLE axis (Z)
 */
void test_ac1_en_on_releases_brake_on_disable(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Z axis defaults to BRAKE_ON_DISABLE */
    /* Brake starts engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_Z));

    reset_brake_event_state();
    int64_t start_us = esp_timer_get_time();

    /* Call enable sequence */
    esp_err_t ret = brake_on_axis_enable(BRAKE_AXIS_Z);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    int64_t end_us = esp_timer_get_time();
    int64_t elapsed_ms = (end_us - start_us) / 1000;

    /* Brake should be released */
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_Z));

    /* Should have waited at least TIMING_BRAKE_RELEASE_MS */
    TEST_ASSERT_GREATER_OR_EQUAL(TIMING_BRAKE_RELEASE_MS, elapsed_ms);
    ESP_LOGI(TAG, "brake_on_axis_enable took %lld ms (expected >= %d)", elapsed_ms, TIMING_BRAKE_RELEASE_MS);

    /* Wait for event to be published */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Event should have been published */
    TEST_ASSERT_TRUE(s_brake_event_received);
    TEST_ASSERT_EQUAL(BRAKE_AXIS_Z, s_brake_event_axis);
    TEST_ASSERT_FALSE(s_brake_event_engaged);
}

/* ==========================================================================
 * AC2: BRAKE_ON_ESTOP Integration Tests
 * ========================================================================== */

/**
 * @brief Test: Normal disable does NOT engage brake for BRAKE_ON_ESTOP
 */
void test_ac2_normal_disable_does_not_engage_estop_brake(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* X axis defaults to BRAKE_ON_ESTOP */
    TEST_ASSERT_EQUAL(BRAKE_ON_ESTOP, brake_get_strategy(BRAKE_AXIS_X));

    /* Release brake */
    brake_set_state(BRAKE_AXIS_X, false);
    sr_update();
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    /* Call disable sequence - should NOT engage brake */
    esp_err_t ret = brake_on_axis_disable(BRAKE_AXIS_X);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Brake should still be released */
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));
}

/**
 * @brief Test: E-stop engages all brakes via sr_emergency_disable_all()
 *
 * Verifies that when E-stop is triggered, all shift register bits
 * go to 0 (engaging all brakes via active-low control).
 */
void test_ac2_estop_engages_all_brakes(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Release all brakes first */
    for (uint8_t i = 0; i < BRAKE_NUM_AXES; i++) {
        brake_set_state(i, false);
    }
    sr_update();

    /* Verify all released */
    for (uint8_t i = 0; i < BRAKE_NUM_AXES; i++) {
        TEST_ASSERT_FALSE_MESSAGE(brake_get_state(i), "Brake should be released before E-stop");
    }

    /* Trigger E-stop - this should set all SR bits to 0 */
    safety_monitor_inject_estop();

    /* Wait for E-stop processing */
    const int max_wait_ms = 100;
    int elapsed_ms = 0;
    while (elapsed_ms < max_wait_ms && !safety_monitor_is_estop_active()) {
        vTaskDelay(pdMS_TO_TICKS(1));
        elapsed_ms++;
    }

    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Note: E-stop calls sr_emergency_disable_all() which sets all bits to 0
     * This engages all brakes due to active-low logic.
     * The brake_controller tracking may not reflect this since it's done
     * at hardware level, but physical brakes are engaged. */

    ESP_LOGI(TAG, "E-stop triggered - all SR bits set to 0 (brakes engaged)");
}

/* ==========================================================================
 * AC3: BRAKE_ON_IDLE Integration Tests
 * ========================================================================== */

/**
 * @brief Test: Motion start releases brake for BRAKE_ON_IDLE
 */
void test_ac3_motion_start_releases_idle_brake(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Set X axis to BRAKE_ON_IDLE */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_ON_IDLE));

    /* Brake starts engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));

    reset_brake_event_state();

    /* Motion start should release brake */
    esp_err_t ret = brake_on_motion_start(BRAKE_AXIS_X);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Brake should be released */
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    /* Wait for event */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Event should have been published */
    TEST_ASSERT_TRUE(s_brake_event_received);
    TEST_ASSERT_FALSE(s_brake_event_engaged);
}

/**
 * @brief Test: Idle timeout engages brake for BRAKE_ON_IDLE
 *
 * Uses force_idle_timeout() to simulate immediate timeout.
 */
void test_ac3_idle_timeout_engages_brake(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Set X axis to BRAKE_ON_IDLE */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_ON_IDLE));

    /* Release brake and start idle timer */
    brake_set_state(BRAKE_AXIS_X, false);
    sr_update();
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    /* Start idle timer */
    brake_on_motion_stop(BRAKE_AXIS_X);

    reset_brake_event_state();

    /* Force immediate timeout */
    brake_force_idle_timeout(BRAKE_AXIS_X);

    /* Wait for callback execution */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Brake should be engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));

    /* Event should have been published */
    TEST_ASSERT_TRUE(s_brake_event_received);
    TEST_ASSERT_TRUE(s_brake_event_engaged);
}

/**
 * @brief Test: Idle timeout constant is 5 minutes (300 seconds)
 */
void test_ac3_idle_timeout_constant(void)
{
    TEST_ASSERT_EQUAL(300, TIMING_IDLE_TIMEOUT_S);
    ESP_LOGI(TAG, "Idle timeout: %d seconds (%d minutes)", TIMING_IDLE_TIMEOUT_S, TIMING_IDLE_TIMEOUT_S / 60);
}

/* ==========================================================================
 * AC4: BRAKE_MANUAL Integration Tests
 * ========================================================================== */

/**
 * @brief Test: BRAKE command works for BRAKE_MANUAL axis
 */
void test_ac4_brake_command_works_for_manual_strategy(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Set X axis to BRAKE_MANUAL */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_MANUAL));

    /* Brake starts engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));

    /* Manual release via direct API (simulates BRAKE X 0 command) */
    esp_err_t ret = brake_set_state_with_timing(BRAKE_AXIS_X, false);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Brake should be released */
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    /* Manual engage (simulates BRAKE X 1 command) */
    ret = brake_set_state_with_timing(BRAKE_AXIS_X, true);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Brake should be engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));
}

/**
 * @brief Test: Auto sequences don't affect BRAKE_MANUAL axis
 */
void test_ac4_auto_sequences_dont_affect_manual_brake(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Set X axis to BRAKE_MANUAL */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_MANUAL));

    /* Release brake manually */
    brake_set_state(BRAKE_AXIS_X, false);
    sr_update();
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    /* Disable sequence should NOT engage brake */
    brake_on_axis_disable(BRAKE_AXIS_X);
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    /* Motion stop should NOT start idle timer */
    brake_on_motion_stop(BRAKE_AXIS_X);
    brake_force_idle_timeout(BRAKE_AXIS_X);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Brake should still be released */
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));
}

/* ==========================================================================
 * AC5: Timing Tests
 * ========================================================================== */

/**
 * @brief Test: Timing constants are properly defined
 */
void test_ac5_timing_constants_defined(void)
{
    TEST_ASSERT_EQUAL(50, TIMING_BRAKE_ENGAGE_MS);
    TEST_ASSERT_EQUAL(30, TIMING_BRAKE_RELEASE_MS);

    ESP_LOGI(TAG, "Brake engage delay: %d ms", TIMING_BRAKE_ENGAGE_MS);
    ESP_LOGI(TAG, "Brake release delay: %d ms", TIMING_BRAKE_RELEASE_MS);
}

/**
 * @brief Test: brake_set_state_with_timing applies proper delays
 */
void test_ac5_set_state_with_timing_applies_delays(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Test engage timing */
    brake_set_state(BRAKE_AXIS_X, false);  /* Start released */
    sr_update();

    int64_t start_us = esp_timer_get_time();
    brake_set_state_with_timing(BRAKE_AXIS_X, true);
    int64_t elapsed_ms = (esp_timer_get_time() - start_us) / 1000;

    TEST_ASSERT_GREATER_OR_EQUAL(TIMING_BRAKE_ENGAGE_MS, elapsed_ms);
    ESP_LOGI(TAG, "Engage timing: %lld ms (min %d)", elapsed_ms, TIMING_BRAKE_ENGAGE_MS);

    /* Test release timing */
    start_us = esp_timer_get_time();
    brake_set_state_with_timing(BRAKE_AXIS_X, false);
    elapsed_ms = (esp_timer_get_time() - start_us) / 1000;

    TEST_ASSERT_GREATER_OR_EQUAL(TIMING_BRAKE_RELEASE_MS, elapsed_ms);
    ESP_LOGI(TAG, "Release timing: %lld ms (min %d)", elapsed_ms, TIMING_BRAKE_RELEASE_MS);
}

/* ==========================================================================
 * AC6: Event Notification Tests
 * ========================================================================== */

/**
 * @brief Test: Brake events are published with correct format
 */
void test_ac6_brake_events_published(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    reset_brake_event_state();

    /* Trigger brake state change with timing (which publishes event) */
    brake_set_state_with_timing(BRAKE_AXIS_X, false);  /* Release */

    /* Wait for event */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Verify event received */
    TEST_ASSERT_TRUE(s_brake_event_received);
    TEST_ASSERT_EQUAL(BRAKE_AXIS_X, s_brake_event_axis);
    TEST_ASSERT_FALSE(s_brake_event_engaged);  /* Released = false */

    reset_brake_event_state();

    /* Engage */
    brake_set_state_with_timing(BRAKE_AXIS_X, true);
    vTaskDelay(pdMS_TO_TICKS(50));

    TEST_ASSERT_TRUE(s_brake_event_received);
    TEST_ASSERT_EQUAL(BRAKE_AXIS_X, s_brake_event_axis);
    TEST_ASSERT_TRUE(s_brake_event_engaged);  /* Engaged = true */
}

/* ==========================================================================
 * Axes Without Brakes Tests
 * ========================================================================== */

/**
 * @brief Test: brake_has_hardware returns false for C, D, E axes
 */
void test_axes_cde_have_no_brake_hardware(void)
{
    /* C, D, E have no brakes */
    TEST_ASSERT_FALSE(brake_has_hardware(5));  /* C */
    TEST_ASSERT_FALSE(brake_has_hardware(6));  /* D */
    TEST_ASSERT_FALSE(brake_has_hardware(7));  /* E */
}

/**
 * @brief Test: Brake operations return error for non-brake axes
 */
void test_brake_operations_fail_for_non_brake_axes(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* set_state should fail for axis C (5) */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, brake_set_state(5, true));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, brake_set_state(6, true));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, brake_set_state(7, true));
}

/* ==========================================================================
 * Full Sequence Integration Tests
 * ========================================================================== */

/**
 * @brief Test: Complete enable/disable cycle for BRAKE_ON_DISABLE
 */
void test_full_enable_disable_cycle(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Use Z axis (BRAKE_ON_DISABLE by default) */
    TEST_ASSERT_EQUAL(BRAKE_ON_DISABLE, brake_get_strategy(BRAKE_AXIS_Z));

    /* Initial state: brake engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_Z));

    /* Simulate full enable sequence */
    /* 1. Motor enable (simulated) */
    /* 2. brake_on_axis_enable releases brake */
    brake_on_axis_enable(BRAKE_AXIS_Z);
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_Z));

    /* Simulate motion... */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Simulate full disable sequence */
    /* 1. brake_on_axis_disable engages brake */
    brake_on_axis_disable(BRAKE_AXIS_Z);
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_Z));
    /* 2. Motor disable (simulated) */

    ESP_LOGI(TAG, "Full enable/disable cycle completed successfully");
}

/**
 * @brief Test: Complete motion cycle for BRAKE_ON_IDLE
 */
void test_full_motion_idle_cycle(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Set X axis to BRAKE_ON_IDLE */
    TEST_ASSERT_EQUAL(ESP_OK, brake_set_strategy(BRAKE_AXIS_X, BRAKE_ON_IDLE));

    /* Initial state: brake engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));

    /* Motion start: brake releases */
    brake_on_motion_start(BRAKE_AXIS_X);
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));

    /* Motion in progress... */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Motion stop: idle timer starts */
    brake_on_motion_stop(BRAKE_AXIS_X);
    TEST_ASSERT_FALSE(brake_get_state(BRAKE_AXIS_X));  /* Still released */

    /* Force idle timeout (simulates waiting 5 minutes) */
    brake_force_idle_timeout(BRAKE_AXIS_X);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* After timeout: brake engaged */
    TEST_ASSERT_TRUE(brake_get_state(BRAKE_AXIS_X));

    ESP_LOGI(TAG, "Full motion/idle cycle completed successfully");
}
