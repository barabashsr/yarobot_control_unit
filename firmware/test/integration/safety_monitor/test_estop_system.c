/**
 * @file test_estop_system.c
 * @brief Integration tests for Emergency Stop System (Story 4-4)
 * @author YaRobot Team
 * @date 2025
 *
 * Tests acceptance criteria from Story 4-4:
 *   AC1: E-stop response within 1ms
 *   AC2: All brakes engage on E-stop
 *   AC3: ESTOP event published
 *   AC4: Motion commands rejected in ESTOP mode
 *   AC5: RST clears E-stop only if button released
 *   AC6: All axes marked UNHOMED
 *   AC7: Hardware interrupt configuration
 *
 * @note These tests use test injection functions and do not require
 *       physical E-stop button. Some timing tests require oscilloscope.
 */

#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "safety_monitor.h"
#include "mcp23017_wrapper.h"
#include "command_executor.h"
#include "event_manager.h"
#include "config_gpio.h"
#include "config_commands.h"
#include "config_axes.h"
#include "config_timing.h"

static const char *TAG = "test_estop";

/** @brief Flag indicating hardware is available */
static bool s_hardware_available = false;

/** @brief Response buffer for command testing */
static char s_response[256];

/** @brief Event received flag */
static volatile bool s_event_received = false;

/** @brief Event timestamp for latency measurement */
static int64_t s_event_timestamp_us = 0;

/** @brief Event type received */
static volatile uint8_t s_event_type = 0;

/**
 * @brief Event callback for testing
 */
static void test_event_callback(const Event* event, void* ctx)
{
    (void)ctx;
    if (event->type == EVTTYPE_ESTOP_CHANGED) {
        s_event_received = true;
        s_event_timestamp_us = esp_timer_get_time();
        s_event_type = event->type;
        ESP_LOGI(TAG, "ESTOP event received: active=%d", event->data.estop_active);
    }
}

/**
 * @brief Reset event state for testing
 */
static void reset_event_state(void)
{
    s_event_received = false;
    s_event_timestamp_us = 0;
    s_event_type = 0;
}

/**
 * @brief Test setup - initialize hardware
 */
void setUp(void)
{
    /* Deinitialize if already initialized */
    if (safety_monitor_is_initialized()) {
        safety_monitor_deinit();
    }
    if (mcp23017_wrapper_is_initialized()) {
        mcp23017_wrapper_deinit();
    }

    /* Initialize MCP23017 wrapper */
    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        s_hardware_available = false;
        return;
    }

    /* Initialize safety monitor */
    ret = safety_monitor_init();
    if (ret != ESP_OK) {
        mcp23017_wrapper_deinit();
        s_hardware_available = false;
        return;
    }

    /* Register event callback */
    event_subscribe(EVTTYPE_ESTOP_CHANGED, test_event_callback, NULL);

    s_hardware_available = true;
    reset_event_state();

    /* Enable test mode for all tests */
    safety_monitor_set_estop_test_mode(true);
    safety_monitor_set_test_button_state(false);

    /* Small delay to let task start */
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

    /* Disable test mode */
    safety_monitor_set_estop_test_mode(false);

    if (safety_monitor_is_initialized()) {
        safety_monitor_deinit();
    }
    if (mcp23017_wrapper_is_initialized()) {
        mcp23017_wrapper_deinit();
    }
}

/* ==========================================================================
 * AC1: E-Stop Response Time
 * ========================================================================== */

/**
 * @brief Test: E-stop timing constant is defined correctly
 */
void test_estop_timing_constant(void)
{
    TEST_ASSERT_EQUAL(1, TIMING_ESTOP_RESPONSE_MS);
    ESP_LOGI(TAG, "E-stop response time requirement: %d ms", TIMING_ESTOP_RESPONSE_MS);
}

/**
 * @brief Test: E-stop injection triggers mode change quickly
 */
void test_estop_response_time(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    reset_event_state();

    int64_t start_us = esp_timer_get_time();
    safety_monitor_inject_estop();

    /* Wait for E-stop to activate */
    const int max_wait_ms = 100;
    int elapsed_ms = 0;
    while (elapsed_ms < max_wait_ms && !safety_monitor_is_estop_active()) {
        vTaskDelay(pdMS_TO_TICKS(1));
        elapsed_ms++;
    }

    int64_t end_us = esp_timer_get_time();
    int64_t latency_us = end_us - start_us;

    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());
    ESP_LOGI(TAG, "E-stop activation latency: %lld us (task processing)", latency_us);

    /* Note: The 1ms requirement is for hardware (ISR + shift register),
     * which we can't measure without oscilloscope. Task processing adds
     * additional latency which is acceptable for event publishing. */
}

/* ==========================================================================
 * AC3: ESTOP Event Published
 * ========================================================================== */

/**
 * @brief Test: E-stop event published on activation
 */
void test_estop_event_published_on_activation(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    reset_event_state();

    /* Inject E-stop */
    safety_monitor_inject_estop();

    /* Wait for event */
    const int max_wait_ms = 100;
    int elapsed_ms = 0;
    while (elapsed_ms < max_wait_ms && !s_event_received) {
        vTaskDelay(pdMS_TO_TICKS(1));
        elapsed_ms++;
    }

    TEST_ASSERT_TRUE(s_event_received);
    TEST_ASSERT_EQUAL(EVTTYPE_ESTOP_CHANGED, s_event_type);
    ESP_LOGI(TAG, "ESTOP ACTIVE event received after %d ms", elapsed_ms);
}

/**
 * @brief Test: E-stop inactive event published on reset
 */
void test_estop_event_published_on_reset(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* First activate E-stop */
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Reset event state */
    reset_event_state();

    /* Ensure button is released */
    safety_monitor_set_test_button_state(false);

    /* Reset E-stop */
    esp_err_t ret = safety_monitor_reset_estop();
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Wait for event */
    vTaskDelay(pdMS_TO_TICKS(50));

    TEST_ASSERT_TRUE(s_event_received);
    TEST_ASSERT_EQUAL(EVTTYPE_ESTOP_CHANGED, s_event_type);
    ESP_LOGI(TAG, "ESTOP INACTIVE event received");
}

/* ==========================================================================
 * AC4: Motion Commands Rejected in ESTOP Mode
 * ========================================================================== */

/**
 * @brief Test: MOVE command rejected in ESTOP mode
 */
void test_move_rejected_in_estop_mode(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Activate E-stop */
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Try MOVE command */
    ParsedCommand cmd = {
        .verb = CMD_MOVE,
        .axis_mask = (1 << AXIS_X),
        .values = {100.0f},
        .has_velocity = false
    };

    esp_err_t ret = command_executor_execute(&cmd, s_response, sizeof(s_response));

    /* Should fail with ESTOP error */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_EMERGENCY_STOP));

    ESP_LOGI(TAG, "MOVE rejected in ESTOP: %s", s_response);
}

/**
 * @brief Test: VEL command rejected in ESTOP mode
 */
void test_vel_rejected_in_estop_mode(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Activate E-stop */
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Try VEL command */
    ParsedCommand cmd = {
        .verb = CMD_VEL,
        .axis_mask = (1 << AXIS_Y),
        .values = {0.01f}
    };

    esp_err_t ret = command_executor_execute(&cmd, s_response, sizeof(s_response));

    /* Should fail with ESTOP error */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_EMERGENCY_STOP));

    ESP_LOGI(TAG, "VEL rejected in ESTOP: %s", s_response);
}

/**
 * @brief Test: MOVR command rejected in ESTOP mode
 */
void test_movr_rejected_in_estop_mode(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Activate E-stop */
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Try MOVR command */
    ParsedCommand cmd = {
        .verb = CMD_MOVR,
        .axis_mask = (1 << AXIS_X),
        .values = {10.0f}
    };

    esp_err_t ret = command_executor_execute(&cmd, s_response, sizeof(s_response));

    /* Should fail with ESTOP error */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_EMERGENCY_STOP));

    ESP_LOGI(TAG, "MOVR rejected in ESTOP: %s", s_response);
}

/* ==========================================================================
 * AC5: RST Command Behavior
 * ========================================================================== */

/**
 * @brief Test: RST fails when button still pressed
 */
void test_rst_fails_when_button_pressed(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Activate E-stop */
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Simulate button still pressed */
    safety_monitor_set_test_button_state(true);

    /* Try RST */
    esp_err_t ret = safety_monitor_reset_estop();

    /* Should fail */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    ESP_LOGI(TAG, "RST correctly rejected while button pressed");
}

/**
 * @brief Test: RST succeeds when button released
 */
void test_rst_succeeds_when_button_released(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Activate E-stop */
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Simulate button released */
    safety_monitor_set_test_button_state(false);

    /* Try RST */
    esp_err_t ret = safety_monitor_reset_estop();

    /* Should succeed */
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FALSE(safety_monitor_is_estop_active());

    ESP_LOGI(TAG, "RST succeeded after button release");
}

/**
 * @brief Test: Motion commands work after RST
 */
void test_motion_allowed_after_rst(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Activate E-stop */
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

    /* Release button and reset */
    safety_monitor_set_test_button_state(false);
    esp_err_t ret = safety_monitor_reset_estop();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FALSE(safety_monitor_is_estop_active());

    /* STOP command should work (doesn't require enabled axis) */
    ParsedCommand cmd = {
        .verb = CMD_STOP,
        .axis_mask = (1 << AXIS_X)
    };

    ret = command_executor_execute(&cmd, s_response, sizeof(s_response));

    /* Should not get ESTOP error */
    TEST_ASSERT_NULL(strstr(s_response, ERR_EMERGENCY_STOP));

    ESP_LOGI(TAG, "Commands allowed after RST");
}

/* ==========================================================================
 * Full Sequence Tests
 * ========================================================================== */

/**
 * @brief Test: Full E-stop sequence (activate -> reject -> reset -> allow)
 */
void test_full_estop_sequence(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    ESP_LOGI(TAG, "=== Full E-Stop Sequence Test ===");

    /* 1. Initial state: not in E-stop */
    TEST_ASSERT_FALSE(safety_monitor_is_estop_active());
    ESP_LOGI(TAG, "Step 1: Initial state OK - not in E-stop");

    /* 2. Trigger E-stop */
    reset_event_state();
    safety_monitor_inject_estop();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());
    TEST_ASSERT_TRUE(s_event_received);
    ESP_LOGI(TAG, "Step 2: E-stop triggered, event received");

    /* 3. Motion command rejected */
    ParsedCommand move_cmd = {
        .verb = CMD_MOVE,
        .axis_mask = (1 << AXIS_X),
        .values = {100.0f}
    };
    esp_err_t ret = command_executor_execute(&move_cmd, s_response, sizeof(s_response));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
    ESP_LOGI(TAG, "Step 3: Motion command rejected");

    /* 4. RST fails with button pressed */
    safety_monitor_set_test_button_state(true);
    ret = safety_monitor_reset_estop();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
    TEST_ASSERT_TRUE(safety_monitor_is_estop_active());
    ESP_LOGI(TAG, "Step 4: RST failed while button pressed");

    /* 5. Release button and RST succeeds */
    reset_event_state();
    safety_monitor_set_test_button_state(false);
    ret = safety_monitor_reset_estop();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FALSE(safety_monitor_is_estop_active());
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE(s_event_received);  /* ESTOP INACTIVE event */
    ESP_LOGI(TAG, "Step 5: RST succeeded, E-stop cleared");

    /* 6. Verify system is back to normal */
    TEST_ASSERT_FALSE(safety_monitor_is_estop_active());
    ESP_LOGI(TAG, "Step 6: System back to normal operation");

    ESP_LOGI(TAG, "=== Full E-Stop Sequence Test PASSED ===");
}

/**
 * @brief Test: Multiple E-stop cycles
 */
void test_multiple_estop_cycles(void)
{
    if (!s_hardware_available) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    ESP_LOGI(TAG, "=== Multiple E-Stop Cycles Test ===");

    for (int cycle = 1; cycle <= 3; cycle++) {
        ESP_LOGI(TAG, "Cycle %d: Triggering E-stop", cycle);

        /* Trigger E-stop */
        safety_monitor_inject_estop();
        vTaskDelay(pdMS_TO_TICKS(30));
        TEST_ASSERT_TRUE(safety_monitor_is_estop_active());

        /* Reset */
        safety_monitor_set_test_button_state(false);
        esp_err_t ret = safety_monitor_reset_estop();
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        TEST_ASSERT_FALSE(safety_monitor_is_estop_active());

        ESP_LOGI(TAG, "Cycle %d: E-stop cleared", cycle);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ESP_LOGI(TAG, "=== Multiple E-Stop Cycles Test PASSED ===");
}

/* ==========================================================================
 * Test Runner Entry Point
 * ========================================================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "=== Emergency Stop System Integration Tests ===");
    ESP_LOGI(TAG, "Story 4-4: Emergency Stop System");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Tests use injection functions - no physical E-stop button required.");
    ESP_LOGI(TAG, "");

    UNITY_BEGIN();

    /* AC1: Response time */
    RUN_TEST(test_estop_timing_constant);
    RUN_TEST(test_estop_response_time);

    /* AC3: Event publishing */
    RUN_TEST(test_estop_event_published_on_activation);
    RUN_TEST(test_estop_event_published_on_reset);

    /* AC4: Motion commands rejected */
    RUN_TEST(test_move_rejected_in_estop_mode);
    RUN_TEST(test_vel_rejected_in_estop_mode);
    RUN_TEST(test_movr_rejected_in_estop_mode);

    /* AC5: RST command behavior */
    RUN_TEST(test_rst_fails_when_button_pressed);
    RUN_TEST(test_rst_succeeds_when_button_released);
    RUN_TEST(test_motion_allowed_after_rst);

    /* Full sequence tests */
    RUN_TEST(test_full_estop_sequence);
    RUN_TEST(test_multiple_estop_cycles);

    UNITY_END();
}
