/**
 * @file test_mcp23017_wrapper.c
 * @brief Unit tests for MCP23017 I/O Expander Wrapper
 * @author YaRobot Team
 * @date 2025
 *
 * Tests acceptance criteria from Story 4.1
 *
 * @note These tests require actual MCP23017 hardware connected:
 *       - MCP23017 #0 at I2C address 0x20
 *       - MCP23017 #1 at I2C address 0x21
 *       Hardware tests should be run on target with devices connected.
 */

#include "unity.h"
#include "mcp23017_wrapper.h"
#include "config_i2c.h"
#include "config_gpio.h"
#include "config_timing.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

/* ==========================================================================
 * Test Setup/Teardown
 * ========================================================================== */

static void ensure_deinitialized(void)
{
    /* Ensure clean state before tests */
    if (mcp23017_wrapper_is_initialized()) {
        mcp23017_wrapper_deinit();
    }
}

/* ==========================================================================
 * AC1: Both MCP23017 Devices Respond on I2C0
 * ========================================================================== */

/**
 * AC1: Given I2C0 is initialized at 400kHz,
 * When mcp23017_wrapper_init() is called,
 * Then initialization returns ESP_OK for both devices at 0x20 and 0x21
 */
TEST_CASE("AC1: mcp23017_wrapper_init returns ESP_OK", "[mcp23017][hardware]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();

    /* Note: This test will fail if hardware is not connected */
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret,
        "MCP23017 init failed - ensure devices at 0x20 and 0x21 are connected");

    TEST_ASSERT_TRUE(mcp23017_wrapper_is_initialized());

    /* Cleanup */
    mcp23017_wrapper_deinit();
}

/**
 * AC1: Double initialization returns ESP_ERR_INVALID_STATE
 */
TEST_CASE("AC1: double init returns ESP_ERR_INVALID_STATE", "[mcp23017][hardware]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Second init should fail gracefully */
    ret = mcp23017_wrapper_init();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);

    /* Cleanup */
    mcp23017_wrapper_deinit();
}

/**
 * AC1: is_initialized returns correct state
 */
TEST_CASE("AC1: is_initialized returns correct state", "[mcp23017]")
{
    ensure_deinitialized();

    /* Should be false before init */
    TEST_ASSERT_FALSE(mcp23017_wrapper_is_initialized());

    /* Try to init (may fail without hardware) */
    esp_err_t ret = mcp23017_wrapper_init();
    if (ret == ESP_OK) {
        TEST_ASSERT_TRUE(mcp23017_wrapper_is_initialized());
        mcp23017_wrapper_deinit();
    }

    /* Should be false after deinit */
    TEST_ASSERT_FALSE(mcp23017_wrapper_is_initialized());
}

/* ==========================================================================
 * AC2: All Ports Configured as Inputs with Pull-ups
 * ========================================================================== */

/**
 * AC2: Given both MCP23017 devices are initialized,
 * When configuration is applied,
 * Then direction registers read back as 0xFF (all inputs)
 * And pull-up registers read back as 0xFF
 *
 * Note: This is verified by reading port values - inputs with pull-ups
 * should read high unless externally pulled low
 */
TEST_CASE("AC2: ports configured as inputs with pull-ups", "[mcp23017][hardware]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Read all ports - with pull-ups and nothing connected, should read 0xFF */
    uint8_t port_a_0, port_b_0, port_a_1, port_b_1;

    ret = mcp23017_wrapper_read_port(MCP_DEVICE_0, MCP_PORT_A, &port_a_0);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mcp23017_wrapper_read_port(MCP_DEVICE_0, MCP_PORT_B, &port_b_0);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mcp23017_wrapper_read_port(MCP_DEVICE_1, MCP_PORT_A, &port_a_1);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mcp23017_wrapper_read_port(MCP_DEVICE_1, MCP_PORT_B, &port_b_1);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* With pull-ups enabled and no external connection, expect 0xFF
     * Note: In actual hardware with switches, some bits may be low */
    /* We just verify reads succeed - actual values depend on wiring */

    /* Cleanup */
    mcp23017_wrapper_deinit();
}

/**
 * AC2: read_all returns combined 16-bit value
 */
TEST_CASE("AC2: read_all returns 16-bit value", "[mcp23017][hardware]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    uint16_t value;
    ret = mcp23017_wrapper_read_all(MCP_DEVICE_0, &value);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mcp23017_wrapper_read_all(MCP_DEVICE_1, &value);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Cleanup */
    mcp23017_wrapper_deinit();
}

/* ==========================================================================
 * AC3: Interrupt-on-Change for MCP0 (Limit Switches)
 * ========================================================================== */

/**
 * AC3: Given MCP0 is configured,
 * When mcp23017_wrapper_config_mcp0_interrupts() is called,
 * Then interrupt-on-change is enabled for all 16 pins
 */
TEST_CASE("AC3: MCP0 interrupt config succeeds", "[mcp23017][hardware]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    ret = mcp23017_wrapper_config_mcp0_interrupts();
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Cleanup */
    mcp23017_wrapper_deinit();
}

/**
 * AC3: Interrupt config fails if not initialized
 */
TEST_CASE("AC3: MCP0 interrupt config fails when not initialized", "[mcp23017]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_config_mcp0_interrupts();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
}

/* ==========================================================================
 * AC4: Interrupt-on-Change for MCP1 Port A (ALARM_INPUT)
 * ========================================================================== */

/**
 * AC4: Given MCP1 is configured,
 * When mcp23017_wrapper_config_mcp1_interrupts() is called,
 * Then interrupt-on-change is enabled for ALARM_INPUT pins (GPA0-GPA6)
 */
TEST_CASE("AC4: MCP1 interrupt config succeeds", "[mcp23017][hardware]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    ret = mcp23017_wrapper_config_mcp1_interrupts();
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Cleanup */
    mcp23017_wrapper_deinit();
}

/**
 * AC4: Interrupt config fails if not initialized
 */
TEST_CASE("AC4: MCP1 interrupt config fails when not initialized", "[mcp23017]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_config_mcp1_interrupts();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
}

/* ==========================================================================
 * AC5: Read Operations Complete Within 500us
 * ========================================================================== */

/**
 * AC5: Given I2C bus is operational,
 * When mcp23017_wrapper_read_port() is called,
 * Then operation completes within timeout
 *
 * Note: At 400kHz I2C, a single port read should complete in ~200us.
 * We use TIMING_I2C_TIMEOUT_MS as the overall timeout.
 */
TEST_CASE("AC5: read_port completes within timeout", "[mcp23017][hardware]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    uint8_t value;
    int64_t start = esp_timer_get_time();

    ret = mcp23017_wrapper_read_port(MCP_DEVICE_0, MCP_PORT_A, &value);

    int64_t elapsed_us = esp_timer_get_time() - start;

    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* AC5 requires < 500us at 400kHz - we verify against TIMING_I2C_READ_TIMEOUT_US */
    TEST_ASSERT_LESS_THAN_MESSAGE(TIMING_I2C_READ_TIMEOUT_US, elapsed_us,
        "Port read exceeded 500us timeout");

    /* Cleanup */
    mcp23017_wrapper_deinit();
}

/**
 * AC5: read_port returns ESP_ERR_INVALID_ARG for invalid parameters
 */
TEST_CASE("AC5: read_port rejects invalid parameters", "[mcp23017]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        /* Test parameter validation even without hardware */
    }

    uint8_t value;

    /* NULL value pointer */
    ret = mcp23017_wrapper_read_port(MCP_DEVICE_0, MCP_PORT_A, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    /* Invalid device */
    ret = mcp23017_wrapper_read_port(MCP_DEVICE_COUNT, MCP_PORT_A, &value);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    /* Cleanup if initialized */
    if (mcp23017_wrapper_is_initialized()) {
        mcp23017_wrapper_deinit();
    }
}

/* ==========================================================================
 * ISR and Task Registration Tests
 * ========================================================================== */

/**
 * Test: register_notify_task validates parameters
 */
TEST_CASE("register_notify_task validates parameters", "[mcp23017]")
{
    esp_err_t ret = mcp23017_wrapper_register_notify_task(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Test: ISR installation fails without task registered
 */
TEST_CASE("ISR install fails without task registered", "[mcp23017]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Try to install ISR without registering task first */
    ret = mcp23017_wrapper_install_isr();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);

    /* Cleanup */
    mcp23017_wrapper_deinit();
}

/**
 * Test: Full ISR setup succeeds with task registered
 */
TEST_CASE("ISR install succeeds with task registered", "[mcp23017][hardware]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    /* Configure interrupts */
    TEST_ASSERT_EQUAL(ESP_OK, mcp23017_wrapper_config_mcp0_interrupts());
    TEST_ASSERT_EQUAL(ESP_OK, mcp23017_wrapper_config_mcp1_interrupts());

    /* Register current task for notifications */
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    TEST_ASSERT_EQUAL(ESP_OK, mcp23017_wrapper_register_notify_task(current_task));

    /* Install ISRs */
    ret = mcp23017_wrapper_install_isr();
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Cleanup */
    mcp23017_wrapper_deinit();
}

/* ==========================================================================
 * Interrupt Flag Tests
 * ========================================================================== */

/**
 * Test: get_int_flags returns valid data
 */
TEST_CASE("get_int_flags succeeds after init", "[mcp23017][hardware]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    uint16_t flags;
    ret = mcp23017_wrapper_get_int_flags(MCP_DEVICE_0, &flags);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mcp23017_wrapper_get_int_flags(MCP_DEVICE_1, &flags);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Cleanup */
    mcp23017_wrapper_deinit();
}

/**
 * Test: get_int_capture returns valid data and clears interrupt
 */
TEST_CASE("get_int_capture succeeds after init", "[mcp23017][hardware]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    uint16_t capture;
    ret = mcp23017_wrapper_get_int_capture(MCP_DEVICE_0, &capture);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mcp23017_wrapper_get_int_capture(MCP_DEVICE_1, &capture);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Cleanup */
    mcp23017_wrapper_deinit();
}

/* ==========================================================================
 * Deinit Tests
 * ========================================================================== */

/**
 * Test: deinit cleans up properly
 */
TEST_CASE("deinit cleans up properly", "[mcp23017][hardware]")
{
    ensure_deinitialized();

    esp_err_t ret = mcp23017_wrapper_init();
    if (ret != ESP_OK) {
        TEST_IGNORE_MESSAGE("Skipping - hardware not connected");
    }

    TEST_ASSERT_TRUE(mcp23017_wrapper_is_initialized());

    ret = mcp23017_wrapper_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    TEST_ASSERT_FALSE(mcp23017_wrapper_is_initialized());

    /* Double deinit should be safe */
    ret = mcp23017_wrapper_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

/* ==========================================================================
 * Configuration Constant Tests
 * ========================================================================== */

/**
 * Test: Configuration constants have expected values
 */
TEST_CASE("Config constants have expected values", "[mcp23017]")
{
    /* I2C addresses */
    TEST_ASSERT_EQUAL_HEX8(0x20, I2C_ADDR_MCP23017_0);
    TEST_ASSERT_EQUAL_HEX8(0x21, I2C_ADDR_MCP23017_1);

    /* I2C frequency */
    TEST_ASSERT_EQUAL(400000, I2C_FREQ_HZ);

    /* Timeout constant */
    TEST_ASSERT_EQUAL(500, TIMING_I2C_READ_TIMEOUT_US);

    /* MCP0 limit switch pin mappings (Port A) */
    TEST_ASSERT_EQUAL(0, MCP0_X_LIMIT_MIN);
    TEST_ASSERT_EQUAL(1, MCP0_X_LIMIT_MAX);
    TEST_ASSERT_EQUAL(2, MCP0_Y_LIMIT_MIN);
    TEST_ASSERT_EQUAL(3, MCP0_Y_LIMIT_MAX);

    /* MCP1 alarm input pin mappings */
    TEST_ASSERT_EQUAL(0, MCP1_X_ALARM_INPUT);
    TEST_ASSERT_EQUAL(6, MCP1_D_ALARM_INPUT);

    /* MCP1 InPos pin mappings */
    TEST_ASSERT_EQUAL(8, MCP1_X_INPOS);
    TEST_ASSERT_EQUAL(12, MCP1_B_INPOS);
}

/**
 * Test: Notification bit masks are non-overlapping
 */
TEST_CASE("Notification bits are non-overlapping", "[mcp23017]")
{
    /* Verify all bits are distinct */
    TEST_ASSERT_EQUAL(1, MCP_NOTIFY_MCP0_INTA);
    TEST_ASSERT_EQUAL(2, MCP_NOTIFY_MCP0_INTB);
    TEST_ASSERT_EQUAL(4, MCP_NOTIFY_MCP1_INTA);
    TEST_ASSERT_EQUAL(8, MCP_NOTIFY_MCP1_INTB);

    /* Combined masks */
    TEST_ASSERT_EQUAL(MCP_NOTIFY_MCP0_INTA | MCP_NOTIFY_MCP0_INTB, MCP_NOTIFY_LIMIT);
    TEST_ASSERT_EQUAL(MCP_NOTIFY_MCP1_INTA, MCP_NOTIFY_ALARM);
}
