/**
 * @file test_tpic6b595.c
 * @brief Unit tests for TPIC6B595N shift register driver
 * @author YaRobot Team
 * @date 2025
 *
 * Tests all acceptance criteria from Story 3.1
 */

#include "unity.h"
#include "tpic6b595.h"
#include "config_sr.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

/* ==========================================================================
 * AC1: sr_init() returns ESP_OK
 * ========================================================================== */

/**
 * AC1: Given SPI2 is initialized, when sr_init() is called,
 * then it returns ESP_OK and the 40-bit shift register chain is ready
 */
TEST_CASE("AC1: sr_init returns ESP_OK", "[tpic6b595]")
{
    // Note: may already be initialized from app startup
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());
    TEST_ASSERT_TRUE(sr_is_initialized());
}

/**
 * Test: Double initialization is safe
 */
TEST_CASE("AC1: double init returns ESP_OK", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());
    TEST_ASSERT_TRUE(sr_is_initialized());
}

/* ==========================================================================
 * AC2: sr_set_direction sets correct bit
 * ========================================================================== */

/**
 * AC2: Given the shift register is initialized, when sr_set_direction(axis, forward)
 * is called, then the corresponding SR_x_DIR bit is set/cleared in shadow register
 */
TEST_CASE("AC2: sr_set_direction sets correct bits", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());

    // Test all 8 axes
    for (uint8_t axis = 0; axis < 8; axis++) {
        // Set direction forward
        TEST_ASSERT_EQUAL(ESP_OK, sr_set_direction(axis, true));
        uint64_t state = sr_get_state();
        TEST_ASSERT_TRUE((state >> SR_DIR_BIT(axis)) & 1);

        // Set direction reverse
        TEST_ASSERT_EQUAL(ESP_OK, sr_set_direction(axis, false));
        state = sr_get_state();
        TEST_ASSERT_FALSE((state >> SR_DIR_BIT(axis)) & 1);
    }
}

/**
 * Test: Invalid axis returns error
 */
TEST_CASE("AC2: sr_set_direction rejects invalid axis", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sr_set_direction(8, true));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sr_set_direction(255, true));
}

/* ==========================================================================
 * AC3: sr_set_enable sets correct bit
 * ========================================================================== */

/**
 * AC3: Given the shift register is initialized, when sr_set_enable(axis, enable)
 * is called, then the corresponding SR_x_EN bit is set/cleared in shadow register
 */
TEST_CASE("AC3: sr_set_enable sets correct bits", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());

    // Test all 8 axes
    for (uint8_t axis = 0; axis < 8; axis++) {
        // Enable axis
        TEST_ASSERT_EQUAL(ESP_OK, sr_set_enable(axis, true));
        uint64_t state = sr_get_state();
        TEST_ASSERT_TRUE((state >> SR_EN_BIT(axis)) & 1);

        // Disable axis
        TEST_ASSERT_EQUAL(ESP_OK, sr_set_enable(axis, false));
        state = sr_get_state();
        TEST_ASSERT_FALSE((state >> SR_EN_BIT(axis)) & 1);
    }
}

/**
 * Test: Invalid axis returns error
 */
TEST_CASE("AC3: sr_set_enable rejects invalid axis", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sr_set_enable(8, true));
}

/* ==========================================================================
 * AC4: sr_set_brake sets correct bit (axes X-D only)
 * ========================================================================== */

/**
 * AC4: Given the shift register is initialized, when sr_set_brake(axis, release)
 * is called, then the corresponding SR_x_BRAKE bit is set/cleared (axes X-D only)
 */
TEST_CASE("AC4: sr_set_brake sets correct bits for X-D", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());

    // Test axes 0-6 (X-D have brakes)
    for (uint8_t axis = 0; axis <= 6; axis++) {
        // Release brake
        TEST_ASSERT_EQUAL(ESP_OK, sr_set_brake(axis, true));
        uint64_t state = sr_get_state();
        TEST_ASSERT_TRUE((state >> SR_BRAKE_BIT(axis)) & 1);

        // Engage brake
        TEST_ASSERT_EQUAL(ESP_OK, sr_set_brake(axis, false));
        state = sr_get_state();
        TEST_ASSERT_FALSE((state >> SR_BRAKE_BIT(axis)) & 1);
    }
}

/**
 * Test: E-axis (7) has no brake - should return error
 */
TEST_CASE("AC4: sr_set_brake rejects E-axis", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sr_set_brake(7, true));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sr_set_brake(8, true));
}

/* ==========================================================================
 * AC5: sr_set_alarm_clear sets correct bit (axes X-D only)
 * ========================================================================== */

/**
 * AC5: Given the shift register is initialized, when sr_set_alarm_clear(axis, active)
 * is called, then the corresponding SR_x_ALARM_CLR bit is set/cleared (axes X-D only)
 */
TEST_CASE("AC5: sr_set_alarm_clear sets correct bits for X-D", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());

    // Test axes 0-6 (X-D have alarm clear)
    for (uint8_t axis = 0; axis <= 6; axis++) {
        // Assert alarm clear
        TEST_ASSERT_EQUAL(ESP_OK, sr_set_alarm_clear(axis, true));
        uint64_t state = sr_get_state();
        TEST_ASSERT_TRUE((state >> SR_ALARM_CLR_BIT(axis)) & 1);

        // Deassert alarm clear
        TEST_ASSERT_EQUAL(ESP_OK, sr_set_alarm_clear(axis, false));
        state = sr_get_state();
        TEST_ASSERT_FALSE((state >> SR_ALARM_CLR_BIT(axis)) & 1);
    }
}

/**
 * Test: E-axis (7) has no alarm clear - should return error
 */
TEST_CASE("AC5: sr_set_alarm_clear rejects E-axis", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sr_set_alarm_clear(7, true));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sr_set_alarm_clear(8, true));
}

/* ==========================================================================
 * AC6: sr_set_gp_output sets correct bit
 * ========================================================================== */

/**
 * AC6: Given the shift register is initialized, when sr_set_gp_output(pin, level)
 * is called, then the corresponding SR_GP_OUT_n bit (0-7) is set/cleared
 */
TEST_CASE("AC6: sr_set_gp_output sets correct bits", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());

    // Test all 8 GP outputs (bits 32-39)
    for (uint8_t pin = 0; pin < 8; pin++) {
        // Set HIGH
        TEST_ASSERT_EQUAL(ESP_OK, sr_set_gp_output(pin, true));
        uint64_t state = sr_get_state();
        TEST_ASSERT_TRUE((state >> (SR_GP_OUT_0 + pin)) & 1);

        // Set LOW
        TEST_ASSERT_EQUAL(ESP_OK, sr_set_gp_output(pin, false));
        state = sr_get_state();
        TEST_ASSERT_FALSE((state >> (SR_GP_OUT_0 + pin)) & 1);
    }
}

/**
 * Test: Invalid pin returns error
 */
TEST_CASE("AC6: sr_set_gp_output rejects invalid pin", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sr_set_gp_output(8, true));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sr_set_gp_output(255, true));
}

/* ==========================================================================
 * AC7: sr_update shifts all 40 bits via SPI
 * ========================================================================== */

/**
 * AC7: Given bit changes have been made, when sr_update() is called,
 * then all 40 bits are shifted out via SPI and latched to outputs
 */
TEST_CASE("AC7: sr_update returns ESP_OK", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());

    // Set some bits
    TEST_ASSERT_EQUAL(ESP_OK, sr_set_enable(0, true));
    TEST_ASSERT_EQUAL(ESP_OK, sr_set_direction(1, true));
    TEST_ASSERT_EQUAL(ESP_OK, sr_set_gp_output(7, true));

    // Update should succeed
    TEST_ASSERT_EQUAL(ESP_OK, sr_update());
}

/* ==========================================================================
 * AC8: sr_get_state returns shadow register
 * ========================================================================== */

/**
 * AC8: Given I call sr_get_state(), when the function returns,
 * then it provides the current 40-bit shadow register state as uint64_t
 */
TEST_CASE("AC8: sr_get_state returns expected value", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());

    // After emergency disable, state should be safe (all zeros)
    sr_emergency_disable_all();
    TEST_ASSERT_EQUAL_UINT64(SR_SAFE_STATE, sr_get_state());

    // Re-init to restore normal operation
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());

    // Set specific bits and verify
    sr_set_enable(0, true);       // Bit 1
    sr_set_direction(1, true);    // Bit 4

    uint64_t state = sr_get_state();
    TEST_ASSERT_TRUE((state >> SR_EN_BIT(0)) & 1);
    TEST_ASSERT_TRUE((state >> SR_DIR_BIT(1)) & 1);
}

/* ==========================================================================
 * AC9: sr_emergency_disable_all sets safe state
 * ========================================================================== */

/**
 * AC9: Given an emergency condition, when sr_emergency_disable_all() is called,
 * then all 40 bits are set to 0 (SR_SAFE_STATE), GPIO_SR_OE goes HIGH
 */
TEST_CASE("AC9: sr_emergency_disable_all sets safe state", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());

    // Set some bits first
    sr_set_enable(0, true);
    sr_set_enable(1, true);
    sr_set_brake(0, true);
    sr_set_gp_output(0, true);
    sr_update();

    // Verify bits are set
    TEST_ASSERT_NOT_EQUAL(SR_SAFE_STATE, sr_get_state());

    // Emergency disable
    sr_emergency_disable_all();

    // Verify safe state
    TEST_ASSERT_EQUAL_UINT64(SR_SAFE_STATE, sr_get_state());

    // Re-init for other tests
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());
}

/**
 * Test: sr_emergency_disable_all executes within 100us
 */
TEST_CASE("AC9: sr_emergency_disable_all completes within 100us", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());

    // Set some bits
    sr_set_enable(0, true);
    sr_update();

    // Measure timing
    int64_t start = esp_timer_get_time();
    sr_emergency_disable_all();
    int64_t elapsed = esp_timer_get_time() - start;

    // Should complete in less than 100us
    TEST_ASSERT_LESS_THAN(100, elapsed);

    // Re-init for other tests
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());
}

/* ==========================================================================
 * AC10: Thread safety - not easily testable in unit tests
 * ========================================================================== */

/**
 * AC10: Given multiple tasks call sr_set_* functions concurrently,
 * when updates occur, then thread safety is maintained via mutex
 *
 * Note: Full thread safety testing requires integration tests with multiple tasks.
 * Here we just verify the functions work in sequence.
 */
TEST_CASE("AC10: sequential operations work correctly", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL(ESP_OK, sr_init());

    // Rapid sequential operations
    for (int i = 0; i < 10; i++) {
        sr_set_direction(i % 8, (i & 1));
        sr_set_enable(i % 8, (i & 1));
        if ((i % 8) < 7) {
            sr_set_brake(i % 8, (i & 1));
        }
        sr_update();
    }

    // Should not crash or corrupt state
    uint64_t state = sr_get_state();
    (void)state;  // Just verify no crash
}

/* ==========================================================================
 * Bit Position Macro Tests
 * ========================================================================== */

/**
 * Test: Verify bit position macros match expected values
 */
TEST_CASE("Bit position macros are correct", "[tpic6b595]")
{
    // X-axis (bits 0-3)
    TEST_ASSERT_EQUAL(0, SR_DIR_BIT(0));
    TEST_ASSERT_EQUAL(1, SR_EN_BIT(0));
    TEST_ASSERT_EQUAL(2, SR_BRAKE_BIT(0));
    TEST_ASSERT_EQUAL(3, SR_ALARM_CLR_BIT(0));

    // Y-axis (bits 4-7)
    TEST_ASSERT_EQUAL(4, SR_DIR_BIT(1));
    TEST_ASSERT_EQUAL(5, SR_EN_BIT(1));

    // E-axis (bits 28-31)
    TEST_ASSERT_EQUAL(28, SR_DIR_BIT(7));
    TEST_ASSERT_EQUAL(29, SR_EN_BIT(7));

    // GP outputs (bits 32-39)
    TEST_ASSERT_EQUAL(32, SR_GP_OUT_0);
    TEST_ASSERT_EQUAL(39, SR_GP_OUT_7);
}

/**
 * Test: Verify SR_SET_BIT and SR_CLR_BIT macros work correctly
 */
TEST_CASE("Bit manipulation macros work correctly", "[tpic6b595]")
{
    uint64_t data = 0;

    // Set bit
    data = SR_SET_BIT(data, 0);
    TEST_ASSERT_EQUAL_UINT64(1, data);

    data = SR_SET_BIT(data, 39);
    TEST_ASSERT_EQUAL_UINT64((1ULL << 39) | 1, data);

    // Clear bit
    data = SR_CLR_BIT(data, 0);
    TEST_ASSERT_EQUAL_UINT64(1ULL << 39, data);

    // Get bit
    TEST_ASSERT_TRUE(SR_GET_BIT(data, 39));
    TEST_ASSERT_FALSE(SR_GET_BIT(data, 0));
}

/* ==========================================================================
 * Safe State Tests
 * ========================================================================== */

/**
 * Test: SR_SAFE_STATE is all zeros
 */
TEST_CASE("SR_SAFE_STATE is all zeros", "[tpic6b595]")
{
    TEST_ASSERT_EQUAL_UINT64(0ULL, SR_SAFE_STATE);
}

/**
 * Test: SR_ALL_EN mask covers all 8 axes
 */
TEST_CASE("SR_ALL_EN mask covers all axes", "[tpic6b595]")
{
    uint64_t expected = 0;
    for (int axis = 0; axis < 8; axis++) {
        expected |= (1ULL << SR_EN_BIT(axis));
    }
    TEST_ASSERT_EQUAL_UINT64(expected, SR_ALL_EN);
}
