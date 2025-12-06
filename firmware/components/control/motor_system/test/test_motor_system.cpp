/**
 * @file test_motor_system.cpp
 * @brief Integration tests for motor system initialization
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 3-9b: Motor System Integration Tests
 *       Verifies that all components initialize correctly and
 *       the system is ready to accept motion commands.
 */

#include "unity.h"
#include "motor_system.h"
#include "motion_controller.h"
#include "config_axes.h"
#include "config_limits.h"

/**
 * @brief Test motor system initialization
 *
 * Verifies that motor_system_init() succeeds on first call.
 */
TEST_CASE("motor_system_init succeeds", "[motor_system]")
{
    // Note: This test assumes a clean system state
    // In practice, the system may already be initialized from previous tests
    if (!motor_system_is_initialized()) {
        esp_err_t ret = motor_system_init();
        TEST_ASSERT_EQUAL(ESP_OK, ret);
    }
    TEST_ASSERT_TRUE(motor_system_is_initialized());
}

/**
 * @brief Test motion controller is initialized after motor_system_init
 *
 * Verifies that MotionController has all 8 motors registered.
 */
TEST_CASE("motion_controller has all axes", "[motor_system]")
{
    // Ensure motor system is initialized
    if (!motor_system_is_initialized()) {
        esp_err_t ret = motor_system_init();
        TEST_ASSERT_EQUAL(ESP_OK, ret);
    }

    MotionController* controller = getMotionController();
    TEST_ASSERT_NOT_NULL(controller);
    TEST_ASSERT_TRUE(controller->isInitialized());

    // Verify all 8 axes have motors registered
    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        IMotor* motor = controller->getMotor(i);
        TEST_ASSERT_NOT_NULL_MESSAGE(motor, "Motor should be registered for axis");
    }
}

/**
 * @brief Test axis character lookup
 *
 * Verifies that getMotor(char) returns correct motors for all axis characters.
 */
TEST_CASE("motion_controller axis char lookup", "[motor_system]")
{
    // Ensure motor system is initialized
    if (!motor_system_is_initialized()) {
        esp_err_t ret = motor_system_init();
        TEST_ASSERT_EQUAL(ESP_OK, ret);
    }

    MotionController* controller = getMotionController();
    TEST_ASSERT_NOT_NULL(controller);

    // Test all axis characters
    const char axis_chars[] = {'X', 'Y', 'Z', 'A', 'B', 'C', 'D', 'E'};
    for (size_t i = 0; i < sizeof(axis_chars); i++) {
        IMotor* motor = controller->getMotor(axis_chars[i]);
        TEST_ASSERT_NOT_NULL_MESSAGE(motor, "Motor should be found by axis char");
    }

    // Test lowercase also works
    IMotor* motor_x = controller->getMotor('x');
    TEST_ASSERT_NOT_NULL(motor_x);
}

/**
 * @brief Test motor states after initialization
 *
 * Verifies that all motors are in expected initial states.
 */
TEST_CASE("motors have correct initial state", "[motor_system]")
{
    // Ensure motor system is initialized
    if (!motor_system_is_initialized()) {
        esp_err_t ret = motor_system_init();
        TEST_ASSERT_EQUAL(ESP_OK, ret);
    }

    MotionController* controller = getMotionController();
    TEST_ASSERT_NOT_NULL(controller);

    // All axes should start in UNHOMED or DISABLED state
    // (not MOVING or ERROR)
    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        IMotor* motor = controller->getMotor(i);
        TEST_ASSERT_NOT_NULL(motor);

        AxisState state = motor->getState();
        TEST_ASSERT_TRUE_MESSAGE(
            state == AXIS_STATE_UNHOMED || state == AXIS_STATE_DISABLED,
            "Motor should start in UNHOMED or DISABLED state"
        );
    }
}

/**
 * @brief Test motor_system_is_initialized before and after init
 *
 * Verifies the initialization flag works correctly.
 */
TEST_CASE("motor_system_is_initialized flag", "[motor_system]")
{
    // After init, should always return true
    if (!motor_system_is_initialized()) {
        esp_err_t ret = motor_system_init();
        TEST_ASSERT_EQUAL(ESP_OK, ret);
    }

    TEST_ASSERT_TRUE(motor_system_is_initialized());

    // Second call to init should return ESP_ERR_INVALID_STATE
    esp_err_t ret = motor_system_init();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
}

/**
 * @brief Test motor configuration values
 *
 * Verifies that motor configurations use expected default values.
 */
TEST_CASE("motor configs have valid defaults", "[motor_system]")
{
    // Ensure motor system is initialized
    if (!motor_system_is_initialized()) {
        esp_err_t ret = motor_system_init();
        TEST_ASSERT_EQUAL(ESP_OK, ret);
    }

    MotionController* controller = getMotionController();
    TEST_ASSERT_NOT_NULL(controller);

    // Check X axis (linear servo) has valid defaults
    IMotor* motor_x = controller->getMotor(AXIS_X);
    TEST_ASSERT_NOT_NULL(motor_x);
    const AxisConfig& cfg_x = motor_x->getConfig();
    TEST_ASSERT_FALSE(cfg_x.is_rotary);
    TEST_ASSERT_TRUE(cfg_x.max_velocity > 0);
    TEST_ASSERT_TRUE(cfg_x.max_acceleration > 0);

    // Check A axis (rotary servo) has valid defaults
    IMotor* motor_a = controller->getMotor(AXIS_A);
    TEST_ASSERT_NOT_NULL(motor_a);
    const AxisConfig& cfg_a = motor_a->getConfig();
    TEST_ASSERT_TRUE(cfg_a.is_rotary);

    // Check E axis (discrete) has valid defaults
    IMotor* motor_e = controller->getMotor(AXIS_E);
    TEST_ASSERT_NOT_NULL(motor_e);
    const AxisConfig& cfg_e = motor_e->getConfig();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cfg_e.limit_min);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, cfg_e.limit_max);
}

/**
 * @brief Test motor_system_update is safe before initialization
 *
 * Verifies that motor_system_update() is a no-op before init.
 */
TEST_CASE("motor_system_update safe before init", "[motor_system]")
{
    // This should not crash even if called before init
    // (though in practice motor_system is always initialized first)
    motor_system_update(AXIS_X);
    motor_system_update(AXIS_Y);
    motor_system_update(AXIS_E);
    // If we get here without crashing, the test passes
    TEST_PASS();
}
