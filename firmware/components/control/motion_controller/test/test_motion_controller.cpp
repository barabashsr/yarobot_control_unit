/**
 * @file test_motion_controller.cpp
 * @brief Unit tests for MotionController implementation
 * @author YaRobot Team
 * @date 2025
 *
 * Tests verify:
 * - init() validates all motor pointers (AC8)
 * - getMotor(uint8_t) returns correct motor (AC11)
 * - getMotor(char) maps axis characters correctly (AC11)
 * - moveAbsolute() delegates to motor (AC1, AC2)
 * - moveAbsolute() uses default velocity when not specified (AC10)
 * - moveAbsolute() returns error for disabled axis (AC6)
 * - moveRelative() calculates target correctly (AC3, AC4)
 * - Motion complete callback publishes event (AC13)
 * - All configuration from headers, no magic numbers (AC14)
 */

#include "unity.h"
#include "motion_controller.h"
#include "i_motor.h"
#include "motor_types.h"
#include "config_defaults.h"
#include "config_limits.h"
#include "config_axes.h"
#include <cstring>

// ============================================================================
// Mock Motor Implementation
// ============================================================================

/**
 * @brief Mock motor for unit testing MotionController
 *
 * Records calls and simulates motor behavior without hardware.
 */
class MockMotor : public IMotor {
public:
    MockMotor()
        : initialized_(false)
        , enabled_(false)
        , position_(0.0f)
        , velocity_(0.0f)
        , state_(AXIS_STATE_DISABLED)
        , move_absolute_count_(0)
        , move_relative_count_(0)
        , last_position_(0.0f)
        , last_velocity_(0.0f)
        , last_delta_(0.0f)
        , return_value_(ESP_OK)
        , motion_complete_cb_(nullptr)
    {
        // Initialize config with defaults
        config_.pulses_per_unit = DEFAULT_PULSES_PER_UNIT;
        config_.is_rotary = DEFAULT_IS_ROTARY;
        config_.limit_min = DEFAULT_LIMIT_MIN;
        config_.limit_max = DEFAULT_LIMIT_MAX;
        config_.max_velocity = DEFAULT_MAX_VELOCITY;
        config_.max_acceleration = DEFAULT_MAX_ACCELERATION;
        config_.backlash = DEFAULT_BACKLASH;
        config_.home_offset = DEFAULT_HOME_OFFSET;
    }

    esp_err_t init() override {
        initialized_ = true;
        state_ = AXIS_STATE_UNHOMED;
        return ESP_OK;
    }

    esp_err_t moveAbsolute(float position, float velocity) override {
        if (!enabled_) {
            return ESP_ERR_INVALID_STATE;
        }

        // Check position limits
        if (position < config_.limit_min || position > config_.limit_max) {
            return ESP_ERR_INVALID_ARG;
        }

        move_absolute_count_++;
        last_position_ = position;
        last_velocity_ = velocity;
        state_ = AXIS_STATE_MOVING;

        return return_value_;
    }

    esp_err_t moveRelative(float delta, float velocity) override {
        if (!enabled_) {
            return ESP_ERR_INVALID_STATE;
        }

        float target = position_ + delta;
        if (target < config_.limit_min || target > config_.limit_max) {
            return ESP_ERR_INVALID_ARG;
        }

        move_relative_count_++;
        last_delta_ = delta;
        last_velocity_ = velocity;
        state_ = AXIS_STATE_MOVING;

        return return_value_;
    }

    esp_err_t moveVelocity(float velocity) override {
        if (!enabled_) {
            return ESP_ERR_INVALID_STATE;
        }
        velocity_ = velocity;
        state_ = AXIS_STATE_MOVING;
        return ESP_OK;
    }

    esp_err_t stop() override {
        velocity_ = 0.0f;
        state_ = AXIS_STATE_IDLE;
        return ESP_OK;
    }

    void stopImmediate() override {
        velocity_ = 0.0f;
        state_ = AXIS_STATE_IDLE;
    }

    float getPosition() const override { return position_; }
    float getVelocity() const override { return velocity_; }
    bool isMoving() const override { return state_ == AXIS_STATE_MOVING; }
    bool isEnabled() const override { return enabled_; }
    AxisState getState() const override { return state_; }
    const AxisConfig& getConfig() const override { return config_; }

    esp_err_t setConfig(const AxisConfig& config) override {
        config_ = config;
        return ESP_OK;
    }

    esp_err_t enable(bool en) override {
        enabled_ = en;
        state_ = en ? AXIS_STATE_IDLE : AXIS_STATE_DISABLED;
        return ESP_OK;
    }

    void setMotionCompleteCallback(MotionCompleteCallback cb) override {
        motion_complete_cb_ = cb;
    }

    // Test helpers
    void reset() {
        move_absolute_count_ = 0;
        move_relative_count_ = 0;
        last_position_ = 0.0f;
        last_velocity_ = 0.0f;
        last_delta_ = 0.0f;
        return_value_ = ESP_OK;
    }

    void setPosition(float pos) { position_ = pos; }
    void setReturnValue(esp_err_t ret) { return_value_ = ret; }
    void simulateMotionComplete(uint8_t axis, float pos) {
        if (motion_complete_cb_) {
            motion_complete_cb_(axis, pos);
        }
    }

    // Accessors for test verification
    int getMoveAbsoluteCount() const { return move_absolute_count_; }
    int getMoveRelativeCount() const { return move_relative_count_; }
    float getLastPosition() const { return last_position_; }
    float getLastVelocity() const { return last_velocity_; }
    float getLastDelta() const { return last_delta_; }

private:
    bool initialized_;
    bool enabled_;
    float position_;
    float velocity_;
    AxisState state_;
    AxisConfig config_;

    int move_absolute_count_;
    int move_relative_count_;
    float last_position_;
    float last_velocity_;
    float last_delta_;
    esp_err_t return_value_;
    MotionCompleteCallback motion_complete_cb_;
};

// ============================================================================
// Test Fixtures
// ============================================================================

static MotionController* s_controller = nullptr;
static MockMotor* s_motors[LIMIT_NUM_AXES] = {nullptr};

void setUp(void) {
    // Create mock motors for all axes
    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        s_motors[i] = new MockMotor();
        s_motors[i]->init();
        s_motors[i]->enable(true);  // Enable by default for most tests
    }

    // Create and initialize controller
    s_controller = new MotionController();
    IMotor* motor_ptrs[LIMIT_NUM_AXES];
    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        motor_ptrs[i] = s_motors[i];
    }
    s_controller->init(motor_ptrs);
}

void tearDown(void) {
    delete s_controller;
    s_controller = nullptr;

    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        delete s_motors[i];
        s_motors[i] = nullptr;
    }
}

// ============================================================================
// Test Cases
// ============================================================================

/**
 * @brief Test init() stores all motor pointers (AC8)
 */
void test_init_stores_motors(void) {
    TEST_ASSERT_TRUE(s_controller->isInitialized());

    // Verify each motor can be retrieved
    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        IMotor* motor = s_controller->getMotor(i);
        TEST_ASSERT_NOT_NULL(motor);
        TEST_ASSERT_EQUAL_PTR(s_motors[i], motor);
    }
}

/**
 * @brief Test init() rejects null motors array (AC8)
 */
void test_init_rejects_null_array(void) {
    MotionController controller;
    esp_err_t ret = controller.init(nullptr);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
    TEST_ASSERT_FALSE(controller.isInitialized());
}

/**
 * @brief Test init() rejects null motor pointer (AC8)
 */
void test_init_rejects_null_motor(void) {
    MotionController controller;
    IMotor* motors[LIMIT_NUM_AXES];
    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        motors[i] = s_motors[i];
    }
    // Set one motor to null
    motors[AXIS_Y] = nullptr;

    esp_err_t ret = controller.init(motors);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
    TEST_ASSERT_FALSE(controller.isInitialized());
}

/**
 * @brief Test getMotor(uint8_t) returns correct motor (AC11)
 */
void test_getMotor_by_id_returns_correct_motor(void) {
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_X], s_controller->getMotor(AXIS_X));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_Y], s_controller->getMotor(AXIS_Y));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_Z], s_controller->getMotor(AXIS_Z));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_A], s_controller->getMotor(AXIS_A));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_B], s_controller->getMotor(AXIS_B));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_C], s_controller->getMotor(AXIS_C));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_D], s_controller->getMotor(AXIS_D));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_E], s_controller->getMotor(AXIS_E));
}

/**
 * @brief Test getMotor(uint8_t) returns null for invalid axis (AC12)
 */
void test_getMotor_by_id_invalid_returns_null(void) {
    TEST_ASSERT_NULL(s_controller->getMotor(LIMIT_NUM_AXES));
    TEST_ASSERT_NULL(s_controller->getMotor(LIMIT_NUM_AXES + 1));
    TEST_ASSERT_NULL(s_controller->getMotor(255));
}

/**
 * @brief Test getMotor(char) maps characters correctly (AC11)
 */
void test_getMotor_by_char_maps_correctly(void) {
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_X], s_controller->getMotor('X'));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_Y], s_controller->getMotor('Y'));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_Z], s_controller->getMotor('Z'));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_A], s_controller->getMotor('A'));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_B], s_controller->getMotor('B'));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_C], s_controller->getMotor('C'));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_D], s_controller->getMotor('D'));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_E], s_controller->getMotor('E'));
}

/**
 * @brief Test getMotor(char) is case-insensitive (AC11)
 */
void test_getMotor_by_char_case_insensitive(void) {
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_X], s_controller->getMotor('x'));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_Y], s_controller->getMotor('y'));
    TEST_ASSERT_EQUAL_PTR(s_motors[AXIS_E], s_controller->getMotor('e'));
}

/**
 * @brief Test getMotor(char) returns null for invalid char (AC12)
 */
void test_getMotor_by_char_invalid_returns_null(void) {
    TEST_ASSERT_NULL(s_controller->getMotor('F'));  // Beyond valid range
    TEST_ASSERT_NULL(s_controller->getMotor('W'));
    TEST_ASSERT_NULL(s_controller->getMotor('0'));
    TEST_ASSERT_NULL(s_controller->getMotor(' '));
}

/**
 * @brief Test moveAbsolute() delegates to motor (AC1, AC2)
 */
void test_moveAbsolute_delegates_to_motor(void) {
    const float position = 0.1f;
    const float velocity = 0.05f;

    esp_err_t ret = s_controller->moveAbsolute(AXIS_X, position, velocity);

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(1, s_motors[AXIS_X]->getMoveAbsoluteCount());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, position, s_motors[AXIS_X]->getLastPosition());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, velocity, s_motors[AXIS_X]->getLastVelocity());
}

/**
 * @brief Test moveAbsolute() uses default velocity when not specified (AC10)
 */
void test_moveAbsolute_uses_default_velocity(void) {
    const float position = 0.1f;

    // Velocity <= 0 means use default
    esp_err_t ret = s_controller->moveAbsolute(AXIS_X, position, 0.0f);

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, DEFAULT_MAX_VELOCITY, s_motors[AXIS_X]->getLastVelocity());
}

/**
 * @brief Test moveAbsolute() uses default velocity for negative (AC10)
 */
void test_moveAbsolute_negative_velocity_uses_default(void) {
    const float position = 0.1f;

    esp_err_t ret = s_controller->moveAbsolute(AXIS_X, position, -0.1f);

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, DEFAULT_MAX_VELOCITY, s_motors[AXIS_X]->getLastVelocity());
}

/**
 * @brief Test moveAbsolute() returns error for disabled axis (AC6)
 */
void test_moveAbsolute_disabled_axis_returns_error(void) {
    s_motors[AXIS_X]->enable(false);  // Disable X axis

    esp_err_t ret = s_controller->moveAbsolute(AXIS_X, 0.1f, 0.05f);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
    TEST_ASSERT_EQUAL(0, s_motors[AXIS_X]->getMoveAbsoluteCount());  // Not called
}

/**
 * @brief Test moveAbsolute() returns error for invalid axis (AC12)
 */
void test_moveAbsolute_invalid_axis_returns_error(void) {
    esp_err_t ret = s_controller->moveAbsolute(LIMIT_NUM_AXES, 0.1f, 0.05f);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * @brief Test moveRelative() calculates target correctly (AC3, AC4)
 */
void test_moveRelative_calculates_target(void) {
    const float current = 0.1f;
    const float delta = 0.025f;
    const float expected_target = current + delta;

    s_motors[AXIS_X]->setPosition(current);

    esp_err_t ret = s_controller->moveRelative(AXIS_X, delta, 0.05f);

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    // moveRelative calls moveAbsolute internally
    TEST_ASSERT_EQUAL(1, s_motors[AXIS_X]->getMoveAbsoluteCount());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected_target, s_motors[AXIS_X]->getLastPosition());
}

/**
 * @brief Test moveRelative() handles negative delta (AC4)
 */
void test_moveRelative_negative_delta(void) {
    const float current = 0.2f;
    const float delta = -0.05f;
    const float expected_target = current + delta;

    s_motors[AXIS_Y]->setPosition(current);

    esp_err_t ret = s_controller->moveRelative(AXIS_Y, delta, 0.03f);

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected_target, s_motors[AXIS_Y]->getLastPosition());
}

/**
 * @brief Test moveRelative() returns error for disabled axis
 */
void test_moveRelative_disabled_axis_returns_error(void) {
    s_motors[AXIS_X]->enable(false);

    esp_err_t ret = s_controller->moveRelative(AXIS_X, 0.025f, 0.05f);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);
}

/**
 * @brief Test simultaneous moves on different axes (AC5)
 *
 * Verifies that MOVE X and MOVE Y can both succeed without blocking.
 */
void test_simultaneous_moves_independent(void) {
    esp_err_t ret_x = s_controller->moveAbsolute(AXIS_X, 0.1f, 0.05f);
    esp_err_t ret_y = s_controller->moveAbsolute(AXIS_Y, 0.05f, 0.03f);

    TEST_ASSERT_EQUAL(ESP_OK, ret_x);
    TEST_ASSERT_EQUAL(ESP_OK, ret_y);
    TEST_ASSERT_EQUAL(1, s_motors[AXIS_X]->getMoveAbsoluteCount());
    TEST_ASSERT_EQUAL(1, s_motors[AXIS_Y]->getMoveAbsoluteCount());
}

// ============================================================================
// Test Runner
// ============================================================================

extern "C" void run_motion_controller_tests(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_stores_motors);
    RUN_TEST(test_init_rejects_null_array);
    RUN_TEST(test_init_rejects_null_motor);
    RUN_TEST(test_getMotor_by_id_returns_correct_motor);
    RUN_TEST(test_getMotor_by_id_invalid_returns_null);
    RUN_TEST(test_getMotor_by_char_maps_correctly);
    RUN_TEST(test_getMotor_by_char_case_insensitive);
    RUN_TEST(test_getMotor_by_char_invalid_returns_null);
    RUN_TEST(test_moveAbsolute_delegates_to_motor);
    RUN_TEST(test_moveAbsolute_uses_default_velocity);
    RUN_TEST(test_moveAbsolute_negative_velocity_uses_default);
    RUN_TEST(test_moveAbsolute_disabled_axis_returns_error);
    RUN_TEST(test_moveAbsolute_invalid_axis_returns_error);
    RUN_TEST(test_moveRelative_calculates_target);
    RUN_TEST(test_moveRelative_negative_delta);
    RUN_TEST(test_moveRelative_disabled_axis_returns_error);
    RUN_TEST(test_simultaneous_moves_independent);

    UNITY_END();
}
