/**
 * @file test_move_handler.cpp
 * @brief Unit tests for MOVE and MOVR command handlers
 * @author YaRobot Team
 * @date 2025
 *
 * Tests verify:
 * - MOVE command parsing and execution (AC1, AC2)
 * - MOVR command parsing and execution (AC3, AC4)
 * - Default velocity when not specified (AC10)
 * - Axis not enabled error (AC6)
 * - Position limit exceeded error (AC7)
 * - Invalid axis errors (AC12)
 * - All configuration from headers, no magic numbers (AC14)
 */

#include "unity.h"
#include "move_handler.h"
#include "movr_handler.h"
#include "motion_controller.h"
#include "i_motor.h"
#include "motor_types.h"
#include "config_defaults.h"
#include "config_limits.h"
#include "config_axes.h"
#include "config_commands.h"
#include "command_parser.h"
#include "response_formatter.h"
#include <cstring>

// ============================================================================
// Mock Motor Implementation for Handler Tests
// ============================================================================

/**
 * @brief Mock motor for testing command handlers
 */
class HandlerMockMotor : public IMotor {
public:
    HandlerMockMotor()
        : initialized_(false)
        , enabled_(true)  // Enabled by default
        , position_(0.0f)
        , velocity_(0.0f)
        , state_(AXIS_STATE_IDLE)
        , return_value_(ESP_OK)
    {
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
        if (position < config_.limit_min || position > config_.limit_max) {
            return ESP_ERR_INVALID_ARG;
        }
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
        last_delta_ = delta;
        last_velocity_ = velocity;
        state_ = AXIS_STATE_MOVING;
        return return_value_;
    }

    esp_err_t moveVelocity(float velocity) override {
        if (!enabled_) return ESP_ERR_INVALID_STATE;
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
    void setEnabled(bool en) { enabled_ = en; state_ = en ? AXIS_STATE_IDLE : AXIS_STATE_DISABLED; }
    void setPosition(float pos) { position_ = pos; }
    void setReturnValue(esp_err_t ret) { return_value_ = ret; }
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
    esp_err_t return_value_;
    float last_position_ = 0.0f;
    float last_velocity_ = 0.0f;
    float last_delta_ = 0.0f;
    MotionCompleteCallback motion_complete_cb_ = nullptr;
};

// ============================================================================
// Test Fixtures
// ============================================================================

static MotionController* s_controller = nullptr;
static HandlerMockMotor* s_motors[LIMIT_NUM_AXES] = {nullptr};
static char s_response[LIMIT_RESPONSE_MAX_LENGTH];

/**
 * @brief Helper to create ParsedCommand for MOVE/MOVR
 */
static void make_move_cmd(ParsedCommand* cmd, const char* verb, char axis,
                          int param_count, float p1 = 0.0f, float p2 = 0.0f)
{
    memset(cmd, 0, sizeof(*cmd));
    strncpy(cmd->verb, verb, sizeof(cmd->verb) - 1);
    cmd->axis = axis;
    cmd->param_count = param_count;
    if (param_count >= 1) cmd->params[0] = p1;
    if (param_count >= 2) cmd->params[1] = p2;
}

void setUp(void) {
    // Create mock motors
    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        s_motors[i] = new HandlerMockMotor();
        s_motors[i]->init();
    }

    // Create and initialize controller
    s_controller = new MotionController();
    IMotor* motor_ptrs[LIMIT_NUM_AXES];
    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        motor_ptrs[i] = s_motors[i];
    }
    s_controller->init(motor_ptrs);

    // Clear response buffer
    memset(s_response, 0, sizeof(s_response));
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
// MOVE Command Tests
// ============================================================================

/**
 * @brief AC1: MOVE command with position and velocity succeeds
 */
void test_move_with_position_and_velocity(void) {
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVE, 'X', 2, 0.1f, 0.05f);

    esp_err_t ret = handle_move(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "OK"));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1f, s_motors[AXIS_X]->getLastPosition());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.05f, s_motors[AXIS_X]->getLastVelocity());
}

/**
 * @brief AC10: MOVE with position only uses default velocity
 */
void test_move_with_position_only_uses_default_velocity(void) {
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVE, 'Y', 1, 0.05f);

    esp_err_t ret = handle_move(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "OK"));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.05f, s_motors[AXIS_Y]->getLastPosition());
    // Velocity should be DEFAULT_MAX_VELOCITY
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, DEFAULT_MAX_VELOCITY, s_motors[AXIS_Y]->getLastVelocity());
}

/**
 * @brief AC12: MOVE without axis returns invalid axis error
 */
void test_move_no_axis_returns_error(void) {
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVE, '\0', 1, 0.1f);

    esp_err_t ret = handle_move(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);  // format_error returns ESP_OK
    TEST_ASSERT_NOT_NULL(strstr(s_response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_INVALID_AXIS));
}

/**
 * @brief AC12: MOVE with invalid axis character returns error
 */
void test_move_invalid_axis_returns_error(void) {
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVE, 'Q', 1, 0.1f);

    esp_err_t ret = handle_move(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);  // format_error returns ESP_OK
    TEST_ASSERT_NOT_NULL(strstr(s_response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_INVALID_AXIS));
}

/**
 * @brief MOVE without position parameter returns error
 */
void test_move_no_position_returns_error(void) {
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVE, 'X', 0);  // No params

    esp_err_t ret = handle_move(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);  // format_error returns ESP_OK
    TEST_ASSERT_NOT_NULL(strstr(s_response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_INVALID_PARAMETER));
}

/**
 * @brief AC6: MOVE when axis disabled returns axis not enabled error
 */
void test_move_disabled_axis_returns_error(void) {
    s_motors[AXIS_X]->setEnabled(false);

    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVE, 'X', 1, 0.1f);

    esp_err_t ret = handle_move(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_AXIS_NOT_ENABLED));
}

/**
 * @brief AC7: MOVE with position exceeding limits returns error
 */
void test_move_position_limit_returns_error(void) {
    // DEFAULT_LIMIT_MAX is 1.0, so 2.0 exceeds it
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVE, 'X', 1, 2.0f);

    esp_err_t ret = handle_move(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_POSITION_LIMIT));
}

/**
 * @brief MOVE works for all valid axes
 */
void test_move_all_valid_axes(void) {
    const char axes[] = {'X', 'Y', 'Z', 'A', 'B', 'C', 'D', 'E'};
    ParsedCommand cmd;

    for (size_t i = 0; i < sizeof(axes); i++) {
        make_move_cmd(&cmd, CMD_MOVE, axes[i], 1, 0.05f);
        memset(s_response, 0, sizeof(s_response));

        esp_err_t ret = handle_move(&cmd, s_response, sizeof(s_response));

        TEST_ASSERT_EQUAL(ESP_OK, ret);
        TEST_ASSERT_NOT_NULL(strstr(s_response, "OK"));
    }
}

/**
 * @brief MOVE accepts lowercase axis
 */
void test_move_lowercase_axis(void) {
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVE, 'x', 1, 0.05f);

    esp_err_t ret = handle_move(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "OK"));
}

/**
 * @brief MOVE validates null arguments
 */
void test_move_null_arguments(void) {
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVE, 'X', 1, 0.1f);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, handle_move(nullptr, s_response, sizeof(s_response)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, handle_move(&cmd, nullptr, sizeof(s_response)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, handle_move(&cmd, s_response, 0));
}

// ============================================================================
// MOVR Command Tests
// ============================================================================

/**
 * @brief AC3: MOVR command with delta and velocity succeeds
 */
void test_movr_with_delta_and_velocity(void) {
    s_motors[AXIS_X]->setPosition(0.1f);  // Start at 0.1

    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVR, 'X', 2, 0.025f, 0.05f);

    esp_err_t ret = handle_movr(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "OK"));
    // Should have moved to 0.1 + 0.025 = 0.125
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.125f, s_motors[AXIS_X]->getLastPosition());
}

/**
 * @brief AC10: MOVR with delta only uses default velocity
 */
void test_movr_with_delta_only_uses_default_velocity(void) {
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVR, 'Y', 1, 0.025f);

    esp_err_t ret = handle_movr(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "OK"));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, DEFAULT_MAX_VELOCITY, s_motors[AXIS_Y]->getLastVelocity());
}

/**
 * @brief AC4: MOVR with negative delta moves backward
 */
void test_movr_negative_delta(void) {
    s_motors[AXIS_X]->setPosition(0.5f);  // Start at 0.5

    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVR, 'X', 1, -0.1f);

    esp_err_t ret = handle_movr(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "OK"));
    // Should have moved to 0.5 - 0.1 = 0.4
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.4f, s_motors[AXIS_X]->getLastPosition());
}

/**
 * @brief AC12: MOVR without axis returns error
 */
void test_movr_no_axis_returns_error(void) {
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVR, '\0', 1, 0.025f);

    esp_err_t ret = handle_movr(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_INVALID_AXIS));
}

/**
 * @brief AC12: MOVR with invalid axis returns error
 */
void test_movr_invalid_axis_returns_error(void) {
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVR, 'W', 1, 0.025f);

    esp_err_t ret = handle_movr(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_INVALID_AXIS));
}

/**
 * @brief MOVR without delta parameter returns error
 */
void test_movr_no_delta_returns_error(void) {
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVR, 'X', 0);  // No params

    esp_err_t ret = handle_movr(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_INVALID_PARAMETER));
}

/**
 * @brief MOVR when axis disabled returns error
 */
void test_movr_disabled_axis_returns_error(void) {
    s_motors[AXIS_X]->setEnabled(false);

    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVR, 'X', 1, 0.025f);

    esp_err_t ret = handle_movr(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_AXIS_NOT_ENABLED));
}

/**
 * @brief MOVR with delta exceeding limits returns error
 */
void test_movr_position_limit_returns_error(void) {
    s_motors[AXIS_X]->setPosition(0.9f);  // Near limit

    // Move +0.2 would exceed DEFAULT_LIMIT_MAX (1.0)
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVR, 'X', 1, 0.2f);

    esp_err_t ret = handle_movr(&cmd, s_response, sizeof(s_response));

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(strstr(s_response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(s_response, ERR_POSITION_LIMIT));
}

/**
 * @brief MOVR works for all valid axes
 */
void test_movr_all_valid_axes(void) {
    const char axes[] = {'X', 'Y', 'Z', 'A', 'B', 'C', 'D', 'E'};
    ParsedCommand cmd;

    for (size_t i = 0; i < sizeof(axes); i++) {
        make_move_cmd(&cmd, CMD_MOVR, axes[i], 1, 0.01f);
        memset(s_response, 0, sizeof(s_response));

        esp_err_t ret = handle_movr(&cmd, s_response, sizeof(s_response));

        TEST_ASSERT_EQUAL(ESP_OK, ret);
        TEST_ASSERT_NOT_NULL(strstr(s_response, "OK"));
    }
}

/**
 * @brief MOVR validates null arguments
 */
void test_movr_null_arguments(void) {
    ParsedCommand cmd;
    make_move_cmd(&cmd, CMD_MOVR, 'X', 1, 0.025f);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, handle_movr(nullptr, s_response, sizeof(s_response)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, handle_movr(&cmd, nullptr, sizeof(s_response)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, handle_movr(&cmd, s_response, 0));
}

// ============================================================================
// Test Runner
// ============================================================================

extern "C" void run_move_handler_tests(void) {
    UNITY_BEGIN();

    // MOVE tests
    RUN_TEST(test_move_with_position_and_velocity);
    RUN_TEST(test_move_with_position_only_uses_default_velocity);
    RUN_TEST(test_move_no_axis_returns_error);
    RUN_TEST(test_move_invalid_axis_returns_error);
    RUN_TEST(test_move_no_position_returns_error);
    RUN_TEST(test_move_disabled_axis_returns_error);
    RUN_TEST(test_move_position_limit_returns_error);
    RUN_TEST(test_move_all_valid_axes);
    RUN_TEST(test_move_lowercase_axis);
    RUN_TEST(test_move_null_arguments);

    // MOVR tests
    RUN_TEST(test_movr_with_delta_and_velocity);
    RUN_TEST(test_movr_with_delta_only_uses_default_velocity);
    RUN_TEST(test_movr_negative_delta);
    RUN_TEST(test_movr_no_axis_returns_error);
    RUN_TEST(test_movr_invalid_axis_returns_error);
    RUN_TEST(test_movr_no_delta_returns_error);
    RUN_TEST(test_movr_disabled_axis_returns_error);
    RUN_TEST(test_movr_position_limit_returns_error);
    RUN_TEST(test_movr_all_valid_axes);
    RUN_TEST(test_movr_null_arguments);

    UNITY_END();
}
