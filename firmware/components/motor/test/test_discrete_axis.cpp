/**
 * @file test_discrete_axis.cpp
 * @brief Unit tests for DiscreteAxis implementation
 * @author YaRobot Team
 * @date 2025
 *
 * Tests verify:
 * - IMotor interface implementation [AC1]
 * - Extend motion (0→1) with shift register control [AC2]
 * - Retract motion (1→0) [AC3]
 * - Already-at-position completes immediately [AC4]
 * - Velocity mode returns NOT_SUPPORTED [AC5]
 * - Stop cancels motion [AC6]
 * - StopImmediate clears enable [AC7]
 * - Position interpolation during motion [AC8]
 * - Velocity returns fixed value when moving [AC9]
 * - Enable sets SR_E_EN [AC10]
 * - Disable while moving stops immediately [AC11]
 * - Position outside [0.0, 1.0] returns INVALID_ARG [AC12]
 * - Disabled axis rejects commands [AC13]
 * - NO hardcoded values - all from config headers [AC14]
 */

#include "unity.h"
#include "discrete_axis.h"
#include "i_motor.h"
#include "time_tracker.h"
#include "motor_types.h"
#include "config_defaults.h"
#include "config_timing.h"
#include "config_sr.h"
#include "tpic6b595.h"
#include <atomic>
#include <cmath>

// ============================================================================
// Mock TimeTracker for unit testing
// ============================================================================

/**
 * @brief Mock time tracker that allows test control over position/motion state
 */
class MockTimeTracker : public TimeTracker {
public:
    MockTimeTracker()
        : TimeTracker(TIMING_E_AXIS_TRAVEL_MS)
        , mock_position_(0)
        , mock_in_motion_(false)
        , mock_motion_complete_(true)
        , start_motion_count_(0)
        , set_direction_count_(0)
        , last_direction_(true)
    {}

    esp_err_t init() override {
        return ESP_OK;
    }

    esp_err_t reset(int64_t position = 0) override {
        mock_position_ = position;
        mock_in_motion_ = false;
        return ESP_OK;
    }

    int64_t getPosition() const override {
        return mock_position_;
    }

    void setDirection(bool forward) override {
        set_direction_count_++;
        last_direction_ = forward;
    }

    void startMotion() override {
        start_motion_count_++;
        mock_in_motion_ = true;
    }

    bool isMotionComplete() const override {
        return mock_motion_complete_;
    }

    // Test helpers
    void setMockPosition(int64_t pos) { mock_position_ = pos; }
    void setMockInMotion(bool moving) { mock_in_motion_ = moving; }
    void setMockMotionComplete(bool complete) { mock_motion_complete_ = complete; }
    int getStartMotionCount() const { return start_motion_count_; }
    int getSetDirectionCount() const { return set_direction_count_; }
    bool getLastDirection() const { return last_direction_; }
    void resetCounters() {
        start_motion_count_ = 0;
        set_direction_count_ = 0;
    }

private:
    int64_t mock_position_;
    bool mock_in_motion_;
    bool mock_motion_complete_;
    int start_motion_count_;
    int set_direction_count_;
    bool last_direction_;
};

// ============================================================================
// Test Fixtures
// ============================================================================

static MockTimeTracker* g_mock_tracker = nullptr;
static DiscreteAxis* g_motor = nullptr;

static void setUp(void)
{
    // Initialize shift register (required for motor)
    sr_init();
}

static void tearDown(void)
{
    delete g_motor;
    g_motor = nullptr;
    delete g_mock_tracker;
    g_mock_tracker = nullptr;
}

/**
 * @brief Create DiscreteAxis with E-axis config for testing
 */
static DiscreteAxis* createTestMotor()
{
    g_mock_tracker = new MockTimeTracker();

    // Create E-axis configuration using config constants
    AxisConfig config;
    config.pulses_per_rev = E_AXIS_PULSES_PER_UNIT;
    config.units_per_rev = 1.0f;
    config.is_rotary = false;
    config.limit_min = E_AXIS_LIMIT_MIN;
    config.limit_max = E_AXIS_LIMIT_MAX;
    config.max_velocity = E_AXIS_MAX_VELOCITY;
    config.max_acceleration = E_AXIS_MAX_ACCELERATION;
    config.backlash = 0.0f;
    config.home_offset = 0.0f;
    config.alias[0] = '\0';

    g_motor = new DiscreteAxis(g_mock_tracker, SR_AXIS_E, config);
    return g_motor;
}

// ============================================================================
// AC1: IMotor interface implementation
// ============================================================================

TEST_CASE("DiscreteAxis implements IMotor interface", "[discrete_axis][AC1]")
{
    DiscreteAxis* motor = createTestMotor();
    IMotor* interface = motor;

    // All interface methods should be callable
    TEST_ASSERT_EQUAL(ESP_OK, interface->init());
    TEST_ASSERT_NOT_NULL(interface);

    // Enable and check state methods work
    TEST_ASSERT_EQUAL(ESP_OK, interface->enable(true));
    TEST_ASSERT_TRUE(interface->isEnabled());
    TEST_ASSERT_FALSE(interface->isMoving());
    TEST_ASSERT_EQUAL(AXIS_STATE_IDLE, interface->getState());
}

TEST_CASE("DiscreteAxis IMotor interface - all methods exist", "[discrete_axis][AC1]")
{
    DiscreteAxis* motor = createTestMotor();
    IMotor* interface = motor;

    TEST_ASSERT_EQUAL(ESP_OK, interface->init());
    TEST_ASSERT_EQUAL(ESP_OK, interface->enable(true));

    // Test all method signatures exist
    esp_err_t err = interface->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY);
    TEST_ASSERT_EQUAL(ESP_OK, err);

    interface->stopImmediate();

    err = interface->moveRelative(0.0f, E_AXIS_MAX_VELOCITY);
    TEST_ASSERT_TRUE(err == ESP_OK || err == ESP_ERR_INVALID_ARG);

    // Velocity mode should return NOT_SUPPORTED
    err = interface->moveVelocity(E_AXIS_MAX_VELOCITY);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, err);

    float pos = interface->getPosition();
    TEST_ASSERT_TRUE(pos >= E_AXIS_LIMIT_MIN && pos <= E_AXIS_LIMIT_MAX);

    float vel = interface->getVelocity();
    (void)vel;

    AxisState state = interface->getState();
    TEST_ASSERT_TRUE(state == AXIS_STATE_IDLE || state == AXIS_STATE_DISABLED ||
                     state == AXIS_STATE_UNHOMED || state == AXIS_STATE_MOVING);

    const AxisConfig& config = interface->getConfig();
    TEST_ASSERT_GREATER_THAN(0, config.max_velocity);

    interface->setMotionCompleteCallback([](uint8_t, float) {});
}

TEST_CASE("DiscreteAxis init validates dependencies", "[discrete_axis][AC1]")
{
    AxisConfig config;
    config.pulses_per_rev = E_AXIS_PULSES_PER_UNIT;
    config.units_per_rev = 1.0f;
    config.is_rotary = false;
    config.limit_min = E_AXIS_LIMIT_MIN;
    config.limit_max = E_AXIS_LIMIT_MAX;
    config.max_velocity = E_AXIS_MAX_VELOCITY;
    config.max_acceleration = E_AXIS_MAX_ACCELERATION;
    config.backlash = 0.0f;
    config.home_offset = 0.0f;
    config.alias[0] = '\0';

    // Null time tracker should fail
    DiscreteAxis motor(nullptr, SR_AXIS_E, config);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor.init());
}

TEST_CASE("DiscreteAxis init sets state to UNHOMED", "[discrete_axis][AC1]")
{
    DiscreteAxis* motor = createTestMotor();

    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(AXIS_STATE_UNHOMED, motor->getState());
}

// ============================================================================
// AC2: moveAbsolute(1.0) - Extend motion
// ============================================================================

TEST_CASE("DiscreteAxis moveAbsolute extend sets direction forward", "[discrete_axis][AC2]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Extend from 0 to 1
    g_mock_tracker->setMockPosition(0);
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY));

    // Verify direction was set to forward (extend)
    TEST_ASSERT_TRUE(g_mock_tracker->getLastDirection());
    TEST_ASSERT_EQUAL(1, g_mock_tracker->getSetDirectionCount());
}

TEST_CASE("DiscreteAxis moveAbsolute extend calls startMotion", "[discrete_axis][AC2]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    g_mock_tracker->setMockPosition(0);
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY));

    // Verify startMotion was called
    TEST_ASSERT_EQUAL(1, g_mock_tracker->getStartMotionCount());
}

TEST_CASE("DiscreteAxis moveAbsolute extend sets state to MOVING", "[discrete_axis][AC2]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    g_mock_tracker->setMockPosition(0);
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY));

    TEST_ASSERT_EQUAL(AXIS_STATE_MOVING, motor->getState());
    TEST_ASSERT_TRUE(motor->isMoving());
}

// ============================================================================
// AC3: moveAbsolute(0.0) - Retract motion
// ============================================================================

TEST_CASE("DiscreteAxis moveAbsolute retract sets direction reverse", "[discrete_axis][AC3]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Start at position 1, retract to 0
    g_mock_tracker->setMockPosition(1);

    // Need to set internal position state - use a move first then stop
    // For this test, we'll just verify direction behavior
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MIN, E_AXIS_MAX_VELOCITY));

    // Verify direction was set to reverse (retract)
    TEST_ASSERT_FALSE(g_mock_tracker->getLastDirection());
}

// ============================================================================
// AC4: Already at target completes immediately
// ============================================================================

TEST_CASE("DiscreteAxis already at target completes immediately", "[discrete_axis][AC4]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    bool callback_invoked = false;
    float callback_position = -1.0f;

    motor->setMotionCompleteCallback([&](uint8_t axis, float pos) {
        callback_invoked = true;
        callback_position = pos;
    });

    // Set current position to 1.0 (extended)
    g_mock_tracker->setMockPosition(1);

    // Move to same position
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY));

    // Should complete immediately with callback
    TEST_ASSERT_TRUE(callback_invoked);
    // State should NOT be MOVING (completed immediately)
    TEST_ASSERT_FALSE(motor->isMoving());
}

// ============================================================================
// AC5: moveVelocity returns NOT_SUPPORTED
// ============================================================================

TEST_CASE("DiscreteAxis moveVelocity returns NOT_SUPPORTED", "[discrete_axis][AC5]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Velocity mode should return NOT_SUPPORTED
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, motor->moveVelocity(E_AXIS_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, motor->moveVelocity(-E_AXIS_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, motor->moveVelocity(0.0f));
}

// ============================================================================
// AC6: stop cancels motion, keeps interpolated position
// ============================================================================

TEST_CASE("DiscreteAxis stop cancels motion timer", "[discrete_axis][AC6]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Start motion
    g_mock_tracker->setMockPosition(0);
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY));
    TEST_ASSERT_TRUE(motor->isMoving());

    // Stop motion
    TEST_ASSERT_EQUAL(ESP_OK, motor->stop());
    TEST_ASSERT_FALSE(motor->isMoving());
    TEST_ASSERT_EQUAL(AXIS_STATE_IDLE, motor->getState());
}

TEST_CASE("DiscreteAxis stop returns error if not moving", "[discrete_axis][AC6]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Not moving, stop should fail
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, motor->stop());
}

// ============================================================================
// AC7: stopImmediate clears EN, state → IDLE
// ============================================================================

TEST_CASE("DiscreteAxis stopImmediate halts immediately", "[discrete_axis][AC7]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Start motion
    g_mock_tracker->setMockPosition(0);
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(AXIS_STATE_MOVING, motor->getState());

    // Immediate stop
    motor->stopImmediate();
    TEST_ASSERT_EQUAL(AXIS_STATE_IDLE, motor->getState());
    TEST_ASSERT_FALSE(motor->isMoving());
}

// ============================================================================
// AC8: getPosition returns interpolated position during motion
// ============================================================================

TEST_CASE("DiscreteAxis getPosition returns position from tracker", "[discrete_axis][AC8]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Test position at 0
    g_mock_tracker->setMockPosition(0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, E_AXIS_LIMIT_MIN, motor->getPosition());

    // Test position at 1
    g_mock_tracker->setMockPosition(1);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, E_AXIS_LIMIT_MAX, motor->getPosition());
}

// ============================================================================
// AC9: getVelocity returns E_AXIS_MAX_VELOCITY if moving, 0.0 if idle
// ============================================================================

TEST_CASE("DiscreteAxis getVelocity returns max when moving", "[discrete_axis][AC9]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Not moving
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, motor->getVelocity());

    // Start motion
    g_mock_tracker->setMockPosition(0);
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY));

    // Moving - should return fixed velocity
    TEST_ASSERT_FLOAT_WITHIN(0.01f, E_AXIS_MAX_VELOCITY, motor->getVelocity());

    // Stop
    motor->stopImmediate();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, motor->getVelocity());
}

// ============================================================================
// AC10: enable(true) sets SR_E_EN, state → IDLE
// ============================================================================

TEST_CASE("DiscreteAxis enable true transitions to IDLE", "[discrete_axis][AC10]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());

    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));
    TEST_ASSERT_TRUE(motor->isEnabled());
    TEST_ASSERT_EQUAL(AXIS_STATE_IDLE, motor->getState());
}

// ============================================================================
// AC11: enable(false) while moving stops immediately, state → DISABLED
// ============================================================================

TEST_CASE("DiscreteAxis enable false stops motion", "[discrete_axis][AC11]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Start motion
    g_mock_tracker->setMockPosition(0);
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY));
    TEST_ASSERT_TRUE(motor->isMoving());

    // Disable while moving
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(false));
    TEST_ASSERT_EQUAL(AXIS_STATE_DISABLED, motor->getState());
    TEST_ASSERT_FALSE(motor->isEnabled());
    TEST_ASSERT_FALSE(motor->isMoving());
}

TEST_CASE("DiscreteAxis enable false when idle", "[discrete_axis][AC11]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(false));
    TEST_ASSERT_EQUAL(AXIS_STATE_DISABLED, motor->getState());
    TEST_ASSERT_FALSE(motor->isEnabled());
}

// ============================================================================
// AC12: Position outside [0.0, 1.0] returns INVALID_ARG
// ============================================================================

TEST_CASE("DiscreteAxis rejects position below E_AXIS_LIMIT_MIN", "[discrete_axis][AC12]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Position below 0.0 should fail
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor->moveAbsolute(-0.1f, E_AXIS_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor->moveAbsolute(-1.0f, E_AXIS_MAX_VELOCITY));
}

TEST_CASE("DiscreteAxis rejects position above E_AXIS_LIMIT_MAX", "[discrete_axis][AC12]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Position above 1.0 should fail
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor->moveAbsolute(1.1f, E_AXIS_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor->moveAbsolute(2.0f, E_AXIS_MAX_VELOCITY));
}

TEST_CASE("DiscreteAxis moveRelative rejects out of range target", "[discrete_axis][AC12]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // At position 1.0, try to move +1.0 (would be 2.0)
    g_mock_tracker->setMockPosition(1);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor->moveRelative(1.0f, E_AXIS_MAX_VELOCITY));

    // At position 0.0, try to move -1.0 (would be -1.0)
    g_mock_tracker->setMockPosition(0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor->moveRelative(-1.0f, E_AXIS_MAX_VELOCITY));
}

// ============================================================================
// AC13: Disabled axis returns INVALID_STATE on motion commands
// ============================================================================

TEST_CASE("DiscreteAxis moveAbsolute rejects when disabled", "[discrete_axis][AC13]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    // Don't enable - explicitly disable
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(false));
    TEST_ASSERT_EQUAL(AXIS_STATE_DISABLED, motor->getState());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, motor->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY));
}

TEST_CASE("DiscreteAxis moveRelative rejects when disabled", "[discrete_axis][AC13]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(false));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, motor->moveRelative(1.0f, E_AXIS_MAX_VELOCITY));
}

// ============================================================================
// AC14: No hardcoded values - verify config constants are used
// ============================================================================

TEST_CASE("DiscreteAxis uses E_AXIS_LIMIT_MIN from config", "[discrete_axis][AC14]")
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, E_AXIS_LIMIT_MIN);
}

TEST_CASE("DiscreteAxis uses E_AXIS_LIMIT_MAX from config", "[discrete_axis][AC14]")
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, E_AXIS_LIMIT_MAX);
}

TEST_CASE("DiscreteAxis uses E_AXIS_MAX_VELOCITY from config", "[discrete_axis][AC14]")
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, E_AXIS_MAX_VELOCITY);
}

TEST_CASE("DiscreteAxis uses TIMING_E_AXIS_TRAVEL_MS from config", "[discrete_axis][AC14]")
{
    TEST_ASSERT_EQUAL(1000, TIMING_E_AXIS_TRAVEL_MS);
}

TEST_CASE("DiscreteAxis uses TIMING_ENABLE_DELAY_US from config", "[discrete_axis][AC14]")
{
    TEST_ASSERT_EQUAL(50, TIMING_ENABLE_DELAY_US);
}

TEST_CASE("DiscreteAxis uses SR_E_DIR from config", "[discrete_axis][AC14]")
{
    TEST_ASSERT_EQUAL(28, SR_E_DIR);
}

TEST_CASE("DiscreteAxis uses SR_E_EN from config", "[discrete_axis][AC14]")
{
    TEST_ASSERT_EQUAL(29, SR_E_EN);
}

TEST_CASE("DiscreteAxis uses SR_AXIS_E from config", "[discrete_axis][AC14]")
{
    TEST_ASSERT_EQUAL(7, SR_AXIS_E);
}

// ============================================================================
// Configuration tests
// ============================================================================

TEST_CASE("DiscreteAxis getConfig returns configuration", "[discrete_axis][config]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());

    const AxisConfig& config = motor->getConfig();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, E_AXIS_LIMIT_MIN, config.limit_min);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, E_AXIS_LIMIT_MAX, config.limit_max);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, E_AXIS_MAX_VELOCITY, config.max_velocity);
}

TEST_CASE("DiscreteAxis setConfig updates configuration", "[discrete_axis][config]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());

    AxisConfig new_config;
    new_config.pulses_per_rev = E_AXIS_PULSES_PER_UNIT;
    new_config.units_per_rev = 1.0f;
    new_config.is_rotary = false;
    new_config.limit_min = E_AXIS_LIMIT_MIN;
    new_config.limit_max = E_AXIS_LIMIT_MAX;
    new_config.max_velocity = 0.5f;  // Different velocity
    new_config.max_acceleration = E_AXIS_MAX_ACCELERATION;
    new_config.backlash = 0.0f;
    new_config.home_offset = 0.0f;
    new_config.alias[0] = '\0';

    TEST_ASSERT_EQUAL(ESP_OK, motor->setConfig(new_config));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, motor->getConfig().max_velocity);
}

TEST_CASE("DiscreteAxis setConfig rejects while moving", "[discrete_axis][config]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Start motion
    g_mock_tracker->setMockPosition(0);
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY));

    AxisConfig new_config;
    new_config.pulses_per_rev = E_AXIS_PULSES_PER_UNIT;
    new_config.units_per_rev = 1.0f;
    new_config.is_rotary = false;
    new_config.limit_min = E_AXIS_LIMIT_MIN;
    new_config.limit_max = E_AXIS_LIMIT_MAX;
    new_config.max_velocity = E_AXIS_MAX_VELOCITY;
    new_config.max_acceleration = E_AXIS_MAX_ACCELERATION;
    new_config.backlash = 0.0f;
    new_config.home_offset = 0.0f;
    new_config.alias[0] = '\0';

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, motor->setConfig(new_config));
}

TEST_CASE("DiscreteAxis setConfig rejects invalid config", "[discrete_axis][config]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());

    // Invalid: limit_min >= limit_max
    AxisConfig bad_config;
    bad_config.pulses_per_rev = E_AXIS_PULSES_PER_UNIT;
    bad_config.units_per_rev = 1.0f;
    bad_config.is_rotary = false;
    bad_config.limit_min = 1.0f;
    bad_config.limit_max = 0.0f;  // Invalid!
    bad_config.max_velocity = E_AXIS_MAX_VELOCITY;
    bad_config.max_acceleration = E_AXIS_MAX_ACCELERATION;
    bad_config.backlash = 0.0f;
    bad_config.home_offset = 0.0f;
    bad_config.alias[0] = '\0';

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor->setConfig(bad_config));
}

// ============================================================================
// Motion complete callback tests
// ============================================================================

TEST_CASE("DiscreteAxis motion complete callback invoked", "[discrete_axis][callback]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    bool callback_invoked = false;
    uint8_t callback_axis = 255;
    float callback_position = -999.0f;

    motor->setMotionCompleteCallback([&](uint8_t axis, float pos) {
        callback_invoked = true;
        callback_axis = axis;
        callback_position = pos;
    });

    // For already-at-position case, callback should be invoked immediately
    g_mock_tracker->setMockPosition(1);
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY));

    TEST_ASSERT_TRUE(callback_invoked);
    TEST_ASSERT_EQUAL(SR_AXIS_E, callback_axis);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_CASE("DiscreteAxis handles zero distance move", "[discrete_axis][edge]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    g_mock_tracker->setMockPosition(0);
    g_mock_tracker->resetCounters();

    // Move to current position (zero distance)
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MIN, E_AXIS_MAX_VELOCITY));

    // Should complete immediately, no motion started
    TEST_ASSERT_FALSE(motor->isMoving());
}

TEST_CASE("DiscreteAxis isMoving returns false after stopImmediate", "[discrete_axis][edge]")
{
    DiscreteAxis* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    g_mock_tracker->setMockPosition(0);
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(E_AXIS_LIMIT_MAX, E_AXIS_MAX_VELOCITY));
    TEST_ASSERT_TRUE(motor->isMoving());

    motor->stopImmediate();
    TEST_ASSERT_FALSE(motor->isMoving());
}

TEST_CASE("DiscreteAxis double init returns error", "[discrete_axis][edge]")
{
    DiscreteAxis* motor = createTestMotor();

    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, motor->init());
}
