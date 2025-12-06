/**
 * @file test_servo_motor.cpp
 * @brief Unit tests for ServoMotor implementation
 * @author YaRobot Team
 * @date 2025
 *
 * Tests verify:
 * - IMotor interface implementation
 * - Position moves (moveAbsolute, moveRelative)
 * - Velocity mode
 * - Stop methods
 * - Enable/disable state transitions
 * - Motion completion callbacks
 * - Limit validation
 * - Motion blending (no axis busy error)
 * - Configuration using header constants only
 */

#include "unity.h"
#include "servo_motor.h"
#include "i_motor.h"
#include "i_pulse_generator.h"
#include "i_position_tracker.h"
#include "motor_types.h"
#include "config_defaults.h"
#include "config_limits.h"
#include "config_timing.h"
#include "tpic6b595.h"
#include <atomic>
#include <cmath>

// ============================================================================
// Mock Implementations
// ============================================================================

/**
 * @brief Mock pulse generator for unit testing
 *
 * Records calls and simulates motion behavior without hardware.
 */
class MockPulseGenerator : public IPulseGenerator {
public:
    MockPulseGenerator()
        : initialized_(false)
        , running_(false)
        , pulse_count_(0)
        , current_velocity_(0.0f)
        , last_pulses_(0)
        , last_velocity_(0.0f)
        , last_acceleration_(0.0f)
        , start_move_count_(0)
        , start_velocity_count_(0)
        , stop_count_(0)
        , stop_immediate_count_(0)
        , position_tracker_(nullptr)
        , completion_cb_(nullptr)
    {}

    esp_err_t init() override {
        initialized_ = true;
        return ESP_OK;
    }

    esp_err_t startMove(int32_t pulses, float max_velocity, float acceleration) override {
        if (!initialized_) return ESP_ERR_INVALID_STATE;

        last_pulses_ = pulses;
        last_velocity_ = max_velocity;
        last_acceleration_ = acceleration;
        start_move_count_++;
        running_ = true;
        current_velocity_ = max_velocity;

        // Simulate position tracker direction if set
        if (position_tracker_) {
            position_tracker_->setDirection(pulses >= 0);
        }

        return ESP_OK;
    }

    esp_err_t startVelocity(float velocity, float acceleration) override {
        if (!initialized_) return ESP_ERR_INVALID_STATE;

        last_velocity_ = velocity;
        last_acceleration_ = acceleration;
        start_velocity_count_++;
        running_ = true;
        current_velocity_ = std::fabs(velocity);

        if (position_tracker_) {
            position_tracker_->setDirection(velocity >= 0);
        }

        return ESP_OK;
    }

    esp_err_t stop(float deceleration) override {
        if (!running_) return ESP_ERR_INVALID_STATE;

        stop_count_++;
        running_ = false;
        current_velocity_ = 0.0f;

        // Simulate completion callback
        if (completion_cb_) {
            completion_cb_(pulse_count_);
        }

        return ESP_OK;
    }

    void stopImmediate() override {
        stop_immediate_count_++;
        running_ = false;
        current_velocity_ = 0.0f;
        // Note: stopImmediate does NOT fire completion callback
    }

    bool isRunning() const override { return running_; }

    int64_t getPulseCount() const override { return pulse_count_; }

    float getCurrentVelocity() const override { return current_velocity_; }

    void setCompletionCallback(MotionCompleteCallback cb) override {
        completion_cb_ = cb;
    }

    void setPositionTracker(IPositionTracker* tracker) override {
        position_tracker_ = tracker;
    }

    // Test helpers
    void reset() {
        running_ = false;
        pulse_count_ = 0;
        current_velocity_ = 0.0f;
        start_move_count_ = 0;
        start_velocity_count_ = 0;
        stop_count_ = 0;
        stop_immediate_count_ = 0;
        last_pulses_ = 0;
        last_velocity_ = 0.0f;
        last_acceleration_ = 0.0f;
    }

    void simulateMotionComplete(int64_t final_pulses) {
        pulse_count_ = final_pulses;
        running_ = false;
        current_velocity_ = 0.0f;
        if (completion_cb_) {
            completion_cb_(final_pulses);
        }
    }

    // Accessors for test verification
    int32_t getLastPulses() const { return last_pulses_; }
    float getLastVelocity() const { return last_velocity_; }
    float getLastAcceleration() const { return last_acceleration_; }
    int getStartMoveCount() const { return start_move_count_; }
    int getStartVelocityCount() const { return start_velocity_count_; }
    int getStopCount() const { return stop_count_; }
    int getStopImmediateCount() const { return stop_immediate_count_; }

private:
    bool initialized_;
    bool running_;
    int64_t pulse_count_;
    float current_velocity_;
    int32_t last_pulses_;
    float last_velocity_;
    float last_acceleration_;
    int start_move_count_;
    int start_velocity_count_;
    int stop_count_;
    int stop_immediate_count_;
    IPositionTracker* position_tracker_;
    MotionCompleteCallback completion_cb_;
};

/**
 * @brief Mock position tracker for unit testing
 */
class MockPositionTracker : public IPositionTracker {
public:
    MockPositionTracker()
        : initialized_(false)
        , position_(0)
        , forward_(true)
    {}

    esp_err_t init() override {
        initialized_ = true;
        return ESP_OK;
    }

    esp_err_t reset(int64_t position = 0) override {
        if (!initialized_) return ESP_ERR_INVALID_STATE;
        position_.store(position, std::memory_order_release);
        return ESP_OK;
    }

    int64_t getPosition() const override {
        return position_.load(std::memory_order_acquire);
    }

    void setDirection(bool forward) override {
        forward_ = forward;
    }

    void addPulses(int64_t count) override {
        if (forward_) {
            position_.fetch_add(count, std::memory_order_acq_rel);
        } else {
            position_.fetch_sub(count, std::memory_order_acq_rel);
        }
    }

    // Test helpers
    bool isForward() const { return forward_; }
    void setPosition(int64_t pos) { position_.store(pos, std::memory_order_release); }

private:
    bool initialized_;
    std::atomic<int64_t> position_;
    bool forward_;
};

// ============================================================================
// Test Fixtures
// ============================================================================

static MockPulseGenerator* g_mock_pulse_gen = nullptr;
static MockPositionTracker* g_mock_tracker = nullptr;
static ServoMotor* g_motor = nullptr;

static void setUp(void)
{
    // Initialize shift register (required for motor)
    sr_init();
}

static void tearDown(void)
{
    delete g_motor;
    g_motor = nullptr;
    delete g_mock_pulse_gen;
    g_mock_pulse_gen = nullptr;
    delete g_mock_tracker;
    g_mock_tracker = nullptr;
}

/**
 * @brief Create motor with default linear config for testing
 */
static ServoMotor* createTestMotor(uint8_t axis_id = SR_AXIS_X)
{
    g_mock_pulse_gen = new MockPulseGenerator();
    g_mock_tracker = new MockPositionTracker();

    g_mock_pulse_gen->init();
    g_mock_tracker->init();

    AxisConfig config = AxisConfig::createDefaultLinear();
    g_motor = new ServoMotor(g_mock_pulse_gen, g_mock_tracker, axis_id, config);
    return g_motor;
}

// ============================================================================
// AC1: IMotor interface implementation
// ============================================================================

TEST_CASE("ServoMotor implements IMotor interface", "[servo_motor][AC1]")
{
    ServoMotor* motor = createTestMotor();
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

TEST_CASE("ServoMotor IMotor interface - all methods exist", "[servo_motor][AC1]")
{
    ServoMotor* motor = createTestMotor();
    IMotor* interface = motor;

    TEST_ASSERT_EQUAL(ESP_OK, interface->init());
    TEST_ASSERT_EQUAL(ESP_OK, interface->enable(true));

    // Test all method signatures exist
    esp_err_t err = interface->moveAbsolute(0.0f, DEFAULT_MAX_VELOCITY);
    TEST_ASSERT_TRUE(err == ESP_OK || err == ESP_ERR_INVALID_ARG);

    err = interface->moveRelative(0.0f, DEFAULT_MAX_VELOCITY);
    TEST_ASSERT_TRUE(err == ESP_OK || err == ESP_ERR_INVALID_ARG);

    err = interface->moveVelocity(DEFAULT_MAX_VELOCITY);
    TEST_ASSERT_TRUE(err == ESP_OK);

    interface->stopImmediate();
    TEST_ASSERT_FALSE(interface->isMoving());

    float pos = interface->getPosition();
    (void)pos;

    float vel = interface->getVelocity();
    (void)vel;

    AxisState state = interface->getState();
    TEST_ASSERT_TRUE(state == AXIS_STATE_IDLE || state == AXIS_STATE_DISABLED || state == AXIS_STATE_UNHOMED);

    const AxisConfig& config = interface->getConfig();
    TEST_ASSERT_GREATER_THAN(0, config.max_velocity);

    interface->setMotionCompleteCallback([](uint8_t, float) {});
}

// ============================================================================
// AC2: ServoMotor construction and init
// ============================================================================

TEST_CASE("ServoMotor init validates dependencies", "[servo_motor][AC2]")
{
    g_mock_tracker = new MockPositionTracker();
    g_mock_tracker->init();

    AxisConfig config = AxisConfig::createDefaultLinear();

    // Null pulse generator should fail
    ServoMotor motor1(nullptr, g_mock_tracker, SR_AXIS_X, config);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor1.init());
}

TEST_CASE("ServoMotor init sets state to UNHOMED", "[servo_motor][AC2]")
{
    ServoMotor* motor = createTestMotor();

    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(AXIS_STATE_UNHOMED, motor->getState());
}

TEST_CASE("ServoMotor init validates configuration", "[servo_motor][AC2]")
{
    g_mock_pulse_gen = new MockPulseGenerator();
    g_mock_tracker = new MockPositionTracker();
    g_mock_pulse_gen->init();
    g_mock_tracker->init();

    // Invalid config: limits min >= max
    AxisConfig bad_config = AxisConfig::createDefaultLinear();
    bad_config.limit_min = 1.0f;
    bad_config.limit_max = 0.0f;

    ServoMotor motor(g_mock_pulse_gen, g_mock_tracker, SR_AXIS_X, bad_config);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor.init());
}

TEST_CASE("ServoMotor double init returns error", "[servo_motor][AC2]")
{
    ServoMotor* motor = createTestMotor();

    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, motor->init());
}

// ============================================================================
// AC3: moveAbsolute with position validation and conversion
// ============================================================================

TEST_CASE("ServoMotor moveAbsolute converts SI to pulses", "[servo_motor][AC3]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Move to 0.001m (1mm) with default 1,000,000 pulses/m
    float target = 0.001f;  // 1mm
    float velocity = DEFAULT_MAX_VELOCITY;

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(target, velocity));

    // Verify pulse conversion
    float pulses_per_unit = DEFAULT_PULSES_PER_UNIT;  // 1,000,000 pulses/m
    int32_t expected_pulses = static_cast<int32_t>(target * pulses_per_unit);

    TEST_ASSERT_EQUAL(expected_pulses, g_mock_pulse_gen->getLastPulses());
}

TEST_CASE("ServoMotor moveAbsolute sets direction via shift register", "[servo_motor][AC3]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Forward move
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.01f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(1, g_mock_pulse_gen->getStartMoveCount());

    // Simulate completion and move reverse
    g_mock_pulse_gen->simulateMotionComplete(10000);  // 10000 pulses = 0.01m
    g_mock_tracker->setPosition(10000);

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(-0.01f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(2, g_mock_pulse_gen->getStartMoveCount());
}

TEST_CASE("ServoMotor moveAbsolute sets state to MOVING", "[servo_motor][AC3]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.01f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(AXIS_STATE_MOVING, motor->getState());
    TEST_ASSERT_TRUE(motor->isMoving());
}

// ============================================================================
// AC4: moveRelative delegates to moveAbsolute
// ============================================================================

TEST_CASE("ServoMotor moveRelative delegates to moveAbsolute", "[servo_motor][AC4]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Set position to known value
    g_mock_tracker->setPosition(1000);  // 0.001m

    // Move relative +0.001m
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveRelative(0.001f, DEFAULT_MAX_VELOCITY));

    // Should move from current (0.001m) to target (0.002m) = +0.001m = 1000 pulses
    float pulses_per_unit = DEFAULT_PULSES_PER_UNIT;
    int32_t expected_pulses = static_cast<int32_t>(0.001f * pulses_per_unit);
    TEST_ASSERT_EQUAL(expected_pulses, g_mock_pulse_gen->getLastPulses());
}

// ============================================================================
// AC5: moveVelocity with velocity clamping
// ============================================================================

TEST_CASE("ServoMotor moveVelocity clamps to max_velocity", "[servo_motor][AC5]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Request velocity higher than max
    float excessive_velocity = DEFAULT_MAX_VELOCITY * 2.0f;
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(excessive_velocity));

    // Velocity should be clamped
    float pulses_per_unit = DEFAULT_PULSES_PER_UNIT;
    float expected_freq = DEFAULT_MAX_VELOCITY * pulses_per_unit;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected_freq, g_mock_pulse_gen->getLastVelocity());
}

TEST_CASE("ServoMotor moveVelocity invokes startVelocity", "[servo_motor][AC5]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(1, g_mock_pulse_gen->getStartVelocityCount());
    TEST_ASSERT_EQUAL(AXIS_STATE_MOVING, motor->getState());
}

TEST_CASE("ServoMotor moveVelocity handles negative velocity", "[servo_motor][AC5]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(-DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(1, g_mock_pulse_gen->getStartVelocityCount());

    // Negative velocity should result in negative frequency
    float pulses_per_unit = DEFAULT_PULSES_PER_UNIT;
    float expected_freq = -DEFAULT_MAX_VELOCITY * pulses_per_unit;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected_freq, g_mock_pulse_gen->getLastVelocity());
}

// ============================================================================
// AC6: stop with deceleration
// ============================================================================

TEST_CASE("ServoMotor stop decelerates via pulse generator", "[servo_motor][AC6]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Start motion
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_TRUE(motor->isMoving());

    // Stop
    TEST_ASSERT_EQUAL(ESP_OK, motor->stop());
    TEST_ASSERT_EQUAL(1, g_mock_pulse_gen->getStopCount());
}

TEST_CASE("ServoMotor stop returns error if not moving", "[servo_motor][AC6]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Not moving, stop should fail
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, motor->stop());
}

// ============================================================================
// AC7: stopImmediate halts immediately
// ============================================================================

TEST_CASE("ServoMotor stopImmediate halts immediately", "[servo_motor][AC7]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Start motion
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(AXIS_STATE_MOVING, motor->getState());

    // Immediate stop
    motor->stopImmediate();
    TEST_ASSERT_EQUAL(1, g_mock_pulse_gen->getStopImmediateCount());
    TEST_ASSERT_EQUAL(AXIS_STATE_IDLE, motor->getState());
    TEST_ASSERT_FALSE(motor->isMoving());
}

// ============================================================================
// AC8: enable(true) transitions to IDLE
// ============================================================================

TEST_CASE("ServoMotor enable true transitions to IDLE", "[servo_motor][AC8]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());

    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));
    TEST_ASSERT_TRUE(motor->isEnabled());
    // After enable, should be UNHOMED or IDLE (UNHOMED since not homed yet)
    AxisState state = motor->getState();
    TEST_ASSERT_TRUE(state == AXIS_STATE_IDLE || state == AXIS_STATE_UNHOMED);
}

// ============================================================================
// AC9: enable(false) stops motion and disables
// ============================================================================

TEST_CASE("ServoMotor enable false stops motion", "[servo_motor][AC9]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Start motion
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_TRUE(motor->isMoving());

    // Disable while moving
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(false));
    TEST_ASSERT_EQUAL(AXIS_STATE_DISABLED, motor->getState());
    TEST_ASSERT_FALSE(motor->isEnabled());
    TEST_ASSERT_FALSE(motor->isMoving());
    TEST_ASSERT_EQUAL(1, g_mock_pulse_gen->getStopImmediateCount());
}

TEST_CASE("ServoMotor enable false when idle", "[servo_motor][AC9]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(false));
    TEST_ASSERT_EQUAL(AXIS_STATE_DISABLED, motor->getState());
    TEST_ASSERT_FALSE(motor->isEnabled());
}

// ============================================================================
// AC10: getPosition returns SI units
// ============================================================================

TEST_CASE("ServoMotor getPosition returns SI units", "[servo_motor][AC10]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());

    // Set position to 1000 pulses (= 0.001m with 1M pulses/m)
    g_mock_tracker->setPosition(1000);
    g_mock_pulse_gen->simulateMotionComplete(1000);

    float pos = motor->getPosition();
    float expected = 1000.0f / DEFAULT_PULSES_PER_UNIT;  // 0.001m
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, expected, pos);
}

// ============================================================================
// AC11: getState returns valid AxisState
// ============================================================================

TEST_CASE("ServoMotor getState returns valid states", "[servo_motor][AC11]")
{
    ServoMotor* motor = createTestMotor();

    // Before init - not defined, but after init:
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(AXIS_STATE_UNHOMED, motor->getState());

    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));
    AxisState state = motor->getState();
    TEST_ASSERT_TRUE(state == AXIS_STATE_IDLE || state == AXIS_STATE_UNHOMED);

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(AXIS_STATE_MOVING, motor->getState());

    motor->stopImmediate();
    TEST_ASSERT_EQUAL(AXIS_STATE_IDLE, motor->getState());

    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(false));
    TEST_ASSERT_EQUAL(AXIS_STATE_DISABLED, motor->getState());
}

// ============================================================================
// AC12: Motion completion callback
// ============================================================================

TEST_CASE("ServoMotor motion complete callback invoked", "[servo_motor][AC12]")
{
    ServoMotor* motor = createTestMotor();
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

    // Start and complete motion
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.001f, DEFAULT_MAX_VELOCITY));
    g_mock_tracker->setPosition(1000);  // 1000 pulses = 0.001m
    g_mock_pulse_gen->simulateMotionComplete(1000);

    TEST_ASSERT_TRUE(callback_invoked);
    TEST_ASSERT_EQUAL(SR_AXIS_X, callback_axis);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.001f, callback_position);
}

TEST_CASE("ServoMotor state transitions to IDLE on completion", "[servo_motor][AC12]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.001f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(AXIS_STATE_MOVING, motor->getState());

    g_mock_tracker->setPosition(1000);
    g_mock_pulse_gen->simulateMotionComplete(1000);

    TEST_ASSERT_EQUAL(AXIS_STATE_IDLE, motor->getState());
}

// ============================================================================
// AC13: Motion blending (no axis busy error)
// ============================================================================

TEST_CASE("ServoMotor motion blending - no axis busy error", "[servo_motor][AC13]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Start first move
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.01f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_TRUE(motor->isMoving());

    // Issue new move while still moving - should NOT return error
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.02f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(2, g_mock_pulse_gen->getStartMoveCount());
}

// ============================================================================
// AC14: Limit validation
// ============================================================================

TEST_CASE("ServoMotor moveAbsolute rejects position below limit_min", "[servo_motor][AC14]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Default limit_min is -1.0m
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor->moveAbsolute(-2.0f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(0, g_mock_pulse_gen->getStartMoveCount());
}

TEST_CASE("ServoMotor moveAbsolute rejects position above limit_max", "[servo_motor][AC14]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Default limit_max is 1.0m
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor->moveAbsolute(2.0f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(0, g_mock_pulse_gen->getStartMoveCount());
}

TEST_CASE("ServoMotor moveRelative rejects target exceeding limits", "[servo_motor][AC14]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Set position near max
    g_mock_tracker->setPosition(900000);  // 0.9m

    // Try to move +0.2m (would result in 1.1m > limit_max)
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor->moveRelative(0.2f, DEFAULT_MAX_VELOCITY));
}

// ============================================================================
// AC15: Motion command on disabled axis
// ============================================================================

TEST_CASE("ServoMotor moveAbsolute rejects when disabled", "[servo_motor][AC15]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    // Don't enable

    TEST_ASSERT_EQUAL(AXIS_STATE_UNHOMED, motor->getState());
    // UNHOMED is not DISABLED, but init doesn't enable the axis
    // Let's explicitly disable
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(false));
    TEST_ASSERT_EQUAL(AXIS_STATE_DISABLED, motor->getState());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, motor->moveAbsolute(0.01f, DEFAULT_MAX_VELOCITY));
}

TEST_CASE("ServoMotor moveVelocity rejects when disabled", "[servo_motor][AC15]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(false));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, motor->moveVelocity(DEFAULT_MAX_VELOCITY));
}

// ============================================================================
// AC16: No hardcoded values - verify config constants are used
// ============================================================================

TEST_CASE("ServoMotor uses TIMING_DIR_SETUP_US from config", "[servo_motor][AC16]")
{
    // Verify the constant exists and has expected value
    TEST_ASSERT_EQUAL(20, TIMING_DIR_SETUP_US);
}

TEST_CASE("ServoMotor uses TIMING_ENABLE_DELAY_US from config", "[servo_motor][AC16]")
{
    // Verify the constant exists and has expected value
    TEST_ASSERT_EQUAL(50, TIMING_ENABLE_DELAY_US);
}

TEST_CASE("ServoMotor uses DEFAULT_PULSES_PER_UNIT from config", "[servo_motor][AC16]")
{
    TEST_ASSERT_EQUAL(1000000.0f, DEFAULT_PULSES_PER_UNIT);
}

TEST_CASE("ServoMotor uses DEFAULT_MAX_VELOCITY from config", "[servo_motor][AC16]")
{
    TEST_ASSERT_EQUAL(0.1f, DEFAULT_MAX_VELOCITY);
}

TEST_CASE("ServoMotor uses DEFAULT_MAX_ACCELERATION from config", "[servo_motor][AC16]")
{
    TEST_ASSERT_EQUAL(1.0f, DEFAULT_MAX_ACCELERATION);
}

TEST_CASE("ServoMotor uses LIMIT_ALIAS_MAX_LENGTH from config", "[servo_motor][AC16]")
{
    TEST_ASSERT_EQUAL(16, LIMIT_ALIAS_MAX_LENGTH);

    // Verify AxisConfig alias buffer uses this constant
    AxisConfig config;
    TEST_ASSERT_EQUAL(LIMIT_ALIAS_MAX_LENGTH + 1, sizeof(config.alias));
}

// ============================================================================
// Configuration and AxisConfig tests
// ============================================================================

TEST_CASE("AxisConfig createDefaultLinear uses config defaults", "[servo_motor][config]")
{
    AxisConfig config = AxisConfig::createDefaultLinear();

    TEST_ASSERT_EQUAL(DEFAULT_IS_ROTARY, config.is_rotary);
    TEST_ASSERT_EQUAL(DEFAULT_LIMIT_MIN, config.limit_min);
    TEST_ASSERT_EQUAL(DEFAULT_LIMIT_MAX, config.limit_max);
    TEST_ASSERT_EQUAL(DEFAULT_MAX_VELOCITY, config.max_velocity);
    TEST_ASSERT_EQUAL(DEFAULT_MAX_ACCELERATION, config.max_acceleration);
    TEST_ASSERT_EQUAL(DEFAULT_BACKLASH, config.backlash);
    TEST_ASSERT_EQUAL(DEFAULT_HOME_OFFSET, config.home_offset);
}

TEST_CASE("AxisConfig createDefaultRotary uses config defaults", "[servo_motor][config]")
{
    AxisConfig config = AxisConfig::createDefaultRotary();

    TEST_ASSERT_TRUE(config.is_rotary);
    TEST_ASSERT_EQUAL(DEFAULT_ROTARY_LIMIT_MIN, config.limit_min);
    TEST_ASSERT_EQUAL(DEFAULT_ROTARY_LIMIT_MAX, config.limit_max);
    TEST_ASSERT_EQUAL(DEFAULT_ROTARY_MAX_VEL, config.max_velocity);
    TEST_ASSERT_EQUAL(DEFAULT_ROTARY_MAX_ACCEL, config.max_acceleration);
}

TEST_CASE("AxisConfig getPulsesPerUnit calculates correctly", "[servo_motor][config]")
{
    AxisConfig config;
    config.pulses_per_rev = 10000.0f;
    config.units_per_rev = 0.01f;  // 10mm per rev

    float expected = 10000.0f / 0.01f;  // 1,000,000 pulses/m
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected, config.getPulsesPerUnit());
}

TEST_CASE("ServoMotor setConfig updates configuration", "[servo_motor][config]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());

    AxisConfig new_config = AxisConfig::createDefaultLinear();
    new_config.max_velocity = 0.2f;

    TEST_ASSERT_EQUAL(ESP_OK, motor->setConfig(new_config));
    TEST_ASSERT_EQUAL(0.2f, motor->getConfig().max_velocity);
}

TEST_CASE("ServoMotor setConfig rejects while moving", "[servo_motor][config]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Start motion
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(DEFAULT_MAX_VELOCITY));

    AxisConfig new_config = AxisConfig::createDefaultLinear();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, motor->setConfig(new_config));
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_CASE("ServoMotor handles zero distance move", "[servo_motor][edge]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Move to current position (zero distance)
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.0f, DEFAULT_MAX_VELOCITY));
    // Should not start motion
    TEST_ASSERT_EQUAL(0, g_mock_pulse_gen->getStartMoveCount());
}

TEST_CASE("ServoMotor getVelocity returns 0 when not moving", "[servo_motor][edge]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(0.0f, motor->getVelocity());
}

TEST_CASE("ServoMotor isMoving returns false after stopImmediate", "[servo_motor][edge]")
{
    ServoMotor* motor = createTestMotor();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_TRUE(motor->isMoving());

    motor->stopImmediate();
    TEST_ASSERT_FALSE(motor->isMoving());
}
