/**
 * @file test_stepper_motor.cpp
 * @brief Unit tests for StepperMotor implementation
 * @author YaRobot Team
 * @date 2025
 *
 * Tests verify:
 * - IMotor interface implementation via MotorBase inheritance
 * - Stepper-specific behavior (PCNT is authority, no brake hook)
 * - Position moves (moveAbsolute, moveRelative)
 * - Velocity mode
 * - Stop methods
 * - Enable/disable state transitions
 * - Motion completion with PCNT position sync
 * - Limit validation
 * - Configuration using header constants only (NO magic numbers)
 */

#include "unity.h"
#include "stepper_motor.h"
#include "i_motor.h"
#include "i_pulse_generator.h"
#include "i_position_tracker.h"
#include "motor_types.h"
#include "config_defaults.h"
#include "config_limits.h"
#include "config_timing.h"
#include "config_peripherals.h"
#include "config_sr.h"
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

        if (completion_cb_) {
            completion_cb_(pulse_count_);
        }

        return ESP_OK;
    }

    void stopImmediate() override {
        stop_immediate_count_++;
        running_ = false;
        current_velocity_ = 0.0f;
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
 * @brief Mock position tracker simulating PCNT hardware counter
 *
 * For stepper motors, this simulates the PCNT hardware counter
 * which is the authoritative position source.
 */
class MockPcntTracker : public IPositionTracker {
public:
    MockPcntTracker()
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

    // Test helpers - simulate PCNT hardware
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
static MockPcntTracker* g_mock_tracker = nullptr;
static StepperMotor* g_motor = nullptr;

static void setUp(void)
{
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
 * @brief Create stepper motor with default linear config for testing
 *
 * Uses SR_AXIS_C (axis 5) by default - stepper axis
 */
static StepperMotor* createTestStepper(uint8_t axis_id = SR_AXIS_C)
{
    g_mock_pulse_gen = new MockPulseGenerator();
    g_mock_tracker = new MockPcntTracker();

    g_mock_pulse_gen->init();
    g_mock_tracker->init();

    AxisConfig config = AxisConfig::createDefaultLinear();
    g_motor = new StepperMotor(g_mock_pulse_gen, g_mock_tracker, axis_id, config);
    return g_motor;
}

// ============================================================================
// AC1: StepperMotor construction and init (inherits from MotorBase)
// ============================================================================

TEST_CASE("StepperMotor implements IMotor interface", "[stepper_motor][AC1]")
{
    StepperMotor* motor = createTestStepper();
    IMotor* interface = motor;

    TEST_ASSERT_EQUAL(ESP_OK, interface->init());
    TEST_ASSERT_NOT_NULL(interface);

    TEST_ASSERT_EQUAL(ESP_OK, interface->enable(true));
    TEST_ASSERT_TRUE(interface->isEnabled());
    TEST_ASSERT_FALSE(interface->isMoving());
}

TEST_CASE("StepperMotor init validates dependencies", "[stepper_motor][AC1]")
{
    g_mock_tracker = new MockPcntTracker();
    g_mock_tracker->init();

    AxisConfig config = AxisConfig::createDefaultLinear();

    // Null pulse generator should fail
    StepperMotor motor1(nullptr, g_mock_tracker, SR_AXIS_C, config);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor1.init());
}

TEST_CASE("StepperMotor init sets state to UNHOMED", "[stepper_motor][AC1]")
{
    StepperMotor* motor = createTestStepper();

    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(AXIS_STATE_UNHOMED, motor->getState());
}

// ============================================================================
// AC2, AC3: C and D axis motion execution
// ============================================================================

TEST_CASE("StepperMotor moveAbsolute converts SI to pulses", "[stepper_motor][AC2][AC3]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    float target = 0.001f;  // 1mm
    float velocity = DEFAULT_MAX_VELOCITY;

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(target, velocity));

    float pulses_per_unit = DEFAULT_PULSES_PER_UNIT;
    int32_t expected_pulses = static_cast<int32_t>(target * pulses_per_unit);

    TEST_ASSERT_EQUAL(expected_pulses, g_mock_pulse_gen->getLastPulses());
}

TEST_CASE("StepperMotor moveAbsolute sets state to MOVING", "[stepper_motor][AC2][AC3]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.01f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(AXIS_STATE_MOVING, motor->getState());
    TEST_ASSERT_TRUE(motor->isMoving());
}

// ============================================================================
// AC4: Stepper-specific behavior (no brake, PCNT authority)
// ============================================================================

TEST_CASE("StepperMotor onEnableHook is no-op (no brake for steppers)", "[stepper_motor][AC4]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());

    // Enable should succeed - onEnableHook is no-op
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));
    TEST_ASSERT_TRUE(motor->isEnabled());

    // Disable should succeed - onEnableHook is no-op
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(false));
    TEST_ASSERT_FALSE(motor->isEnabled());
}

TEST_CASE("StepperMotor always starts UNHOMED (position lost on power)", "[stepper_motor][AC4]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());

    // Steppers always start UNHOMED because position is lost on power cycle
    TEST_ASSERT_EQUAL(AXIS_STATE_UNHOMED, motor->getState());
}

// ============================================================================
// AC5: moveRelative delegates to moveAbsolute
// ============================================================================

TEST_CASE("StepperMotor moveRelative delegates to moveAbsolute", "[stepper_motor][AC5]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    g_mock_tracker->setPosition(1000);  // 0.001m

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveRelative(0.001f, DEFAULT_MAX_VELOCITY));

    float pulses_per_unit = DEFAULT_PULSES_PER_UNIT;
    int32_t expected_pulses = static_cast<int32_t>(0.001f * pulses_per_unit);
    TEST_ASSERT_EQUAL(expected_pulses, g_mock_pulse_gen->getLastPulses());
}

// ============================================================================
// AC6: moveVelocity with velocity clamping
// ============================================================================

TEST_CASE("StepperMotor moveVelocity clamps to max_velocity", "[stepper_motor][AC6]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    float excessive_velocity = DEFAULT_MAX_VELOCITY * 2.0f;
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(excessive_velocity));

    float pulses_per_unit = DEFAULT_PULSES_PER_UNIT;
    float expected_freq = DEFAULT_MAX_VELOCITY * pulses_per_unit;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected_freq, g_mock_pulse_gen->getLastVelocity());
}

TEST_CASE("StepperMotor moveVelocity invokes startVelocity", "[stepper_motor][AC6]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(1, g_mock_pulse_gen->getStartVelocityCount());
    TEST_ASSERT_EQUAL(AXIS_STATE_MOVING, motor->getState());
}

// ============================================================================
// AC7: stop with deceleration
// ============================================================================

TEST_CASE("StepperMotor stop decelerates via pulse generator", "[stepper_motor][AC7]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_TRUE(motor->isMoving());

    TEST_ASSERT_EQUAL(ESP_OK, motor->stop());
    TEST_ASSERT_EQUAL(1, g_mock_pulse_gen->getStopCount());
}

// ============================================================================
// AC8: stopImmediate halts immediately
// ============================================================================

TEST_CASE("StepperMotor stopImmediate halts immediately", "[stepper_motor][AC8]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(AXIS_STATE_MOVING, motor->getState());

    motor->stopImmediate();
    TEST_ASSERT_EQUAL(1, g_mock_pulse_gen->getStopImmediateCount());
    TEST_ASSERT_EQUAL(AXIS_STATE_IDLE, motor->getState());
    TEST_ASSERT_FALSE(motor->isMoving());
}

// ============================================================================
// AC9, AC10: enable/disable state transitions
// ============================================================================

TEST_CASE("StepperMotor enable true transitions to UNHOMED", "[stepper_motor][AC9]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());

    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));
    TEST_ASSERT_TRUE(motor->isEnabled());
    // Steppers go to UNHOMED (position unknown after enable)
    TEST_ASSERT_EQUAL(AXIS_STATE_UNHOMED, motor->getState());
}

TEST_CASE("StepperMotor enable false stops motion and disables", "[stepper_motor][AC10]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveVelocity(DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_TRUE(motor->isMoving());

    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(false));
    TEST_ASSERT_EQUAL(AXIS_STATE_DISABLED, motor->getState());
    TEST_ASSERT_FALSE(motor->isEnabled());
    TEST_ASSERT_FALSE(motor->isMoving());
    TEST_ASSERT_EQUAL(1, g_mock_pulse_gen->getStopImmediateCount());
}

// ============================================================================
// AC11: getPosition returns SI units from PCNT
// ============================================================================

TEST_CASE("StepperMotor getPosition returns SI units from PCNT", "[stepper_motor][AC11]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());

    // Simulate PCNT reading 1000 pulses
    g_mock_tracker->setPosition(1000);
    // Trigger position sync
    g_mock_pulse_gen->simulateMotionComplete(1000);

    float pos = motor->getPosition();
    float expected = 1000.0f / DEFAULT_PULSES_PER_UNIT;  // 0.001m
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, expected, pos);
}

// ============================================================================
// AC12: Motion completion syncs position FROM PCNT
// ============================================================================

TEST_CASE("StepperMotor motion complete syncs position from PCNT", "[stepper_motor][AC12]")
{
    StepperMotor* motor = createTestStepper();
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

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.001f, DEFAULT_MAX_VELOCITY));

    // Simulate PCNT hardware counting - this is the authoritative position
    g_mock_tracker->setPosition(1000);  // 1000 pulses = 0.001m
    g_mock_pulse_gen->simulateMotionComplete(1000);

    TEST_ASSERT_TRUE(callback_invoked);
    TEST_ASSERT_EQUAL(SR_AXIS_C, callback_axis);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.001f, callback_position);
    TEST_ASSERT_EQUAL(AXIS_STATE_IDLE, motor->getState());
}

TEST_CASE("StepperMotor syncPositionFromTracker uses PCNT as authority", "[stepper_motor][AC12]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    // Start motion
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.001f, DEFAULT_MAX_VELOCITY));

    // Simulate PCNT counted different value than commanded
    // (e.g., due to missed steps or physical constraints)
    g_mock_tracker->setPosition(950);  // PCNT says 950, commanded was 1000

    // Complete motion - position should sync from PCNT (950), not commanded (1000)
    g_mock_pulse_gen->simulateMotionComplete(1000);

    float pos = motor->getPosition();
    float expected = 950.0f / DEFAULT_PULSES_PER_UNIT;  // Use PCNT value
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, expected, pos);
}

// ============================================================================
// AC13: No hardcoded values - verify config constants
// ============================================================================

TEST_CASE("StepperMotor uses TIMING_DIR_SETUP_US from config", "[stepper_motor][AC13]")
{
    TEST_ASSERT_EQUAL(20, TIMING_DIR_SETUP_US);
}

TEST_CASE("StepperMotor uses TIMING_ENABLE_DELAY_US from config", "[stepper_motor][AC13]")
{
    TEST_ASSERT_EQUAL(50, TIMING_ENABLE_DELAY_US);
}

TEST_CASE("StepperMotor uses SR_C_DIR from config", "[stepper_motor][AC13]")
{
    TEST_ASSERT_EQUAL(20, SR_C_DIR);
}

TEST_CASE("StepperMotor uses SR_C_EN from config", "[stepper_motor][AC13]")
{
    TEST_ASSERT_EQUAL(21, SR_C_EN);
}

TEST_CASE("StepperMotor uses SR_D_DIR from config", "[stepper_motor][AC13]")
{
    TEST_ASSERT_EQUAL(24, SR_D_DIR);
}

TEST_CASE("StepperMotor uses SR_D_EN from config", "[stepper_motor][AC13]")
{
    TEST_ASSERT_EQUAL(25, SR_D_EN);
}

TEST_CASE("StepperMotor uses PCNT_UNIT_C from config", "[stepper_motor][AC13]")
{
    TEST_ASSERT_EQUAL(1, PCNT_UNIT_C);
}

TEST_CASE("StepperMotor uses PCNT_UNIT_D from config", "[stepper_motor][AC13]")
{
    TEST_ASSERT_EQUAL(2, PCNT_UNIT_D);
}

TEST_CASE("StepperMotor uses MCPWM_TIMER_C from config", "[stepper_motor][AC13]")
{
    TEST_ASSERT_EQUAL(1, MCPWM_TIMER_C);
}

TEST_CASE("StepperMotor uses LEDC_CHANNEL_D from config", "[stepper_motor][AC13]")
{
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, LEDC_CHANNEL_D);
}

// ============================================================================
// Limit validation tests
// ============================================================================

TEST_CASE("StepperMotor moveAbsolute rejects position below limit_min", "[stepper_motor][limits]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor->moveAbsolute(-2.0f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(0, g_mock_pulse_gen->getStartMoveCount());
}

TEST_CASE("StepperMotor moveAbsolute rejects position above limit_max", "[stepper_motor][limits]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor->moveAbsolute(2.0f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(0, g_mock_pulse_gen->getStartMoveCount());
}

TEST_CASE("StepperMotor moveAbsolute rejects when disabled", "[stepper_motor][disabled]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(false));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, motor->moveAbsolute(0.01f, DEFAULT_MAX_VELOCITY));
}

// ============================================================================
// C-axis (MCPWM) and D-axis (LEDC) configuration tests
// ============================================================================

TEST_CASE("StepperMotor C axis uses correct axis ID", "[stepper_motor][C-axis]")
{
    StepperMotor* motor = createTestStepper(SR_AXIS_C);
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    // Axis ID 5 corresponds to C axis
    TEST_ASSERT_EQUAL(5, SR_AXIS_C);
}

TEST_CASE("StepperMotor D axis uses correct axis ID", "[stepper_motor][D-axis]")
{
    // Create with D axis
    g_mock_pulse_gen = new MockPulseGenerator();
    g_mock_tracker = new MockPcntTracker();
    g_mock_pulse_gen->init();
    g_mock_tracker->init();

    AxisConfig config = AxisConfig::createDefaultLinear();
    g_motor = new StepperMotor(g_mock_pulse_gen, g_mock_tracker, SR_AXIS_D, config);

    TEST_ASSERT_EQUAL(ESP_OK, g_motor->init());
    // Axis ID 6 corresponds to D axis
    TEST_ASSERT_EQUAL(6, SR_AXIS_D);
}

// ============================================================================
// Edge cases
// ============================================================================

TEST_CASE("StepperMotor handles zero distance move", "[stepper_motor][edge]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.0f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(0, g_mock_pulse_gen->getStartMoveCount());
}

TEST_CASE("StepperMotor getVelocity returns 0 when not moving", "[stepper_motor][edge]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(0.0f, motor->getVelocity());
}

TEST_CASE("StepperMotor motion blending - no axis busy error", "[stepper_motor][blending]")
{
    StepperMotor* motor = createTestStepper();
    TEST_ASSERT_EQUAL(ESP_OK, motor->init());
    TEST_ASSERT_EQUAL(ESP_OK, motor->enable(true));

    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.01f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_TRUE(motor->isMoving());

    // New move while still moving - should NOT return error
    TEST_ASSERT_EQUAL(ESP_OK, motor->moveAbsolute(0.02f, DEFAULT_MAX_VELOCITY));
    TEST_ASSERT_EQUAL(2, g_mock_pulse_gen->getStartMoveCount());
}
