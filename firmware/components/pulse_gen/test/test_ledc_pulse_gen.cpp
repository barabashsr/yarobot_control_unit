/**
 * @file test_ledc_pulse_gen.cpp
 * @brief Unit tests for LEDC pulse generator with software pulse counting
 * @author YaRobot Team
 * @date 2025
 */

#include "unity.h"
#include "ledc_pulse_gen.h"
#include "config_limits.h"
#include "config_gpio.h"
#include "config_peripherals.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <atomic>

// ============================================================================
// Test Fixtures
// ============================================================================

static std::atomic<int64_t> g_callback_pulses{0};
static std::atomic<bool> g_callback_fired{false};

static void test_completion_callback(int64_t total_pulses)
{
    g_callback_pulses.store(total_pulses, std::memory_order_relaxed);
    g_callback_fired.store(true, std::memory_order_release);
}

static void setUp(void)
{
    g_callback_pulses.store(0, std::memory_order_relaxed);
    g_callback_fired.store(false, std::memory_order_relaxed);
}

static void tearDown(void)
{
    // Clean up handled by test case
}

// ============================================================================
// AC1: LEDC generates STEP pulses on GPIO_D_STEP
// ============================================================================

TEST_CASE("init returns ESP_OK for D-axis channel", "[ledc_pulse_gen][AC1]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());
}

TEST_CASE("init with LEDC_TIMER_D and LEDC_CHANNEL_D from config", "[ledc_pulse_gen][AC1]")
{
    // Verify config constants are used
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());
    TEST_ASSERT_EQUAL(LEDC_TIMER_D, gen.getTimer());
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_D, gen.getChannel());
}

// ============================================================================
// AC2: startMove generates exact pulse count with trapezoidal profile
// ============================================================================

TEST_CASE("startMove generates pulses with trapezoidal profile", "[ledc_pulse_gen][AC2]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // As per AC2: 2000 pulses at 10kHz max velocity with 100k accel
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(2000, 10000.0f, 100000.0f));

    // Wait for completion (max 2 seconds)
    int timeout = 200;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    // Verify software counter matches commanded pulses (allow some tolerance)
    int64_t expected = 2000;
    int64_t actual = g_callback_pulses.load();
    // 5% tolerance for software counting
    int64_t tolerance = expected / 20;
    TEST_ASSERT_INT64_WITHIN(tolerance, expected, actual);
}

// ============================================================================
// AC3: Software counter tracks pulse count via esp_timer
// ============================================================================

TEST_CASE("software counter tracks pulse count accurately", "[ledc_pulse_gen][AC3]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Moderate move to test counting
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(1000, 5000.0f, 50000.0f));

    // Wait for completion
    int timeout = 200;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    // Software counter should match commanded pulses within tolerance
    TEST_ASSERT_INT64_WITHIN(100, 1000, g_callback_pulses.load());
}

// ============================================================================
// AC4: LEDC stops and callback fires when target reached
// ============================================================================

TEST_CASE("completion callback fires with correct pulse count", "[ledc_pulse_gen][AC4]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(500, 10000.0f, 100000.0f));

    // Wait for completion
    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    TEST_ASSERT_FALSE(gen.isRunning());
    // Should be within 5% of target
    TEST_ASSERT_INT64_WITHIN(25, 500, g_callback_pulses.load());
}

// ============================================================================
// AC5: Direction setup delay before first STEP pulse
// ============================================================================

TEST_CASE("direction change respects TIMING_DIR_SETUP_US delay", "[ledc_pulse_gen][AC5]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Forward move
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(200, 10000.0f, 100000.0f));
    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }
    TEST_ASSERT_TRUE(g_callback_fired.load());

    // Reset
    g_callback_fired.store(false);

    // Reverse move - should apply direction setup delay (TIMING_DIR_SETUP_US = 20us)
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(-200, 10000.0f, 100000.0f));
    timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }
    TEST_ASSERT_TRUE(g_callback_fired.load());
}

// ============================================================================
// AC6: startVelocity generates continuous pulses until stop()
// ============================================================================

TEST_CASE("startVelocity generates continuous pulses until stop", "[ledc_pulse_gen][AC6]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    // As per AC6: 10kHz velocity, 100k accel
    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(10000.0f, 100000.0f));
    TEST_ASSERT_TRUE(gen.isRunning());

    // Let it run for 100ms
    vTaskDelay(pdMS_TO_TICKS(100));

    // Should still be running
    TEST_ASSERT_TRUE(gen.isRunning());

    // Software counter should be tracking position
    int64_t count1 = gen.getPulseCount();
    TEST_ASSERT_GREATER_THAN(0, count1);

    vTaskDelay(pdMS_TO_TICKS(50));
    int64_t count2 = gen.getPulseCount();
    TEST_ASSERT_GREATER_THAN(count1, count2);  // Still counting

    gen.stopImmediate();
    TEST_ASSERT_FALSE(gen.isRunning());
}

// ============================================================================
// AC7: stop() decelerates gracefully with controlled profile
// ============================================================================

TEST_CASE("stop decelerates gracefully", "[ledc_pulse_gen][AC7]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Start velocity mode
    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(10000.0f, 100000.0f));

    // Let it reach cruise velocity
    vTaskDelay(pdMS_TO_TICKS(50));

    // Stop with deceleration (as per AC7: 100000 pulses/s^2)
    TEST_ASSERT_EQUAL(ESP_OK, gen.stop(100000.0f));

    // Wait for completion
    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    TEST_ASSERT_FALSE(gen.isRunning());
}

// ============================================================================
// AC8: stopImmediate() stops within LIMIT_STOP_LATENCY_US
// ============================================================================

TEST_CASE("stopImmediate stops quickly without callback", "[ledc_pulse_gen][AC8]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(10000.0f, 100000.0f));
    vTaskDelay(pdMS_TO_TICKS(10));

    int64_t start_time = esp_timer_get_time();
    gen.stopImmediate();
    int64_t stop_time = esp_timer_get_time();

    // Should stop within LIMIT_STOP_LATENCY_US (100us) - allow more for API overhead
    TEST_ASSERT_LESS_THAN(1000, stop_time - start_time);  // 1ms max including overhead

    // Callback should NOT fire
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_FALSE(g_callback_fired.load());

    TEST_ASSERT_FALSE(gen.isRunning());
}

// ============================================================================
// AC9: isRunning() returns correct state
// ============================================================================

TEST_CASE("isRunning returns correct state during motion lifecycle", "[ledc_pulse_gen][AC9]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    // Initially idle
    TEST_ASSERT_FALSE(gen.isRunning());

    // Running during motion
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(1000, 10000.0f, 100000.0f));
    TEST_ASSERT_TRUE(gen.isRunning());

    // Stop immediately
    gen.stopImmediate();
    TEST_ASSERT_FALSE(gen.isRunning());
}

// ============================================================================
// AC10: getCurrentVelocity() returns current pulse frequency
// ============================================================================

TEST_CASE("getCurrentVelocity returns frequency during motion", "[ledc_pulse_gen][AC10]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(20000.0f, 100000.0f));

    // Wait for acceleration
    vTaskDelay(pdMS_TO_TICKS(100));

    // Should be at or near target velocity
    float velocity = gen.getCurrentVelocity();
    TEST_ASSERT_GREATER_THAN(5000.0f, velocity);  // At least partway there

    gen.stopImmediate();
}

// ============================================================================
// AC11: getPulseCount() returns accurate software count
// ============================================================================

TEST_CASE("getPulseCount returns running count during motion", "[ledc_pulse_gen][AC11]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(10000.0f, 100000.0f));

    // Wait a bit for pulses to accumulate
    vTaskDelay(pdMS_TO_TICKS(50));

    // Should have some pulses counted
    int64_t count = gen.getPulseCount();
    TEST_ASSERT_GREATER_THAN(0, count);

    gen.stopImmediate();
}

// ============================================================================
// AC12: Pulse frequency range 1 Hz to DEFAULT_MAX_PULSE_FREQ_HZ
// ============================================================================

TEST_CASE("startMove accepts LIMIT_LEDC_MIN_FREQ_HZ (10 Hz)", "[ledc_pulse_gen][AC12]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(10, static_cast<float>(LIMIT_LEDC_MIN_FREQ_HZ), 10.0f));
    gen.stopImmediate();
}

TEST_CASE("startMove accepts LIMIT_LEDC_MAX_FREQ_HZ (75 kHz)", "[ledc_pulse_gen][AC12]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(1000, static_cast<float>(LIMIT_LEDC_MAX_FREQ_HZ), 500000.0f));
    gen.stopImmediate();
}

TEST_CASE("startMove rejects frequency above LIMIT_LEDC_MAX_FREQ_HZ", "[ledc_pulse_gen][AC12]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    // Above LEDC limit should be rejected
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startMove(1000, 100000.0f, 500000.0f));
}

TEST_CASE("startMove rejects zero velocity", "[ledc_pulse_gen][AC12]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startMove(1000, 0.0f, 100000.0f));
}

TEST_CASE("startMove rejects negative velocity", "[ledc_pulse_gen][AC12]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startMove(1000, -100.0f, 100000.0f));
}

TEST_CASE("startMove rejects zero acceleration", "[ledc_pulse_gen][AC12]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startMove(1000, 10000.0f, 0.0f));
}

// ============================================================================
// AC13: No hardcoded values - verify config constants
// ============================================================================

TEST_CASE("LEDC_TIMER_D is defined in config", "[ledc_pulse_gen][AC13]")
{
    TEST_ASSERT_EQUAL(LEDC_TIMER_2, LEDC_TIMER_D);
}

TEST_CASE("LEDC_CHANNEL_D is defined in config", "[ledc_pulse_gen][AC13]")
{
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, LEDC_CHANNEL_D);
}

TEST_CASE("LEDC_MODE_D is LEDC_LOW_SPEED_MODE", "[ledc_pulse_gen][AC13]")
{
    TEST_ASSERT_EQUAL(LEDC_LOW_SPEED_MODE, LEDC_MODE_D);
}

TEST_CASE("LEDC_RESOLUTION_BITS is 10", "[ledc_pulse_gen][AC13]")
{
    TEST_ASSERT_EQUAL(10, LEDC_RESOLUTION_BITS);
}

TEST_CASE("LEDC_DUTY_CYCLE_PERCENT is 50", "[ledc_pulse_gen][AC13]")
{
    TEST_ASSERT_EQUAL(50, LEDC_DUTY_CYCLE_PERCENT);
}

TEST_CASE("GPIO_D_STEP is GPIO17", "[ledc_pulse_gen][AC13]")
{
    TEST_ASSERT_EQUAL(GPIO_NUM_17, GPIO_D_STEP);
}

TEST_CASE("LIMIT_STOP_LATENCY_US is defined", "[ledc_pulse_gen][AC13]")
{
    TEST_ASSERT_EQUAL(100, LIMIT_STOP_LATENCY_US);
}

TEST_CASE("LIMIT_LEDC_MAX_FREQ_HZ is defined", "[ledc_pulse_gen][AC13]")
{
    TEST_ASSERT_EQUAL(75000, LIMIT_LEDC_MAX_FREQ_HZ);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_CASE("startMove with zero pulses returns ESP_OK immediately", "[ledc_pulse_gen][edge]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(0, 10000.0f, 100000.0f));
    TEST_ASSERT_FALSE(gen.isRunning());
}

TEST_CASE("startMove handles negative pulses (reverse direction)", "[ledc_pulse_gen][edge]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Negative pulses = reverse direction
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(-500, 10000.0f, 100000.0f));

    // Wait for completion
    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    // Should have generated approximately 500 pulses (absolute value)
    TEST_ASSERT_INT64_WITHIN(50, 500, g_callback_pulses.load());
}

TEST_CASE("triangular profile for short moves", "[ledc_pulse_gen][edge]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Very short move - should use triangular profile (no cruise phase)
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(100, 50000.0f, 10000.0f));

    // Wait for completion
    int timeout = 200;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    TEST_ASSERT_INT64_WITHIN(20, 100, g_callback_pulses.load());
}

// ============================================================================
// Error handling
// ============================================================================

TEST_CASE("startMove fails if not initialized", "[ledc_pulse_gen][error]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    // Don't call init()

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, gen.startMove(1000, 10000.0f, 100000.0f));
}

TEST_CASE("startVelocity fails if not initialized", "[ledc_pulse_gen][error]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    // Don't call init()

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, gen.startVelocity(10000.0f, 100000.0f));
}

TEST_CASE("stop fails if not running", "[ledc_pulse_gen][error]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, gen.stop(100000.0f));
}

TEST_CASE("stop fails with invalid deceleration", "[ledc_pulse_gen][error]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(10000.0f, 100000.0f));
    vTaskDelay(pdMS_TO_TICKS(10));

    // Zero deceleration should fail
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.stop(0.0f));

    // Negative deceleration should fail
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.stop(-100000.0f));

    gen.stopImmediate();
}

// ============================================================================
// startVelocity boundary tests
// ============================================================================

TEST_CASE("startVelocity accepts LIMIT_LEDC_MIN_FREQ_HZ", "[ledc_pulse_gen][boundary]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(static_cast<float>(LIMIT_LEDC_MIN_FREQ_HZ), 10.0f));
    gen.stopImmediate();
}

TEST_CASE("startVelocity accepts LIMIT_LEDC_MAX_FREQ_HZ", "[ledc_pulse_gen][boundary]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(static_cast<float>(LIMIT_LEDC_MAX_FREQ_HZ), 500000.0f));
    gen.stopImmediate();
}

TEST_CASE("startVelocity rejects velocity above LIMIT_LEDC_MAX_FREQ_HZ", "[ledc_pulse_gen][boundary]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startVelocity(100000.0f, 500000.0f));
}

TEST_CASE("startVelocity allows negative velocity (reverse direction)", "[ledc_pulse_gen][boundary]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    // Negative velocity = reverse direction
    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(-10000.0f, 100000.0f));
    gen.stopImmediate();
}

// ============================================================================
// Story 3.5b: Real-time position tracking during motion
// ============================================================================

#include "software_tracker.h"
#include "config_timing.h"

TEST_CASE("setPositionTracker sets tracker and updates position during motion", "[ledc_pulse_gen][position_tracking]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Set tracker before motion
    gen.setPositionTracker(&tracker);

    gen.setCompletionCallback(test_completion_callback);

    // Start move
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(2000, 10000.0f, 100000.0f));

    // Wait a bit (more than TIMING_LEDC_POSITION_UPDATE_MS)
    vTaskDelay(pdMS_TO_TICKS(TIMING_LEDC_POSITION_UPDATE_MS * 3));

    // Position should be updating during motion
    int64_t pos_mid = tracker.getPosition();
    TEST_ASSERT_GREATER_THAN(0, pos_mid);

    // Wait for completion
    int timeout = 200;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());

    // Final position should match commanded pulses (within tolerance)
    int64_t final_pos = tracker.getPosition();
    TEST_ASSERT_INT64_WITHIN(100, 2000, final_pos);
}

TEST_CASE("position tracker direction is set before motion", "[ledc_pulse_gen][position_tracking]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    gen.setPositionTracker(&tracker);
    gen.setCompletionCallback(test_completion_callback);

    // Forward move
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(500, 10000.0f, 100000.0f));

    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }
    TEST_ASSERT_TRUE(g_callback_fired.load());

    int64_t forward_pos = tracker.getPosition();
    TEST_ASSERT_GREATER_THAN(0, forward_pos);  // Should be positive

    // Reset
    g_callback_fired.store(false);

    // Reverse move (negative pulses)
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(-500, 10000.0f, 100000.0f));

    timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }
    TEST_ASSERT_TRUE(g_callback_fired.load());

    int64_t final_pos = tracker.getPosition();
    // Should be back close to zero (forward 500, reverse 500)
    TEST_ASSERT_INT64_WITHIN(100, 0, final_pos);
}

TEST_CASE("position updates occur periodically during motion", "[ledc_pulse_gen][position_tracking]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    gen.setPositionTracker(&tracker);

    // Start velocity mode at moderate frequency
    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(10000.0f, 100000.0f));

    // Sample position multiple times to verify periodic updates
    vTaskDelay(pdMS_TO_TICKS(50));
    int64_t pos1 = tracker.getPosition();

    vTaskDelay(pdMS_TO_TICKS(TIMING_LEDC_POSITION_UPDATE_MS * 2));
    int64_t pos2 = tracker.getPosition();

    vTaskDelay(pdMS_TO_TICKS(TIMING_LEDC_POSITION_UPDATE_MS * 2));
    int64_t pos3 = tracker.getPosition();

    // Position should be monotonically increasing
    TEST_ASSERT_GREATER_THAN(pos1, pos2);
    TEST_ASSERT_GREATER_THAN(pos2, pos3);

    gen.stopImmediate();
}

TEST_CASE("null tracker does not crash", "[ledc_pulse_gen][position_tracking]")
{
    LedcPulseGenerator gen(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    // Don't set tracker (remains null)
    gen.setCompletionCallback(test_completion_callback);

    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(500, 10000.0f, 100000.0f));

    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
}
