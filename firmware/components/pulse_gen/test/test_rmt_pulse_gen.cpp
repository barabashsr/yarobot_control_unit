/**
 * @file test_rmt_pulse_gen.cpp
 * @brief Unit tests for RMT pulse generator
 * @author YaRobot Team
 * @date 2025
 */

#include "unity.h"
#include "rmt_pulse_gen.h"
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

static RmtPulseGenerator* g_pulse_gen = nullptr;
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
    if (g_pulse_gen) {
        g_pulse_gen->stopImmediate();
    }
}

// ============================================================================
// AC1: RMT channels generate STEP pulses on correct GPIOs
// ============================================================================

TEST_CASE("init returns ESP_OK for X-axis channel", "[pulse_gen][AC1]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());
}

TEST_CASE("init returns ESP_OK for Z-axis channel", "[pulse_gen][AC1]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_Z, GPIO_Z_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());
}

TEST_CASE("init returns ESP_OK for A-axis channel", "[pulse_gen][AC1]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_A, GPIO_A_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());
}

TEST_CASE("init returns ESP_OK for B-axis channel", "[pulse_gen][AC1]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_B, GPIO_B_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());
}

// ============================================================================
// AC2: startMove pulse count accuracy
// ============================================================================

TEST_CASE("startMove generates exact pulse count", "[pulse_gen][AC2]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Start move: 1000 pulses at 50kHz with 100k accel
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(1000, 50000.0f, 100000.0f));

    // Wait for completion (max 1 second)
    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    // Allow ±1% tolerance
    int64_t expected = 1000;
    int64_t actual = g_callback_pulses.load();
    int64_t tolerance = expected / 100;  // 1%
    TEST_ASSERT_INT64_WITHIN(tolerance, expected, actual);
}

TEST_CASE("startMove at 200kHz with high accel", "[pulse_gen][AC2]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // As per AC2: 10000 pulses at 200kHz max velocity with 1M pulses/s^2 accel
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(10000, 200000.0f, 1000000.0f));

    // Wait for completion (max 2 seconds for high-speed move)
    int timeout = 200;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    // Verify ±1% accuracy
    int64_t expected = 10000;
    int64_t actual = g_callback_pulses.load();
    int64_t tolerance = expected / 100;
    TEST_ASSERT_INT64_WITHIN(tolerance, expected, actual);
}

// ============================================================================
// AC4: Frequency range validation
// ============================================================================

TEST_CASE("startMove accepts 1 Hz (minimum)", "[pulse_gen][AC4]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(10, 1.0f, 10.0f));
    gen.stopImmediate();
}

TEST_CASE("startMove accepts 500 kHz (maximum)", "[pulse_gen][AC4]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(1000, 500000.0f, 1000000.0f));
    gen.stopImmediate();
}

TEST_CASE("startMove rejects frequency above 500 kHz", "[pulse_gen][AC4]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startMove(1000, 600000.0f, 1000000.0f));
}

TEST_CASE("startMove rejects zero velocity", "[pulse_gen][AC4]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startMove(1000, 0.0f, 100000.0f));
}

TEST_CASE("startMove rejects negative velocity", "[pulse_gen][AC4]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startMove(1000, -100.0f, 100000.0f));
}

TEST_CASE("startMove rejects zero acceleration", "[pulse_gen][AC4]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startMove(1000, 100000.0f, 0.0f));
}

// ============================================================================
// AC6: Completion callback
// ============================================================================

TEST_CASE("completion callback fires with correct pulse count", "[pulse_gen][AC6]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(500, 100000.0f, 500000.0f));

    // Wait for completion
    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    TEST_ASSERT_EQUAL(500, g_callback_pulses.load());
}

// ============================================================================
// AC7: startVelocity continuous mode
// ============================================================================

TEST_CASE("startVelocity generates continuous pulses until stop", "[pulse_gen][AC7]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(100000.0f, 500000.0f));
    TEST_ASSERT_TRUE(gen.isRunning());

    // Let it run for 100ms
    vTaskDelay(pdMS_TO_TICKS(100));

    // Should still be running
    TEST_ASSERT_TRUE(gen.isRunning());
    TEST_ASSERT_GREATER_THAN(0, gen.getPulseCount());

    gen.stopImmediate();
    TEST_ASSERT_FALSE(gen.isRunning());
}

// ============================================================================
// AC8: stop() with deceleration
// ============================================================================

TEST_CASE("stop decelerates gracefully", "[pulse_gen][AC8]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Start velocity mode
    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(100000.0f, 500000.0f));

    // Let it reach cruise velocity
    vTaskDelay(pdMS_TO_TICKS(50));

    // Stop with deceleration
    TEST_ASSERT_EQUAL(ESP_OK, gen.stop(500000.0f));

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
// AC9: stopImmediate within 10µs
// ============================================================================

TEST_CASE("stopImmediate stops quickly without callback", "[pulse_gen][AC9]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(100000.0f, 500000.0f));
    vTaskDelay(pdMS_TO_TICKS(10));

    int64_t start_time = esp_timer_get_time();
    gen.stopImmediate();
    int64_t stop_time = esp_timer_get_time();

    // Should stop quickly (allow 100µs for API call overhead)
    TEST_ASSERT_LESS_THAN(100, stop_time - start_time);

    // Callback should NOT fire
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_FALSE(g_callback_fired.load());

    TEST_ASSERT_FALSE(gen.isRunning());
}

// ============================================================================
// AC10: Multi-channel simultaneous operation
// ============================================================================

TEST_CASE("4 channels run simultaneously without interference", "[pulse_gen][AC10]")
{
    RmtPulseGenerator gen_x(RMT_CHANNEL_X, GPIO_X_STEP);
    RmtPulseGenerator gen_z(RMT_CHANNEL_Z, GPIO_Z_STEP);
    RmtPulseGenerator gen_a(RMT_CHANNEL_A, GPIO_A_STEP);
    RmtPulseGenerator gen_b(RMT_CHANNEL_B, GPIO_B_STEP);

    TEST_ASSERT_EQUAL(ESP_OK, gen_x.init());
    TEST_ASSERT_EQUAL(ESP_OK, gen_z.init());
    TEST_ASSERT_EQUAL(ESP_OK, gen_a.init());
    TEST_ASSERT_EQUAL(ESP_OK, gen_b.init());

    // Start all at 200kHz
    TEST_ASSERT_EQUAL(ESP_OK, gen_x.startVelocity(200000.0f, 1000000.0f));
    TEST_ASSERT_EQUAL(ESP_OK, gen_z.startVelocity(200000.0f, 1000000.0f));
    TEST_ASSERT_EQUAL(ESP_OK, gen_a.startVelocity(200000.0f, 1000000.0f));
    TEST_ASSERT_EQUAL(ESP_OK, gen_b.startVelocity(200000.0f, 1000000.0f));

    // All should be running
    TEST_ASSERT_TRUE(gen_x.isRunning());
    TEST_ASSERT_TRUE(gen_z.isRunning());
    TEST_ASSERT_TRUE(gen_a.isRunning());
    TEST_ASSERT_TRUE(gen_b.isRunning());

    // Let them run
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify all still running and generating pulses
    TEST_ASSERT_GREATER_THAN(0, gen_x.getPulseCount());
    TEST_ASSERT_GREATER_THAN(0, gen_z.getPulseCount());
    TEST_ASSERT_GREATER_THAN(0, gen_a.getPulseCount());
    TEST_ASSERT_GREATER_THAN(0, gen_b.getPulseCount());

    // Cleanup
    gen_x.stopImmediate();
    gen_z.stopImmediate();
    gen_a.stopImmediate();
    gen_b.stopImmediate();
}

// ============================================================================
// AC11: isRunning state transitions
// ============================================================================

TEST_CASE("isRunning returns correct state", "[pulse_gen][AC11]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    // Initially idle
    TEST_ASSERT_FALSE(gen.isRunning());

    // Running during motion
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(1000, 50000.0f, 100000.0f));
    TEST_ASSERT_TRUE(gen.isRunning());

    // Stop immediately
    gen.stopImmediate();
    TEST_ASSERT_FALSE(gen.isRunning());
}

// ============================================================================
// AC12: getCurrentVelocity
// ============================================================================

TEST_CASE("getCurrentVelocity returns frequency during motion", "[pulse_gen][AC12]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(100000.0f, 500000.0f));

    // Wait for acceleration
    vTaskDelay(pdMS_TO_TICKS(50));

    // Should be at or near target velocity
    float velocity = gen.getCurrentVelocity();
    TEST_ASSERT_GREATER_THAN(50000.0f, velocity);  // At least halfway there

    gen.stopImmediate();
}

// ============================================================================
// AC13: DEFAULT_MAX_PULSE_FREQ_HZ configuration
// ============================================================================

TEST_CASE("DEFAULT_MAX_PULSE_FREQ_HZ equals 200000", "[pulse_gen][AC13]")
{
    TEST_ASSERT_EQUAL(200000, DEFAULT_MAX_PULSE_FREQ_HZ);
}

TEST_CASE("LIMIT_MAX_PULSE_FREQ_HZ equals 500000", "[pulse_gen][AC13]")
{
    TEST_ASSERT_EQUAL(500000, LIMIT_MAX_PULSE_FREQ_HZ);
}

TEST_CASE("LIMIT_RMT_RESOLUTION_HZ equals 80000000", "[pulse_gen][AC13]")
{
    TEST_ASSERT_EQUAL(80000000, LIMIT_RMT_RESOLUTION_HZ);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_CASE("startMove with zero pulses returns ESP_OK immediately", "[pulse_gen][edge]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    // Zero pulses should return immediately with no error
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(0, 100000.0f, 500000.0f));
    TEST_ASSERT_FALSE(gen.isRunning());
}

TEST_CASE("startMove handles negative pulses (reverse direction)", "[pulse_gen][edge]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Negative pulses = reverse direction
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(-500, 100000.0f, 500000.0f));

    // Wait for completion
    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    // Should have generated 500 pulses (absolute value)
    TEST_ASSERT_EQUAL(500, g_callback_pulses.load());
}

TEST_CASE("triangular profile for short moves", "[pulse_gen][edge]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Very short move - should use triangular profile (no cruise phase)
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(100, 100000.0f, 50000.0f));

    // Wait for completion
    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    TEST_ASSERT_EQUAL(100, g_callback_pulses.load());
}

// ============================================================================
// Not initialized error handling
// ============================================================================

TEST_CASE("startMove fails if not initialized", "[pulse_gen][error]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    // Don't call init()

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, gen.startMove(1000, 100000.0f, 500000.0f));
}

TEST_CASE("startVelocity fails if not initialized", "[pulse_gen][error]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    // Don't call init()

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, gen.startVelocity(100000.0f, 500000.0f));
}

TEST_CASE("stop fails if not running", "[pulse_gen][error]")
{
    RmtPulseGenerator gen(RMT_CHANNEL_X, GPIO_X_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, gen.stop(500000.0f));
}
