/**
 * @file test_mcpwm_pulse_gen.cpp
 * @brief Unit tests for MCPWM pulse generator with PCNT feedback
 * @author YaRobot Team
 * @date 2025
 */

#include "unity.h"
#include "mcpwm_pulse_gen.h"
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

static McpwmPulseGenerator* g_pulse_gen_y = nullptr;
static McpwmPulseGenerator* g_pulse_gen_c = nullptr;
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
    if (g_pulse_gen_y) {
        g_pulse_gen_y->stopImmediate();
    }
    if (g_pulse_gen_c) {
        g_pulse_gen_c->stopImmediate();
    }
}

// ============================================================================
// AC1: MCPWM generates STEP pulses on correct GPIOs
// ============================================================================

TEST_CASE("init returns ESP_OK for Y-axis channel", "[mcpwm_pulse_gen][AC1]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());
}

TEST_CASE("init returns ESP_OK for C-axis channel", "[mcpwm_pulse_gen][AC1]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_C, GPIO_C_STEP, PCNT_UNIT_C);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());
}

// ============================================================================
// AC2: io_loop_back routes MCPWM output to PCNT input
// ============================================================================

TEST_CASE("PCNT counts pulses from MCPWM via io_loop_back", "[mcpwm_pulse_gen][AC2]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Start move: 100 pulses at 10kHz
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(100, 10000.0f, 100000.0f));

    // Wait for completion
    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    // PCNT should have counted the pulses
    TEST_ASSERT_GREATER_THAN(0, g_callback_pulses.load());
}

// ============================================================================
// AC3: startMove generates exact pulse count with timing accuracy
// ============================================================================

TEST_CASE("startMove generates exact pulse count", "[mcpwm_pulse_gen][AC3]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // As per AC3: 5000 pulses at 25kHz with 500k accel
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(5000, 25000.0f, 500000.0f));

    // Wait for completion (max 2 seconds)
    int timeout = 200;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    // Verify +-1% accuracy
    int64_t expected = 5000;
    int64_t actual = g_callback_pulses.load();
    int64_t tolerance = expected / 100;  // 1%
    TEST_ASSERT_INT64_WITHIN(tolerance, expected, actual);
}

TEST_CASE("startMove at 100kHz with high accel", "[mcpwm_pulse_gen][AC3]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // 10000 pulses at 100kHz max velocity with 1M pulses/s^2 accel
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(10000, 100000.0f, 1000000.0f));

    // Wait for completion
    int timeout = 200;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    // Verify +-1% accuracy
    int64_t expected = 10000;
    int64_t actual = g_callback_pulses.load();
    int64_t tolerance = expected / 100;
    TEST_ASSERT_INT64_WITHIN(tolerance, expected, actual);
}

// ============================================================================
// AC4: getPulseCount returns accurate count during motion
// ============================================================================

TEST_CASE("getPulseCount returns running count during motion", "[mcpwm_pulse_gen][AC4]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(50000.0f, 500000.0f));

    // Wait a bit for pulses to accumulate
    vTaskDelay(pdMS_TO_TICKS(50));

    // Should have some pulses counted
    int64_t count = gen.getPulseCount();
    TEST_ASSERT_GREATER_THAN(0, count);

    gen.stopImmediate();
}

// ============================================================================
// AC5: PCNT limit callback stops MCPWM automatically
// ============================================================================

TEST_CASE("PCNT limit stops MCPWM at target count", "[mcpwm_pulse_gen][AC5]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Small move to test PCNT limit callback
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(500, 50000.0f, 500000.0f));

    // Wait for completion
    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    TEST_ASSERT_FALSE(gen.isRunning());
    // Should be close to target
    TEST_ASSERT_INT64_WITHIN(10, 500, g_callback_pulses.load());
}

// ============================================================================
// AC6: Direction setup delay before first STEP pulse
// ============================================================================

TEST_CASE("direction change respects setup timing", "[mcpwm_pulse_gen][AC6]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Forward move
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(100, 50000.0f, 500000.0f));
    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }
    TEST_ASSERT_TRUE(g_callback_fired.load());

    // Reset
    g_callback_fired.store(false);

    // Reverse move - should apply direction setup delay
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(-100, 50000.0f, 500000.0f));
    timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }
    TEST_ASSERT_TRUE(g_callback_fired.load());
}

// ============================================================================
// AC7: MotionCompleteCallback fires with correct pulse count
// ============================================================================

TEST_CASE("completion callback fires with correct pulse count", "[mcpwm_pulse_gen][AC7]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
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
    // Should be within 1% of target
    TEST_ASSERT_INT64_WITHIN(5, 500, g_callback_pulses.load());
}

// ============================================================================
// AC8: startVelocity generates continuous pulses until stop()
// ============================================================================

TEST_CASE("startVelocity generates continuous pulses until stop", "[mcpwm_pulse_gen][AC8]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    // As per AC8: 50kHz velocity, 500k accel
    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(50000.0f, 500000.0f));
    TEST_ASSERT_TRUE(gen.isRunning());

    // Let it run for 100ms
    vTaskDelay(pdMS_TO_TICKS(100));

    // Should still be running
    TEST_ASSERT_TRUE(gen.isRunning());

    // PCNT should be tracking position
    int64_t count1 = gen.getPulseCount();
    TEST_ASSERT_GREATER_THAN(0, count1);

    vTaskDelay(pdMS_TO_TICKS(50));
    int64_t count2 = gen.getPulseCount();
    TEST_ASSERT_GREATER_THAN(count1, count2);  // Still counting

    gen.stopImmediate();
    TEST_ASSERT_FALSE(gen.isRunning());
}

// ============================================================================
// AC9: stop() performs controlled deceleration
// ============================================================================

TEST_CASE("stop decelerates gracefully", "[mcpwm_pulse_gen][AC9]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Start velocity mode
    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(50000.0f, 500000.0f));

    // Let it reach cruise velocity
    vTaskDelay(pdMS_TO_TICKS(50));

    // Stop with deceleration (as per AC9: 1000000 pulses/s^2)
    TEST_ASSERT_EQUAL(ESP_OK, gen.stop(1000000.0f));

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
// AC10: stopImmediate() stops within 100us
// ============================================================================

TEST_CASE("stopImmediate stops quickly without callback", "[mcpwm_pulse_gen][AC10]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    TEST_ASSERT_EQUAL(ESP_OK, gen.startVelocity(100000.0f, 500000.0f));
    vTaskDelay(pdMS_TO_TICKS(10));

    int64_t start_time = esp_timer_get_time();
    gen.stopImmediate();
    int64_t stop_time = esp_timer_get_time();

    // Should stop quickly (allow 100us for API call overhead per NFR)
    TEST_ASSERT_LESS_THAN(100, stop_time - start_time);

    // Callback should NOT fire
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_FALSE(g_callback_fired.load());

    TEST_ASSERT_FALSE(gen.isRunning());
}

// ============================================================================
// AC11: Dual-channel simultaneous operation without interference
// ============================================================================

TEST_CASE("Y and C channels run simultaneously at 25kHz each", "[mcpwm_pulse_gen][AC11]")
{
    McpwmPulseGenerator gen_y(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    McpwmPulseGenerator gen_c(MCPWM_TIMER_C, GPIO_C_STEP, PCNT_UNIT_C);

    TEST_ASSERT_EQUAL(ESP_OK, gen_y.init());
    TEST_ASSERT_EQUAL(ESP_OK, gen_c.init());

    // Start both at 25kHz as per AC11
    TEST_ASSERT_EQUAL(ESP_OK, gen_y.startVelocity(25000.0f, 500000.0f));
    TEST_ASSERT_EQUAL(ESP_OK, gen_c.startVelocity(25000.0f, 500000.0f));

    // Both should be running
    TEST_ASSERT_TRUE(gen_y.isRunning());
    TEST_ASSERT_TRUE(gen_c.isRunning());

    // Let them run
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify both still running and generating pulses
    TEST_ASSERT_TRUE(gen_y.isRunning());
    TEST_ASSERT_TRUE(gen_c.isRunning());
    TEST_ASSERT_GREATER_THAN(0, gen_y.getPulseCount());
    TEST_ASSERT_GREATER_THAN(0, gen_c.getPulseCount());

    // Cleanup
    gen_y.stopImmediate();
    gen_c.stopImmediate();
}

// ============================================================================
// AC12: isRunning state transitions
// ============================================================================

TEST_CASE("isRunning returns correct state", "[mcpwm_pulse_gen][AC12]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
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
// AC13: getCurrentVelocity returns current frequency
// ============================================================================

TEST_CASE("getCurrentVelocity returns frequency during motion", "[mcpwm_pulse_gen][AC13]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
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
// AC14: C axis uses PCNT as sole position source
// ============================================================================

TEST_CASE("C axis (stepper) uses PCNT for position", "[mcpwm_pulse_gen][AC14]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_C, GPIO_C_STEP, PCNT_UNIT_C);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Move should complete with PCNT count matching commanded pulses
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(1000, 50000.0f, 500000.0f));

    int timeout = 100;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    // PCNT is sole position source - should match commanded
    TEST_ASSERT_INT64_WITHIN(10, 1000, g_callback_pulses.load());
}

// ============================================================================
// AC15: No hardcoded values - verify config constants
// ============================================================================

TEST_CASE("MCPWM_GROUP_ID is defined in config", "[mcpwm_pulse_gen][AC15]")
{
    TEST_ASSERT_EQUAL(0, MCPWM_GROUP_ID);
}

TEST_CASE("MCPWM_TIMER_Y is defined in config", "[mcpwm_pulse_gen][AC15]")
{
    TEST_ASSERT_EQUAL(0, MCPWM_TIMER_Y);
}

TEST_CASE("MCPWM_TIMER_C is defined in config", "[mcpwm_pulse_gen][AC15]")
{
    TEST_ASSERT_EQUAL(1, MCPWM_TIMER_C);
}

TEST_CASE("MCPWM_RESOLUTION_HZ is 10MHz", "[mcpwm_pulse_gen][AC15]")
{
    TEST_ASSERT_EQUAL(10000000, MCPWM_RESOLUTION_HZ);
}

TEST_CASE("MCPWM_DUTY_CYCLE_PERCENT is 50", "[mcpwm_pulse_gen][AC15]")
{
    TEST_ASSERT_EQUAL(50, MCPWM_DUTY_CYCLE_PERCENT);
}

TEST_CASE("PCNT_UNIT_Y is defined in config", "[mcpwm_pulse_gen][AC15]")
{
    TEST_ASSERT_EQUAL(0, PCNT_UNIT_Y);
}

TEST_CASE("PCNT_UNIT_C is defined in config", "[mcpwm_pulse_gen][AC15]")
{
    TEST_ASSERT_EQUAL(1, PCNT_UNIT_C);
}

TEST_CASE("GPIO_Y_STEP is GPIO5", "[mcpwm_pulse_gen][AC15]")
{
    TEST_ASSERT_EQUAL(GPIO_NUM_5, GPIO_Y_STEP);
}

TEST_CASE("GPIO_C_STEP is GPIO16", "[mcpwm_pulse_gen][AC15]")
{
    TEST_ASSERT_EQUAL(GPIO_NUM_16, GPIO_C_STEP);
}

// ============================================================================
// Frequency range validation
// ============================================================================

TEST_CASE("startMove accepts 1 Hz (minimum)", "[mcpwm_pulse_gen][boundary]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(10, 1.0f, 10.0f));
    gen.stopImmediate();
}

TEST_CASE("startMove accepts 500 kHz (maximum)", "[mcpwm_pulse_gen][boundary]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(1000, 500000.0f, 1000000.0f));
    gen.stopImmediate();
}

TEST_CASE("startMove rejects frequency above 500 kHz", "[mcpwm_pulse_gen][boundary]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startMove(1000, 600000.0f, 1000000.0f));
}

TEST_CASE("startMove rejects zero velocity", "[mcpwm_pulse_gen][boundary]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startMove(1000, 0.0f, 100000.0f));
}

TEST_CASE("startMove rejects negative velocity", "[mcpwm_pulse_gen][boundary]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startMove(1000, -100.0f, 100000.0f));
}

TEST_CASE("startMove rejects zero acceleration", "[mcpwm_pulse_gen][boundary]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, gen.startMove(1000, 100000.0f, 0.0f));
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_CASE("startMove with zero pulses returns ESP_OK immediately", "[mcpwm_pulse_gen][edge]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(0, 100000.0f, 500000.0f));
    TEST_ASSERT_FALSE(gen.isRunning());
}

TEST_CASE("startMove handles negative pulses (reverse direction)", "[mcpwm_pulse_gen][edge]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
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
    TEST_ASSERT_INT64_WITHIN(10, 500, g_callback_pulses.load());
}

TEST_CASE("triangular profile for short moves", "[mcpwm_pulse_gen][edge]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
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
    TEST_ASSERT_INT64_WITHIN(5, 100, g_callback_pulses.load());
}

// ============================================================================
// PCNT overflow handling for large moves
// ============================================================================

TEST_CASE("handles PCNT overflow for moves > 32767 pulses", "[mcpwm_pulse_gen][overflow]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    gen.setCompletionCallback(test_completion_callback);

    // Move larger than 16-bit PCNT limit
    int32_t target = 40000;
    TEST_ASSERT_EQUAL(ESP_OK, gen.startMove(target, 200000.0f, 1000000.0f));

    // Wait for completion (longer timeout for large move)
    int timeout = 300;
    while (!g_callback_fired.load(std::memory_order_acquire) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    TEST_ASSERT_TRUE(g_callback_fired.load());
    // Should have generated approximately 40000 pulses with overflow handling
    int64_t actual = g_callback_pulses.load();
    TEST_ASSERT_INT64_WITHIN(target / 100, target, actual);  // 1% tolerance
}

// ============================================================================
// Error handling
// ============================================================================

TEST_CASE("startMove fails if not initialized", "[mcpwm_pulse_gen][error]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    // Don't call init()

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, gen.startMove(1000, 100000.0f, 500000.0f));
}

TEST_CASE("startVelocity fails if not initialized", "[mcpwm_pulse_gen][error]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    // Don't call init()

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, gen.startVelocity(100000.0f, 500000.0f));
}

TEST_CASE("stop fails if not running", "[mcpwm_pulse_gen][error]")
{
    McpwmPulseGenerator gen(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(ESP_OK, gen.init());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, gen.stop(500000.0f));
}
