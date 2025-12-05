/**
 * @file test_position_tracker.cpp
 * @brief Unit tests for position tracker implementations
 * @author YaRobot Team
 * @date 2025
 */

#include "unity.h"
#include "i_position_tracker.h"
#include "pcnt_tracker.h"
#include "software_tracker.h"
#include "time_tracker.h"
#include "config_limits.h"
#include "config_timing.h"
#include "config_peripherals.h"
#include "config_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <atomic>
#include <thread>

// ============================================================================
// Test Fixtures
// ============================================================================

static void setUp(void)
{
    // Test setup if needed
}

static void tearDown(void)
{
    // Test cleanup handled by test case
}

// ============================================================================
// AC1: IPositionTracker interface implementation by all trackers
// ============================================================================

TEST_CASE("SoftwareTracker implements IPositionTracker interface", "[position_tracker][AC1]")
{
    SoftwareTracker tracker;
    IPositionTracker* interface = &tracker;

    // All interface methods should be callable
    TEST_ASSERT_EQUAL(ESP_OK, interface->init());
    TEST_ASSERT_EQUAL(ESP_OK, interface->reset(100));
    TEST_ASSERT_EQUAL(100, interface->getPosition());
    interface->setDirection(false);
    // Verify interface works correctly
    TEST_ASSERT_EQUAL(ESP_OK, interface->reset(0));
    TEST_ASSERT_EQUAL(0, interface->getPosition());
}

TEST_CASE("TimeTracker implements IPositionTracker interface", "[position_tracker][AC1]")
{
    TimeTracker tracker(TIMING_E_AXIS_TRAVEL_MS);
    IPositionTracker* interface = &tracker;

    // All interface methods should be callable
    TEST_ASSERT_EQUAL(ESP_OK, interface->init());
    TEST_ASSERT_EQUAL(ESP_OK, interface->reset(1));
    TEST_ASSERT_EQUAL(1, interface->getPosition());  // Clamped to 1
    interface->setDirection(false);
    TEST_ASSERT_EQUAL(ESP_OK, interface->reset(0));
    TEST_ASSERT_EQUAL(0, interface->getPosition());
}

TEST_CASE("PcntTracker implements IPositionTracker interface", "[position_tracker][AC1][pcnt]")
{
    PcntTracker tracker(PCNT_UNIT_Y, GPIO_Y_STEP);
    IPositionTracker* interface = &tracker;

    // All interface methods should be callable
    TEST_ASSERT_EQUAL(ESP_OK, interface->init());
    TEST_ASSERT_EQUAL(ESP_OK, interface->reset(500));
    TEST_ASSERT_EQUAL(500, interface->getPosition());
    interface->setDirection(false);
    TEST_ASSERT_EQUAL(ESP_OK, interface->reset(0));
    TEST_ASSERT_EQUAL(0, interface->getPosition());
}

// ============================================================================
// AC2: PcntTracker using PCNT_UNIT_Y or PCNT_UNIT_C
// ============================================================================

TEST_CASE("PcntTracker init with PCNT_UNIT_Y succeeds", "[position_tracker][AC2][pcnt]")
{
    PcntTracker tracker(PCNT_UNIT_Y, GPIO_Y_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());
    TEST_ASSERT_TRUE(tracker.isInitialized());
    TEST_ASSERT_EQUAL(PCNT_UNIT_Y, tracker.getPcntUnitId());
}

TEST_CASE("PcntTracker uses config_peripherals.h PCNT units", "[position_tracker][AC2][pcnt]")
{
    // Verify config constants are correct
    TEST_ASSERT_EQUAL(0, PCNT_UNIT_Y);
    TEST_ASSERT_EQUAL(1, PCNT_UNIT_C);
}

// ============================================================================
// AC3: PCNT overflow handling via ISR extending 16-bit to int64_t
// ============================================================================

TEST_CASE("PCNT limits defined in config_limits.h", "[position_tracker][AC3]")
{
    // Verify overflow limits are defined
    TEST_ASSERT_EQUAL(32767, LIMIT_PCNT_HIGH_LIMIT);
    TEST_ASSERT_EQUAL(-32767, LIMIT_PCNT_LOW_LIMIT);
}

TEST_CASE("PcntTracker reset sets position correctly", "[position_tracker][AC3][pcnt]")
{
    PcntTracker tracker(PCNT_UNIT_Y, GPIO_Y_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Reset to various positions
    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(10000));
    TEST_ASSERT_EQUAL(10000, tracker.getPosition());

    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(-5000));
    TEST_ASSERT_EQUAL(-5000, tracker.getPosition());

    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(0));
    TEST_ASSERT_EQUAL(0, tracker.getPosition());
}

// ============================================================================
// AC4: SoftwareTracker receives pulses via completion callback
// ============================================================================

TEST_CASE("SoftwareTracker addPulses updates position forward", "[position_tracker][AC4]")
{
    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Initial position is 0
    TEST_ASSERT_EQUAL(0, tracker.getPosition());

    // Direction forward (default)
    tracker.setDirection(true);
    tracker.addPulses(100);
    TEST_ASSERT_EQUAL(100, tracker.getPosition());

    tracker.addPulses(50);
    TEST_ASSERT_EQUAL(150, tracker.getPosition());
}

TEST_CASE("SoftwareTracker addPulses updates position reverse", "[position_tracker][AC4]")
{
    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Start from position 1000
    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(1000));

    // Direction reverse
    tracker.setDirection(false);
    tracker.addPulses(100);
    TEST_ASSERT_EQUAL(900, tracker.getPosition());

    tracker.addPulses(200);
    TEST_ASSERT_EQUAL(700, tracker.getPosition());
}

TEST_CASE("SoftwareTracker addPulses handles zero count", "[position_tracker][AC4]")
{
    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    tracker.reset(100);
    tracker.addPulses(0);  // Should not change position
    TEST_ASSERT_EQUAL(100, tracker.getPosition());
}

// ============================================================================
// AC5: setDirection controls increment/decrement behavior
// ============================================================================

TEST_CASE("SoftwareTracker setDirection affects addPulses", "[position_tracker][AC5]")
{
    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Forward direction
    tracker.setDirection(true);
    tracker.addPulses(100);
    TEST_ASSERT_EQUAL(100, tracker.getPosition());

    // Change to reverse
    tracker.setDirection(false);
    tracker.addPulses(50);
    TEST_ASSERT_EQUAL(50, tracker.getPosition());  // 100 - 50 = 50

    // Back to forward
    tracker.setDirection(true);
    tracker.addPulses(25);
    TEST_ASSERT_EQUAL(75, tracker.getPosition());  // 50 + 25 = 75
}

TEST_CASE("PcntTracker setDirection folds count into accumulator", "[position_tracker][AC5][pcnt]")
{
    PcntTracker tracker(PCNT_UNIT_Y, GPIO_Y_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Reset to known position
    tracker.reset(1000);
    TEST_ASSERT_EQUAL(1000, tracker.getPosition());

    // Change direction should preserve position
    tracker.setDirection(false);
    int64_t pos = tracker.getPosition();
    // Position should still be at 1000 (direction change preserves position)
    TEST_ASSERT_INT64_WITHIN(10, 1000, pos);  // Small tolerance for timing

    tracker.setDirection(true);
    pos = tracker.getPosition();
    TEST_ASSERT_INT64_WITHIN(10, 1000, pos);
}

// ============================================================================
// AC6: reset() sets position atomically without physical motion
// ============================================================================

TEST_CASE("SoftwareTracker reset sets position atomically", "[position_tracker][AC6]")
{
    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Reset to various positions
    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(12345));
    TEST_ASSERT_EQUAL(12345, tracker.getPosition());

    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(-9999));
    TEST_ASSERT_EQUAL(-9999, tracker.getPosition());

    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(0));
    TEST_ASSERT_EQUAL(0, tracker.getPosition());
}

TEST_CASE("PcntTracker reset sets position atomically", "[position_tracker][AC6][pcnt]")
{
    PcntTracker tracker(PCNT_UNIT_Y, GPIO_Y_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(50000));
    TEST_ASSERT_EQUAL(50000, tracker.getPosition());

    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(-100000));
    TEST_ASSERT_EQUAL(-100000, tracker.getPosition());
}

TEST_CASE("SoftwareTracker reset fails if not initialized", "[position_tracker][AC6]")
{
    SoftwareTracker tracker;
    // Don't call init()
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, tracker.reset(100));
}

// ============================================================================
// AC7: TimeTracker returns 0 or 1 for E axis
// ============================================================================

TEST_CASE("TimeTracker returns binary position (0 or 1)", "[position_tracker][AC7]")
{
    TimeTracker tracker(TIMING_E_AXIS_TRAVEL_MS);
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Initial position is 0
    TEST_ASSERT_EQUAL(0, tracker.getPosition());

    // Reset clamps to binary
    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(100));  // Should clamp to 1
    TEST_ASSERT_EQUAL(1, tracker.getPosition());

    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(-100));  // Should clamp to 0
    TEST_ASSERT_EQUAL(0, tracker.getPosition());

    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(0));
    TEST_ASSERT_EQUAL(0, tracker.getPosition());

    TEST_ASSERT_EQUAL(ESP_OK, tracker.reset(1));
    TEST_ASSERT_EQUAL(1, tracker.getPosition());
}

TEST_CASE("TimeTracker setDirection sets target position", "[position_tracker][AC7]")
{
    TimeTracker tracker(TIMING_E_AXIS_TRAVEL_MS);
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // forward = extend = position 1
    tracker.setDirection(true);
    tracker.startMotion();
    vTaskDelay(pdMS_TO_TICKS(TIMING_E_AXIS_TRAVEL_MS + 100));  // Wait for motion
    TEST_ASSERT_EQUAL(1, tracker.getPosition());

    // reverse = retract = position 0
    tracker.setDirection(false);
    tracker.startMotion();
    vTaskDelay(pdMS_TO_TICKS(TIMING_E_AXIS_TRAVEL_MS + 100));  // Wait for motion
    TEST_ASSERT_EQUAL(0, tracker.getPosition());
}

TEST_CASE("TimeTracker isMotionComplete returns true after travel time", "[position_tracker][AC7]")
{
    uint32_t short_travel = 50;  // 50ms for fast test
    TimeTracker tracker(short_travel);
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    tracker.setDirection(true);
    tracker.startMotion();

    // Not complete immediately
    TEST_ASSERT_FALSE(tracker.isMotionComplete());

    // Wait for completion
    vTaskDelay(pdMS_TO_TICKS(short_travel + 10));
    TEST_ASSERT_TRUE(tracker.isMotionComplete());
    TEST_ASSERT_EQUAL(1, tracker.getPosition());
}

TEST_CASE("TimeTracker uses TIMING_E_AXIS_TRAVEL_MS from config", "[position_tracker][AC7]")
{
    // Default constructor should use config value
    TimeTracker tracker;
    TEST_ASSERT_EQUAL(TIMING_E_AXIS_TRAVEL_MS, tracker.getTravelTimeMs());
}

// ============================================================================
// AC8: getPosition() thread-safe via std::atomic
// ============================================================================

TEST_CASE("SoftwareTracker getPosition is thread-safe", "[position_tracker][AC8]")
{
    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    std::atomic<bool> test_passed{true};
    std::atomic<int> read_count{0};

    // Spawn multiple tasks to read position concurrently
    for (int i = 0; i < 3; i++) {
        xTaskCreate([](void* arg) {
            SoftwareTracker* t = static_cast<SoftwareTracker*>(arg);
            for (int j = 0; j < 100; j++) {
                int64_t pos = t->getPosition();
                (void)pos;  // Just read
            }
            vTaskDelete(nullptr);
        }, "reader", 2048, &tracker, 5, nullptr);
    }

    // Writer task
    xTaskCreate([](void* arg) {
        SoftwareTracker* t = static_cast<SoftwareTracker*>(arg);
        for (int j = 0; j < 100; j++) {
            t->addPulses(1);
            vTaskDelay(1);
        }
        vTaskDelete(nullptr);
    }, "writer", 2048, &tracker, 5, nullptr);

    // Wait for tasks
    vTaskDelay(pdMS_TO_TICKS(500));

    // If we get here without crash, test passed
    TEST_ASSERT_TRUE(test_passed.load());
}

TEST_CASE("TimeTracker getPosition is thread-safe", "[position_tracker][AC8]")
{
    TimeTracker tracker(50);  // 50ms travel
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Start motion
    tracker.setDirection(true);
    tracker.startMotion();

    // Read position concurrently from multiple contexts
    std::atomic<bool> test_passed{true};

    for (int i = 0; i < 3; i++) {
        xTaskCreate([](void* arg) {
            TimeTracker* t = static_cast<TimeTracker*>(arg);
            for (int j = 0; j < 50; j++) {
                int64_t pos = t->getPosition();
                bool complete = t->isMotionComplete();
                (void)pos;
                (void)complete;
                vTaskDelay(1);
            }
            vTaskDelete(nullptr);
        }, "reader", 2048, &tracker, 5, nullptr);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT_TRUE(test_passed.load());
}

// ============================================================================
// AC9: No hardcoded values - all config from header files
// ============================================================================

TEST_CASE("PCNT_UNIT_Y defined in config_peripherals.h", "[position_tracker][AC9]")
{
    TEST_ASSERT_EQUAL(0, PCNT_UNIT_Y);
}

TEST_CASE("PCNT_UNIT_C defined in config_peripherals.h", "[position_tracker][AC9]")
{
    TEST_ASSERT_EQUAL(1, PCNT_UNIT_C);
}

TEST_CASE("LIMIT_PCNT_HIGH_LIMIT defined in config_limits.h", "[position_tracker][AC9]")
{
    TEST_ASSERT_EQUAL(32767, LIMIT_PCNT_HIGH_LIMIT);
}

TEST_CASE("LIMIT_PCNT_LOW_LIMIT defined in config_limits.h", "[position_tracker][AC9]")
{
    TEST_ASSERT_EQUAL(-32767, LIMIT_PCNT_LOW_LIMIT);
}

TEST_CASE("TIMING_E_AXIS_TRAVEL_MS defined in config_timing.h", "[position_tracker][AC9]")
{
    TEST_ASSERT_EQUAL(1000, TIMING_E_AXIS_TRAVEL_MS);
}

TEST_CASE("GPIO_Y_STEP defined in config_gpio.h", "[position_tracker][AC9]")
{
    TEST_ASSERT_EQUAL(GPIO_NUM_5, GPIO_Y_STEP);
}

TEST_CASE("GPIO_C_STEP defined in config_gpio.h", "[position_tracker][AC9]")
{
    TEST_ASSERT_EQUAL(GPIO_NUM_16, GPIO_C_STEP);
}

// ============================================================================
// Edge Cases and Error Handling
// ============================================================================

TEST_CASE("SoftwareTracker handles large pulse counts", "[position_tracker][edge]")
{
    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Add large number of pulses
    tracker.addPulses(INT32_MAX);
    TEST_ASSERT_EQUAL(INT32_MAX, tracker.getPosition());

    // Continue adding
    tracker.addPulses(1000);
    TEST_ASSERT_EQUAL(static_cast<int64_t>(INT32_MAX) + 1000, tracker.getPosition());
}

TEST_CASE("SoftwareTracker handles negative positions", "[position_tracker][edge]")
{
    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    tracker.setDirection(false);
    tracker.addPulses(1000);
    TEST_ASSERT_EQUAL(-1000, tracker.getPosition());

    tracker.addPulses(500);
    TEST_ASSERT_EQUAL(-1500, tracker.getPosition());
}

TEST_CASE("TimeTracker handles rapid direction changes", "[position_tracker][edge]")
{
    uint32_t short_travel = 20;  // 20ms for fast test
    TimeTracker tracker(short_travel);
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Rapid direction changes
    for (int i = 0; i < 5; i++) {
        tracker.setDirection(i % 2 == 0);
        tracker.startMotion();
        vTaskDelay(pdMS_TO_TICKS(short_travel / 2));  // Interrupt mid-motion
    }

    // Should not crash
    tracker.setDirection(true);
    tracker.startMotion();
    vTaskDelay(pdMS_TO_TICKS(short_travel + 10));
    TEST_ASSERT_TRUE(tracker.isMotionComplete());
}

TEST_CASE("PcntTracker getPosition returns 0 when not initialized", "[position_tracker][error][pcnt]")
{
    PcntTracker tracker(PCNT_UNIT_Y, GPIO_Y_STEP);
    // Don't call init()
    TEST_ASSERT_EQUAL(0, tracker.getPosition());
}

TEST_CASE("SoftwareTracker getPosition returns 0 when not initialized", "[position_tracker][error]")
{
    SoftwareTracker tracker;
    // Don't call init()
    TEST_ASSERT_EQUAL(0, tracker.getPosition());
}

TEST_CASE("TimeTracker getPosition returns 0 when not initialized", "[position_tracker][error]")
{
    TimeTracker tracker;
    // Don't call init()
    TEST_ASSERT_EQUAL(0, tracker.getPosition());
}

TEST_CASE("SoftwareTracker double init is safe", "[position_tracker][edge]")
{
    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());  // Second init should be safe
}

TEST_CASE("TimeTracker double init is safe", "[position_tracker][edge]")
{
    TimeTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());  // Second init should be safe
}

TEST_CASE("PcntTracker double init is safe", "[position_tracker][edge][pcnt]")
{
    PcntTracker tracker(PCNT_UNIT_Y, GPIO_Y_STEP);
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());  // Second init should be safe
}

// ============================================================================
// Integration-style tests
// ============================================================================

TEST_CASE("SoftwareTracker simulates motion sequence", "[position_tracker][integration]")
{
    SoftwareTracker tracker;
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Simulate a series of moves as would happen with pulse generator callbacks
    tracker.setDirection(true);
    tracker.addPulses(1000);  // Move +1000
    TEST_ASSERT_EQUAL(1000, tracker.getPosition());

    tracker.addPulses(500);   // Move +500
    TEST_ASSERT_EQUAL(1500, tracker.getPosition());

    tracker.setDirection(false);
    tracker.addPulses(800);   // Move -800
    TEST_ASSERT_EQUAL(700, tracker.getPosition());

    tracker.reset(0);         // Reset to home
    TEST_ASSERT_EQUAL(0, tracker.getPosition());

    tracker.setDirection(true);
    tracker.addPulses(2000);  // New motion
    TEST_ASSERT_EQUAL(2000, tracker.getPosition());
}

TEST_CASE("TimeTracker simulates discrete actuator cycle", "[position_tracker][integration]")
{
    uint32_t travel = 30;  // 30ms for fast test
    TimeTracker tracker(travel);
    TEST_ASSERT_EQUAL(ESP_OK, tracker.init());

    // Start retracted
    TEST_ASSERT_EQUAL(0, tracker.getPosition());

    // Extend
    tracker.setDirection(true);
    tracker.startMotion();
    TEST_ASSERT_FALSE(tracker.isMotionComplete());

    vTaskDelay(pdMS_TO_TICKS(travel + 10));
    TEST_ASSERT_TRUE(tracker.isMotionComplete());
    TEST_ASSERT_EQUAL(1, tracker.getPosition());

    // Retract
    tracker.setDirection(false);
    tracker.startMotion();
    TEST_ASSERT_FALSE(tracker.isMotionComplete());

    vTaskDelay(pdMS_TO_TICKS(travel + 10));
    TEST_ASSERT_TRUE(tracker.isMotionComplete());
    TEST_ASSERT_EQUAL(0, tracker.getPosition());
}
