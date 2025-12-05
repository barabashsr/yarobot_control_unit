/**
 * @file time_tracker.cpp
 * @brief Time-based position tracker implementation for E axis
 * @author YaRobot Team
 * @date 2025
 *
 * Tracks binary position for discrete actuators. Position is interpolated
 * during motion based on elapsed time since startMotion() was called.
 */

#include "time_tracker.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char* TAG = "TIME_TRACKER";

// Microseconds per millisecond for time conversion
static constexpr int64_t US_PER_MS = 1000;

// ============================================================================
// Constructor
// ============================================================================

TimeTracker::TimeTracker(uint32_t travel_time_ms)
    : travel_time_ms_(travel_time_ms)
    , position_(0)
    , target_position_(0)
    , motion_start_us_(0)
    , in_motion_(false)
    , initialized_(false)
{
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t TimeTracker::init()
{
    if (initialized_) {
        ESP_LOGW(TAG, "Time tracker already initialized");
        return ESP_OK;
    }

    // Initialize state - start retracted (position 0)
    position_.store(0, std::memory_order_relaxed);
    target_position_.store(0, std::memory_order_relaxed);
    motion_start_us_.store(0, std::memory_order_relaxed);
    in_motion_.store(false, std::memory_order_relaxed);

    initialized_ = true;
    ESP_LOGI(TAG, "Time tracker initialized with travel time %lu ms",
             (unsigned long)travel_time_ms_);
    return ESP_OK;
}

// ============================================================================
// IPositionTracker Interface
// ============================================================================

esp_err_t TimeTracker::reset(int64_t position)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Clamp to binary (0 or 1)
    int64_t clamped = (position > 0) ? 1 : 0;

    position_.store(clamped, std::memory_order_release);
    target_position_.store(clamped, std::memory_order_release);
    in_motion_.store(false, std::memory_order_release);

    ESP_LOGD(TAG, "Time tracker reset to position %lld", (long long)clamped);
    return ESP_OK;
}

int64_t TimeTracker::getPosition() const
{
    if (!initialized_) {
        return 0;
    }

    // If not in motion, return current position
    if (!in_motion_.load(std::memory_order_acquire)) {
        return position_.load(std::memory_order_relaxed);
    }

    // Interpolate position based on elapsed time
    int64_t start_us = motion_start_us_.load(std::memory_order_relaxed);
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = now_us - start_us;
    int64_t travel_us = static_cast<int64_t>(travel_time_ms_) * US_PER_MS;

    if (elapsed_us >= travel_us) {
        // Motion complete - return target position
        return target_position_.load(std::memory_order_relaxed);
    }

    // During motion: return target position (binary actuator either moving or at destination)
    // For binary actuator semantics, we report the target during motion
    // This is consistent with "position requested" vs "position confirmed"
    return target_position_.load(std::memory_order_relaxed);
}

void TimeTracker::setDirection(bool forward)
{
    if (!initialized_) {
        return;
    }

    // forward = true means extend (position 1)
    // forward = false means retract (position 0)
    int64_t target = forward ? 1 : 0;
    target_position_.store(target, std::memory_order_release);
}

// ============================================================================
// Motion Control
// ============================================================================

void TimeTracker::startMotion()
{
    if (!initialized_) {
        return;
    }

    // Record start time
    motion_start_us_.store(esp_timer_get_time(), std::memory_order_release);
    in_motion_.store(true, std::memory_order_release);

    ESP_LOGD(TAG, "Motion started, target=%lld",
             (long long)target_position_.load(std::memory_order_relaxed));
}

bool TimeTracker::isMotionComplete() const
{
    if (!initialized_ || !in_motion_.load(std::memory_order_acquire)) {
        return true;  // Not moving means motion is "complete"
    }

    int64_t start_us = motion_start_us_.load(std::memory_order_relaxed);
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = now_us - start_us;
    int64_t travel_us = static_cast<int64_t>(travel_time_ms_) * US_PER_MS;

    if (elapsed_us >= travel_us) {
        // Update position and clear motion flag
        // Note: This is a const method so we use mutable atomics
        int64_t target = target_position_.load(std::memory_order_relaxed);
        const_cast<std::atomic<int64_t>&>(position_).store(target, std::memory_order_release);
        const_cast<std::atomic<bool>&>(in_motion_).store(false, std::memory_order_release);
        return true;
    }

    return false;
}
