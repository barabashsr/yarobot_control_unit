/**
 * @file software_tracker.cpp
 * @brief Software-based position tracker implementation
 * @author YaRobot Team
 * @date 2025
 *
 * Tracks position via atomic counters, updated by pulse generator
 * completion callbacks. No hardware dependencies.
 */

#include "software_tracker.h"
#include "esp_log.h"

static const char* TAG = "SW_TRACKER";

// ============================================================================
// Constructor
// ============================================================================

SoftwareTracker::SoftwareTracker()
    : position_(0)
    , direction_(true)
    , initialized_(false)
{
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t SoftwareTracker::init()
{
    if (initialized_) {
        ESP_LOGW(TAG, "Software tracker already initialized");
        return ESP_OK;
    }

    // Initialize state
    position_.store(0, std::memory_order_relaxed);
    direction_.store(true, std::memory_order_relaxed);

    initialized_ = true;
    ESP_LOGI(TAG, "Software tracker initialized");
    return ESP_OK;
}

// ============================================================================
// IPositionTracker Interface
// ============================================================================

esp_err_t SoftwareTracker::reset(int64_t position)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    position_.store(position, std::memory_order_release);
    ESP_LOGD(TAG, "Software tracker reset to position %lld", (long long)position);
    return ESP_OK;
}

int64_t SoftwareTracker::getPosition() const
{
    return position_.load(std::memory_order_acquire);
}

void SoftwareTracker::setDirection(bool forward)
{
    direction_.store(forward, std::memory_order_release);
}

// ============================================================================
// Pulse Addition (called from pushCommand - task context)
// ============================================================================

void SoftwareTracker::addPulses(int64_t count)
{
    // NOTE: This function is now called from task context (pushCommand)
    // No longer called from ISR - queue-based position tracking

    if (!initialized_ || count == 0) {
        return;
    }

    // Determine sign based on direction
    bool forward = direction_.load(std::memory_order_relaxed);
    int64_t delta = forward ? count : -count;

    // Atomic add
    position_.fetch_add(delta, std::memory_order_acq_rel);
}
