/**
 * @file software_tracker.h
 * @brief Software-based position tracker for RMT/LEDC axes
 * @author YaRobot Team
 * @date 2025
 *
 * @note Tracks position via pulse generator completion callback.
 *       Used for X, Z, A, B, D axes where RMT or LEDC generates pulses
 *       and reports the count to this tracker.
 */

#ifndef SOFTWARE_TRACKER_H
#define SOFTWARE_TRACKER_H

#include "i_position_tracker.h"
#include <atomic>

/**
 * @brief Software-based position tracker using atomic counters
 *
 * Features:
 * - Receives pulse counts from IPulseGenerator completion callback
 * - Atomic position storage for thread-safe access
 * - Direction flag controls increment/decrement
 * - No hardware dependencies - pure software tracking
 *
 * Usage:
 * - Create tracker for axis
 * - Set direction before motion via setDirection()
 * - Pulse generator calls addPulses() on completion
 * - Read position anytime via getPosition()
 */
class SoftwareTracker : public IPositionTracker {
public:
    /**
     * @brief Construct software position tracker
     */
    SoftwareTracker();

    ~SoftwareTracker() override = default;

    // IPositionTracker interface implementation
    esp_err_t init() override;
    esp_err_t reset(int64_t position = 0) override;
    int64_t getPosition() const override;
    void setDirection(bool forward) override;
    void addPulses(int64_t count) override;

    /**
     * @brief Check if tracker is initialized
     * @return true if init() succeeded
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Get current direction
     * @return true if forward, false if reverse
     */
    bool getDirection() const { return direction_.load(std::memory_order_relaxed); }

private:
    std::atomic<int64_t> position_;   ///< Current position in pulses
    std::atomic<bool> direction_;     ///< true = forward (increment)
    bool initialized_;                 ///< Initialization state
};

#endif // SOFTWARE_TRACKER_H
