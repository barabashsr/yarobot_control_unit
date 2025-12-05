/**
 * @file time_tracker.h
 * @brief Time-based position tracker for E axis discrete actuator
 * @author YaRobot Team
 * @date 2025
 *
 * @note Tracks binary position (0 = retracted, 1 = extended) for discrete
 *       actuators like pneumatic cylinders or solenoids. Position is
 *       interpolated based on elapsed time during motion.
 */

#ifndef TIME_TRACKER_H
#define TIME_TRACKER_H

#include "i_position_tracker.h"
#include "config_timing.h"
#include <atomic>
#include <cstdint>

/**
 * @brief Time-based position tracker for discrete actuators
 *
 * Features:
 * - Binary position: 0 (retracted) or 1 (extended)
 * - Position interpolated during motion based on elapsed time
 * - Fixed travel time from config (TIMING_E_AXIS_TRAVEL_MS)
 * - startMotion() records start time for interpolation
 *
 * Usage:
 * - setDirection(true) for extend, setDirection(false) for retract
 * - startMotion() begins timing
 * - getPosition() returns interpolated position during motion
 * - isMotionComplete() checks if travel time elapsed
 */
class TimeTracker : public IPositionTracker {
public:
    /**
     * @brief Construct time-based position tracker
     *
     * @param travel_time_ms Time for full travel in milliseconds
     *        (default: TIMING_E_AXIS_TRAVEL_MS from config_timing.h)
     */
    explicit TimeTracker(uint32_t travel_time_ms = TIMING_E_AXIS_TRAVEL_MS);

    ~TimeTracker() override = default;

    // IPositionTracker interface implementation
    esp_err_t init() override;
    esp_err_t reset(int64_t position = 0) override;
    int64_t getPosition() const override;
    void setDirection(bool forward) override;

    /**
     * @brief Start motion timing
     *
     * Records current time as motion start. getPosition() will interpolate
     * based on elapsed time until travel_time_ms has passed.
     */
    void startMotion();

    /**
     * @brief Check if motion is complete
     *
     * @return true if travel time has elapsed since startMotion()
     */
    bool isMotionComplete() const;

    /**
     * @brief Check if tracker is initialized
     * @return true if init() succeeded
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Get configured travel time
     * @return Travel time in milliseconds
     */
    uint32_t getTravelTimeMs() const { return travel_time_ms_; }

private:
    uint32_t travel_time_ms_;              ///< Time for full travel (ms)
    std::atomic<int64_t> position_;        ///< Current position (0 or 1)
    std::atomic<int64_t> target_position_; ///< Target position (0 or 1)
    std::atomic<int64_t> motion_start_us_; ///< Motion start time (µs)
    std::atomic<bool> in_motion_;          ///< Motion in progress flag
    bool initialized_;                      ///< Initialization state
};

#endif // TIME_TRACKER_H
