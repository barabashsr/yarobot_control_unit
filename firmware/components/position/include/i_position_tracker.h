/**
 * @file i_position_tracker.h
 * @brief Abstract interface for position tracking in motor control
 * @author YaRobot Team
 * @date 2025
 *
 * @note This interface provides a unified API for different position tracking
 *       technologies (PCNT hardware counting, software counting, time-based).
 *       All implementations must be thread-safe for concurrent access from
 *       ISR and task contexts.
 */

#ifndef I_POSITION_TRACKER_H
#define I_POSITION_TRACKER_H

#include <cstdint>
#include "esp_err.h"

/**
 * @brief Abstract interface for position tracking
 *
 * Implementations:
 * - PcntTracker: For Y, C axes using ESP32-S3 PCNT hardware with overflow handling
 * - SoftwareTracker: For X, Z, A, B, D axes tracking commanded pulses via callback
 * - TimeTracker: For E axis discrete actuator (binary 0/1 position)
 *
 * All position values are in pulses (int64_t for extended range).
 * SI unit conversion happens in the motor abstraction layer.
 *
 * Thread Safety:
 * - All getPosition() implementations must use atomic operations
 * - ISR-safe: no allocations, no FPU operations in hot path
 */
class IPositionTracker {
public:
    virtual ~IPositionTracker() = default;

    /**
     * @brief Initialize the position tracker
     *
     * Configures hardware peripherals (PCNT) or initializes software state.
     * Sets initial position to 0 and direction to forward.
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if configuration parameters are invalid
     * @return ESP_ERR_NO_MEM if memory allocation fails
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Reset position to specified value without physical motion
     *
     * Atomically sets the position counter to the specified value.
     * For PCNT: clears hardware counter and sets accumulator.
     * For Software: directly sets atomic position variable.
     * For Time: clamps to 0 or 1 (binary actuator).
     *
     * @param position New position value (default 0)
     * @return ESP_OK on success
     */
    virtual esp_err_t reset(int64_t position = 0) = 0;

    /**
     * @brief Get current position in pulses
     *
     * Thread-safe read of current position. For PCNT, this combines
     * the hardware counter value with the overflow accumulator.
     *
     * @return Current position in pulses (may be negative)
     */
    virtual int64_t getPosition() const = 0;

    /**
     * @brief Set counting direction
     *
     * Configures whether pulses increment or decrement the position.
     * Must be called BEFORE motion starts (during direction setup time).
     *
     * @param forward true = pulses increment position, false = decrement
     */
    virtual void setDirection(bool forward) = 0;

    /**
     * @brief Add pulses to position (for software-based tracking)
     *
     * Called by pulse generators to update position incrementally during motion.
     * For RMT: Called on each DMA buffer TX-done ISR callback.
     * For LEDC: Called periodically every TIMING_LEDC_POSITION_UPDATE_MS.
     *
     * Default implementation is a no-op (for PCNT/Time trackers that don't need it).
     * SoftwareTracker overrides this to update its atomic position counter.
     *
     * Thread-safe for ISR context (must use atomics only).
     *
     * @param count Number of pulses to add (always positive, direction applied internally)
     */
    virtual void addPulses(int64_t count) { (void)count; }
};

#endif // I_POSITION_TRACKER_H
