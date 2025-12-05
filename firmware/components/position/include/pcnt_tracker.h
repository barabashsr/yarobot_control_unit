/**
 * @file pcnt_tracker.h
 * @brief PCNT-based position tracker for Y and C axes
 * @author YaRobot Team
 * @date 2025
 *
 * @note Uses ESP32-S3 PCNT peripheral for hardware pulse counting.
 *       Extends 16-bit counter to int64_t via overflow ISR handling.
 *       Designed for Y-axis servo and C-axis stepper which require
 *       precise hardware-based position feedback.
 */

#ifndef PCNT_TRACKER_H
#define PCNT_TRACKER_H

#include "i_position_tracker.h"
#include "config_peripherals.h"
#include "config_limits.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include <atomic>

/**
 * @brief PCNT-based position tracker with overflow handling
 *
 * Features:
 * - Hardware pulse counting via ESP32-S3 PCNT peripheral
 * - 16-bit counter extended to int64_t via overflow ISR
 * - Watch points at ±LIMIT_PCNT_HIGH_LIMIT for overflow detection
 * - Direction control via PCNT edge mode configuration
 * - Thread-safe position reads via atomic operations
 *
 * Usage:
 * - Y axis: PcntTracker(PCNT_UNIT_Y, GPIO_Y_STEP)
 * - C axis: PcntTracker(PCNT_UNIT_C, GPIO_C_STEP)
 */
class PcntTracker : public IPositionTracker {
public:
    /**
     * @brief Construct PCNT position tracker
     *
     * @param pcnt_unit_id PCNT unit index (PCNT_UNIT_Y or PCNT_UNIT_C from config_peripherals.h)
     * @param pulse_gpio GPIO pin for pulse input (GPIO_Y_STEP or GPIO_C_STEP from config_gpio.h)
     */
    PcntTracker(int pcnt_unit_id, gpio_num_t pulse_gpio);

    ~PcntTracker() override;

    // IPositionTracker interface implementation
    esp_err_t init() override;
    esp_err_t reset(int64_t position = 0) override;
    int64_t getPosition() const override;
    void setDirection(bool forward) override;

    /**
     * @brief Get the PCNT unit ID
     * @return PCNT unit index
     */
    int getPcntUnitId() const { return pcnt_unit_id_; }

    /**
     * @brief Check if tracker is initialized
     * @return true if init() succeeded
     */
    bool isInitialized() const { return initialized_; }

private:
    // Configuration (from constructor)
    int pcnt_unit_id_;
    gpio_num_t pulse_gpio_;

    // PCNT handles
    pcnt_unit_handle_t pcnt_unit_;
    pcnt_channel_handle_t pcnt_channel_;
    bool initialized_;

    // Position tracking with overflow handling
    std::atomic<int64_t> overflow_accumulator_;  ///< Accumulated overflow counts
    std::atomic<bool> direction_;                 ///< true = forward (increment)

    // PCNT callbacks (static trampolines) - ISR SAFE
    static bool IRAM_ATTR onPcntWatch(pcnt_unit_handle_t unit,
                                       const pcnt_watch_event_data_t* event_data,
                                       void* user_data);

    // Instance callback handlers
    bool handlePcntWatchISR(const pcnt_watch_event_data_t* event_data);
};

#endif // PCNT_TRACKER_H
