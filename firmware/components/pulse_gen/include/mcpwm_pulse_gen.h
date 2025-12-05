/**
 * @file mcpwm_pulse_gen.h
 * @brief MCPWM-based pulse generator for Y and C axes with PCNT feedback
 * @author YaRobot Team
 * @date 2025
 *
 * @note Uses ESP32-S3 MCPWM peripheral with internal GPIO loopback to PCNT
 *       for hardware pulse counting. PCNT limit callback stops MCPWM
 *       automatically when target pulse count is reached.
 *
 * Architecture:
 * - MCPWM generates PWM pulses at variable frequency for trapezoidal profile
 * - PCNT counts pulses via internal io_loop_back (no external wire needed)
 * - PCNT watch point callback stops MCPWM when target reached
 * - Completion callback fires from task context via FreeRTOS notification
 */

#ifndef MCPWM_PULSE_GEN_H
#define MCPWM_PULSE_GEN_H

#include "i_pulse_generator.h"
#include "i_position_tracker.h"
#include "config_limits.h"
#include "config_peripherals.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <atomic>

/**
 * @brief Motion profile state machine states (shared with RMT)
 */
enum class McpwmProfileState {
    IDLE,           ///< Not moving
    ACCELERATING,   ///< Ramping up velocity
    CRUISING,       ///< At constant velocity
    DECELERATING,   ///< Ramping down velocity
    STOPPING        ///< Controlled stop in progress
};

/**
 * @brief Motion mode for profile generation (shared with RMT)
 */
enum class McpwmMotionMode {
    POSITION,       ///< Fixed pulse count move
    VELOCITY        ///< Continuous velocity until stop
};

/**
 * @brief Trapezoidal motion profile parameters (shared with RMT)
 */
struct McpwmTrapezoidalProfile {
    int32_t target_pulses;      ///< Total pulses to generate (position mode)
    float max_velocity;         ///< Peak velocity (pulses/second)
    float acceleration;         ///< Acceleration rate (pulses/second^2)
    float deceleration;         ///< Deceleration rate (pulses/second^2)

    // Calculated profile segments
    int32_t accel_pulses;       ///< Pulses during acceleration phase
    int32_t cruise_pulses;      ///< Pulses during cruise phase
    int32_t decel_pulses;       ///< Pulses during deceleration phase
    float cruise_velocity;      ///< Actual cruise velocity achieved

    // Runtime state
    int64_t current_pulse;      ///< Current position within profile
    float current_velocity;     ///< Current instantaneous velocity
    bool is_triangular;         ///< True if no cruise phase (short move)
};

/**
 * @brief MCPWM-based pulse generator with PCNT feedback
 *
 * Uses MCPWM peripheral for PWM generation and PCNT for hardware pulse
 * counting via internal GPIO loopback. Designed for Y and C axes.
 *
 * Features:
 * - 10 MHz MCPWM resolution (100ns per tick)
 * - Up to 500 kHz pulse frequency
 * - 50% duty cycle pulses
 * - PCNT hardware counting with limit callback
 * - Automatic stop via PCNT watch point
 * - 64-bit pulse count via overflow tracking
 * - ISR-safe: completion callback runs in task context
 */
class McpwmPulseGenerator : public IPulseGenerator {
public:
    /**
     * @brief Construct MCPWM pulse generator
     *
     * @param timer_id MCPWM timer index (MCPWM_TIMER_Y or MCPWM_TIMER_C)
     * @param gpio_num GPIO pin for STEP output
     * @param pcnt_unit_id PCNT unit index (PCNT_UNIT_Y or PCNT_UNIT_C)
     */
    McpwmPulseGenerator(int timer_id, int gpio_num, int pcnt_unit_id);

    ~McpwmPulseGenerator() override;

    // IPulseGenerator interface implementation
    esp_err_t init() override;
    esp_err_t startMove(int32_t pulses, float max_velocity, float acceleration) override;
    esp_err_t startVelocity(float velocity, float acceleration) override;
    esp_err_t stop(float deceleration) override;
    void stopImmediate() override;
    bool isRunning() const override;
    int64_t getPulseCount() const override;
    float getCurrentVelocity() const override;
    void setCompletionCallback(MotionCompleteCallback cb) override;
    void setPositionTracker(IPositionTracker* tracker) override;

    /**
     * @brief Get the timer ID
     * @return MCPWM timer index
     */
    int getTimerId() const { return timer_id_; }

    /**
     * @brief Get the PCNT unit ID
     * @return PCNT unit index
     */
    int getPcntUnitId() const { return pcnt_unit_id_; }

private:
    // Configuration (from constructor)
    int timer_id_;
    int gpio_num_;
    int pcnt_unit_id_;

    // MCPWM handles
    mcpwm_timer_handle_t timer_handle_;
    mcpwm_oper_handle_t oper_handle_;
    mcpwm_gen_handle_t gen_handle_;
    mcpwm_cmpr_handle_t cmpr_handle_;
    bool initialized_;

    // PCNT handles
    pcnt_unit_handle_t pcnt_unit_;
    pcnt_channel_handle_t pcnt_channel_;

    // Motion state
    std::atomic<McpwmProfileState> state_;
    McpwmMotionMode mode_;
    McpwmTrapezoidalProfile profile_;
    bool direction_;  // true = forward, false = reverse
    bool last_direction_;  // For tracking direction changes

    // Pulse counting
    std::atomic<int64_t> pulse_count_;      ///< Total pulses (PCNT + overflow tracking)
    std::atomic<int32_t> overflow_count_;   ///< Number of PCNT overflows
    std::atomic<float> current_velocity_;

    // Profile update task
    TaskHandle_t profile_task_handle_;
    std::atomic<bool> task_should_exit_;

    // Callback
    MotionCompleteCallback completion_callback_;
    SemaphoreHandle_t callback_mutex_;

    // Position tracker (stored but not used - PCNT provides real-time position)
    IPositionTracker* position_tracker_;

    // Profile calculation (same as RMT)
    void calculateTrapezoidalProfile(int32_t pulses, float max_vel, float accel);
    void calculateVelocityProfile(float velocity, float acceleration);
    float velocityAtPosition(int64_t position) const;

    // MCPWM frequency control
    esp_err_t setFrequency(float frequency);
    uint32_t velocityToPeriodTicks(float velocity) const;

    // PCNT configuration
    esp_err_t configurePcntWatchPoint(int32_t target_pulses);
    int64_t readPcntCount() const;

    // Profile update task
    static void profileTaskEntry(void* arg);
    void profileTaskLoop();
    void updateProfile();

    // PCNT callbacks (static trampolines) - ISR SAFE
    static bool IRAM_ATTR onPcntReach(pcnt_unit_handle_t unit,
                                       const pcnt_watch_event_data_t* event_data,
                                       void* user_data);

    // Instance callback handlers
    bool handlePcntReachISR(const pcnt_watch_event_data_t* event_data);
    void notifyCompletion();
};

#endif // MCPWM_PULSE_GEN_H
