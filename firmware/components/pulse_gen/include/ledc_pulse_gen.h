/**
 * @file ledc_pulse_gen.h
 * @brief LEDC-based pulse generator for D-axis stepper with software pulse counting
 * @author YaRobot Team
 * @date 2025
 *
 * @note Uses ESP32-S3 LEDC peripheral for PWM generation with software-based
 *       pulse counting via high-resolution esp_timer. Designed for D-axis
 *       stepper which has lower precision requirements than servo axes.
 *
 * Architecture:
 * - LEDC generates PWM pulses at variable frequency for trapezoidal profile
 * - esp_timer counts pulses via periodic callback (no hardware PCNT)
 * - Profile update timer adjusts LEDC frequency during motion
 * - Completion callback fires from task context via FreeRTOS notification
 */

#ifndef LEDC_PULSE_GEN_H
#define LEDC_PULSE_GEN_H

#include "i_pulse_generator.h"
#include "i_position_tracker.h"
#include "config_limits.h"
#include "config_peripherals.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <atomic>

/**
 * @brief Motion profile state machine states
 */
enum class LedcProfileState {
    IDLE,           ///< Not moving
    ACCELERATING,   ///< Ramping up velocity
    CRUISING,       ///< At constant velocity
    DECELERATING,   ///< Ramping down velocity
    STOPPING        ///< Controlled stop in progress
};

/**
 * @brief Motion mode for profile generation
 */
enum class LedcMotionMode {
    POSITION,       ///< Fixed pulse count move
    VELOCITY        ///< Continuous velocity until stop
};

/**
 * @brief Trapezoidal motion profile parameters
 */
struct LedcTrapezoidalProfile {
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
 * @brief LEDC-based pulse generator with software counting
 *
 * Uses LEDC peripheral for PWM generation and esp_timer for software pulse
 * counting. Designed for D-axis stepper motor.
 *
 * Features:
 * - 10-bit LEDC resolution (1024 duty levels)
 * - Up to LIMIT_LEDC_MAX_FREQ_HZ pulse frequency
 * - 50% duty cycle pulses
 * - Software pulse counting via esp_timer
 * - Trapezoidal velocity profile with acceleration/deceleration
 * - ISR-safe: completion callback runs in task context
 */
class LedcPulseGenerator : public IPulseGenerator {
public:
    /**
     * @brief Construct LEDC pulse generator
     *
     * @param gpio_num GPIO pin for STEP output (use GPIO_D_STEP)
     * @param timer LEDC timer index (use LEDC_TIMER_D)
     * @param channel LEDC channel index (use LEDC_CHANNEL_D)
     */
    LedcPulseGenerator(int gpio_num, ledc_timer_t timer, ledc_channel_t channel);

    ~LedcPulseGenerator() override;

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
     * @brief Get the LEDC timer
     * @return LEDC timer index
     */
    ledc_timer_t getTimer() const { return timer_; }

    /**
     * @brief Get the LEDC channel
     * @return LEDC channel index
     */
    ledc_channel_t getChannel() const { return channel_; }

    /**
     * @brief Check if position tracker is set (debug helper)
     * @return true if position_tracker_ is not null
     */
    bool hasPositionTracker() const { return position_tracker_ != nullptr; }

private:
    // Configuration (from constructor)
    int gpio_num_;
    ledc_timer_t timer_;
    ledc_channel_t channel_;
    bool initialized_;

    // Motion state
    std::atomic<LedcProfileState> state_;
    LedcMotionMode mode_;
    LedcTrapezoidalProfile profile_;
    bool direction_;  // true = forward, false = reverse
    bool last_direction_;  // For tracking direction changes

    // Time-based pulse estimation (no hardware PCNT available)
    int64_t start_time_us_;              ///< Motion start time from esp_timer_get_time()
    std::atomic<float> current_velocity_;
    std::atomic<double> accumulated_pulses_;  ///< Accumulated pulse estimate (fractional)

    // Profile update timer
    esp_timer_handle_t profile_timer_;   ///< Timer for profile updates

    // Position tracker for real-time updates
    esp_timer_handle_t position_timer_;  ///< Timer for position updates (TIMING_LEDC_POSITION_UPDATE_MS)
    IPositionTracker* position_tracker_;
    std::atomic<int64_t> last_reported_pulses_;  ///< Tracks pulses reported to position tracker

    // Completion handling
    TaskHandle_t completion_task_handle_;
    std::atomic<bool> task_should_exit_;
    MotionCompleteCallback completion_callback_;
    SemaphoreHandle_t callback_mutex_;

    // Profile calculation
    void calculateTrapezoidalProfile(int32_t pulses, float max_vel, float accel);
    void calculateVelocityProfile(float velocity, float acceleration);
    float velocityAtPosition(int64_t position) const;

    // LEDC control
    esp_err_t setFrequency(uint32_t frequency);
    esp_err_t startPulseOutput();
    void stopPulseOutput();

    // Timer callbacks
    static void profileTimerCallback(void* arg);
    static void positionTimerCallback(void* arg);

    // Instance callback handlers
    void handleProfileUpdate();
    void handlePositionUpdate();

    // Completion task
    static void completionTaskEntry(void* arg);
    void completionTaskLoop();
    void notifyCompletion();
};

#endif // LEDC_PULSE_GEN_H
