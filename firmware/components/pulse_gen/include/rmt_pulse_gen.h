/**
 * @file rmt_pulse_gen.h
 * @brief RMT-based pulse generator for servo axes X, Z, A, B
 * @author YaRobot Team
 * @date 2025
 *
 * @note Uses ESP32-S3 RMT peripheral with DMA streaming for precise pulse
 *       generation up to 500 kHz. Implements streaming double-buffer
 *       architecture for unlimited motion length.
 *
 * Architecture:
 * - RMT TX done callback (ISR) signals a dedicated refill task
 * - Refill task runs at high priority to minimize latency
 * - Double-buffering ensures continuous pulse output
 * - Trapezoidal velocity profile for smooth acceleration
 */

#ifndef RMT_PULSE_GEN_H
#define RMT_PULSE_GEN_H

#include "i_pulse_generator.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <atomic>

/**
 * @brief Motion profile state machine states
 */
enum class ProfileState {
    IDLE,           ///< Not moving
    ACCELERATING,   ///< Ramping up velocity
    CRUISING,       ///< At constant velocity
    DECELERATING,   ///< Ramping down velocity
    STOPPING        ///< Controlled stop in progress
};

/**
 * @brief Motion mode for profile generation
 */
enum class MotionMode {
    POSITION,       ///< Fixed pulse count move
    VELOCITY        ///< Continuous velocity until stop
};

/**
 * @brief Trapezoidal motion profile parameters
 */
struct TrapezoidalProfile {
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
 * @brief RMT-based pulse generator implementation
 *
 * Uses RMT peripheral with DMA for precise pulse generation on servo axes.
 * Features:
 * - 80 MHz resolution (12.5ns per tick)
 * - Up to 500 kHz pulse frequency
 * - 50% duty cycle pulses
 * - DMA streaming for unlimited motion length
 * - Trapezoidal velocity profile
 * - ISR-safe: buffer refill runs in dedicated task
 */
class RmtPulseGenerator : public IPulseGenerator {
public:
    /**
     * @brief Construct RMT pulse generator
     *
     * @param channel_id RMT channel index (0-3 for X, Z, A, B)
     * @param gpio_num GPIO pin for STEP output
     * @param resolution_hz RMT clock resolution (default 80 MHz)
     */
    RmtPulseGenerator(int channel_id, int gpio_num,
                      uint32_t resolution_hz = 80000000);

    ~RmtPulseGenerator() override;

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

    /**
     * @brief Get the channel ID
     * @return RMT channel index
     */
    int getChannelId() const { return channel_id_; }

private:
    // Configuration
    int channel_id_;
    int gpio_num_;
    uint32_t resolution_hz_;

    // RMT handles
    rmt_channel_handle_t channel_handle_;
    rmt_encoder_handle_t encoder_handle_;
    bool initialized_;

    // Motion state
    std::atomic<ProfileState> state_;
    MotionMode mode_;
    TrapezoidalProfile profile_;
    bool direction_;  // true = forward, false = reverse

    // Pulse counting
    std::atomic<int64_t> pulse_count_;
    std::atomic<float> current_velocity_;

    // DMA buffer management
    static constexpr size_t BUFFER_SYMBOLS = 64;  // RMT symbols per buffer
    rmt_symbol_word_t buffer_a_[BUFFER_SYMBOLS];
    rmt_symbol_word_t buffer_b_[BUFFER_SYMBOLS];
    std::atomic<bool> use_buffer_a_;
    std::atomic<size_t> symbols_in_current_buffer_;
    std::atomic<size_t> symbols_in_next_buffer_;

    // Refill task management
    TaskHandle_t refill_task_handle_;
    std::atomic<bool> refill_pending_;
    std::atomic<bool> task_should_exit_;

    // Callback
    MotionCompleteCallback completion_callback_;
    SemaphoreHandle_t callback_mutex_;

    // Profile calculation
    void calculateTrapezoidalProfile(int32_t pulses, float max_vel, float accel);
    void calculateVelocityProfile(float velocity, float acceleration);
    float velocityAtPosition(int64_t position) const;
    uint32_t velocityToTickPeriod(float velocity) const;

    // Buffer management
    size_t fillBuffer(rmt_symbol_word_t* buffer, size_t max_symbols);
    void primeBuffers();

    // Refill task
    static void refillTaskEntry(void* arg);
    void refillTaskLoop();
    void handleRefillRequest();

    // RMT callbacks (static trampolines) - ISR SAFE
    static bool IRAM_ATTR onTxDone(rmt_channel_handle_t channel,
                                    const rmt_tx_done_event_data_t* event_data,
                                    void* user_data);

    // Instance callback handlers
    bool handleTxDoneISR();
    void notifyCompletion();
};

#endif // RMT_PULSE_GEN_H
