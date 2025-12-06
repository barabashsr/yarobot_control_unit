/**
 * @file rmt_pulse_gen.h
 * @brief RMT-based pulse generator for servo axes X, Z, A, B
 * @author YaRobot Team
 * @date 2025
 *
 * @note Uses ESP32-S3 RMT peripheral with FastAccelStepper-pattern callback encoder.
 *       No DMA - enables all 4 RMT channels to operate simultaneously.
 *
 * Architecture (FastAccelStepper pattern):
 * - Command queue filled by ramp generator task
 * - RMT simple encoder callback reads from queue (ISR context)
 * - Position tracked in encoder callback via atomic pulse_count_
 * - Bounded-latency position updates (<10ms at all frequencies)
 * - Mid-motion blending via atomic parameter replacement
 */

#ifndef RMT_PULSE_GEN_H
#define RMT_PULSE_GEN_H

#include "i_pulse_generator.h"
#include "i_position_tracker.h"
#include "rmt_pulse_gen_types.h"
#include "config_limits.h"
#include "config_timing.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <atomic>

/**
 * @brief RMT pulse generator with callback encoder and software position tracking
 *
 * Implements FastAccelStepper architecture:
 * - Command queue filled by ramp generator task
 * - RMT encoder callback reads from queue (ISR context)
 * - Position tracked in encoder callback (pulse_count_ incremented per command)
 * - IPositionTracker::addPulses() called from ISR for external tracking
 *
 * Key features:
 * - 16 MHz resolution (62.5ns per tick)
 * - Up to 500 kHz pulse frequency
 * - 50% duty cycle pulses
 * - NO DMA - all 4 RMT channels available simultaneously
 * - Trapezoidal velocity profile with mid-motion blending
 */
class RmtPulseGenerator : public IPulseGenerator {
public:
    /**
     * @brief Construct RMT pulse generator
     *
     * @param channel_id RMT channel index (0-3 for X, Z, A, B)
     * @param step_pin GPIO pin number for STEP output
     *
     * @note Direction pin not controlled here - managed via shift register at MotorBase level
     */
    RmtPulseGenerator(int channel_id, int step_pin);

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
    void setPositionTracker(IPositionTracker* tracker) override;

    /**
     * @brief Get the channel ID
     * @return RMT channel index
     */
    int getChannelId() const { return channel_id_; }

private:
    // ============================================================
    // HARDWARE HANDLES
    // ============================================================

    rmt_channel_handle_t rmt_channel_{nullptr};
    rmt_encoder_handle_t rmt_encoder_{nullptr};

    // ============================================================
    // CONFIGURATION (immutable after construction)
    // ============================================================

    const int channel_id_;
    const int step_pin_;

    // ============================================================
    // COMMAND QUEUE (FastAccelStepper pattern)
    // ============================================================

    StepCommand queue_[LIMIT_RMT_QUEUE_LEN];
    std::atomic<uint8_t> read_idx_{0};      ///< ISR reads from here
    std::atomic<uint8_t> write_idx_{0};     ///< Task writes to here
    QueueState queue_end_{};                 ///< State at queue end

    // ============================================================
    // RAMP GENERATOR STATE
    // ============================================================

    RampParameters params_;                  ///< Motion parameters (atomic)
    RampState ramp_state_{RampState::IDLE};
    float current_velocity_{0.0f};           ///< Instantaneous velocity (pulses/s)
    uint32_t current_ticks_{0};              ///< Current step period in ticks
    int32_t ramp_steps_up_{0};               ///< Steps in acceleration phase
    int32_t ramp_steps_down_{0};             ///< Steps in deceleration phase

    // ============================================================
    // POSITION TRACKING (Software - encoder callback based)
    // ============================================================

    std::atomic<int64_t> pulse_count_{0};        ///< Total pulses (updated in encoder callback)
    std::atomic<bool> direction_{true};          ///< Current direction (true=forward)

    // ============================================================
    // STATE FLAGS
    // ============================================================

    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> rmt_stopped_{true};
    std::atomic<bool> completion_pending_{false};
    bool last_chunk_had_steps_{false};           ///< For direction pause insertion (ISR only)

    // ============================================================
    // RAMP TASK (Event-driven, one per axis)
    // ============================================================

    TaskHandle_t ramp_task_handle_{nullptr};
    SemaphoreHandle_t ramp_semaphore_{nullptr};  ///< Wake-up signal for queue refill

    // ============================================================
    // CALLBACKS
    // ============================================================

    MotionCompleteCallback completion_callback_{nullptr};
    IPositionTracker* position_tracker_{nullptr};  ///< Called from encoder ISR

    // ============================================================
    // PRIVATE METHODS - Initialization
    // ============================================================

    esp_err_t initRmt();
    esp_err_t createEncoder();
    void createRampTask();

    // ============================================================
    // PRIVATE METHODS - Queue operations
    // ============================================================

    bool isQueueFull() const;
    bool isQueueEmpty() const;
    uint8_t queueSpace() const;
    uint32_t ticksInQueue() const;
    bool pushCommand(const StepCommand& cmd);
    void clearQueue();

    // ============================================================
    // PRIVATE METHODS - Ramp generation (task context)
    // ============================================================

    static void rampTaskFunc(void* arg);
    void fillQueue();
    StepCommand generateNextCommand();
    void recalculateRamp();
    uint8_t calculateStepsForLatency(uint32_t ticks) const;
    int32_t calculateDecelDistance(float velocity, float decel) const;

    // ============================================================
    // PRIVATE METHODS - ISR helpers
    // ============================================================

    void IRAM_ATTR wakeRampTask();

    // ============================================================
    // PRIVATE METHODS - Motion control
    // ============================================================

    void startMotion();
    void startRmtTransmission();
    void stopMotion();

    // ============================================================
    // STATIC CALLBACKS (ISR context)
    // ============================================================

    static size_t IRAM_ATTR encodeCallback(
        const void* data, size_t data_size,
        size_t symbols_written, size_t symbols_free,
        rmt_symbol_word_t* symbols, bool* done, void* arg);

    static bool IRAM_ATTR onTransmitDone(
        rmt_channel_handle_t channel,
        const rmt_tx_done_event_data_t* event_data, void* user_ctx);

    // ============================================================
    // PRIVATE METHODS - Symbol generation (ISR context - no FPU)
    // ============================================================

    size_t IRAM_ATTR fillSymbols(rmt_symbol_word_t* symbols, StepCommand* cmd);
};

#endif // RMT_PULSE_GEN_H
