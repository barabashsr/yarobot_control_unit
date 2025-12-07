/**
 * @file rmt_pulse_gen.cpp
 * @brief RMT-based pulse generator implementation using FastAccelStepper patterns
 * @author YaRobot Team
 * @date 2025
 *
 * Architecture (FastAccelStepper pattern):
 * - Command queue filled by ramp generator task (TASK context - FPU allowed)
 * - RMT simple encoder callback reads from queue (ISR context - no FPU!)
 * - Position tracked in encoder callback via atomic pulse_count_
 * - Bounded-latency position updates (<10ms at all frequencies)
 * - Mid-motion blending via atomic parameter replacement
 *
 * Key changes from DMA streaming architecture:
 * - with_dma = false (enables all 4 RMT channels simultaneously)
 * - rmt_new_simple_encoder() with on-demand callback
 * - Command queue instead of double-buffer
 * - Event-driven ramp task instead of continuous refill
 */

#include "rmt_pulse_gen.h"
#include "esp_log.h"
#include <cmath>
#include <algorithm>

static const char* TAG = "RMT_PULSE";

// Task configuration from config_timing.h (task stack defined there)
static constexpr UBaseType_t RAMP_TASK_PRIORITY = configTIMER_TASK_PRIORITY - 1;

// ============================================================================
// Constructor / Destructor
// ============================================================================

RmtPulseGenerator::RmtPulseGenerator(int channel_id, int step_pin)
    : channel_id_(channel_id)
    , step_pin_(step_pin)
{
    // Initialize queue state
    queue_end_.position = 0;
    queue_end_.direction = true;
    queue_end_.ticks_queued = 0;
}

RmtPulseGenerator::~RmtPulseGenerator()
{
    // Signal ramp task to exit
    if (ramp_task_handle_) {
        running_.store(false, std::memory_order_release);
        rmt_stopped_.store(true, std::memory_order_release);
        if (ramp_semaphore_) {
            xSemaphoreGive(ramp_semaphore_);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        vTaskDelete(ramp_task_handle_);
        ramp_task_handle_ = nullptr;
    }

    if (ramp_semaphore_) {
        vSemaphoreDelete(ramp_semaphore_);
        ramp_semaphore_ = nullptr;
    }

    if (initialized_.load()) {
        stopImmediate();
        if (rmt_channel_) {
            rmt_disable(rmt_channel_);
            rmt_del_channel(rmt_channel_);
        }
        if (rmt_encoder_) {
            rmt_del_encoder(rmt_encoder_);
        }
    }
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t RmtPulseGenerator::init()
{
    if (initialized_.load()) {
        ESP_LOGW(TAG, "Channel %d already initialized", channel_id_);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing RMT channel %d on GPIO %d (step)",
             channel_id_, step_pin_);

    // Initialize RMT with callback encoder (NO DMA)
    // Note: Direction pin managed externally via shift register at MotorBase level
    esp_err_t ret = initRmt();
    if (ret != ESP_OK) {
        return ret;
    }

    // Create ramp generator task
    createRampTask();

    initialized_.store(true, std::memory_order_release);
    ESP_LOGI(TAG, "RMT channel %d initialized successfully (FastAccelStepper pattern)", channel_id_);
    return ESP_OK;
}

esp_err_t RmtPulseGenerator::initRmt()
{
    // Create RMT TX channel (NO DMA - critical for 4-channel operation)
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = static_cast<gpio_num_t>(step_pin_),
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LIMIT_RMT_RESOLUTION_HZ,  // 16 MHz
        .mem_block_symbols = LIMIT_RMT_MEM_BLOCK_SYMBOLS,  // 48
        .trans_queue_depth = 1,
        .intr_priority = 0,
        .flags = {
            .invert_out = 0,
            .with_dma = 0,  // CRITICAL: No DMA - enables all 4 channels
            .io_loop_back = 0,
            .io_od_mode = 0,
            .allow_pd = 0
        }
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_config, &rmt_channel_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create simple encoder with callback
    ret = createEncoder();
    if (ret != ESP_OK) {
        rmt_del_channel(rmt_channel_);
        rmt_channel_ = nullptr;
        return ret;
    }

    // Register transmit done callback
    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = onTransmitDone
    };
    ret = rmt_tx_register_event_callbacks(rmt_channel_, &cbs, this);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register callbacks: %s", esp_err_to_name(ret));
        rmt_del_encoder(rmt_encoder_);
        rmt_del_channel(rmt_channel_);
        rmt_encoder_ = nullptr;
        rmt_channel_ = nullptr;
        return ret;
    }

    // Enable channel
    ret = rmt_enable(rmt_channel_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        rmt_del_encoder(rmt_encoder_);
        rmt_del_channel(rmt_channel_);
        rmt_encoder_ = nullptr;
        rmt_channel_ = nullptr;
        return ret;
    }

    return ESP_OK;
}

esp_err_t RmtPulseGenerator::createEncoder()
{
    // Create simple encoder with callback (FastAccelStepper pattern)
    rmt_simple_encoder_config_t enc_config = {
        .callback = encodeCallback,
        .arg = this,
        .min_chunk_size = LIMIT_RMT_PART_SIZE  // 24 symbols minimum per call
    };
    esp_err_t ret = rmt_new_simple_encoder(&enc_config, &rmt_encoder_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create simple encoder: %s", esp_err_to_name(ret));
    }
    return ret;
}

void RmtPulseGenerator::createRampTask()
{
    // Create binary semaphore for wake-up
    ramp_semaphore_ = xSemaphoreCreateBinary();
    configASSERT(ramp_semaphore_ != nullptr);

    // Task name includes channel ID for debugging
    char task_name[16];
    snprintf(task_name, sizeof(task_name), "ramp_%d", channel_id_);

    // Create task pinned to Core 1 (motion core)
    BaseType_t result = xTaskCreatePinnedToCore(
        rampTaskFunc,
        task_name,
        TIMING_RMT_RAMP_TASK_STACK,
        this,
        RAMP_TASK_PRIORITY,
        &ramp_task_handle_,
        1  // Core 1 (motion core)
    );
    configASSERT(result == pdPASS);
}

// ============================================================================
// Queue Operations
// ============================================================================

bool RmtPulseGenerator::isQueueFull() const
{
    uint8_t rp = read_idx_.load(std::memory_order_acquire);
    uint8_t wp = write_idx_.load(std::memory_order_acquire);
    return ((wp + 1) & LIMIT_RMT_QUEUE_LEN_MASK) == rp;
}

bool RmtPulseGenerator::isQueueEmpty() const
{
    uint8_t rp = read_idx_.load(std::memory_order_acquire);
    uint8_t wp = write_idx_.load(std::memory_order_acquire);
    return rp == wp;
}

uint8_t RmtPulseGenerator::queueSpace() const
{
    uint8_t rp = read_idx_.load(std::memory_order_acquire);
    uint8_t wp = write_idx_.load(std::memory_order_acquire);
    return (rp - wp - 1) & LIMIT_RMT_QUEUE_LEN_MASK;
}

uint32_t RmtPulseGenerator::ticksInQueue() const
{
    return queue_end_.ticks_queued;
}

bool RmtPulseGenerator::pushCommand(const StepCommand& cmd)
{
    if (isQueueFull()) {
        return false;
    }

    uint8_t wp = write_idx_.load(std::memory_order_relaxed);
    queue_[wp] = cmd;
    write_idx_.store((wp + 1) & LIMIT_RMT_QUEUE_LEN_MASK, std::memory_order_release);

    // Update queue end state (internal tracking for ramp generator)
    if (cmd.flags & CMD_FLAG_DIRECTION) {
        queue_end_.position += cmd.steps;
    } else {
        queue_end_.position -= cmd.steps;
    }
    queue_end_.ticks_queued += static_cast<uint32_t>(cmd.ticks) * cmd.steps;

    // Update position tracker when queuing (not in ISR)
    // Position represents where we'll be after this command executes
    // Note: addPulses() uses direction_ stored in tracker, so always pass positive count
    if (position_tracker_ && cmd.steps > 0) {
        position_tracker_->addPulses(cmd.steps);
    }

    return true;
}

void RmtPulseGenerator::clearQueue()
{
    read_idx_.store(0, std::memory_order_relaxed);
    write_idx_.store(0, std::memory_order_release);
    queue_end_.position = 0;
    queue_end_.direction = direction_.load();
    queue_end_.ticks_queued = 0;
}

// ============================================================================
// Ramp Generator Task (TASK context - FPU allowed)
// ============================================================================

void RmtPulseGenerator::rampTaskFunc(void* arg)
{
    RmtPulseGenerator* self = static_cast<RmtPulseGenerator*>(arg);

    ESP_LOGI(TAG, "Ramp task started for channel %d", self->channel_id_);

    while (true) {
        // Wait for wake-up signal or timeout
        if (xSemaphoreTake(self->ramp_semaphore_,
                          pdMS_TO_TICKS(TIMING_RMT_RAMP_TASK_PERIOD_MS))) {
            // Semaphore taken - new motion or queue low
        }

        // Handle completion notification FIRST (before fillQueue)
        // This prevents race condition where new startMove() sets ACCELERATING
        // but we overwrite it with IDLE from previous motion's completion
        bool pending = self->completion_pending_.load(std::memory_order_acquire);
        if (pending) {
            ESP_LOGW(TAG, "DEBUG rampTask: completion_pending=true, calling callback");
            self->completion_pending_.store(false, std::memory_order_release);
            self->running_.store(false, std::memory_order_release);
            self->ramp_state_ = RampState::IDLE;
            if (self->completion_callback_) {
                int64_t pulses = self->pulse_count_.load(std::memory_order_relaxed);
                ESP_LOGW(TAG, "DEBUG rampTask: calling completion_callback with pulses=%lld", (long long)pulses);
                self->completion_callback_(pulses);
            } else {
                ESP_LOGW(TAG, "DEBUG rampTask: NO completion_callback set!");
            }
        }

        // Now check if queue needs filling (for new or ongoing motion)
        if (self->ramp_state_ != RampState::IDLE) {
            self->fillQueue();  // FPU operations OK here
        }
    }
}

void RmtPulseGenerator::fillQueue()
{
    // Check for parameter changes
    if (params_.params_changed.exchange(false, std::memory_order_acq_rel)) {
        recalculateRamp();
    }

    int cmds_added = 0;

    // Calculate current queue depth
    uint8_t rp = read_idx_.load(std::memory_order_acquire);
    uint8_t wp = write_idx_.load(std::memory_order_acquire);
    uint8_t queue_depth = (wp - rp) & LIMIT_RMT_QUEUE_LEN_MASK;

    // Fill queue while space available and motion not complete
    while (queueSpace() > 0 && ramp_state_ != RampState::IDLE) {
        // Only apply ticks limit if we have minimum commands queued
        // This prevents underrun at low frequencies where each command has many ticks
        if (queue_depth >= LIMIT_RMT_MIN_QUEUE_CMDS &&
            ticksInQueue() >= LIMIT_RMT_FORWARD_PLANNING_TICKS) {
            break;
        }

        StepCommand cmd = generateNextCommand();

        if (cmd.ticks == 0 && cmd.steps == 0) {
            // Motion complete
            ESP_LOGW(TAG, "DEBUG fillQueue: motion complete, cmds_added=%d, ramp_state=%d, queue_end_pos=%ld, target=%ld",
                     cmds_added, (int)ramp_state_, (long)queue_end_.position,
                     (long)params_.target_position.load());
            break;
        }

        pushCommand(cmd);
        cmds_added++;
        queue_depth++;  // Track queue depth as we add

        if (cmds_added <= 3) {
            ESP_LOGW(TAG, "DEBUG fillQueue cmd[%d]: ticks=%u, steps=%u, flags=0x%02X",
                     cmds_added, cmd.ticks, cmd.steps, cmd.flags);
        }

        if (cmd.flags & CMD_FLAG_LAST) {
            ESP_LOGW(TAG, "DEBUG fillQueue: LAST flag set, total cmds=%d", cmds_added);
            ramp_state_ = RampState::IDLE;
            break;
        }
    }

    if (cmds_added > 0) {
        ESP_LOGW(TAG, "DEBUG fillQueue: added %d cmds, queue_space=%u, ticks_in_queue=%lu",
                 cmds_added, queueSpace(), (unsigned long)ticksInQueue());
    }
}

StepCommand RmtPulseGenerator::generateNextCommand()
{
    StepCommand cmd = {};

    // Get current parameters
    float target_vel = params_.target_velocity.load(std::memory_order_acquire);
    float accel = params_.acceleration.load(std::memory_order_acquire);
    int32_t target_pos = params_.target_position.load(std::memory_order_acquire);
    bool position_mode = params_.position_mode.load(std::memory_order_acquire);

    static int gen_call_count = 0;
    gen_call_count++;
    if (gen_call_count <= 5) {
        ESP_LOGW(TAG, "DEBUG genCmd[%d]: target_vel=%.1f, accel=%.1f, target_pos=%ld, pos_mode=%d, ramp_state=%d",
                 gen_call_count, target_vel, accel, (long)target_pos, position_mode, (int)ramp_state_);
    }

    // Set direction flag
    if (direction_.load(std::memory_order_relaxed)) {
        cmd.flags |= CMD_FLAG_DIRECTION;
    }

    // State machine for ramp generation
    switch (ramp_state_) {
        case RampState::ACCELERATING: {
            // Calculate velocity at current ramp position: v = sqrt(2 * a * d)
            float velocity = std::sqrt(2.0f * accel * static_cast<float>(ramp_steps_up_));

            if (velocity >= target_vel) {
                // Reached target velocity
                velocity = target_vel;
                ramp_state_ = RampState::CRUISING;
            }

            // Check if need to start deceleration (position mode)
            if (position_mode) {
                int32_t decel_dist = calculateDecelDistance(velocity, accel);
                int32_t remaining;
                if (direction_.load(std::memory_order_relaxed)) {
                    // FWD: position goes 0 → target_pos
                    remaining = target_pos - queue_end_.position;
                } else {
                    // REV: position goes 0 → -target_pos, remaining = target - |position|
                    remaining = target_pos + queue_end_.position;
                }
                if (remaining <= decel_dist) {
                    ramp_state_ = RampState::DECELERATING;
                    ramp_steps_down_ = 0;
                }
            }

            // Convert to ticks
            // Clamp velocity to minimum starting frequency to avoid zero-velocity burst
            // Min freq = 16MHz / 65535 = 244 Hz (uint16_t ticks limit)
            static constexpr float MIN_START_FREQ_HZ = 250.0f;
            if (velocity < MIN_START_FREQ_HZ) {
                velocity = MIN_START_FREQ_HZ;
            }
            current_velocity_ = velocity;
            current_ticks_ = LIMIT_RMT_RESOLUTION_HZ / static_cast<uint32_t>(velocity);

            // Calculate steps for this command
            uint8_t steps = calculateStepsForLatency(current_ticks_);
            ramp_steps_up_ += steps;

            cmd.ticks = (current_ticks_ > 65535) ? 65535 : static_cast<uint16_t>(current_ticks_);
            cmd.steps = steps;
            break;
        }

        case RampState::CRUISING: {
            // Constant velocity
            static int cruising_count = 0;
            cruising_count++;
            if (cruising_count <= 5 || cruising_count % 100 == 0) {
                ESP_LOGW(TAG, "DEBUG CRUISING[%d]: pos_mode=%d, vel=%.1f, queue_pos=%ld",
                         cruising_count, position_mode, current_velocity_, (long)queue_end_.position);
            }

            if (position_mode) {
                // Check if need to start deceleration
                int32_t decel_dist = calculateDecelDistance(current_velocity_, accel);
                int32_t remaining;
                if (direction_.load(std::memory_order_relaxed)) {
                    // FWD: position goes 0 → target_pos
                    remaining = target_pos - queue_end_.position;
                } else {
                    // REV: position goes 0 → -target_pos, remaining = target - |position|
                    remaining = target_pos + queue_end_.position;
                }
                if (remaining <= decel_dist) {
                    ESP_LOGW(TAG, "DEBUG CRUISING: decel triggered, remaining=%ld, decel_dist=%ld",
                             (long)remaining, (long)decel_dist);
                    ramp_state_ = RampState::DECELERATING;
                    ramp_steps_down_ = 0;
                }
            }

            uint8_t steps = calculateStepsForLatency(current_ticks_);
            cmd.ticks = (current_ticks_ > 65535) ? 65535 : static_cast<uint16_t>(current_ticks_);
            cmd.steps = steps;
            break;
        }

        case RampState::DECELERATING: {
            // Calculate remaining distance to target position
            // For position mode, use actual remaining distance to target
            // For velocity mode (continuous), use theoretical decel distance
            int32_t remaining_to_target;
            if (position_mode) {
                // target_pos is ALWAYS positive (absolute distance to travel)
                // queue_end_.position is positive for FWD, negative for REV
                if (direction_.load(std::memory_order_relaxed)) {
                    // FWD: position goes 0 → target_pos, remaining = target - position
                    remaining_to_target = target_pos - queue_end_.position;
                } else {
                    // REV: position goes 0 → -target_pos, remaining = target - |position|
                    remaining_to_target = target_pos + queue_end_.position;
                }
            } else {
                // Velocity mode: use theoretical decel distance
                int32_t decel_dist = calculateDecelDistance(current_velocity_, accel);
                remaining_to_target = decel_dist - ramp_steps_down_;
            }

            if (remaining_to_target <= 0) {
                // Motion complete
                cmd.ticks = 0;
                cmd.steps = 0;
                cmd.flags |= CMD_FLAG_LAST;
                ESP_LOGW(TAG, "DEBUG DECEL: motion complete, remaining_to_target=%ld, queue_pos=%ld, target=%ld",
                         (long)remaining_to_target, (long)queue_end_.position, (long)target_pos);
                break;
            }

            // Calculate velocity from decel ramp: v = sqrt(2 * a * remaining)
            float velocity = std::sqrt(2.0f * accel * static_cast<float>(remaining_to_target));

            if (velocity < static_cast<float>(LIMIT_MIN_PULSE_FREQ_HZ)) {
                // Motion complete
                cmd.ticks = 0;
                cmd.steps = 0;
                cmd.flags |= CMD_FLAG_LAST;
                ESP_LOGW(TAG, "DEBUG DECEL: velocity too low (%.1f Hz), stopping", velocity);
                break;
            }

            current_velocity_ = velocity;
            current_ticks_ = LIMIT_RMT_RESOLUTION_HZ / static_cast<uint32_t>(velocity);

            uint8_t steps = calculateStepsForLatency(current_ticks_);
            // Don't generate more steps than remaining to target
            if (steps > remaining_to_target) {
                steps = static_cast<uint8_t>(remaining_to_target);
            }
            ramp_steps_down_ += steps;

            cmd.ticks = (current_ticks_ > 65535) ? 65535 : static_cast<uint16_t>(current_ticks_);
            cmd.steps = steps;

            // Check if this command completes the motion
            if (position_mode) {
                // Calculate remaining after this command
                // For FWD: remaining = target - (position + steps) = target - position - steps
                // For REV: remaining = target - |position + (-steps)| = target + position - steps
                int32_t remaining_after;
                if (direction_.load(std::memory_order_relaxed)) {
                    remaining_after = target_pos - queue_end_.position - steps;
                } else {
                    remaining_after = target_pos + queue_end_.position - steps;
                }
                if (remaining_after <= 0) {
                    cmd.flags |= CMD_FLAG_LAST;
                }
            } else {
                int32_t decel_dist = calculateDecelDistance(current_velocity_, accel);
                if (ramp_steps_down_ >= decel_dist) {
                    cmd.flags |= CMD_FLAG_LAST;
                }
            }
            break;
        }

        default:
            cmd.ticks = 0;
            cmd.steps = 0;
            break;
    }

    return cmd;
}

void RmtPulseGenerator::recalculateRamp()
{
    // Recalculate profile based on new parameters
    float target_vel = params_.target_velocity.load(std::memory_order_acquire);
    float accel = params_.acceleration.load(std::memory_order_acquire);

    // If velocity changed, may need to adjust state
    if (current_velocity_ > target_vel) {
        // Need to decelerate to new target
        ramp_state_ = RampState::DECELERATING;
        ramp_steps_down_ = 0;
    } else if (current_velocity_ < target_vel && ramp_state_ == RampState::CRUISING) {
        // Need to accelerate to new target
        ramp_state_ = RampState::ACCELERATING;
    }

    ESP_LOGD(TAG, "Ramp recalculated: target_vel=%.1f, accel=%.1f, state=%d",
             target_vel, accel, static_cast<int>(ramp_state_));
}

uint8_t RmtPulseGenerator::calculateStepsForLatency(uint32_t ticks) const
{
    // Frequency in Hz
    uint32_t freq = LIMIT_RMT_RESOLUTION_HZ / ticks;

    // Calculate steps for position update latency target
    // max_steps = freq * (TIMING_POSITION_UPDATE_LATENCY_MS / 1000)
    //           = freq / (1000 / TIMING_POSITION_UPDATE_LATENCY_MS)
    constexpr uint32_t latency_divisor = 1000 / TIMING_POSITION_UPDATE_LATENCY_MS;  // 100 for 10ms
    uint32_t max_steps = freq / latency_divisor;

    // Clamp to valid range [1, 255]
    if (max_steps < 1) max_steps = 1;
    if (max_steps > LIMIT_RMT_MAX_STEPS_PER_CMD) {
        max_steps = LIMIT_RMT_MAX_STEPS_PER_CMD;
    }

    return static_cast<uint8_t>(max_steps);
}

int32_t RmtPulseGenerator::calculateDecelDistance(float velocity, float decel) const
{
    // d = v² / (2 * a)
    return static_cast<int32_t>((velocity * velocity) / (2.0f * decel));
}

// ============================================================================
// Encoder Callback and Symbol Generation (ISR context - NO FPU!)
// ============================================================================

size_t IRAM_ATTR RmtPulseGenerator::encodeCallback(
    const void* data, size_t data_size,
    size_t symbols_written, size_t symbols_free,
    rmt_symbol_word_t* symbols, bool* done, void* arg)
{
    RmtPulseGenerator* self = static_cast<RmtPulseGenerator*>(arg);
    *done = false;

    // Need at least PART_SIZE symbols available
    if (symbols_free < LIMIT_RMT_PART_SIZE) {
        return 0;
    }

    // ============================================================
    // TWO-PHASE COMPLETION (like FastAccelStepper)
    // Phase 1: Queue empty -> set rmt_stopped_, fill final pause
    // Phase 2: rmt_stopped_ already set -> return done=true
    // This ensures RMT finishes transmitting before signaling done
    // ============================================================

    // Phase 2: Already stopped - signal completion to RMT
    if (self->rmt_stopped_.load(std::memory_order_acquire)) {
        *done = true;
        return 0;
    }

    // Check queue
    uint8_t rp = self->read_idx_.load(std::memory_order_acquire);
    uint8_t wp = self->write_idx_.load(std::memory_order_acquire);

    // Phase 1: Queue empty - fill final pause and set stopped flag
    if (rp == wp) {
        self->rmt_stopped_.store(true, std::memory_order_release);
        // Fill a minimum pause to let RMT finish gracefully
        // This pause will be transmitted, then next callback sees rmt_stopped_
        for (size_t i = 0; i < LIMIT_RMT_PART_SIZE; i++) {
            symbols[i].val = 0;  // Zero symbols = pause
        }
        // Set first symbol with minimum duration to ensure transmission
        symbols[0].duration0 = LIMIT_RMT_MIN_CMD_TICKS;
        symbols[0].level0 = 0;
        symbols[0].duration1 = 0;
        symbols[0].level1 = 0;
        return LIMIT_RMT_PART_SIZE;
    }

    // Get current command (may be partially consumed)
    StepCommand* cmd = &self->queue_[rp & LIMIT_RMT_QUEUE_LEN_MASK];

    // Fill symbols using the command
    size_t steps_generated = self->fillSymbols(symbols, cmd);

    // ============================================================
    // PULSE COUNT TRACKING (ISR context - atomic operations only)
    // Note: Position tracker is updated when commands are QUEUED,
    // not here. This pulse_count_ is just for internal tracking.
    // ============================================================
    if (steps_generated > 0) {
        bool dir = self->direction_.load(std::memory_order_relaxed);
        int64_t delta = dir ? static_cast<int64_t>(steps_generated)
                            : -static_cast<int64_t>(steps_generated);
        self->pulse_count_.fetch_add(delta, std::memory_order_relaxed);
        // Position tracker updated in pushCommand(), not here
    }

    // Wake ramp task if queue running low (< 8 entries)
    uint8_t queue_depth = (wp - rp) & LIMIT_RMT_QUEUE_LEN_MASK;
    if (queue_depth < 8) {
        self->wakeRampTask();
    }

    return LIMIT_RMT_PART_SIZE;
}

size_t RmtPulseGenerator::fillSymbols(rmt_symbol_word_t* symbols, StepCommand* cmd)
{
    // ============================================================
    // DIRECTION CHANGE HANDLING
    // ============================================================
    if (cmd->flags & CMD_FLAG_TOGGLE_DIR) {
        if (last_chunk_had_steps_) {
            // Insert pause for direction setup time
            // At 16MHz: LIMIT_RMT_DIR_SETUP_US * 16 ticks per µs
            uint16_t pause_ticks = LIMIT_RMT_DIR_SETUP_US * (LIMIT_RMT_RESOLUTION_HZ / 1000000);
            uint16_t ticks_per_symbol = pause_ticks / LIMIT_RMT_PART_SIZE;
            if (ticks_per_symbol < 2) ticks_per_symbol = 2;

            for (uint8_t i = 0; i < LIMIT_RMT_PART_SIZE; i++) {
                symbols[i].duration0 = ticks_per_symbol;
                symbols[i].level0 = 0;
                symbols[i].duration1 = ticks_per_symbol;
                symbols[i].level1 = 0;
            }

            last_chunk_had_steps_ = false;
            return 0;  // No steps generated
        }

        // Toggle direction flag (actual pin managed externally via shift register)
        bool new_dir = !direction_.load(std::memory_order_relaxed);
        direction_.store(new_dir, std::memory_order_relaxed);

        // Clear toggle flag
        cmd->flags &= ~CMD_FLAG_TOGGLE_DIR;
    }

    uint8_t steps = cmd->steps;
    uint16_t ticks = cmd->ticks;

    // ============================================================
    // PAUSE COMMAND (steps == 0)
    // ============================================================
    if (steps == 0) {
        last_chunk_had_steps_ = false;

        // Fill with idle symbols
        uint16_t ticks_per_symbol = ticks / LIMIT_RMT_PART_SIZE;
        if (ticks_per_symbol < 2) ticks_per_symbol = 2;

        for (uint8_t i = 0; i < LIMIT_RMT_PART_SIZE; i++) {
            symbols[i].duration0 = ticks_per_symbol;
            symbols[i].level0 = 0;
            symbols[i].duration1 = ticks_per_symbol;
            symbols[i].level1 = 0;
        }

        // Advance to next command
        read_idx_.store((read_idx_.load(std::memory_order_relaxed) + 1) & LIMIT_RMT_QUEUE_LEN_MASK,
                        std::memory_order_release);
        return 0;
    }

    // ============================================================
    // STEP GENERATION
    // ============================================================
    last_chunk_had_steps_ = true;

    // Determine how many steps to generate this callback
    uint8_t steps_to_do = steps;
    if (steps_to_do > LIMIT_RMT_PART_SIZE) {
        steps_to_do = LIMIT_RMT_PART_SIZE;
    }

    // Calculate 50% duty cycle timing (integer math only!)
    uint16_t half_ticks_high = ticks >> 1;
    uint16_t half_ticks_low = ticks - half_ticks_high;

    // Ensure minimum tick count
    if (half_ticks_high < 1) half_ticks_high = 1;
    if (half_ticks_low < 1) half_ticks_low = 1;

    if (steps_to_do < LIMIT_RMT_PART_SIZE) {
        // Need to fill unused symbols with idle first
        uint8_t idle_count = LIMIT_RMT_PART_SIZE - steps_to_do;

        // Generate idle symbols first
        for (uint8_t i = 0; i < idle_count; i++) {
            symbols[i].duration0 = 2;  // Minimal idle
            symbols[i].level0 = 0;
            symbols[i].duration1 = 2;
            symbols[i].level1 = 0;
        }

        // Then generate step pulses
        for (uint8_t i = 0; i < steps_to_do; i++) {
            // Step pulse: LOW for half, HIGH for half (rising edge is step)
            symbols[idle_count + i].duration0 = half_ticks_low;
            symbols[idle_count + i].level0 = 0;
            symbols[idle_count + i].duration1 = half_ticks_high;
            symbols[idle_count + i].level1 = 1;
        }
    } else {
        // All PART_SIZE symbols are step pulses
        for (uint8_t i = 0; i < LIMIT_RMT_PART_SIZE; i++) {
            symbols[i].duration0 = half_ticks_low;
            symbols[i].level0 = 0;
            symbols[i].duration1 = half_ticks_high;
            symbols[i].level1 = 1;
        }
    }

    // Update command or advance to next
    uint8_t remaining = steps - steps_to_do;

    // Decrement ticks_queued for consumed steps (ISR context - simple decrement OK)
    uint32_t consumed_ticks = static_cast<uint32_t>(ticks) * steps_to_do;
    if (queue_end_.ticks_queued >= consumed_ticks) {
        queue_end_.ticks_queued -= consumed_ticks;
    } else {
        queue_end_.ticks_queued = 0;
    }

    if (remaining == 0) {
        // Command fully consumed - advance read index
        read_idx_.store((read_idx_.load(std::memory_order_relaxed) + 1) & LIMIT_RMT_QUEUE_LEN_MASK,
                        std::memory_order_release);
    } else {
        // Partial consumption - update remaining steps in place
        cmd->steps = remaining;
    }

    return steps_to_do;
}

void RmtPulseGenerator::wakeRampTask()
{
    if (ramp_semaphore_) {
        BaseType_t higher_priority_woken = pdFALSE;
        xSemaphoreGiveFromISR(ramp_semaphore_, &higher_priority_woken);
        if (higher_priority_woken) {
            portYIELD_FROM_ISR();
        }
    }
}

bool IRAM_ATTR RmtPulseGenerator::onTransmitDone(
    rmt_channel_handle_t channel,
    const rmt_tx_done_event_data_t* event_data,
    void* user_ctx)
{
    RmtPulseGenerator* self = static_cast<RmtPulseGenerator*>(user_ctx);

    // RMT transmission complete - motion is truly finished now
    // Mark as not running and signal completion
    self->running_.store(false, std::memory_order_release);
    self->completion_pending_.store(true, std::memory_order_release);

    // Wake ramp task to execute completion callback (can't call from ISR)
    if (self->ramp_semaphore_) {
        BaseType_t higher_priority_woken = pdFALSE;
        xSemaphoreGiveFromISR(self->ramp_semaphore_, &higher_priority_woken);
        if (higher_priority_woken) {
            portYIELD_FROM_ISR();
        }
    }

    return false;
}

// ============================================================================
// Motion Control
// ============================================================================

esp_err_t RmtPulseGenerator::startMove(int32_t pulses, float max_velocity, float acceleration)
{
    if (!initialized_.load()) {
        ESP_LOGE(TAG, "DEBUG startMove: NOT INITIALIZED!");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate parameters
    if (pulses == 0) {
        ESP_LOGW(TAG, "DEBUG startMove: pulses=0, nothing to do");
        return ESP_OK;  // Nothing to do
    }
    if (max_velocity <= 0 || max_velocity > LIMIT_MAX_PULSE_FREQ_HZ) {
        ESP_LOGE(TAG, "Invalid velocity: %.1f (limit=%d)", max_velocity, LIMIT_MAX_PULSE_FREQ_HZ);
        return ESP_ERR_INVALID_ARG;
    }
    if (acceleration <= 0) {
        ESP_LOGE(TAG, "Invalid acceleration: %.1f", acceleration);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGW(TAG, "DEBUG startMove: pulses=%ld, vel=%.1f Hz, accel=%.1f pulses/s^2",
             (long)pulses, max_velocity, acceleration);

    // Atomic profile replacement - works whether running or not
    bool new_direction = (pulses >= 0);
    int32_t abs_pulses = (pulses >= 0) ? pulses : -pulses;

    // Update parameters atomically
    params_.target_position.store(abs_pulses, std::memory_order_relaxed);
    params_.target_velocity.store(max_velocity, std::memory_order_relaxed);
    params_.acceleration.store(acceleration, std::memory_order_relaxed);
    params_.position_mode.store(true, std::memory_order_relaxed);

    // Handle direction (actual pin managed externally via shift register)
    direction_.store(new_direction, std::memory_order_relaxed);

    // For position moves, always start fresh (stop any current motion first)
    // Blending is only for velocity mode changes
    bool was_running = running_.load(std::memory_order_acquire);
    ESP_LOGW(TAG, "DEBUG startMove: was_running=%d, ramp_state=%d", was_running, (int)ramp_state_);

    if (was_running) {
        ESP_LOGW(TAG, "DEBUG startMove: stopping previous motion first");
        stopImmediate();
    }

    // Starting fresh
    ESP_LOGW(TAG, "DEBUG startMove: starting fresh, dir=%s, abs_pulses=%ld",
             new_direction ? "FWD" : "REV", (long)abs_pulses);
    clearQueue();
    pulse_count_.store(0, std::memory_order_relaxed);
    current_velocity_ = 0.0f;
    current_ticks_ = LIMIT_RMT_MIN_CMD_TICKS;
    ramp_steps_up_ = 0;
    ramp_steps_down_ = 0;
    ramp_state_ = RampState::ACCELERATING;
    queue_end_.position = 0;
    queue_end_.direction = new_direction;
    queue_end_.ticks_queued = 0;

    // Set direction on position tracker before motion
    if (position_tracker_) {
        position_tracker_->setDirection(new_direction);
        ESP_LOGW(TAG, "DEBUG startMove: position_tracker set direction=%s", new_direction ? "FWD" : "REV");
    } else {
        ESP_LOGW(TAG, "DEBUG startMove: NO position_tracker!");
    }

    // Start motion
    running_.store(true, std::memory_order_release);
    rmt_stopped_.store(false, std::memory_order_release);
    ESP_LOGW(TAG, "DEBUG startMove: calling startRmtTransmission()");
    startRmtTransmission();

    // Wake ramp task to process new profile immediately
    xSemaphoreGive(ramp_semaphore_);

    ESP_LOGW(TAG, "DEBUG startMove: completed, running=%d", running_.load());
    return ESP_OK;
}

esp_err_t RmtPulseGenerator::startVelocity(float velocity, float acceleration)
{
    if (!initialized_.load()) {
        return ESP_ERR_INVALID_STATE;
    }

    float abs_velocity = std::fabs(velocity);
    if (abs_velocity < LIMIT_MIN_PULSE_FREQ_HZ || abs_velocity > LIMIT_MAX_PULSE_FREQ_HZ) {
        ESP_LOGE(TAG, "Invalid velocity: %.1f (min=%.1f, max=%d)", velocity,
                 (float)LIMIT_MIN_PULSE_FREQ_HZ, LIMIT_MAX_PULSE_FREQ_HZ);
        return ESP_ERR_INVALID_ARG;
    }
    if (acceleration <= 0) {
        ESP_LOGE(TAG, "Invalid acceleration: %.1f", acceleration);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGW(TAG, "DEBUG startVelocity: vel=%.1f Hz, accel=%.1f, abs_vel=%.1f",
             velocity, acceleration, abs_velocity);

    bool new_direction = (velocity >= 0);

    // Update parameters atomically
    params_.target_velocity.store(abs_velocity, std::memory_order_relaxed);
    params_.acceleration.store(acceleration, std::memory_order_relaxed);
    params_.position_mode.store(false, std::memory_order_relaxed);  // Velocity mode

    // Handle direction (actual pin managed externally via shift register)
    direction_.store(new_direction, std::memory_order_relaxed);

    if (running_.load(std::memory_order_acquire)) {
        // Already running - adjust velocity
        ESP_LOGW(TAG, "DEBUG startVelocity: already running, setting params_changed");
        params_.params_changed.store(true, std::memory_order_release);
    } else {
        // Starting fresh
        ESP_LOGW(TAG, "DEBUG startVelocity: starting fresh, dir=%s", new_direction ? "FWD" : "REV");
        clearQueue();
        pulse_count_.store(0, std::memory_order_relaxed);
        current_velocity_ = 0.0f;
        current_ticks_ = LIMIT_RMT_MIN_CMD_TICKS;
        ramp_steps_up_ = 0;
        ramp_steps_down_ = 0;
        ramp_state_ = RampState::ACCELERATING;
        queue_end_.position = 0;
        queue_end_.direction = new_direction;
        queue_end_.ticks_queued = 0;

        if (position_tracker_) {
            position_tracker_->setDirection(new_direction);
            ESP_LOGW(TAG, "DEBUG startVelocity: position_tracker set direction=%s", new_direction ? "FWD" : "REV");
        }

        running_.store(true, std::memory_order_release);
        rmt_stopped_.store(false, std::memory_order_release);
        ESP_LOGW(TAG, "DEBUG startVelocity: calling startRmtTransmission, position_mode=%d",
                 params_.position_mode.load());
        startRmtTransmission();
    }

    xSemaphoreGive(ramp_semaphore_);

    ESP_LOGW(TAG, "DEBUG startVelocity: completed, running=%d, ramp_state=%d",
             running_.load(), (int)ramp_state_);
    return ESP_OK;
}

esp_err_t RmtPulseGenerator::stop(float deceleration)
{
    if (!running_.load(std::memory_order_acquire)) {
        return ESP_ERR_INVALID_STATE;
    }

    if (deceleration <= 0) {
        ESP_LOGE(TAG, "Invalid deceleration: %.1f", deceleration);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "stop: decel=%.1f", deceleration);

    // Switch to position mode with target = current + decel distance
    // For FWD: target = position + stop_distance (positive target)
    // For REV: target = |position| + stop_distance (still positive target, position is negative)
    float current_vel = current_velocity_;
    int32_t stop_distance = calculateDecelDistance(current_vel, deceleration);
    int32_t current_abs_position;
    if (direction_.load(std::memory_order_relaxed)) {
        current_abs_position = queue_end_.position;
    } else {
        current_abs_position = -queue_end_.position;  // Make positive
    }
    int32_t new_target = current_abs_position + stop_distance;

    params_.target_position.store(new_target, std::memory_order_relaxed);
    params_.acceleration.store(deceleration, std::memory_order_relaxed);
    params_.position_mode.store(true, std::memory_order_relaxed);
    params_.params_changed.store(true, std::memory_order_release);

    ramp_state_ = RampState::DECELERATING;
    ramp_steps_down_ = 0;

    xSemaphoreGive(ramp_semaphore_);

    return ESP_OK;
}

void RmtPulseGenerator::stopImmediate()
{
    if (!running_.load(std::memory_order_acquire)) {
        return;
    }

    ESP_LOGD(TAG, "stopImmediate");

    // Stop RMT transmission
    rmt_stopped_.store(true, std::memory_order_release);

    if (rmt_channel_) {
        rmt_disable(rmt_channel_);
        rmt_enable(rmt_channel_);  // Re-enable for next use
    }

    // Clear state
    running_.store(false, std::memory_order_release);
    ramp_state_ = RampState::IDLE;
    current_velocity_ = 0.0f;
    clearQueue();

    // Note: Do NOT fire completion callback on immediate stop
}

void RmtPulseGenerator::startRmtTransmission()
{
    ESP_LOGW(TAG, "DEBUG startRmtTransmission: filling initial queue");
    // Fill initial commands
    fillQueue();

    ESP_LOGW(TAG, "DEBUG startRmtTransmission: queue filled, starting RMT transmit");

    // Start transmission with a dummy payload (encoder callback provides actual data)
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,  // No loop
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = 0
        }
    };

    // The simple encoder callback will be called on-demand to provide symbols
    static const uint8_t dummy = 0;
    esp_err_t ret = rmt_transmit(rmt_channel_, rmt_encoder_, &dummy, sizeof(dummy), &tx_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start RMT transmission: %s", esp_err_to_name(ret));
        running_.store(false, std::memory_order_release);
        ramp_state_ = RampState::IDLE;
    } else {
        ESP_LOGW(TAG, "DEBUG startRmtTransmission: rmt_transmit() returned OK");
    }
}

// ============================================================================
// Status Methods
// ============================================================================

bool RmtPulseGenerator::isRunning() const
{
    return running_.load(std::memory_order_acquire);
}

int64_t RmtPulseGenerator::getPulseCount() const
{
    return pulse_count_.load(std::memory_order_acquire);
}

float RmtPulseGenerator::getCurrentVelocity() const
{
    if (!running_.load(std::memory_order_acquire)) {
        return 0.0f;
    }
    return current_velocity_;
}

void RmtPulseGenerator::setCompletionCallback(MotionCompleteCallback cb)
{
    completion_callback_ = cb;
}

void RmtPulseGenerator::setPositionTracker(IPositionTracker* tracker)
{
    position_tracker_ = tracker;
}
