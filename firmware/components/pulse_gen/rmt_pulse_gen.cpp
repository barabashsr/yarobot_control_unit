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

    // Update queue end state
    if (cmd.flags & CMD_FLAG_DIRECTION) {
        queue_end_.position += cmd.steps;
    } else {
        queue_end_.position -= cmd.steps;
    }
    queue_end_.ticks_queued += static_cast<uint32_t>(cmd.ticks) * cmd.steps;

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
        // Either way, check if queue needs filling

        if (self->ramp_state_ != RampState::IDLE) {
            self->fillQueue();  // FPU operations OK here
        }

        // Handle completion notification (task context, not ISR)
        if (self->completion_pending_.exchange(false, std::memory_order_acq_rel)) {
            self->running_.store(false, std::memory_order_release);
            self->ramp_state_ = RampState::IDLE;
            if (self->completion_callback_) {
                self->completion_callback_(self->pulse_count_.load(std::memory_order_relaxed));
            }
        }
    }
}

void RmtPulseGenerator::fillQueue()
{
    // Check for parameter changes
    if (params_.params_changed.exchange(false, std::memory_order_acq_rel)) {
        recalculateRamp();
    }

    // Fill queue while space available and motion not complete
    while (queueSpace() > 0 && ramp_state_ != RampState::IDLE) {
        // Check forward planning limit
        if (ticksInQueue() >= LIMIT_RMT_FORWARD_PLANNING_TICKS) {
            break;
        }

        StepCommand cmd = generateNextCommand();

        if (cmd.ticks == 0 && cmd.steps == 0) {
            // Motion complete
            break;
        }

        pushCommand(cmd);

        if (cmd.flags & CMD_FLAG_LAST) {
            ramp_state_ = RampState::IDLE;
            break;
        }
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
                int32_t remaining = target_pos - queue_end_.position;
                if (direction_.load(std::memory_order_relaxed)) {
                    if (remaining <= decel_dist) {
                        ramp_state_ = RampState::DECELERATING;
                        ramp_steps_down_ = 0;
                    }
                }
            }

            // Convert to ticks
            current_velocity_ = velocity;
            if (velocity > 0) {
                current_ticks_ = LIMIT_RMT_RESOLUTION_HZ / static_cast<uint32_t>(velocity);
            } else {
                current_ticks_ = LIMIT_RMT_MIN_CMD_TICKS;
            }

            // Calculate steps for this command
            uint8_t steps = calculateStepsForLatency(current_ticks_);
            ramp_steps_up_ += steps;

            cmd.ticks = (current_ticks_ > 65535) ? 65535 : static_cast<uint16_t>(current_ticks_);
            cmd.steps = steps;
            break;
        }

        case RampState::CRUISING: {
            // Constant velocity
            if (position_mode) {
                // Check if need to start deceleration
                int32_t decel_dist = calculateDecelDistance(current_velocity_, accel);
                int32_t remaining = target_pos - queue_end_.position;

                if (direction_.load(std::memory_order_relaxed)) {
                    if (remaining <= decel_dist) {
                        ramp_state_ = RampState::DECELERATING;
                        ramp_steps_down_ = 0;
                    }
                }
            }

            uint8_t steps = calculateStepsForLatency(current_ticks_);
            cmd.ticks = (current_ticks_ > 65535) ? 65535 : static_cast<uint16_t>(current_ticks_);
            cmd.steps = steps;
            break;
        }

        case RampState::DECELERATING: {
            // Calculate remaining distance to stop
            int32_t decel_dist = calculateDecelDistance(current_velocity_, accel);
            int32_t remaining_steps = decel_dist - ramp_steps_down_;

            if (remaining_steps <= 0) {
                // Motion complete
                cmd.ticks = 0;
                cmd.steps = 0;
                cmd.flags |= CMD_FLAG_LAST;
                break;
            }

            // Calculate velocity from decel ramp: v = sqrt(2 * a * remaining)
            float velocity = std::sqrt(2.0f * accel * static_cast<float>(remaining_steps));

            if (velocity < static_cast<float>(LIMIT_MIN_PULSE_FREQ_HZ)) {
                // Motion complete
                cmd.ticks = 0;
                cmd.steps = 0;
                cmd.flags |= CMD_FLAG_LAST;
                break;
            }

            current_velocity_ = velocity;
            current_ticks_ = LIMIT_RMT_RESOLUTION_HZ / static_cast<uint32_t>(velocity);

            uint8_t steps = calculateStepsForLatency(current_ticks_);
            // Don't generate more steps than remaining
            if (steps > remaining_steps) {
                steps = static_cast<uint8_t>(remaining_steps);
            }
            ramp_steps_down_ += steps;

            cmd.ticks = (current_ticks_ > 65535) ? 65535 : static_cast<uint16_t>(current_ticks_);
            cmd.steps = steps;

            // Check if this is the last command
            if (ramp_steps_down_ >= decel_dist) {
                cmd.flags |= CMD_FLAG_LAST;
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

    // Check if stopped
    if (self->rmt_stopped_.load(std::memory_order_acquire)) {
        *done = true;
        return 0;
    }

    // Check queue
    uint8_t rp = self->read_idx_.load(std::memory_order_acquire);
    uint8_t wp = self->write_idx_.load(std::memory_order_acquire);

    if (rp == wp) {
        // Queue empty - signal completion and stop
        self->rmt_stopped_.store(true, std::memory_order_release);
        self->completion_pending_.store(true, std::memory_order_release);
        *done = true;
        return 0;
    }

    // Get current command (may be partially consumed)
    StepCommand* cmd = &self->queue_[rp & LIMIT_RMT_QUEUE_LEN_MASK];

    // Fill symbols using the command
    size_t steps_generated = self->fillSymbols(symbols, cmd);

    // ============================================================
    // POSITION TRACKING (ISR context - atomic operations only)
    // ============================================================
    if (steps_generated > 0) {
        bool dir = self->direction_.load(std::memory_order_relaxed);
        int64_t delta = dir ? static_cast<int64_t>(steps_generated)
                            : -static_cast<int64_t>(steps_generated);
        self->pulse_count_.fetch_add(delta, std::memory_order_relaxed);

        // Notify IPositionTracker if set (MUST be ISR-safe)
        if (self->position_tracker_) {
            self->position_tracker_->addPulses(delta);
        }
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
    // Transmission complete - this is expected when all commands processed
    // The encoder callback already handled completion_pending_
    return false;
}

// ============================================================================
// Motion Control
// ============================================================================

esp_err_t RmtPulseGenerator::startMove(int32_t pulses, float max_velocity, float acceleration)
{
    if (!initialized_.load()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Validate parameters
    if (pulses == 0) {
        return ESP_OK;  // Nothing to do
    }
    if (max_velocity <= 0 || max_velocity > LIMIT_MAX_PULSE_FREQ_HZ) {
        ESP_LOGE(TAG, "Invalid velocity: %.1f", max_velocity);
        return ESP_ERR_INVALID_ARG;
    }
    if (acceleration <= 0) {
        ESP_LOGE(TAG, "Invalid acceleration: %.1f", acceleration);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "startMove: pulses=%ld, vel=%.1f, accel=%.1f",
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

    // Determine ramp state
    if (running_.load(std::memory_order_acquire)) {
        // Already running - blend from current velocity
        params_.params_changed.store(true, std::memory_order_release);
    } else {
        // Starting fresh
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
        }

        // Start motion
        running_.store(true, std::memory_order_release);
        rmt_stopped_.store(false, std::memory_order_release);
        startRmtTransmission();
    }

    // Wake ramp task to process new profile immediately
    xSemaphoreGive(ramp_semaphore_);

    return ESP_OK;
}

esp_err_t RmtPulseGenerator::startVelocity(float velocity, float acceleration)
{
    if (!initialized_.load()) {
        return ESP_ERR_INVALID_STATE;
    }

    float abs_velocity = std::fabs(velocity);
    if (abs_velocity < LIMIT_MIN_PULSE_FREQ_HZ || abs_velocity > LIMIT_MAX_PULSE_FREQ_HZ) {
        ESP_LOGE(TAG, "Invalid velocity: %.1f", velocity);
        return ESP_ERR_INVALID_ARG;
    }
    if (acceleration <= 0) {
        ESP_LOGE(TAG, "Invalid acceleration: %.1f", acceleration);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "startVelocity: vel=%.1f, accel=%.1f", velocity, acceleration);

    bool new_direction = (velocity >= 0);

    // Update parameters atomically
    params_.target_velocity.store(abs_velocity, std::memory_order_relaxed);
    params_.acceleration.store(acceleration, std::memory_order_relaxed);
    params_.position_mode.store(false, std::memory_order_relaxed);  // Velocity mode

    // Handle direction (actual pin managed externally via shift register)
    direction_.store(new_direction, std::memory_order_relaxed);

    if (running_.load(std::memory_order_acquire)) {
        // Already running - adjust velocity
        params_.params_changed.store(true, std::memory_order_release);
    } else {
        // Starting fresh
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
        }

        running_.store(true, std::memory_order_release);
        rmt_stopped_.store(false, std::memory_order_release);
        startRmtTransmission();
    }

    xSemaphoreGive(ramp_semaphore_);

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
    float current_vel = current_velocity_;
    int32_t stop_distance = calculateDecelDistance(current_vel, deceleration);
    int32_t new_target = queue_end_.position + stop_distance;

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
    // Fill initial commands
    fillQueue();

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
