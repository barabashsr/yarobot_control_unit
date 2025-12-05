/**
 * @file rmt_pulse_gen.cpp
 * @brief RMT-based pulse generator implementation
 * @author YaRobot Team
 * @date 2025
 *
 * Architecture:
 * - ISR callback only updates counters and signals refill task (ISR-safe)
 * - Dedicated high-priority task handles buffer refill (uses FPU, safe for sqrt)
 * - Double-buffering: while one buffer transmits, next is prepared
 * - Trapezoidal profile for smooth acceleration/deceleration
 */

#include "rmt_pulse_gen.h"
#include "config_limits.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>
#include <algorithm>

static const char* TAG = "RMT_PULSE";

// Task priority for buffer refill (high priority for low latency)
static constexpr UBaseType_t REFILL_TASK_PRIORITY = configMAX_PRIORITIES - 2;
static constexpr uint32_t REFILL_TASK_STACK = 4096;

// Task notification index for refill signaling
static constexpr uint32_t NOTIFY_REFILL_BIT = 0x01;
static constexpr uint32_t NOTIFY_EXIT_BIT = 0x02;

// ============================================================================
// Constructor / Destructor
// ============================================================================

RmtPulseGenerator::RmtPulseGenerator(int channel_id, int gpio_num, uint32_t resolution_hz)
    : channel_id_(channel_id)
    , gpio_num_(gpio_num)
    , resolution_hz_(resolution_hz)
    , channel_handle_(nullptr)
    , encoder_handle_(nullptr)
    , initialized_(false)
    , state_(ProfileState::IDLE)
    , mode_(MotionMode::POSITION)
    , profile_{}
    , direction_(true)
    , pulse_count_(0)
    , current_velocity_(0.0f)
    , use_buffer_a_(true)
    , symbols_in_current_buffer_(0)
    , symbols_in_next_buffer_(0)
    , refill_task_handle_(nullptr)
    , refill_pending_(false)
    , task_should_exit_(false)
    , completion_callback_(nullptr)
    , callback_mutex_(nullptr)
    , position_tracker_(nullptr)
    , last_reported_pulses_(0)
{
    callback_mutex_ = xSemaphoreCreateMutex();
}

RmtPulseGenerator::~RmtPulseGenerator()
{
    // Signal task to exit
    if (refill_task_handle_) {
        task_should_exit_.store(true, std::memory_order_release);
        xTaskNotify(refill_task_handle_, NOTIFY_EXIT_BIT, eSetBits);
        // Wait for task to exit
        vTaskDelay(pdMS_TO_TICKS(100));
        // Task should have deleted itself, but clean up handle
        refill_task_handle_ = nullptr;
    }

    if (initialized_) {
        stopImmediate();
        if (channel_handle_) {
            rmt_disable(channel_handle_);
            rmt_del_channel(channel_handle_);
        }
        if (encoder_handle_) {
            rmt_del_encoder(encoder_handle_);
        }
    }
    if (callback_mutex_) {
        vSemaphoreDelete(callback_mutex_);
    }
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t RmtPulseGenerator::init()
{
    if (initialized_) {
        ESP_LOGW(TAG, "Channel %d already initialized", channel_id_);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing RMT channel %d on GPIO %d at %lu Hz",
             channel_id_, gpio_num_, (unsigned long)resolution_hz_);

    // Configure RMT TX channel
    rmt_tx_channel_config_t tx_config = {};
    tx_config.gpio_num = static_cast<gpio_num_t>(gpio_num_);
    tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_config.resolution_hz = resolution_hz_;
    tx_config.mem_block_symbols = LIMIT_RMT_BUFFER_SYMBOLS;
    tx_config.trans_queue_depth = 4;   // Queue depth for streaming
    tx_config.flags.with_dma = true;   // Enable DMA for streaming
    tx_config.flags.invert_out = false;

    esp_err_t ret = rmt_new_tx_channel(&tx_config, &channel_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create copy encoder for raw symbol transmission
    rmt_copy_encoder_config_t encoder_config = {};
    ret = rmt_new_copy_encoder(&encoder_config, &encoder_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create copy encoder: %s", esp_err_to_name(ret));
        rmt_del_channel(channel_handle_);
        channel_handle_ = nullptr;
        return ret;
    }

    // Register TX done callback for signaling refill task
    rmt_tx_event_callbacks_t cbs = {};
    cbs.on_trans_done = onTxDone;
    ret = rmt_tx_register_event_callbacks(channel_handle_, &cbs, this);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register callbacks: %s", esp_err_to_name(ret));
        rmt_del_encoder(encoder_handle_);
        rmt_del_channel(channel_handle_);
        encoder_handle_ = nullptr;
        channel_handle_ = nullptr;
        return ret;
    }

    // Enable the channel
    ret = rmt_enable(channel_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        rmt_del_encoder(encoder_handle_);
        rmt_del_channel(channel_handle_);
        encoder_handle_ = nullptr;
        channel_handle_ = nullptr;
        return ret;
    }

    // Create refill task
    char task_name[16];
    snprintf(task_name, sizeof(task_name), "rmt_refill_%d", channel_id_);
    BaseType_t xret = xTaskCreatePinnedToCore(
        refillTaskEntry,
        task_name,
        REFILL_TASK_STACK,
        this,
        REFILL_TASK_PRIORITY,
        &refill_task_handle_,
        1  // Pin to core 1 (motion core)
    );
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create refill task");
        rmt_disable(channel_handle_);
        rmt_del_encoder(encoder_handle_);
        rmt_del_channel(channel_handle_);
        encoder_handle_ = nullptr;
        channel_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "RMT channel %d initialized successfully", channel_id_);
    return ESP_OK;
}

// ============================================================================
// Refill Task
// ============================================================================

void RmtPulseGenerator::refillTaskEntry(void* arg)
{
    RmtPulseGenerator* self = static_cast<RmtPulseGenerator*>(arg);
    self->refillTaskLoop();
}

void RmtPulseGenerator::refillTaskLoop()
{
    ESP_LOGI(TAG, "Refill task started for channel %d", channel_id_);

    while (!task_should_exit_.load(std::memory_order_acquire)) {
        // Wait for notification from ISR or exit signal
        uint32_t notification = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &notification, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (notification & NOTIFY_EXIT_BIT) {
                break;
            }
            if (notification & NOTIFY_REFILL_BIT) {
                handleRefillRequest();
            }
        }
    }

    ESP_LOGI(TAG, "Refill task exiting for channel %d", channel_id_);
    vTaskDelete(nullptr);
}

void RmtPulseGenerator::handleRefillRequest()
{
    ProfileState current = state_.load(std::memory_order_acquire);
    if (current == ProfileState::IDLE) {
        // Motion just completed - notify callback
        notifyCompletion();
        return;
    }

    // Get the next buffer to fill
    bool current_is_a = use_buffer_a_.load(std::memory_order_acquire);
    rmt_symbol_word_t* next_buffer = current_is_a ? buffer_b_ : buffer_a_;

    // Fill the next buffer
    size_t new_symbols = fillBuffer(next_buffer, LIMIT_RMT_BUFFER_SYMBOLS);
    symbols_in_next_buffer_.store(new_symbols, std::memory_order_release);

    if (new_symbols == 0) {
        // Motion complete - ISR will set state to IDLE and signal us again
        // We'll call notifyCompletion() when we see IDLE state
        return;
    }

    // Queue next transmission
    rmt_transmit_config_t tx_config = {};
    tx_config.loop_count = 0;

    esp_err_t ret = rmt_transmit(channel_handle_, encoder_handle_,
                                  next_buffer, new_symbols * sizeof(rmt_symbol_word_t),
                                  &tx_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to queue next buffer: %s", esp_err_to_name(ret));
        state_.store(ProfileState::IDLE, std::memory_order_release);
        current_velocity_.store(0.0f, std::memory_order_relaxed);
    }
}

// ============================================================================
// Motion Control
// ============================================================================

esp_err_t RmtPulseGenerator::startMove(int32_t pulses, float max_velocity, float acceleration)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Validate parameters
    if (pulses == 0) {
        return ESP_OK;  // Nothing to do
    }
    if (max_velocity <= 0 || max_velocity > LIMIT_MAX_PULSE_FREQ_HZ) {
        ESP_LOGE(TAG, "Invalid velocity: %.1f (must be 0 < v <= %d)",
                 max_velocity, LIMIT_MAX_PULSE_FREQ_HZ);
        return ESP_ERR_INVALID_ARG;
    }
    if (acceleration <= 0) {
        ESP_LOGE(TAG, "Invalid acceleration: %.1f (must be > 0)", acceleration);
        return ESP_ERR_INVALID_ARG;
    }

    // Handle direction
    direction_ = (pulses > 0);
    int32_t abs_pulses = std::abs(pulses);

    ESP_LOGD(TAG, "startMove: pulses=%ld, max_vel=%.1f, accel=%.1f, dir=%s",
             (long)pulses, max_velocity, acceleration, direction_ ? "FWD" : "REV");

    // Calculate trapezoidal profile
    mode_ = MotionMode::POSITION;
    calculateTrapezoidalProfile(abs_pulses, max_velocity, acceleration);

    // Reset counters
    pulse_count_.store(0, std::memory_order_relaxed);
    current_velocity_.store(0.0f, std::memory_order_relaxed);
    profile_.current_pulse = 0;
    profile_.current_velocity = 0.0f;
    last_reported_pulses_ = 0;

    // Set direction on position tracker before motion
    if (position_tracker_) {
        position_tracker_->setDirection(direction_);
    }

    // Prime both buffers
    primeBuffers();

    // Start transmission
    state_.store(ProfileState::ACCELERATING, std::memory_order_release);

    rmt_transmit_config_t tx_config = {};
    tx_config.loop_count = 0;  // No loop, we stream manually

    // Transmit first buffer
    rmt_symbol_word_t* buffer = buffer_a_;
    size_t symbols = symbols_in_current_buffer_.load(std::memory_order_acquire);
    esp_err_t ret = rmt_transmit(channel_handle_, encoder_handle_,
                                  buffer, symbols * sizeof(rmt_symbol_word_t),
                                  &tx_config);
    if (ret != ESP_OK) {
        state_.store(ProfileState::IDLE, std::memory_order_release);
        ESP_LOGE(TAG, "Failed to start RMT transmission: %s", esp_err_to_name(ret));
        return ret;
    }

    // Immediately queue second buffer if available
    if (symbols_in_next_buffer_.load(std::memory_order_acquire) > 0) {
        ret = rmt_transmit(channel_handle_, encoder_handle_,
                           buffer_b_, symbols_in_next_buffer_.load() * sizeof(rmt_symbol_word_t),
                           &tx_config);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to queue second buffer: %s", esp_err_to_name(ret));
        }
    }

    return ESP_OK;
}

esp_err_t RmtPulseGenerator::startVelocity(float velocity, float acceleration)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    float abs_velocity = std::fabs(velocity);
    if (abs_velocity < LIMIT_MIN_PULSE_FREQ_HZ || abs_velocity > LIMIT_MAX_PULSE_FREQ_HZ) {
        ESP_LOGE(TAG, "Invalid velocity: %.1f (must be %d <= |v| <= %d)",
                 velocity, LIMIT_MIN_PULSE_FREQ_HZ, LIMIT_MAX_PULSE_FREQ_HZ);
        return ESP_ERR_INVALID_ARG;
    }
    if (acceleration <= 0) {
        ESP_LOGE(TAG, "Invalid acceleration: %.1f (must be > 0)", acceleration);
        return ESP_ERR_INVALID_ARG;
    }

    direction_ = (velocity >= 0);

    ESP_LOGD(TAG, "startVelocity: vel=%.1f, accel=%.1f, dir=%s",
             velocity, acceleration, direction_ ? "FWD" : "REV");

    // Set up velocity mode profile
    mode_ = MotionMode::VELOCITY;
    calculateVelocityProfile(std::fabs(velocity), acceleration);

    // Reset counters
    pulse_count_.store(0, std::memory_order_relaxed);
    current_velocity_.store(0.0f, std::memory_order_relaxed);
    profile_.current_pulse = 0;
    profile_.current_velocity = 0.0f;
    last_reported_pulses_ = 0;

    // Set direction on position tracker before motion
    if (position_tracker_) {
        position_tracker_->setDirection(direction_);
    }

    // Prime buffers
    primeBuffers();

    // Start transmission
    state_.store(ProfileState::ACCELERATING, std::memory_order_release);

    rmt_transmit_config_t tx_config = {};
    tx_config.loop_count = 0;

    rmt_symbol_word_t* buffer = buffer_a_;
    size_t symbols = symbols_in_current_buffer_.load(std::memory_order_acquire);
    esp_err_t ret = rmt_transmit(channel_handle_, encoder_handle_,
                                  buffer, symbols * sizeof(rmt_symbol_word_t),
                                  &tx_config);
    if (ret != ESP_OK) {
        state_.store(ProfileState::IDLE, std::memory_order_release);
        ESP_LOGE(TAG, "Failed to start RMT transmission: %s", esp_err_to_name(ret));
        return ret;
    }

    // Queue second buffer
    if (symbols_in_next_buffer_.load(std::memory_order_acquire) > 0) {
        rmt_transmit(channel_handle_, encoder_handle_,
                     buffer_b_, symbols_in_next_buffer_.load() * sizeof(rmt_symbol_word_t),
                     &tx_config);
    }

    return ESP_OK;
}

esp_err_t RmtPulseGenerator::stop(float deceleration)
{
    ProfileState current = state_.load(std::memory_order_acquire);
    if (current == ProfileState::IDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    if (deceleration <= 0) {
        ESP_LOGE(TAG, "Invalid deceleration: %.1f (must be > 0)", deceleration);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "stop: decel=%.1f", deceleration);

    // Switch to deceleration mode
    profile_.deceleration = deceleration;

    // Calculate pulses needed to stop from current velocity
    float current_vel = current_velocity_.load(std::memory_order_relaxed);
    float stop_pulses = (current_vel * current_vel) / (2.0f * deceleration);

    // Update profile for stopping
    profile_.target_pulses = static_cast<int32_t>(profile_.current_pulse + stop_pulses);
    profile_.decel_pulses = static_cast<int32_t>(stop_pulses);

    state_.store(ProfileState::STOPPING, std::memory_order_release);

    return ESP_OK;
}

void RmtPulseGenerator::stopImmediate()
{
    ProfileState current = state_.load(std::memory_order_acquire);
    if (current == ProfileState::IDLE) {
        return;
    }

    ESP_LOGD(TAG, "stopImmediate");

    // Disable RMT channel immediately
    if (channel_handle_) {
        rmt_disable(channel_handle_);
        // Re-enable for next use (but don't transmit)
        rmt_enable(channel_handle_);
    }

    // Clear state
    state_.store(ProfileState::IDLE, std::memory_order_release);
    current_velocity_.store(0.0f, std::memory_order_relaxed);

    // Note: Do NOT fire completion callback on immediate stop
}

// ============================================================================
// Status Methods
// ============================================================================

bool RmtPulseGenerator::isRunning() const
{
    return state_.load(std::memory_order_acquire) != ProfileState::IDLE;
}

int64_t RmtPulseGenerator::getPulseCount() const
{
    return pulse_count_.load(std::memory_order_relaxed);
}

float RmtPulseGenerator::getCurrentVelocity() const
{
    return current_velocity_.load(std::memory_order_relaxed);
}

void RmtPulseGenerator::setCompletionCallback(MotionCompleteCallback cb)
{
    if (xSemaphoreTake(callback_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        completion_callback_ = cb;
        xSemaphoreGive(callback_mutex_);
    }
}

void RmtPulseGenerator::setPositionTracker(IPositionTracker* tracker)
{
    position_tracker_ = tracker;
}

// ============================================================================
// Profile Calculation
// ============================================================================

void RmtPulseGenerator::calculateTrapezoidalProfile(int32_t pulses, float max_vel, float accel)
{
    profile_.target_pulses = pulses;
    profile_.max_velocity = max_vel;
    profile_.acceleration = accel;
    profile_.deceleration = accel;  // Symmetric by default

    // Calculate acceleration distance: v² / (2 * a)
    float accel_distance = (max_vel * max_vel) / (2.0f * accel);
    float total_accel_distance = 2.0f * accel_distance;  // Accel + decel

    if (static_cast<float>(pulses) <= total_accel_distance) {
        // Triangular profile - can't reach max velocity
        profile_.is_triangular = true;
        profile_.accel_pulses = pulses / 2;
        profile_.decel_pulses = pulses - profile_.accel_pulses;
        profile_.cruise_pulses = 0;
        // Peak velocity achieved: v = sqrt(2 * a * d)
        profile_.cruise_velocity = std::sqrt(2.0f * accel * static_cast<float>(profile_.accel_pulses));
    } else {
        // Full trapezoidal profile
        profile_.is_triangular = false;
        profile_.accel_pulses = static_cast<int32_t>(accel_distance);
        profile_.decel_pulses = static_cast<int32_t>(accel_distance);
        profile_.cruise_pulses = pulses - profile_.accel_pulses - profile_.decel_pulses;
        profile_.cruise_velocity = max_vel;
    }

    ESP_LOGD(TAG, "Profile: accel=%ld, cruise=%ld, decel=%ld, peak_vel=%.1f, tri=%d",
             (long)profile_.accel_pulses, (long)profile_.cruise_pulses,
             (long)profile_.decel_pulses, profile_.cruise_velocity,
             profile_.is_triangular);
}

void RmtPulseGenerator::calculateVelocityProfile(float velocity, float acceleration)
{
    profile_.max_velocity = velocity;
    profile_.acceleration = acceleration;
    profile_.deceleration = acceleration;
    profile_.target_pulses = INT32_MAX;  // Continuous mode
    profile_.cruise_velocity = velocity;
    profile_.is_triangular = false;

    // Calculate pulses needed to reach target velocity
    profile_.accel_pulses = static_cast<int32_t>((velocity * velocity) / (2.0f * acceleration));
    profile_.cruise_pulses = INT32_MAX;  // Unlimited cruise
    profile_.decel_pulses = 0;  // Will be set on stop()
}

float RmtPulseGenerator::velocityAtPosition(int64_t position) const
{
    if (mode_ == MotionMode::VELOCITY) {
        ProfileState current = state_.load(std::memory_order_acquire);
        if (current == ProfileState::ACCELERATING) {
            // v = sqrt(2 * a * d)
            float v = std::sqrt(2.0f * profile_.acceleration * static_cast<float>(position));
            return std::min(v, profile_.cruise_velocity);
        } else if (current == ProfileState::STOPPING) {
            int64_t stop_start = profile_.target_pulses - profile_.decel_pulses;
            if (position >= stop_start) {
                int64_t decel_pos = position - stop_start;
                float remaining = static_cast<float>(profile_.decel_pulses - decel_pos);
                if (remaining <= 0) return LIMIT_MIN_PULSE_FREQ_HZ;
                return std::sqrt(2.0f * profile_.deceleration * remaining);
            }
        }
        return profile_.cruise_velocity;
    }

    // Position mode
    if (position < profile_.accel_pulses) {
        // Accelerating: v = sqrt(2 * a * d)
        return std::sqrt(2.0f * profile_.acceleration * static_cast<float>(position));
    } else if (position < profile_.accel_pulses + profile_.cruise_pulses) {
        // Cruising
        return profile_.cruise_velocity;
    } else {
        // Decelerating: v = sqrt(2 * a * remaining)
        int64_t remaining = profile_.target_pulses - position;
        if (remaining <= 0) return LIMIT_MIN_PULSE_FREQ_HZ;
        return std::sqrt(2.0f * profile_.deceleration * static_cast<float>(remaining));
    }
}

uint32_t RmtPulseGenerator::velocityToTickPeriod(float velocity) const
{
    if (velocity < LIMIT_MIN_PULSE_FREQ_HZ) {
        velocity = LIMIT_MIN_PULSE_FREQ_HZ;
    }
    // Period in ticks = resolution / frequency
    return static_cast<uint32_t>(resolution_hz_ / velocity);
}

// ============================================================================
// Buffer Management
// ============================================================================

size_t RmtPulseGenerator::fillBuffer(rmt_symbol_word_t* buffer, size_t max_symbols)
{
    size_t filled = 0;
    int64_t target = profile_.target_pulses;

    while (filled < max_symbols) {
        // Check if we've completed the motion
        if (mode_ == MotionMode::POSITION && profile_.current_pulse >= target) {
            break;
        }

        // Check for stopping mode completion
        ProfileState current = state_.load(std::memory_order_acquire);
        if (current == ProfileState::STOPPING && profile_.current_pulse >= profile_.target_pulses) {
            break;
        }

        // Get velocity at current position
        float velocity = velocityAtPosition(profile_.current_pulse);

        // Clamp velocity to valid range
        if (velocity < LIMIT_MIN_PULSE_FREQ_HZ) {
            velocity = LIMIT_MIN_PULSE_FREQ_HZ;
        }
        if (velocity > profile_.cruise_velocity) {
            velocity = profile_.cruise_velocity;
        }

        // Update state machine
        if (current == ProfileState::ACCELERATING && velocity >= profile_.cruise_velocity - 1.0f) {
            if (profile_.cruise_pulses > 0 || mode_ == MotionMode::VELOCITY) {
                state_.store(ProfileState::CRUISING, std::memory_order_release);
            } else {
                state_.store(ProfileState::DECELERATING, std::memory_order_release);
            }
        } else if (current == ProfileState::CRUISING &&
                   mode_ == MotionMode::POSITION &&
                   profile_.current_pulse >= profile_.accel_pulses + profile_.cruise_pulses) {
            state_.store(ProfileState::DECELERATING, std::memory_order_release);
        }

        // Calculate tick period for this pulse
        uint32_t period_ticks = velocityToTickPeriod(velocity);
        uint32_t half_period = period_ticks / 2;

        // Ensure minimum tick count (RMT constraint)
        if (half_period < 1) half_period = 1;
        if (half_period > 32767) half_period = 32767;  // 15-bit limit

        // Create RMT symbol: HIGH for half period, LOW for half period
        buffer[filled].duration0 = half_period;
        buffer[filled].level0 = 1;
        buffer[filled].duration1 = half_period;
        buffer[filled].level1 = 0;

        filled++;
        profile_.current_pulse++;
        profile_.current_velocity = velocity;
    }

    return filled;
}

void RmtPulseGenerator::primeBuffers()
{
    use_buffer_a_.store(true, std::memory_order_release);

    // Fill buffer A
    size_t symbols_a = fillBuffer(buffer_a_, LIMIT_RMT_BUFFER_SYMBOLS);
    symbols_in_current_buffer_.store(symbols_a, std::memory_order_release);

    // Fill buffer B (next buffer)
    size_t symbols_b = fillBuffer(buffer_b_, LIMIT_RMT_BUFFER_SYMBOLS);
    symbols_in_next_buffer_.store(symbols_b, std::memory_order_release);

    ESP_LOGD(TAG, "Primed buffers: A=%zu, B=%zu symbols", symbols_a, symbols_b);
}

// ============================================================================
// RMT Callbacks - ISR SAFE
// ============================================================================

bool IRAM_ATTR RmtPulseGenerator::onTxDone(rmt_channel_handle_t channel,
                                            const rmt_tx_done_event_data_t* event_data,
                                            void* user_data)
{
    RmtPulseGenerator* self = static_cast<RmtPulseGenerator*>(user_data);
    return self->handleTxDoneISR();
}

bool RmtPulseGenerator::handleTxDoneISR()
{
    // Update pulse count with symbols from completed buffer
    size_t completed_symbols = symbols_in_current_buffer_.load(std::memory_order_relaxed);
    pulse_count_.fetch_add(completed_symbols, std::memory_order_relaxed);
    current_velocity_.store(profile_.current_velocity, std::memory_order_relaxed);

    // Update position tracker with buffer pulses (real-time position update)
    // Note: addPulses() only uses atomics, safe for ISR context
    if (position_tracker_ && completed_symbols > 0) {
        position_tracker_->addPulses(static_cast<int64_t>(completed_symbols));
    }

    // Swap buffers
    bool was_buffer_a = use_buffer_a_.load(std::memory_order_relaxed);
    use_buffer_a_.store(!was_buffer_a, std::memory_order_release);

    // Move next buffer count to current
    size_t next_symbols = symbols_in_next_buffer_.load(std::memory_order_relaxed);
    symbols_in_current_buffer_.store(next_symbols, std::memory_order_release);

    // Check if motion is complete
    ProfileState current = state_.load(std::memory_order_relaxed);
    if (current == ProfileState::IDLE) {
        return false;  // Already stopped
    }

    // Check if this was the last buffer
    if (next_symbols == 0) {
        state_.store(ProfileState::IDLE, std::memory_order_release);
        current_velocity_.store(0.0f, std::memory_order_relaxed);

        // Notify completion from task context (signal refill task)
        if (refill_task_handle_) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR(refill_task_handle_, NOTIFY_REFILL_BIT,
                               eSetBits, &xHigherPriorityTaskWoken);
            return xHigherPriorityTaskWoken == pdTRUE;
        }
        return false;
    }

    // Signal refill task to prepare next buffer
    if (refill_task_handle_) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(refill_task_handle_, NOTIFY_REFILL_BIT,
                           eSetBits, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }

    return false;
}

void RmtPulseGenerator::notifyCompletion()
{
    if (xSemaphoreTake(callback_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (completion_callback_) {
            int64_t total = pulse_count_.load(std::memory_order_relaxed);
            completion_callback_(total);
        }
        xSemaphoreGive(callback_mutex_);
    }
}
