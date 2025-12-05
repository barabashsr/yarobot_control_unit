/**
 * @file ledc_pulse_gen.cpp
 * @brief LEDC-based pulse generator implementation with software pulse counting
 * @author YaRobot Team
 * @date 2025
 *
 * Architecture:
 * - LEDC generates variable-frequency PWM for trapezoidal motion profile
 * - esp_timer counts pulses via periodic callback (matches LEDC frequency)
 * - Profile update timer adjusts LEDC frequency during motion
 * - Completion callback deferred to task context via FreeRTOS notification
 */

#include "ledc_pulse_gen.h"
#include "tpic6b595.h"
#include "config_limits.h"
#include "config_timing.h"
#include "config_gpio.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include <cmath>
#include <algorithm>

static const char* TAG = "LEDC_PULSE";

// Task priority for completion handler (high priority for responsive callback)
static constexpr UBaseType_t COMPLETION_TASK_PRIORITY = configMAX_PRIORITIES - 2;
static constexpr uint32_t COMPLETION_TASK_STACK = 4096;

// Task notification bits
static constexpr uint32_t NOTIFY_COMPLETE_BIT = 0x01;
static constexpr uint32_t NOTIFY_EXIT_BIT = 0x02;

// Profile update interval in microseconds (1ms = 1000us)
static constexpr uint64_t PROFILE_UPDATE_INTERVAL_US = 1000;

// ============================================================================
// Constructor / Destructor
// ============================================================================

LedcPulseGenerator::LedcPulseGenerator(int gpio_num, ledc_timer_t timer, ledc_channel_t channel)
    : gpio_num_(gpio_num)
    , timer_(timer)
    , channel_(channel)
    , initialized_(false)
    , state_(LedcProfileState::IDLE)
    , mode_(LedcMotionMode::POSITION)
    , profile_{}
    , direction_(true)
    , last_direction_(true)
    , start_time_us_(0)
    , current_velocity_(0.0f)
    , accumulated_pulses_(0.0)
    , profile_timer_(nullptr)
    , position_timer_(nullptr)
    , position_tracker_(nullptr)
    , last_reported_pulses_(0)
    , completion_task_handle_(nullptr)
    , task_should_exit_(false)
    , completion_callback_(nullptr)
    , callback_mutex_(nullptr)
{
    callback_mutex_ = xSemaphoreCreateMutex();
}

LedcPulseGenerator::~LedcPulseGenerator()
{
    // Signal task to exit
    if (completion_task_handle_) {
        task_should_exit_.store(true, std::memory_order_release);
        xTaskNotify(completion_task_handle_, NOTIFY_EXIT_BIT, eSetBits);
        // Wait for task to exit
        vTaskDelay(pdMS_TO_TICKS(100));
        completion_task_handle_ = nullptr;
    }

    if (initialized_) {
        stopImmediate();

        // Clean up timers
        if (profile_timer_) {
            esp_timer_stop(profile_timer_);
            esp_timer_delete(profile_timer_);
            profile_timer_ = nullptr;
        }
        if (position_timer_) {
            esp_timer_stop(position_timer_);
            esp_timer_delete(position_timer_);
            position_timer_ = nullptr;
        }

        // Stop LEDC
        ledc_stop(LEDC_MODE_D, channel_, 0);
    }

    if (callback_mutex_) {
        vSemaphoreDelete(callback_mutex_);
    }
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t LedcPulseGenerator::init()
{
    if (initialized_) {
        ESP_LOGW(TAG, "LEDC timer %d already initialized", timer_);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing LEDC timer %d, channel %d on GPIO %d",
             timer_, channel_, gpio_num_);

    // =========================================================================
    // LEDC Timer Configuration
    // =========================================================================
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = LEDC_MODE_D;
    timer_config.duty_resolution = static_cast<ledc_timer_bit_t>(LEDC_RESOLUTION_BITS);
    timer_config.timer_num = timer_;
    timer_config.freq_hz = LIMIT_LEDC_MIN_FREQ_HZ;  // Start at LEDC minimum frequency
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // =========================================================================
    // LEDC Channel Configuration
    // =========================================================================
    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = gpio_num_;
    channel_config.speed_mode = LEDC_MODE_D;
    channel_config.channel = channel_;
    channel_config.timer_sel = timer_;
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.duty = 0;  // Start with duty = 0 (no output)
    channel_config.hpoint = 0;

    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // =========================================================================
    // Profile Update Timer (esp_timer) - updates LEDC frequency and accumulates pulses
    // =========================================================================
    esp_timer_create_args_t profile_timer_args = {};
    profile_timer_args.callback = profileTimerCallback;
    profile_timer_args.arg = this;
    profile_timer_args.dispatch_method = ESP_TIMER_TASK;
    profile_timer_args.name = "ledc_profile";

    ret = esp_timer_create(&profile_timer_args, &profile_timer_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create profile timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // =========================================================================
    // Position Update Timer (esp_timer) - for real-time position tracking
    // =========================================================================
    esp_timer_create_args_t position_timer_args = {};
    position_timer_args.callback = positionTimerCallback;
    position_timer_args.arg = this;
    position_timer_args.dispatch_method = ESP_TIMER_TASK;
    position_timer_args.name = "ledc_position";

    ret = esp_timer_create(&position_timer_args, &position_timer_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create position timer: %s", esp_err_to_name(ret));
        esp_timer_delete(profile_timer_);
        profile_timer_ = nullptr;
        return ret;
    }

    // =========================================================================
    // Create Completion Handler Task
    // =========================================================================
    char task_name[24];
    snprintf(task_name, sizeof(task_name), "ledc_complete_%d", timer_);
    BaseType_t xret = xTaskCreatePinnedToCore(
        completionTaskEntry,
        task_name,
        COMPLETION_TASK_STACK,
        this,
        COMPLETION_TASK_PRIORITY,
        &completion_task_handle_,
        1  // Pin to core 1 (motion core)
    );
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create completion task");
        esp_timer_delete(profile_timer_);
        esp_timer_delete(position_timer_);
        profile_timer_ = nullptr;
        position_timer_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "LEDC timer %d initialized successfully", timer_);
    return ESP_OK;
}

// ============================================================================
// Completion Task
// ============================================================================

void LedcPulseGenerator::completionTaskEntry(void* arg)
{
    LedcPulseGenerator* self = static_cast<LedcPulseGenerator*>(arg);
    self->completionTaskLoop();
}

void LedcPulseGenerator::completionTaskLoop()
{
    ESP_LOGI(TAG, "Completion task started for LEDC timer %d", timer_);

    while (!task_should_exit_.load(std::memory_order_acquire)) {
        uint32_t notification = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &notification, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (notification & NOTIFY_EXIT_BIT) {
                break;
            }
            if (notification & NOTIFY_COMPLETE_BIT) {
                notifyCompletion();
            }
        }
    }

    ESP_LOGI(TAG, "Completion task exiting for LEDC timer %d", timer_);
    vTaskDelete(nullptr);
}

// ============================================================================
// Timer Callbacks
// ============================================================================

void LedcPulseGenerator::profileTimerCallback(void* arg)
{
    LedcPulseGenerator* self = static_cast<LedcPulseGenerator*>(arg);
    self->handleProfileUpdate();
}

void LedcPulseGenerator::handleProfileUpdate()
{
    LedcProfileState current_state = state_.load(std::memory_order_acquire);
    if (current_state == LedcProfileState::IDLE) {
        return;
    }

    // Get current velocity and accumulate pulses based on time elapsed
    // Profile timer runs every PROFILE_UPDATE_INTERVAL_US (1ms)
    float velocity = current_velocity_.load(std::memory_order_relaxed);

    // Calculate pulses generated in this interval: pulses = freq * time
    // time = PROFILE_UPDATE_INTERVAL_US / 1000000.0 seconds
    double pulses_this_interval = velocity * (PROFILE_UPDATE_INTERVAL_US / 1000000.0);
    double current_accumulated = accumulated_pulses_.load(std::memory_order_relaxed);
    current_accumulated += pulses_this_interval;
    accumulated_pulses_.store(current_accumulated, std::memory_order_relaxed);

    int64_t current_count = static_cast<int64_t>(current_accumulated);
    profile_.current_pulse = current_count;

    // Check for completion in position mode
    if (mode_ == LedcMotionMode::POSITION && current_count >= profile_.target_pulses) {
        // Stop output
        stopPulseOutput();

        // Update state
        state_.store(LedcProfileState::IDLE, std::memory_order_release);
        current_velocity_.store(0.0f, std::memory_order_relaxed);

        // Signal completion to task
        if (completion_task_handle_) {
            xTaskNotify(completion_task_handle_, NOTIFY_COMPLETE_BIT, eSetBits);
        }
        return;
    }

    // Calculate target velocity at current position for profile following
    float target_velocity = velocityAtPosition(current_count);

    // Clamp velocity to LEDC limits
    if (target_velocity < LIMIT_LEDC_MIN_FREQ_HZ) {
        target_velocity = LIMIT_LEDC_MIN_FREQ_HZ;
    }
    if (target_velocity > LIMIT_LEDC_MAX_FREQ_HZ) {
        target_velocity = LIMIT_LEDC_MAX_FREQ_HZ;
    }
    if (target_velocity > profile_.cruise_velocity) {
        target_velocity = profile_.cruise_velocity;
    }

    // Update state machine
    if (current_state == LedcProfileState::ACCELERATING &&
        target_velocity >= profile_.cruise_velocity - 1.0f) {
        if (profile_.cruise_pulses > 0 || mode_ == LedcMotionMode::VELOCITY) {
            state_.store(LedcProfileState::CRUISING, std::memory_order_release);
        } else {
            state_.store(LedcProfileState::DECELERATING, std::memory_order_release);
        }
    } else if (current_state == LedcProfileState::CRUISING &&
               mode_ == LedcMotionMode::POSITION &&
               current_count >= profile_.accel_pulses + profile_.cruise_pulses) {
        state_.store(LedcProfileState::DECELERATING, std::memory_order_release);
    }

    // Update LEDC frequency and store current velocity
    setFrequency(static_cast<uint32_t>(target_velocity));
    current_velocity_.store(target_velocity, std::memory_order_relaxed);
    profile_.current_velocity = target_velocity;
}

// ============================================================================
// Motion Control
// ============================================================================

esp_err_t LedcPulseGenerator::startMove(int32_t pulses, float max_velocity, float acceleration)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Validate parameters
    if (pulses == 0) {
        return ESP_OK;  // Nothing to do
    }
    if (max_velocity <= 0 || max_velocity > LIMIT_LEDC_MAX_FREQ_HZ) {
        ESP_LOGE(TAG, "Invalid velocity: %.1f (must be 0 < v <= %d)",
                 max_velocity, LIMIT_LEDC_MAX_FREQ_HZ);
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

    // Set direction via shift register if direction changed
    if (direction_ != last_direction_) {
        sr_set_direction(SR_AXIS_D, direction_);
        sr_update();

        // Wait for direction setup time
        ets_delay_us(TIMING_DIR_SETUP_US);
        last_direction_ = direction_;
    }

    // Calculate trapezoidal profile
    mode_ = LedcMotionMode::POSITION;
    calculateTrapezoidalProfile(abs_pulses, max_velocity, acceleration);

    // Reset counters - start with minimum frequency (will ramp up)
    accumulated_pulses_.store(0.0, std::memory_order_relaxed);
    current_velocity_.store(static_cast<float>(LIMIT_LEDC_MIN_FREQ_HZ), std::memory_order_relaxed);
    start_time_us_ = esp_timer_get_time();
    profile_.current_pulse = 0;
    profile_.current_velocity = LIMIT_LEDC_MIN_FREQ_HZ;
    last_reported_pulses_.store(0, std::memory_order_relaxed);

    // Set direction on position tracker before motion
    if (position_tracker_) {
        position_tracker_->setDirection(direction_);
    }

    // Start output
    state_.store(LedcProfileState::ACCELERATING, std::memory_order_release);
    esp_err_t ret = startPulseOutput();
    if (ret != ESP_OK) {
        state_.store(LedcProfileState::IDLE, std::memory_order_release);
        return ret;
    }

    return ESP_OK;
}

esp_err_t LedcPulseGenerator::startVelocity(float velocity, float acceleration)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    float abs_velocity = std::fabs(velocity);
    if (abs_velocity < LIMIT_LEDC_MIN_FREQ_HZ || abs_velocity > LIMIT_LEDC_MAX_FREQ_HZ) {
        ESP_LOGE(TAG, "Invalid velocity: %.1f (must be %d <= |v| <= %d)",
                 velocity, LIMIT_LEDC_MIN_FREQ_HZ, LIMIT_LEDC_MAX_FREQ_HZ);
        return ESP_ERR_INVALID_ARG;
    }
    if (acceleration <= 0) {
        ESP_LOGE(TAG, "Invalid acceleration: %.1f (must be > 0)", acceleration);
        return ESP_ERR_INVALID_ARG;
    }

    direction_ = (velocity >= 0);

    ESP_LOGD(TAG, "startVelocity: vel=%.1f, accel=%.1f, dir=%s",
             velocity, acceleration, direction_ ? "FWD" : "REV");

    // Set direction via shift register if direction changed
    if (direction_ != last_direction_) {
        sr_set_direction(SR_AXIS_D, direction_);
        sr_update();
        ets_delay_us(TIMING_DIR_SETUP_US);
        last_direction_ = direction_;
    }

    // Set up velocity mode profile
    mode_ = LedcMotionMode::VELOCITY;
    calculateVelocityProfile(abs_velocity, acceleration);

    // Reset counters - start with minimum frequency (will ramp up)
    accumulated_pulses_.store(0.0, std::memory_order_relaxed);
    current_velocity_.store(static_cast<float>(LIMIT_LEDC_MIN_FREQ_HZ), std::memory_order_relaxed);
    start_time_us_ = esp_timer_get_time();
    profile_.current_pulse = 0;
    profile_.current_velocity = LIMIT_LEDC_MIN_FREQ_HZ;
    last_reported_pulses_.store(0, std::memory_order_relaxed);

    // Set direction on position tracker before motion
    if (position_tracker_) {
        position_tracker_->setDirection(direction_);
    }

    // Start output
    state_.store(LedcProfileState::ACCELERATING, std::memory_order_release);
    esp_err_t ret = startPulseOutput();
    if (ret != ESP_OK) {
        state_.store(LedcProfileState::IDLE, std::memory_order_release);
        return ret;
    }

    return ESP_OK;
}

esp_err_t LedcPulseGenerator::stop(float deceleration)
{
    LedcProfileState current = state_.load(std::memory_order_acquire);
    if (current == LedcProfileState::IDLE) {
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
    int64_t current_pos = static_cast<int64_t>(accumulated_pulses_.load(std::memory_order_relaxed));
    profile_.target_pulses = static_cast<int32_t>(current_pos + stop_pulses);
    profile_.decel_pulses = static_cast<int32_t>(stop_pulses);

    // Switch mode to position for target-based stop
    mode_ = LedcMotionMode::POSITION;
    state_.store(LedcProfileState::STOPPING, std::memory_order_release);

    return ESP_OK;
}

void LedcPulseGenerator::stopImmediate()
{
    LedcProfileState current = state_.load(std::memory_order_acquire);
    if (current == LedcProfileState::IDLE) {
        return;
    }

    ESP_LOGD(TAG, "stopImmediate");

    // Stop output immediately
    stopPulseOutput();

    // Read final count
    int64_t final_count = static_cast<int64_t>(accumulated_pulses_.load(std::memory_order_relaxed));

    // Clear state
    state_.store(LedcProfileState::IDLE, std::memory_order_release);
    current_velocity_.store(0.0f, std::memory_order_relaxed);

    ESP_LOGD(TAG, "Immediate stop complete, final count: %lld", (long long)final_count);

    // Note: Do NOT fire completion callback on immediate stop
}

// ============================================================================
// Status Methods
// ============================================================================

bool LedcPulseGenerator::isRunning() const
{
    return state_.load(std::memory_order_acquire) != LedcProfileState::IDLE;
}

int64_t LedcPulseGenerator::getPulseCount() const
{
    return static_cast<int64_t>(accumulated_pulses_.load(std::memory_order_relaxed));
}

float LedcPulseGenerator::getCurrentVelocity() const
{
    return current_velocity_.load(std::memory_order_relaxed);
}

void LedcPulseGenerator::setCompletionCallback(MotionCompleteCallback cb)
{
    if (xSemaphoreTake(callback_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        completion_callback_ = cb;
        xSemaphoreGive(callback_mutex_);
    }
}

void LedcPulseGenerator::setPositionTracker(IPositionTracker* tracker)
{
    position_tracker_ = tracker;
    ESP_LOGI(TAG, "setPositionTracker: tracker=%p", (void*)tracker);
}

// ============================================================================
// Position Timer Callback
// ============================================================================

void LedcPulseGenerator::positionTimerCallback(void* arg)
{
    LedcPulseGenerator* self = static_cast<LedcPulseGenerator*>(arg);
    self->handlePositionUpdate();
}

void LedcPulseGenerator::handlePositionUpdate()
{
    if (!position_tracker_) {
        return;
    }

    // Get current pulse count (from time-based accumulation) and compute delta since last report
    int64_t current = static_cast<int64_t>(accumulated_pulses_.load(std::memory_order_relaxed));
    int64_t last = last_reported_pulses_.load(std::memory_order_relaxed);
    int64_t delta = current - last;

    if (delta > 0) {
        position_tracker_->addPulses(delta);
        last_reported_pulses_.store(current, std::memory_order_relaxed);
    }
}

// ============================================================================
// Profile Calculation
// ============================================================================

void LedcPulseGenerator::calculateTrapezoidalProfile(int32_t pulses, float max_vel, float accel)
{
    profile_.target_pulses = pulses;
    profile_.max_velocity = max_vel;
    profile_.acceleration = accel;
    profile_.deceleration = accel;  // Symmetric by default

    // Clamp max velocity to LEDC limit
    if (max_vel > LIMIT_LEDC_MAX_FREQ_HZ) {
        max_vel = LIMIT_LEDC_MAX_FREQ_HZ;
        profile_.max_velocity = max_vel;
    }

    // Calculate acceleration distance: v^2 / (2 * a)
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

void LedcPulseGenerator::calculateVelocityProfile(float velocity, float acceleration)
{
    // Clamp velocity to LEDC limit
    if (velocity > LIMIT_LEDC_MAX_FREQ_HZ) {
        velocity = LIMIT_LEDC_MAX_FREQ_HZ;
    }

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

float LedcPulseGenerator::velocityAtPosition(int64_t position) const
{
    if (mode_ == LedcMotionMode::VELOCITY) {
        LedcProfileState current = state_.load(std::memory_order_acquire);
        if (current == LedcProfileState::ACCELERATING) {
            // v = sqrt(2 * a * d)
            float v = std::sqrt(2.0f * profile_.acceleration * static_cast<float>(position));
            return std::min(v, profile_.cruise_velocity);
        } else if (current == LedcProfileState::STOPPING) {
            int64_t stop_start = profile_.target_pulses - profile_.decel_pulses;
            if (position >= stop_start) {
                int64_t decel_pos = position - stop_start;
                float remaining = static_cast<float>(profile_.decel_pulses - decel_pos);
                if (remaining <= 0) return LIMIT_LEDC_MIN_FREQ_HZ;
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
        if (remaining <= 0) return LIMIT_LEDC_MIN_FREQ_HZ;
        return std::sqrt(2.0f * profile_.deceleration * static_cast<float>(remaining));
    }
}

// ============================================================================
// LEDC Control
// ============================================================================

esp_err_t LedcPulseGenerator::setFrequency(uint32_t frequency)
{
    if (frequency < LIMIT_LEDC_MIN_FREQ_HZ) {
        frequency = LIMIT_LEDC_MIN_FREQ_HZ;
    }
    if (frequency > LIMIT_LEDC_MAX_FREQ_HZ) {
        frequency = LIMIT_LEDC_MAX_FREQ_HZ;
    }

    return ledc_set_freq(LEDC_MODE_D, timer_, frequency);
}

esp_err_t LedcPulseGenerator::startPulseOutput()
{
    // Set initial frequency
    esp_err_t ret = setFrequency(LIMIT_LEDC_MIN_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set initial frequency: %s", esp_err_to_name(ret));
        return ret;
    }

    // Calculate 50% duty cycle value
    uint32_t duty = (1 << LEDC_RESOLUTION_BITS) * LEDC_DUTY_CYCLE_PERCENT / 100;

    // Set duty cycle
    ret = ledc_set_duty(LEDC_MODE_D, channel_, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set duty: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_update_duty(LEDC_MODE_D, channel_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update duty: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start profile update timer (also handles pulse accumulation)
    ret = esp_timer_start_periodic(profile_timer_, PROFILE_UPDATE_INTERVAL_US);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start profile timer: %s", esp_err_to_name(ret));
        ledc_set_duty(LEDC_MODE_D, channel_, 0);
        ledc_update_duty(LEDC_MODE_D, channel_);
        return ret;
    }

    // Start position update timer (only if tracker is set)
    if (position_tracker_ && position_timer_) {
        // Stop first in case it's already running from previous motion
        esp_timer_stop(position_timer_);
        ret = esp_timer_start_periodic(position_timer_, TIMING_LEDC_POSITION_UPDATE_MS * 1000);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to start position timer: %s", esp_err_to_name(ret));
            // Non-fatal - position tracking just won't work in real-time
        } else {
            ESP_LOGI(TAG, "Position timer started, interval=%d ms", TIMING_LEDC_POSITION_UPDATE_MS);
        }
    }

    return ESP_OK;
}

void LedcPulseGenerator::stopPulseOutput()
{
    // Stop timers
    if (profile_timer_) {
        esp_timer_stop(profile_timer_);
    }
    if (position_timer_) {
        esp_timer_stop(position_timer_);
    }

    // Final position update to sync exact count
    if (position_tracker_) {
        int64_t current = static_cast<int64_t>(accumulated_pulses_.load(std::memory_order_relaxed));
        int64_t last = last_reported_pulses_.load(std::memory_order_relaxed);
        int64_t delta = current - last;
        if (delta > 0) {
            position_tracker_->addPulses(delta);
            last_reported_pulses_.store(current, std::memory_order_relaxed);
        }
    }

    // Set duty to 0 to stop LEDC output
    ledc_set_duty(LEDC_MODE_D, channel_, 0);
    ledc_update_duty(LEDC_MODE_D, channel_);
}

// ============================================================================
// Completion Notification
// ============================================================================

void LedcPulseGenerator::notifyCompletion()
{
    if (xSemaphoreTake(callback_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (completion_callback_) {
            int64_t total = static_cast<int64_t>(accumulated_pulses_.load(std::memory_order_relaxed));
            completion_callback_(total);
        }
        xSemaphoreGive(callback_mutex_);
    }
}
