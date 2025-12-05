/**
 * @file mcpwm_pulse_gen.cpp
 * @brief MCPWM-based pulse generator implementation with PCNT feedback
 * @author YaRobot Team
 * @date 2025
 *
 * Architecture:
 * - MCPWM generates variable-frequency PWM for trapezoidal motion profile
 * - PCNT counts pulses via internal GPIO io_loop_back
 * - PCNT watch point callback stops MCPWM when target pulse count reached
 * - Profile update task adjusts MCPWM frequency during motion
 * - Completion callback deferred to task context via FreeRTOS notification
 */

#include "mcpwm_pulse_gen.h"
#include "tpic6b595.h"
#include "config_limits.h"
#include "config_timing.h"
#include "config_gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include <cmath>
#include <algorithm>

static const char* TAG = "MCPWM_PULSE";

// Task priority for profile update (high priority for smooth motion)
static constexpr UBaseType_t PROFILE_TASK_PRIORITY = configMAX_PRIORITIES - 2;
static constexpr uint32_t PROFILE_TASK_STACK = 4096;

// Task notification bits
static constexpr uint32_t NOTIFY_UPDATE_BIT = 0x01;
static constexpr uint32_t NOTIFY_COMPLETE_BIT = 0x02;
static constexpr uint32_t NOTIFY_EXIT_BIT = 0x04;

// Profile update interval (ms) - controls how often frequency is adjusted
static constexpr uint32_t PROFILE_UPDATE_INTERVAL_MS = 1;

// PCNT high limit for watch point (16-bit signed max)
static constexpr int16_t PCNT_HIGH_LIMIT = INT16_MAX;
static constexpr int16_t PCNT_LOW_LIMIT = INT16_MIN;

// ============================================================================
// Constructor / Destructor
// ============================================================================

McpwmPulseGenerator::McpwmPulseGenerator(int timer_id, int gpio_num, int pcnt_unit_id)
    : timer_id_(timer_id)
    , gpio_num_(gpio_num)
    , pcnt_unit_id_(pcnt_unit_id)
    , timer_handle_(nullptr)
    , oper_handle_(nullptr)
    , gen_handle_(nullptr)
    , cmpr_handle_(nullptr)
    , initialized_(false)
    , pcnt_unit_(nullptr)
    , pcnt_channel_(nullptr)
    , state_(McpwmProfileState::IDLE)
    , mode_(McpwmMotionMode::POSITION)
    , profile_{}
    , direction_(true)
    , last_direction_(true)
    , pulse_count_(0)
    , overflow_count_(0)
    , current_velocity_(0.0f)
    , profile_task_handle_(nullptr)
    , task_should_exit_(false)
    , completion_callback_(nullptr)
    , callback_mutex_(nullptr)
    , position_tracker_(nullptr)
{
    callback_mutex_ = xSemaphoreCreateMutex();
}

McpwmPulseGenerator::~McpwmPulseGenerator()
{
    // Signal task to exit
    if (profile_task_handle_) {
        task_should_exit_.store(true, std::memory_order_release);
        xTaskNotify(profile_task_handle_, NOTIFY_EXIT_BIT, eSetBits);
        // Wait for task to exit
        vTaskDelay(pdMS_TO_TICKS(100));
        profile_task_handle_ = nullptr;
    }

    if (initialized_) {
        stopImmediate();

        // Clean up PCNT
        if (pcnt_channel_) {
            pcnt_del_channel(pcnt_channel_);
        }
        if (pcnt_unit_) {
            pcnt_unit_stop(pcnt_unit_);
            pcnt_del_unit(pcnt_unit_);
        }

        // Clean up MCPWM
        if (gen_handle_) {
            mcpwm_del_generator(gen_handle_);
        }
        if (cmpr_handle_) {
            mcpwm_del_comparator(cmpr_handle_);
        }
        if (oper_handle_) {
            mcpwm_del_operator(oper_handle_);
        }
        if (timer_handle_) {
            mcpwm_timer_disable(timer_handle_);
            mcpwm_del_timer(timer_handle_);
        }
    }

    if (callback_mutex_) {
        vSemaphoreDelete(callback_mutex_);
    }
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t McpwmPulseGenerator::init()
{
    if (initialized_) {
        ESP_LOGW(TAG, "Timer %d already initialized", timer_id_);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing MCPWM timer %d on GPIO %d with PCNT unit %d",
             timer_id_, gpio_num_, pcnt_unit_id_);

    // Configure GPIO for bidirectional operation (MCPWM output + PCNT input)
    gpio_config_t gpio_conf = {};
    gpio_conf.pin_bit_mask = (1ULL << gpio_num_);
    gpio_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;

    esp_err_t ret = gpio_config(&gpio_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", gpio_num_, esp_err_to_name(ret));
        return ret;
    }

    // =========================================================================
    // MCPWM Timer Configuration
    // =========================================================================
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = MCPWM_GROUP_ID;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = MCPWM_RESOLUTION_HZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = MCPWM_RESOLUTION_HZ / DEFAULT_MAX_PULSE_FREQ_HZ;  // Initial period

    ret = mcpwm_new_timer(&timer_config, &timer_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create MCPWM timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // =========================================================================
    // MCPWM Operator Configuration
    // =========================================================================
    mcpwm_operator_config_t oper_config = {};
    oper_config.group_id = MCPWM_GROUP_ID;

    ret = mcpwm_new_operator(&oper_config, &oper_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create MCPWM operator: %s", esp_err_to_name(ret));
        mcpwm_del_timer(timer_handle_);
        timer_handle_ = nullptr;
        return ret;
    }

    ret = mcpwm_operator_connect_timer(oper_handle_, timer_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect operator to timer: %s", esp_err_to_name(ret));
        mcpwm_del_operator(oper_handle_);
        mcpwm_del_timer(timer_handle_);
        oper_handle_ = nullptr;
        timer_handle_ = nullptr;
        return ret;
    }

    // =========================================================================
    // MCPWM Comparator Configuration
    // =========================================================================
    mcpwm_comparator_config_t cmpr_config = {};
    cmpr_config.flags.update_cmp_on_tez = true;  // Update on timer zero

    ret = mcpwm_new_comparator(oper_handle_, &cmpr_config, &cmpr_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create MCPWM comparator: %s", esp_err_to_name(ret));
        mcpwm_del_operator(oper_handle_);
        mcpwm_del_timer(timer_handle_);
        oper_handle_ = nullptr;
        timer_handle_ = nullptr;
        return ret;
    }

    // Set initial compare value for 50% duty cycle
    uint32_t initial_period = MCPWM_RESOLUTION_HZ / DEFAULT_MAX_PULSE_FREQ_HZ;
    uint32_t compare_value = (initial_period * MCPWM_DUTY_CYCLE_PERCENT) / 100;
    mcpwm_comparator_set_compare_value(cmpr_handle_, compare_value);

    // =========================================================================
    // MCPWM Generator Configuration
    // =========================================================================
    mcpwm_generator_config_t gen_config = {};
    gen_config.gen_gpio_num = gpio_num_;
    gen_config.flags.io_loop_back = true;  // Enable internal loopback to PCNT

    ret = mcpwm_new_generator(oper_handle_, &gen_config, &gen_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create MCPWM generator: %s", esp_err_to_name(ret));
        mcpwm_del_comparator(cmpr_handle_);
        mcpwm_del_operator(oper_handle_);
        mcpwm_del_timer(timer_handle_);
        cmpr_handle_ = nullptr;
        oper_handle_ = nullptr;
        timer_handle_ = nullptr;
        return ret;
    }

    // Set generator actions: HIGH on timer zero, LOW on compare match
    ret = mcpwm_generator_set_action_on_timer_event(gen_handle_,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set timer action: %s", esp_err_to_name(ret));
    }

    ret = mcpwm_generator_set_action_on_compare_event(gen_handle_,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_handle_, MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set compare action: %s", esp_err_to_name(ret));
    }

    // =========================================================================
    // PCNT Unit Configuration
    // =========================================================================
    pcnt_unit_config_t pcnt_unit_config = {};
    pcnt_unit_config.high_limit = PCNT_HIGH_LIMIT;
    pcnt_unit_config.low_limit = PCNT_LOW_LIMIT;
    pcnt_unit_config.flags.accum_count = true;  // Enable accumulation for overflow tracking

    ret = pcnt_new_unit(&pcnt_unit_config, &pcnt_unit_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT unit: %s", esp_err_to_name(ret));
        mcpwm_del_generator(gen_handle_);
        mcpwm_del_comparator(cmpr_handle_);
        mcpwm_del_operator(oper_handle_);
        mcpwm_del_timer(timer_handle_);
        gen_handle_ = nullptr;
        cmpr_handle_ = nullptr;
        oper_handle_ = nullptr;
        timer_handle_ = nullptr;
        return ret;
    }

    // =========================================================================
    // PCNT Channel Configuration with io_loop_back
    // =========================================================================
    pcnt_chan_config_t pcnt_chan_config = {};
    pcnt_chan_config.edge_gpio_num = gpio_num_;  // Same GPIO as MCPWM output
    pcnt_chan_config.level_gpio_num = -1;        // No level signal
    pcnt_chan_config.flags.io_loop_back = true;  // Enable internal loopback

    ret = pcnt_new_channel(pcnt_unit_, &pcnt_chan_config, &pcnt_channel_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel: %s", esp_err_to_name(ret));
        pcnt_del_unit(pcnt_unit_);
        mcpwm_del_generator(gen_handle_);
        mcpwm_del_comparator(cmpr_handle_);
        mcpwm_del_operator(oper_handle_);
        mcpwm_del_timer(timer_handle_);
        pcnt_unit_ = nullptr;
        gen_handle_ = nullptr;
        cmpr_handle_ = nullptr;
        oper_handle_ = nullptr;
        timer_handle_ = nullptr;
        return ret;
    }

    // Configure channel to increment on rising edge, hold on falling edge
    ret = pcnt_channel_set_edge_action(pcnt_channel_,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // Rising edge: increment
        PCNT_CHANNEL_EDGE_ACTION_HOLD);      // Falling edge: hold
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PCNT edge action: %s", esp_err_to_name(ret));
    }

    // Configure level action (no effect since level_gpio_num = -1)
    ret = pcnt_channel_set_level_action(pcnt_channel_,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      // High level: keep
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);     // Low level: keep
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PCNT level action: %s", esp_err_to_name(ret));
    }

    // Register PCNT watch point callback
    pcnt_event_callbacks_t pcnt_cbs = {};
    pcnt_cbs.on_reach = onPcntReach;
    ret = pcnt_unit_register_event_callbacks(pcnt_unit_, &pcnt_cbs, this);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register PCNT callbacks: %s", esp_err_to_name(ret));
    }

    // Enable PCNT unit
    ret = pcnt_unit_enable(pcnt_unit_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable PCNT unit: %s", esp_err_to_name(ret));
    }

    // Enable MCPWM timer
    ret = mcpwm_timer_enable(timer_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable MCPWM timer: %s", esp_err_to_name(ret));
        pcnt_del_channel(pcnt_channel_);
        pcnt_del_unit(pcnt_unit_);
        mcpwm_del_generator(gen_handle_);
        mcpwm_del_comparator(cmpr_handle_);
        mcpwm_del_operator(oper_handle_);
        mcpwm_del_timer(timer_handle_);
        pcnt_channel_ = nullptr;
        pcnt_unit_ = nullptr;
        gen_handle_ = nullptr;
        cmpr_handle_ = nullptr;
        oper_handle_ = nullptr;
        timer_handle_ = nullptr;
        return ret;
    }

    // =========================================================================
    // Create Profile Update Task
    // =========================================================================
    char task_name[20];
    snprintf(task_name, sizeof(task_name), "mcpwm_prof_%d", timer_id_);
    BaseType_t xret = xTaskCreatePinnedToCore(
        profileTaskEntry,
        task_name,
        PROFILE_TASK_STACK,
        this,
        PROFILE_TASK_PRIORITY,
        &profile_task_handle_,
        1  // Pin to core 1 (motion core)
    );
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create profile task");
        mcpwm_timer_disable(timer_handle_);
        pcnt_del_channel(pcnt_channel_);
        pcnt_del_unit(pcnt_unit_);
        mcpwm_del_generator(gen_handle_);
        mcpwm_del_comparator(cmpr_handle_);
        mcpwm_del_operator(oper_handle_);
        mcpwm_del_timer(timer_handle_);
        pcnt_channel_ = nullptr;
        pcnt_unit_ = nullptr;
        gen_handle_ = nullptr;
        cmpr_handle_ = nullptr;
        oper_handle_ = nullptr;
        timer_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "MCPWM timer %d with PCNT unit %d initialized successfully",
             timer_id_, pcnt_unit_id_);
    return ESP_OK;
}

// ============================================================================
// Profile Update Task
// ============================================================================

void McpwmPulseGenerator::profileTaskEntry(void* arg)
{
    McpwmPulseGenerator* self = static_cast<McpwmPulseGenerator*>(arg);
    self->profileTaskLoop();
}

void McpwmPulseGenerator::profileTaskLoop()
{
    ESP_LOGI(TAG, "Profile task started for timer %d", timer_id_);

    while (!task_should_exit_.load(std::memory_order_acquire)) {
        uint32_t notification = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &notification, pdMS_TO_TICKS(PROFILE_UPDATE_INTERVAL_MS)) == pdTRUE) {
            if (notification & NOTIFY_EXIT_BIT) {
                break;
            }
            if (notification & NOTIFY_COMPLETE_BIT) {
                notifyCompletion();
            }
        }

        // Update profile if running
        McpwmProfileState current = state_.load(std::memory_order_acquire);
        if (current != McpwmProfileState::IDLE) {
            updateProfile();
        }
    }

    ESP_LOGI(TAG, "Profile task exiting for timer %d", timer_id_);
    vTaskDelete(nullptr);
}

void McpwmPulseGenerator::updateProfile()
{
    // Read current pulse count from PCNT
    int64_t current_count = readPcntCount();
    profile_.current_pulse = current_count;
    pulse_count_.store(current_count, std::memory_order_relaxed);

    McpwmProfileState current_state = state_.load(std::memory_order_acquire);

    // Check for completion in position mode
    if (mode_ == McpwmMotionMode::POSITION && current_count >= profile_.target_pulses) {
        // Motion complete - handled by PCNT callback
        return;
    }

    // Calculate velocity at current position
    float velocity = velocityAtPosition(current_count);

    // Clamp velocity
    if (velocity < LIMIT_MIN_PULSE_FREQ_HZ) {
        velocity = LIMIT_MIN_PULSE_FREQ_HZ;
    }
    if (velocity > profile_.cruise_velocity) {
        velocity = profile_.cruise_velocity;
    }

    // Update state machine
    if (current_state == McpwmProfileState::ACCELERATING &&
        velocity >= profile_.cruise_velocity - 1.0f) {
        if (profile_.cruise_pulses > 0 || mode_ == McpwmMotionMode::VELOCITY) {
            state_.store(McpwmProfileState::CRUISING, std::memory_order_release);
        } else {
            state_.store(McpwmProfileState::DECELERATING, std::memory_order_release);
        }
    } else if (current_state == McpwmProfileState::CRUISING &&
               mode_ == McpwmMotionMode::POSITION &&
               current_count >= profile_.accel_pulses + profile_.cruise_pulses) {
        state_.store(McpwmProfileState::DECELERATING, std::memory_order_release);
    }

    // Update MCPWM frequency
    setFrequency(velocity);
    current_velocity_.store(velocity, std::memory_order_relaxed);
    profile_.current_velocity = velocity;
}

// ============================================================================
// Motion Control
// ============================================================================

esp_err_t McpwmPulseGenerator::startMove(int32_t pulses, float max_velocity, float acceleration)
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

    // Set direction via shift register if direction changed
    if (direction_ != last_direction_) {
        // Determine axis from timer_id
        uint8_t axis = (timer_id_ == MCPWM_TIMER_Y) ? SR_AXIS_Y : SR_AXIS_C;
        sr_set_direction(axis, direction_);
        sr_update();

        // Wait for direction setup time
        ets_delay_us(TIMING_DIR_SETUP_US);
        last_direction_ = direction_;
    }

    // Calculate trapezoidal profile
    mode_ = McpwmMotionMode::POSITION;
    calculateTrapezoidalProfile(abs_pulses, max_velocity, acceleration);

    // Reset counters
    pulse_count_.store(0, std::memory_order_relaxed);
    overflow_count_.store(0, std::memory_order_relaxed);
    current_velocity_.store(0.0f, std::memory_order_relaxed);
    profile_.current_pulse = 0;
    profile_.current_velocity = LIMIT_MIN_PULSE_FREQ_HZ;

    // Clear and restart PCNT
    pcnt_unit_stop(pcnt_unit_);
    pcnt_unit_clear_count(pcnt_unit_);

    // Configure PCNT watch point for target pulse count
    esp_err_t ret = configurePcntWatchPoint(abs_pulses);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PCNT watch point: %s", esp_err_to_name(ret));
        return ret;
    }

    pcnt_unit_start(pcnt_unit_);

    // Set initial frequency (low for ramp-up)
    setFrequency(LIMIT_MIN_PULSE_FREQ_HZ);

    // Start MCPWM timer
    state_.store(McpwmProfileState::ACCELERATING, std::memory_order_release);
    ret = mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
        state_.store(McpwmProfileState::IDLE, std::memory_order_release);
        ESP_LOGE(TAG, "Failed to start MCPWM timer: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t McpwmPulseGenerator::startVelocity(float velocity, float acceleration)
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

    // Set direction via shift register if direction changed
    if (direction_ != last_direction_) {
        uint8_t axis = (timer_id_ == MCPWM_TIMER_Y) ? SR_AXIS_Y : SR_AXIS_C;
        sr_set_direction(axis, direction_);
        sr_update();
        ets_delay_us(TIMING_DIR_SETUP_US);
        last_direction_ = direction_;
    }

    // Set up velocity mode profile
    mode_ = McpwmMotionMode::VELOCITY;
    calculateVelocityProfile(abs_velocity, acceleration);

    // Reset counters
    pulse_count_.store(0, std::memory_order_relaxed);
    overflow_count_.store(0, std::memory_order_relaxed);
    current_velocity_.store(0.0f, std::memory_order_relaxed);
    profile_.current_pulse = 0;
    profile_.current_velocity = LIMIT_MIN_PULSE_FREQ_HZ;

    // Clear and restart PCNT (no watch point for velocity mode)
    pcnt_unit_stop(pcnt_unit_);
    pcnt_unit_clear_count(pcnt_unit_);

    // Remove any existing watch points
    pcnt_unit_remove_watch_point(pcnt_unit_, PCNT_HIGH_LIMIT);

    pcnt_unit_start(pcnt_unit_);

    // Set initial frequency
    setFrequency(LIMIT_MIN_PULSE_FREQ_HZ);

    // Start MCPWM timer
    state_.store(McpwmProfileState::ACCELERATING, std::memory_order_release);
    esp_err_t ret = mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
        state_.store(McpwmProfileState::IDLE, std::memory_order_release);
        ESP_LOGE(TAG, "Failed to start MCPWM timer: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t McpwmPulseGenerator::stop(float deceleration)
{
    McpwmProfileState current = state_.load(std::memory_order_acquire);
    if (current == McpwmProfileState::IDLE) {
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
    int64_t current_pos = pulse_count_.load(std::memory_order_relaxed);
    profile_.target_pulses = static_cast<int32_t>(current_pos + stop_pulses);
    profile_.decel_pulses = static_cast<int32_t>(stop_pulses);

    // Configure PCNT watch point for stop position
    configurePcntWatchPoint(profile_.target_pulses);

    state_.store(McpwmProfileState::STOPPING, std::memory_order_release);

    return ESP_OK;
}

void McpwmPulseGenerator::stopImmediate()
{
    McpwmProfileState current = state_.load(std::memory_order_acquire);
    if (current == McpwmProfileState::IDLE) {
        return;
    }

    ESP_LOGD(TAG, "stopImmediate");

    // Stop MCPWM timer immediately
    if (timer_handle_) {
        mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_STOP_FULL);
    }

    // Read final count
    int64_t final_count = readPcntCount();
    pulse_count_.store(final_count, std::memory_order_relaxed);

    // Stop PCNT
    if (pcnt_unit_) {
        pcnt_unit_stop(pcnt_unit_);
    }

    // Clear state
    state_.store(McpwmProfileState::IDLE, std::memory_order_release);
    current_velocity_.store(0.0f, std::memory_order_relaxed);

    // Note: Do NOT fire completion callback on immediate stop
}

// ============================================================================
// Status Methods
// ============================================================================

bool McpwmPulseGenerator::isRunning() const
{
    return state_.load(std::memory_order_acquire) != McpwmProfileState::IDLE;
}

int64_t McpwmPulseGenerator::getPulseCount() const
{
    // Read current PCNT count for live value
    if (state_.load(std::memory_order_acquire) != McpwmProfileState::IDLE) {
        return readPcntCount();
    }
    return pulse_count_.load(std::memory_order_relaxed);
}

float McpwmPulseGenerator::getCurrentVelocity() const
{
    return current_velocity_.load(std::memory_order_relaxed);
}

void McpwmPulseGenerator::setCompletionCallback(MotionCompleteCallback cb)
{
    if (xSemaphoreTake(callback_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        completion_callback_ = cb;
        xSemaphoreGive(callback_mutex_);
    }
}

void McpwmPulseGenerator::setPositionTracker(IPositionTracker* tracker)
{
    // Store but don't use - MCPWM axes (Y, C) use PcntTracker which
    // provides real-time position directly via hardware PCNT.
    // This stub exists for interface compliance.
    position_tracker_ = tracker;
    (void)position_tracker_;  // Suppress unused warning
}

// ============================================================================
// Profile Calculation
// ============================================================================

void McpwmPulseGenerator::calculateTrapezoidalProfile(int32_t pulses, float max_vel, float accel)
{
    profile_.target_pulses = pulses;
    profile_.max_velocity = max_vel;
    profile_.acceleration = accel;
    profile_.deceleration = accel;  // Symmetric by default

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

void McpwmPulseGenerator::calculateVelocityProfile(float velocity, float acceleration)
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

float McpwmPulseGenerator::velocityAtPosition(int64_t position) const
{
    if (mode_ == McpwmMotionMode::VELOCITY) {
        McpwmProfileState current = state_.load(std::memory_order_acquire);
        if (current == McpwmProfileState::ACCELERATING) {
            // v = sqrt(2 * a * d)
            float v = std::sqrt(2.0f * profile_.acceleration * static_cast<float>(position));
            return std::min(v, profile_.cruise_velocity);
        } else if (current == McpwmProfileState::STOPPING) {
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

// ============================================================================
// MCPWM Frequency Control
// ============================================================================

esp_err_t McpwmPulseGenerator::setFrequency(float frequency)
{
    if (frequency < LIMIT_MIN_PULSE_FREQ_HZ) {
        frequency = LIMIT_MIN_PULSE_FREQ_HZ;
    }
    if (frequency > LIMIT_MAX_PULSE_FREQ_HZ) {
        frequency = LIMIT_MAX_PULSE_FREQ_HZ;
    }

    uint32_t period_ticks = velocityToPeriodTicks(frequency);

    // Update timer period
    esp_err_t ret = mcpwm_timer_set_period(timer_handle_, period_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    // Update comparator for 50% duty cycle
    uint32_t compare_value = (period_ticks * MCPWM_DUTY_CYCLE_PERCENT) / 100;
    return mcpwm_comparator_set_compare_value(cmpr_handle_, compare_value);
}

uint32_t McpwmPulseGenerator::velocityToPeriodTicks(float velocity) const
{
    if (velocity < LIMIT_MIN_PULSE_FREQ_HZ) {
        velocity = LIMIT_MIN_PULSE_FREQ_HZ;
    }
    // Period in ticks = resolution / frequency
    return static_cast<uint32_t>(MCPWM_RESOLUTION_HZ / velocity);
}

// ============================================================================
// PCNT Configuration
// ============================================================================

esp_err_t McpwmPulseGenerator::configurePcntWatchPoint(int32_t target_pulses)
{
    // Remove existing watch points
    pcnt_unit_remove_watch_point(pcnt_unit_, PCNT_HIGH_LIMIT);

    // For targets <= PCNT_HIGH_LIMIT, set watch point directly
    if (target_pulses <= PCNT_HIGH_LIMIT) {
        return pcnt_unit_add_watch_point(pcnt_unit_, static_cast<int>(target_pulses));
    }

    // For larger targets, use overflow tracking + final watch point
    // Set watch point at max limit; callback will handle overflow and final count
    return pcnt_unit_add_watch_point(pcnt_unit_, PCNT_HIGH_LIMIT);
}

int64_t McpwmPulseGenerator::readPcntCount() const
{
    int count = 0;
    if (pcnt_unit_) {
        pcnt_unit_get_count(pcnt_unit_, &count);
    }

    // Add overflow contribution
    int32_t overflows = overflow_count_.load(std::memory_order_relaxed);
    return static_cast<int64_t>(count) + (static_cast<int64_t>(overflows) * PCNT_HIGH_LIMIT);
}

// ============================================================================
// PCNT Callbacks - ISR SAFE
// ============================================================================

bool IRAM_ATTR McpwmPulseGenerator::onPcntReach(pcnt_unit_handle_t unit,
                                                  const pcnt_watch_event_data_t* event_data,
                                                  void* user_data)
{
    McpwmPulseGenerator* self = static_cast<McpwmPulseGenerator*>(user_data);
    return self->handlePcntReachISR(event_data);
}

bool McpwmPulseGenerator::handlePcntReachISR(const pcnt_watch_event_data_t* event_data)
{
    int watch_point = event_data->watch_point_value;
    int32_t target = profile_.target_pulses;

    // Check if this is an overflow event (watch point at PCNT_HIGH_LIMIT)
    if (watch_point == PCNT_HIGH_LIMIT && target > PCNT_HIGH_LIMIT) {
        // Increment overflow counter
        overflow_count_.fetch_add(1, std::memory_order_relaxed);

        // Clear PCNT count and continue
        pcnt_unit_clear_count(pcnt_unit_);

        // Check if we've reached the target with overflows
        int64_t total_count = readPcntCount();
        if (total_count >= target) {
            // Stop MCPWM
            mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_STOP_FULL);

            // Update final count
            pulse_count_.store(total_count, std::memory_order_relaxed);
            state_.store(McpwmProfileState::IDLE, std::memory_order_release);
            current_velocity_.store(0.0f, std::memory_order_relaxed);

            // Signal completion to task
            if (profile_task_handle_) {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xTaskNotifyFromISR(profile_task_handle_, NOTIFY_COMPLETE_BIT,
                                   eSetBits, &xHigherPriorityTaskWoken);
                return xHigherPriorityTaskWoken == pdTRUE;
            }
        }
        return false;
    }

    // Target reached - stop MCPWM
    mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_STOP_FULL);

    // Read final count
    int64_t final_count = readPcntCount();
    pulse_count_.store(final_count, std::memory_order_relaxed);

    // Stop PCNT
    pcnt_unit_stop(pcnt_unit_);

    // Update state
    state_.store(McpwmProfileState::IDLE, std::memory_order_release);
    current_velocity_.store(0.0f, std::memory_order_relaxed);

    // Signal completion to task
    if (profile_task_handle_) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(profile_task_handle_, NOTIFY_COMPLETE_BIT,
                           eSetBits, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }

    return false;
}

void McpwmPulseGenerator::notifyCompletion()
{
    if (xSemaphoreTake(callback_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (completion_callback_) {
            int64_t total = pulse_count_.load(std::memory_order_relaxed);
            completion_callback_(total);
        }
        xSemaphoreGive(callback_mutex_);
    }
}
