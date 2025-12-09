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
#include "soc/io_mux_reg.h"  // For PIN_INPUT_ENABLE
#include "soc/gpio_periph.h" // For GPIO_PIN_MUX_REG
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

// Profile update interval uses global timing constant for all axes (10ms)
// This controls how often frequency is adjusted AND position is updated

// Maximum frequency change per millisecond for smooth ramping
// 1000 Hz/ms means 0→600 Hz takes 0.6ms (very smooth but responsive)
static constexpr float MAX_FREQ_CHANGE_PER_MS = 1000.0f;

// Minimum starting frequency for responsive motion (Hz)
// Higher than LIMIT_MIN_PULSE_FREQ_HZ for better motor startup
static constexpr float MIN_MOTION_START_FREQ = 300.0f;

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
    , absolute_position_(0)
    , overflow_count_(0)
    , current_velocity_(0.0f)
    , last_watch_point_(0)
    , move_start_position_(0)
    , move_start_pcnt_count_(0)
    , last_completed_position_(0)
    , pcnt_at_last_completion_(0)
    , profile_timer_(nullptr)
    , completion_callback_(nullptr)
    , callback_mutex_(nullptr)
    , completion_notified_(false)
    , motion_start_time_(0)
    , position_tracker_(nullptr)
{
    callback_mutex_ = xSemaphoreCreateMutex();
}

McpwmPulseGenerator::~McpwmPulseGenerator()
{
    // Stop and delete profile timer
    if (profile_timer_) {
        esp_timer_stop(profile_timer_);  // Stop if running
        esp_timer_delete(profile_timer_);
        profile_timer_ = nullptr;
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
    // Initial period: use 1000 Hz as safe starting frequency
    // With 100 kHz resolution: period = 100000 / 1000 = 100 ticks
    timer_config.period_ticks = MCPWM_RESOLUTION_HZ / 1000;

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

    ESP_LOGW(TAG, "DEBUG init: MCPWM generator created on GPIO %d with io_loop_back=true", gpio_num_);

    // Manually enable input on the GPIO pin to ensure PCNT can read it
    // This fixes the issue where MCPWM configures the pin as output-only
    // Reference: https://github.com/ataboo/esp-mcpwm-pcnt-combined
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[gpio_num_]);
    ESP_LOGI(TAG, "GPIO %d input enabled for PCNT loopback, settling for %d µs",
             gpio_num_, TIMING_GPIO_LOOPBACK_SETTLE_US);

    // CRITICAL: Wait for GPIO input buffer to stabilize for loopback connection.
    // Without this delay, PCNT cannot detect MCPWM pulses on first move (650ms stall).
    // By settling during init(), first startMove() has zero latency.
    ets_delay_us(TIMING_GPIO_LOOPBACK_SETTLE_US);
    ESP_LOGI(TAG, "GPIO %d loopback settled and ready", gpio_num_);

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
    // NOTE: Do NOT use accum_count=true - it prevents pcnt_unit_clear_count() from
    // actually resetting the counter, causing stale counts to persist between moves.
    // We handle overflow tracking manually with overflow_count_.
    pcnt_unit_config.flags.accum_count = false;

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
    // Note: io_loop_back is deprecated in ESP-IDF v5.x (removed in v6.0)
    // The gpio_config() call above with GPIO_MODE_INPUT_OUTPUT already enables
    // bidirectional operation. PCNT driver will append input config automatically.

    ESP_LOGW(TAG, "DEBUG init: Creating PCNT channel on GPIO %d", gpio_num_);
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

    // DISABLED: Watch point ISR was causing stale watch point bugs that led to
    // premature motion stopping and position corruption (180° instead of 360°).
    // Task-based polling in updateProfile() is now the sole stop mechanism.
    // pcnt_event_callbacks_t pcnt_cbs = {};
    // pcnt_cbs.on_reach = onPcntReach;
    // ret = pcnt_unit_register_event_callbacks(pcnt_unit_, &pcnt_cbs, this);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to register PCNT callbacks: %s", esp_err_to_name(ret));
    // }

    // Enable PCNT unit
    ret = pcnt_unit_enable(pcnt_unit_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable PCNT unit: %s", esp_err_to_name(ret));
    }

    // Start PCNT counting immediately - runs continuously for position tracking
    ret = pcnt_unit_start(pcnt_unit_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start PCNT unit: %s", esp_err_to_name(ret));
    }

    // Initialize PCNT baseline for continuous counting
    // Read current PCNT to establish baseline (may have spurious edges from startup)
    int init_pcnt_count = 0;
    pcnt_unit_get_count(pcnt_unit_, &init_pcnt_count);
    move_start_pcnt_count_ = init_pcnt_count;
    pcnt_at_last_completion_ = init_pcnt_count;  // Initialize stable reference
    last_completed_position_ = 0;
    ESP_LOGI(TAG, "PCNT started for continuous position tracking (baseline: %d)", init_pcnt_count);

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
    // Create Profile Update Timer (esp_timer for guaranteed timing)
    // =========================================================================
    // Create periodic timer for profile updates (20ms default)
    // Timer starts/stops with motion - created here but not started yet
    char timer_name[20];
    snprintf(timer_name, sizeof(timer_name), "mcpwm_prof_%d", timer_id_);

    const esp_timer_create_args_t timer_args = {
        .callback = profileTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,  // Dispatch from esp_timer task (allows longer callbacks)
        .name = timer_name,
        .skip_unhandled_events = false  // Don't skip events if callback takes longer than period
    };

    ret = esp_timer_create(&timer_args, &profile_timer_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create profile timer: %s", esp_err_to_name(ret));
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
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "MCPWM timer %d with PCNT unit %d initialized successfully",
             timer_id_, pcnt_unit_id_);
    return ESP_OK;
}

// ============================================================================
// Profile Update Timer Callback (esp_timer for guaranteed timing)
// ============================================================================

void McpwmPulseGenerator::profileTimerCallback(void* arg)
{
    McpwmPulseGenerator* self = static_cast<McpwmPulseGenerator*>(arg);

    // CRITICAL: Always read PCNT and update position (even when IDLE)
    // This ensures:
    // 1. Position queries work correctly when IDLE
    // 2. Guaranteed 20ms update interval (no FreeRTOS scheduler delays!)
    // 3. Position-based acceleration gets accurate position
    //
    // Update position continuously using STABLE reference (NO RACE)
    // Uses last_completed_position_ and pcnt_at_last_completion_ which are
    // ONLY updated in completion handlers - no mid-move changes!
    int count = 0;
    if (self->pcnt_unit_) {
        pcnt_unit_get_count(self->pcnt_unit_, &count);
    }
    int32_t overflows = self->overflow_count_.load(std::memory_order_relaxed);
    int32_t hw_pcnt = count + (overflows * PCNT_HIGH_LIMIT);

    // Calculate position delta from last completion (STABLE reference point)
    int32_t pcnt_delta = hw_pcnt - self->pcnt_at_last_completion_;
    bool current_dir = self->direction_;  // Read current direction (not atomic, but OK here)
    int64_t position_delta = current_dir ? pcnt_delta : -pcnt_delta;

    // Update absolute position (SINGLE SOURCE OF TRUTH)
    int64_t new_position = self->last_completed_position_ + position_delta;
    self->absolute_position_.store(new_position, std::memory_order_relaxed);

    // Also update pulse_count_ for profile tracking (relative to move start)
    // Use SAME hw_pcnt reading to avoid race condition (no second PCNT read!)
    int64_t current_count = hw_pcnt - self->move_start_pcnt_count_;
    self->pulse_count_.store(current_count, std::memory_order_relaxed);

    // Only update motion profile when running (frequency changes, state transitions)
    McpwmProfileState current = self->state_.load(std::memory_order_acquire);
    if (current != McpwmProfileState::IDLE) {
        self->updateProfile();
    }
}

void McpwmPulseGenerator::updateProfile()
{
    // =========================================================================
    // PCNT-BASED PROFILE TRACKING (Hardware Pulse Counting)
    // =========================================================================
    // Read actual pulse count from PCNT hardware (not ISR-based!)
    // PCNT is reliable for counting - the unreliable part was the watch point ISR.
    // By polling PCNT in task context (here), we get accurate count + reliable stopping.
    // =========================================================================

    McpwmProfileState current_state = state_.load(std::memory_order_acquire);

    // Reset timing on motion start
    if (current_state == McpwmProfileState::ACCELERATING && motion_start_time_ == 0) {
        motion_start_time_ = esp_timer_get_time();
        completion_notified_.store(false, std::memory_order_relaxed);
    }

    // Note: Position is updated continuously in profileTaskLoop() every 20ms
    // This function only handles motion profile updates (frequency changes, state transitions)

    // Get current pulse count (already updated by task loop)
    int64_t current_count = pulse_count_.load(std::memory_order_relaxed);
    profile_.current_pulse = current_count;

    // DEBUG: Log every 50 updates
    static int debug_counter = 0;
    if (++debug_counter % 50 == 0) {
        ESP_LOGW(TAG, "DEBUG updateProfile: pcnt=%lld, target=%ld, freq=%.1f, state=%d",
                 (long long)current_count, (long)profile_.target_pulses,
                 current_velocity_.load(std::memory_order_relaxed),
                 static_cast<int>(state_.load()));
    }

    // =========================================================================
    // COMPLETION CHECK - PCNT POLLED IN TASK (RELIABLE)
    // This is the PRIMARY stopping mechanism - no ISR dependency!
    // =========================================================================
    if (mode_ == McpwmMotionMode::POSITION && current_count >= profile_.target_pulses) {
        ESP_LOGW(TAG, "updateProfile: STOP - pcnt=%lld >= target=%ld",
                 (long long)current_count, (long)profile_.target_pulses);

        // Stop MCPWM timer immediately (STOP_EMPTY prevents leftover GPIO HIGH state)
        mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_STOP_EMPTY);

        // Reconfigure generator action to force GPIO LOW when timer is at zero
        // This overrides the HIGH action used for pulse generation
        esp_err_t ret = mcpwm_generator_set_action_on_timer_event(gen_handle_,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set generator LOW action: %s", esp_err_to_name(ret));
        }
        ESP_LOGW(TAG, "DEBUG: Reconfigured generator to force GPIO LOW when stopped");

        // NOTE: Keep PCNT running continuously for position tracking (don't stop it)

        // Calculate new absolute position based on ACTUAL PCNT count (not target!)
        // PCNT is the sole source of truth for position.
        // current_count is the actual hardware PCNT value read at line 460.
        int64_t actual_pulses = current_count;
        int64_t delta = direction_ ? actual_pulses : -actual_pulses;
        int64_t new_abs_pos = move_start_position_ + delta;
        absolute_position_.store(new_abs_pos, std::memory_order_relaxed);
        pulse_count_.store(actual_pulses, std::memory_order_relaxed);

        ESP_LOGW(TAG, "updateProfile: COMPLETED - actual_pcnt=%lld, start=%lld, new_abs=%lld",
                 (long long)actual_pulses, (long long)move_start_position_, (long long)new_abs_pos);

        // Update stable reference for background position calculation (NO RACE)
        last_completed_position_ = new_abs_pos;
        int count_hw = 0;
        pcnt_unit_get_count(pcnt_unit_, &count_hw);
        int32_t overflows_hw = overflow_count_.load(std::memory_order_relaxed);
        pcnt_at_last_completion_ = count_hw + (overflows_hw * PCNT_HIGH_LIMIT);
        ESP_LOGW(TAG, "DEBUG: Updated stable reference - last_pos=%lld, pcnt_hw=%ld",
                 (long long)last_completed_position_, (long)pcnt_at_last_completion_);

        // Read PCNT again after stop to verify it's stable (planning phase starts now)
        int count_after_stop = 0;
        pcnt_unit_get_count(pcnt_unit_, &count_after_stop);
        ESP_LOGW(TAG, "DEBUG PLANNING: PCNT after stop = %d (should equal %lld)",
                 count_after_stop, (long long)actual_pulses);

        // Stop profile update timer
        esp_timer_stop(profile_timer_);

        state_.store(McpwmProfileState::IDLE, std::memory_order_release);
        current_velocity_.store(0.0f, std::memory_order_relaxed);

        // Reset timing for next motion
        motion_start_time_ = 0;

        // Fire completion callback with target count (notifyCompletion has double-call protection)
        notifyCompletion();
        return;
    }

    // =========================================================================
    // STOPPING STATE COMPLETION CHECK (controlled deceleration stop)
    // When stop() is called, it sets state to STOPPING and calculates target_pulses
    // for deceleration. We must check if we've reached that target and stop.
    // =========================================================================
    if (current_state == McpwmProfileState::STOPPING && current_count >= profile_.target_pulses) {
        ESP_LOGW(TAG, "updateProfile: STOPPING COMPLETE - pcnt=%lld >= target=%ld",
                 (long long)current_count, (long)profile_.target_pulses);

        // Stop MCPWM timer immediately
        mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_STOP_EMPTY);

        // Reconfigure generator action to force GPIO LOW when timer is at zero
        esp_err_t ret = mcpwm_generator_set_action_on_timer_event(gen_handle_,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set generator LOW action: %s", esp_err_to_name(ret));
        }
        ESP_LOGW(TAG, "DEBUG STOPPING: Reconfigured generator to force GPIO LOW");

        // NOTE: Keep PCNT running continuously for position tracking (don't stop it)

        // Calculate new absolute position based on ACTUAL PCNT count
        int64_t actual_pulses = current_count;
        int64_t delta = direction_ ? actual_pulses : -actual_pulses;
        int64_t new_abs_pos = move_start_position_ + delta;
        absolute_position_.store(new_abs_pos, std::memory_order_relaxed);
        pulse_count_.store(actual_pulses, std::memory_order_relaxed);

        ESP_LOGW(TAG, "updateProfile: STOP DONE - actual_pcnt=%lld, start=%lld, new_abs=%lld",
                 (long long)actual_pulses, (long long)move_start_position_, (long long)new_abs_pos);

        // Update stable reference for background position calculation (NO RACE)
        last_completed_position_ = new_abs_pos;
        int count_hw = 0;
        pcnt_unit_get_count(pcnt_unit_, &count_hw);
        int32_t overflows_hw = overflow_count_.load(std::memory_order_relaxed);
        pcnt_at_last_completion_ = count_hw + (overflows_hw * PCNT_HIGH_LIMIT);
        ESP_LOGW(TAG, "DEBUG: Updated stable reference (STOPPING) - last_pos=%lld, pcnt_hw=%ld",
                 (long long)last_completed_position_, (long)pcnt_at_last_completion_);

        // Stop profile update timer
        esp_timer_stop(profile_timer_);

        state_.store(McpwmProfileState::IDLE, std::memory_order_release);
        current_velocity_.store(0.0f, std::memory_order_relaxed);

        // Reset timing for next motion
        motion_start_time_ = 0;

        // Fire completion callback (stop() initiated the controlled stop)
        notifyCompletion();
        return;
    }

    // =========================================================================
    // TIME-BASED ACCELERATION (for responsive startup)
    // During initial acceleration, use time-based velocity to avoid the
    // chicken-and-egg problem where low frequency leads to slow PCNT advance.
    // =========================================================================
    float current_freq = current_velocity_.load(std::memory_order_relaxed);
    float velocity;

    if (current_state == McpwmProfileState::ACCELERATING) {
        // CRITICAL FIX: Position-based acceleration (FastAccelStepper method)
        // v² = v0² + 2×a×s - immune to task scheduling delays!
        // Velocity calculated from actual PCNT position, not elapsed time.

        // Use absolute distance traveled to avoid NaN from sqrt(negative)
        float distance_traveled = std::fabs(static_cast<float>(current_count));  // From PCNT hardware
        float v0_squared = MIN_MOTION_START_FREQ * MIN_MOTION_START_FREQ;
        float v_squared = v0_squared + (2.0f * profile_.acceleration * distance_traveled);

        // Ensure v_squared is non-negative before sqrt (safety check)
        if (v_squared < v0_squared) {
            v_squared = v0_squared;
        }

        float position_based_velocity = sqrtf(v_squared);

        // Clamp to cruise velocity
        float target_velocity = fmin(position_based_velocity, profile_.cruise_velocity);

        // Clamp to safe range
        if (target_velocity < MIN_MOTION_START_FREQ) {
            target_velocity = MIN_MOTION_START_FREQ;
        }

        // Apply smooth ramping (prevents abrupt frequency changes)
        if (target_velocity > current_freq) {
            velocity = fmin(target_velocity, current_freq + MAX_FREQ_CHANGE_PER_MS);
        } else {
            velocity = fmax(target_velocity, current_freq - MAX_FREQ_CHANGE_PER_MS);
        }

        // Check for state transitions
        // 1. If we've reached end of acceleration phase → cruise or decelerate
        if (current_count >= profile_.accel_pulses) {
            if (profile_.cruise_pulses > 0 || mode_ == McpwmMotionMode::VELOCITY) {
                // Has cruise phase (or velocity mode) → transition to CRUISING
                state_.store(McpwmProfileState::CRUISING, std::memory_order_release);
            } else {
                // No cruise phase (triangular profile) → start decelerating
                state_.store(McpwmProfileState::DECELERATING, std::memory_order_release);
            }
        }
        // 2. If velocity has reached cruise velocity → transition to CRUISING
        else if (velocity >= profile_.cruise_velocity - 1.0f) {
            if (profile_.cruise_pulses > 0 || mode_ == McpwmMotionMode::VELOCITY) {
                state_.store(McpwmProfileState::CRUISING, std::memory_order_release);
            } else {
                state_.store(McpwmProfileState::DECELERATING, std::memory_order_release);
            }
        }
    } else {
        // Cruising or decelerating: use position-based velocity
        float target_velocity = velocityAtPosition(current_count);

        // Clamp target velocity
        if (target_velocity < LIMIT_MIN_PULSE_FREQ_HZ) {
            target_velocity = LIMIT_MIN_PULSE_FREQ_HZ;
        }
        if (target_velocity > profile_.cruise_velocity) {
            target_velocity = profile_.cruise_velocity;
        }

        // Smooth ramping
        if (target_velocity > current_freq) {
            velocity = std::min(target_velocity, current_freq + MAX_FREQ_CHANGE_PER_MS);
        } else if (target_velocity < current_freq) {
            velocity = std::max(target_velocity, current_freq - MAX_FREQ_CHANGE_PER_MS);
        } else {
            velocity = target_velocity;
        }

        // State transitions
        if (current_state == McpwmProfileState::CRUISING &&
            mode_ == McpwmMotionMode::POSITION &&
            current_count >= profile_.accel_pulses + profile_.cruise_pulses) {
            state_.store(McpwmProfileState::DECELERATING, std::memory_order_release);
        }
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
        ESP_LOGE(TAG, "DEBUG startMove: NOT INITIALIZED!");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate parameters
    if (pulses == 0) {
        ESP_LOGW(TAG, "DEBUG startMove: pulses=0, nothing to do");
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

    // CRITICAL FIX: If already running, just update move target (smooth transition)
    // Don't re-initialize hardware - this was causing motor to stop!
    McpwmProfileState current_state = state_.load(std::memory_order_acquire);
    if (current_state != McpwmProfileState::IDLE) {
        ESP_LOGW(TAG, "startMove: Already running, updating target smoothly (no hardware re-init)");

        // Update direction if needed
        if (direction_ != last_direction_) {
            uint8_t axis = (timer_id_ == MCPWM_TIMER_Y) ? SR_AXIS_Y : SR_AXIS_C;
            sr_set_direction(axis, direction_);
            sr_update();
            ets_delay_us(TIMING_DIR_SETUP_US);
            last_direction_ = direction_;
        }

        // Update profile for new target
        mode_ = McpwmMotionMode::POSITION;
        calculateTrapezoidalProfile(abs_pulses, max_velocity, acceleration);

        // Reset motion start time for new acceleration calculation
        motion_start_time_ = 0;
        completion_notified_.store(false, std::memory_order_relaxed);

        // Transition to ACCELERATING for smooth ramp
        state_.store(McpwmProfileState::ACCELERATING, std::memory_order_release);

        // NO hardware re-initialization - motor keeps running!
        return ESP_OK;
    }

    // If IDLE, do full hardware initialization
    ESP_LOGW(TAG, "DEBUG startMove: pulses=%ld, max_vel=%.1f Hz, accel=%.1f, dir=%s",
             (long)pulses, max_velocity, acceleration, direction_ ? "FWD" : "REV");

    // DEBUG: Read PCNT at entry (planning phase just ended)
    int count_at_entry = 0;
    pcnt_unit_get_count(pcnt_unit_, &count_at_entry);
    ESP_LOGW(TAG, "DEBUG PLANNING: PCNT at startMove entry = %d (end of planning phase)",
             count_at_entry);

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
    ESP_LOGW(TAG, "DEBUG startMove: profile calculated: accel=%ld, cruise=%ld, decel=%ld, cruise_vel=%.1f",
             (long)profile_.accel_pulses, (long)profile_.cruise_pulses,
             (long)profile_.decel_pulses, profile_.cruise_velocity);

    // Save starting position for absolute position tracking
    move_start_position_ = absolute_position_.load(std::memory_order_relaxed);
    ESP_LOGW(TAG, "DEBUG startMove: move_start_position=%lld, direction=%s",
             (long long)move_start_position_, direction_ ? "FWD" : "REV");

    // Reset counters and flags for new motion
    pulse_count_.store(0, std::memory_order_relaxed);
    overflow_count_.store(0, std::memory_order_relaxed);
    current_velocity_.store(MIN_MOTION_START_FREQ, std::memory_order_relaxed);
    completion_notified_.store(false, std::memory_order_relaxed);
    motion_start_time_ = 0;  // Will be set in first updateProfile call
    profile_.current_pulse = 0;
    profile_.current_velocity = MIN_MOTION_START_FREQ;

    // CRITICAL: Keep PCNT running continuously for accurate position tracking
    // Read current PCNT count as baseline for this move (don't clear it!)
    int current_pcnt = 0;
    pcnt_unit_get_count(pcnt_unit_, &current_pcnt);
    move_start_pcnt_count_ = current_pcnt;
    ESP_LOGW(TAG, "DEBUG startMove: PCNT baseline = %ld (continuous counting, never cleared)", (long)move_start_pcnt_count_);

    // Calculate absolute watch point target: current PCNT + relative pulses
    int32_t watch_point_target = move_start_pcnt_count_ + abs_pulses;
    ESP_LOGW(TAG, "DEBUG startMove: watch point target = %ld + %ld = %ld (absolute PCNT value)",
             (long)move_start_pcnt_count_, (long)abs_pulses, (long)watch_point_target);

    // GPIO input is permanently enabled during init() - no need to re-enable here.
    // MCPWM timer start/stop does NOT affect GPIO input buffer state (verified ESP-IDF behavior).
    // Removing redundant PIN_INPUT_ENABLE() call to avoid unnecessary 650ms settling delays.

    // Configure PCNT watch point using absolute target value
    // Watch point ISR provides backup but task polling is primary stop mechanism
    ESP_LOGW(TAG, "DEBUG startMove: configuring PCNT watch point for absolute count %ld", (long)watch_point_target);
    esp_err_t ret = configurePcntWatchPoint(watch_point_target);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "PCNT watch point config failed: %s (task polling will handle stop)",
                 esp_err_to_name(ret));
        // Don't return error - task polling is the primary mechanism
    }

    // NOTE: PCNT will be started AFTER MCPWM to avoid counting leftover pulse edges
    // Generator actions are always active - no need for force level manipulation

    // Set initial frequency (responsive start)
    ESP_LOGW(TAG, "DEBUG startMove: setting initial freq=%.0f Hz", MIN_MOTION_START_FREQ);
    ret = setFrequency(MIN_MOTION_START_FREQ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DEBUG startMove: setFrequency FAILED: %s", esp_err_to_name(ret));
        return ret;
    }

    // Restore generator action to HIGH on timer empty (for pulse generation)
    // This must be done BEFORE starting the timer
    ret = mcpwm_generator_set_action_on_timer_event(gen_handle_,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore generator HIGH action: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGW(TAG, "DEBUG startMove: Restored generator HIGH action for pulse generation");

    // DEBUG: Check PCNT after restoring HIGH action (PCNT already running continuously)
    int count_after_high = 0;
    pcnt_unit_get_count(pcnt_unit_, &count_after_high);
    ESP_LOGW(TAG, "DEBUG PLANNING: PCNT after HIGH action = %d (continuous counting)", count_after_high);

    // NOTE: PCNT runs continuously (started in init()) - no need to start/stop it
    // This ensures accurate position tracking across multiple moves

    // Start MCPWM timer - PCNT is already counting
    state_.store(McpwmProfileState::ACCELERATING, std::memory_order_release);
    ESP_LOGW(TAG, "DEBUG startMove: starting MCPWM timer (PCNT continuously running)");
    ret = mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
        state_.store(McpwmProfileState::IDLE, std::memory_order_release);
        ESP_LOGE(TAG, "Failed to start MCPWM timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start profile update timer for smooth acceleration (esp_timer guarantees timing)
    ret = esp_timer_start_periodic(profile_timer_, TIMING_POSITION_UPDATE_LATENCY_MS * 1000);
    if (ret != ESP_OK) {
        mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_STOP_EMPTY);
        state_.store(McpwmProfileState::IDLE, std::memory_order_release);
        ESP_LOGE(TAG, "Failed to start profile timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // DEBUG: Check PCNT shortly after MCPWM start
    esp_rom_delay_us(100);  // 100µs delay to let a few edges occur
    int count_after_mcpwm = 0;
    pcnt_unit_get_count(pcnt_unit_, &count_after_mcpwm);
    ESP_LOGW(TAG, "DEBUG PLANNING: PCNT after MCPWM start = %d (should be ~30 at 300Hz)", count_after_mcpwm);

    ESP_LOGW(TAG, "DEBUG startMove: completed successfully");
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

    // CRITICAL FIX: If already running, just update velocity target (smooth transition)
    // Don't re-initialize hardware - this was causing motor to stop on every VEL command!
    McpwmProfileState current_state = state_.load(std::memory_order_acquire);
    if (current_state != McpwmProfileState::IDLE) {
        ESP_LOGW(TAG, "startVelocity: Already running, updating velocity smoothly (no hardware re-init)");

        // Update direction if needed
        direction_ = (velocity >= 0);
        if (direction_ != last_direction_) {
            uint8_t axis = (timer_id_ == MCPWM_TIMER_Y) ? SR_AXIS_Y : SR_AXIS_C;
            sr_set_direction(axis, direction_);
            sr_update();
            ets_delay_us(TIMING_DIR_SETUP_US);
            last_direction_ = direction_;
        }

        // Update profile parameters for smooth ramp to new velocity
        mode_ = McpwmMotionMode::VELOCITY;
        calculateVelocityProfile(abs_velocity, acceleration);

        // Reset motion start time to recalculate acceleration from current velocity
        motion_start_time_ = 0;
        completion_notified_.store(false, std::memory_order_relaxed);

        // Transition to ACCELERATING to ramp to new velocity
        state_.store(McpwmProfileState::ACCELERATING, std::memory_order_release);

        // NO hardware re-initialization - motor keeps running!
        // Profile task will smoothly ramp to new velocity
        return ESP_OK;
    }

    // If IDLE, do full hardware initialization
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

    // Save starting position for absolute position tracking
    move_start_position_ = absolute_position_.load(std::memory_order_relaxed);
    ESP_LOGW(TAG, "DEBUG startVelocity: move_start_position=%lld, direction=%s",
             (long long)move_start_position_, direction_ ? "FWD" : "REV");

    // Reset counters
    pulse_count_.store(0, std::memory_order_relaxed);
    overflow_count_.store(0, std::memory_order_relaxed);
    current_velocity_.store(0.0f, std::memory_order_relaxed);
    profile_.current_pulse = 0;
    profile_.current_velocity = LIMIT_MIN_PULSE_FREQ_HZ;

    // CRITICAL: Keep PCNT running continuously for accurate position tracking
    // Read current PCNT count as baseline for this move (don't clear it!)
    int current_pcnt = 0;
    pcnt_unit_get_count(pcnt_unit_, &current_pcnt);
    move_start_pcnt_count_ = current_pcnt;
    ESP_LOGW(TAG, "DEBUG startVelocity: PCNT baseline = %ld (continuous counting)", (long)move_start_pcnt_count_);

    // Remove any existing watch points (no target in velocity mode)
    pcnt_unit_remove_watch_point(pcnt_unit_, PCNT_HIGH_LIMIT);

    // Set initial frequency
    setFrequency(LIMIT_MIN_PULSE_FREQ_HZ);

    // Restore generator action to HIGH on timer empty (for pulse generation)
    esp_err_t ret = mcpwm_generator_set_action_on_timer_event(gen_handle_,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore generator HIGH action: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGW(TAG, "DEBUG startVelocity: Restored generator HIGH action");

    // NOTE: PCNT runs continuously (started in init()) - no need to start/stop it

    // Start MCPWM timer
    state_.store(McpwmProfileState::ACCELERATING, std::memory_order_release);
    ret = mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
        state_.store(McpwmProfileState::IDLE, std::memory_order_release);
        ESP_LOGE(TAG, "Failed to start MCPWM timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start profile update timer for smooth acceleration (esp_timer guarantees timing)
    ret = esp_timer_start_periodic(profile_timer_, TIMING_POSITION_UPDATE_LATENCY_MS * 1000);
    if (ret != ESP_OK) {
        mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_STOP_EMPTY);
        state_.store(McpwmProfileState::IDLE, std::memory_order_release);
        ESP_LOGE(TAG, "Failed to start profile timer: %s", esp_err_to_name(ret));
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

    // Stop MCPWM timer immediately (STOP_EMPTY prevents leftover GPIO HIGH state)
    if (timer_handle_) {
        mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_STOP_EMPTY);
    }

    // Reconfigure generator action to force GPIO LOW when timer is at zero
    esp_err_t ret = mcpwm_generator_set_action_on_timer_event(gen_handle_,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set generator LOW action: %s", esp_err_to_name(ret));
    }
    ESP_LOGW(TAG, "DEBUG stopImmediate: Reconfigured generator to force GPIO LOW");

    // Read final count and update absolute position (CRITICAL for velocity mode!)
    int64_t final_count = readPcntCount();
    pulse_count_.store(final_count, std::memory_order_relaxed);

    // Calculate new absolute position based on actual pulses moved
    int64_t delta = direction_ ? final_count : -final_count;
    int64_t new_abs_pos = move_start_position_ + delta;
    absolute_position_.store(new_abs_pos, std::memory_order_relaxed);

    ESP_LOGW(TAG, "stopImmediate: moved %lld pulses, start=%lld, new_abs=%lld",
             (long long)final_count, (long long)move_start_position_, (long long)new_abs_pos);

    // Update stable reference for background position calculation (NO RACE)
    last_completed_position_ = new_abs_pos;
    int count_hw = 0;
    pcnt_unit_get_count(pcnt_unit_, &count_hw);
    int32_t overflows_hw = overflow_count_.load(std::memory_order_relaxed);
    pcnt_at_last_completion_ = count_hw + (overflows_hw * PCNT_HIGH_LIMIT);
    ESP_LOGW(TAG, "DEBUG: Updated stable reference (stopImmediate) - last_pos=%lld, pcnt_hw=%ld",
             (long long)last_completed_position_, (long)pcnt_at_last_completion_);

    // NOTE: Keep PCNT running continuously for position tracking (don't stop it)

    // Stop profile update timer
    esp_timer_stop(profile_timer_);

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
    // Return absolute position for IPositionTracker interface
    // absolute_position_ is updated continuously every 20ms in updateProfile()
    // when motor is enabled (PCNT counting), so always return the stored value
    return absolute_position_.load(std::memory_order_relaxed);
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
// IPositionTracker Implementation
// ============================================================================

esp_err_t McpwmPulseGenerator::reset(int64_t position)
{
    // Clear PCNT hardware counter
    if (pcnt_unit_) {
        pcnt_unit_clear_count(pcnt_unit_);
    }
    // Set software accumulators
    overflow_count_.store(0, std::memory_order_relaxed);
    pulse_count_.store(0, std::memory_order_relaxed);
    absolute_position_.store(position, std::memory_order_relaxed);
    move_start_position_ = position;
    return ESP_OK;
}

void McpwmPulseGenerator::setDirection(bool forward)
{
    direction_ = forward;
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

    if (period_ticks == 0) {
        ESP_LOGE(TAG, "setFrequency: period_ticks=0 is INVALID for freq=%.1f", frequency);
        return ESP_ERR_INVALID_ARG;
    }

    // Update timer period
    esp_err_t ret = mcpwm_timer_set_period(timer_handle_, period_ticks);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "setFrequency: mcpwm_timer_set_period FAILED: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update comparator for 50% duty cycle
    // NO DELAY - MCPWM peripheral handles period/duty updates correctly without glitches
    // Previous delay was CAUSING the jerky motion by blocking for milliseconds
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
    // DISABLED: Watch point ISR was causing stale watch point bugs.
    // Task-based polling in updateProfile() is now the sole stop mechanism.
    // PCNT is used only for counting, not for triggering stop via watch points.
    (void)target_pulses;
    last_watch_point_ = 0;
    return ESP_OK;
}

int64_t McpwmPulseGenerator::readPcntCount() const
{
    int count = 0;
    if (pcnt_unit_) {
        pcnt_unit_get_count(pcnt_unit_, &count);
    }

    // Add overflow contribution for absolute PCNT count
    int32_t overflows = overflow_count_.load(std::memory_order_relaxed);
    int64_t absolute_pcnt = static_cast<int64_t>(count) + (static_cast<int64_t>(overflows) * PCNT_HIGH_LIMIT);

    // Return relative position for current move (pulses since move start)
    // PCNT runs continuously, so we subtract the baseline count from move start
    int64_t relative_pulses = absolute_pcnt - move_start_pcnt_count_;
    return relative_pulses;
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

    // DEBUG: Log that ISR was called (use ROM printf which is ISR-safe)
    ets_printf("PCNT ISR: watch_point=%d, target=%ld\n", watch_point, (long)target);

    // Check if this is an overflow event (watch point at PCNT_HIGH_LIMIT)
    if (watch_point == PCNT_HIGH_LIMIT && target > PCNT_HIGH_LIMIT) {
        // Increment overflow counter
        int32_t new_overflow = overflow_count_.fetch_add(1, std::memory_order_relaxed) + 1;

        // Clear PCNT count and continue
        pcnt_unit_clear_count(pcnt_unit_);

        // Calculate remaining pulses after this overflow
        int64_t total_counted = static_cast<int64_t>(new_overflow) * PCNT_HIGH_LIMIT;
        int64_t remaining = target - total_counted;

        ets_printf("PCNT overflow: count=%ld, remaining=%lld\n", (long)new_overflow, (long long)remaining);

        // If remaining is less than HIGH_LIMIT, set watch point for final count
        if (remaining > 0 && remaining < PCNT_HIGH_LIMIT) {
            // Remove overflow watch point
            pcnt_unit_remove_watch_point(pcnt_unit_, PCNT_HIGH_LIMIT);
            // Add final watch point
            pcnt_unit_add_watch_point(pcnt_unit_, static_cast<int>(remaining));
        }
        // Otherwise, keep the HIGH_LIMIT watch point for next overflow

        return false;
    }

    // Target watch point reached - stop MCPWM immediately
    // NOTE: Do NOT stop PCNT here - calling pcnt_unit_stop() from ISR context
    // causes issues with subsequent moves. MCPWM stop is enough since no more
    // pulses will be generated. Task will stop PCNT when it handles completion.
    // Using STOP_EMPTY prevents leftover GPIO HIGH state.
    mcpwm_timer_start_stop(timer_handle_, MCPWM_TIMER_STOP_EMPTY);

    // Calculate position using TARGET (commanded value), not PCNT read
    // Watch point fired at target, so we know we reached exactly target pulses
    int64_t delta = direction_ ? target : -target;
    absolute_position_.store(move_start_position_ + delta, std::memory_order_relaxed);
    pulse_count_.store(target, std::memory_order_relaxed);

    // Update state to IDLE so timer callback stops processing
    state_.store(McpwmProfileState::IDLE, std::memory_order_release);
    current_velocity_.store(0.0f, std::memory_order_relaxed);

    // Note: PCNT watch point callback is disabled - this function is not used
    // Profile timer callback (profileTimerCallback) detects completion via state/position checks
    return false;
}

void McpwmPulseGenerator::notifyCompletion()
{
    // Atomic check-and-set to prevent double calls from task and ISR paths
    bool expected = false;
    if (!completion_notified_.compare_exchange_strong(expected, true)) {
        return;  // Already notified for this motion
    }

    if (xSemaphoreTake(callback_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (completion_callback_) {
            // Pass absolute position (not relative PCNT count) to callback
            int64_t abs_pos = absolute_position_.load(std::memory_order_relaxed);
            completion_callback_(abs_pos);
        }
        xSemaphoreGive(callback_mutex_);
    }
    // Note: completion_notified_ is reset in updateProfile when new motion starts
}
