/**
 * @file pcnt_tracker.cpp
 * @brief PCNT-based position tracker implementation
 * @author YaRobot Team
 * @date 2025
 *
 * Architecture:
 * - PCNT counts pulses from external STEP signal (from MCPWM)
 * - Watch points at ±LIMIT_PCNT_HIGH/LOW_LIMIT detect overflow
 * - ISR accumulates overflow count to extend 16-bit to 64-bit range
 * - Direction flag controls increment/decrement semantics
 */

#include "pcnt_tracker.h"
#include "config_gpio.h"
#include "esp_log.h"

static const char* TAG = "PCNT_TRACKER";

// ============================================================================
// Constructor / Destructor
// ============================================================================

PcntTracker::PcntTracker(int pcnt_unit_id, gpio_num_t pulse_gpio)
    : pcnt_unit_id_(pcnt_unit_id)
    , pulse_gpio_(pulse_gpio)
    , pcnt_unit_(nullptr)
    , pcnt_channel_(nullptr)
    , initialized_(false)
    , overflow_accumulator_(0)
    , direction_(true)
{
}

PcntTracker::~PcntTracker()
{
    if (initialized_) {
        if (pcnt_channel_) {
            pcnt_del_channel(pcnt_channel_);
        }
        if (pcnt_unit_) {
            pcnt_unit_stop(pcnt_unit_);
            pcnt_unit_disable(pcnt_unit_);
            pcnt_del_unit(pcnt_unit_);
        }
    }
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t PcntTracker::init()
{
    if (initialized_) {
        ESP_LOGW(TAG, "PCNT unit %d already initialized", pcnt_unit_id_);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing PCNT tracker unit %d on GPIO %d",
             pcnt_unit_id_, pulse_gpio_);

    // =========================================================================
    // PCNT Unit Configuration
    // =========================================================================
    pcnt_unit_config_t pcnt_unit_config = {};
    pcnt_unit_config.high_limit = LIMIT_PCNT_HIGH_LIMIT;
    pcnt_unit_config.low_limit = LIMIT_PCNT_LOW_LIMIT;
    pcnt_unit_config.flags.accum_count = true;  // Enable accumulation mode

    esp_err_t ret = pcnt_new_unit(&pcnt_unit_config, &pcnt_unit_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // =========================================================================
    // PCNT Channel Configuration
    // =========================================================================
    pcnt_chan_config_t pcnt_chan_config = {};
    pcnt_chan_config.edge_gpio_num = pulse_gpio_;
    pcnt_chan_config.level_gpio_num = -1;  // No level signal (direction via SW)

    ret = pcnt_new_channel(pcnt_unit_, &pcnt_chan_config, &pcnt_channel_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel: %s", esp_err_to_name(ret));
        pcnt_del_unit(pcnt_unit_);
        pcnt_unit_ = nullptr;
        return ret;
    }

    // Configure channel to increment on rising edge, hold on falling edge
    // Direction handled in software via overflow accumulator sign
    ret = pcnt_channel_set_edge_action(pcnt_channel_,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // Rising edge: increment
        PCNT_CHANNEL_EDGE_ACTION_HOLD);      // Falling edge: hold
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PCNT edge action: %s", esp_err_to_name(ret));
    }

    // No level action (direction controlled via setDirection)
    ret = pcnt_channel_set_level_action(pcnt_channel_,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PCNT level action: %s", esp_err_to_name(ret));
    }

    // =========================================================================
    // Watch Points for Overflow Handling
    // =========================================================================
    ret = pcnt_unit_add_watch_point(pcnt_unit_, LIMIT_PCNT_HIGH_LIMIT);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add high watch point: %s", esp_err_to_name(ret));
    }

    ret = pcnt_unit_add_watch_point(pcnt_unit_, LIMIT_PCNT_LOW_LIMIT);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add low watch point: %s", esp_err_to_name(ret));
    }

    // Register watch point callback for overflow handling
    pcnt_event_callbacks_t pcnt_cbs = {};
    pcnt_cbs.on_reach = onPcntWatch;
    ret = pcnt_unit_register_event_callbacks(pcnt_unit_, &pcnt_cbs, this);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register PCNT callbacks: %s", esp_err_to_name(ret));
    }

    // Enable and start PCNT unit
    ret = pcnt_unit_enable(pcnt_unit_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable PCNT unit: %s", esp_err_to_name(ret));
        pcnt_del_channel(pcnt_channel_);
        pcnt_del_unit(pcnt_unit_);
        pcnt_channel_ = nullptr;
        pcnt_unit_ = nullptr;
        return ret;
    }

    ret = pcnt_unit_clear_count(pcnt_unit_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear PCNT count: %s", esp_err_to_name(ret));
    }

    ret = pcnt_unit_start(pcnt_unit_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start PCNT unit: %s", esp_err_to_name(ret));
        pcnt_unit_disable(pcnt_unit_);
        pcnt_del_channel(pcnt_channel_);
        pcnt_del_unit(pcnt_unit_);
        pcnt_channel_ = nullptr;
        pcnt_unit_ = nullptr;
        return ret;
    }

    // Initialize state
    overflow_accumulator_.store(0, std::memory_order_relaxed);
    direction_.store(true, std::memory_order_relaxed);

    initialized_ = true;
    ESP_LOGI(TAG, "PCNT tracker unit %d initialized successfully", pcnt_unit_id_);
    return ESP_OK;
}

// ============================================================================
// IPositionTracker Interface
// ============================================================================

esp_err_t PcntTracker::reset(int64_t position)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Stop PCNT, clear hardware counter, set accumulator to position
    pcnt_unit_stop(pcnt_unit_);
    pcnt_unit_clear_count(pcnt_unit_);
    overflow_accumulator_.store(position, std::memory_order_release);
    pcnt_unit_start(pcnt_unit_);

    ESP_LOGD(TAG, "PCNT unit %d reset to position %lld", pcnt_unit_id_, (long long)position);
    return ESP_OK;
}

int64_t PcntTracker::getPosition() const
{
    if (!initialized_) {
        return 0;
    }

    // Read hardware counter
    int count = 0;
    pcnt_unit_get_count(pcnt_unit_, &count);

    // Combine with overflow accumulator
    int64_t accumulator = overflow_accumulator_.load(std::memory_order_acquire);

    // Direction determines sign of hardware count contribution
    if (direction_.load(std::memory_order_relaxed)) {
        return accumulator + count;  // Forward: add count
    } else {
        return accumulator - count;  // Reverse: subtract count
    }
}

void PcntTracker::setDirection(bool forward)
{
    if (!initialized_) {
        return;
    }

    bool old_direction = direction_.load(std::memory_order_relaxed);
    if (old_direction == forward) {
        return;  // No change
    }

    // When direction changes, fold current hardware count into accumulator
    // This ensures position is continuous across direction changes
    int count = 0;
    pcnt_unit_get_count(pcnt_unit_, &count);

    int64_t accumulator = overflow_accumulator_.load(std::memory_order_relaxed);
    if (old_direction) {
        accumulator += count;  // Was forward: add count
    } else {
        accumulator -= count;  // Was reverse: subtract count
    }
    overflow_accumulator_.store(accumulator, std::memory_order_release);

    // Clear hardware counter for new direction
    pcnt_unit_clear_count(pcnt_unit_);

    // Store new direction
    direction_.store(forward, std::memory_order_release);

    ESP_LOGD(TAG, "PCNT unit %d direction changed to %s, accumulator=%lld",
             pcnt_unit_id_, forward ? "FWD" : "REV", (long long)accumulator);
}

// ============================================================================
// PCNT Callbacks - ISR SAFE
// ============================================================================

bool IRAM_ATTR PcntTracker::onPcntWatch(pcnt_unit_handle_t unit,
                                         const pcnt_watch_event_data_t* event_data,
                                         void* user_data)
{
    PcntTracker* self = static_cast<PcntTracker*>(user_data);
    return self->handlePcntWatchISR(event_data);
}

bool PcntTracker::handlePcntWatchISR(const pcnt_watch_event_data_t* event_data)
{
    int watch_point = event_data->watch_point_value;

    // Handle overflow: fold watch point value into accumulator
    int64_t accumulator = overflow_accumulator_.load(std::memory_order_relaxed);
    bool forward = direction_.load(std::memory_order_relaxed);

    if (watch_point == LIMIT_PCNT_HIGH_LIMIT) {
        // Positive overflow
        if (forward) {
            accumulator += LIMIT_PCNT_HIGH_LIMIT;
        } else {
            accumulator -= LIMIT_PCNT_HIGH_LIMIT;
        }
    } else if (watch_point == LIMIT_PCNT_LOW_LIMIT) {
        // Negative overflow
        if (forward) {
            accumulator += LIMIT_PCNT_LOW_LIMIT;
        } else {
            accumulator -= LIMIT_PCNT_LOW_LIMIT;
        }
    }

    overflow_accumulator_.store(accumulator, std::memory_order_release);

    // Clear counter to continue from zero
    pcnt_unit_clear_count(pcnt_unit_);

    return false;  // No context switch needed
}
