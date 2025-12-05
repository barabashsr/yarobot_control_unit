/**
 * @file test_pulse_cmd.cpp
 * @brief Test command handler for RMT pulse generator verification
 * @author YaRobot Team
 * @date 2025
 *
 * ============================================================================
 * STORY-3-2-TEST: Remove this entire file after hardware verification
 * ============================================================================
 *
 * This file implements temporary PULSE and MOVE commands for testing RMT pulse
 * generation with a multimeter or oscilloscope. It will be removed once
 * Story 3-2 hardware verification is complete.
 *
 * Usage via USB serial:
 *   PULSE X 10000   - Generate 10 kHz pulses on X-axis GPIO (uses RMT loop)
 *   PULSE Z 50000   - Generate 50 kHz pulses on Z-axis GPIO
 *   PULSE A 100000  - Generate 100 kHz pulses on A-axis GPIO
 *   PULSE B 200000  - Generate 200 kHz pulses on B-axis GPIO
 *   PULSE STOP      - Stop all pulse generation
 *   PULSE           - Query current pulse state
 *
 *   MOVE X 1000 10000 50000 - Move X-axis 1000 pulses at 10kHz max, 50kHz/s accel
 *   MOVE STOP               - Stop all moves immediately
 *
 * PULSE uses RMT infinite loop mode for stable constant frequency output.
 * MOVE uses RmtPulseGenerator with streaming for trapezoidal profiles.
 */

// STORY-3-2-TEST: Remove this include block after hardware verification
#include <cstring>
#include <cstdlib>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"

extern "C" {
#include "command_executor.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_gpio.h"
#include "config_limits.h"
}

// STORY-3-2-TEST: Include RmtPulseGenerator for MOVE command
#include "rmt_pulse_gen.h"

// STORY-3-3-TEST: Include McpwmPulseGenerator for Y/C axes
#include "mcpwm_pulse_gen.h"
#include "config_peripherals.h"

// STORY-3-4-TEST: Include LedcPulseGenerator for D axis
#include "ledc_pulse_gen.h"

// STORY-3-5-TEST: Include position trackers for POS command
#include "pcnt_tracker.h"
#include "software_tracker.h"
#include "time_tracker.h"

// STORY-3-2-TEST: Remove this entire section after hardware verification
static const char* TAG = "PULSE_TEST";

// STORY-3-2-TEST: RMT resolution (80 MHz = 12.5ns per tick)
static constexpr uint32_t RMT_RESOLUTION_HZ = 80000000;

// STORY-3-2-TEST: Static RMT handles for PULSE command (RMT loop mode)
static rmt_channel_handle_t s_rmt_channel[4] = {nullptr, nullptr, nullptr, nullptr};
static rmt_encoder_handle_t s_encoder[4] = {nullptr, nullptr, nullptr, nullptr};
static bool s_running[4] = {false, false, false, false};
static float s_frequency[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static rmt_symbol_word_t s_pulse_symbol[4];
// STORY-3-5-TEST: Track start time for pulse counting in continuous mode
static int64_t s_pulse_start_us[4] = {0, 0, 0, 0};

// STORY-3-2-TEST: Static RmtPulseGenerator instances for MOVE command (X, Z, A, B)
static RmtPulseGenerator* s_move_gen[4] = {nullptr, nullptr, nullptr, nullptr};
// Track direction for each RMT axis (for SoftwareTracker integration)
static bool s_rmt_direction[4] = {true, true, true, true};  // true = forward

// STORY-3-3-TEST: Static McpwmPulseGenerator instances for Y/C axes
static McpwmPulseGenerator* s_mcpwm_gen[2] = {nullptr, nullptr};  // Y=0, C=1
static bool s_mcpwm_running[2] = {false, false};
static float s_mcpwm_frequency[2] = {0.0f, 0.0f};
static int64_t s_mcpwm_start_us[2] = {0, 0};

// STORY-3-4-TEST: Static LedcPulseGenerator instance for D axis
static LedcPulseGenerator* s_ledc_gen = nullptr;
static bool s_ledc_running = false;
static float s_ledc_frequency = 0.0f;
static bool s_ledc_direction = true;  // true = forward
static int64_t s_ledc_start_us = 0;

// STORY-3-5-TEST: Static position tracker instances
// PcntTracker for Y and C axes (hardware PCNT)
static PcntTracker* s_pcnt_tracker[2] = {nullptr, nullptr};  // Y=0, C=1
// SoftwareTracker for X, Z, A, B, D axes
static SoftwareTracker* s_sw_tracker[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};  // X=0, Z=1, A=2, B=3, D=4
// TimeTracker for E axis (discrete actuator)
static TimeTracker* s_time_tracker = nullptr;

/**
 * @brief Get RMT axis index (0-3 for X,Z,A,B)
 * STORY-3-2-TEST: Remove after hardware verification
 */
static int get_rmt_axis_index(char axis)
{
    switch (axis) {
        case 'X': case 'x': return 0;
        case 'Z': case 'z': return 1;
        case 'A': case 'a': return 2;
        case 'B': case 'b': return 3;
        default: return -1;
    }
}

/**
 * @brief Get MCPWM axis index (0=Y, 1=C)
 * STORY-3-3-TEST: Remove after hardware verification
 */
static int get_mcpwm_axis_index(char axis)
{
    switch (axis) {
        case 'Y': case 'y': return 0;
        case 'C': case 'c': return 1;
        default: return -1;
    }
}

/**
 * @brief Check if axis uses MCPWM (Y, C) vs RMT (X, Z, A, B)
 * STORY-3-3-TEST: Remove after hardware verification
 */
static bool is_mcpwm_axis(char axis)
{
    return (axis == 'Y' || axis == 'y' || axis == 'C' || axis == 'c');
}

/**
 * @brief Check if axis uses LEDC (D)
 * STORY-3-4-TEST: Remove after hardware verification
 */
static bool is_ledc_axis(char axis)
{
    return (axis == 'D' || axis == 'd');
}

/**
 * @brief Check if axis is E (discrete actuator)
 * STORY-3-5-TEST: Remove after hardware verification
 */
static bool is_discrete_axis(char axis)
{
    return (axis == 'E' || axis == 'e');
}

/**
 * @brief Get SoftwareTracker index for axis (0=X, 1=Z, 2=A, 3=B, 4=D)
 * STORY-3-5-TEST: Remove after hardware verification
 */
static int get_sw_tracker_index(char axis)
{
    switch (axis) {
        case 'X': case 'x': return 0;
        case 'Z': case 'z': return 1;
        case 'A': case 'a': return 2;
        case 'B': case 'b': return 3;
        case 'D': case 'd': return 4;
        default: return -1;
    }
}

/**
 * @brief Ensure SoftwareTracker exists for axis and set direction
 * STORY-3-5-TEST: Remove after hardware verification
 * @return SoftwareTracker pointer, or nullptr if not applicable (Y/C/E)
 */
static SoftwareTracker* ensure_sw_tracker(char axis, bool forward)
{
    int idx = get_sw_tracker_index(axis);
    if (idx < 0) return nullptr;  // Not a software tracker axis

    if (s_sw_tracker[idx] == nullptr) {
        ESP_LOGI(TAG, "Creating SoftwareTracker for axis %c", axis);
        s_sw_tracker[idx] = new SoftwareTracker();
        esp_err_t ret = s_sw_tracker[idx]->init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init SoftwareTracker: %s", esp_err_to_name(ret));
            delete s_sw_tracker[idx];
            s_sw_tracker[idx] = nullptr;
            return nullptr;
        }
    }
    s_sw_tracker[idx]->setDirection(forward);
    return s_sw_tracker[idx];
}

/**
 * @brief Get GPIO for axis (all axes including Y/C/D)
 * STORY-3-2-TEST: Remove after hardware verification
 */
static int get_gpio_for_axis(char axis)
{
    switch (axis) {
        case 'X': case 'x': return GPIO_X_STEP;
        case 'Y': case 'y': return GPIO_Y_STEP;
        case 'Z': case 'z': return GPIO_Z_STEP;
        case 'A': case 'a': return GPIO_A_STEP;
        case 'B': case 'b': return GPIO_B_STEP;
        case 'C': case 'c': return GPIO_C_STEP;
        case 'D': case 'd': return GPIO_D_STEP;
        default: return -1;
    }
}

/**
 * @brief Cleanup PULSE RMT channel for axis (release resources)
 * STORY-3-2-TEST: Remove after hardware verification
 */
static void cleanup_pulse_axis(int idx)
{
    if (idx >= 0 && idx < 4) {
        if (s_rmt_channel[idx]) {
            rmt_disable(s_rmt_channel[idx]);
            if (s_encoder[idx]) {
                rmt_del_encoder(s_encoder[idx]);
                s_encoder[idx] = nullptr;
            }
            rmt_del_channel(s_rmt_channel[idx]);
            s_rmt_channel[idx] = nullptr;
        }
        s_running[idx] = false;
        s_frequency[idx] = 0.0f;
    }
}

/**
 * @brief Cleanup MOVE generator for axis (release resources)
 * STORY-3-2-TEST: Remove after hardware verification
 */
static void cleanup_move_axis(int idx)
{
    if (idx >= 0 && idx < 4 && s_move_gen[idx]) {
        delete s_move_gen[idx];
        s_move_gen[idx] = nullptr;
    }
}

/**
 * @brief Stop pulse generation on axis and update position tracker
 * STORY-3-2-TEST: Remove after hardware verification
 */
static void stop_axis(int idx)
{
    if (idx < 0 || idx >= 4) return;

    // If using RmtPulseGenerator (velocity mode), get real pulse count
    if (s_move_gen[idx] && s_move_gen[idx]->isRunning()) {
        int64_t pulses = s_move_gen[idx]->getPulseCount();
        s_move_gen[idx]->stopImmediate();

        // Update SoftwareTracker with actual pulses
        if (s_sw_tracker[idx] && pulses > 0) {
            s_sw_tracker[idx]->addPulses(pulses);
            ESP_LOGI(TAG, "Axis %d: added %lld pulses from generator", idx, (long long)pulses);
        }

        s_running[idx] = false;
        s_frequency[idx] = 0.0f;
        ESP_LOGI(TAG, "Stopped axis %d (generator mode)", idx);
        return;
    }

    // Legacy: raw RMT channel (should not happen anymore, but keep for safety)
    if (s_rmt_channel[idx] && s_running[idx]) {
        rmt_disable(s_rmt_channel[idx]);
        rmt_enable(s_rmt_channel[idx]);
        s_running[idx] = false;
        s_frequency[idx] = 0.0f;
        ESP_LOGI(TAG, "Stopped axis %d (raw RMT)", idx);
    }
}

/**
 * @brief Cleanup MCPWM generator for axis
 * STORY-3-3-TEST: Remove after hardware verification
 */
static void cleanup_mcpwm_axis(int idx)
{
    if (idx >= 0 && idx < 2 && s_mcpwm_gen[idx]) {
        s_mcpwm_gen[idx]->stopImmediate();
        delete s_mcpwm_gen[idx];
        s_mcpwm_gen[idx] = nullptr;
        s_mcpwm_running[idx] = false;
        s_mcpwm_frequency[idx] = 0.0f;
    }
}

/**
 * @brief Stop MCPWM pulse generation on axis
 * STORY-3-3-TEST: Remove after hardware verification
 */
static void stop_mcpwm_axis(int idx)
{
    if (idx >= 0 && idx < 2 && s_mcpwm_gen[idx] && s_mcpwm_running[idx]) {
        s_mcpwm_gen[idx]->stopImmediate();
        s_mcpwm_running[idx] = false;
        s_mcpwm_frequency[idx] = 0.0f;
        ESP_LOGI(TAG, "Stopped MCPWM axis %d", idx);
    }
}

/**
 * @brief Cleanup LEDC generator for D axis
 * STORY-3-4-TEST: Remove after hardware verification
 */
static void cleanup_ledc_axis(void)
{
    if (s_ledc_gen) {
        s_ledc_gen->stopImmediate();
        delete s_ledc_gen;
        s_ledc_gen = nullptr;
        s_ledc_running = false;
        s_ledc_frequency = 0.0f;
    }
}

/**
 * @brief Stop LEDC pulse generation on D axis and update position tracker
 * STORY-3-4-TEST: Remove after hardware verification
 */
static void stop_ledc_axis(void)
{
    if (s_ledc_running) {
        int64_t pulses = 0;

        // If using LedcPulseGenerator, get real pulse count from generator
        if (s_ledc_gen && s_ledc_gen->isRunning()) {
            pulses = s_ledc_gen->getPulseCount();
            s_ledc_gen->stopImmediate();
            ESP_LOGI(TAG, "D axis: got %lld pulses from generator", (long long)pulses);
        } else if (s_ledc_gen) {
            // Generator exists but not running - just stop it
            s_ledc_gen->stopImmediate();
        } else {
            // Legacy: direct LEDC API (shouldn't happen anymore)
            ledc_stop(LEDC_MODE_D, LEDC_CHANNEL_D, 0);
            // Fallback to time-based calculation
            int64_t elapsed_us = esp_timer_get_time() - s_ledc_start_us;
            pulses = static_cast<int64_t>(s_ledc_frequency * elapsed_us / 1000000.0f);
        }

        // Update SoftwareTracker with actual pulses (D is index 4)
        if (s_sw_tracker[4] && pulses > 0) {
            s_sw_tracker[4]->addPulses(pulses);
            ESP_LOGI(TAG, "D axis: added %lld pulses to tracker", (long long)pulses);
        }

        s_ledc_running = false;
        s_ledc_frequency = 0.0f;
        s_ledc_start_us = 0;
        ESP_LOGI(TAG, "Stopped LEDC axis D");
    }
}

/**
 * @brief Stop all pulse generation (RMT, MCPWM, and LEDC)
 * STORY-3-2-TEST: Remove after hardware verification
 */
static void stop_all_pulses(void)
{
    // Stop RMT axes (X, Z, A, B)
    for (int i = 0; i < 4; i++) {
        stop_axis(i);
    }
    // Stop MCPWM axes (Y, C)
    for (int i = 0; i < 2; i++) {
        stop_mcpwm_axis(i);
    }
    // STORY-3-4-TEST: Stop LEDC axis (D)
    stop_ledc_axis();
}

/**
 * @brief Initialize RMT channel for axis
 * STORY-3-2-TEST: Remove after hardware verification
 */
static esp_err_t init_rmt_channel(int idx, int gpio)
{
    if (s_rmt_channel[idx] != nullptr) {
        return ESP_OK;  // Already initialized
    }

    ESP_LOGI(TAG, "Initializing RMT channel %d on GPIO %d", idx, gpio);

    // Configure RMT TX channel - simple, no DMA needed for loop mode
    rmt_tx_channel_config_t tx_config = {};
    tx_config.gpio_num = static_cast<gpio_num_t>(gpio);
    tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_config.resolution_hz = RMT_RESOLUTION_HZ;
    tx_config.mem_block_symbols = 64;
    tx_config.trans_queue_depth = 1;  // Only need 1 for loop mode
    tx_config.flags.with_dma = false;  // No DMA for simple loop
    tx_config.flags.invert_out = false;

    esp_err_t ret = rmt_new_tx_channel(&tx_config, &s_rmt_channel[idx]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create copy encoder
    rmt_copy_encoder_config_t encoder_config = {};
    ret = rmt_new_copy_encoder(&encoder_config, &s_encoder[idx]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create encoder: %s", esp_err_to_name(ret));
        rmt_del_channel(s_rmt_channel[idx]);
        s_rmt_channel[idx] = nullptr;
        return ret;
    }

    // Enable channel
    ret = rmt_enable(s_rmt_channel[idx]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT: %s", esp_err_to_name(ret));
        rmt_del_encoder(s_encoder[idx]);
        rmt_del_channel(s_rmt_channel[idx]);
        s_encoder[idx] = nullptr;
        s_rmt_channel[idx] = nullptr;
        return ret;
    }

    ESP_LOGI(TAG, "RMT channel %d initialized on GPIO %d", idx, gpio);
    return ESP_OK;
}

/**
 * @brief Start continuous pulses at frequency using RMT infinite loop
 * STORY-3-2-TEST: Remove after hardware verification
 */
static esp_err_t start_pulses(int idx, float freq_hz)
{
    // Calculate tick period: ticks = resolution / frequency
    uint32_t period_ticks = static_cast<uint32_t>(RMT_RESOLUTION_HZ / freq_hz);
    uint32_t half_period = period_ticks / 2;

    // Clamp to valid range (15-bit max for each duration)
    if (half_period < 1) half_period = 1;
    if (half_period > 32767) half_period = 32767;

    // Create pulse symbol: HIGH for half, LOW for half
    s_pulse_symbol[idx].duration0 = static_cast<uint16_t>(half_period);
    s_pulse_symbol[idx].level0 = 1;
    s_pulse_symbol[idx].duration1 = static_cast<uint16_t>(half_period);
    s_pulse_symbol[idx].level1 = 0;

    // Calculate actual frequency
    float actual_freq = static_cast<float>(RMT_RESOLUTION_HZ) / static_cast<float>(half_period * 2);

    ESP_LOGI(TAG, "Axis %d: requested=%.0f Hz, period=%lu ticks, half=%lu, actual=%.1f Hz",
             idx, freq_hz, (unsigned long)period_ticks, (unsigned long)half_period, actual_freq);

    // Configure for infinite loop
    rmt_transmit_config_t tx_config = {};
    tx_config.loop_count = -1;  // Infinite loop!

    // Transmit the single symbol in infinite loop
    esp_err_t ret = rmt_transmit(s_rmt_channel[idx], s_encoder[idx],
                                  &s_pulse_symbol[idx], sizeof(rmt_symbol_word_t),
                                  &tx_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start transmission: %s", esp_err_to_name(ret));
        return ret;
    }

    s_running[idx] = true;
    s_frequency[idx] = actual_freq;
    s_pulse_start_us[idx] = esp_timer_get_time();  // Record start time for pulse counting
    ESP_LOGI(TAG, "Started axis %d at %.1f Hz", idx, actual_freq);

    return ESP_OK;
}

/**
 * @brief Handle PULSE test command
 * STORY-3-2-TEST: Remove this entire function after hardware verification
 *
 * Format:
 *   PULSE <axis> <freq_hz>  - Start continuous pulses at frequency
 *   PULSE STOP              - Stop all pulse generation
 *   PULSE                   - Query status
 *
 * Supported axes:
 *   X, Z, A, B - RMT-based (infinite loop mode)
 *   Y, C       - MCPWM-based (velocity mode)
 *   D          - LEDC-based (velocity mode)
 */
static esp_err_t handle_pulse_test(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Handle PULSE STOP - parser sees 'S' as axis since it's a single letter
    // Check for axis 'S' which indicates "STOP" command (S is not a valid motor axis)
    if (cmd->axis == 'S' || cmd->axis == 's') {
        stop_all_pulses();
        ESP_LOGI(TAG, "Stopped all pulse generation");
        return format_ok_data(response, resp_len, "STOPPED");
    }

    // Also handle via str_param for backward compatibility
    if (cmd->has_str_param && strcasecmp(cmd->str_param, "STOP") == 0) {
        stop_all_pulses();
        ESP_LOGI(TAG, "Stopped all pulse generation");
        return format_ok_data(response, resp_len, "STOPPED");
    }

    // Handle PULSE (query status) - show all 7 axes
    if (cmd->axis == '\0' && cmd->param_count == 0 && !cmd->has_str_param) {
        return format_ok_data(response, resp_len,
            "X:%s(%.0f) Y:%s(%.0f) Z:%s(%.0f) A:%s(%.0f) B:%s(%.0f) C:%s(%.0f) D:%s(%.0f)",
            s_running[0] ? "ON" : "OFF", s_frequency[0],
            s_mcpwm_running[0] ? "ON" : "OFF", s_mcpwm_frequency[0],
            s_running[1] ? "ON" : "OFF", s_frequency[1],
            s_running[2] ? "ON" : "OFF", s_frequency[2],
            s_running[3] ? "ON" : "OFF", s_frequency[3],
            s_mcpwm_running[1] ? "ON" : "OFF", s_mcpwm_frequency[1],
            s_ledc_running ? "ON" : "OFF", s_ledc_frequency);
    }

    // Handle PULSE <axis> <freq>
    if (cmd->axis == '\0') {
        return format_error(response, resp_len, ERR_INVALID_AXIS, "Specify axis X/Y/Z/A/B/C/D");
    }

    char axis = cmd->axis;
    int gpio = get_gpio_for_axis(axis);
    if (gpio < 0) {
        return format_error(response, resp_len, ERR_INVALID_AXIS, "Invalid axis");
    }

    // Get frequency parameter
    if (cmd->param_count < 1) {
        return format_error(response, resp_len, ERR_INVALID_PARAMETER, "Specify frequency in Hz");
    }

    float freq_hz = cmd->params[0];
    if (freq_hz < LIMIT_MIN_PULSE_FREQ_HZ || freq_hz > LIMIT_MAX_PULSE_FREQ_HZ) {
        return format_error(response, resp_len, ERR_INVALID_PARAMETER,
            "Freq must be 1-500000 Hz");
    }

    esp_err_t ret;

    // STORY-3-3-TEST: Handle MCPWM axes (Y, C)
    if (is_mcpwm_axis(axis)) {
        int idx = get_mcpwm_axis_index(axis);

        // Stop if already running
        if (s_mcpwm_running[idx]) {
            stop_mcpwm_axis(idx);
        }

        // Create generator if needed
        if (s_mcpwm_gen[idx] == nullptr) {
            int timer_id = (idx == 0) ? MCPWM_TIMER_Y : MCPWM_TIMER_C;
            int pcnt_id = (idx == 0) ? PCNT_UNIT_Y : PCNT_UNIT_C;
            ESP_LOGI(TAG, "Creating McpwmPulseGenerator for axis %c (timer=%d, pcnt=%d, gpio=%d)",
                     axis, timer_id, pcnt_id, gpio);
            s_mcpwm_gen[idx] = new McpwmPulseGenerator(timer_id, gpio, pcnt_id);
            ret = s_mcpwm_gen[idx]->init();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to init McpwmPulseGenerator: %s", esp_err_to_name(ret));
                delete s_mcpwm_gen[idx];
                s_mcpwm_gen[idx] = nullptr;
                return format_error(response, resp_len, ERR_CONFIGURATION, "MCPWM init failed");
            }
        }

        // Start velocity mode (continuous pulses)
        ret = s_mcpwm_gen[idx]->startVelocity(freq_hz, freq_hz * 10.0f);  // Fast accel
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "startVelocity failed: %s", esp_err_to_name(ret));
            return format_error(response, resp_len, ERR_CONFIGURATION, "Start failed");
        }

        s_mcpwm_running[idx] = true;
        s_mcpwm_frequency[idx] = freq_hz;
        ESP_LOGI(TAG, "Started MCPWM axis %c at %.0f Hz (GPIO %d)", axis, freq_hz, gpio);

        return format_ok_data(response, resp_len, "%c FREQ:%.0f GPIO:%d (MCPWM)",
            axis, freq_hz, gpio);
    }

    // STORY-3-4-TEST: Handle LEDC axis (D) - use LedcPulseGenerator for real-time pulse counting
    if (is_ledc_axis(axis)) {
        // Validate LEDC frequency limits
        float actual_freq = freq_hz;
        if (actual_freq < LIMIT_LEDC_MIN_FREQ_HZ) {
            actual_freq = LIMIT_LEDC_MIN_FREQ_HZ;
            ESP_LOGW(TAG, "Clamped D-axis freq to %d Hz (LEDC min)", LIMIT_LEDC_MIN_FREQ_HZ);
        }
        if (actual_freq > LIMIT_LEDC_MAX_FREQ_HZ) {
            actual_freq = LIMIT_LEDC_MAX_FREQ_HZ;
            ESP_LOGW(TAG, "Clamped D-axis freq to %d Hz (LEDC max)", LIMIT_LEDC_MAX_FREQ_HZ);
        }

        // Stop if already running
        if (s_ledc_running) {
            stop_ledc_axis();
        }

        // Create generator if needed
        if (s_ledc_gen == nullptr) {
            ESP_LOGI(TAG, "Creating LedcPulseGenerator for PULSE on axis D (GPIO %d)", gpio);
            s_ledc_gen = new LedcPulseGenerator(gpio, LEDC_TIMER_D, LEDC_CHANNEL_D);
            ret = s_ledc_gen->init();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to init LedcPulseGenerator: %s", esp_err_to_name(ret));
                delete s_ledc_gen;
                s_ledc_gen = nullptr;
                return format_error(response, resp_len, ERR_CONFIGURATION, "LEDC init failed");
            }
        }

        // STORY-3-5-TEST: Ensure SoftwareTracker exists and connect to generator
        SoftwareTracker* tracker = ensure_sw_tracker(axis, true);  // Default to forward direction
        if (tracker) {
            s_ledc_gen->setPositionTracker(tracker);
        }

        // Start velocity mode (continuous pulses with real-time pulse counting)
        ESP_LOGI(TAG, "Starting PULSE D at %.0f Hz via LedcPulseGenerator velocity mode", actual_freq);
        ret = s_ledc_gen->startVelocity(actual_freq, actual_freq * 10.0f);  // Fast acceleration
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "startVelocity failed: %s", esp_err_to_name(ret));
            return format_error(response, resp_len, ERR_CONFIGURATION, "Start failed");
        }

        s_ledc_running = true;
        s_ledc_frequency = actual_freq;

        return format_ok_data(response, resp_len, "%c FREQ:%.0f GPIO:%d (LEDC)",
            axis, actual_freq, gpio);
    }

    // Handle RMT axes (X, Z, A, B) - use RmtPulseGenerator for real-time pulse counting
    int idx = get_rmt_axis_index(axis);
    if (idx < 0) {
        return format_error(response, resp_len, ERR_INVALID_AXIS, "Invalid RMT axis");
    }

    // Stop if already running (either raw RMT or generator)
    if (s_running[idx]) {
        stop_axis(idx);
    }
    if (s_move_gen[idx] && s_move_gen[idx]->isRunning()) {
        s_move_gen[idx]->stopImmediate();
    }

    // Cleanup raw RMT channel if exists (we'll use RmtPulseGenerator instead)
    cleanup_pulse_axis(idx);

    // Create generator if needed
    if (s_move_gen[idx] == nullptr) {
        ESP_LOGI(TAG, "Creating RmtPulseGenerator for PULSE on axis %c (GPIO %d)", axis, gpio);
        s_move_gen[idx] = new RmtPulseGenerator(idx, gpio);
        ret = s_move_gen[idx]->init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init RmtPulseGenerator: %s", esp_err_to_name(ret));
            delete s_move_gen[idx];
            s_move_gen[idx] = nullptr;
            return format_error(response, resp_len, ERR_CONFIGURATION, "RMT init failed");
        }
    }

    // STORY-3-5-TEST: Ensure SoftwareTracker exists and connect to generator
    SoftwareTracker* tracker = ensure_sw_tracker(axis, true);
    if (tracker) {
        s_move_gen[idx]->setPositionTracker(tracker);
    }

    // Start velocity mode (continuous pulses with real-time pulse counting)
    ESP_LOGI(TAG, "Starting PULSE %c at %.0f Hz via RmtPulseGenerator velocity mode", axis, freq_hz);
    ret = s_move_gen[idx]->startVelocity(freq_hz, freq_hz * 10.0f);  // Fast acceleration
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "startVelocity failed: %s", esp_err_to_name(ret));
        return format_error(response, resp_len, ERR_CONFIGURATION, "Start failed");
    }

    s_running[idx] = true;
    s_frequency[idx] = freq_hz;

    return format_ok_data(response, resp_len, "%c FREQ:%.0f GPIO:%d (RMT)",
        axis, freq_hz, gpio);
}

// ============================================================================
// STORY-3-2-TEST: MOVE command - tests RmtPulseGenerator streaming mode
// ============================================================================

/**
 * @brief Stop all moves immediately (RMT and LEDC)
 * STORY-3-2-TEST: Remove after hardware verification
 */
static void stop_all_moves(void)
{
    // Stop RMT axes (X, Z, A, B)
    for (int i = 0; i < 4; i++) {
        if (s_move_gen[i] && s_move_gen[i]->isRunning()) {
            s_move_gen[i]->stopImmediate();
            ESP_LOGI(TAG, "Stopped MOVE on axis %d", i);
        }
    }
    // STORY-3-4-TEST: Stop LEDC axis (D)
    if (s_ledc_gen && s_ledc_gen->isRunning()) {
        s_ledc_gen->stopImmediate();
        ESP_LOGI(TAG, "Stopped MOVE on axis D");
    }
}

/**
 * @brief Handle MOVE test command
 * STORY-3-2-TEST: Remove after hardware verification
 *
 * Format:
 *   MOVE <axis> <pulses> <max_freq> <accel>  - Start trapezoidal move
 *   MOVE STOP                                 - Stop all moves
 *   MOVE                                      - Query status
 *
 * Supported axes:
 *   X, Z, A, B - RMT-based (RmtPulseGenerator)
 *   D          - LEDC-based (LedcPulseGenerator) - STORY-3-4-TEST
 */
static esp_err_t handle_move_test(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // STORY-3-2-TEST: Handle MOVE STOP - parser sees 'S' as axis
    if (cmd->axis == 'S' || cmd->axis == 's') {
        stop_all_moves();
        return format_ok_data(response, resp_len, "STOPPED");
    }

    // Also handle via str_param for backward compatibility
    if (cmd->has_str_param && strcasecmp(cmd->str_param, "STOP") == 0) {
        stop_all_moves();
        return format_ok_data(response, resp_len, "STOPPED");
    }

    // STORY-3-2-TEST: Handle MOVE (query status) - now includes D axis
    if (cmd->axis == '\0' && cmd->param_count == 0 && !cmd->has_str_param) {
        return format_ok_data(response, resp_len,
            "X:%s(%lld) Z:%s(%lld) A:%s(%lld) B:%s(%lld) D:%s(%lld)",
            s_move_gen[0] && s_move_gen[0]->isRunning() ? "RUN" : "IDLE",
            s_move_gen[0] ? s_move_gen[0]->getPulseCount() : 0LL,
            s_move_gen[1] && s_move_gen[1]->isRunning() ? "RUN" : "IDLE",
            s_move_gen[1] ? s_move_gen[1]->getPulseCount() : 0LL,
            s_move_gen[2] && s_move_gen[2]->isRunning() ? "RUN" : "IDLE",
            s_move_gen[2] ? s_move_gen[2]->getPulseCount() : 0LL,
            s_move_gen[3] && s_move_gen[3]->isRunning() ? "RUN" : "IDLE",
            s_move_gen[3] ? s_move_gen[3]->getPulseCount() : 0LL,
            s_ledc_gen && s_ledc_gen->isRunning() ? "RUN" : "IDLE",
            s_ledc_gen ? s_ledc_gen->getPulseCount() : 0LL);
    }

    // STORY-3-2-TEST: Handle MOVE <axis> <pulses> <max_freq> <accel>
    if (cmd->axis == '\0') {
        return format_error(response, resp_len, ERR_INVALID_AXIS, "Specify axis X/Z/A/B/D");
    }

    char axis = cmd->axis;
    int gpio = get_gpio_for_axis(axis);
    if (gpio < 0) {
        return format_error(response, resp_len, ERR_INVALID_AXIS, "Invalid axis");
    }

    // Need 3 parameters: pulses, max_freq, accel
    if (cmd->param_count < 3) {
        return format_error(response, resp_len, ERR_INVALID_PARAMETER,
            "Usage: MOVE <axis> <pulses> <max_freq> <accel>");
    }

    int32_t pulses = static_cast<int32_t>(cmd->params[0]);
    float max_freq = cmd->params[1];
    float accel = cmd->params[2];

    if (pulses == 0) {
        return format_error(response, resp_len, ERR_INVALID_PARAMETER, "Pulses must be non-zero");
    }
    if (accel <= 0) {
        return format_error(response, resp_len, ERR_INVALID_PARAMETER, "Accel must be > 0");
    }

    esp_err_t ret;

    // STORY-3-4-TEST: Handle D axis with LedcPulseGenerator
    if (is_ledc_axis(axis)) {
        // Validate LEDC frequency limits
        if (max_freq < LIMIT_LEDC_MIN_FREQ_HZ || max_freq > LIMIT_LEDC_MAX_FREQ_HZ) {
            return format_error(response, resp_len, ERR_INVALID_PARAMETER,
                "D-axis freq must be 100-75000 Hz");
        }

        // Stop if already running
        if (s_ledc_gen && s_ledc_gen->isRunning()) {
            s_ledc_gen->stopImmediate();
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Create generator if needed
        if (s_ledc_gen == nullptr) {
            ESP_LOGI(TAG, "Creating LedcPulseGenerator for axis D (timer=%d, channel=%d, gpio=%d)",
                     LEDC_TIMER_D, LEDC_CHANNEL_D, gpio);
            s_ledc_gen = new LedcPulseGenerator(gpio, LEDC_TIMER_D, LEDC_CHANNEL_D);
            ret = s_ledc_gen->init();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to init LedcPulseGenerator: %s", esp_err_to_name(ret));
                delete s_ledc_gen;
                s_ledc_gen = nullptr;
                return format_error(response, resp_len, ERR_CONFIGURATION, "LEDC init failed");
            }
        }

        // STORY-3-5-TEST: Set direction and connect tracker for real-time position updates
        bool forward = (pulses > 0);
        s_ledc_direction = forward;
        SoftwareTracker* tracker = ensure_sw_tracker(axis, forward);
        if (tracker) {
            // Connect tracker for real-time updates during motion
            s_ledc_gen->setPositionTracker(tracker);
            // Note: No completion callback needed - tracker is updated in real-time
            // and final sync happens in stopPulseOutput()
        }

        // Start the move
        ESP_LOGI(TAG, "Starting MOVE D: pulses=%ld freq=%.0f accel=%.0f dir=%s (LEDC)",
                 (long)pulses, max_freq, accel, forward ? "FWD" : "REV");
        ret = s_ledc_gen->startMove(pulses, max_freq, accel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "startMove failed: %s", esp_err_to_name(ret));
            return format_error(response, resp_len, ERR_CONFIGURATION, "Start failed");
        }

        return format_ok_data(response, resp_len, "%c PULSES:%ld FREQ:%.0f ACCEL:%.0f (LEDC)",
            axis, (long)pulses, max_freq, accel);
    }

    // Handle RMT axes (X, Z, A, B)
    int idx = get_rmt_axis_index(axis);
    if (idx < 0) {
        return format_error(response, resp_len, ERR_INVALID_AXIS, "Only X/Z/A/B/D");
    }

    // Validate RMT frequency limits
    if (max_freq < LIMIT_MIN_PULSE_FREQ_HZ || max_freq > LIMIT_MAX_PULSE_FREQ_HZ) {
        return format_error(response, resp_len, ERR_INVALID_PARAMETER,
            "Freq must be 1-500000 Hz");
    }

    // Stop if already running
    if (s_move_gen[idx] && s_move_gen[idx]->isRunning()) {
        s_move_gen[idx]->stopImmediate();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Cleanup any PULSE channel on this axis (they share GPIO)
    cleanup_pulse_axis(idx);

    // Create generator if needed
    if (s_move_gen[idx] == nullptr) {
        ESP_LOGI(TAG, "Creating RmtPulseGenerator for axis %c on GPIO %d", axis, gpio);
        s_move_gen[idx] = new RmtPulseGenerator(idx, gpio);
        ret = s_move_gen[idx]->init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init RmtPulseGenerator: %s", esp_err_to_name(ret));
            delete s_move_gen[idx];
            s_move_gen[idx] = nullptr;
            return format_error(response, resp_len, ERR_CONFIGURATION, "RMT init failed");
        }
    }

    // STORY-3-5b: Set direction and connect tracker for real-time position updates
    bool forward = (pulses > 0);
    s_rmt_direction[idx] = forward;
    SoftwareTracker* tracker = ensure_sw_tracker(axis, forward);
    if (tracker) {
        // Connect tracker for real-time updates during motion (on each DMA buffer completion)
        s_move_gen[idx]->setPositionTracker(tracker);
        // Note: No completion callback needed - tracker is updated in real-time
    }

    // Start the move
    ESP_LOGI(TAG, "Starting MOVE: axis=%c pulses=%ld freq=%.0f accel=%.0f dir=%s",
             axis, (long)pulses, max_freq, accel, forward ? "FWD" : "REV");
    ret = s_move_gen[idx]->startMove(pulses, max_freq, accel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "startMove failed: %s", esp_err_to_name(ret));
        return format_error(response, resp_len, ERR_CONFIGURATION, "Start failed");
    }

    return format_ok_data(response, resp_len, "%c PULSES:%ld FREQ:%.0f ACCEL:%.0f",
        axis, (long)pulses, max_freq, accel);
}

// ============================================================================
// STORY-3-5-TEST: POS command - tests position tracker implementations
// ============================================================================

/**
 * @brief Get or create position tracker for axis
 * STORY-3-5-TEST: Remove after hardware verification
 */
static IPositionTracker* get_or_create_tracker(char axis)
{
    esp_err_t ret;

    // PCNT trackers for Y, C
    if (is_mcpwm_axis(axis)) {
        int idx = get_mcpwm_axis_index(axis);
        if (s_pcnt_tracker[idx] == nullptr) {
            int pcnt_id = (idx == 0) ? PCNT_UNIT_Y : PCNT_UNIT_C;
            gpio_num_t gpio = (idx == 0) ? GPIO_Y_STEP : GPIO_C_STEP;
            ESP_LOGI(TAG, "Creating PcntTracker for axis %c (pcnt=%d, gpio=%d)", axis, pcnt_id, gpio);
            s_pcnt_tracker[idx] = new PcntTracker(pcnt_id, gpio);
            ret = s_pcnt_tracker[idx]->init();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to init PcntTracker: %s", esp_err_to_name(ret));
                delete s_pcnt_tracker[idx];
                s_pcnt_tracker[idx] = nullptr;
                return nullptr;
            }
        }
        return s_pcnt_tracker[idx];
    }

    // TimeTracker for E
    if (is_discrete_axis(axis)) {
        if (s_time_tracker == nullptr) {
            ESP_LOGI(TAG, "Creating TimeTracker for axis E");
            s_time_tracker = new TimeTracker(TIMING_E_AXIS_TRAVEL_MS);
            ret = s_time_tracker->init();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to init TimeTracker: %s", esp_err_to_name(ret));
                delete s_time_tracker;
                s_time_tracker = nullptr;
                return nullptr;
            }
        }
        return s_time_tracker;
    }

    // SoftwareTracker for X, Z, A, B, D
    int idx = get_sw_tracker_index(axis);
    if (idx >= 0) {
        if (s_sw_tracker[idx] == nullptr) {
            ESP_LOGI(TAG, "Creating SoftwareTracker for axis %c", axis);
            s_sw_tracker[idx] = new SoftwareTracker();
            ret = s_sw_tracker[idx]->init();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to init SoftwareTracker: %s", esp_err_to_name(ret));
                delete s_sw_tracker[idx];
                s_sw_tracker[idx] = nullptr;
                return nullptr;
            }
        }
        return s_sw_tracker[idx];
    }

    return nullptr;
}

/**
 * @brief Get tracker type string for display
 * STORY-3-5-TEST: Remove after hardware verification
 */
static const char* get_tracker_type(char axis)
{
    if (is_mcpwm_axis(axis)) return "PCNT";
    if (is_discrete_axis(axis)) return "TIME";
    return "SW";
}

/**
 * @brief Get real-time position for RMT axis including running pulses
 * STORY-3-5-TEST: Remove after hardware verification
 */
static int64_t get_rmt_axis_position(int idx)
{
    // With setPositionTracker() connected, tracker is updated in real-time
    // by the generator on each DMA buffer completion - no need to add getPulseCount()
    return s_sw_tracker[idx] ? s_sw_tracker[idx]->getPosition() : 0;
}

/**
 * @brief Get real-time position for LEDC axis (D)
 * STORY-3-5-TEST: Remove after hardware verification
 */
static int64_t get_ledc_axis_position(void)
{
    // With setPositionTracker() connected, tracker is updated in real-time
    // by the generator via periodic timer - no need to add getPulseCount()
    return s_sw_tracker[4] ? s_sw_tracker[4]->getPosition() : 0;
}

/**
 * @brief Get diagnostic info for LEDC axis (D) - shows both tracker and generator counts
 * STORY-3-5-TEST: Debug helper - remove after hardware verification
 */
static void get_ledc_debug_info(int64_t* tracker_pos, int64_t* gen_count, bool* running, bool* has_tracker)
{
    *tracker_pos = s_sw_tracker[4] ? s_sw_tracker[4]->getPosition() : 0;
    *gen_count = s_ledc_gen ? s_ledc_gen->getPulseCount() : 0;
    *running = s_ledc_gen ? s_ledc_gen->isRunning() : false;
    *has_tracker = s_ledc_gen ? s_ledc_gen->hasPositionTracker() : false;
}

/**
 * @brief Handle POS test command
 * STORY-3-5-TEST: Remove after hardware verification
 *
 * Format (numeric params only due to parser limitations):
 *   POS                    - Query all positions
 *   POS <axis>             - Query single axis position
 *   POS <axis> <value>     - Reset axis position to value
 *   POS <axis> <p1> <p2>   - p1=mode (1=add, 2=dir, 3=start), p2=value
 *                            POS X 1 100  = add 100 pulses
 *                            POS X 2 1    = set direction forward
 *                            POS E 3 0    = start E motion
 *
 * Note: SoftwareTrackers are standalone - not connected to pulse generators.
 * PcntTracker (Y/C) counts actual pulses via PCNT hardware.
 */
static esp_err_t handle_pos_test(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Handle POS (query all positions)
    if (cmd->axis == '\0' && cmd->param_count == 0 && !cmd->has_str_param) {
        // Get all positions - use real-time for RMT and LEDC axes (includes running pulses)
        int64_t pos_x = get_rmt_axis_position(0);  // X = RMT index 0
        int64_t pos_y = s_pcnt_tracker[0] ? s_pcnt_tracker[0]->getPosition() : 0;
        int64_t pos_z = get_rmt_axis_position(1);  // Z = RMT index 1
        int64_t pos_a = get_rmt_axis_position(2);  // A = RMT index 2
        int64_t pos_b = get_rmt_axis_position(3);  // B = RMT index 3
        int64_t pos_c = s_pcnt_tracker[1] ? s_pcnt_tracker[1]->getPosition() : 0;
        int64_t pos_d = get_ledc_axis_position();  // D uses LEDC with real-time counting
        int64_t pos_e = s_time_tracker ? s_time_tracker->getPosition() : 0;

        return format_ok_data(response, resp_len,
            "X:%lld Y:%lld Z:%lld A:%lld B:%lld C:%lld D:%lld E:%lld",
            (long long)pos_x, (long long)pos_y, (long long)pos_z,
            (long long)pos_a, (long long)pos_b, (long long)pos_c,
            (long long)pos_d, (long long)pos_e);
    }

    // Need axis for all other operations
    if (cmd->axis == '\0') {
        return format_error(response, resp_len, ERR_INVALID_AXIS, "Specify axis X/Y/Z/A/B/C/D/E");
    }

    char axis = cmd->axis;

    // Validate axis
    if (get_gpio_for_axis(axis) < 0 && !is_discrete_axis(axis)) {
        return format_error(response, resp_len, ERR_INVALID_AXIS, "Invalid axis");
    }

    // Get or create tracker
    IPositionTracker* tracker = get_or_create_tracker(axis);
    if (tracker == nullptr) {
        return format_error(response, resp_len, ERR_CONFIGURATION, "Tracker init failed");
    }

    // Handle POS <axis> <mode> <value> - special operations
    // mode: 1=add pulses, 2=set direction, 3=start motion (E only)
    if (cmd->param_count >= 2) {
        int mode = static_cast<int>(cmd->params[0]);
        int64_t value = static_cast<int64_t>(cmd->params[1]);

        switch (mode) {
            case 1: {  // ADD pulses (SoftwareTracker only)
                int sw_idx = get_sw_tracker_index(axis);
                if (sw_idx < 0 || s_sw_tracker[sw_idx] == nullptr) {
                    return format_error(response, resp_len, ERR_INVALID_AXIS, "ADD only for X/Z/A/B/D");
                }
                s_sw_tracker[sw_idx]->addPulses(value);
                int64_t pos = s_sw_tracker[sw_idx]->getPosition();
                ESP_LOGI(TAG, "Added %lld pulses to axis %c, position=%lld", (long long)value, axis, (long long)pos);
                return format_ok_data(response, resp_len, "%c ADD:%lld POS:%lld", axis, (long long)value, (long long)pos);
            }

            case 2: {  // DIR - set direction
                bool forward = (value != 0);
                tracker->setDirection(forward);
                ESP_LOGI(TAG, "Set axis %c direction to %s", axis, forward ? "FWD" : "REV");
                return format_ok_data(response, resp_len, "%c DIR:%s", axis, forward ? "FWD" : "REV");
            }

            case 3: {  // START motion (E axis TimeTracker only)
                if (!is_discrete_axis(axis)) {
                    return format_error(response, resp_len, ERR_INVALID_AXIS, "START only for E axis");
                }
                if (s_time_tracker) {
                    s_time_tracker->startMotion();
                    int64_t target = s_time_tracker->getPosition();
                    ESP_LOGI(TAG, "Started E axis motion, target=%lld", (long long)target);
                    return format_ok_data(response, resp_len, "E START TARGET:%lld", (long long)target);
                }
                return format_error(response, resp_len, ERR_CONFIGURATION, "TimeTracker not initialized");
            }

            default:
                return format_error(response, resp_len, ERR_INVALID_PARAMETER, "Mode: 1=add, 2=dir, 3=start");
        }
    }

    // Handle POS <axis> <value> (reset position) - single param
    if (cmd->param_count == 1) {
        int64_t new_pos = static_cast<int64_t>(cmd->params[0]);
        esp_err_t ret = tracker->reset(new_pos);
        if (ret != ESP_OK) {
            return format_error(response, resp_len, ERR_CONFIGURATION, "Reset failed");
        }
        int64_t pos = tracker->getPosition();
        ESP_LOGI(TAG, "Reset axis %c to %lld, actual=%lld", axis, (long long)new_pos, (long long)pos);
        return format_ok_data(response, resp_len, "%c RESET:%lld POS:%lld (%s)",
            axis, (long long)new_pos, (long long)pos, get_tracker_type(axis));
    }

    // Handle POS <axis> (query single axis)
    // With setPositionTracker() connected, tracker is updated in real-time
    // by the generator - no need to add getPulseCount()
    int64_t pos = tracker->getPosition();
    const char* type = get_tracker_type(axis);

    // Extra info for TimeTracker
    if (is_discrete_axis(axis) && s_time_tracker) {
        bool complete = s_time_tracker->isMotionComplete();
        return format_ok_data(response, resp_len, "%c POS:%lld (%s) COMPLETE:%s",
            axis, (long long)pos, type, complete ? "YES" : "NO");
    }

    // STORY-3-5-DEBUG: Extra debug info for D axis to diagnose position tracking issue
    if (is_ledc_axis(axis)) {
        int64_t tracker_pos, gen_count;
        bool running, has_tracker;
        get_ledc_debug_info(&tracker_pos, &gen_count, &running, &has_tracker);
        return format_ok_data(response, resp_len, "%c POS:%lld GEN:%lld RUN:%s TRK:%s (%s)",
            axis, (long long)tracker_pos, (long long)gen_count,
            running ? "Y" : "N", has_tracker ? "Y" : "N", type);
    }

    return format_ok_data(response, resp_len, "%c POS:%lld (%s)", axis, (long long)pos, type);
}

// STORY-3-2-TEST: Command registration - remove after hardware verification
extern "C" {

/**
 * @brief Register PULSE and MOVE test commands
 * STORY-3-2-TEST: Remove this function after hardware verification
 */
esp_err_t register_pulse_test_command(void)
{
    esp_err_t ret;

    // Register PULSE command (constant frequency via RMT loop)
    static const char* CMD_PULSE = "PULSE";
    CommandEntry pulse_entry = {
        .verb = CMD_PULSE,
        .handler = handle_pulse_test,
        .allowed_states = STATE_ANY
    };
    ret = cmd_executor_register(&pulse_entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered PULSE test command");
    } else {
        ESP_LOGE(TAG, "Failed to register PULSE: 0x%x", ret);
        return ret;
    }

    // Register MOVE command (trapezoidal profile via RmtPulseGenerator)
    static const char* CMD_MOVE_TEST = "MOVE";
    CommandEntry move_entry = {
        .verb = CMD_MOVE_TEST,
        .handler = handle_move_test,
        .allowed_states = STATE_ANY
    };
    ret = cmd_executor_register(&move_entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered MOVE test command");
    } else {
        ESP_LOGE(TAG, "Failed to register MOVE: 0x%x", ret);
        return ret;
    }

    // STORY-3-5-TEST: Register POS command (position tracker testing)
    static const char* CMD_POS_TEST = "POS";
    CommandEntry pos_entry = {
        .verb = CMD_POS_TEST,
        .handler = handle_pos_test,
        .allowed_states = STATE_ANY
    };
    ret = cmd_executor_register(&pos_entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered POS test command");
    } else {
        ESP_LOGE(TAG, "Failed to register POS: 0x%x", ret);
    }

    return ret;
}

/**
 * @brief Cleanup pulse test resources
 * STORY-3-2-TEST: Remove this function after hardware verification
 */
void cleanup_pulse_test_command(void)
{
    stop_all_pulses();
    stop_all_moves();

    // Cleanup RMT resources (X, Z, A, B)
    for (int i = 0; i < 4; i++) {
        // Cleanup PULSE resources
        if (s_encoder[i]) {
            rmt_del_encoder(s_encoder[i]);
            s_encoder[i] = nullptr;
        }
        if (s_rmt_channel[i]) {
            rmt_disable(s_rmt_channel[i]);
            rmt_del_channel(s_rmt_channel[i]);
            s_rmt_channel[i] = nullptr;
        }

        // Cleanup MOVE resources
        delete s_move_gen[i];
        s_move_gen[i] = nullptr;
    }

    // STORY-3-3-TEST: Cleanup MCPWM resources (Y, C)
    for (int i = 0; i < 2; i++) {
        delete s_mcpwm_gen[i];
        s_mcpwm_gen[i] = nullptr;
        s_mcpwm_running[i] = false;
        s_mcpwm_frequency[i] = 0.0f;
    }

    // STORY-3-4-TEST: Cleanup LEDC resources (D)
    cleanup_ledc_axis();

    // STORY-3-5-TEST: Cleanup position trackers
    for (int i = 0; i < 2; i++) {
        delete s_pcnt_tracker[i];
        s_pcnt_tracker[i] = nullptr;
    }
    for (int i = 0; i < 5; i++) {
        delete s_sw_tracker[i];
        s_sw_tracker[i] = nullptr;
    }
    delete s_time_tracker;
    s_time_tracker = nullptr;

    ESP_LOGI(TAG, "Cleaned up PULSE/MOVE/POS test resources");
}

} // extern "C"
