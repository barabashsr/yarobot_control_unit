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

// STORY-3-2-TEST: Static RmtPulseGenerator instances for MOVE command
static RmtPulseGenerator* s_move_gen[4] = {nullptr, nullptr, nullptr, nullptr};

/**
 * @brief Get axis index (0-3 for X,Z,A,B)
 * STORY-3-2-TEST: Remove after hardware verification
 */
static int get_axis_index(char axis)
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
 * @brief Get GPIO for axis
 * STORY-3-2-TEST: Remove after hardware verification
 */
static int get_gpio_for_axis(char axis)
{
    switch (axis) {
        case 'X': case 'x': return GPIO_X_STEP;
        case 'Z': case 'z': return GPIO_Z_STEP;
        case 'A': case 'a': return GPIO_A_STEP;
        case 'B': case 'b': return GPIO_B_STEP;
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
 * @brief Stop pulse generation on axis
 * STORY-3-2-TEST: Remove after hardware verification
 */
static void stop_axis(int idx)
{
    if (idx >= 0 && idx < 4 && s_rmt_channel[idx] && s_running[idx]) {
        rmt_disable(s_rmt_channel[idx]);
        rmt_enable(s_rmt_channel[idx]);  // Re-enable for next use
        s_running[idx] = false;
        s_frequency[idx] = 0.0f;
        ESP_LOGI(TAG, "Stopped axis %d", idx);
    }
}

/**
 * @brief Stop all pulse generation
 * STORY-3-2-TEST: Remove after hardware verification
 */
static void stop_all_pulses(void)
{
    for (int i = 0; i < 4; i++) {
        stop_axis(i);
    }
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
 */
static esp_err_t handle_pulse_test(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // STORY-3-2-TEST: Handle PULSE STOP
    if (cmd->has_str_param && strcasecmp(cmd->str_param, "STOP") == 0) {
        stop_all_pulses();
        ESP_LOGI(TAG, "Stopped all pulse generation");
        return format_ok_data(response, resp_len, "STOPPED");
    }

    // STORY-3-2-TEST: Handle PULSE (query status)
    if (cmd->axis == '\0' && cmd->param_count == 0 && !cmd->has_str_param) {
        return format_ok_data(response, resp_len,
            "X:%s(%.0f) Z:%s(%.0f) A:%s(%.0f) B:%s(%.0f)",
            s_running[0] ? "ON" : "OFF", s_frequency[0],
            s_running[1] ? "ON" : "OFF", s_frequency[1],
            s_running[2] ? "ON" : "OFF", s_frequency[2],
            s_running[3] ? "ON" : "OFF", s_frequency[3]);
    }

    // STORY-3-2-TEST: Handle PULSE <axis> <freq>
    if (cmd->axis == '\0') {
        return format_error(response, resp_len, ERR_INVALID_AXIS, "Specify axis X/Z/A/B");
    }

    // Validate axis (only RMT axes: X, Z, A, B)
    char axis = cmd->axis;
    int idx = get_axis_index(axis);
    if (idx < 0) {
        return format_error(response, resp_len, ERR_INVALID_AXIS, "Only X/Z/A/B (RMT axes)");
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

    // Stop if already running
    if (s_running[idx]) {
        stop_axis(idx);
    }

    // Cleanup any MOVE generator on this axis (they share GPIO)
    cleanup_move_axis(idx);

    // Initialize RMT channel if needed
    int gpio = get_gpio_for_axis(axis);
    esp_err_t ret = init_rmt_channel(idx, gpio);
    if (ret != ESP_OK) {
        return format_error(response, resp_len, ERR_CONFIGURATION, "RMT init failed");
    }

    // Start pulses
    ret = start_pulses(idx, freq_hz);
    if (ret != ESP_OK) {
        return format_error(response, resp_len, ERR_CONFIGURATION, "Start failed");
    }

    return format_ok_data(response, resp_len, "%c FREQ:%.0f GPIO:%d",
        axis, s_frequency[idx], gpio);
}

// ============================================================================
// STORY-3-2-TEST: MOVE command - tests RmtPulseGenerator streaming mode
// ============================================================================

/**
 * @brief Stop all moves immediately
 * STORY-3-2-TEST: Remove after hardware verification
 */
static void stop_all_moves(void)
{
    for (int i = 0; i < 4; i++) {
        if (s_move_gen[i] && s_move_gen[i]->isRunning()) {
            s_move_gen[i]->stopImmediate();
            ESP_LOGI(TAG, "Stopped MOVE on axis %d", i);
        }
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
 */
static esp_err_t handle_move_test(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // STORY-3-2-TEST: Handle MOVE STOP
    if (cmd->has_str_param && strcasecmp(cmd->str_param, "STOP") == 0) {
        stop_all_moves();
        return format_ok_data(response, resp_len, "STOPPED");
    }

    // STORY-3-2-TEST: Handle MOVE (query status)
    if (cmd->axis == '\0' && cmd->param_count == 0 && !cmd->has_str_param) {
        return format_ok_data(response, resp_len,
            "X:%s(%lld) Z:%s(%lld) A:%s(%lld) B:%s(%lld)",
            s_move_gen[0] && s_move_gen[0]->isRunning() ? "RUN" : "IDLE",
            s_move_gen[0] ? s_move_gen[0]->getPulseCount() : 0LL,
            s_move_gen[1] && s_move_gen[1]->isRunning() ? "RUN" : "IDLE",
            s_move_gen[1] ? s_move_gen[1]->getPulseCount() : 0LL,
            s_move_gen[2] && s_move_gen[2]->isRunning() ? "RUN" : "IDLE",
            s_move_gen[2] ? s_move_gen[2]->getPulseCount() : 0LL,
            s_move_gen[3] && s_move_gen[3]->isRunning() ? "RUN" : "IDLE",
            s_move_gen[3] ? s_move_gen[3]->getPulseCount() : 0LL);
    }

    // STORY-3-2-TEST: Handle MOVE <axis> <pulses> <max_freq> <accel>
    if (cmd->axis == '\0') {
        return format_error(response, resp_len, ERR_INVALID_AXIS, "Specify axis X/Z/A/B");
    }

    char axis = cmd->axis;
    int idx = get_axis_index(axis);
    if (idx < 0) {
        return format_error(response, resp_len, ERR_INVALID_AXIS, "Only X/Z/A/B");
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
    if (max_freq < LIMIT_MIN_PULSE_FREQ_HZ || max_freq > LIMIT_MAX_PULSE_FREQ_HZ) {
        return format_error(response, resp_len, ERR_INVALID_PARAMETER,
            "Freq must be 1-500000 Hz");
    }
    if (accel <= 0) {
        return format_error(response, resp_len, ERR_INVALID_PARAMETER, "Accel must be > 0");
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
        int gpio = get_gpio_for_axis(axis);
        ESP_LOGI(TAG, "Creating RmtPulseGenerator for axis %c on GPIO %d", axis, gpio);
        s_move_gen[idx] = new RmtPulseGenerator(idx, gpio);
        esp_err_t ret = s_move_gen[idx]->init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init RmtPulseGenerator: %s", esp_err_to_name(ret));
            delete s_move_gen[idx];
            s_move_gen[idx] = nullptr;
            return format_error(response, resp_len, ERR_CONFIGURATION, "RMT init failed");
        }
    }

    // Start the move
    ESP_LOGI(TAG, "Starting MOVE: axis=%c pulses=%ld freq=%.0f accel=%.0f",
             axis, (long)pulses, max_freq, accel);
    esp_err_t ret = s_move_gen[idx]->startMove(pulses, max_freq, accel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "startMove failed: %s", esp_err_to_name(ret));
        return format_error(response, resp_len, ERR_CONFIGURATION, "Start failed");
    }

    return format_ok_data(response, resp_len, "%c PULSES:%ld FREQ:%.0f ACCEL:%.0f",
        axis, (long)pulses, max_freq, accel);
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

    ESP_LOGI(TAG, "Cleaned up PULSE/MOVE test resources");
}

} // extern "C"
