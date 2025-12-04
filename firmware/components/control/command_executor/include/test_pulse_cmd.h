/**
 * @file test_pulse_cmd.h
 * @brief Test command handler for RMT pulse generator verification
 * @author YaRobot Team
 * @date 2025
 *
 * ============================================================================
 * STORY-3-2-TEST: Remove this entire file after hardware verification
 * ============================================================================
 */

#ifndef TEST_PULSE_CMD_H
#define TEST_PULSE_CMD_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register PULSE test command
 * STORY-3-2-TEST: Remove after hardware verification
 *
 * Usage via USB serial:
 *   PULSE X 10000   - Generate 10 kHz pulses on X-axis GPIO
 *   PULSE Z 50000   - Generate 50 kHz pulses on Z-axis GPIO
 *   PULSE A 100000  - Generate 100 kHz pulses on A-axis GPIO
 *   PULSE B 200000  - Generate 200 kHz pulses on B-axis GPIO
 *   PULSE STOP      - Stop all pulse generation
 *   PULSE           - Query current pulse state
 *
 * @return ESP_OK on success
 */
esp_err_t register_pulse_test_command(void);

/**
 * @brief Cleanup pulse test resources
 * STORY-3-2-TEST: Remove after hardware verification
 */
void cleanup_pulse_test_command(void);

#ifdef __cplusplus
}
#endif

#endif // TEST_PULSE_CMD_H
