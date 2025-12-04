/**
 * @file test_utils.h
 * @brief Hardware test utilities for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note Provides test functions for I2C, SPI shift registers, and GPIO
 *       verification. Used by CMD_TEST command handler.
 */

#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup test_utils Hardware Test Utilities
 * @brief Functions for hardware verification testing
 * @{
 */

/** @brief Maximum length of test result string */
#define TEST_RESULT_MAX_LEN 128

/**
 * @brief Test results structure
 */
typedef struct {
    bool passed;
    char message[TEST_RESULT_MAX_LEN];
} test_result_t;

/**
 * @brief Test I2C buses and report found devices
 *
 * Scans I2C0 for MCP23017 devices (0x20, 0x21) and
 * I2C1 for OLED display (0x3C).
 *
 * @param[out] result Test result with formatted message
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note Response format: "OK I2C0:0x20,0x21 I2C1:0x3C" or "ERROR I2C0:NO_DEVICES"
 */
esp_err_t test_i2c(test_result_t *result);

/**
 * @brief Test shift register chain with patterns
 *
 * Writes test patterns (0xAAAAAA, 0x555555, 0x000000, 0xFFFFFF)
 * to the shift register chain.
 *
 * @param[out] result Test result with PASS/FAIL
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note Response format: "OK SR:PASS" or "ERROR SR:FAIL"
 */
esp_err_t test_sr(test_result_t *result);

/**
 * @brief Test GPIO pins and report states
 *
 * Briefly toggles STEP pins and reads E-STOP state.
 *
 * @param[out] result Test result with GPIO states
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note Response format: "OK GPIO:ESTOP=1 STEPS=0"
 */
esp_err_t test_gpio(test_result_t *result);

/**
 * @brief Run all hardware tests
 *
 * Runs I2C, SR, and GPIO tests sequentially.
 *
 * @param[out] result Combined test result
 *
 * @return esp_err_t ESP_OK if all tests pass
 */
esp_err_t test_all(test_result_t *result);

/** @} */ // end test_utils

#ifdef __cplusplus
}
#endif

#endif // TEST_UTILS_H
