/**
 * @file digital_io.h
 * @brief Digital I/O Manager API
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-7: General-Purpose Digital I/O
 *       Manages 4 digital inputs (DIN0-3) via MCP23017 #1
 *       and 8 digital outputs (DOUT0-7) via shift register chain.
 */

#ifndef DIGITAL_IO_H
#define DIGITAL_IO_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup digital_io Digital I/O Manager
 * @brief General-purpose digital input/output control
 * @{
 */

/**
 * @brief Number of digital inputs (DIN0-3)
 */
#define DIO_NUM_INPUTS      4

/**
 * @brief Number of digital outputs (DOUT0-7)
 */
#define DIO_NUM_OUTPUTS     8

/**
 * @brief Maximum alias name length
 */
#define DIO_ALIAS_MAX_LEN   16

/**
 * @brief Maximum number of aliases
 */
#define DIO_MAX_ALIASES     16

/**
 * @brief I/O type for alias resolution
 */
typedef enum {
    DIO_TYPE_INPUT = 0,
    DIO_TYPE_OUTPUT = 1
} dio_type_t;

/**
 * @brief Alias entry structure
 */
typedef struct {
    char name[DIO_ALIAS_MAX_LEN];  /**< Alias name (null-terminated) */
    dio_type_t type;               /**< Input or output */
    uint8_t pin;                   /**< Pin number (0-3 for inputs, 0-7 for outputs) */
} dio_alias_t;

/**
 * @brief Initialize the digital I/O manager
 *
 * Initializes input debounce state, output state tracking,
 * and loads default aliases.
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note Must be called after sr_init() and mcp23017 initialization
 */
esp_err_t digital_io_init(void);

/**
 * @brief Check if digital I/O manager is initialized
 *
 * @return true if initialized, false otherwise
 */
bool digital_io_is_initialized(void);

/**
 * @brief Read a single debounced digital input
 *
 * @param[in] pin Input pin number (0-3)
 * @param[out] value Pointer to store the debounced input state
 *
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if pin > 3
 */
esp_err_t digital_io_read_input(uint8_t pin, bool* value);

/**
 * @brief Read all digital inputs as a bitmap
 *
 * @return uint8_t 4-bit bitmap with DIN3:DIN2:DIN1:DIN0 (MSB to LSB, bits 3-0)
 */
uint8_t digital_io_read_all_inputs(void);

/**
 * @brief Set a single digital output
 *
 * @param[in] pin Output pin number (0-7)
 * @param[in] level true for HIGH, false for LOW
 *
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if pin > 7
 *
 * @note Calls sr_set_gp_output() and sr_update() internally
 */
esp_err_t digital_io_set_output(uint8_t pin, bool level);

/**
 * @brief Get all digital outputs as a bitmap
 *
 * @return uint8_t 8-bit bitmap with DOUT7:...:DOUT0 (MSB to LSB)
 */
uint8_t digital_io_get_all_outputs(void);

/**
 * @brief Resolve an alias to pin type and number
 *
 * @param[in] name Alias name (case-insensitive)
 * @param[out] type Pointer to store I/O type (input or output)
 * @param[out] pin Pointer to store pin number
 *
 * @return esp_err_t ESP_OK if alias found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t digital_io_resolve_alias(const char* name, dio_type_t* type, uint8_t* pin);

/**
 * @brief Get alias name for a pin (if configured)
 *
 * @param[in] type I/O type (input or output)
 * @param[in] pin Pin number
 *
 * @return const char* Alias name or NULL if no alias configured
 */
const char* digital_io_get_alias(dio_type_t type, uint8_t pin);

/**
 * @brief Update debounce state (called periodically)
 *
 * Reads raw input states from MCP23017 and updates debounced values.
 * Should be called from polling task at TIMING_I2C_POLL_MS interval.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t digital_io_update_debounce(void);

/**
 * @brief Enable/disable event mode for an input
 *
 * When enabled, input state changes will publish EVENT DIN events.
 *
 * @param[in] pin Input pin number (0-3)
 * @param[in] enabled true to enable events, false to disable
 *
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if pin > 3
 */
esp_err_t digital_io_set_event_mode(uint8_t pin, bool enabled);

/**
 * @brief Check if event mode is enabled for an input
 *
 * @param[in] pin Input pin number (0-3)
 *
 * @return true if event mode enabled, false otherwise
 */
bool digital_io_get_event_mode(uint8_t pin);

/** @} */ // end digital_io

#ifdef __cplusplus
}
#endif

#endif // DIGITAL_IO_H
