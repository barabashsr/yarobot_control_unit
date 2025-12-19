/**
 * @file mcp23017_wrapper.h
 * @brief MCP23017 I/O Expander Wrapper API
 * @author YaRobot Team
 * @date 2025
 *
 * @note Provides high-level interface to both MCP23017 I/O expanders:
 *       - MCP0 (0x20): 16 limit switch inputs
 *       - MCP1 (0x21): 7 ALARM_INPUT + 5 InPos + 4 spare inputs
 *
 * @see config_i2c.h for pin mappings
 * @see config_gpio.h for interrupt GPIO assignments
 */

#ifndef MCP23017_WRAPPER_H
#define MCP23017_WRAPPER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup mcp23017_wrapper MCP23017 Wrapper Driver
 * @brief High-level interface for MCP23017 I/O expanders
 * @{
 */

/**
 * @brief MCP23017 device index
 */
typedef enum {
    MCP_DEVICE_0 = 0,   /**< MCP23017 #0 at 0x20 (limit switches) */
    MCP_DEVICE_1 = 1,   /**< MCP23017 #1 at 0x21 (ALARM_INPUT + InPos) */
    MCP_DEVICE_COUNT
} mcp_device_t;

/**
 * @brief MCP23017 port index
 */
typedef enum {
    MCP_PORT_A = 0,     /**< Port A (pins 0-7) */
    MCP_PORT_B = 1      /**< Port B (pins 8-15) */
} mcp_port_t;

/**
 * @brief Interrupt notification bits for safety_monitor_task
 */
#define MCP_NOTIFY_MCP0_INTA    (1 << 0)    /**< MCP0 Port A interrupt (X-A limits) */
#define MCP_NOTIFY_MCP0_INTB    (1 << 1)    /**< MCP0 Port B interrupt (B-E limits) */
#define MCP_NOTIFY_MCP1_INTA    (1 << 2)    /**< MCP1 Port A interrupt (ALARM_INPUT) */
#define MCP_NOTIFY_MCP1_INTB    (1 << 3)    /**< MCP1 Port B interrupt (InPos) */
#define MCP_NOTIFY_LIMIT        (MCP_NOTIFY_MCP0_INTA | MCP_NOTIFY_MCP0_INTB)
#define MCP_NOTIFY_ALARM        MCP_NOTIFY_MCP1_INTA

/**
 * @brief Initialize both MCP23017 expanders
 *
 * Performs the following:
 * - Creates I2C bus on I2C_PORT at I2C_FREQ_HZ (400kHz)
 * - Initializes MCP23017 #0 (0x20) and #1 (0x21)
 * - Configures all 32 pins as inputs (IODIR = 0xFF)
 * - Enables pull-ups on all pins (GPPU = 0xFF)
 *
 * @return esp_err_t
 *         - ESP_OK: Both devices initialized successfully
 *         - ESP_ERR_NOT_FOUND: One or both devices not responding
 *         - ESP_ERR_INVALID_STATE: Already initialized
 *         - ESP_FAIL: I2C bus creation failed
 */
esp_err_t mcp23017_wrapper_init(void);

/**
 * @brief Configure interrupt-on-change for MCP0 (limit switches)
 *
 * Enables interrupt on any pin change for both Port A and Port B.
 * Interrupt mode: compare against previous value (not DEFVAL).
 *
 * @return esp_err_t
 *         - ESP_OK: Interrupts configured successfully
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t mcp23017_wrapper_config_mcp0_interrupts(void);

/**
 * @brief Configure interrupt-on-change for MCP1 Port A (ALARM_INPUT)
 *
 * Enables interrupt on ALARM_INPUT pins (GPA0-GPA6, 7 pins).
 * GPA7 is spare and excluded from interrupt mask.
 *
 * @return esp_err_t
 *         - ESP_OK: Interrupts configured successfully
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t mcp23017_wrapper_config_mcp1_interrupts(void);

/**
 * @brief Register task for interrupt notifications
 *
 * The registered task will receive FreeRTOS task notifications
 * when MCP23017 interrupt lines assert. Use MCP_NOTIFY_* bits
 * to identify the interrupt source.
 *
 * @param[in] task_handle Handle of task to notify (e.g., safety_monitor_task)
 *
 * @return esp_err_t
 *         - ESP_OK: Task registered successfully
 *         - ESP_ERR_INVALID_ARG: task_handle is NULL
 */
esp_err_t mcp23017_wrapper_register_notify_task(TaskHandle_t task_handle);

/**
 * @brief Install GPIO ISRs for MCP interrupt lines
 *
 * Configures ESP32 GPIO pins for MCP interrupt lines:
 * - GPIO_MCP0_INTA, GPIO_MCP0_INTB (limit switch interrupts)
 * - GPIO_MCP1_INTA, GPIO_MCP1_INTB (ALARM_INPUT, InPos interrupts)
 *
 * ISR handlers notify the registered task via xTaskNotifyFromISR().
 *
 * @return esp_err_t
 *         - ESP_OK: ISRs installed successfully
 *         - ESP_ERR_INVALID_STATE: No task registered or not initialized
 */
esp_err_t mcp23017_wrapper_install_isr(void);

/**
 * @brief Read a port from specified MCP23017 device
 *
 * Thread-safe read with timeout protection.
 *
 * @param[in] device Device index (MCP_DEVICE_0 or MCP_DEVICE_1)
 * @param[in] port Port to read (MCP_PORT_A or MCP_PORT_B)
 * @param[out] value Pointer to store 8-bit port value
 *
 * @return esp_err_t
 *         - ESP_OK: Read successful
 *         - ESP_ERR_INVALID_ARG: Invalid device/port or NULL value
 *         - ESP_ERR_INVALID_STATE: Not initialized
 *         - ESP_ERR_TIMEOUT: Read exceeded TIMING_I2C_TIMEOUT_MS
 */
esp_err_t mcp23017_wrapper_read_port(mcp_device_t device, mcp_port_t port, uint8_t *value);

/**
 * @brief Read both ports from specified MCP23017 device
 *
 * Thread-safe read of 16-bit value (Port A = low byte, Port B = high byte).
 *
 * @param[in] device Device index (MCP_DEVICE_0 or MCP_DEVICE_1)
 * @param[out] value Pointer to store 16-bit value
 *
 * @return esp_err_t
 *         - ESP_OK: Read successful
 *         - ESP_ERR_INVALID_ARG: Invalid device or NULL value
 *         - ESP_ERR_INVALID_STATE: Not initialized
 *         - ESP_ERR_TIMEOUT: Read exceeded timeout
 */
esp_err_t mcp23017_wrapper_read_all(mcp_device_t device, uint16_t *value);

/**
 * @brief Get interrupt flags from specified device
 *
 * Returns which pins triggered the interrupt without clearing.
 *
 * @param[in] device Device index
 * @param[out] flags Pointer to store 16-bit interrupt flags
 *
 * @return esp_err_t
 *         - ESP_OK: Read successful
 *         - ESP_ERR_INVALID_ARG: Invalid device or NULL flags
 */
esp_err_t mcp23017_wrapper_get_int_flags(mcp_device_t device, uint16_t *flags);

/**
 * @brief Get interrupt capture and clear interrupt
 *
 * Returns the pin states captured at interrupt time and clears
 * the interrupt flag.
 *
 * @param[in] device Device index
 * @param[out] capture Pointer to store 16-bit captured values
 *
 * @return esp_err_t
 *         - ESP_OK: Read successful
 *         - ESP_ERR_INVALID_ARG: Invalid device or NULL capture
 */
esp_err_t mcp23017_wrapper_get_int_capture(mcp_device_t device, uint16_t *capture);

/**
 * @brief Check if wrapper is initialized
 *
 * @return true if initialized, false otherwise
 */
bool mcp23017_wrapper_is_initialized(void);

/**
 * @brief Deinitialize MCP23017 wrapper
 *
 * Releases resources, removes ISRs, and deletes I2C bus.
 *
 * @return esp_err_t
 *         - ESP_OK: Deinitialized successfully
 */
esp_err_t mcp23017_wrapper_deinit(void);

/** @} */ // end mcp23017_wrapper

#ifdef __cplusplus
}
#endif

#endif // MCP23017_WRAPPER_H
