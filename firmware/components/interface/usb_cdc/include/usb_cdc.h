/**
 * @file usb_cdc.h
 * @brief USB CDC ACM Serial Interface
 * @author YaRobot Team
 * @date 2025
 *
 * @details Provides USB CDC ACM serial communication for command/response interface.
 *          Uses ESP-IDF TinyUSB component for USB device stack.
 *
 * @note This component implements Story 2.1 USB CDC Serial Interface.
 *       - Device enumerates as USB CDC ACM
 *       - Line-based input (commands terminated by \n or \r\n)
 *       - All responses terminated with \r\n
 *       - RX/TX queues for task communication
 */

#ifndef USB_CDC_H
#define USB_CDC_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup usb_cdc USB CDC Interface
 * @brief USB CDC ACM serial communication
 * @{
 */

/**
 * @brief USB RX queue handle
 *
 * @details Queue for received command lines from USB.
 *          Each item is a null-terminated string (max LIMIT_CMD_MAX_LENGTH bytes).
 *          usb_rx_task pushes complete lines, cmd_executor_task consumes them.
 */
extern QueueHandle_t usb_rx_queue;

/**
 * @brief USB TX queue handle
 *
 * @details Queue for responses to send via USB.
 *          Each item is a null-terminated string (max LIMIT_RESPONSE_MAX_LENGTH bytes).
 *          Command handlers push responses, usb_tx_task sends them.
 */
extern QueueHandle_t usb_tx_queue;

/**
 * @brief Initialize USB CDC interface
 *
 * @details Initializes TinyUSB stack, configures CDC ACM descriptor,
 *          creates RX/TX queues, and registers USB event callbacks.
 *          Must be called before creating USB tasks.
 *
 * @return
 *      - ESP_OK: Initialization successful
 *      - ESP_ERR_NO_MEM: Failed to allocate queues
 *      - ESP_FAIL: TinyUSB initialization failed
 */
esp_err_t usb_cdc_init(void);

/**
 * @brief Send raw data via USB CDC
 *
 * @details Pushes data to TX queue for transmission.
 *          Non-blocking with configurable timeout.
 *
 * @param[in] data Data buffer to send
 * @param[in] len  Length of data in bytes
 *
 * @return
 *      - ESP_OK: Data queued successfully
 *      - ESP_ERR_INVALID_ARG: data is NULL or len is 0
 *      - ESP_ERR_TIMEOUT: Queue full, data not sent
 *      - ESP_FAIL: USB not connected
 */
esp_err_t usb_cdc_send(const char* data, size_t len);

/**
 * @brief Send a line via USB CDC with CRLF termination
 *
 * @details Sends the line followed by \r\n.
 *          If line already ends with \n, replaces with \r\n.
 *          Pushes to TX queue for transmission.
 *
 * @param[in] line Null-terminated string to send
 *
 * @return
 *      - ESP_OK: Line queued successfully
 *      - ESP_ERR_INVALID_ARG: line is NULL
 *      - ESP_ERR_TIMEOUT: Queue full, line not sent
 *      - ESP_FAIL: USB not connected
 */
esp_err_t usb_cdc_send_line(const char* line);

/**
 * @brief Check if USB CDC is connected
 *
 * @details Returns true if host has opened the serial port (DTR active).
 *
 * @return true if connected, false otherwise
 */
bool usb_cdc_is_connected(void);

/**
 * @brief Get USB CDC connection state change count
 *
 * @details Returns number of connect/disconnect events since init.
 *          Useful for detecting reconnections.
 *
 * @return Number of state changes
 */
uint32_t usb_cdc_get_state_changes(void);

/** @} */ // end usb_cdc

#ifdef __cplusplus
}
#endif

#endif // USB_CDC_H
