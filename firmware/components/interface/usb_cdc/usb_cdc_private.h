/**
 * @file usb_cdc_private.h
 * @brief USB CDC internal types and declarations
 * @author YaRobot Team
 * @date 2025
 *
 * @note Internal header - not for external use
 */

#ifndef USB_CDC_PRIVATE_H
#define USB_CDC_PRIVATE_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief USB CDC internal state
 */
typedef struct {
    bool initialized;           /**< True if usb_cdc_init() succeeded */
    volatile bool connected;    /**< True if DTR is active (host connected) */
    volatile uint32_t state_changes; /**< Connect/disconnect event counter */
    SemaphoreHandle_t tx_mutex; /**< Protects TX operations */
    char rx_line_buf[256];      /**< Line accumulation buffer */
    size_t rx_line_pos;         /**< Current position in line buffer */
} usb_cdc_state_t;

/**
 * @brief Global CDC state (internal use only)
 */
extern usb_cdc_state_t g_cdc_state;

/**
 * @brief Process received data from USB
 *
 * @param[in] data Received data buffer
 * @param[in] len  Length of received data
 */
void usb_cdc_process_rx(const uint8_t* data, size_t len);

/**
 * @brief Handle line state change (DTR/RTS)
 *
 * @param[in] dtr DTR line state
 * @param[in] rts RTS line state
 */
void usb_cdc_line_state_changed(bool dtr, bool rts);

#ifdef __cplusplus
}
#endif

#endif // USB_CDC_PRIVATE_H
