/**
 * @file usb_cdc.c
 * @brief USB CDC ACM Serial Interface Implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @details Implements USB CDC ACM serial communication using ESP-IDF TinyUSB.
 *          Provides line-based command input and formatted response output.
 *
 * @note Story 2.1 - USB CDC Serial Interface
 */

#include "usb_cdc.h"
#include "usb_cdc_private.h"
#include "config_limits.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <string.h>
#include <stdio.h>

static const char* TAG = "usb_cdc";

/** @brief Timeout for queue operations in ticks */
#define QUEUE_TIMEOUT_TICKS     pdMS_TO_TICKS(100)

/** @brief TX queue send timeout in ticks */
#define TX_QUEUE_TIMEOUT_TICKS  pdMS_TO_TICKS(50)

/* Queue handles - extern in header */
QueueHandle_t usb_rx_queue = NULL;
QueueHandle_t usb_tx_queue = NULL;

/* Global CDC state */
usb_cdc_state_t g_cdc_state = {
    .initialized = false,
    .connected = false,
    .state_changes = 0,
    .tx_mutex = NULL,
    .rx_line_buf = {0},
    .rx_line_pos = 0
};

/**
 * @brief CDC RX callback - called when data is received from USB host
 */
static void cdc_rx_callback(int itf, cdcacm_event_t* event)
{
    (void)itf;

    if (event->type != CDC_EVENT_RX) {
        return;
    }

    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
    size_t rx_size = 0;

    /* Read all available data */
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, sizeof(buf), &rx_size);
    if (ret != ESP_OK || rx_size == 0) {
        return;
    }

    /* Process received data - accumulate into line buffer */
    usb_cdc_process_rx(buf, rx_size);
}

/**
 * @brief CDC line state callback - called when DTR/RTS changes
 */
static void cdc_line_state_callback(int itf, cdcacm_event_t* event)
{
    (void)itf;

    if (event->type != CDC_EVENT_LINE_STATE_CHANGED) {
        return;
    }

    bool dtr = event->line_state_changed_data.dtr;
    bool rts = event->line_state_changed_data.rts;

    usb_cdc_line_state_changed(dtr, rts);
}

void usb_cdc_process_rx(const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];

        /* Check for line terminator */
        if (c == '\n' || c == '\r') {
            if (g_cdc_state.rx_line_pos > 0) {
                /* Null-terminate and copy to queue */
                g_cdc_state.rx_line_buf[g_cdc_state.rx_line_pos] = '\0';

                /* Allocate buffer for queue item */
                char* line_copy = pvPortMalloc(g_cdc_state.rx_line_pos + 1);
                if (line_copy != NULL) {
                    memcpy(line_copy, g_cdc_state.rx_line_buf, g_cdc_state.rx_line_pos + 1);

                    /* Push to RX queue - pointer to allocated string */
                    if (xQueueSend(usb_rx_queue, &line_copy, 0) != pdTRUE) {
                        /* Queue full - free the copy and log warning */
                        vPortFree(line_copy);
                        ESP_LOGW(TAG, "RX queue full, dropping line");
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to allocate RX line buffer");
                }
            }
            /* Reset line buffer */
            g_cdc_state.rx_line_pos = 0;

            /* Skip \n after \r (handle \r\n) */
            if (c == '\r' && i + 1 < len && data[i + 1] == '\n') {
                i++;
            }
        } else if (g_cdc_state.rx_line_pos < LIMIT_CMD_MAX_LENGTH - 1) {
            /* Accumulate character */
            g_cdc_state.rx_line_buf[g_cdc_state.rx_line_pos++] = c;
        } else {
            /* Buffer overflow - discard partial line */
            ESP_LOGW(TAG, "RX line buffer overflow, discarding");
            g_cdc_state.rx_line_pos = 0;
        }
    }
}

void usb_cdc_line_state_changed(bool dtr, bool rts)
{
    bool was_connected = g_cdc_state.connected;
    g_cdc_state.connected = dtr;

    if (was_connected != dtr) {
        g_cdc_state.state_changes++;

        if (dtr) {
            ESP_LOGI(TAG, "USB CDC connected (DTR=%d, RTS=%d)", dtr, rts);
        } else {
            ESP_LOGI(TAG, "USB CDC disconnected (DTR=%d, RTS=%d)", dtr, rts);
            /* Clear RX buffer on disconnect */
            g_cdc_state.rx_line_pos = 0;
        }
    }
}

esp_err_t usb_cdc_init(void)
{
    if (g_cdc_state.initialized) {
        ESP_LOGW(TAG, "USB CDC already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing USB CDC interface");

    /* Create RX queue - stores pointers to allocated strings */
    usb_rx_queue = xQueueCreate(LIMIT_COMMAND_QUEUE_DEPTH, sizeof(char*));
    if (usb_rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return ESP_ERR_NO_MEM;
    }

    /* Create TX queue - stores pointers to allocated strings */
    usb_tx_queue = xQueueCreate(LIMIT_RESPONSE_QUEUE_DEPTH, sizeof(char*));
    if (usb_tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        vQueueDelete(usb_rx_queue);
        usb_rx_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Create TX mutex */
    g_cdc_state.tx_mutex = xSemaphoreCreateMutex();
    if (g_cdc_state.tx_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create TX mutex");
        vQueueDelete(usb_rx_queue);
        vQueueDelete(usb_tx_queue);
        usb_rx_queue = NULL;
        usb_tx_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Initialize TinyUSB driver */
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,  /* Use default ESP device descriptor */
        .string_descriptor = NULL,  /* Use default strings */
        .string_descriptor_count = 0,
        .external_phy = false,
        .configuration_descriptor = NULL,  /* Use default */
        .self_powered = false,
        .vbus_monitor_io = -1,  /* Not monitoring VBUS */
    };

    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB driver install failed: %s", esp_err_to_name(ret));
        vQueueDelete(usb_rx_queue);
        vQueueDelete(usb_tx_queue);
        vSemaphoreDelete(g_cdc_state.tx_mutex);
        usb_rx_queue = NULL;
        usb_tx_queue = NULL;
        g_cdc_state.tx_mutex = NULL;
        return ret;
    }

    /* Configure CDC ACM */
    tinyusb_config_cdcacm_t cdc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 256,
        .callback_rx = &cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &cdc_line_state_callback,
        .callback_line_coding_changed = NULL,
    };

    ret = tusb_cdc_acm_init(&cdc_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CDC ACM init failed: %s", esp_err_to_name(ret));
        /* TinyUSB driver is installed but we can't use CDC */
        return ret;
    }

    g_cdc_state.initialized = true;
    ESP_LOGI(TAG, "USB CDC initialized successfully");

    return ESP_OK;
}

esp_err_t usb_cdc_send(const char* data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_cdc_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!g_cdc_state.connected) {
        /* Not connected, but don't fail - just drop silently */
        return ESP_OK;
    }

    if (xSemaphoreTake(g_cdc_state.tx_mutex, TX_QUEUE_TIMEOUT_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* Write directly to CDC */
    size_t written = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (const uint8_t*)data, len);
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, TX_QUEUE_TIMEOUT_TICKS);

    xSemaphoreGive(g_cdc_state.tx_mutex);

    return (written == len) ? ESP_OK : ESP_FAIL;
}

esp_err_t usb_cdc_send_line(const char* line)
{
    if (line == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_cdc_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t len = strlen(line);

    /* Allocate buffer for line + \r\n + null */
    char* buf = pvPortMalloc(len + 3);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Copy line, ensuring CRLF termination */
    memcpy(buf, line, len);

    /* Remove existing line endings if present */
    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
        len--;
    }

    /* Add CRLF */
    buf[len] = '\r';
    buf[len + 1] = '\n';
    buf[len + 2] = '\0';

    /* Push to TX queue for usb_tx_task to send */
    if (xQueueSend(usb_tx_queue, &buf, TX_QUEUE_TIMEOUT_TICKS) != pdTRUE) {
        vPortFree(buf);
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

bool usb_cdc_is_connected(void)
{
    return g_cdc_state.connected;
}

uint32_t usb_cdc_get_state_changes(void)
{
    return g_cdc_state.state_changes;
}
