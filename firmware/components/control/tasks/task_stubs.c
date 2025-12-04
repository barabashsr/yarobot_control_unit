/**
 * @file task_stubs.c
 * @brief FreeRTOS task stub implementations
 * @author YaRobot Team
 * @date 2025
 *
 * @note Stub implementations for all control system tasks.
 *       Each stub logs its name and core ID, then enters an infinite loop.
 *       Real implementations will be added in later epics.
 */

#include "task_defs.h"
#include "usb_cdc.h"
#include "config_limits.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "test_utils.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

static const char* TAG = "tasks";

/** @brief Command input buffer size */
#define CMD_BUF_SIZE 64

void safety_monitor_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Task %s started on core %d", pcTaskGetName(NULL), xPortGetCoreID());

    for (;;) {
        // Placeholder - real implementation in Epic 4
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void usb_rx_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Task %s started on core %d", pcTaskGetName(NULL), xPortGetCoreID());

    /*
     * USB CDC RX is handled via callbacks in usb_cdc.c.
     * The cdc_rx_callback() is called by TinyUSB when data arrives,
     * which accumulates lines and pushes them to usb_rx_queue.
     *
     * This task monitors the TinyUSB stack by periodically calling
     * tud_task() to process USB events if needed (some configurations
     * require this, though ESP-IDF typically handles it internally).
     *
     * For now, we just monitor connection state and yield.
     */
    static bool last_connected = false;

    for (;;) {
        bool connected = usb_cdc_is_connected();
        if (connected != last_connected) {
            ESP_LOGI(TAG, "USB CDC connection state: %s",
                     connected ? "connected" : "disconnected");
            last_connected = connected;
        }

        /* Yield to other tasks - USB RX is interrupt/callback driven */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void usb_tx_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Task %s started on core %d", pcTaskGetName(NULL), xPortGetCoreID());

    char* tx_line = NULL;

    for (;;) {
        /* Wait for data in TX queue */
        if (xQueueReceive(usb_tx_queue, &tx_line, portMAX_DELAY) == pdTRUE) {
            if (tx_line != NULL) {
                /* Send via TinyUSB CDC */
                size_t len = strlen(tx_line);
                if (usb_cdc_is_connected() && len > 0) {
                    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0,
                                               (const uint8_t*)tx_line, len);
                    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0,
                                               pdMS_TO_TICKS(50));
                }
                /* Free the allocated string */
                vPortFree(tx_line);
                tx_line = NULL;
            }
        }
    }
}

/**
 * @brief Send a response via USB CDC TX queue
 *
 * @param response Response string to send
 */
static void send_response(const char* response)
{
    /* Send via USB CDC if available */
    usb_cdc_send_line(response);

    /* Also echo to printf for USB Serial JTAG console */
    printf("%s\r\n", response);
}

/**
 * @brief Process a command string
 *
 * @param cmd Command string (null-terminated, may include newline)
 */
static void process_command(char* cmd)
{
    // Strip trailing whitespace
    size_t len = strlen(cmd);
    while (len > 0 && (cmd[len-1] == '\n' || cmd[len-1] == '\r' || cmd[len-1] == ' ')) {
        cmd[--len] = '\0';
    }

    // Skip empty lines
    if (len == 0) {
        return;
    }

    // Convert to uppercase for comparison
    char cmd_upper[CMD_BUF_SIZE];
    for (size_t i = 0; i <= len && i < CMD_BUF_SIZE - 1; i++) {
        cmd_upper[i] = toupper((unsigned char)cmd[i]);
    }
    cmd_upper[CMD_BUF_SIZE - 1] = '\0';

    test_result_t result;

    if (strcmp(cmd_upper, "TEST I2C") == 0) {
        test_i2c(&result);
        send_response(result.message);
    } else if (strcmp(cmd_upper, "TEST SR") == 0) {
        test_sr(&result);
        send_response(result.message);
    } else if (strcmp(cmd_upper, "TEST GPIO") == 0) {
        test_gpio(&result);
        send_response(result.message);
    } else if (strcmp(cmd_upper, "DIAG") == 0 || strcmp(cmd_upper, "TEST ALL") == 0) {
        test_all(&result);
        send_response(result.message);
    } else if (strcmp(cmd_upper, "HELP") == 0 || strcmp(cmd_upper, "?") == 0) {
        send_response("OK Commands: TEST I2C, TEST SR, TEST GPIO, DIAG, HELP");
    } else if (strncmp(cmd_upper, "ECHO ", 5) == 0) {
        /* Echo command - return the text after ECHO */
        char echo_response[LIMIT_CMD_MAX_LENGTH + 4];
        snprintf(echo_response, sizeof(echo_response), "OK %s", cmd + 5);
        send_response(echo_response);
    } else {
        ESP_LOGD(TAG, "Unknown command: %s", cmd);
        send_response("ERROR E001 Unknown command");
    }
}

void cmd_executor_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Task %s started on core %d", pcTaskGetName(NULL), xPortGetCoreID());

    // Disable line buffering on stdin for character-by-character input
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    char cmd_buf[CMD_BUF_SIZE];
    size_t cmd_pos = 0;
    char* usb_cdc_line = NULL;

    for (;;) {
        /*
         * Check USB CDC RX queue first (non-blocking).
         * This is the preferred path for TinyUSB CDC connections.
         */
        if (usb_rx_queue != NULL &&
            xQueueReceive(usb_rx_queue, &usb_cdc_line, 0) == pdTRUE) {
            if (usb_cdc_line != NULL) {
                ESP_LOGD(TAG, "CDC RX: %s", usb_cdc_line);
                process_command(usb_cdc_line);
                vPortFree(usb_cdc_line);
                usb_cdc_line = NULL;
            }
        }

        /*
         * Also check stdin (USB Serial JTAG) for backward compatibility.
         * This allows commands from idf.py monitor.
         */
        int c = getchar();
        if (c != EOF) {
            if (c == '\n' || c == '\r') {
                // Echo newline and process command
                putchar('\n');
                fflush(stdout);
                cmd_buf[cmd_pos] = '\0';
                if (cmd_pos > 0) {
                    process_command(cmd_buf);
                }
                cmd_pos = 0;
            } else if (c == '\b' || c == 0x7F) {
                // Backspace handling
                if (cmd_pos > 0) {
                    cmd_pos--;
                    printf("\b \b");  // Erase character on terminal
                    fflush(stdout);
                }
            } else if (cmd_pos < CMD_BUF_SIZE - 1) {
                // Echo character and store
                cmd_buf[cmd_pos++] = (char)c;
                putchar(c);
                fflush(stdout);
            }
        } else {
            // No input from either source, yield briefly
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void i2c_monitor_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Task %s started on core %d", pcTaskGetName(NULL), xPortGetCoreID());

    for (;;) {
        // Placeholder - real implementation in Epic 4
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void idle_monitor_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Task %s started on core %d", pcTaskGetName(NULL), xPortGetCoreID());

    for (;;) {
        // Placeholder - diagnostics task
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void motion_task(void* arg)
{
    int axis = (int)(intptr_t)arg;
    ESP_LOGI(TAG, "Task %s (axis %d) started on core %d",
             pcTaskGetName(NULL), axis, xPortGetCoreID());

    for (;;) {
        // Placeholder - real implementation in Epic 3
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void display_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Task %s started on core %d", pcTaskGetName(NULL), xPortGetCoreID());

    for (;;) {
        // Placeholder - real implementation in Epic 5
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
