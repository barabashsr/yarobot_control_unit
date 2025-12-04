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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "test_utils.h"
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

    for (;;) {
        // Placeholder - real implementation in Epic 2
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void usb_tx_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Task %s started on core %d", pcTaskGetName(NULL), xPortGetCoreID());

    for (;;) {
        // Placeholder - real implementation in Epic 2
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
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
        printf("%s\n", result.message);
    } else if (strcmp(cmd_upper, "TEST SR") == 0) {
        test_sr(&result);
        printf("%s\n", result.message);
    } else if (strcmp(cmd_upper, "TEST GPIO") == 0) {
        test_gpio(&result);
        printf("%s\n", result.message);
    } else if (strcmp(cmd_upper, "DIAG") == 0 || strcmp(cmd_upper, "TEST ALL") == 0) {
        test_all(&result);
        printf("%s\n", result.message);
    } else if (strcmp(cmd_upper, "HELP") == 0 || strcmp(cmd_upper, "?") == 0) {
        printf("Commands: TEST I2C, TEST SR, TEST GPIO, DIAG, HELP\n");
    } else {
        ESP_LOGD(TAG, "Unknown command: %s", cmd);
        printf("ERROR UNKNOWN_CMD\n");
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

    for (;;) {
        // Read from stdin (USB CDC)
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
            // No input available, yield
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
