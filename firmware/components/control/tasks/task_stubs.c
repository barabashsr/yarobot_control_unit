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
#include <stdint.h>

static const char* TAG = "tasks";

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

void cmd_executor_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Task %s started on core %d", pcTaskGetName(NULL), xPortGetCoreID());

    for (;;) {
        // Placeholder - real implementation in Epic 2
        vTaskDelay(pdMS_TO_TICKS(1000));
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
