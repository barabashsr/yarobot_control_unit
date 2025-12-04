#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "task_defs.h"
#include <stdio.h>

static const char* TAG = "main";

extern "C" void app_main(void)
{
    // Longer delay to allow USB CDC to enumerate with host
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Hello from app_main");

    // Verify PSRAM is available
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram_size > 0) {
        ESP_LOGI(TAG, "PSRAM detected: %zu bytes (%.1f MB)", psram_size, psram_size / (1024.0 * 1024.0));
    } else {
        ESP_LOGW(TAG, "PSRAM not detected!");
    }

    // Verify config headers are accessible (Story 1.3)
    ESP_LOGI(TAG, "Firmware: %s v%s", FIRMWARE_NAME, FIRMWARE_VERSION_STRING);
    ESP_LOGI(TAG, "Axes: %d (Servos:%d, Steppers:%d, Discrete:%d)",
             LIMIT_NUM_AXES, LIMIT_NUM_SERVOS, LIMIT_NUM_STEPPERS, LIMIT_NUM_DISCRETE);
    ESP_LOGI(TAG, "GPIO X_STEP: %d, I2C MCP0: 0x%02X", GPIO_X_STEP, I2C_ADDR_MCP23017_0);

    // =========================================================================
    // FreeRTOS Task Creation (Story 1.5)
    // =========================================================================
    ESP_LOGI(TAG, "Creating FreeRTOS tasks...");

    // Core 0 tasks: Communication, safety, coordination
    xTaskCreatePinnedToCore(safety_monitor_task, "safety", STACK_SAFETY_TASK,
                            NULL, 24, NULL, 0);
    xTaskCreatePinnedToCore(usb_rx_task, "usb_rx", STACK_USB_RX_TASK,
                            NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(usb_tx_task, "usb_tx", STACK_USB_TX_TASK,
                            NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(cmd_executor_task, "cmd_exec", STACK_CMD_EXECUTOR_TASK,
                            NULL, 12, NULL, 0);
    xTaskCreatePinnedToCore(i2c_monitor_task, "i2c_mon", STACK_I2C_MONITOR_TASK,
                            NULL, 8, NULL, 0);
    xTaskCreatePinnedToCore(idle_monitor_task, "idle_mon", STACK_IDLE_MONITOR_TASK,
                            NULL, 4, NULL, 0);

    // Core 1 tasks: Motion control (time-critical, 8 axes)
    for (int axis = 0; axis < LIMIT_NUM_AXES; axis++) {
        char name[16];
        snprintf(name, sizeof(name), "motion_%c", 'X' + axis);
        xTaskCreatePinnedToCore(motion_task, name, STACK_MOTION_TASK,
                                (void*)(intptr_t)axis, 15, NULL, 1);
    }
    xTaskCreatePinnedToCore(display_task, "display", STACK_DISPLAY_TASK,
                            NULL, 5, NULL, 1);

    // Send boot notification (AC7)
    printf("EVENT BOOT V1.0.0 AXES:8 STATE:IDLE\n");

    ESP_LOGI(TAG, "YaRobot Control Unit - All tasks started");
}
