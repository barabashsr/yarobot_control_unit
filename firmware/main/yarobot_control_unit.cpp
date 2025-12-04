#include "esp_log.h"
#include "esp_heap_caps.h"
#include "config.h"

static const char* TAG = "main";

extern "C" void app_main(void)
{
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

    ESP_LOGI(TAG, "YaRobot Control Unit - Story 1.3 Configuration Headers Ready");
}
