#include "esp_log.h"
#include "esp_private/gdma.h"
#include "hal/gdma_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

static const char* TAG = "GDMA_PROBE";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== GDMA Channel Availability Probe ===");
    ESP_LOGI(TAG, "ESP32-S3 has %d GDMA pairs (TX+RX each)", SOC_GDMA_PAIRS_PER_GROUP);

    gdma_channel_handle_t tx_channels[SOC_GDMA_PAIRS_PER_GROUP] = {nullptr};
    gdma_channel_handle_t rx_channels[SOC_GDMA_PAIRS_PER_GROUP] = {nullptr};

    // Try to allocate all TX channels
    ESP_LOGI(TAG, "\n--- Probing TX GDMA Channels ---");
    int tx_allocated = 0;
    for (int i = 0; i < SOC_GDMA_PAIRS_PER_GROUP; i++) {
        gdma_channel_alloc_config_t config = {
            .sibling_chan = nullptr,
            .direction = GDMA_CHANNEL_DIRECTION_TX,
            .flags = {
                .reserve_sibling = 0,
            }
        };
        esp_err_t ret = gdma_new_ahb_channel(&config, &tx_channels[i]);
        if (ret == ESP_OK) {
            int group_id = -1, pair_id = -1;
            gdma_get_group_channel_id(tx_channels[i], &group_id, &pair_id);
            ESP_LOGI(TAG, "  TX[%d]: ALLOCATED (group=%d, pair=%d)", i, group_id, pair_id);
            tx_allocated++;
        } else {
            ESP_LOGW(TAG, "  TX[%d]: FAILED - %s (0x%x)", i, esp_err_to_name(ret), ret);
            break;
        }
    }

    // Try to allocate all RX channels
    ESP_LOGI(TAG, "\n--- Probing RX GDMA Channels ---");
    int rx_allocated = 0;
    for (int i = 0; i < SOC_GDMA_PAIRS_PER_GROUP; i++) {
        gdma_channel_alloc_config_t config = {
            .sibling_chan = nullptr,
            .direction = GDMA_CHANNEL_DIRECTION_RX,
            .flags = {
                .reserve_sibling = 0,
            }
        };
        esp_err_t ret = gdma_new_ahb_channel(&config, &rx_channels[i]);
        if (ret == ESP_OK) {
            int group_id = -1, pair_id = -1;
            gdma_get_group_channel_id(rx_channels[i], &group_id, &pair_id);
            ESP_LOGI(TAG, "  RX[%d]: ALLOCATED (group=%d, pair=%d)", i, group_id, pair_id);
            rx_allocated++;
        } else {
            ESP_LOGW(TAG, "  RX[%d]: FAILED - %s (0x%x)", i, esp_err_to_name(ret), ret);
            break;
        }
    }

    ESP_LOGI(TAG, "\n=== RESULTS ===");
    ESP_LOGI(TAG, "TX Channels: %d/%d available", tx_allocated, SOC_GDMA_PAIRS_PER_GROUP);
    ESP_LOGI(TAG, "RX Channels: %d/%d available", rx_allocated, SOC_GDMA_PAIRS_PER_GROUP);
    ESP_LOGI(TAG, "TX in use by other peripherals: %d", SOC_GDMA_PAIRS_PER_GROUP - tx_allocated);
    ESP_LOGI(TAG, "RX in use by other peripherals: %d", SOC_GDMA_PAIRS_PER_GROUP - rx_allocated);

    // Cleanup
    ESP_LOGI(TAG, "\nCleaning up allocated channels...");
    for (int i = 0; i < tx_allocated; i++) {
        gdma_del_channel(tx_channels[i]);
    }
    for (int i = 0; i < rx_allocated; i++) {
        gdma_del_channel(rx_channels[i]);
    }

    ESP_LOGI(TAG, "Probe complete. Halting.");
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
