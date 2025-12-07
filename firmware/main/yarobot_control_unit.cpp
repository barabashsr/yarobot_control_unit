#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "task_defs.h"
#include "usb_cdc.h"
#include "i2c_hal.h"
#include "spi_hal.h"
#include "gpio_hal.h"
#include "config_gpio.h"
#include "config_i2c.h"
#include "config_oled.h"
#include "config_sr.h"
#include "tpic6b595.h"
#include <stdio.h>
#include <string.h>

static const char* TAG = "main";

/**
 * @brief Initialize OLED display with SSD1306 commands
 *
 * Sends initialization sequence and displays "BOOT OK" message.
 *
 * @return esp_err_t ESP_OK on success
 */
static esp_err_t oled_init_and_display(void)
{
    // SSD1306 initialization sequence
    static const uint8_t init_cmds[] = {
        0x00,       // Command stream
        0xAE,       // Display OFF
        0xD5, 0x80, // Set display clock divide ratio
        0xA8, 0x3F, // Set multiplex ratio (64-1)
        0xD3, 0x00, // Set display offset
        0x40,       // Set start line to 0
        0x8D, 0x14, // Enable charge pump
        0x20, 0x00, // Set memory addressing mode (horizontal)
        0xA1,       // Set segment re-map (column 127 = SEG0)
        0xC8,       // Set COM output scan direction (remapped)
        0xDA, 0x12, // Set COM pins hardware configuration
        0x81, 0xCF, // Set contrast
        0xD9, 0xF1, // Set pre-charge period
        0xDB, 0x40, // Set VCOMH deselect level
        0xA4,       // Display from RAM
        0xA6,       // Normal display (not inverted)
        0xAF,       // Display ON
    };

    esp_err_t ret = i2c_hal_write(I2C_OLED_PORT, OLED_ADDRESS,
                                   init_cmds, sizeof(init_cmds));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED init sequence failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set column and page address range for entire display
    static const uint8_t addr_cmd[] = {0x00, 0x21, 0x00, 0x7F, 0x22, 0x00, 0x07};
    ret = i2c_hal_write(I2C_OLED_PORT, OLED_ADDRESS, addr_cmd, sizeof(addr_cmd));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED address set failed");
        return ret;
    }

    // Clear entire display RAM (128x64 / 8 = 1024 bytes)
    // Send in chunks to avoid I2C buffer limits
    uint8_t clear_buf[65];  // 1 control byte + 64 data bytes
    clear_buf[0] = 0x40;    // Data stream
    memset(&clear_buf[1], 0x00, 64);

    for (int chunk = 0; chunk < 16; chunk++) {  // 16 chunks x 64 bytes = 1024 bytes
        ret = i2c_hal_write(I2C_OLED_PORT, OLED_ADDRESS, clear_buf, sizeof(clear_buf));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "OLED clear chunk %d failed", chunk);
            return ret;
        }
    }

    // Reset to top-left corner for text
    ret = i2c_hal_write(I2C_OLED_PORT, OLED_ADDRESS, addr_cmd, sizeof(addr_cmd));
    if (ret != ESP_OK) {
        return ret;
    }

    // Display "BOOT OK" at top of screen using 6x8 font bitmap
    static const uint8_t boot_ok_pattern[] = {
        0x40,  // Data stream
        0xFF, 0x89, 0x89, 0x76, 0x00,  // B
        0x7E, 0x81, 0x81, 0x7E, 0x00,  // O
        0x7E, 0x81, 0x81, 0x7E, 0x00,  // O
        0x01, 0x01, 0xFF, 0x01, 0x01, 0x00,  // T
        0x00, 0x00, 0x00,  // space
        0x7E, 0x81, 0x81, 0x7E, 0x00,  // O
        0xFF, 0x10, 0x28, 0xC6, 0x00,  // K
    };

    ret = i2c_hal_write(I2C_OLED_PORT, OLED_ADDRESS, boot_ok_pattern, sizeof(boot_ok_pattern));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OLED text write failed");
        return ret;
    }

    ESP_LOGI(TAG, "OLED initialized with test pattern");
    return ESP_OK;
}

/**
 * @brief Verify MCP23017 devices on I2C0
 *
 * @return esp_err_t ESP_OK if both devices respond
 */
static esp_err_t verify_mcp23017(void)
{
    // Read IODIR register (0x00) from both MCP23017 devices
    uint8_t reg = 0x00;
    uint8_t data;

    // Test MCP23017 #0 (0x20)
    esp_err_t ret = i2c_hal_write_read(I2C_PORT, I2C_ADDR_MCP23017_0,
                                        &reg, 1, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MCP23017 #0 (0x%02X) read failed", I2C_ADDR_MCP23017_0);
        return ret;
    }
    ESP_LOGI(TAG, "MCP23017 #0 (0x%02X) IODIR_A=0x%02X", I2C_ADDR_MCP23017_0, data);

    // Test MCP23017 #1 (0x21)
    ret = i2c_hal_write_read(I2C_PORT, I2C_ADDR_MCP23017_1,
                              &reg, 1, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MCP23017 #1 (0x%02X) read failed", I2C_ADDR_MCP23017_1);
        return ret;
    }
    ESP_LOGI(TAG, "MCP23017 #1 (0x%02X) IODIR_A=0x%02X", I2C_ADDR_MCP23017_1, data);

    return ESP_OK;
}

/**
 * @brief TPIC6B595 visual blink test
 *
 * Tests shift register outputs by blinking different patterns:
 * 1. All ENABLEs on/off
 * 2. All DIRECTIONs on/off
 * 3. Walking pattern across all axes
 *
 * @param delay_ms Delay in milliseconds between pattern changes
 */
static void tpic_blink_test(uint32_t delay_ms)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "TPIC6B595 Blink Test Starting");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SPI pins: MOSI=GPIO%d, SCLK=GPIO%d, CS=GPIO%d, OE=GPIO%d",
             GPIO_SR_MOSI, GPIO_SR_SCLK, GPIO_SR_CS, GPIO_SR_OE);

    // Initialize shift register driver
    esp_err_t ret = sr_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SR init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SR initialized, OE should be LOW (outputs enabled)");

    // Verify OE pin level
    int oe_level = gpio_get_level(GPIO_SR_OE);
    ESP_LOGI(TAG, "OE pin (GPIO%d) current level: %d (should be 0 for outputs enabled)",
             GPIO_SR_OE, oe_level);
    if (oe_level != 0) {
        ESP_LOGW(TAG, "WARNING: OE pin is HIGH - outputs are DISABLED!");
        // Force OE low
        gpio_set_level(GPIO_SR_OE, 0);
        ESP_LOGI(TAG, "Forced OE LOW, level now: %d", gpio_get_level(GPIO_SR_OE));
    }

    // Test 1: Blink all ENABLE pins (3 cycles)
    ESP_LOGI(TAG, "Test 1: Blinking all ENABLE pins...");
    for (int cycle = 0; cycle < 3; cycle++) {
        // All enables ON
        for (uint8_t axis = 0; axis < SR_NUM_AXES; axis++) {
            sr_set_enable(axis, true);
        }
        sr_update();
        ESP_LOGI(TAG, "  ENABLEs ON  (state=0x%010llX)", (unsigned long long)sr_get_state());
        vTaskDelay(pdMS_TO_TICKS(delay_ms));

        // All enables OFF
        for (uint8_t axis = 0; axis < SR_NUM_AXES; axis++) {
            sr_set_enable(axis, false);
        }
        sr_update();
        ESP_LOGI(TAG, "  ENABLEs OFF (state=0x%010llX)", (unsigned long long)sr_get_state());
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    // Test 2: Blink all DIRECTION pins (3 cycles)
    ESP_LOGI(TAG, "Test 2: Blinking all DIRECTION pins...");
    for (int cycle = 0; cycle < 3; cycle++) {
        // All directions ON (forward)
        for (uint8_t axis = 0; axis < SR_NUM_AXES; axis++) {
            sr_set_direction(axis, true);
        }
        sr_update();
        ESP_LOGI(TAG, "  DIRs ON  (state=0x%010llX)", (unsigned long long)sr_get_state());
        vTaskDelay(pdMS_TO_TICKS(delay_ms));

        // All directions OFF (reverse)
        for (uint8_t axis = 0; axis < SR_NUM_AXES; axis++) {
            sr_set_direction(axis, false);
        }
        sr_update();
        ESP_LOGI(TAG, "  DIRs OFF (state=0x%010llX)", (unsigned long long)sr_get_state());
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    // Test 3: Walking enable pattern (one axis at a time)
    ESP_LOGI(TAG, "Test 3: Walking ENABLE pattern...");
    for (uint8_t axis = 0; axis < SR_NUM_AXES; axis++) {
        sr_set_enable(axis, true);
        sr_update();
        ESP_LOGI(TAG, "  Axis %d EN ON (state=0x%010llX)", axis, (unsigned long long)sr_get_state());
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        sr_set_enable(axis, false);
        sr_update();
    }

    // Test 4: All GP outputs blink (3 cycles)
    ESP_LOGI(TAG, "Test 4: Blinking all GP outputs...");
    for (int cycle = 0; cycle < 3; cycle++) {
        // All GP outputs ON
        for (uint8_t pin = 0; pin < SR_NUM_GP_OUTPUTS; pin++) {
            sr_set_gp_output(pin, true);
        }
        sr_update();
        ESP_LOGI(TAG, "  GP ON  (state=0x%010llX)", (unsigned long long)sr_get_state());
        vTaskDelay(pdMS_TO_TICKS(delay_ms));

        // All GP outputs OFF
        for (uint8_t pin = 0; pin < SR_NUM_GP_OUTPUTS; pin++) {
            sr_set_gp_output(pin, false);
        }
        sr_update();
        ESP_LOGI(TAG, "  GP OFF (state=0x%010llX)", (unsigned long long)sr_get_state());
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    // Return to safe state
    for (uint8_t axis = 0; axis < SR_NUM_AXES; axis++) {
        sr_set_enable(axis, false);
        sr_set_direction(axis, false);
    }
    for (uint8_t pin = 0; pin < SR_NUM_GP_OUTPUTS; pin++) {
        sr_set_gp_output(pin, false);
    }
    sr_update();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "TPIC6B595 Blink Test Complete");
    ESP_LOGI(TAG, "Final state=0x%010llX (should be 0)", (unsigned long long)sr_get_state());
    ESP_LOGI(TAG, "========================================");
}

/**
 * @brief Initialize all hardware and run verification
 *
 * @return true if all hardware initialized successfully
 */
static bool hardware_init_and_verify(void)
{
    bool all_ok = true;
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Hardware Initialization Starting");
    ESP_LOGI(TAG, "========================================");

    // Initialize GPIO first (needed for other peripherals)
    ret = gpio_hal_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "GPIO HAL init failed - continuing in degraded mode");
        all_ok = false;
    }

    // Initialize I2C0 for MCP23017 expanders
    ret = i2c_hal_init(I2C_PORT, GPIO_I2C_SDA, GPIO_I2C_SCL, I2C_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C0 init failed - continuing in degraded mode");
        all_ok = false;
    } else {
        // Verify MCP23017 devices
        ret = verify_mcp23017();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "MCP23017 verification failed");
            all_ok = false;
        }
    }

    // Initialize I2C1 for OLED display
    ret = i2c_hal_init(I2C_OLED_PORT, GPIO_OLED_SDA, GPIO_OLED_SCL, I2C_OLED_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C1 init failed - continuing in degraded mode");
        all_ok = false;
    } else {
        // Initialize OLED and display boot message
        ret = oled_init_and_display();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "OLED init failed");
            all_ok = false;
        }
    }

    // Verify I2C bus isolation by re-checking I2C0 after I2C1 init
    if (i2c_hal_is_initialized(I2C_PORT)) {
        uint8_t found_addrs[8];
        size_t count;
        ret = i2c_hal_scan_bus(I2C_PORT, found_addrs, 8, &count);
        if (ret == ESP_OK && count >= 2) {
            ESP_LOGI(TAG, "I2C bus isolation verified: I2C0 still operational");
        } else {
            ESP_LOGW(TAG, "I2C bus isolation test failed");
            all_ok = false;
        }
    }

    // Initialize SPI for shift registers
    ret = yarobot_spi_init(SPI2_HOST, GPIO_SR_MOSI, GPIO_SR_SCLK, GPIO_SR_CS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SPI init failed - continuing in degraded mode");
        all_ok = false;
    } else {
        // Initialize SR driver without running blink test
        ret = sr_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "SR init failed - continuing in degraded mode");
            all_ok = false;
        } else {
            ESP_LOGI(TAG, "TPIC6B595 shift register initialized (test skipped)");
        }
    }

    ESP_LOGI(TAG, "========================================");
    if (all_ok) {
        ESP_LOGI(TAG, "Hardware Initialization: ALL PASSED");
    } else {
        ESP_LOGW(TAG, "Hardware Initialization: DEGRADED MODE");
    }
    ESP_LOGI(TAG, "========================================");

    return all_ok;
}

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
    // USB CDC Initialization (Story 2.1)
    // =========================================================================
    ESP_LOGI(TAG, "Initializing USB CDC interface...");
    esp_err_t usb_ret = usb_cdc_init();
    if (usb_ret != ESP_OK) {
        ESP_LOGW(TAG, "USB CDC init failed: %s - falling back to USB Serial JTAG",
                 esp_err_to_name(usb_ret));
    } else {
        ESP_LOGI(TAG, "USB CDC initialized successfully");
    }

    // =========================================================================
    // Hardware Initialization and Verification (Story 1.6)
    // =========================================================================
    (void)hardware_init_and_verify();  // Result used for future degraded mode handling

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
    static const char* axis_names[] = {"X", "Y", "Z", "A", "B", "C", "D", "E"};
    for (int axis = 0; axis < LIMIT_NUM_AXES; axis++) {
        char name[16];
        snprintf(name, sizeof(name), "motion_%s", axis_names[axis]);
        xTaskCreatePinnedToCore(motion_task, name, STACK_MOTION_TASK,
                                (void*)(intptr_t)axis, 15, NULL, 1);
    }
    xTaskCreatePinnedToCore(display_task, "display", STACK_DISPLAY_TASK,
                            NULL, 5, NULL, 1);

    // Boot notification is now sent by event_manager_init() via EVT_BOOT event (Story 2-7)
    // cmd_executor_init() is called by cmd_executor_task, which initializes event_manager

    ESP_LOGI(TAG, "YaRobot Control Unit - All tasks started");
}
