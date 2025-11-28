# Power-On Initialization Sequence

## Overview
Detailed power-on initialization sequence for yarobot_control_unit, ensuring safe and predictable startup with proper hardware initialization, configuration loading, and system validation.

## Initialization Phases

```
┌─────────────────┐
│   BOOTLOADER    │
└────────┬────────┘
         v
┌─────────────────┐
│ EARLY HARDWARE  │ - Clocks, Power, GPIO
└────────┬────────┘
         v
┌─────────────────┐
│  SAFETY FIRST   │ - E-stop, Motor disable
└────────┬────────┘
         v
┌─────────────────┐
│   PERIPHERALS   │ - I2C, SPI, UART, USB
└────────┬────────┘
         v
┌─────────────────┐
│  LOAD CONFIG    │ - NVS, Motor params
└────────┬────────┘
         v
┌─────────────────┐
│   VALIDATION    │ - Self-test, Limits
└────────┬────────┘
         v
┌─────────────────┐
│  START TASKS    │ - FreeRTOS scheduler
└────────┬────────┘
         v
┌─────────────────┐
│     READY       │ - Accept commands
└─────────────────┘
```

## Phase 1: Early Hardware Initialization

```cpp
void app_main(void) {
    ESP_LOGI(TAG, "YaRobot Control Unit v%s starting...", VERSION);
    
    // 1.1 Configure CPU and Memory
    esp_pm_config_esp32s3_t pm_config = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 80,
        .light_sleep_enable = false
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    
    // 1.2 Initialize heap and memory
    heap_caps_init();
    
    // 1.3 Configure brownout detector
    esp_brownout_init();
    esp_brownout_disable();  // Temporarily disable
    
    // 1.4 Basic GPIO configuration
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    
    // Continue to Phase 2
    initializeSafetySystems();
}
```

## Phase 2: Safety Systems First

```cpp
void initializeSafetySystems() {
    ESP_LOGI(TAG, "Initializing safety systems...");
    
    // 2.1 Configure E-stop GPIO (before anything else)
    gpio_config_t estop_config = {
        .pin_bit_mask = (1ULL << GPIO_E_STOP),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&estop_config);
    
    // 2.2 Set all motor control outputs to safe state
    // Direction pins
    uint64_t dir_mask = (1ULL << GPIO_DIR_0) | (1ULL << GPIO_DIR_1) | 
                       (1ULL << GPIO_DIR_2) | (1ULL << GPIO_DIR_3) |
                       (1ULL << GPIO_DIR_4) | (1ULL << GPIO_DIR_5);
    
    gpio_config_t dir_config = {
        .pin_bit_mask = dir_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&dir_config);
    
    // Set directions to safe default
    for (int i = 0; i < 6; i++) {
        gpio_set_level(dir_pins[i], 0);
    }
    
    // 2.3 Initialize I2C for motor enables (disabled state)
    initializeI2CEarly();
    
    // 2.4 Disable all motors via I2C expander
    disableAllMotors();
    
    // 2.5 Install E-stop ISR (only after motors disabled)
    gpio_isr_handler_add(GPIO_E_STOP, emergency_stop_isr, NULL);
    
    // Check if E-stop is already active
    if (gpio_get_level(GPIO_E_STOP) == 0) {
        ESP_LOGE(TAG, "E-stop active at startup!");
        system_state = STATE_EMERGENCY_STOP;
    }
}

void disableAllMotors() {
    // Assuming motor enables on PCF8575 at 0x20
    uint16_t enables = 0x0000;  // All low (disabled)
    i2c_write_word_early(I2C_ADDR_EXPANDER_0, enables);
}
```

## Phase 3: Peripheral Initialization

```cpp
void initializePeripherals() {
    ESP_LOGI(TAG, "Initializing peripherals...");
    
    // 3.1 Full I2C initialization
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,  // 100kHz standard
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
    
    // 3.2 Scan I2C bus
    scanI2CBus();
    
    // 3.3 Initialize OLED display
    if (oled_init() == ESP_OK) {
        oled_clear();
        oled_write_line(0, "YaRobot Init...");
        oled_update();
    }
    
    // 3.4 Initialize USB CDC
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    
    tinyusb_config_cdcacm_t amc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 256,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = usb_line_state_changed,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
    
    // 3.5 Initialize RMT for motor control
    initializeRMT();
    
    // 3.6 Initialize MCPWM for axes 4-5
    initializeMCPWM();
    
    // 3.7 Initialize PCNT for position counting
    initializePCNT();
}
```

## Phase 4: Configuration Loading

```cpp
void loadConfiguration() {
    ESP_LOGI(TAG, "Loading configuration...");
    
    // 4.1 Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || 
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 4.2 Initialize NVS manager
    nvsManager.init();
    
    // 4.3 Load system configuration
    SystemConfig_t sys_config;
    if (!nvsManager.loadSystemConfig(&sys_config)) {
        ESP_LOGW(TAG, "Using default system configuration");
        setDefaultSystemConfig(&sys_config);
    }
    
    // 4.4 Create motor instances
    for (int i = 0; i < 6; i++) {
        if (i < 4) {
            // RMT-based servo motors
            motors[i] = new RMTMotor(i, rmt_channels[i]);
        } else {
            // MCPWM-based stepper motors
            motors[i] = new MCPWMMotor(i, mcpwm_units[i-4]);
        }
    }
    
    // 4.5 Load motor configurations
    for (int i = 0; i < 6; i++) {
        MotorConfig_t config;
        if (nvsManager.loadMotorConfig(i, &config)) {
            motors[i]->setConfiguration(&config);
            ESP_LOGI(TAG, "Loaded config for motor %d", i);
        } else {
            ESP_LOGW(TAG, "Using defaults for motor %d", i);
        }
        
        // Load last position if available
        float last_pos = 0;
        if (nvsManager.loadLastPosition(i, &last_pos)) {
            motors[i]->setCurrentPosition(last_pos);
            ESP_LOGI(TAG, "Motor %d last position: %.2f", i, last_pos);
        }
    }
    
    // Update display
    if (oled_initialized) {
        oled_write_line(1, "Config loaded");
        oled_update();
    }
}
```

## Phase 5: System Validation

```cpp
void validateSystem() {
    ESP_LOGI(TAG, "Validating system...");
    
    bool validation_passed = true;
    
    // 5.1 Test I2C communication
    for (int i = 0; i < 2; i++) {
        uint8_t addr = (i == 0) ? 0x20 : 0x21;
        uint16_t test_data;
        
        if (i2c_read_word(addr, &test_data) != ESP_OK) {
            ESP_LOGE(TAG, "I2C expander %d not responding", i);
            validation_passed = false;
        }
    }
    
    // 5.2 Check limit switches
    uint16_t switches = readLimitSwitches();
    for (int i = 0; i < 6; i++) {
        bool min_triggered = (switches & (1 << (i * 2))) != 0;
        bool max_triggered = (switches & (1 << (i * 2 + 1))) != 0;
        
        if (min_triggered && max_triggered) {
            ESP_LOGE(TAG, "Motor %d: Both limits triggered!", i);
            validation_passed = false;
        }
        
        if (min_triggered || max_triggered) {
            ESP_LOGW(TAG, "Motor %d: %s limit active", i,
                    min_triggered ? "MIN" : "MAX");
        }
    }
    
    // 5.3 Test motor control signals
    for (int i = 0; i < 6; i++) {
        // Send single test pulse (motor disabled)
        if (!motors[i]->testPulseGeneration()) {
            ESP_LOGE(TAG, "Motor %d pulse generation failed", i);
            validation_passed = false;
        }
    }
    
    // 5.4 Memory check
    size_t free_heap = esp_get_free_heap_size();
    if (free_heap < 50000) {  // Less than 50KB
        ESP_LOGE(TAG, "Low memory: %d bytes free", free_heap);
        validation_passed = false;
    }
    
    // 5.5 Update display with results
    if (oled_initialized) {
        oled_write_line(2, validation_passed ? "Tests PASSED" : "Tests FAILED");
        oled_update();
    }
    
    if (!validation_passed && !ALLOW_DEGRADED_MODE) {
        ESP_LOGE(TAG, "System validation failed - halting");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
```

## Phase 6: FreeRTOS Task Startup

```cpp
void startSystemTasks() {
    ESP_LOGI(TAG, "Starting system tasks...");
    
    // 6.1 Create synchronization objects
    command_queue = xQueueCreate(32, sizeof(Command_t));
    response_queue = xQueueCreate(32, sizeof(Response_t));
    system_events = xEventGroupCreate();
    
    for (int i = 0; i < 6; i++) {
        motion_events[i] = xEventGroupCreate();
        position_mutex[i] = xSemaphoreCreateMutex();
    }
    
    i2c_mutex = xSemaphoreCreateMutex();
    
    // 6.2 Create high priority tasks first
    xTaskCreatePinnedToCore(safetyMonitorTask, "safety", 4096, NULL,
                           PRIORITY_SAFETY_MONITOR, &safety_task, 0);
    
    // 6.3 Create motion control tasks
    for (int i = 0; i < 6; i++) {
        char name[16];
        snprintf(name, sizeof(name), "motor_%d", i);
        xTaskCreatePinnedToCore(motionTask, name, 3072, (void*)i,
                               PRIORITY_MOTION_CONTROL, &motion_tasks[i], 1);
    }
    
    // 6.4 Create communication tasks
    xTaskCreatePinnedToCore(usbRxTask, "usb_rx", 2048, NULL,
                           PRIORITY_USB_RX, &usb_rx_task, 0);
    
    xTaskCreatePinnedToCore(usbTxTask, "usb_tx", 2048, NULL,
                           PRIORITY_USB_TX, &usb_tx_task, 1);
    
    xTaskCreatePinnedToCore(commandExecutorTask, "cmd_exec", 8192, NULL,
                           PRIORITY_COMMAND_EXECUTOR, &cmd_exec_task, 1);
    
    // 6.5 Create monitoring tasks
    xTaskCreate(i2cMonitorTask, "i2c_mon", 2048, NULL,
               PRIORITY_I2C_MONITOR, &i2c_monitor_task);
    
    xTaskCreate(displayUpdateTask, "display", 4096, NULL,
               PRIORITY_DISPLAY_UPDATE, &display_task);
    
    xTaskCreate(idleMonitorTask, "idle_mon", 2048, NULL,
               PRIORITY_IDLE_MONITOR, NULL);
    
    // 6.6 Create diagnostic task
    xTaskCreate(diagnosticTask, "diag", 3072, NULL,
               PRIORITY_DIAGNOSTICS, NULL);
    
    ESP_LOGI(TAG, "All tasks started");
}
```

## Phase 7: Final System Ready

```cpp
void enterOperationalMode() {
    ESP_LOGI(TAG, "Entering operational mode...");
    
    // 7.1 Set system ready flag
    xEventGroupSetBits(system_events, SYS_EVENT_READY);
    
    // 7.2 Enable watchdog
    esp_task_wdt_init(5, true);  // 5 second timeout
    esp_task_wdt_add(NULL);  // Add main task
    
    // 7.3 Send startup message
    Response resp;
    snprintf(resp.message, sizeof(resp.message),
            "READY YaRobot Control Unit v%s", VERSION);
    xQueueSend(response_queue, &resp, 0);
    
    // 7.4 Update display
    if (oled_initialized) {
        oled_clear();
        for (int i = 0; i < 6; i++) {
            char line[32];
            snprintf(line, sizeof(line), "%d: READY", i);
            oled_write_line(i, line);
        }
        oled_write_line(7, "System Ready");
        oled_update();
    }
    
    // 7.5 Log memory usage
    ESP_LOGI(TAG, "Free heap: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Minimum free heap: %d bytes", 
             esp_get_minimum_free_heap_size());
    
    // Main task can now idle
    while (1) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Periodic health check
        if (xTaskGetTickCount() % 60000 == 0) {  // Every minute
            printSystemHealth();
        }
    }
}
```

## Error Handling During Initialization

```cpp
// Global initialization state
typedef enum {
    INIT_STATE_BOOT,
    INIT_STATE_SAFETY,
    INIT_STATE_PERIPHERALS,
    INIT_STATE_CONFIG,
    INIT_STATE_VALIDATION,
    INIT_STATE_TASKS,
    INIT_STATE_READY,
    INIT_STATE_ERROR
} InitState_t;

InitState_t init_state = INIT_STATE_BOOT;

// Error handler
void handleInitError(const char* phase, esp_err_t error) {
    ESP_LOGE(TAG, "Init failed at %s: %s", phase, esp_err_to_name(error));
    
    // Update display if possible
    if (oled_initialized) {
        oled_clear();
        oled_write_line(0, "INIT ERROR");
        oled_write_line(1, phase);
        oled_write_line(2, esp_err_to_name(error));
        oled_update();
    }
    
    // Try to maintain safety
    gpio_set_level(GPIO_E_STOP, 0);  // Simulate E-stop
    
    // Enter error state
    init_state = INIT_STATE_ERROR;
    
    // Blink LED or other indication
    while (1) {
        // Toggle error LED
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

## Power Fail Recovery

```cpp
// Check for previous unexpected shutdown
void checkPowerFailRecovery() {
    uint32_t reset_reason = esp_reset_reason();
    
    switch (reset_reason) {
        case ESP_RST_BROWNOUT:
            ESP_LOGW(TAG, "Recovering from brownout");
            // Load quick-saved positions
            loadEmergencyPositions();
            break;
            
        case ESP_RST_PANIC:
            ESP_LOGW(TAG, "Recovering from panic");
            // Full re-initialization needed
            break;
            
        case ESP_RST_POWERON:
            ESP_LOGI(TAG, "Normal power-on");
            break;
    }
}

void loadEmergencyPositions() {
    for (int i = 0; i < 6; i++) {
        float qpos;
        if (nvs_get_blob(motor_handles[i], "qpos", 
                         &qpos, sizeof(float)) == ESP_OK) {
            motors[i]->setCurrentPosition(qpos);
            ESP_LOGW(TAG, "Motor %d emergency position: %.2f", i, qpos);
        }
    }
}
```

## Boot Time Optimization

1. **Parallel Initialization**: Where possible, initialize independent systems concurrently
2. **Lazy Loading**: Defer non-critical initialization until after system ready
3. **Fast Path**: Skip validation in development mode (compile flag)
4. **Cached Configs**: Keep frequently accessed configs in RAM

```cpp
// Example parallel initialization
void parallelPeripheralInit() {
    TaskHandle_t init_tasks[3];
    
    xTaskCreate(initI2CTask, "init_i2c", 2048, NULL, 5, &init_tasks[0]);
    xTaskCreate(initUSBTask, "init_usb", 2048, NULL, 5, &init_tasks[1]);
    xTaskCreate(initDisplayTask, "init_disp", 2048, NULL, 5, &init_tasks[2]);
    
    // Wait for all to complete
    for (int i = 0; i < 3; i++) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelete(init_tasks[i]);
    }
}
```