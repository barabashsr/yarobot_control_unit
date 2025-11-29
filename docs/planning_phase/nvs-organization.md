# NVS (Non-Volatile Storage) Organization

## Overview
Non-volatile storage organization for persisting motor configurations, calibration data, and system settings across power cycles. Uses ESP-IDF's NVS library with wear-leveling.

## NVS Partition Layout

```
nvs,      data, nvs,     0x9000,  0x6000,  // 24KB for NVS
nvs_keys, data, nvs_keys,0xF000,  0x1000,  // 4KB for encrypted keys
```

## Namespace Structure

### 1. System Configuration Namespace
```cpp
// Namespace: "system"
typedef struct {
    uint32_t magic;              // 0xDEADBEEF - validation
    uint16_t version;            // Configuration version
    uint32_t idle_timeout_ms;    // Motor idle timeout
    uint8_t i2c_retry_count;     // I2C retry attempts
    uint8_t emergency_stop_mode; // 0=halt, 1=controlled stop
    uint32_t crc32;             // Data integrity check
} SystemConfig_t;

// Keys
#define NVS_KEY_SYSTEM_CONFIG    "sys_cfg"
#define NVS_KEY_USB_SERIAL       "usb_serial"
#define NVS_KEY_DEVICE_NAME      "dev_name"
```

### 2. Motor Configuration Namespace (per axis)
```cpp
// Namespace: "motor_0" through "motor_7" (8 axes: X,Y,Z,A,B,C,D,E)
typedef struct {
    // Basic parameters
    float steps_per_mm;          // Resolution
    float max_velocity;          // mm/s
    float acceleration;          // mm/s²
    
    // Limits
    float min_position;          // Software limit min
    float max_position;          // Software limit max
    bool  limit_switches_enabled;
    bool  limit_switch_normally_open;
    
    // Motion profiles
    float jerk_limit;            // mm/s³ (future)
    float homing_velocity;       // mm/s
    float homing_backoff;        // mm
    
    // Driver configuration
    uint8_t step_pulse_width_us; // 1-10μs typical
    uint8_t dir_setup_time_us;   // Direction setup time
    bool    step_active_high;    // Step polarity
    bool    dir_active_high;     // Direction polarity
    bool    enable_active_high;  // Enable polarity
    
    // Servo specific (axes 0-3)
    bool    use_position_complete;
    uint16_t position_window;     // Pulses
    bool    use_z_signal;        // Index pulse
    
    uint32_t crc32;             // Data integrity
} MotorConfig_t;

// Keys
#define NVS_KEY_MOTOR_CONFIG     "config"
#define NVS_KEY_CALIBRATION      "calib"
#define NVS_KEY_POSITION         "last_pos"
```

### 3. Calibration Data Namespace
```cpp
// Namespace: "calib_0" through "calib_7" (8 axes: X,Y,Z,A,B,C,D,E)
typedef struct {
    // Homing results
    float home_position;         // Absolute position at home
    float z_signal_offset;       // For servos with index
    
    // Backlash compensation
    float backlash_distance;     // mm
    bool  backlash_enabled;
    
    // Error compensation
    float position_error_map[10]; // Simple error map
    uint8_t error_map_points;     // Number of valid points
    
    // Statistics
    uint32_t total_distance;     // Total mm traveled
    uint32_t total_moves;        // Move count
    uint32_t error_count;        // Cumulative errors
    
    // Timestamps
    uint32_t calibration_time;   // When calibrated
    uint32_t last_move_time;     // Last activity
    
    uint32_t crc32;
} CalibrationData_t;
```

## NVS API Implementation

### Initialization
```cpp
class NVSManager {
private:
    nvs_handle_t system_handle;
    nvs_handle_t motor_handles[8];  // 8 axes: X,Y,Z,A,B,C,D,E
    nvs_handle_t calib_handles[8];  // 8 axes: X,Y,Z,A,B,C,D,E
    
public:
    esp_err_t init() {
        // Initialize NVS
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || 
            ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        
        // Open namespaces
        ESP_ERROR_CHECK(nvs_open("system", NVS_READWRITE, &system_handle));
        
        // Open motor namespaces (8 axes: X,Y,Z,A,B,C,D,E)
        for (int i = 0; i < 8; i++) {
            char namespace_name[16];
            snprintf(namespace_name, sizeof(namespace_name), "motor_%d", i);
            ESP_ERROR_CHECK(nvs_open(namespace_name, NVS_READWRITE, &motor_handles[i]));
            
            snprintf(namespace_name, sizeof(namespace_name), "calib_%d", i);
            ESP_ERROR_CHECK(nvs_open(namespace_name, NVS_READWRITE, &calib_handles[i]));
        }
        
        return ESP_OK;
    }
```

### Loading Configuration
```cpp
    bool loadMotorConfig(uint8_t axis, MotorConfig_t* config) {
        size_t length = sizeof(MotorConfig_t);
        esp_err_t ret = nvs_get_blob(motor_handles[axis], 
                                      NVS_KEY_MOTOR_CONFIG, 
                                      config, &length);
        
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "No config for motor %d, using defaults", axis);
            setDefaultMotorConfig(axis, config);
            return false;
        }
        
        // Verify CRC
        uint32_t calculated_crc = calculateCRC32(config, 
                                                 sizeof(MotorConfig_t) - sizeof(uint32_t));
        if (calculated_crc != config->crc32) {
            ESP_LOGE(TAG, "CRC mismatch for motor %d config", axis);
            setDefaultMotorConfig(axis, config);
            return false;
        }
        
        ESP_LOGI(TAG, "Loaded config for motor %d", axis);
        return true;
    }
```

### Saving Configuration
```cpp
    esp_err_t saveMotorConfig(uint8_t axis, const MotorConfig_t* config) {
        // Calculate CRC
        MotorConfig_t config_copy = *config;
        config_copy.crc32 = calculateCRC32(&config_copy, 
                                          sizeof(MotorConfig_t) - sizeof(uint32_t));
        
        // Save to NVS
        esp_err_t ret = nvs_set_blob(motor_handles[axis], 
                                     NVS_KEY_MOTOR_CONFIG,
                                     &config_copy, 
                                     sizeof(MotorConfig_t));
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save motor %d config: %s", 
                     axis, esp_err_to_name(ret));
            return ret;
        }
        
        // Commit changes
        ret = nvs_commit(motor_handles[axis]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit motor %d config: %s", 
                     axis, esp_err_to_name(ret));
        }
        
        return ret;
    }
```

### Position Persistence
```cpp
    // Save last known position before power down
    esp_err_t saveLastPosition(uint8_t axis, float position) {
        esp_err_t ret = nvs_set_blob(motor_handles[axis], 
                                     NVS_KEY_POSITION,
                                     &position, sizeof(float));
        
        // Don't commit immediately - batch commits for wear leveling
        if (ret == ESP_OK) {
            position_dirty[axis] = true;
        }
        
        return ret;
    }
    
    // Periodic commit task
    void commitTask(void* param) {
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(5000));  // Every 5 seconds

            for (int i = 0; i < 8; i++) {  // 8 axes
                if (position_dirty[i]) {
                    nvs_commit(motor_handles[i]);
                    position_dirty[i] = false;
                }
            }
        }
    }
```

## Default Configurations

```cpp
void setDefaultMotorConfig(uint8_t axis, MotorConfig_t* config) {
    memset(config, 0, sizeof(MotorConfig_t));
    
    // Common defaults
    config->steps_per_mm = 80.0f;        // 1/16 microstepping
    config->max_velocity = 50.0f;        // mm/s
    config->acceleration = 500.0f;       // mm/s²
    config->min_position = -100.0f;      // mm
    config->max_position = 100.0f;       // mm
    config->step_pulse_width_us = 2;     // 2μs pulse
    config->dir_setup_time_us = 5;       // 5μs setup
    config->step_active_high = true;
    config->dir_active_high = true;
    config->enable_active_high = false;  // Active low enable
    
    // Servo specific (axes 0-4: X,Y,Z,A,B)
    if (axis < 5) {
        config->use_position_complete = true;
        config->position_window = 20;     // pulses
        config->use_z_signal = true;
        config->homing_velocity = 10.0f;  // Slower for servos
    } else if (axis < 7) {
        // Stepper specific (axes 5-6: C,D)
        config->use_position_complete = false;
        config->use_z_signal = false;
        config->homing_velocity = 20.0f;  // Faster for steppers
    } else {
        // Discrete axis (axis 7: E)
        config->use_position_complete = false;
        config->use_z_signal = false;
        config->homing_velocity = 100.0f; // Full speed for discrete
    }
    
    config->homing_backoff = 5.0f;       // mm
}
```

## Configuration Commands

### Save Current Configuration
```cpp
// SAVE <axis>
Response cmdSave(uint8_t axis) {
    if (axis >= 8) {  // 8 axes: X,Y,Z,A,B,C,D,E
        return Response::error("Invalid axis");
    }
    
    // Get current motor configuration
    MotorConfig_t config;
    motors[axis]->getConfiguration(&config);
    
    // Save to NVS
    if (nvsManager.saveMotorConfig(axis, &config) == ESP_OK) {
        return Response::ok("Configuration saved");
    } else {
        return Response::error("Failed to save configuration");
    }
}
```

### Load Configuration
```cpp
// LOAD <axis>
Response cmdLoad(uint8_t axis) {
    if (axis >= 8) {  // 8 axes: X,Y,Z,A,B,C,D,E
        return Response::error("Invalid axis");
    }
    
    MotorConfig_t config;
    if (nvsManager.loadMotorConfig(axis, &config)) {
        motors[axis]->setConfiguration(&config);
        return Response::ok("Configuration loaded");
    } else {
        return Response::error("No saved configuration");
    }
}
```

### Reset to Defaults
```cpp
// DEFAULTS <axis>
Response cmdDefaults(uint8_t axis) {
    if (axis >= 8) {  // 8 axes: X,Y,Z,A,B,C,D,E
        return Response::error("Invalid axis");
    }
    
    MotorConfig_t config;
    setDefaultMotorConfig(axis, &config);
    motors[axis]->setConfiguration(&config);
    
    return Response::ok("Defaults loaded");
}
```

## Wear Leveling Strategy

1. **Batch Commits**: Group multiple parameter changes before committing
2. **Position Caching**: Only save position on significant changes (>0.1mm)
3. **Time-based Commits**: Periodic commits every 5 seconds if dirty
4. **Power-fail Detection**: Save critical state on brownout detection

```cpp
// Brownout detector callback
void IRAM_ATTR brownout_handler(void) {
    // Quick save of critical data
    for (int i = 0; i < 8; i++) {  // 8 axes
        float pos = quick_position_read(i);
        nvs_set_blob_fast(motor_handles[i], "qpos", &pos, sizeof(float));
    }
    nvs_commit_all();  // Force immediate commit
}
```

## NVS Usage Statistics

```cpp
void printNVSStats() {
    nvs_stats_t nvs_stats;
    nvs_get_stats(NULL, &nvs_stats);
    
    ESP_LOGI(TAG, "NVS Stats:");
    ESP_LOGI(TAG, "  Used entries: %d", nvs_stats.used_entries);
    ESP_LOGI(TAG, "  Free entries: %d", nvs_stats.free_entries);
    ESP_LOGI(TAG, "  Total entries: %d", nvs_stats.total_entries);
    ESP_LOGI(TAG, "  Namespace count: %d", nvs_stats.namespace_count);
}
```

## Migration and Versioning

```cpp
// Handle configuration version changes
void migrateConfiguration() {
    uint16_t version = 0;
    size_t length = sizeof(version);
    
    if (nvs_get_u16(system_handle, "cfg_version", &version) != ESP_OK) {
        version = 0;  // First run
    }
    
    if (version < CURRENT_CONFIG_VERSION) {
        ESP_LOGI(TAG, "Migrating config from v%d to v%d", 
                 version, CURRENT_CONFIG_VERSION);
        
        // Perform migration based on version
        switch (version) {
            case 0:  // Initial version
                // Set all defaults
                initializeAllConfigs();
                break;
                
            case 1:  // Added backlash compensation
                // Migrate v1 -> v2
                addBacklashFields();
                break;
        }
        
        // Update version
        nvs_set_u16(system_handle, "cfg_version", CURRENT_CONFIG_VERSION);
        nvs_commit(system_handle);
    }
}
```

## Error Recovery

```cpp
// Detect and recover from corrupted NVS
esp_err_t verifyNVSIntegrity() {
    bool corrupted = false;

    // Check each motor config (8 axes)
    for (int i = 0; i < 8; i++) {
        MotorConfig_t config;
        size_t length = sizeof(config);
        
        if (nvs_get_blob(motor_handles[i], NVS_KEY_MOTOR_CONFIG, 
                         &config, &length) == ESP_OK) {
            uint32_t crc = calculateCRC32(&config, 
                                         sizeof(config) - sizeof(uint32_t));
            if (crc != config.crc32) {
                ESP_LOGE(TAG, "Motor %d config corrupted", i);
                corrupted = true;
                
                // Reset to defaults
                setDefaultMotorConfig(i, &config);
                saveMotorConfig(i, &config);
            }
        }
    }
    
    return corrupted ? ESP_ERR_INVALID_CRC : ESP_OK;
}
```