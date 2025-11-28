# Command System Architecture

## Overview
Multi-layered command processing system with USB CDC interface, command parsing, validation, and asynchronous execution.

## Architecture Layers

```
┌─────────────────┐
│   USB CDC RX    │ ← Raw bytes from host
├─────────────────┤
│ Command Parser  │ ← Tokenize and validate syntax
├─────────────────┤
│Command Validator│ ← Check parameters and limits
├─────────────────┤
│ Command Queue   │ ← Priority-based execution queue
├─────────────────┤
│Command Executor │ ← Dispatch to motor/IO handlers
├─────────────────┤
│Response Builder │ ← Format responses
├─────────────────┤
│   USB CDC TX    │ → Send responses to host
└─────────────────┘
```

## Core Classes

### CommandInterface

```cpp
class CommandInterface {
public:
    CommandInterface();
    esp_err_t init();
    
    // Main processing loop (runs in dedicated task)
    void processTask();
    
private:
    // USB CDC handles
    int usb_fd_;
    
    // Command processing
    LineBuffer rx_buffer_;
    QueueHandle_t command_queue_;
    QueueHandle_t response_queue_;
    
    // Tasks
    TaskHandle_t rx_task_;
    TaskHandle_t executor_task_;
    TaskHandle_t tx_task_;
};
```

### LineBuffer

```cpp
class LineBuffer {
private:
    static constexpr size_t MAX_LINE_LENGTH = 256;
    char buffer_[MAX_LINE_LENGTH];
    size_t position_ = 0;
    
public:
    // Add character, return true if line complete
    bool addChar(char c) {
        if (c == '\n' || c == '\r') {
            if (position_ > 0) {
                buffer_[position_] = '\0';
                return true;
            }
            return false;
        }
        
        if (position_ < MAX_LINE_LENGTH - 1) {
            buffer_[position_++] = c;
        }
        return false;
    }
    
    const char* getLine() const { return buffer_; }
    void reset() { position_ = 0; }
};
```

### Command Structure

```cpp
struct Command {
    enum Type {
        // Motion commands
        MOVE, MOVR, VEL, STOP,
        
        // Status commands  
        POS, STAT, INFO,
        
        // Config commands
        SETU, SETL, SETV, SETB,
        
        // Control commands
        EN, BRAKE, HOME, CALB, ZERO,
        
        // I/O commands
        DIN, DOUT,
        
        // System commands
        SAVE, LOAD, RST, ECHO, TEST,
        
        // Diagnostic commands
        LOG, DIAG, STREAM,
        
        INVALID
    };
    
    Type type;
    uint8_t axis;           // 0xFF for ALL
    float params[4];        // Up to 4 float parameters
    uint8_t param_count;
    bool has_string_param;  // For EMERGENCY, etc.
    char string_param[16];
    
    uint32_t timestamp;     // When received
    uint8_t priority;       // Execution priority
};
```

### CommandParser

```cpp
class CommandParser {
public:
    static Command parse(const char* line) {
        Command cmd;
        cmd.timestamp = esp_timer_get_time() / 1000;  // ms
        
        char command[16];
        char axis_str[8];
        
        // Extract command and axis
        int n = sscanf(line, "%15s %7s", command, axis_str);
        if (n < 1) {
            cmd.type = Command::INVALID;
            return cmd;
        }
        
        // Convert command to uppercase
        strupper(command);
        
        // Parse command type
        cmd.type = parseCommandType(command);
        
        // Parse axis if present
        if (n >= 2) {
            if (strcasecmp(axis_str, "ALL") == 0) {
                cmd.axis = 0xFF;
            } else {
                cmd.axis = atoi(axis_str);
                if (cmd.axis >= NUM_AXES) {
                    cmd.type = Command::INVALID;
                    return cmd;
                }
            }
        }
        
        // Parse remaining parameters based on command type
        parseParameters(cmd, line);
        
        // Set priority
        setPriority(cmd);
        
        return cmd;
    }
    
private:
    static Command::Type parseCommandType(const char* str);
    static void parseParameters(Command& cmd, const char* line);
    static void setPriority(Command& cmd);
};
```

### CommandValidator

```cpp
class CommandValidator {
public:
    enum ValidationError {
        VALID = 0,
        INVALID_AXIS,
        AXIS_NOT_ENABLED,
        POSITION_LIMIT_EXCEEDED,
        VELOCITY_LIMIT_EXCEEDED,
        INVALID_PARAMETER_COUNT,
        INVALID_PARAMETER_VALUE,
        SYSTEM_NOT_READY,
        EMERGENCY_STOP_ACTIVE
    };
    
    static ValidationError validate(const Command& cmd) {
        // Check emergency stop
        if (MotorController::getInstance().isEmergencyStopped() && 
            cmd.type != Command::STOP && 
            cmd.type != Command::RST) {
            return EMERGENCY_STOP_ACTIVE;
        }
        
        // Validate based on command type
        switch (cmd.type) {
            case Command::MOVE:
                return validateMove(cmd);
            case Command::VEL:
                return validateVelocity(cmd);
            // ... other commands
        }
        
        return VALID;
    }
    
private:
    static ValidationError validateMove(const Command& cmd);
    static ValidationError validateVelocity(const Command& cmd);
    // ... other validation methods
};
```

### CommandExecutor

```cpp
class CommandExecutor {
public:
    struct Response {
        bool success;
        char message[256];
        bool is_stream;  // For continuous streaming
    };
    
    static Response execute(const Command& cmd) {
        Response resp;
        
        switch (cmd.type) {
            case Command::MOVE:
                return executeMove(cmd);
                
            case Command::STOP:
                return executeStop(cmd);
                
            case Command::POS:
                return executeGetPosition(cmd);
                
            // ... other commands
        }
        
        resp.success = false;
        snprintf(resp.message, sizeof(resp.message), 
                 "ERROR E001 Invalid command");
        return resp;
    }
    
private:
    static Response executeMove(const Command& cmd) {
        Response resp;
        auto& controller = MotorController::getInstance();
        
        if (cmd.axis == 0xFF) {
            // Move all axes - requires special handling
            resp.success = false;
            snprintf(resp.message, sizeof(resp.message),
                     "ERROR E002 ALL not supported for MOVE");
        } else {
            Axis* axis = controller.getAxis(cmd.axis);
            if (!axis) {
                resp.success = false;
                snprintf(resp.message, sizeof(resp.message),
                         "ERROR E002 Invalid axis %d", cmd.axis);
                return resp;
            }
            
            float position = cmd.params[0];
            float velocity = (cmd.param_count > 1) ? cmd.params[1] : 0;
            float accel = (cmd.param_count > 2) ? cmd.params[2] : 0;
            
            esp_err_t err = axis->moveToPosition(position, velocity, accel);
            
            if (err == ESP_OK) {
                resp.success = true;
                strcpy(resp.message, "OK MOVING");
            } else {
                resp.success = false;
                snprintf(resp.message, sizeof(resp.message),
                         "ERROR E008 Motor fault");
            }
        }
        
        return resp;
    }
    
    // ... other execute methods
};
```

### Command Flow Tasks

```cpp
// USB RX Task - High Priority
void usbRxTask(void* arg) {
    CommandInterface* iface = (CommandInterface*)arg;
    char byte;
    LineBuffer line_buffer;
    
    while (1) {
        // Read from USB CDC
        int len = read(iface->usb_fd_, &byte, 1);
        if (len > 0) {
            if (line_buffer.addChar(byte)) {
                // Complete line received
                Command cmd = CommandParser::parse(line_buffer.getLine());
                
                if (cmd.type != Command::INVALID) {
                    // Quick validation
                    if (CommandValidator::validate(cmd) == CommandValidator::VALID) {
                        xQueueSend(iface->command_queue_, &cmd, portMAX_DELAY);
                    } else {
                        // Send immediate error response
                        CommandExecutor::Response resp;
                        resp.success = false;
                        snprintf(resp.message, sizeof(resp.message),
                                "ERROR E003 Invalid parameter");
                        xQueueSend(iface->response_queue_, &resp, 0);
                    }
                }
                
                line_buffer.reset();
            }
        }
        
        vTaskDelay(1);  // Yield
    }
}

// Command Executor Task - Medium Priority  
void commandExecutorTask(void* arg) {
    CommandInterface* iface = (CommandInterface*)arg;
    Command cmd;
    
    while (1) {
        if (xQueueReceive(iface->command_queue_, &cmd, portMAX_DELAY)) {
            // Execute command
            CommandExecutor::Response resp = CommandExecutor::execute(cmd);
            
            // Send response
            xQueueSend(iface->response_queue_, &resp, portMAX_DELAY);
        }
    }
}

// USB TX Task - Medium Priority
void usbTxTask(void* arg) {
    CommandInterface* iface = (CommandInterface*)arg;
    CommandExecutor::Response resp;
    
    while (1) {
        if (xQueueReceive(iface->response_queue_, &resp, portMAX_DELAY)) {
            // Add line ending
            strcat(resp.message, "\r\n");
            
            // Write to USB CDC
            write(iface->usb_fd_, resp.message, strlen(resp.message));
        }
    }
}
```

## Priority Command Handling

```cpp
class PriorityCommandQueue {
private:
    struct PrioritizedCommand {
        Command cmd;
        uint8_t priority;
        
        bool operator<(const PrioritizedCommand& other) const {
            return priority < other.priority;  // Higher number = higher priority
        }
    };
    
    std::priority_queue<PrioritizedCommand> queue_;
    SemaphoreHandle_t mutex_;
    
public:
    void push(const Command& cmd) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        
        PrioritizedCommand pcmd;
        pcmd.cmd = cmd;
        pcmd.priority = getPriority(cmd);
        
        queue_.push(pcmd);
        
        xSemaphoreGive(mutex_);
    }
    
private:
    uint8_t getPriority(const Command& cmd) {
        switch (cmd.type) {
            case Command::STOP:
                return (cmd.has_string_param && 
                       strcmp(cmd.string_param, "EMERGENCY") == 0) ? 255 : 200;
            case Command::INFO:
            case Command::POS:
            case Command::STAT:
                return 150;  // Status commands
            case Command::MOVE:
            case Command::VEL:
                return 100;  // Motion commands
            default:
                return 50;   // Configuration
        }
    }
};
```

## Streaming Support

```cpp
class StreamingManager {
private:
    struct StreamConfig {
        uint32_t interval_ms;
        uint8_t axis_mask;  // Bit mask of axes to stream
        uint32_t last_send_time;
        bool active;
    };
    
    StreamConfig config_;
    TaskHandle_t stream_task_;
    
public:
    esp_err_t startStreaming(uint32_t interval_ms, uint8_t axis_mask) {
        config_.interval_ms = interval_ms;
        config_.axis_mask = axis_mask;
        config_.active = true;
        config_.last_send_time = 0;
        
        if (!stream_task_) {
            xTaskCreate(streamTask, "stream", 2048, this, 5, &stream_task_);
        }
        
        return ESP_OK;
    }
    
    void stopStreaming() {
        config_.active = false;
    }
    
private:
    static void streamTask(void* arg) {
        StreamingManager* mgr = (StreamingManager*)arg;
        char buffer[256];
        
        while (1) {
            if (mgr->config_.active) {
                uint32_t now = esp_timer_get_time() / 1000;
                
                if (now - mgr->config_.last_send_time >= mgr->config_.interval_ms) {
                    // Build stream message
                    strcpy(buffer, "STRM ");
                    
                    auto& controller = MotorController::getInstance();
                    bool first = true;
                    
                    for (int i = 0; i < NUM_AXES; i++) {
                        if (mgr->config_.axis_mask & (1 << i)) {
                            if (!first) strcat(buffer, ",");
                            
                            char pos_str[32];
                            Axis* axis = controller.getAxis(i);
                            float pos = axis ? axis->getCurrentPosition() : 0.0f;
                            
                            snprintf(pos_str, sizeof(pos_str), "%d:%.2f", i, pos);
                            strcat(buffer, pos_str);
                            
                            first = false;
                        }
                    }
                    
                    // Send stream data
                    // ... send via USB
                    
                    mgr->config_.last_send_time = now;
                }
            }
            
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
};
```

## Error Handling

```cpp
class ErrorManager {
public:
    static const char* getErrorMessage(CommandValidator::ValidationError error) {
        switch (error) {
            case CommandValidator::INVALID_AXIS:
                return "ERROR E002 Invalid axis number";
            case CommandValidator::AXIS_NOT_ENABLED:
                return "ERROR E004 Axis not enabled";
            case CommandValidator::POSITION_LIMIT_EXCEEDED:
                return "ERROR E005 Position limit exceeded";
            case CommandValidator::VELOCITY_LIMIT_EXCEEDED:
                return "ERROR E005 Velocity limit exceeded";
            case CommandValidator::EMERGENCY_STOP_ACTIVE:
                return "ERROR E006 Emergency stop active";
            default:
                return "ERROR E001 Unknown error";
        }
    }
};
```

## Integration Example

```cpp
// In main.cpp
void initCommandSystem() {
    // Initialize USB CDC
    tinyusb_config_t tusb_cfg = {};
    tinyusb_driver_install(&tusb_cfg);
    
    // Create command interface
    static CommandInterface cmd_interface;
    cmd_interface.init();
    
    // Start command processing
    xTaskCreate(
        [](void* arg) {
            CommandInterface* iface = (CommandInterface*)arg;
            iface->processTask();
        },
        "cmd_main",
        4096,
        &cmd_interface,
        10,
        nullptr
    );
    
    ESP_LOGI(TAG, "Command system initialized");
}
```