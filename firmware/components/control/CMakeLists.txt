# Control umbrella component
# Aggregates all control sub-components for a unified build

# STORY-3-2-TEST: Remove test_pulse_cmd.cpp and pulse_gen dependency after hardware verification
# STORY-3-5-TEST: Remove position dependency after hardware verification
idf_component_register(
    SRCS
        "tasks/task_stubs.c"
        "command_executor/command_executor.c"
        "command_executor/test_pulse_cmd.cpp"
    INCLUDE_DIRS
        "tasks/include"
        "command_executor/include"
    REQUIRES
        config
        command_parser
        event_manager
        freertos
        test_utils
        usb_cdc
        pulse_gen
        position
    PRIV_REQUIRES
        esp_tinyusb
        esp_timer
        log
        driver
)
