# Control umbrella component
# Aggregates all control sub-components for a unified build

idf_component_register(
    SRCS
        "tasks/task_stubs.c"
        "command_executor/command_executor.c"
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
    PRIV_REQUIRES
        esp_tinyusb
        esp_timer
        log
)
