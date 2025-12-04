# Control umbrella component
# Aggregates all control sub-components for a unified build

idf_component_register(
    SRCS
        "tasks/task_stubs.c"
    INCLUDE_DIRS
        "tasks/include"
    REQUIRES
        config
        freertos
        test_utils
        usb_cdc
    PRIV_REQUIRES
        esp_tinyusb
        log
)
