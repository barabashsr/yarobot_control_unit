# Control umbrella component
# Aggregates all control sub-components for a unified build

idf_component_register(
    SRCS
        "tasks/task_stubs.c"
        "command_executor/command_executor.c"
        "command_executor/move_handler.cpp"
        "command_executor/movr_handler.cpp"
        "motion_controller/motion_controller.cpp"
        "motor_system/motor_system.cpp"
    # NOTE: test_pulse_cmd.cpp removed - was creating duplicate RMT/MCPWM/LEDC instances
    INCLUDE_DIRS
        "tasks/include"
        "command_executor/include"
        "motion_controller/include"
        "motor_system/include"
    REQUIRES
        config
        command_parser
        event_manager
        freertos
        test_utils
        usb_cdc
        pulse_gen
        position
        motor
    PRIV_REQUIRES
        esp_tinyusb
        esp_timer
        log
        driver
)
