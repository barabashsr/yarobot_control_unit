# Control umbrella component
# Aggregates all control sub-components for a unified build

idf_component_register(
    SRCS
        "tasks/task_stubs.c"
        "command_executor/command_executor.c"
        "command_executor/move_handler.cpp"
        "command_executor/movr_handler.cpp"
        "command_executor/enable_handler.cpp"
        "command_executor/position_handler.cpp"
        "command_executor/velocity_handler.cpp"
        "command_executor/stop_handler.cpp"
        "command_executor/brake_handler.cpp"
        "motion_controller/motion_controller.cpp"
        "motor_system/motor_system.cpp"
        "safety_monitor/safety_monitor.c"
        "safety_monitor/lim_handler.c"
        "safety_monitor/brake_controller.c"
    # NOTE: test_pulse_cmd.cpp removed - was creating duplicate RMT/MCPWM/LEDC instances
    INCLUDE_DIRS
        "tasks/include"
        "command_executor/include"
        "motion_controller/include"
        "motor_system/include"
        "safety_monitor/include"
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
        mcp23017_wrapper
        tpic6b595
    PRIV_REQUIRES
        esp_tinyusb
        esp_timer
        log
        driver
)
