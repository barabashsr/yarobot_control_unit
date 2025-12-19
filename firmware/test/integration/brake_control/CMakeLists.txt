# Integration test for Brake Control System (Story 4-5)
# Run as standalone app: idf.py -C test/integration/brake_control build flash monitor

idf_component_register(
    SRCS "test_brake_control_integration.c"
    INCLUDE_DIRS "."
    REQUIRES
        unity
        control
        config
        freertos
        esp_timer
        event_manager
        command_parser
        mcp23017_wrapper
        tpic6b595
    PRIV_REQUIRES
        log
)
