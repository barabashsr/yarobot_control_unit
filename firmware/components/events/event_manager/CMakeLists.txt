idf_component_register(
    SRCS "event_manager.c"
          "test/test_event_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_timer config command_parser usb_cdc unity
    PRIV_REQUIRES log
    WHOLE_ARCHIVE
)
