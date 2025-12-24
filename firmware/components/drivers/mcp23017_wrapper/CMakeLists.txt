idf_component_register(
    SRCS "mcp23017_wrapper.c"
          "test/test_mcp23017_wrapper.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos config espressif__mcp23017 espressif__i2c_bus unity
    PRIV_REQUIRES log esp_timer driver control
    WHOLE_ARCHIVE
)
