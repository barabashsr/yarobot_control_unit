idf_component_register(
    SRCS "tpic6b595.c"
          "test/test_tpic6b595.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos config yarobot_hal unity
    PRIV_REQUIRES log esp_timer driver
    WHOLE_ARCHIVE
)
