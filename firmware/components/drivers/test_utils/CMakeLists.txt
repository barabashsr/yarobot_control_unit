idf_component_register(
    SRCS "test_utils.c"
    INCLUDE_DIRS "include"
    REQUIRES
        config
        yarobot_hal
        esp_driver_gpio
        usb_cdc
)
