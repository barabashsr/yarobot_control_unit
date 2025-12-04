idf_component_register(
    SRCS "usb_cdc.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos config
    PRIV_REQUIRES esp_tinyusb esp_timer log
)
