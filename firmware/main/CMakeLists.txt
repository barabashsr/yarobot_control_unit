# TEMP: GDMA probe - comment out to restore normal build
idf_component_register(
    SRCS "gdma_probe.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_hw_support soc hal
)

# ORIGINAL:
# idf_component_register(
#     SRCS "yarobot_control_unit.cpp"
#     INCLUDE_DIRS "."
#     REQUIRES config control yarobot_hal usb_cdc
# )
