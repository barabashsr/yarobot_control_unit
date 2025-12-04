idf_component_register(
    SRCS "rmt_pulse_gen.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver config esp_driver_rmt freertos esp_timer
)
