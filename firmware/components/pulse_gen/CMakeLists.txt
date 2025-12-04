idf_component_register(
    SRCS "rmt_pulse_gen.cpp" "mcpwm_pulse_gen.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver config esp_driver_rmt esp_driver_mcpwm esp_driver_pcnt freertos esp_timer tpic6b595
)
