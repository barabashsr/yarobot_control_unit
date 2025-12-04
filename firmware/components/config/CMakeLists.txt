# Configuration headers component for YaRobot Control Unit
# This is a header-only component providing all configuration constants

idf_component_register(
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_ledc
        esp_driver_mcpwm
        esp_driver_pcnt
        driver
)
