# HAL umbrella component
# Aggregates all HAL sub-components for a unified build

idf_component_register(
    SRCS
        "gpio_hal/gpio_hal.c"
        "i2c_hal/i2c_hal.c"
        "spi_hal/spi_hal.c"
    INCLUDE_DIRS
        "gpio_hal/include"
        "i2c_hal/include"
        "spi_hal/include"
    REQUIRES
        config
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_spi
)
