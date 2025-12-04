/**
 * @file gpio_hal.h
 * @brief GPIO Hardware Abstraction Layer for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note Provides a hardware abstraction for GPIO operations.
 *       Stub implementation returns ESP_OK without side effects.
 */

#ifndef GPIO_HAL_H
#define GPIO_HAL_H

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup gpio_hal GPIO Hardware Abstraction Layer
 * @brief Hardware abstraction for GPIO operations
 * @{
 */

/**
 * @brief Initialize the GPIO HAL
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note This is a stub implementation - returns ESP_OK without side effects
 */
esp_err_t gpio_hal_init(void);

/**
 * @brief Set GPIO pin direction
 *
 * @param[in] pin GPIO pin number
 * @param[in] mode GPIO mode (input, output, etc.)
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note This is a stub implementation - returns ESP_OK without side effects
 */
esp_err_t gpio_hal_set_direction(gpio_num_t pin, gpio_mode_t mode);

/**
 * @brief Set GPIO pin output level
 *
 * @param[in] pin GPIO pin number
 * @param[in] level Output level (0 = low, non-zero = high)
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note This is a stub implementation - returns ESP_OK without side effects
 */
esp_err_t gpio_hal_set_level(gpio_num_t pin, uint32_t level);

/**
 * @brief Get GPIO pin input level
 *
 * @param[in] pin GPIO pin number
 *
 * @return int Pin level (0 or 1), returns 0 as placeholder
 *
 * @note This is a stub implementation - returns 0 without side effects
 */
int gpio_hal_get_level(gpio_num_t pin);

/**
 * @brief Configure GPIO interrupt
 *
 * @param[in] pin GPIO pin number
 * @param[in] type Interrupt type (rising edge, falling edge, etc.)
 * @param[in] handler ISR handler function
 * @param[in] arg Argument to pass to handler
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note This is a stub implementation - returns ESP_OK without side effects
 */
esp_err_t gpio_hal_set_interrupt(gpio_num_t pin, gpio_int_type_t type,
                                  gpio_isr_t handler, void *arg);

/** @} */ // end gpio_hal

#ifdef __cplusplus
}
#endif

#endif // GPIO_HAL_H
