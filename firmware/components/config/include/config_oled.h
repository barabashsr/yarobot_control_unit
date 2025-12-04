/**
 * @file config_oled.h
 * @brief OLED display configuration
 * @author YaRobot Team
 * @date 2025
 *
 * @note Defines I2C bus 1 settings and display parameters for the OLED module.
 *       The OLED uses a dedicated I2C bus isolated from the main MCP23017 bus.
 *
 * @see config_gpio.h for GPIO_OLED_SDA and GPIO_OLED_SCL pin assignments.
 */

#ifndef CONFIG_OLED_H
#define CONFIG_OLED_H

#include "driver/i2c_types.h"

/**
 * @defgroup config_oled OLED Display Configuration
 * @brief OLED display I2C and layout parameters
 * @{
 */

/**
 * @defgroup oled_i2c OLED I2C Bus Configuration
 * @brief I2C settings for dedicated OLED bus (I2C_NUM_1)
 * @{
 */

/** @brief I2C port for OLED display (separate from MCP23017 bus) */
#define I2C_OLED_PORT       I2C_NUM_1

/** @brief OLED I2C bus frequency (400kHz Fast Mode) */
#define I2C_OLED_FREQ_HZ    400000

/** @brief OLED display I2C address (SSD1306) */
#define OLED_ADDRESS        0x3C

/** @} */ // end oled_i2c

/**
 * @defgroup oled_display Display Parameters
 * @brief Physical display dimensions and characteristics
 * @{
 */

/** @brief Display width in pixels */
#define OLED_WIDTH          128

/** @brief Display height in pixels */
#define OLED_HEIGHT         64

/**
 * @brief Display refresh rate (Hz)
 *
 * 2 Hz is sufficient for status display - reduces I2C bus load.
 */
#define OLED_UPDATE_HZ      2

/** @} */ // end oled_display

/**
 * @defgroup oled_layout Layout Constants
 * @brief Text layout parameters for the display
 * @{
 */

/** @brief Default font width in pixels (6x8 font) */
#define OLED_FONT_WIDTH     6

/** @brief Default font height in pixels (6x8 font) */
#define OLED_FONT_HEIGHT    8

/** @brief Characters per line (128 / 6 = 21) */
#define OLED_CHARS_PER_LINE 21

/** @brief Number of text lines (64 / 8 = 8) */
#define OLED_NUM_LINES      8

/** @} */ // end oled_layout

/**
 * @defgroup oled_timing Display Timing
 * @brief Timing values for OLED display behavior
 * @{
 */

/**
 * @brief Event message display duration (ms)
 *
 * How long to show event messages (errors, limits, etc.) before
 * reverting to normal status display.
 */
#define OLED_EVENT_DISPLAY_MS   2000

/** @} */ // end oled_timing

/** @} */ // end config_oled

#endif // CONFIG_OLED_H
