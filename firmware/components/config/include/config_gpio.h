/**
 * @file config_gpio.h
 * @brief GPIO pin assignments for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note All GPIO assignments match the ESP32-S3-DevKitC-1 N16R8 board layout.
 *       See docs/gpio-assignment.md for physical pin mapping.
 *
 * @warning GPIO26-37 are reserved for Flash/PSRAM on N16R8 - DO NOT USE.
 * @warning GPIO19-20 are fixed for USB D-/D+ - cannot be reassigned.
 */

#ifndef CONFIG_GPIO_H
#define CONFIG_GPIO_H

#include "driver/gpio.h"

/**
 * @defgroup config_gpio GPIO Pin Assignments
 * @brief Hardware GPIO pin definitions for ESP32-S3-DevKitC-1 N16R8
 * @{
 */

/**
 * @defgroup gpio_step Motor STEP Pulse Outputs
 * @brief Step pulse output pins for each motor axis
 *
 * All STEP outputs are grouped on the LEFT side (J1) of the board.
 * Servo axes have corresponding Z_SIGNAL inputs on the RIGHT side (J3)
 * at the same row for clean cable routing.
 * @{
 */

/** @brief X-axis servo step output (RMT CH0) - J1 pin 4 */
#define GPIO_X_STEP         GPIO_NUM_4

/** @brief Y-axis servo step output (MCPWM T0) - J1 pin 5 */
#define GPIO_Y_STEP         GPIO_NUM_5

/** @brief Z-axis servo step output (RMT CH1) - J1 pin 6 */
#define GPIO_Z_STEP         GPIO_NUM_6

/** @brief A-axis servo step output (RMT CH2) - J1 pin 7 */
#define GPIO_A_STEP         GPIO_NUM_7

/** @brief B-axis servo step output (RMT CH3) - J1 pin 8 */
#define GPIO_B_STEP         GPIO_NUM_15

/** @brief C-axis stepper step output (MCPWM T1) - J1 pin 9 */
#define GPIO_C_STEP         GPIO_NUM_16

/** @brief D-axis stepper step output (LEDC CH0) - J1 pin 10 */
#define GPIO_D_STEP         GPIO_NUM_17

/** @} */ // end gpio_step

/**
 * @defgroup gpio_zsignal Servo Z-Signal (Index) Inputs
 * @brief Encoder index pulse inputs for servo axes
 *
 * All Z_SIGNAL inputs are on the RIGHT side (J3) of the board,
 * at the same row as the corresponding STEP output on the LEFT side.
 * Hardware interrupts are used for precise encoder index capture.
 * @{
 */

/** @brief X servo encoder index pulse input - J3 pin 4 */
#define GPIO_X_Z_SIGNAL     GPIO_NUM_1

/** @brief Y servo encoder index pulse input - J3 pin 5 */
#define GPIO_Y_Z_SIGNAL     GPIO_NUM_2

/** @brief Z servo encoder index pulse input - J3 pin 6 */
#define GPIO_Z_Z_SIGNAL     GPIO_NUM_42

/** @brief A servo encoder index pulse input - J3 pin 7 */
#define GPIO_A_Z_SIGNAL     GPIO_NUM_41

/** @brief B servo encoder index pulse input - J3 pin 8 */
#define GPIO_B_Z_SIGNAL     GPIO_NUM_40

/** @} */ // end gpio_zsignal

/**
 * @defgroup gpio_sr Shift Register SPI Control
 * @brief SPI pins for TPIC6B595N shift register chain (5x 8-bit = 40 bits)
 *
 * Grouped on LEFT side (J1 pins 15-18) for single cable to shift register board.
 * @{
 */

/** @brief Shift register output enable (active-low) - J1 pin 15 */
#define GPIO_SR_OE          GPIO_NUM_9

/** @brief Shift register latch/chip select - J1 pin 16 */
#define GPIO_SR_CS          GPIO_NUM_10

/** @brief Shift register SPI data (MOSI) - J1 pin 17 */
#define GPIO_SR_MOSI        GPIO_NUM_11

/** @brief Shift register SPI clock - J1 pin 18 */
#define GPIO_SR_SCLK        GPIO_NUM_12

/** @} */ // end gpio_sr

/**
 * @defgroup gpio_i2c I2C Bus 0 (MCP23017 Expanders)
 * @brief Main I2C bus for I/O expanders
 *
 * I2C lines grouped on LEFT side (J1 pins 11-12).
 * Connected to MCP23017 #0 (0x20) for limit switches and
 * MCP23017 #1 (0x21) for ALARM_INPUT and InPos signals.
 * @{
 */

/** @brief I2C bus 0 clock - J1 pin 11 */
#define GPIO_I2C_SCL        GPIO_NUM_18

/** @brief I2C bus 0 data - J1 pin 12 */
#define GPIO_I2C_SDA        GPIO_NUM_8

/** @} */ // end gpio_i2c

/**
 * @defgroup gpio_mcp_int MCP23017 Interrupt Lines
 * @brief Interrupt outputs from I2C expanders
 *
 * 4 interrupt lines for 2 MCP23017 devices (2 ports each).
 * INTA lines on LEFT side, INTB lines on RIGHT side.
 * @{
 */

/** @brief MCP23017 #0 Port A interrupt (X-A limits) - J1 pin 13 */
#define GPIO_MCP0_INTA      GPIO_NUM_3

/** @brief MCP23017 #0 Port B interrupt (B-E limits) - J1 pin 14 */
#define GPIO_MCP0_INTB      GPIO_NUM_46

/** @brief MCP23017 #1 Port A interrupt (ALARM_INPUT) - J3 pin 9 */
#define GPIO_MCP1_INTA      GPIO_NUM_39

/** @brief MCP23017 #1 Port B interrupt (InPos signals) - J3 pin 10 */
#define GPIO_MCP1_INTB      GPIO_NUM_38

/** @} */ // end gpio_mcp_int

/**
 * @defgroup gpio_safety Safety Signals
 * @brief Emergency stop and safety-critical inputs
 * @{
 */

/** @brief Emergency stop input (active-low) - J1 pin 19 */
#define GPIO_E_STOP         GPIO_NUM_13

/** @} */ // end gpio_safety

/**
 * @defgroup gpio_oled OLED Display I2C Bus 1
 * @brief Dedicated I2C bus for OLED display (isolated from main I2C)
 * @{
 */

/** @brief OLED I2C data (I2C_NUM_1) - J1 pin 20 */
#define GPIO_OLED_SDA       GPIO_NUM_14

/** @brief OLED I2C clock (I2C_NUM_1) - J3 pin 18 */
#define GPIO_OLED_SCL       GPIO_NUM_21

/** @} */ // end gpio_oled

/**
 * @defgroup gpio_usb USB Interface (Fixed)
 * @brief Native USB D+/D- pins - cannot be reassigned
 * @{
 */

/** @brief USB D- (fixed by hardware) - J3 pin 20 */
#define GPIO_USB_DN         GPIO_NUM_19

/** @brief USB D+ (fixed by hardware) - J3 pin 19 */
#define GPIO_USB_DP         GPIO_NUM_20

/** @} */ // end gpio_usb

/** @} */ // end config_gpio

#endif // CONFIG_GPIO_H
