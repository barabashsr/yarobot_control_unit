/**
 * @file config_i2c.h
 * @brief I2C addresses and MCP23017 pin mappings
 * @author YaRobot Team
 * @date 2025
 *
 * @note Defines I2C bus configuration and MCP23017 I/O expander pin assignments.
 *       All inputs are handled by MCP23017; outputs are on shift registers.
 *
 * @see docs/gpio-assignment.md for detailed pin mapping documentation.
 */

#ifndef CONFIG_I2C_H
#define CONFIG_I2C_H

#include "driver/i2c_types.h"

/**
 * @defgroup config_i2c I2C Configuration
 * @brief I2C bus settings and MCP23017 expander mappings
 * @{
 */

/**
 * @defgroup i2c_bus I2C Bus Configuration
 * @brief Main I2C bus settings for MCP23017 expanders
 * @{
 */

/** @brief I2C port for MCP23017 expanders */
#define I2C_PORT            I2C_NUM_0

/** @brief I2C bus frequency (400kHz Fast Mode) */
#define I2C_FREQ_HZ         400000

/** @} */ // end i2c_bus

/**
 * @defgroup i2c_addr MCP23017 I2C Addresses
 * @brief Device addresses for I2C I/O expanders
 *
 * Address pins (A0, A1, A2) are hardwired on PCB.
 * @{
 */

/** @brief MCP23017 #0 address - Limit switches (all 8 axes) */
#define I2C_ADDR_MCP23017_0     0x20

/** @brief MCP23017 #1 address - ALARM_INPUT + InPos signals */
#define I2C_ADDR_MCP23017_1     0x21

/** @} */ // end i2c_addr

/**
 * @defgroup mcp0_pins MCP23017 #0 Pin Mappings (0x20) - Limit Switches
 * @brief Port A and Port B assignments for limit switch inputs
 *
 * All 16 pins configured as inputs with pull-ups enabled.
 * Organized by axis pair (MIN/MAX adjacent) for logical grouping.
 * @{
 */

/**
 * @defgroup mcp0_porta MCP23017 #0 Port A (GPA0-GPA7)
 * @brief Limit switches for axes X, Y, Z, A (8 switches)
 * @{
 */

/** @brief X-axis MIN limit switch - GPA0 */
#define MCP0_X_LIMIT_MIN    0

/** @brief X-axis MAX limit switch - GPA1 */
#define MCP0_X_LIMIT_MAX    1

/** @brief Y-axis MIN limit switch - GPA2 */
#define MCP0_Y_LIMIT_MIN    2

/** @brief Y-axis MAX limit switch - GPA3 */
#define MCP0_Y_LIMIT_MAX    3

/** @brief Z-axis MIN limit switch - GPA4 */
#define MCP0_Z_LIMIT_MIN    4

/** @brief Z-axis MAX limit switch - GPA5 */
#define MCP0_Z_LIMIT_MAX    5

/** @brief A-axis MIN limit switch - GPA6 */
#define MCP0_A_LIMIT_MIN    6

/** @brief A-axis MAX limit switch - GPA7 */
#define MCP0_A_LIMIT_MAX    7

/** @} */ // end mcp0_porta

/**
 * @defgroup mcp0_portb MCP23017 #0 Port B (GPB0-GPB7)
 * @brief Limit switches for axes B, C, D, E (8 switches)
 * @{
 */

/** @brief B-axis MIN limit switch - GPB0 */
#define MCP0_B_LIMIT_MIN    8

/** @brief B-axis MAX limit switch - GPB1 */
#define MCP0_B_LIMIT_MAX    9

/** @brief C-axis MIN limit switch - GPB2 */
#define MCP0_C_LIMIT_MIN    10

/** @brief C-axis MAX limit switch (floating mode) - GPB3 */
#define MCP0_C_LIMIT_MAX    11

/** @brief D-axis MIN limit switch - GPB4 */
#define MCP0_D_LIMIT_MIN    12

/** @brief D-axis MAX limit switch - GPB5 */
#define MCP0_D_LIMIT_MAX    13

/** @brief E-axis MIN limit switch - GPB6 */
#define MCP0_E_LIMIT_MIN    14

/** @brief E-axis MAX limit switch - GPB7 */
#define MCP0_E_LIMIT_MAX    15

/** @} */ // end mcp0_portb

/** @} */ // end mcp0_pins

/**
 * @defgroup mcp1_pins MCP23017 #1 Pin Mappings (0x21) - ALARM + InPos
 * @brief Port A for ALARM_INPUT, Port B for InPos + spare inputs
 *
 * All 16 pins configured as inputs with pull-ups enabled.
 * @{
 */

/**
 * @defgroup mcp1_porta MCP23017 #1 Port A (GPA0-GPA7)
 * @brief ALARM_INPUT signals from motor drivers (inputs)
 *
 * @note ALARM_CLEAR outputs are on shift register chain (SR bits 3,7,11,15,19,23,27,31)
 * @{
 */

/** @brief X servo/driver alarm input - GPA0 */
#define MCP1_X_ALARM_INPUT  0

/** @brief Y servo/driver alarm input - GPA1 */
#define MCP1_Y_ALARM_INPUT  1

/** @brief Z servo/driver alarm input - GPA2 */
#define MCP1_Z_ALARM_INPUT  2

/** @brief A servo/driver alarm input - GPA3 */
#define MCP1_A_ALARM_INPUT  3

/** @brief B servo/driver alarm input - GPA4 */
#define MCP1_B_ALARM_INPUT  4

/** @brief C stepper driver alarm input - GPA5 */
#define MCP1_C_ALARM_INPUT  5

/** @brief D stepper driver alarm input - GPA6 */
#define MCP1_D_ALARM_INPUT  6

/** @brief General purpose input 0 - GPA7 */
#define MCP1_GP_IN_0        7

/** @} */ // end mcp1_porta

/**
 * @defgroup mcp1_portb MCP23017 #1 Port B (GPB0-GPB7)
 * @brief InPos (In-Position) signals from servo drives + spare inputs
 * @{
 */

/** @brief X servo in-position signal - GPB0 */
#define MCP1_X_INPOS        8

/** @brief Y servo in-position signal - GPB1 */
#define MCP1_Y_INPOS        9

/** @brief Z servo in-position signal - GPB2 */
#define MCP1_Z_INPOS        10

/** @brief A servo in-position signal - GPB3 */
#define MCP1_A_INPOS        11

/** @brief B servo in-position signal - GPB4 */
#define MCP1_B_INPOS        12

/** @brief General purpose input 1 - GPB5 */
#define MCP1_GP_IN_1        13

/** @brief General purpose input 2 - GPB6 */
#define MCP1_GP_IN_2        14

/** @brief General purpose input 3 - GPB7 */
#define MCP1_GP_IN_3        15

/** @} */ // end mcp1_portb

/** @} */ // end mcp1_pins

/** @} */ // end config_i2c

#endif // CONFIG_I2C_H
