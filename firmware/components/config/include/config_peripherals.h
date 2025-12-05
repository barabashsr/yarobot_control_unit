/**
 * @file config_peripherals.h
 * @brief ESP32 peripheral assignments for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note Defines RMT channels, MCPWM timers, PCNT units, LEDC channels,
 *       and SPI host assignments for motor control peripherals.
 */

#ifndef CONFIG_PERIPHERALS_H
#define CONFIG_PERIPHERALS_H

#include "driver/ledc.h"
#include "driver/spi_master.h"

/**
 * @defgroup config_peripherals ESP32 Peripheral Assignments
 * @brief Hardware peripheral configuration for motor control
 * @{
 */

/**
 * @defgroup periph_rmt RMT Channel Assignments
 * @brief RMT (Remote Control) peripheral for precise pulse generation
 *
 * Used for servo axes requiring accurate step pulse timing.
 * ESP32-S3 has 4 TX channels available.
 * @{
 */

/** @brief RMT channel for X-axis servo step pulses */
#define RMT_CHANNEL_X       0

/** @brief RMT channel for Z-axis servo step pulses */
#define RMT_CHANNEL_Z       1

/** @brief RMT channel for A-axis servo step pulses */
#define RMT_CHANNEL_A       2

/** @brief RMT channel for B-axis servo step pulses */
#define RMT_CHANNEL_B       3

/** @} */ // end periph_rmt

/**
 * @defgroup periph_mcpwm MCPWM Assignments
 * @brief Motor Control PWM for Y and C axes
 *
 * MCPWM provides hardware-based PWM with dead-time and fault handling.
 * @{
 */

/** @brief MCPWM group ID (ESP32-S3 has groups 0 and 1) */
#define MCPWM_GROUP_ID      0

/** @brief MCPWM timer for Y-axis servo */
#define MCPWM_TIMER_Y       0

/** @brief MCPWM timer for C-axis stepper */
#define MCPWM_TIMER_C       1

/**
 * @brief MCPWM timer resolution (Hz)
 *
 * 10 MHz clock provides 100ns resolution per tick.
 * At 100kHz: period = 10000000 / 100000 = 100 ticks
 * At 25kHz: period = 10000000 / 25000 = 400 ticks
 */
#define MCPWM_RESOLUTION_HZ     10000000

/**
 * @brief MCPWM duty cycle percentage (0-100)
 *
 * 50% duty cycle for step pulses (symmetric high/low)
 */
#define MCPWM_DUTY_CYCLE_PERCENT    50

/** @} */ // end periph_mcpwm

/**
 * @defgroup periph_pcnt PCNT Unit Assignments
 * @brief Pulse Counter for position verification
 *
 * PCNT units count pulses for cross-checking commanded vs actual pulses.
 * ESP32-S3 has 4 PCNT units (0-3).
 * @{
 */

/** @brief PCNT unit for Y-axis position counting */
#define PCNT_UNIT_Y         0

/** @brief PCNT unit for C-axis position counting */
#define PCNT_UNIT_C         1

/** @brief PCNT unit for D-axis position counting (via LEDC internal loopback) */
#define PCNT_UNIT_D         2

/** @} */ // end periph_pcnt

/**
 * @defgroup periph_ledc LEDC Assignments
 * @brief LED Controller PWM for D-axis stepper
 *
 * LEDC provides simple PWM output for step pulse generation.
 * @{
 */

/** @brief LEDC timer for D-axis stepper (using timer 2 to avoid conflicts) */
#define LEDC_TIMER_D        LEDC_TIMER_2

/** @brief LEDC channel for D-axis step output (using channel 2 to avoid conflicts) */
#define LEDC_CHANNEL_D      LEDC_CHANNEL_2

/** @brief LEDC speed mode (low-speed mode for ESP32-S3) */
#define LEDC_MODE_D         LEDC_LOW_SPEED_MODE

/**
 * @brief LEDC timer resolution in bits
 *
 * 10-bit resolution = 1024 duty levels
 * At 80 MHz APB clock: max freq = 80MHz / 1024 = ~78 kHz
 * Adequate for D-axis stepper which has lower precision requirements
 */
#define LEDC_RESOLUTION_BITS    10

/**
 * @brief LEDC duty cycle percentage (0-100)
 *
 * 50% duty cycle for symmetric step pulses (equal high/low time)
 */
#define LEDC_DUTY_CYCLE_PERCENT 50

/** @} */ // end periph_ledc

/**
 * @defgroup periph_spi SPI Host Assignment
 * @brief SPI bus for shift register chain
 *
 * Uses SPI2 (HSPI) for fast communication with TPIC6B595N shift registers.
 * @{
 */

/** @brief SPI host for shift register communication */
#define SPI_HOST_SR         SPI2_HOST

/** @} */ // end periph_spi

/** @} */ // end config_peripherals

#endif // CONFIG_PERIPHERALS_H
