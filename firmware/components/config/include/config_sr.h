/**
 * @file config_sr.h
 * @brief Shift register bit positions and helper macros
 * @author YaRobot Team
 * @date 2025
 *
 * @note Defines bit positions for 5x TPIC6B595N shift register chain (40 bits).
 *       Organization: 4 bits per axis [DIR, EN, BRAKE, ALARM_CLR] + 8 GP outputs.
 *
 * @warning TPIC6B595N outputs are open-drain with internal clamp diodes.
 *          Power loss = all outputs LOW = all brakes ENGAGED (fail-safe).
 */

#ifndef CONFIG_SR_H
#define CONFIG_SR_H

#include <stdint.h>

/**
 * @defgroup config_sr Shift Register Configuration
 * @brief Bit positions and macros for TPIC6B595N shift register chain
 * @{
 */

/**
 * @defgroup sr_bits Shift Register Bit Positions
 * @brief Per-axis bit assignments (4 bits per axis)
 *
 * Chain: MOSI -> SR0 -> SR1 -> SR2 -> SR3 -> SR4
 * Total: 40 bits (5 x 8-bit TPIC6B595N)
 *
 * Each axis uses 4 consecutive bits:
 * - Bit 0: DIR (Direction: 1=Forward)
 * - Bit 1: EN (Enable: 1=Enabled)
 * - Bit 2: BRAKE (Brake: 1=Released)
 * - Bit 3: ALARM_CLR (Alarm Clear: pulse to clear)
 * @{
 */

/**
 * @defgroup sr_x X-axis (Servo) - SR0.Q0-Q3
 * @{
 */
/** @brief X-axis direction - SR0.Q0 */
#define SR_X_DIR            0
/** @brief X-axis enable - SR0.Q1 */
#define SR_X_EN             1
/** @brief X-axis brake release - SR0.Q2 */
#define SR_X_BRAKE          2
/** @brief X-axis alarm clear - SR0.Q3 */
#define SR_X_ALARM_CLR      3
/** @} */

/**
 * @defgroup sr_y Y-axis (Servo) - SR0.Q4-Q7
 * @{
 */
/** @brief Y-axis direction - SR0.Q4 */
#define SR_Y_DIR            4
/** @brief Y-axis enable - SR0.Q5 */
#define SR_Y_EN             5
/** @brief Y-axis brake release - SR0.Q6 */
#define SR_Y_BRAKE          6
/** @brief Y-axis alarm clear - SR0.Q7 */
#define SR_Y_ALARM_CLR      7
/** @} */

/**
 * @defgroup sr_z Z-axis (Servo) - SR1.Q0-Q3
 * @{
 */
/** @brief Z-axis direction - SR1.Q0 */
#define SR_Z_DIR            8
/** @brief Z-axis enable - SR1.Q1 */
#define SR_Z_EN             9
/** @brief Z-axis brake release - SR1.Q2 */
#define SR_Z_BRAKE          10
/** @brief Z-axis alarm clear - SR1.Q3 */
#define SR_Z_ALARM_CLR      11
/** @} */

/**
 * @defgroup sr_a A-axis (Servo) - SR1.Q4-Q7
 * @{
 */
/** @brief A-axis direction - SR1.Q4 */
#define SR_A_DIR            12
/** @brief A-axis enable - SR1.Q5 */
#define SR_A_EN             13
/** @brief A-axis brake release - SR1.Q6 */
#define SR_A_BRAKE          14
/** @brief A-axis alarm clear - SR1.Q7 */
#define SR_A_ALARM_CLR      15
/** @} */

/**
 * @defgroup sr_b B-axis (Servo) - SR2.Q0-Q3
 * @{
 */
/** @brief B-axis direction - SR2.Q0 */
#define SR_B_DIR            16
/** @brief B-axis enable - SR2.Q1 */
#define SR_B_EN             17
/** @brief B-axis brake release - SR2.Q2 */
#define SR_B_BRAKE          18
/** @brief B-axis alarm clear - SR2.Q3 */
#define SR_B_ALARM_CLR      19
/** @} */

/**
 * @defgroup sr_c C-axis (Stepper) - SR2.Q4-Q7
 * @note Stepper axes have no physical brake - BRAKE bit not connected
 * @{
 */
/** @brief C-axis direction - SR2.Q4 */
#define SR_C_DIR            20
/** @brief C-axis enable - SR2.Q5 */
#define SR_C_EN             21
/** @brief C-axis brake (not connected on steppers) - SR2.Q6 */
#define SR_C_BRAKE          22
/** @brief C-axis alarm clear - SR2.Q7 */
#define SR_C_ALARM_CLR      23
/** @} */

/**
 * @defgroup sr_d D-axis (Stepper) - SR3.Q0-Q3
 * @note Stepper axes have no physical brake - BRAKE bit not connected
 * @{
 */
/** @brief D-axis direction - SR3.Q0 */
#define SR_D_DIR            24
/** @brief D-axis enable - SR3.Q1 */
#define SR_D_EN             25
/** @brief D-axis brake (not connected on steppers) - SR3.Q2 */
#define SR_D_BRAKE          26
/** @brief D-axis alarm clear - SR3.Q3 */
#define SR_D_ALARM_CLR      27
/** @} */

/**
 * @defgroup sr_e E-axis (Discrete) - SR3.Q4-Q7
 * @{
 */
/** @brief E-axis direction - SR3.Q4 */
#define SR_E_DIR            28
/** @brief E-axis enable - SR3.Q5 */
#define SR_E_EN             29
/** @brief E-axis brake release - SR3.Q6 */
#define SR_E_BRAKE          30
/** @brief E-axis alarm clear - SR3.Q7 */
#define SR_E_ALARM_CLR      31
/** @} */

/**
 * @defgroup sr_gp General Purpose Outputs - SR4.Q0-Q7
 * @brief 8 user-configurable digital outputs
 * @{
 */
/** @brief General purpose output 0 - SR4.Q0 */
#define SR_GP_OUT_0         32
/** @brief General purpose output 1 - SR4.Q1 */
#define SR_GP_OUT_1         33
/** @brief General purpose output 2 - SR4.Q2 */
#define SR_GP_OUT_2         34
/** @brief General purpose output 3 - SR4.Q3 */
#define SR_GP_OUT_3         35
/** @brief General purpose output 4 - SR4.Q4 */
#define SR_GP_OUT_4         36
/** @brief General purpose output 5 - SR4.Q5 */
#define SR_GP_OUT_5         37
/** @brief General purpose output 6 - SR4.Q6 */
#define SR_GP_OUT_6         38
/** @brief General purpose output 7 - SR4.Q7 */
#define SR_GP_OUT_7         39
/** @} */

/** @} */ // end sr_bits

/**
 * @defgroup sr_macros Shift Register Helper Macros
 * @brief Macros for computing bit positions and manipulating SR data
 * @{
 */

/**
 * @brief Get DIR bit position for axis index (0-7)
 * @param axis Axis index (0=X, 1=Y, ..., 7=E)
 * @return Bit position for direction signal
 */
#define SR_DIR_BIT(axis)       ((axis) * 4 + 0)

/**
 * @brief Get EN bit position for axis index (0-7)
 * @param axis Axis index (0=X, 1=Y, ..., 7=E)
 * @return Bit position for enable signal
 */
#define SR_EN_BIT(axis)        ((axis) * 4 + 1)

/**
 * @brief Get BRAKE bit position for axis index (0-7)
 * @param axis Axis index (0=X, 1=Y, ..., 7=E)
 * @return Bit position for brake release signal
 */
#define SR_BRAKE_BIT(axis)     ((axis) * 4 + 2)

/**
 * @brief Get ALARM_CLR bit position for axis index (0-7)
 * @param axis Axis index (0=X, 1=Y, ..., 7=E)
 * @return Bit position for alarm clear signal
 */
#define SR_ALARM_CLR_BIT(axis) ((axis) * 4 + 3)

/** @} */ // end sr_macros

/**
 * @defgroup sr_bit_ops Bit Manipulation Macros
 * @brief Set, clear, and test individual bits (use uint64_t for 40-bit support)
 * @{
 */

/**
 * @brief Set a bit in the shift register data
 * @param data 64-bit shift register data variable
 * @param bit Bit position (0-39)
 * @return Modified data with bit set
 */
#define SR_SET_BIT(data, bit)   ((data) | (1ULL << (bit)))

/**
 * @brief Clear a bit in the shift register data
 * @param data 64-bit shift register data variable
 * @param bit Bit position (0-39)
 * @return Modified data with bit cleared
 */
#define SR_CLR_BIT(data, bit)   ((data) & ~(1ULL << (bit)))

/**
 * @brief Get the state of a bit in shift register data
 * @param data 64-bit shift register data variable
 * @param bit Bit position (0-39)
 * @return 1 if bit is set, 0 otherwise
 */
#define SR_GET_BIT(data, bit)   (((data) >> (bit)) & 1)

/** @} */ // end sr_bit_ops

/**
 * @defgroup sr_defaults Fail-Safe State Definitions
 * @brief Default states for safety-critical behavior
 *
 * On power-up or reset, all TPIC6B595N outputs go LOW (open-drain).
 * This means: all enables OFF, all brakes ENGAGED (spring-applied).
 * @{
 */

/**
 * @brief Number of TPIC6B595N chips in the daisy chain
 */
#define SR_CHAIN_LENGTH     5

/**
 * @brief Total number of bits in the shift register chain (5 chips × 8 bits)
 */
#define SR_BITS_TOTAL       40

/**
 * @brief Safe state value (all bits 0)
 *
 * All enables OFF, all brakes engaged, all alarm clears inactive.
 * This is the power-on default and emergency state.
 */
#define SR_SAFE_STATE       0x0000000000ULL

/**
 * @brief Mask of all enable bits
 *
 * Useful for enabling/disabling all axes at once.
 */
#define SR_ALL_EN           ((1ULL << SR_X_EN) | (1ULL << SR_Y_EN) | \
                             (1ULL << SR_Z_EN) | (1ULL << SR_A_EN) | \
                             (1ULL << SR_B_EN) | (1ULL << SR_C_EN) | \
                             (1ULL << SR_D_EN) | (1ULL << SR_E_EN))

/**
 * @brief Mask of all brake release bits (servo axes only)
 *
 * Stepper axes (C, D) don't have physical brakes.
 */
#define SR_ALL_BRAKE_REL    ((1ULL << SR_X_BRAKE) | (1ULL << SR_Y_BRAKE) | \
                             (1ULL << SR_Z_BRAKE) | (1ULL << SR_A_BRAKE) | \
                             (1ULL << SR_B_BRAKE) | (1ULL << SR_E_BRAKE))

/** @} */ // end sr_defaults

/** @} */ // end config_sr

#endif // CONFIG_SR_H
