/**
 * @file config_axes.h
 * @brief Axis identifiers and mapping constants
 * @author YaRobot Team
 * @date 2025
 *
 * @note Defines axis ID constants for use throughout the control system.
 *       Use these constants instead of hardcoded numbers per AC14.
 */

#ifndef CONFIG_AXES_H
#define CONFIG_AXES_H

#include "config_limits.h"

/**
 * @defgroup config_axes Axis Identifiers
 * @brief Axis ID constants for motor array indexing
 * @{
 */

/**
 * @defgroup axis_ids Axis ID Constants
 * @brief Integer identifiers for each axis (0-7)
 *
 * These constants map to the motors_[] array index in MotionController.
 * Use these instead of magic numbers (e.g., motors_[AXIS_X] not motors_[0]).
 * @{
 */

/** @brief X-axis identifier (index 0) */
#define AXIS_X      0

/** @brief Y-axis identifier (index 1) */
#define AXIS_Y      1

/** @brief Z-axis identifier (index 2) */
#define AXIS_Z      2

/** @brief A-axis identifier (index 3) */
#define AXIS_A      3

/** @brief B-axis identifier (index 4) */
#define AXIS_B      4

/** @brief C-axis identifier (index 5) */
#define AXIS_C      5

/** @brief D-axis identifier (index 6) */
#define AXIS_D      6

/** @brief E-axis identifier (index 7) */
#define AXIS_E      7

/** @} */ // end axis_ids

/**
 * @defgroup axis_chars Axis Character Constants
 * @brief Character representations for each axis
 * @{
 */

/** @brief X-axis character */
#define AXIS_CHAR_X     'X'

/** @brief Y-axis character */
#define AXIS_CHAR_Y     'Y'

/** @brief Z-axis character */
#define AXIS_CHAR_Z     'Z'

/** @brief A-axis character */
#define AXIS_CHAR_A     'A'

/** @brief B-axis character */
#define AXIS_CHAR_B     'B'

/** @brief C-axis character */
#define AXIS_CHAR_C     'C'

/** @brief D-axis character */
#define AXIS_CHAR_D     'D'

/** @brief E-axis character */
#define AXIS_CHAR_E     'E'

/** @} */ // end axis_chars

/**
 * @defgroup axis_validation Axis Validation
 * @brief Compile-time validation of axis constants
 * @{
 */

/**
 * @brief Validate axis ID is within range
 * @param idx Axis index to validate
 * @return true if 0 <= idx < LIMIT_NUM_AXES
 */
#define AXIS_IS_VALID(idx) ((idx) >= 0 && (idx) < LIMIT_NUM_AXES)

/**
 * @brief Check if axis has servo feedback (InPos signal)
 * @param axis Axis index to check
 * @return true for servo axes (X,Y,Z,A,B), false for stepper/discrete axes (C,D,E)
 *
 * Servo axes X,Y,Z,A,B have InPos (in-position) feedback signals from servo drivers.
 * Stepper axes C,D and discrete axis E have no servo feedback hardware.
 * @note Story 4-8: Used by servo_feedback module and INPOS command handler.
 */
#define AXIS_HAS_SERVO_FEEDBACK(axis) ((axis) <= AXIS_B)

/**
 * @brief Check if axis has floating switch (object width measurement)
 * @param axis Axis index to check
 * @return true only for C axis (picker jaw), false for all other axes
 *
 * Only C axis has a floating switch for object width measurement.
 * The floating switch uses C_LIMIT_MAX input (MCP0 GPB3) and triggers
 * width measurement during closing motion without setting error state.
 * @note Story 4-9: Used by floating_switch module and WIDTH command handler.
 */
#define AXIS_HAS_FLOATING_SWITCH(axis) ((axis) == AXIS_C)

/**
 * @brief Validate E axis is the last axis
 *
 * E-axis (discrete actuator) must be the last axis index.
 * This invariant is assumed by motor type checks.
 */
static_assert(AXIS_E == LIMIT_NUM_AXES - 1,
              "AXIS_E must be the last axis index");

/** @} */ // end axis_validation

/** @} */ // end config_axes

#endif // CONFIG_AXES_H
