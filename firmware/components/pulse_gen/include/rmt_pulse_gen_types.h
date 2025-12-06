/**
 * @file rmt_pulse_gen_types.h
 * @brief Type definitions for FastAccelStepper-pattern RMT implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Defines data structures for command queue and ramp generation.
 *       Based on FastAccelStepper patterns for proven reliability.
 */

#ifndef RMT_PULSE_GEN_TYPES_H
#define RMT_PULSE_GEN_TYPES_H

#include <cstdint>
#include <atomic>

/**
 * @defgroup rmt_pulse_gen_types RMT Pulse Generator Types
 * @brief Data structures for RMT-based pulse generation
 * @{
 */

/**
 * @brief Single motion command in the queue
 *
 * Matches FastAccelStepper queue_entry structure for compatibility.
 * Uses ticks (not frequency) for ISR efficiency - no division needed.
 *
 * @note Size: 4 bytes (packed for cache efficiency)
 */
struct StepCommand {
    uint16_t ticks;             ///< Period between steps in RMT ticks (16 MHz)
    uint8_t steps;              ///< Number of steps in this command (0 = pause)
    uint8_t flags;              ///< Control flags (see CMD_FLAG_*)
};

/**
 * @defgroup cmd_flags Command Flags
 * @brief Control flags for StepCommand
 * @{
 */

/** @brief Direction: 1=forward (positive), 0=reverse (negative) */
constexpr uint8_t CMD_FLAG_DIRECTION    = 0x01;

/** @brief Toggle direction before this command */
constexpr uint8_t CMD_FLAG_TOGGLE_DIR   = 0x02;

/** @brief Last command in motion sequence (triggers completion) */
constexpr uint8_t CMD_FLAG_LAST         = 0x04;

/** @} */ // end cmd_flags

/**
 * @brief Ramp generator state machine states
 *
 * @note Direction reversal is handled at MotorBase level, not here.
 *       See fastaccelstepper-migration-strategy.md Section 5.3.5.1.
 */
enum class RampState : uint8_t {
    IDLE,           ///< No motion, waiting for startMove/startVelocity
    ACCELERATING,   ///< Ramping up to target velocity
    CRUISING,       ///< At constant target velocity
    DECELERATING    ///< Ramping down to stop or new velocity
};

/**
 * @brief Motion profile parameters (atomic for thread safety)
 *
 * These parameters can be updated from any thread/task context.
 * The ramp generator reads them atomically for profile calculation.
 *
 * @note std::atomic ensures safe cross-thread access without mutex.
 */
struct RampParameters {
    std::atomic<float> target_velocity{0.0f};    ///< Target velocity (pulses/s)
    std::atomic<float> acceleration{0.0f};       ///< Acceleration rate (pulses/s^2)
    std::atomic<int32_t> target_position{0};     ///< Target position (pulses, absolute)
    std::atomic<bool> position_mode{false};      ///< True = position mode, false = velocity mode
    std::atomic<bool> params_changed{false};     ///< Flag: parameters updated, recalc needed
};

/**
 * @brief Queue state tracking for forward planning
 *
 * Tracks the state at the END of the command queue (future state).
 * Used by ramp generator to calculate deceleration points.
 *
 * @note volatile for ISR visibility (queue is read from ISR context)
 */
struct QueueState {
    volatile int32_t position;      ///< Position at queue end (planned, pulses)
    volatile bool direction;        ///< Direction at queue end
    volatile uint32_t ticks_queued; ///< Total ticks in queue (for forward planning)
};

/** @} */ // end rmt_pulse_gen_types

#endif // RMT_PULSE_GEN_TYPES_H
