/**
 * @file config_commands.h
 * @brief Command strings, response prefixes, error codes, and event types
 * @author YaRobot Team
 * @date 2025
 *
 * @note Defines the complete command protocol for USB communication.
 *       All string literals for commands, responses, errors, and events.
 */

#ifndef CONFIG_COMMANDS_H
#define CONFIG_COMMANDS_H

/**
 * @defgroup config_commands Command Protocol Definitions
 * @brief USB command/response protocol strings and codes
 * @{
 */

/**
 * @defgroup cmd_strings Command Strings
 * @brief ASCII command identifiers received from host
 * @{
 */

/** @brief Move to absolute position */
#define CMD_MOVE            "MOVE"

/** @brief Move relative distance */
#define CMD_MOVR            "MOVR"

/** @brief Velocity mode (continuous motion) */
#define CMD_VEL             "VEL"

/** @brief Stop motion */
#define CMD_STOP            "STOP"

/** @brief Query current position */
#define CMD_POS             "POS"

/** @brief Query axis status */
#define CMD_STAT            "STAT"

/** @brief Query system information */
#define CMD_INFO            "INFO"

/** @brief Enable/disable axis */
#define CMD_EN              "EN"

/** @brief Brake control */
#define CMD_BRAKE           "BRAKE"

/** @brief Acknowledge position loss (Story 4-6) */
#define CMD_POSOK           "POSOK"

/** @brief Execute homing sequence */
#define CMD_HOME            "HOME"

/** @brief Set position to zero */
#define CMD_ZERO            "ZERO"

/** @brief Start calibration */
#define CMD_CALB            "CALB"

/** @brief Get axis travel width */
#define CMD_GETWIDTH        "GETWIDTH"

/** @brief Set position units */
#define CMD_SETU            "SETU"

/** @brief Set position limits */
#define CMD_SETL            "SETL"

/** @brief Set velocity limit */
#define CMD_SETV            "SETV"

/** @brief Set backlash compensation */
#define CMD_SETB            "SETB"

/** @brief Set axis alias */
#define CMD_ALIAS           "ALIAS"

/** @brief Read digital input */
#define CMD_DIN             "DIN"

/** @brief Write digital output */
#define CMD_DOUT            "DOUT"

/** @brief Query servo InPos (in-position) status (Story 4-8) */
#define CMD_INPOS           "INPOS"

/** @brief Query C-axis floating switch width measurement (Story 4-9) */
#define CMD_WIDTH           "WIDTH"

/** @brief Query error counts (Story 4-10) */
#define CMD_ERRCNT          "ERRCNT"

/** @brief Clear error state (Story 4-10) */
#define CMD_CLRERR          "CLRERR"

/** @brief I2C health status and scan (Story 4-11) */
#define CMD_I2C             "I2C"

/** @brief Set operation mode */
#define CMD_MODE            "MODE"

/** @brief Save configuration to NVS */
#define CMD_SAVE            "SAVE"

/** @brief Load configuration from NVS */
#define CMD_LOAD            "LOAD"

/** @brief Reset to defaults */
#define CMD_RST             "RST"

/** @brief Echo test command */
#define CMD_ECHO            "ECHO"

/** @brief Hardware test mode */
#define CMD_TEST            "TEST"

/** @brief Set log level */
#define CMD_LOG             "LOG"

/** @brief Diagnostic output */
#define CMD_DIAG            "DIAG"

/** @brief Streaming mode control */
#define CMD_STREAM          "STREAM"

/** @brief Clear alarm/fault */
#define CMD_CLR             "CLR"

/** @brief Query limit switch states */
#define CMD_LIM             "LIM"

/** @brief Start YAML configuration upload */
#define CMD_CFGSTART        "CFGSTART"

/** @brief YAML configuration data chunk */
#define CMD_CFGDATA         "CFGDATA"

/** @brief End YAML configuration upload */
#define CMD_CFGEND          "CFGEND"

/** @brief Export current configuration as YAML */
#define CMD_CFGEXPORT       "CFGEXPORT"

/** @} */ // end cmd_strings

/**
 * @defgroup resp_prefixes Response Prefixes
 * @brief Prefixes for response messages sent to host
 * @{
 */

/** @brief Success response prefix */
#define RESP_OK             "OK"

/** @brief Error response prefix */
#define RESP_ERROR          "ERROR"

/** @brief Asynchronous event prefix */
#define RESP_EVENT          "EVENT"

/** @brief YAML data response prefix */
#define RESP_YAML           "YAML:"

/** @brief Streaming data prefix */
#define RESP_STREAM         "STRM"

/** @} */ // end resp_prefixes

/**
 * @defgroup err_codes Error Codes
 * @brief Error code identifiers (E001-E0xx format)
 * @{
 */

/** @brief Invalid or unknown command */
#define ERR_INVALID_COMMAND         "E001"

/** @brief Invalid axis identifier */
#define ERR_INVALID_AXIS            "E002"

/** @brief Invalid parameter value */
#define ERR_INVALID_PARAMETER       "E003"

/** @brief Axis not enabled */
#define ERR_AXIS_NOT_ENABLED        "E004"

/** @brief Position limit exceeded */
#define ERR_POSITION_LIMIT          "E005"

/** @brief Emergency stop active */
#define ERR_EMERGENCY_STOP          "E006"

/** @brief Calibration/homing required */
#define ERR_CALIBRATION_REQUIRED    "E007"

/** @brief Motor driver fault */
#define ERR_MOTOR_FAULT             "E008"

/** @brief Communication error */
#define ERR_COMMUNICATION           "E009"

/** @brief Configuration error */
#define ERR_CONFIGURATION           "E010"

/** @brief Event buffer overflow */
#define ERR_EVENT_OVERFLOW          "E011"

/** @brief Command blocked in current mode */
#define ERR_MODE_BLOCKED            "E012"

/** @brief Motion active - stop first */
#define ERR_MOTION_ACTIVE           "E013"

/** @brief Driver alarm active */
#define ERR_DRIVER_ALARM            "E014"

/** @brief Alarm clear operation failed */
#define ERR_ALARM_CLEAR_FAILED      "E015"

/** @brief Z-signal drift exceeded threshold */
#define ERR_ZSYNC_DRIFT             "E016"

/** @brief CFGSTART rejected - axes moving */
#define ERR_CONFIG_AXES_MOVING      "E017"

/** @brief YAML parse error */
#define ERR_CONFIG_PARSE_FAILED     "E018"

/** @brief YAML validation failed */
#define ERR_CONFIG_VALIDATION       "E019"

/** @brief YAML buffer exceeded 8KB limit */
#define ERR_CONFIG_BUFFER_OVERFLOW  "E030"

/** @brief System error state - requires reset */
#define ERR_SYSTEM_ERROR            "E031"

/** @brief Limit switch active - motion blocked in this direction (Story 4-3) */
#define ERR_LIMIT_ACTIVE            "E032"

/** @brief Both limits active - axis faulted (Story 4-3 ADR-024) */
#define ERR_BOTH_LIMITS_FAULT       "E033"

/** @brief E-stop button still pressed during RST (Story 4-4) */
#define ERR_ESTOP_ACTIVE            "E034"

/** @brief Brake is auto-managed - manual control rejected (Story 4-5) */
#define ERR_BRAKE_AUTO              "E035"

/** @brief Axis has no brake hardware (Story 4-5) */
#define ERR_AXIS_NO_BRAKE           "E036"

/** @brief Position may be lost - homing or POSOK required (Story 4-6) */
#define ERR_POSLOS                  "E037"

/** @brief Invalid pin number for DIN/DOUT command (Story 4-7) */
#define ERR_INVALID_PIN             "E038"

/** @brief Unknown alias for DIN/DOUT command (Story 4-7) */
#define ERR_UNKNOWN_ALIAS           "E039"

/** @brief WIDTH command only supports C axis (Story 4-9) */
#define ERR_WIDTH_C_ONLY            "E040"

/** @brief Axis is in ERROR state - clear error first (Story 4-10) */
#define ERR_AXIS_ERROR              "E041"

/** @brief I2C communication failure (Story 4-10) */
#define ERR_I2C_FAILURE             "E020"

/** @brief I2C transaction timeout (Story 4-11) */
#define ERR_I2C_TIMEOUT             "E021"

/** @brief I2C device offline (Story 4-11) */
#define ERR_I2C_DEVICE_OFFLINE      "E022"

/** @} */ // end err_codes

/**
 * @defgroup err_messages Error Messages
 * @brief Human-readable error message strings
 * @{
 */

/** @brief Message for ERR_INVALID_COMMAND */
#define MSG_INVALID_COMMAND         "Invalid command"

/** @brief Message for ERR_INVALID_AXIS */
#define MSG_INVALID_AXIS            "Invalid axis"

/** @brief Message for ERR_INVALID_PARAMETER */
#define MSG_INVALID_PARAMETER       "Invalid parameter"

/** @brief Message for ERR_AXIS_NOT_ENABLED */
#define MSG_AXIS_NOT_ENABLED        "Axis not enabled"

/** @brief Message for ERR_POSITION_LIMIT */
#define MSG_POSITION_LIMIT          "Position limit exceeded"

/** @brief Message for ERR_EMERGENCY_STOP */
#define MSG_EMERGENCY_STOP          "Emergency stop active"

/** @brief Message for ERR_CALIBRATION_REQUIRED */
#define MSG_CALIBRATION_REQUIRED    "Calibration required"

/** @brief Message for ERR_MOTOR_FAULT */
#define MSG_MOTOR_FAULT             "Motor fault"

/** @brief Message for ERR_COMMUNICATION */
#define MSG_COMMUNICATION           "Communication error"

/** @brief Message for ERR_CONFIGURATION */
#define MSG_CONFIGURATION           "Configuration error"

/** @brief Message for ERR_EVENT_OVERFLOW */
#define MSG_EVENT_OVERFLOW          "Event buffer overflow"

/** @brief Message for ERR_MODE_BLOCKED */
#define MSG_MODE_BLOCKED            "Command blocked in current mode"

/** @brief Message for ERR_MOTION_ACTIVE */
#define MSG_MOTION_ACTIVE           "Motion active - stop first"

/** @brief Message for ERR_DRIVER_ALARM */
#define MSG_DRIVER_ALARM            "Driver alarm active"

/** @brief Message for ERR_ALARM_CLEAR_FAILED */
#define MSG_ALARM_CLEAR_FAILED      "Alarm clear failed"

/** @brief Message for ERR_ZSYNC_DRIFT */
#define MSG_ZSYNC_DRIFT             "Z-signal drift exceeded threshold"

/** @brief Message for ERR_CONFIG_AXES_MOVING */
#define MSG_CONFIG_AXES_MOVING      "Stop all axes before configuration"

/** @brief Message for ERR_CONFIG_PARSE_FAILED */
#define MSG_CONFIG_PARSE_FAILED     "YAML parse error"

/** @brief Message for ERR_CONFIG_VALIDATION */
#define MSG_CONFIG_VALIDATION       "Configuration validation failed"

/** @brief Message for ERR_CONFIG_BUFFER_OVERFLOW */
#define MSG_CONFIG_BUFFER_OVERFLOW  "Config exceeds 8KB buffer limit"

/** @brief Message for ERR_SYSTEM_ERROR */
#define MSG_SYSTEM_ERROR            "System in error state"

/** @brief Message for ERR_LIMIT_ACTIVE (Story 4-3) */
#define MSG_LIMIT_ACTIVE            "Limit switch active - motion blocked"

/** @brief Message for ERR_BOTH_LIMITS_FAULT (Story 4-3) */
#define MSG_BOTH_LIMITS_FAULT       "Both limits active - axis faulted"

/** @brief Message for ERR_ESTOP_ACTIVE (Story 4-4) */
#define MSG_ESTOP_ACTIVE            "E-stop button still pressed"

/** @brief Message for ERR_BRAKE_AUTO (Story 4-5) */
#define MSG_BRAKE_AUTO              "Brake is auto-managed"

/** @brief Message for ERR_AXIS_NO_BRAKE (Story 4-5) */
#define MSG_AXIS_NO_BRAKE           "Axis has no brake hardware"

/** @brief Message for ERR_POSLOS (Story 4-6) */
#define MSG_POSITION_UNKNOWN        "Position unknown"

/** @brief Message for ERR_INVALID_PIN (Story 4-7) */
#define MSG_PIN_OUT_OF_RANGE        "Pin out of range"

/** @brief Message for ERR_UNKNOWN_ALIAS (Story 4-7) */
#define MSG_ALIAS_NOT_FOUND         "Alias not found"

/** @brief Message for ERR_WIDTH_C_ONLY (Story 4-9) */
#define MSG_WIDTH_C_ONLY            "WIDTH only for C axis"

/** @brief Message for ERR_AXIS_ERROR (Story 4-10) */
#define MSG_CLEAR_ERROR_FIRST       "Clear error first"

/** @brief Message for ERR_I2C_FAILURE (Story 4-10) */
#define MSG_I2C_FAILURE             "I2C communication failure"

/** @brief Message for ERR_I2C_TIMEOUT (Story 4-11) */
#define MSG_I2C_TIMEOUT             "I2C transaction timeout"

/** @brief Message for ERR_I2C_DEVICE_OFFLINE (Story 4-11) */
#define MSG_I2C_DEVICE_OFFLINE      "I2C device offline"

/** @} */ // end err_messages

/**
 * @defgroup evt_types Event Type Strings
 * @brief Asynchronous event identifiers sent to host
 * @{
 */

/** @brief System boot complete: "EVENT BOOT V1.0.0 AXES:8 STATE:IDLE" */
#define EVT_BOOT                    "BOOT"

/** @brief Motion completed successfully */
#define EVT_MOTION_COMPLETE         "DONE"

/** @brief Limit switch triggered: "EVENT LIMIT <axis> MIN|MAX" */
#define EVT_LIMIT_TRIGGERED         "LIMIT"

/** @brief Emergency stop activated */
#define EVT_ESTOP_ACTIVATED         "ESTOP"

/** @brief Driver alarm triggered */
#define EVT_ALARM_TRIGGERED         "ALARM"

/** @brief Driver alarm cleared */
#define EVT_ALARM_CLEARED           "ALARMCLR"

/** @brief Homing sequence completed */
#define EVT_HOMING_COMPLETE         "HOMED"

/** @brief Position update (streaming mode) */
#define EVT_POSITION_UPDATE         "POS"

/** @brief Z-signal drift detected (not yet corrected) */
#define EVT_ZSYNC_DETECTED          "ZSYNC"

/** @brief Z-signal correction applied (after motion complete) */
#define EVT_ZSYNC_CORRECTED         "ZSYNCD"

/** @brief Approaching soft limit in velocity mode */
#define EVT_SOFT_LIMIT_APPROACH     "SLIMIT"

/** @brief Soft limit reached: "EVENT SOFT_LIMIT <axis> POS:<pos> TARGET:<tgt>" */
#define EVT_SOFT_LIMIT_REACHED      "SLIMIT"

/** @brief Stepper PCNT mismatch detected (warning only) */
#define EVT_PCNT_MISMATCH           "PCNTM"

/** @brief Homing in progress: "EVENT HOMING <axis> <state> POS:<pos>" */
#define EVT_HOMING                  "HOMING"

/** @brief Homing degraded - Z-signal fallback to limit-only */
#define EVT_HOMING_DEGRADED         "HOMEDEG"

/** @brief Homing paused - awaiting user command */
#define EVT_HOMING_PAUSED           "HOMEPAUSE"

/** @brief Brake state changed: "EVENT BRAKE <axis> ENGAGED|RELEASED" (Story 4-5) */
#define EVT_BRAKE                   "BRAKE"

/** @brief Position loss detected: "EVENT POSLOS <axis>" (Story 4-6) */
#define EVT_POSLOS                  "POSLOS"

/** @brief Digital input change: "EVENT DIN DINx value" or "EVENT DIN <alias> value" (Story 4-7) */
#define EVT_DIN                     "DIN"

/** @brief Width measurement: "EVENT WIDTH C <value>" (Story 4-9) */
#define EVT_WIDTH                   "WIDTH"

/** @brief Boot limit scan complete */
#define EVT_LIMITS_SCANNED          "LIMITSCAN"

/** @brief Switch fault: "EVENT SWITCH_FAULT <axis> BOTH_ACTIVE" */
#define EVT_SWITCH_FAULT            "SWFAULT"

/** @brief Mode changed: "EVENT MODE <mode_name>" */
#define EVT_MODE                    "MODE"

/** @brief Error event: "EVENT ERROR <axis|SYSTEM> <code>" (Story 4-10) */
#define EVT_ERROR                   "ERROR"

/** @brief I2C recovered event: "EVENT I2C RECOVERED" (Story 4-11) */
#define EVT_I2C_RECOVERED           "I2C"

/** @brief Limits degraded event: "EVENT WARN LIMITS_DEGRADED" (Story 4-11) */
#define EVT_LIMITS_DEGRADED         "LIMITS_DEGRADED"

/** @brief Feedback degraded event: "EVENT WARN FEEDBACK_DEGRADED" (Story 4-11) */
#define EVT_FEEDBACK_DEGRADED       "FEEDBACK_DEGRADED"

/** @} */ // end evt_types

/**
 * @defgroup mode_names Mode Name Strings
 * @brief System operating mode names for MODE command
 * @{
 */

/** @brief IDLE mode name */
#define MODE_NAME_IDLE              "IDLE"

/** @brief READY mode name */
#define MODE_NAME_READY             "READY"

/** @brief CONFIG mode name */
#define MODE_NAME_CONFIG            "CONFIG"

/** @brief ESTOP mode name */
#define MODE_NAME_ESTOP             "ESTOP"

/** @brief ERROR mode name */
#define MODE_NAME_ERROR             "ERROR"

/** @} */ // end mode_names

/**
 * @defgroup homing_states Homing State Values
 * @brief State names for EVT_HOMING event reporting
 * @{
 */

/** @brief Seeking limit switch */
#define HOMING_STATE_SEEK_LIMIT     "SEEK_LIMIT"

/** @brief Limit switch triggered */
#define HOMING_STATE_LIMIT_HIT      "LIMIT_HIT"

/** @brief Backing off from limit */
#define HOMING_STATE_BACKOFF        "BACKOFF"

/** @brief Seeking Z-signal index pulse */
#define HOMING_STATE_SEEK_ZSIG      "SEEK_ZSIG"

/** @brief Homing complete */
#define HOMING_STATE_COMPLETE       "COMPLETE"

/** @} */ // end homing_states

/**
 * @defgroup end_switch_mode End Switch Mode Enum
 * @brief Operating modes for limit switches
 * @{
 */

/**
 * @brief End switch operating modes
 *
 * Each limit switch (MIN and MAX) can be configured independently.
 */
typedef enum {
    /** @brief Switch not connected or disabled */
    END_SWITCH_MODE_NONE = 0,

    /** @brief Immediately stop motion on trigger (default) */
    END_SWITCH_MODE_HARD_STOP = 1,

    /** @brief Decelerate to stop, restrict motion direction */
    END_SWITCH_MODE_RESTRICT = 2,

    /** @brief Fire event only, don't stop motion */
    END_SWITCH_MODE_EVENT_ONLY = 3
} EndSwitchMode;

/** @brief Default switch mode for new configurations */
#define END_SWITCH_MODE_DEFAULT     END_SWITCH_MODE_HARD_STOP

/** @} */ // end end_switch_mode

/**
 * @defgroup error_categories Error Category Enum (Story 4-10)
 * @brief Categories for error tracking and counting
 * @{
 */

/**
 * @brief Error category enumeration
 *
 * Used by ErrorManager to track cumulative error counts per category.
 * Each category can be incremented globally and per-axis.
 */
typedef enum {
    /** @brief I2C communication failures */
    ERR_CAT_I2C = 0,

    /** @brief Motion or operation timeouts */
    ERR_CAT_TIMEOUT = 1,

    /** @brief Limit switch triggers */
    ERR_CAT_LIMIT = 2,

    /** @brief Emergency stop activations */
    ERR_CAT_ESTOP = 3,

    /** @brief Position loss events */
    ERR_CAT_POSLOS = 4,

    /** @brief Number of error categories (must be last) */
    ERR_CAT_COUNT = 5
} ErrorCategory;

/** @} */ // end error_categories

/** @} */ // end config_commands

#endif // CONFIG_COMMANDS_H
