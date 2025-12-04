/**
 * @file command_parser.h
 * @brief Command parser public API for parsing USB command lines
 * @author YaRobot Team
 * @date 2025
 *
 * @note Parses text lines into structured ParsedCommand objects.
 *       Thread-safe implementation using strtok_r.
 */

#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup command_parser Command Parser
 * @brief Parse ASCII command lines into structured commands
 * @{
 */

/**
 * @brief Maximum number of numeric parameters per command
 */
#define PARSER_MAX_PARAMS       4

/**
 * @brief Maximum verb length (excluding null terminator)
 */
#define PARSER_MAX_VERB_LEN     15

/**
 * @brief Maximum string parameter length (excluding null terminator)
 */
#define PARSER_MAX_STR_PARAM    31

/**
 * @brief Parsed command structure
 *
 * Contains all fields extracted from a command line.
 * Stack-allocated, no heap memory required.
 */
typedef struct {
    /** @brief Command verb (CMD_MOVE, CMD_STOP, etc.) uppercase */
    char verb[16];

    /** @brief Axis letter ('X'-'E') or '\0' if none */
    char axis;

    /** @brief Numeric parameters */
    float params[PARSER_MAX_PARAMS];

    /** @brief Number of parameters parsed (0-4) */
    uint8_t param_count;

    /** @brief String parameter (for CMD_ALIAS, etc.) */
    char str_param[32];

    /** @brief Whether str_param is valid */
    bool has_str_param;
} ParsedCommand;

/**
 * @brief Initialize the command parser
 *
 * Initializes any static parser state. Must be called before
 * using other parser functions.
 *
 * @return ESP_OK on success
 */
esp_err_t parser_init(void);

/**
 * @brief Parse a command line into a ParsedCommand structure
 *
 * Parses a null-terminated command line and populates the cmd structure.
 * The line is tokenized and parsed into verb, axis, numeric parameters,
 * and optional string parameter.
 *
 * @param[in] line Null-terminated command line to parse
 * @param[out] cmd Pointer to ParsedCommand structure to populate
 *
 * @return ESP_OK on success (including empty/comment lines)
 * @return ESP_ERR_INVALID_ARG if line or cmd is NULL, or input is malformed
 *
 * @note Empty lines and comment lines (starting with #) return ESP_OK
 *       with an empty verb field (verb[0] == '\0').
 * @note Thread-safe: uses strtok_r for tokenization.
 */
esp_err_t parse_command(const char* line, ParsedCommand* cmd);

/**
 * @brief Check if a character is a valid axis letter
 *
 * Valid axes are X, Y, Z, A, B, C, D, E (case-insensitive).
 *
 * @param[in] axis Character to check
 *
 * @return true if axis is valid (X-E), false otherwise
 */
bool is_valid_axis(char axis);

/**
 * @brief Convert axis letter to array index
 *
 * Maps axis letters to array indices:
 * 'X'->0, 'Y'->1, 'Z'->2, 'A'->3, 'B'->4, 'C'->5, 'D'->6, 'E'->7
 *
 * @param[in] axis Axis letter (case-insensitive)
 *
 * @return Array index (0-7) or -1 if invalid
 */
int8_t axis_to_index(char axis);

/**
 * @brief Convert array index to axis letter
 *
 * Maps array indices to axis letters:
 * 0->'X', 1->'Y', 2->'Z', 3->'A', 4->'B', 5->'C', 6->'D', 7->'E'
 *
 * @param[in] index Array index (0-7)
 *
 * @return Axis letter ('X'-'E') or '\0' if invalid
 */
char index_to_axis(uint8_t index);

/** @} */ // end command_parser

#ifdef __cplusplus
}
#endif

#endif // COMMAND_PARSER_H
