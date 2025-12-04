/**
 * @file command_parser.c
 * @brief Command parser implementation
 * @author YaRobot Team
 * @date 2025
 */

#include "command_parser.h"
#include "config_limits.h"
#include "config_commands.h"

#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/**
 * @brief Axis letter lookup table
 *
 * Maps index 0-7 to axis letters X, Y, Z, A, B, C, D, E
 */
static const char axis_letters[] = {'X', 'Y', 'Z', 'A', 'B', 'C', 'D', 'E'};

/**
 * @brief Check if command expects a string parameter
 *
 * Some commands (like ALIAS) take a string as their final parameter
 * instead of a numeric value.
 *
 * @param[in] verb Uppercase command verb
 * @return true if command expects string parameter
 */
static bool command_expects_string_param(const char* verb)
{
    return (strcmp(verb, CMD_ALIAS) == 0);
}

/**
 * @brief Convert string to uppercase in-place
 *
 * @param[in,out] str String to convert
 */
static void str_to_upper(char* str)
{
    while (*str) {
        *str = (char)toupper((unsigned char)*str);
        str++;
    }
}

esp_err_t parser_init(void)
{
    return ESP_OK;
}

esp_err_t parse_command(const char* line, ParsedCommand* cmd)
{
    if (line == NULL || cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(cmd, 0, sizeof(ParsedCommand));

    size_t line_len = strlen(line);
    if (line_len > LIMIT_CMD_MAX_LENGTH) {
        return ESP_ERR_INVALID_ARG;
    }

    const char* ptr = line;
    while (*ptr && isspace((unsigned char)*ptr)) {
        ptr++;
    }

    if (*ptr == '\0') {
        return ESP_OK;
    }

    if (*ptr == '#') {
        return ESP_OK;
    }

    char buffer[LIMIT_CMD_MAX_LENGTH + 1];
    strncpy(buffer, ptr, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char* saveptr = NULL;
    char* token = strtok_r(buffer, " \t\r\n", &saveptr);

    if (token == NULL) {
        return ESP_OK;
    }

    str_to_upper(token);

    size_t verb_len = strlen(token);
    if (verb_len > PARSER_MAX_VERB_LEN) {
        verb_len = PARSER_MAX_VERB_LEN;
    }
    strncpy(cmd->verb, token, verb_len);
    cmd->verb[verb_len] = '\0';

    token = strtok_r(NULL, " \t\r\n", &saveptr);

    if (token == NULL) {
        return ESP_OK;
    }

    if (strlen(token) == 1 && isalpha((unsigned char)token[0])) {
        char axis_char = (char)toupper((unsigned char)token[0]);
        if (is_valid_axis(axis_char)) {
            cmd->axis = axis_char;
            token = strtok_r(NULL, " \t\r\n", &saveptr);
        }
    }

    bool expects_string = command_expects_string_param(cmd->verb);

    while (token != NULL) {
        if (expects_string && cmd->axis != '\0') {
            size_t str_len = strlen(token);
            if (str_len > PARSER_MAX_STR_PARAM) {
                str_len = PARSER_MAX_STR_PARAM;
            }
            strncpy(cmd->str_param, token, str_len);
            cmd->str_param[str_len] = '\0';
            str_to_upper(cmd->str_param);
            cmd->has_str_param = true;
            break;
        }

        if (cmd->param_count >= PARSER_MAX_PARAMS) {
            return ESP_ERR_INVALID_ARG;
        }

        char* endptr;
        float value = strtof(token, &endptr);

        if (endptr == token || (*endptr != '\0' && !isspace((unsigned char)*endptr))) {
            return ESP_ERR_INVALID_ARG;
        }

        cmd->params[cmd->param_count] = value;
        cmd->param_count++;

        token = strtok_r(NULL, " \t\r\n", &saveptr);
    }

    return ESP_OK;
}

bool is_valid_axis(char axis)
{
    char upper = (char)toupper((unsigned char)axis);
    for (size_t i = 0; i < sizeof(axis_letters); i++) {
        if (upper == axis_letters[i]) {
            return true;
        }
    }
    return false;
}

int8_t axis_to_index(char axis)
{
    char upper = (char)toupper((unsigned char)axis);
    for (size_t i = 0; i < sizeof(axis_letters); i++) {
        if (upper == axis_letters[i]) {
            return (int8_t)i;
        }
    }
    return -1;
}

char index_to_axis(uint8_t index)
{
    if (index >= sizeof(axis_letters)) {
        return '\0';
    }
    return axis_letters[index];
}
