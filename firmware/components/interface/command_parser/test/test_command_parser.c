/**
 * @file test_command_parser.c
 * @brief Unit tests for command parser component
 * @author YaRobot Team
 * @date 2025
 *
 * Tests all acceptance criteria from Story 2.2
 */

#include "unity.h"
#include "command_parser.h"
#include "config_commands.h"
#include "config_limits.h"
#include <string.h>
#include <math.h>

/**
 * @brief Helper to check float equality with tolerance
 */
static bool float_eq(float a, float b, float eps)
{
    return fabsf(a - b) < eps;
}

/**
 * AC1: Given a text line is received from USB, when the parser processes it,
 * then a ParsedCommand structure is populated with verb, axis, params, and str_param fields
 */
TEST_CASE("AC1: ParsedCommand structure populated correctly", "[command_parser]")
{
    ParsedCommand cmd;
    TEST_ASSERT_EQUAL(ESP_OK, parser_init());

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("MOVE X 100", &cmd));
    TEST_ASSERT_EQUAL_STRING("MOVE", cmd.verb);
    TEST_ASSERT_EQUAL('X', cmd.axis);
    TEST_ASSERT_EQUAL(1, cmd.param_count);
    TEST_ASSERT_TRUE(float_eq(100.0f, cmd.params[0], 0.001f));
    TEST_ASSERT_FALSE(cmd.has_str_param);
}

/**
 * AC2: Given command input, when parsing, then command verb is case-insensitive
 */
TEST_CASE("AC2: Case-insensitive verb parsing", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("move X 100", &cmd));
    TEST_ASSERT_EQUAL_STRING("MOVE", cmd.verb);

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("MOVE X 100", &cmd));
    TEST_ASSERT_EQUAL_STRING("MOVE", cmd.verb);

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("Move X 100", &cmd));
    TEST_ASSERT_EQUAL_STRING("MOVE", cmd.verb);

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("mOvE X 100", &cmd));
    TEST_ASSERT_EQUAL_STRING("MOVE", cmd.verb);
}

/**
 * AC3: Given axis input, when parsing, then axis letters are case-insensitive
 */
TEST_CASE("AC3: Case-insensitive axis parsing", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("MOVE x 100", &cmd));
    TEST_ASSERT_EQUAL('X', cmd.axis);

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("MOVE X 100", &cmd));
    TEST_ASSERT_EQUAL('X', cmd.axis);

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("MOVE y 100", &cmd));
    TEST_ASSERT_EQUAL('Y', cmd.axis);
}

/**
 * AC4: Given input with whitespace, when parsing, then whitespace correctly separates tokens
 */
TEST_CASE("AC4: Whitespace token separation", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("  MOVE   X   100  ", &cmd));
    TEST_ASSERT_EQUAL_STRING("MOVE", cmd.verb);
    TEST_ASSERT_EQUAL('X', cmd.axis);
    TEST_ASSERT_EQUAL(1, cmd.param_count);

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("\tMOVE\tX\t100\t", &cmd));
    TEST_ASSERT_EQUAL_STRING("MOVE", cmd.verb);
    TEST_ASSERT_EQUAL('X', cmd.axis);
    TEST_ASSERT_EQUAL(1, cmd.param_count);
}

/**
 * AC5: Given an empty line, when parsing, then line is ignored (no error, no command)
 */
TEST_CASE("AC5: Empty line handling", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("", &cmd));
    TEST_ASSERT_EQUAL_STRING("", cmd.verb);

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("   ", &cmd));
    TEST_ASSERT_EQUAL_STRING("", cmd.verb);

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("\t\n\r", &cmd));
    TEST_ASSERT_EQUAL_STRING("", cmd.verb);
}

/**
 * AC6: Given a line starting with #, when parsing, then line is treated as comment (ignored)
 */
TEST_CASE("AC6: Comment line handling", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("# This is a comment", &cmd));
    TEST_ASSERT_EQUAL_STRING("", cmd.verb);

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("#MOVE X 100", &cmd));
    TEST_ASSERT_EQUAL_STRING("", cmd.verb);

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("  # leading whitespace comment", &cmd));
    TEST_ASSERT_EQUAL_STRING("", cmd.verb);
}

/**
 * AC7: Given `MOVE X 100`, when parsed, then verb=CMD_MOVE, axis='X', params=[100.0], param_count=1
 */
TEST_CASE("AC7: MOVE X 100 parsing", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("MOVE X 100", &cmd));
    TEST_ASSERT_EQUAL_STRING(CMD_MOVE, cmd.verb);
    TEST_ASSERT_EQUAL('X', cmd.axis);
    TEST_ASSERT_EQUAL(1, cmd.param_count);
    TEST_ASSERT_TRUE(float_eq(100.0f, cmd.params[0], 0.001f));
}

/**
 * AC8: Given `MOVR Y -50.5 200`, when parsed, then verb=CMD_MOVR, axis='Y', params=[-50.5, 200.0], param_count=2
 */
TEST_CASE("AC8: MOVR Y -50.5 200 parsing", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("MOVR Y -50.5 200", &cmd));
    TEST_ASSERT_EQUAL_STRING(CMD_MOVR, cmd.verb);
    TEST_ASSERT_EQUAL('Y', cmd.axis);
    TEST_ASSERT_EQUAL(2, cmd.param_count);
    TEST_ASSERT_TRUE(float_eq(-50.5f, cmd.params[0], 0.001f));
    TEST_ASSERT_TRUE(float_eq(200.0f, cmd.params[1], 0.001f));
}

/**
 * AC9: Given `STOP`, when parsed, then verb=CMD_STOP, axis='\0', param_count=0
 */
TEST_CASE("AC9: STOP parsing (no axis)", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("STOP", &cmd));
    TEST_ASSERT_EQUAL_STRING(CMD_STOP, cmd.verb);
    TEST_ASSERT_EQUAL('\0', cmd.axis);
    TEST_ASSERT_EQUAL(0, cmd.param_count);
}

/**
 * AC10: Given `STOP Z`, when parsed, then verb=CMD_STOP, axis='Z', param_count=0
 */
TEST_CASE("AC10: STOP Z parsing (with axis)", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("STOP Z", &cmd));
    TEST_ASSERT_EQUAL_STRING(CMD_STOP, cmd.verb);
    TEST_ASSERT_EQUAL('Z', cmd.axis);
    TEST_ASSERT_EQUAL(0, cmd.param_count);
}

/**
 * AC11: Given `ALIAS X RAILWAY`, when parsed, then verb=CMD_ALIAS, axis='X', str_param="RAILWAY", has_str_param=true
 */
TEST_CASE("AC11: ALIAS X RAILWAY parsing", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("ALIAS X RAILWAY", &cmd));
    TEST_ASSERT_EQUAL_STRING(CMD_ALIAS, cmd.verb);
    TEST_ASSERT_EQUAL('X', cmd.axis);
    TEST_ASSERT_EQUAL_STRING("RAILWAY", cmd.str_param);
    TEST_ASSERT_TRUE(cmd.has_str_param);
}

/**
 * AC12: Given `EN X 1`, when parsed, then verb=CMD_EN, axis='X', params=[1.0], param_count=1
 */
TEST_CASE("AC12: EN X 1 parsing", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("EN X 1", &cmd));
    TEST_ASSERT_EQUAL_STRING(CMD_EN, cmd.verb);
    TEST_ASSERT_EQUAL('X', cmd.axis);
    TEST_ASSERT_EQUAL(1, cmd.param_count);
    TEST_ASSERT_TRUE(float_eq(1.0f, cmd.params[0], 0.001f));
}

/**
 * AC13: Given invalid/malformed input, when parsing, then return ESP_ERR_INVALID_ARG (no crash)
 */
TEST_CASE("AC13: Malformed input handling", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, parse_command(NULL, &cmd));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, parse_command("MOVE X abc", &cmd));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, parse_command("MOVE X 1 2 3 4 5", &cmd));
}

/**
 * AC14: Given axis letter in input, when validating, then only valid axes (X,Y,Z,A,B,C,D,E) are accepted
 */
TEST_CASE("AC14: Valid axis validation", "[command_parser]")
{
    TEST_ASSERT_TRUE(is_valid_axis('X'));
    TEST_ASSERT_TRUE(is_valid_axis('Y'));
    TEST_ASSERT_TRUE(is_valid_axis('Z'));
    TEST_ASSERT_TRUE(is_valid_axis('A'));
    TEST_ASSERT_TRUE(is_valid_axis('B'));
    TEST_ASSERT_TRUE(is_valid_axis('C'));
    TEST_ASSERT_TRUE(is_valid_axis('D'));
    TEST_ASSERT_TRUE(is_valid_axis('E'));

    TEST_ASSERT_TRUE(is_valid_axis('x'));
    TEST_ASSERT_TRUE(is_valid_axis('e'));

    TEST_ASSERT_FALSE(is_valid_axis('F'));
    TEST_ASSERT_FALSE(is_valid_axis('W'));
    TEST_ASSERT_FALSE(is_valid_axis('Q'));
    TEST_ASSERT_FALSE(is_valid_axis('0'));
    TEST_ASSERT_FALSE(is_valid_axis(' '));
}

/**
 * AC15: Given input exceeding LIMIT_CMD_MAX_LENGTH (256), when parsing, then return error without buffer overflow
 */
TEST_CASE("AC15: Buffer overflow protection", "[command_parser]")
{
    ParsedCommand cmd;
    char long_line[LIMIT_CMD_MAX_LENGTH + 50];
    memset(long_line, 'A', sizeof(long_line) - 1);
    long_line[sizeof(long_line) - 1] = '\0';

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, parse_command(long_line, &cmd));
}

/**
 * Additional test: axis_to_index and index_to_axis round-trip
 */
TEST_CASE("axis_to_index and index_to_axis round-trip", "[command_parser]")
{
    TEST_ASSERT_EQUAL(0, axis_to_index('X'));
    TEST_ASSERT_EQUAL(1, axis_to_index('Y'));
    TEST_ASSERT_EQUAL(2, axis_to_index('Z'));
    TEST_ASSERT_EQUAL(3, axis_to_index('A'));
    TEST_ASSERT_EQUAL(4, axis_to_index('B'));
    TEST_ASSERT_EQUAL(5, axis_to_index('C'));
    TEST_ASSERT_EQUAL(6, axis_to_index('D'));
    TEST_ASSERT_EQUAL(7, axis_to_index('E'));

    TEST_ASSERT_EQUAL(-1, axis_to_index('F'));
    TEST_ASSERT_EQUAL(-1, axis_to_index('W'));

    TEST_ASSERT_EQUAL('X', index_to_axis(0));
    TEST_ASSERT_EQUAL('Y', index_to_axis(1));
    TEST_ASSERT_EQUAL('Z', index_to_axis(2));
    TEST_ASSERT_EQUAL('A', index_to_axis(3));
    TEST_ASSERT_EQUAL('B', index_to_axis(4));
    TEST_ASSERT_EQUAL('C', index_to_axis(5));
    TEST_ASSERT_EQUAL('D', index_to_axis(6));
    TEST_ASSERT_EQUAL('E', index_to_axis(7));

    TEST_ASSERT_EQUAL('\0', index_to_axis(8));
    TEST_ASSERT_EQUAL('\0', index_to_axis(255));

    for (uint8_t i = 0; i < 8; i++) {
        char axis = index_to_axis(i);
        TEST_ASSERT_EQUAL(i, axis_to_index(axis));
    }
}

/**
 * Additional test: commands without axis but with numeric param
 */
TEST_CASE("Commands with numeric param but no axis", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("VEL 1000", &cmd));
    TEST_ASSERT_EQUAL_STRING("VEL", cmd.verb);
    TEST_ASSERT_EQUAL('\0', cmd.axis);
    TEST_ASSERT_EQUAL(1, cmd.param_count);
    TEST_ASSERT_TRUE(float_eq(1000.0f, cmd.params[0], 0.001f));
}

/**
 * Additional test: Invalid axis letter treated as numeric param
 */
TEST_CASE("Invalid axis letter treated as numeric param attempt", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, parse_command("MOVE W 100", &cmd));
}

/**
 * Additional test: Multiple numeric parameters
 */
TEST_CASE("Multiple numeric parameters", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("MOVR X 10 20 30 40", &cmd));
    TEST_ASSERT_EQUAL_STRING("MOVR", cmd.verb);
    TEST_ASSERT_EQUAL('X', cmd.axis);
    TEST_ASSERT_EQUAL(4, cmd.param_count);
    TEST_ASSERT_TRUE(float_eq(10.0f, cmd.params[0], 0.001f));
    TEST_ASSERT_TRUE(float_eq(20.0f, cmd.params[1], 0.001f));
    TEST_ASSERT_TRUE(float_eq(30.0f, cmd.params[2], 0.001f));
    TEST_ASSERT_TRUE(float_eq(40.0f, cmd.params[3], 0.001f));
}

/**
 * Additional test: Decimal and negative numbers
 */
TEST_CASE("Decimal and negative number parsing", "[command_parser]")
{
    ParsedCommand cmd;

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("MOVE X -123.456", &cmd));
    TEST_ASSERT_TRUE(float_eq(-123.456f, cmd.params[0], 0.001f));

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("MOVE X .5", &cmd));
    TEST_ASSERT_TRUE(float_eq(0.5f, cmd.params[0], 0.001f));

    TEST_ASSERT_EQUAL(ESP_OK, parse_command("MOVE X -.5", &cmd));
    TEST_ASSERT_TRUE(float_eq(-0.5f, cmd.params[0], 0.001f));
}
