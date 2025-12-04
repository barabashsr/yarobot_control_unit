/**
 * @file test_command_executor.c
 * @brief Unit tests for command executor component
 * @author YaRobot Team
 * @date 2025
 *
 * Tests all acceptance criteria from Story 2.4
 */

#include "unity.h"
#include "command_executor.h"
#include "command_parser.h"
#include "config_commands.h"
#include "config_limits.h"
#include <string.h>

/* ==========================================================================
 * Test Helpers
 * ========================================================================== */

/**
 * @brief Helper to create a parsed command
 */
static void make_command(ParsedCommand* cmd, const char* verb, char axis,
                         int param_count, const char* str_param)
{
    memset(cmd, 0, sizeof(*cmd));
    strncpy(cmd->verb, verb, sizeof(cmd->verb) - 1);
    cmd->axis = axis;
    cmd->param_count = param_count;
    if (str_param) {
        strncpy(cmd->str_param, str_param, sizeof(cmd->str_param) - 1);
        cmd->has_str_param = true;
    }
}

/**
 * @brief Custom handler for testing registration
 */
static esp_err_t test_handler(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    (void)cmd;
    snprintf(response, resp_len, "OK TEST_HANDLER\r\n");
    return ESP_OK;
}

/**
 * @brief Handler that returns error
 */
static esp_err_t error_handler(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    (void)cmd;
    snprintf(response, resp_len, "ERROR E999 Test error\r\n");
    return ESP_FAIL;
}

/* ==========================================================================
 * State Management Tests (AC10-12)
 * ========================================================================== */

/**
 * AC10: Given get_system_state() is called, when querying state,
 * then current SystemState enum value is returned
 */
TEST_CASE("AC10: get_system_state returns current state", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    // After init, state should be IDLE
    TEST_ASSERT_EQUAL(STATE_IDLE, get_system_state());

    // After setting to READY
    set_system_state(STATE_READY);
    TEST_ASSERT_EQUAL(STATE_READY, get_system_state());

    // Restore
    set_system_state(STATE_IDLE);
}

/**
 * AC11: Given set_system_state() is called, when changing state,
 * then system state is updated atomically
 */
TEST_CASE("AC11: set_system_state updates atomically", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    // Test all valid states
    TEST_ASSERT_EQUAL(ESP_OK, set_system_state(STATE_IDLE));
    TEST_ASSERT_EQUAL(STATE_IDLE, get_system_state());

    TEST_ASSERT_EQUAL(ESP_OK, set_system_state(STATE_READY));
    TEST_ASSERT_EQUAL(STATE_READY, get_system_state());

    TEST_ASSERT_EQUAL(ESP_OK, set_system_state(STATE_CONFIG));
    TEST_ASSERT_EQUAL(STATE_CONFIG, get_system_state());

    TEST_ASSERT_EQUAL(ESP_OK, set_system_state(STATE_ESTOP));
    TEST_ASSERT_EQUAL(STATE_ESTOP, get_system_state());

    TEST_ASSERT_EQUAL(ESP_OK, set_system_state(STATE_ERROR));
    TEST_ASSERT_EQUAL(STATE_ERROR, get_system_state());

    // Restore
    set_system_state(STATE_IDLE);
}

/**
 * AC11: Invalid state values are rejected
 */
TEST_CASE("AC11: set_system_state rejects invalid states", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    // Zero is invalid
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, set_system_state(0));

    // Non-power-of-2 (combined states) are invalid for setting
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, set_system_state(STATE_IDLE | STATE_READY));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, set_system_state(0x03));

    // STATE_ANY is special - allowed for setting (acts as wildcard)
    TEST_ASSERT_EQUAL(ESP_OK, set_system_state(STATE_ANY));

    // Restore
    set_system_state(STATE_IDLE);
}

/**
 * AC12: Given is_state_allowed() is called with a bitmask,
 * when checking permission, then returns true if current state matches mask
 */
TEST_CASE("AC12: is_state_allowed checks bitmask", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    // In IDLE state
    set_system_state(STATE_IDLE);
    TEST_ASSERT_TRUE(is_state_allowed(STATE_IDLE));
    TEST_ASSERT_TRUE(is_state_allowed(STATE_IDLE | STATE_READY));
    TEST_ASSERT_TRUE(is_state_allowed(STATE_ANY));
    TEST_ASSERT_FALSE(is_state_allowed(STATE_READY));
    TEST_ASSERT_FALSE(is_state_allowed(STATE_CONFIG));

    // In READY state
    set_system_state(STATE_READY);
    TEST_ASSERT_TRUE(is_state_allowed(STATE_READY));
    TEST_ASSERT_TRUE(is_state_allowed(STATE_IDLE | STATE_READY));
    TEST_ASSERT_TRUE(is_state_allowed(STATE_ANY));
    TEST_ASSERT_FALSE(is_state_allowed(STATE_IDLE));
    TEST_ASSERT_FALSE(is_state_allowed(STATE_ESTOP));

    // Restore
    set_system_state(STATE_IDLE);
}

/* ==========================================================================
 * Initialization and Registration Tests (AC5-6)
 * ========================================================================== */

/**
 * AC5: Given cmd_executor_init() is called, when initialization completes,
 * then all built-in command handlers are registered in the table
 */
TEST_CASE("AC5: cmd_executor_init registers built-in commands", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    // Verify ECHO is registered
    make_command(&cmd, CMD_ECHO, '\0', 0, "hello");
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "OK"));

    // Verify INFO is registered
    make_command(&cmd, CMD_INFO, '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "OK"));
    TEST_ASSERT_NOT_NULL(strstr(response, "YAROBOT"));

    // Verify STAT is registered
    make_command(&cmd, CMD_STAT, '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "OK"));

    // Verify MODE is registered
    make_command(&cmd, CMD_MODE, '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "OK"));

    set_system_state(STATE_IDLE);
}

/**
 * AC6: Given cmd_executor_register() is called, when adding a new entry,
 * then the command becomes available for dispatch
 */
TEST_CASE("AC6: cmd_executor_register adds new command", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    // First verify TEST command doesn't exist
    make_command(&cmd, "TESTCMD", '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, ERR_INVALID_COMMAND));

    // Register new command
    CommandEntry entry = {
        .verb = "TESTCMD",
        .handler = test_handler,
        .allowed_states = STATE_ANY
    };
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_register(&entry));

    // Now it should work
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "TEST_HANDLER"));
}

/**
 * Registration validation
 */
TEST_CASE("cmd_executor_register validates input", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    // NULL entry
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cmd_executor_register(NULL));

    // Empty verb
    CommandEntry entry = { .verb = "", .handler = test_handler, .allowed_states = STATE_ANY };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cmd_executor_register(&entry));

    // NULL verb
    entry.verb = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cmd_executor_register(&entry));

    // NULL handler
    entry.verb = "TEST";
    entry.handler = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cmd_executor_register(&entry));
}

/* ==========================================================================
 * Dispatch Tests (AC1-4, AC7-8)
 * ========================================================================== */

/**
 * AC1: Given a ParsedCommand is ready, when the dispatcher processes it,
 * then it looks up the handler in the command table using cmd->verb
 */
TEST_CASE("AC1: dispatch_command looks up handler by verb", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    // ECHO command
    make_command(&cmd, CMD_ECHO, '\0', 0, "test");
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "test"));

    // INFO command
    make_command(&cmd, CMD_INFO, '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "YAROBOT"));
}

/**
 * AC1: Case-insensitive verb lookup
 */
TEST_CASE("AC1: dispatch_command is case-insensitive", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    // Lowercase
    make_command(&cmd, "echo", '\0', 0, "lower");
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "lower"));

    // Mixed case
    make_command(&cmd, "Echo", '\0', 0, "mixed");
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "mixed"));

    // Uppercase
    make_command(&cmd, "ECHO", '\0', 0, "upper");
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "upper"));
}

/**
 * AC2: Given a command table entry is found, when state validation runs,
 * then the handler is only invoked if current_state & entry.allowed_states is non-zero
 */
TEST_CASE("AC2: dispatch_command validates state", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    // Register a command only allowed in READY state
    CommandEntry entry = {
        .verb = "READYONLY",
        .handler = test_handler,
        .allowed_states = STATE_READY
    };
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_register(&entry));

    make_command(&cmd, "READYONLY", '\0', 0, NULL);

    // In IDLE state - should be blocked
    set_system_state(STATE_IDLE);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, ERR_MODE_BLOCKED));

    // In READY state - should work
    set_system_state(STATE_READY);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "TEST_HANDLER"));

    // Restore
    set_system_state(STATE_IDLE);
}

/**
 * AC3: Given state validation fails, when generating response,
 * then format is "ERROR E012 Command blocked in current mode\r\n"
 */
TEST_CASE("AC3: state blocked returns ERR_MODE_BLOCKED", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    // Register a command only allowed in CONFIG state
    CommandEntry entry = {
        .verb = "CONFIGONLY",
        .handler = test_handler,
        .allowed_states = STATE_CONFIG
    };
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_register(&entry));

    // Ensure we're in IDLE state
    set_system_state(STATE_IDLE);

    make_command(&cmd, "CONFIGONLY", '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, dispatch_command(&cmd, response, sizeof(response)));

    // Verify exact error format
    TEST_ASSERT_NOT_NULL(strstr(response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(response, ERR_MODE_BLOCKED));
    TEST_ASSERT_NOT_NULL(strstr(response, MSG_MODE_BLOCKED));
}

/**
 * AC4: Given command verb is not in command table, when generating response,
 * then format is "ERROR E001 Invalid command\r\n"
 */
TEST_CASE("AC4: unknown command returns ERR_INVALID_COMMAND", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    make_command(&cmd, "NOTACOMMAND", '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, dispatch_command(&cmd, response, sizeof(response)));

    // Verify exact error format
    TEST_ASSERT_NOT_NULL(strstr(response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(response, ERR_INVALID_COMMAND));
    TEST_ASSERT_NOT_NULL(strstr(response, MSG_INVALID_COMMAND));
}

/**
 * AC7: Given dispatch_command() is called, when handler executes successfully,
 * then response buffer contains handler's response
 */
TEST_CASE("AC7: successful handler response in buffer", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    // ECHO with data
    make_command(&cmd, CMD_ECHO, '\0', 0, "hello world");
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "OK"));
    TEST_ASSERT_NOT_NULL(strstr(response, "hello world"));

    // INFO
    make_command(&cmd, CMD_INFO, '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "OK"));
    TEST_ASSERT_NOT_NULL(strstr(response, "YAROBOT_CONTROL_UNIT"));
}

/**
 * AC8: Given dispatch_command() is called, when handler returns error,
 * then response buffer contains formatted error
 */
TEST_CASE("AC8: handler error response in buffer", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    // Register a handler that returns error
    CommandEntry entry = {
        .verb = "ERRORTEST",
        .handler = error_handler,
        .allowed_states = STATE_ANY
    };
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_register(&entry));

    make_command(&cmd, "ERRORTEST", '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_command(&cmd, response, sizeof(response)));

    // Handler should have set the error response
    TEST_ASSERT_NOT_NULL(strstr(response, "ERROR"));
}

/* ==========================================================================
 * Stub Handler Tests
 * ========================================================================== */

/**
 * Test ECHO handler
 */
TEST_CASE("ECHO handler returns input", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    // With string param
    make_command(&cmd, CMD_ECHO, '\0', 0, "test message");
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_EQUAL_STRING("OK test message\r\n", response);

    // Without string param
    make_command(&cmd, CMD_ECHO, '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_EQUAL_STRING("OK\r\n", response);
}

/**
 * Test INFO handler
 */
TEST_CASE("INFO handler returns system info", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    make_command(&cmd, CMD_INFO, '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "OK"));
    TEST_ASSERT_NOT_NULL(strstr(response, "YAROBOT_CONTROL_UNIT"));
    TEST_ASSERT_NOT_NULL(strstr(response, "1.0.0"));
    TEST_ASSERT_NOT_NULL(strstr(response, "AXES:8"));
}

/**
 * Test STAT handler - system status
 */
TEST_CASE("STAT handler returns system status", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    // System status in IDLE state
    set_system_state(STATE_IDLE);
    make_command(&cmd, CMD_STAT, '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "STATE:IDLE"));

    // System status in READY state
    set_system_state(STATE_READY);
    make_command(&cmd, CMD_STAT, '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "STATE:READY"));

    // Restore
    set_system_state(STATE_IDLE);
}

/**
 * Test STAT handler - axis status
 */
TEST_CASE("STAT handler returns axis status", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    make_command(&cmd, CMD_STAT, 'X', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "OK"));
    TEST_ASSERT_NOT_NULL(strstr(response, "X"));
    TEST_ASSERT_NOT_NULL(strstr(response, "IDLE"));
    TEST_ASSERT_NOT_NULL(strstr(response, "POS:"));
}

/**
 * Test MODE handler - query
 */
TEST_CASE("MODE handler queries current mode", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    set_system_state(STATE_IDLE);
    make_command(&cmd, CMD_MODE, '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "IDLE"));

    set_system_state(STATE_READY);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "READY"));

    set_system_state(STATE_IDLE);
}

/**
 * Test MODE handler - set mode
 */
TEST_CASE("MODE handler sets mode", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    // Set to READY
    make_command(&cmd, CMD_MODE, '\0', 0, "READY");
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "OK"));
    TEST_ASSERT_EQUAL(STATE_READY, get_system_state());

    // Set to CONFIG
    make_command(&cmd, CMD_MODE, '\0', 0, "CONFIG");
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "OK"));
    TEST_ASSERT_EQUAL(STATE_CONFIG, get_system_state());

    // Set to IDLE
    make_command(&cmd, CMD_MODE, '\0', 0, "IDLE");
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "OK"));
    TEST_ASSERT_EQUAL(STATE_IDLE, get_system_state());
}

/**
 * Test MODE handler - invalid mode
 */
TEST_CASE("MODE handler rejects invalid mode", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    set_system_state(STATE_IDLE);

    make_command(&cmd, CMD_MODE, '\0', 0, "INVALID");
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_NOT_NULL(strstr(response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(response, ERR_INVALID_PARAMETER));

    // State should not have changed
    TEST_ASSERT_EQUAL(STATE_IDLE, get_system_state());
}

/* ==========================================================================
 * Thread Safety Tests (AC13)
 * ========================================================================== */

/**
 * AC13: Given multiple tasks may call dispatch_command(), when executing handlers,
 * then shared state access is mutex-protected
 *
 * Note: Full multi-task testing requires FreeRTOS task creation which is
 * complex in unit tests. This test verifies the basic thread-safety design.
 */
TEST_CASE("AC13: Thread safety - rapid consecutive calls", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response1[LIMIT_RESPONSE_MAX_LENGTH];
    char response2[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd1, cmd2;

    // Rapidly alternate between different commands
    for (int i = 0; i < 100; i++) {
        make_command(&cmd1, CMD_ECHO, '\0', 0, "first");
        make_command(&cmd2, CMD_INFO, '\0', 0, NULL);

        TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd1, response1, sizeof(response1)));
        TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd2, response2, sizeof(response2)));

        // Verify responses are not mixed
        TEST_ASSERT_NOT_NULL(strstr(response1, "first"));
        TEST_ASSERT_NOT_NULL(strstr(response2, "YAROBOT"));
    }
}

/**
 * AC13: State changes are atomic
 */
TEST_CASE("AC13: State changes are atomic", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    // Rapidly toggle states
    for (int i = 0; i < 1000; i++) {
        set_system_state(STATE_IDLE);
        TEST_ASSERT_EQUAL(STATE_IDLE, get_system_state());

        set_system_state(STATE_READY);
        TEST_ASSERT_EQUAL(STATE_READY, get_system_state());
    }

    set_system_state(STATE_IDLE);
}

/* ==========================================================================
 * Edge Cases
 * ========================================================================== */

/**
 * Empty command verb
 */
TEST_CASE("Empty command verb returns OK", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;

    make_command(&cmd, "", '\0', 0, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    TEST_ASSERT_EQUAL_STRING("OK\r\n", response);
}

/**
 * NULL arguments
 */
TEST_CASE("NULL arguments handled", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;
    make_command(&cmd, CMD_ECHO, '\0', 0, NULL);

    // NULL cmd
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, dispatch_command(NULL, response, sizeof(response)));

    // NULL response
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, dispatch_command(&cmd, NULL, sizeof(response)));

    // Zero length
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, dispatch_command(&cmd, response, 0));
}

/**
 * Response terminates with CRLF
 */
TEST_CASE("All responses terminate with CRLF", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;
    size_t len;

    // Success response
    make_command(&cmd, CMD_ECHO, '\0', 0, "test");
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
    len = strlen(response);
    TEST_ASSERT_EQUAL('\r', response[len - 2]);
    TEST_ASSERT_EQUAL('\n', response[len - 1]);

    // Error response
    make_command(&cmd, "INVALID", '\0', 0, NULL);
    dispatch_command(&cmd, response, sizeof(response));
    len = strlen(response);
    TEST_ASSERT_EQUAL('\r', response[len - 2]);
    TEST_ASSERT_EQUAL('\n', response[len - 1]);
}

/**
 * Double initialization is safe
 */
TEST_CASE("Double initialization is safe", "[command_executor]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());
    TEST_ASSERT_EQUAL(ESP_OK, cmd_executor_init());

    // Should still work
    char response[LIMIT_RESPONSE_MAX_LENGTH];
    ParsedCommand cmd;
    make_command(&cmd, CMD_ECHO, '\0', 0, "test");
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_command(&cmd, response, sizeof(response)));
}
