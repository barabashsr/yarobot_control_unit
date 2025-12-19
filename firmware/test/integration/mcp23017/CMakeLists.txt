# Integration test for MCP23017 interrupt latency
# Run as standalone app: idf.py -C test/integration/mcp23017 build flash monitor

idf_component_register(
    SRCS "test_mcp23017_interrupt_latency.c"
    INCLUDE_DIRS "."
    REQUIRES unity mcp23017_wrapper config freertos esp_timer
    PRIV_REQUIRES log
)
