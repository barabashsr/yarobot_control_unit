/**
 * @file motor_system.cpp
 * @brief Motor system initialization implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 3-9b: Motor System Integration
 *       Instantiates all motor control components and wires them together.
 */

#include "motor_system.h"
#include "esp_log.h"

// Config headers
#include "config_gpio.h"
#include "config_peripherals.h"
#include "config_limits.h"
#include "config_defaults.h"
#include "config_axes.h"
#include "config_timing.h"

// Pulse generator headers
#include "rmt_pulse_gen.h"
#include "mcpwm_pulse_gen.h"
#include "ledc_pulse_gen.h"

// Position tracker headers
#include "software_tracker.h"
#include "time_tracker.h"
// Note: PcntTracker not needed - MCPWM/LEDC generators implement IPositionTracker directly

// Motor headers
#include "servo_motor.h"
#include "stepper_motor.h"
#include "discrete_axis.h"

// Motion controller
#include "motion_controller.h"

// Command handlers
#include "move_handler.h"
#include "movr_handler.h"

static const char* TAG = "motor_system";

// ============================================================================
// Static Instance Storage
// ============================================================================

/**
 * @defgroup pulse_generators Static Pulse Generator Instances
 * @{
 */

/** @brief RMT pulse generators for servo axes X, Z, A, B */
static RmtPulseGenerator s_rmt_x(RMT_CHANNEL_X, GPIO_X_STEP);
static RmtPulseGenerator s_rmt_z(RMT_CHANNEL_Z, GPIO_Z_STEP);
static RmtPulseGenerator s_rmt_a(RMT_CHANNEL_A, GPIO_A_STEP);
static RmtPulseGenerator s_rmt_b(RMT_CHANNEL_B, GPIO_B_STEP);

/** @brief MCPWM pulse generators for Y and C axes */
static McpwmPulseGenerator s_mcpwm_y(MCPWM_TIMER_Y, GPIO_Y_STEP, PCNT_UNIT_Y);
static McpwmPulseGenerator s_mcpwm_c(MCPWM_TIMER_C, GPIO_C_STEP, PCNT_UNIT_C);

/** @brief LEDC pulse generator for D axis */
static LedcPulseGenerator s_ledc_d(GPIO_D_STEP, LEDC_TIMER_D, LEDC_CHANNEL_D);

/** @} */ // end pulse_generators

/**
 * @defgroup position_trackers Static Position Tracker Instances
 * @{
 */

/** @brief Software trackers for RMT axes (X, Z, A, B) */
static SoftwareTracker s_tracker_x;
static SoftwareTracker s_tracker_z;
static SoftwareTracker s_tracker_a;
static SoftwareTracker s_tracker_b;

/**
 * @brief MCPWM/LEDC pulse generators also implement IPositionTracker
 *
 * These pulse generators have internal PCNT hardware counting, so they
 * directly provide position tracking. No separate PcntTracker needed.
 * Pointers to s_mcpwm_y, s_mcpwm_c, s_ledc_d are used as IPositionTracker*.
 */

/** @brief Time tracker for E axis discrete actuator */
static TimeTracker s_tracker_e(TIMING_E_AXIS_TRAVEL_MS);

/** @} */ // end position_trackers

/**
 * @defgroup axis_configs Static Axis Configuration Instances
 * @{
 */

/** @brief Axis configurations for all 8 axes */
static AxisConfig s_config_x;
static AxisConfig s_config_y;
static AxisConfig s_config_z;
static AxisConfig s_config_a;
static AxisConfig s_config_b;
static AxisConfig s_config_c;
static AxisConfig s_config_d;
static AxisConfig s_config_e;

/** @} */ // end axis_configs

/**
 * @defgroup motors Static Motor Instances
 * @{
 */

/** @brief Servo motors for X, Y, Z, A, B axes */
static ServoMotor* s_servo_x = nullptr;
static ServoMotor* s_servo_y = nullptr;
static ServoMotor* s_servo_z = nullptr;
static ServoMotor* s_servo_a = nullptr;
static ServoMotor* s_servo_b = nullptr;

/** @brief Stepper motors for C, D axes */
static StepperMotor* s_stepper_c = nullptr;
static StepperMotor* s_stepper_d = nullptr;

/** @brief Discrete axis for E axis */
static DiscreteAxis* s_discrete_e = nullptr;

/** @brief Storage for motor objects (placement new) */
static uint8_t s_servo_x_storage[sizeof(ServoMotor)] __attribute__((aligned(alignof(ServoMotor))));
static uint8_t s_servo_y_storage[sizeof(ServoMotor)] __attribute__((aligned(alignof(ServoMotor))));
static uint8_t s_servo_z_storage[sizeof(ServoMotor)] __attribute__((aligned(alignof(ServoMotor))));
static uint8_t s_servo_a_storage[sizeof(ServoMotor)] __attribute__((aligned(alignof(ServoMotor))));
static uint8_t s_servo_b_storage[sizeof(ServoMotor)] __attribute__((aligned(alignof(ServoMotor))));
static uint8_t s_stepper_c_storage[sizeof(StepperMotor)] __attribute__((aligned(alignof(StepperMotor))));
static uint8_t s_stepper_d_storage[sizeof(StepperMotor)] __attribute__((aligned(alignof(StepperMotor))));
static uint8_t s_discrete_e_storage[sizeof(DiscreteAxis)] __attribute__((aligned(alignof(DiscreteAxis))));

/** @} */ // end motors

/** @brief Initialization flag */
static bool s_initialized = false;

// ============================================================================
// Private Helper Functions
// ============================================================================

/**
 * @brief Initialize axis configurations with default values
 */
static void init_axis_configs(void)
{
    // Linear servo axes (X, Y, Z) - meters
    s_config_x = AxisConfig::createDefaultLinear();
    s_config_x.alias[0] = 'X';
    s_config_x.alias[1] = '\0';

    s_config_y = AxisConfig::createDefaultLinear();
    s_config_y.alias[0] = 'Y';
    s_config_y.alias[1] = '\0';

    s_config_z = AxisConfig::createDefaultLinear();
    s_config_z.alias[0] = 'Z';
    s_config_z.alias[1] = '\0';

    // Rotary servo axes (A, B) - radians
    s_config_a = AxisConfig::createDefaultRotary();
    s_config_a.alias[0] = 'A';
    s_config_a.alias[1] = '\0';

    s_config_b = AxisConfig::createDefaultRotary();
    s_config_b.alias[0] = 'B';
    s_config_b.alias[1] = '\0';

    // Stepper axes (C, D) - linear defaults
    s_config_c = AxisConfig::createDefaultLinear();
    s_config_c.alias[0] = 'C';
    s_config_c.alias[1] = '\0';

    s_config_d = AxisConfig::createDefaultLinear();
    s_config_d.alias[0] = 'D';
    s_config_d.alias[1] = '\0';

    // E axis - discrete actuator (binary 0/1 position)
    s_config_e.pulses_per_rev = E_AXIS_PULSES_PER_UNIT;
    s_config_e.units_per_rev = 1.0f;
    s_config_e.is_rotary = false;
    s_config_e.limit_min = E_AXIS_LIMIT_MIN;
    s_config_e.limit_max = E_AXIS_LIMIT_MAX;
    s_config_e.max_velocity = E_AXIS_MAX_VELOCITY;
    s_config_e.max_acceleration = E_AXIS_MAX_ACCELERATION;
    s_config_e.backlash = 0.0f;
    s_config_e.home_offset = 0.0f;
    s_config_e.alias[0] = 'E';
    s_config_e.alias[1] = '\0';

    ESP_LOGI(TAG, "Axis configurations initialized");
}

/**
 * @brief Initialize all pulse generators
 *
 * @return ESP_OK on success
 */
static esp_err_t init_pulse_generators(void)
{
    esp_err_t ret;

    // Initialize RMT pulse generators (X, Z, A, B)
    ret = s_rmt_x.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init RMT X: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_rmt_z.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init RMT Z: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_rmt_a.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init RMT A: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_rmt_b.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init RMT B: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize MCPWM pulse generators (Y, C)
    ret = s_mcpwm_y.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init MCPWM Y: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_mcpwm_c.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init MCPWM C: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize LEDC pulse generator (D)
    ret = s_ledc_d.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init LEDC D: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "All pulse generators initialized");
    return ESP_OK;
}

/**
 * @brief Initialize all position trackers
 *
 * @return ESP_OK on success
 */
static esp_err_t init_position_trackers(void)
{
    esp_err_t ret;

    // Initialize software trackers (X, Z, A, B)
    ret = s_tracker_x.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init tracker X: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_tracker_z.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init tracker Z: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_tracker_a.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init tracker A: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_tracker_b.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init tracker B: %s", esp_err_to_name(ret));
        return ret;
    }

    // Note: MCPWM/LEDC axes (Y, C, D) don't need separate PCNT trackers.
    // Their pulse generators already have internal PCNT and implement IPositionTracker.

    // Initialize time tracker (E)
    ret = s_tracker_e.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init tracker E: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "All position trackers initialized");
    return ESP_OK;
}

/**
 * @brief Wire pulse generators to position trackers
 *
 * RMT generators notify SoftwareTrackers on DMA completion.
 * MCPWM and LEDC don't need wiring - they use PCNT hardware counting.
 */
static void wire_generators_to_trackers(void)
{
    // Wire RMT generators to software trackers
    s_rmt_x.setPositionTracker(&s_tracker_x);
    s_rmt_z.setPositionTracker(&s_tracker_z);
    s_rmt_a.setPositionTracker(&s_tracker_a);
    s_rmt_b.setPositionTracker(&s_tracker_b);

    // MCPWM and LEDC use PCNT hardware counting - no software tracker wiring needed
    // The PCNT units are already configured to count pulses from the GPIO pins
    // via internal loopback in the pulse generator init()

    ESP_LOGI(TAG, "Pulse generators wired to position trackers");
}

/**
 * @brief Create motor objects using placement new
 *
 * @return ESP_OK on success
 */
static esp_err_t create_motor_objects(void)
{
    // Create servo motors (X, Y, Z, A, B) using placement new
    // RMT axes use SoftwareTracker, MCPWM axes use pulse generator as tracker (implements IPositionTracker)
    s_servo_x = new (s_servo_x_storage) ServoMotor(&s_rmt_x, &s_tracker_x, AXIS_X, s_config_x);
    s_servo_y = new (s_servo_y_storage) ServoMotor(&s_mcpwm_y, &s_mcpwm_y, AXIS_Y, s_config_y);
    s_servo_z = new (s_servo_z_storage) ServoMotor(&s_rmt_z, &s_tracker_z, AXIS_Z, s_config_z);
    s_servo_a = new (s_servo_a_storage) ServoMotor(&s_rmt_a, &s_tracker_a, AXIS_A, s_config_a);
    s_servo_b = new (s_servo_b_storage) ServoMotor(&s_rmt_b, &s_tracker_b, AXIS_B, s_config_b);

    // Create stepper motors (C, D) - pulse generators implement IPositionTracker
    s_stepper_c = new (s_stepper_c_storage) StepperMotor(&s_mcpwm_c, &s_mcpwm_c, AXIS_C, s_config_c);
    s_stepper_d = new (s_stepper_d_storage) StepperMotor(&s_ledc_d, &s_ledc_d, AXIS_D, s_config_d);

    // Create discrete axis (E)
    s_discrete_e = new (s_discrete_e_storage) DiscreteAxis(&s_tracker_e, AXIS_E, s_config_e);

    ESP_LOGI(TAG, "Motor objects created");
    return ESP_OK;
}

/**
 * @brief Initialize all motor objects
 *
 * @return ESP_OK on success
 */
static esp_err_t init_motor_objects(void)
{
    esp_err_t ret;

    ret = s_servo_x->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init servo X: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_servo_y->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init servo Y: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_servo_z->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init servo Z: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_servo_a->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init servo A: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_servo_b->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init servo B: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_stepper_c->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init stepper C: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_stepper_d->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init stepper D: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = s_discrete_e->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init discrete E: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "All motor objects initialized");
    return ESP_OK;
}

/**
 * @brief Initialize the motion controller with motor array
 *
 * @return ESP_OK on success
 */
static esp_err_t init_motion_controller(void)
{
    // Build motor array
    IMotor* motors[LIMIT_NUM_AXES] = {
        s_servo_x,      // AXIS_X = 0
        s_servo_y,      // AXIS_Y = 1
        s_servo_z,      // AXIS_Z = 2
        s_servo_a,      // AXIS_A = 3
        s_servo_b,      // AXIS_B = 4
        s_stepper_c,    // AXIS_C = 5
        s_stepper_d,    // AXIS_D = 6
        s_discrete_e    // AXIS_E = 7
    };

    // Initialize the global motion controller
    MotionController* controller = getMotionController();
    esp_err_t ret = controller->init(motors);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init motion controller: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Motion controller initialized with %d axes", LIMIT_NUM_AXES);
    return ESP_OK;
}

/**
 * @brief Register command handlers (MOVE, MOVR)
 *
 * @return ESP_OK on success
 */
static esp_err_t register_command_handlers(void)
{
    esp_err_t ret;

    ret = move_handler_register();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MOVE handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = movr_handler_register();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MOVR handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Command handlers registered (MOVE, MOVR)");
    return ESP_OK;
}

// ============================================================================
// Public API Implementation
// ============================================================================

esp_err_t motor_system_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Motor system already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Motor System Initialization Starting");
    ESP_LOGI(TAG, "========================================");

    esp_err_t ret;

    // Step 1: Initialize axis configurations
    init_axis_configs();

    // Step 2: Initialize pulse generators
    ret = init_pulse_generators();
    if (ret != ESP_OK) {
        return ret;
    }

    // Step 3: Initialize position trackers
    ret = init_position_trackers();
    if (ret != ESP_OK) {
        return ret;
    }

    // Step 4: Wire pulse generators to position trackers
    wire_generators_to_trackers();

    // Step 5: Create motor objects
    ret = create_motor_objects();
    if (ret != ESP_OK) {
        return ret;
    }

    // Step 6: Initialize motor objects
    ret = init_motor_objects();
    if (ret != ESP_OK) {
        return ret;
    }

    // Step 7: Initialize motion controller
    ret = init_motion_controller();
    if (ret != ESP_OK) {
        return ret;
    }

    // Step 8: Register command handlers
    ret = register_command_handlers();
    if (ret != ESP_OK) {
        return ret;
    }

    s_initialized = true;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Motor System Initialization: SUCCESS");
    ESP_LOGI(TAG, "  Servos: %d (X, Y, Z, A, B)", LIMIT_NUM_SERVOS);
    ESP_LOGI(TAG, "  Steppers: %d (C, D)", LIMIT_NUM_STEPPERS);
    ESP_LOGI(TAG, "  Discrete: %d (E)", LIMIT_NUM_DISCRETE);
    ESP_LOGI(TAG, "========================================");

    return ESP_OK;
}

bool motor_system_is_initialized(void)
{
    return s_initialized;
}

void motor_system_update(uint8_t axis_id)
{
    if (!s_initialized) {
        return;
    }

    // Currently no per-axis update logic needed
    // RMT/MCPWM/LEDC handle streaming internally via their own tasks
    // This function is a placeholder for future per-axis motion monitoring
    (void)axis_id;
}
