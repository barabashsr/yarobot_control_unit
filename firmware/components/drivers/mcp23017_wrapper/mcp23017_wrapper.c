/**
 * @file mcp23017_wrapper.c
 * @brief MCP23017 I/O Expander Wrapper Implementation
 * @author YaRobot Team
 * @date 2025
 */

#include "mcp23017_wrapper.h"
#include "mcp23017.h"
#include "i2c_bus.h"
#include "config_i2c.h"
#include "config_gpio.h"
#include "config_timing.h"
#include "i2c_health_monitor.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/semphr.h"

static const char *TAG = "mcp23017_wrapper";

/** @brief I2C bus handle shared by both MCP23017 devices */
static i2c_bus_handle_t s_i2c_bus = NULL;

/** @brief MCP23017 device handles */
static mcp23017_handle_t s_mcp_handles[MCP_DEVICE_COUNT] = {NULL, NULL};

/** @brief Mutex for thread-safe I2C access */
static SemaphoreHandle_t s_mutex = NULL;

/** @brief Task handle for interrupt notifications */
static TaskHandle_t s_notify_task = NULL;

/** @brief Initialization state */
static bool s_initialized = false;

/** @brief ISR installed state */
static bool s_isr_installed = false;

/* Forward declarations for ISR handlers (IRAM_ATTR only on definition) */
static void mcp0_inta_isr_handler(void *arg);
static void mcp0_intb_isr_handler(void *arg);
static void mcp1_inta_isr_handler(void *arg);
static void mcp1_intb_isr_handler(void *arg);

esp_err_t mcp23017_wrapper_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    /* Create mutex for thread-safe access */
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Configure I2C bus */
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = I2C_FREQ_HZ,
        .clk_flags = 0,
    };

    s_i2c_bus = i2c_bus_create(I2C_PORT, &i2c_conf);
    if (s_i2c_bus == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C bus");
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "I2C bus created on port %d at %d Hz", I2C_PORT, I2C_FREQ_HZ);

    /* Scan I2C bus for devices */
    uint8_t found_addrs[8];
    uint8_t found_count = i2c_bus_scan(s_i2c_bus, found_addrs, 8);
    ESP_LOGI(TAG, "I2C scan found %d device(s)", found_count);
    for (int i = 0; i < found_count; i++) {
        ESP_LOGI(TAG, "  Device at 0x%02X", found_addrs[i]);
    }

    /* Create MCP23017 #0 (limit switches) */
    s_mcp_handles[MCP_DEVICE_0] = mcp23017_create(s_i2c_bus, I2C_ADDR_MCP23017_0);
    if (s_mcp_handles[MCP_DEVICE_0] == NULL) {
        ESP_LOGE(TAG, "Failed to create MCP23017 #0 at 0x%02X", I2C_ADDR_MCP23017_0);
        ret = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }

    /* Verify MCP23017 #0 is present */
    ret = mcp23017_check_present(s_mcp_handles[MCP_DEVICE_0]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MCP23017 #0 not responding at 0x%02X", I2C_ADDR_MCP23017_0);
        if (i2c_health_is_initialized()) {
            i2c_health_record_failure(I2C_PORT, I2C_ADDR_MCP23017_0);
        }
        ret = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }
    if (i2c_health_is_initialized()) {
        i2c_health_record_success(I2C_PORT, I2C_ADDR_MCP23017_0);
    }
    ESP_LOGI(TAG, "MCP23017 #0 (0x%02X) present", I2C_ADDR_MCP23017_0);

    /* Create MCP23017 #1 (ALARM_INPUT + InPos) */
    s_mcp_handles[MCP_DEVICE_1] = mcp23017_create(s_i2c_bus, I2C_ADDR_MCP23017_1);
    if (s_mcp_handles[MCP_DEVICE_1] == NULL) {
        ESP_LOGE(TAG, "Failed to create MCP23017 #1 at 0x%02X", I2C_ADDR_MCP23017_1);
        ret = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }

    /* Verify MCP23017 #1 is present */
    ret = mcp23017_check_present(s_mcp_handles[MCP_DEVICE_1]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MCP23017 #1 not responding at 0x%02X", I2C_ADDR_MCP23017_1);
        if (i2c_health_is_initialized()) {
            i2c_health_record_failure(I2C_PORT, I2C_ADDR_MCP23017_1);
        }
        ret = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }
    if (i2c_health_is_initialized()) {
        i2c_health_record_success(I2C_PORT, I2C_ADDR_MCP23017_1);
    }
    ESP_LOGI(TAG, "MCP23017 #1 (0x%02X) present", I2C_ADDR_MCP23017_1);

    /* Configure MCP23017 #0: all pins as inputs */
    ret = mcp23017_set_io_dir(s_mcp_handles[MCP_DEVICE_0], 0xFF, MCP23017_GPIOA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MCP0 Port A direction");
        goto cleanup;
    }
    ret = mcp23017_set_io_dir(s_mcp_handles[MCP_DEVICE_0], 0xFF, MCP23017_GPIOB);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MCP0 Port B direction");
        goto cleanup;
    }

    /* Configure MCP23017 #0: enable pull-ups on all pins */
    ret = mcp23017_set_pullup(s_mcp_handles[MCP_DEVICE_0], MCP23017_ALLPINS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MCP0 pull-ups");
        goto cleanup;
    }
    ESP_LOGI(TAG, "MCP23017 #0 configured: all inputs with pull-ups");

    /* Configure MCP23017 #1: all pins as inputs */
    ret = mcp23017_set_io_dir(s_mcp_handles[MCP_DEVICE_1], 0xFF, MCP23017_GPIOA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MCP1 Port A direction");
        goto cleanup;
    }
    ret = mcp23017_set_io_dir(s_mcp_handles[MCP_DEVICE_1], 0xFF, MCP23017_GPIOB);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MCP1 Port B direction");
        goto cleanup;
    }

    /* Configure MCP23017 #1: enable pull-ups on all pins */
    ret = mcp23017_set_pullup(s_mcp_handles[MCP_DEVICE_1], MCP23017_ALLPINS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MCP1 pull-ups");
        goto cleanup;
    }
    ESP_LOGI(TAG, "MCP23017 #1 configured: all inputs with pull-ups");

    s_initialized = true;
    ESP_LOGI(TAG, "MCP23017 wrapper initialized successfully");
    return ESP_OK;

cleanup:
    if (s_mcp_handles[MCP_DEVICE_1] != NULL) {
        mcp23017_delete(&s_mcp_handles[MCP_DEVICE_1]);
    }
    if (s_mcp_handles[MCP_DEVICE_0] != NULL) {
        mcp23017_delete(&s_mcp_handles[MCP_DEVICE_0]);
    }
    if (s_i2c_bus != NULL) {
        i2c_bus_delete(&s_i2c_bus);
    }
    if (s_mutex != NULL) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }
    return ret;
}

esp_err_t mcp23017_wrapper_config_mcp0_interrupts(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    /* Enable interrupt-on-change for all pins on MCP0
     * Mode 0 = compare against previous value (not DEFVAL)
     * Default value is ignored when mode = 0
     */
    ret = mcp23017_interrupt_en(s_mcp_handles[MCP_DEVICE_0], MCP23017_ALLPINS, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable MCP0 interrupts: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set interrupt polarity to active-low (default, open-drain friendly) */
    ret = mcp23017_set_interrupt_polarity(s_mcp_handles[MCP_DEVICE_0], MCP23017_GPIOA, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MCP0 Port A interrupt polarity");
        return ret;
    }
    ret = mcp23017_set_interrupt_polarity(s_mcp_handles[MCP_DEVICE_0], MCP23017_GPIOB, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MCP0 Port B interrupt polarity");
        return ret;
    }

    /* Disable interrupt mirroring (separate INTA for Port A, INTB for Port B) */
    ret = mcp23017_mirror_interrupt(s_mcp_handles[MCP_DEVICE_0], 0, MCP23017_GPIOA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MCP0 interrupt mirror");
        return ret;
    }

    ESP_LOGI(TAG, "MCP0 interrupt-on-change enabled for all 16 limit switch pins");
    return ESP_OK;
}

esp_err_t mcp23017_wrapper_config_mcp1_interrupts(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    /* Enable interrupt-on-change for ALARM_INPUT pins on MCP1 Port A
     * Pins GPA0-GPA6 (7 ALARM_INPUT pins), GPA7 is spare
     * Mask = 0x7F (bits 0-6)
     */
    uint16_t alarm_pins = MCP23017_PIN0 | MCP23017_PIN1 | MCP23017_PIN2 |
                          MCP23017_PIN3 | MCP23017_PIN4 | MCP23017_PIN5 |
                          MCP23017_PIN6;  /* 0x007F */

    ret = mcp23017_interrupt_en(s_mcp_handles[MCP_DEVICE_1], alarm_pins, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable MCP1 Port A interrupts: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set interrupt polarity to active-low */
    ret = mcp23017_set_interrupt_polarity(s_mcp_handles[MCP_DEVICE_1], MCP23017_GPIOA, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MCP1 Port A interrupt polarity");
        return ret;
    }

    /* Disable interrupt mirroring */
    ret = mcp23017_mirror_interrupt(s_mcp_handles[MCP_DEVICE_1], 0, MCP23017_GPIOA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MCP1 interrupt mirror");
        return ret;
    }

    ESP_LOGI(TAG, "MCP1 interrupt-on-change enabled for 7 ALARM_INPUT pins (GPA0-GPA6)");
    return ESP_OK;
}

esp_err_t mcp23017_wrapper_register_notify_task(TaskHandle_t task_handle)
{
    if (task_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_notify_task = task_handle;
    ESP_LOGI(TAG, "Registered notify task: %s", pcTaskGetName(task_handle));
    return ESP_OK;
}

esp_err_t mcp23017_wrapper_install_isr(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_notify_task == NULL) {
        ESP_LOGE(TAG, "No notify task registered");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_isr_installed) {
        ESP_LOGW(TAG, "ISRs already installed");
        return ESP_OK;
    }

    esp_err_t ret;

    /* Install GPIO ISR service if not already installed */
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means already installed, which is OK */
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure GPIO_MCP0_INTA */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_MCP0_INTA),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  /* Falling edge (active-low) */
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO_MCP0_INTA (%d)", GPIO_MCP0_INTA);
        return ret;
    }
    ret = gpio_isr_handler_add(GPIO_MCP0_INTA, mcp0_inta_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MCP0_INTA ISR handler");
        return ret;
    }

    /* Configure GPIO_MCP0_INTB */
    io_conf.pin_bit_mask = (1ULL << GPIO_MCP0_INTB);
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO_MCP0_INTB (%d)", GPIO_MCP0_INTB);
        return ret;
    }
    ret = gpio_isr_handler_add(GPIO_MCP0_INTB, mcp0_intb_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MCP0_INTB ISR handler");
        return ret;
    }

    /* Configure GPIO_MCP1_INTA */
    io_conf.pin_bit_mask = (1ULL << GPIO_MCP1_INTA);
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO_MCP1_INTA (%d)", GPIO_MCP1_INTA);
        return ret;
    }
    ret = gpio_isr_handler_add(GPIO_MCP1_INTA, mcp1_inta_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MCP1_INTA ISR handler");
        return ret;
    }

    /* Configure GPIO_MCP1_INTB */
    io_conf.pin_bit_mask = (1ULL << GPIO_MCP1_INTB);
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO_MCP1_INTB (%d)", GPIO_MCP1_INTB);
        return ret;
    }
    ret = gpio_isr_handler_add(GPIO_MCP1_INTB, mcp1_intb_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MCP1_INTB ISR handler");
        return ret;
    }

    s_isr_installed = true;
    ESP_LOGI(TAG, "MCP23017 GPIO ISRs installed on pins %d, %d, %d, %d",
             GPIO_MCP0_INTA, GPIO_MCP0_INTB, GPIO_MCP1_INTA, GPIO_MCP1_INTB);
    return ESP_OK;
}

esp_err_t mcp23017_wrapper_read_port(mcp_device_t device, mcp_port_t port, uint8_t *value)
{
    if (device >= MCP_DEVICE_COUNT || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Take mutex with timeout */
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(TIMING_I2C_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout waiting for I2C mutex");
        return ESP_ERR_TIMEOUT;
    }

    mcp23017_gpio_port_t mcp_port = (port == MCP_PORT_A) ? MCP23017_GPIOA : MCP23017_GPIOB;
    *value = mcp23017_read_io(s_mcp_handles[device], mcp_port);

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t mcp23017_wrapper_read_all(mcp_device_t device, uint16_t *value)
{
    if (device >= MCP_DEVICE_COUNT || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Take mutex with timeout */
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(TIMING_I2C_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout waiting for I2C mutex");
        return ESP_ERR_TIMEOUT;
    }

    uint8_t port_a = mcp23017_read_io(s_mcp_handles[device], MCP23017_GPIOA);
    uint8_t port_b = mcp23017_read_io(s_mcp_handles[device], MCP23017_GPIOB);
    *value = (uint16_t)port_a | ((uint16_t)port_b << 8);

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t mcp23017_wrapper_get_int_flags(mcp_device_t device, uint16_t *flags)
{
    if (device >= MCP_DEVICE_COUNT || flags == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(TIMING_I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    *flags = mcp23017_get_int_flag(s_mcp_handles[device]);

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t mcp23017_wrapper_get_int_capture(mcp_device_t device, uint16_t *capture)
{
    if (device >= MCP_DEVICE_COUNT || capture == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(TIMING_I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    *capture = mcp23017_get_int_pin(s_mcp_handles[device]);

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

bool mcp23017_wrapper_is_initialized(void)
{
    return s_initialized;
}

esp_err_t mcp23017_wrapper_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    /* Remove ISRs if installed */
    if (s_isr_installed) {
        gpio_isr_handler_remove(GPIO_MCP0_INTA);
        gpio_isr_handler_remove(GPIO_MCP0_INTB);
        gpio_isr_handler_remove(GPIO_MCP1_INTA);
        gpio_isr_handler_remove(GPIO_MCP1_INTB);
        s_isr_installed = false;
    }

    /* Delete MCP23017 devices */
    if (s_mcp_handles[MCP_DEVICE_1] != NULL) {
        mcp23017_delete(&s_mcp_handles[MCP_DEVICE_1]);
    }
    if (s_mcp_handles[MCP_DEVICE_0] != NULL) {
        mcp23017_delete(&s_mcp_handles[MCP_DEVICE_0]);
    }

    /* Delete I2C bus */
    if (s_i2c_bus != NULL) {
        i2c_bus_delete(&s_i2c_bus);
    }

    /* Delete mutex */
    if (s_mutex != NULL) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }

    s_notify_task = NULL;
    s_initialized = false;

    ESP_LOGI(TAG, "MCP23017 wrapper deinitialized");
    return ESP_OK;
}

/* ISR handlers - must be in IRAM */

static void IRAM_ATTR mcp0_inta_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_notify_task != NULL) {
        xTaskNotifyFromISR(s_notify_task, MCP_NOTIFY_MCP0_INTA,
                           eSetBits, &higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void IRAM_ATTR mcp0_intb_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_notify_task != NULL) {
        xTaskNotifyFromISR(s_notify_task, MCP_NOTIFY_MCP0_INTB,
                           eSetBits, &higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void IRAM_ATTR mcp1_inta_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_notify_task != NULL) {
        xTaskNotifyFromISR(s_notify_task, MCP_NOTIFY_MCP1_INTA,
                           eSetBits, &higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void IRAM_ATTR mcp1_intb_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_notify_task != NULL) {
        xTaskNotifyFromISR(s_notify_task, MCP_NOTIFY_MCP1_INTB,
                           eSetBits, &higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}
