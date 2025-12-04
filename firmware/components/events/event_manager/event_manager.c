/**
 * @file event_manager.c
 * @brief Event publish/subscribe system implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Implements a pub/sub event system using FreeRTOS queues for
 *       asynchronous event delivery. Events are processed by a dedicated
 *       task that invokes registered callbacks.
 */

#include "event_manager.h"
#include "config.h"
#include "config_limits.h"
#include "config_commands.h"
#include "usb_cdc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <string.h>

static const char* TAG = "EVENT_MGR";

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** @brief Number of event types in EventType enum */
#define EVENT_TYPE_COUNT        8

/** @brief Event processor task priority (above normal) */
#define EVENT_TASK_PRIORITY     (tskIDLE_PRIORITY + 3)

/** @brief Maximum delivery latency before warning (microseconds) */
#define MAX_DELIVERY_LATENCY_US (5 * 1000)  // 5ms per AC2.14

/* ==========================================================================
 * Subscriber Storage
 * ========================================================================== */

/**
 * @brief Subscriber entry
 */
typedef struct {
    EventCallback callback;
    void* ctx;
} Subscriber;

/**
 * @brief Subscriber list for one event type
 */
typedef struct {
    Subscriber subscribers[LIMIT_EVENT_SUBSCRIBERS];
    size_t count;
} SubscriberList;

/* ==========================================================================
 * Static Variables
 * ========================================================================== */

/** @brief Initialization flag */
static bool s_initialized = false;

/** @brief Event queue handle */
static QueueHandle_t s_event_queue = NULL;

/** @brief Event processor task handle */
static TaskHandle_t s_event_task = NULL;

/** @brief Mutex for subscriber list access */
static SemaphoreHandle_t s_subscriber_mutex = NULL;

/** @brief Subscriber lists indexed by EventType */
static SubscriberList s_subscribers[EVENT_TYPE_COUNT];

/** @brief Queue high watermark tracking */
static uint32_t s_queue_high_watermark = 0;

/* ==========================================================================
 * Forward Declarations
 * ========================================================================== */

static void event_processor_task(void* arg);
static void usb_event_subscriber(const Event* event, void* ctx);

/* ==========================================================================
 * Initialization
 * ========================================================================== */

esp_err_t event_manager_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Create event queue
    s_event_queue = xQueueCreate(LIMIT_EVENT_QUEUE_DEPTH, sizeof(Event));
    if (s_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return ESP_ERR_NO_MEM;
    }

    // Create subscriber mutex
    s_subscriber_mutex = xSemaphoreCreateMutex();
    if (s_subscriber_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create subscriber mutex");
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Initialize subscriber lists
    memset(s_subscribers, 0, sizeof(s_subscribers));

    // Create event processor task
    BaseType_t ret = xTaskCreate(
        event_processor_task,
        "event_proc",
        STACK_EVENT_TASK,
        NULL,
        EVENT_TASK_PRIORITY,
        &s_event_task
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create event processor task");
        vSemaphoreDelete(s_subscriber_mutex);
        s_subscriber_mutex = NULL;
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    s_queue_high_watermark = 0;

    ESP_LOGI(TAG, "Initialized (queue=%d, max_subs=%d)",
             LIMIT_EVENT_QUEUE_DEPTH, LIMIT_EVENT_SUBSCRIBERS);

    // Register USB event subscriber for all event types
    for (int i = 0; i < EVENT_TYPE_COUNT; i++) {
        esp_err_t err = event_subscribe((EventType)i, usb_event_subscriber, NULL);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to subscribe USB handler to event type %d", i);
        }
    }

    // Publish EVT_BOOT event (AC10)
    Event boot_event = {
        .type = EVTTYPE_BOOT,
        .axis = 0xFF,  // System-wide
        .timestamp = esp_timer_get_time()
    };
    event_publish(&boot_event);

    return ESP_OK;
}

/* ==========================================================================
 * Subscribe / Unsubscribe
 * ========================================================================== */

esp_err_t event_subscribe(EventType type, EventCallback callback, void* ctx)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if ((int)type < 0 || (int)type >= EVENT_TYPE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = ESP_OK;

    if (xSemaphoreTake(s_subscriber_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire subscriber mutex");
        return ESP_ERR_TIMEOUT;
    }

    SubscriberList* list = &s_subscribers[(int)type];

    // Check for duplicate
    for (size_t i = 0; i < list->count; i++) {
        if (list->subscribers[i].callback == callback) {
            ESP_LOGW(TAG, "Callback already registered for type %d", (int)type);
            result = ESP_OK;  // Not an error, just a no-op
            goto done;
        }
    }

    // Check capacity
    if (list->count >= LIMIT_EVENT_SUBSCRIBERS) {
        ESP_LOGE(TAG, "Max subscribers reached for type %d", (int)type);
        result = ESP_ERR_NO_MEM;
        goto done;
    }

    // Add subscriber
    list->subscribers[list->count].callback = callback;
    list->subscribers[list->count].ctx = ctx;
    list->count++;

    ESP_LOGD(TAG, "Subscribed to type %d (count=%d)", (int)type, (int)list->count);

done:
    xSemaphoreGive(s_subscriber_mutex);
    return result;
}

esp_err_t event_unsubscribe(EventType type, EventCallback callback)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if ((int)type < 0 || (int)type >= EVENT_TYPE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = ESP_ERR_NOT_FOUND;

    if (xSemaphoreTake(s_subscriber_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire subscriber mutex");
        return ESP_ERR_TIMEOUT;
    }

    SubscriberList* list = &s_subscribers[(int)type];

    // Find and remove
    for (size_t i = 0; i < list->count; i++) {
        if (list->subscribers[i].callback == callback) {
            // Shift remaining entries
            for (size_t j = i; j < list->count - 1; j++) {
                list->subscribers[j] = list->subscribers[j + 1];
            }
            list->count--;
            result = ESP_OK;
            ESP_LOGD(TAG, "Unsubscribed from type %d (count=%d)", (int)type, (int)list->count);
            break;
        }
    }

    xSemaphoreGive(s_subscriber_mutex);

    if (result == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Callback not found for type %d", (int)type);
    }

    return result;
}

/* ==========================================================================
 * Publish
 * ========================================================================== */

esp_err_t event_publish(const Event* event)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Create a copy with timestamp if not set
    Event evt_copy = *event;
    if (evt_copy.timestamp == 0) {
        evt_copy.timestamp = esp_timer_get_time();
    }

    // Try to send to queue (non-blocking)
    if (xQueueSend(s_event_queue, &evt_copy, 0) != pdTRUE) {
        // Queue is full - drop oldest event (AC5)
        Event dropped;
        if (xQueueReceive(s_event_queue, &dropped, 0) == pdTRUE) {
            ESP_LOGW(TAG, "%s: Queue full, dropped oldest event (type=%d)",
                     ERR_EVENT_OVERFLOW, (int)dropped.type);
        }
        // Try again
        if (xQueueSend(s_event_queue, &evt_copy, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to queue event after dropping oldest");
            return ESP_ERR_NO_MEM;
        }
    }

    // Update high watermark
    UBaseType_t current_count = uxQueueMessagesWaiting(s_event_queue);
    if (current_count > s_queue_high_watermark) {
        s_queue_high_watermark = current_count;
    }

    return ESP_OK;
}

esp_err_t event_publish_from_isr(const Event* event, BaseType_t* woken)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (event == NULL || woken == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Create a copy with timestamp
    Event evt_copy = *event;
    if (evt_copy.timestamp == 0) {
        // Note: esp_timer_get_time() is ISR-safe on ESP32
        evt_copy.timestamp = esp_timer_get_time();
    }

    *woken = pdFALSE;

    // ISR-safe queue send (AC6)
    if (xQueueSendFromISR(s_event_queue, &evt_copy, woken) != pdTRUE) {
        // Queue full from ISR - can't drop oldest safely, just fail
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/* ==========================================================================
 * Event Processor Task
 * ========================================================================== */

/**
 * @brief Event processor task - delivers events to subscribers
 *
 * Blocks on queue, receives events in FIFO order (AC7), and invokes
 * all registered callbacks for the event type.
 */
static void event_processor_task(void* arg)
{
    (void)arg;
    Event event;

    ESP_LOGI(TAG, "Event processor task started");

    while (1) {
        // Block waiting for events (AC7: FIFO order maintained by queue)
        if (xQueueReceive(s_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            int64_t now = esp_timer_get_time();
            int64_t latency_us = now - event.timestamp;

            // Check delivery latency (AC8: warn if > 5ms)
            if (latency_us > MAX_DELIVERY_LATENCY_US) {
                ESP_LOGW(TAG, "Event delivery latency %lldus > %dus",
                         (long long)latency_us, MAX_DELIVERY_LATENCY_US);
            }

            // Deliver to subscribers
            if (xSemaphoreTake(s_subscriber_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                int type_idx = (int)event.type;
                if (type_idx >= 0 && type_idx < EVENT_TYPE_COUNT) {
                    SubscriberList* list = &s_subscribers[type_idx];

                    for (size_t i = 0; i < list->count; i++) {
                        if (list->subscribers[i].callback != NULL) {
                            list->subscribers[i].callback(&event, list->subscribers[i].ctx);
                        }
                    }
                }
                xSemaphoreGive(s_subscriber_mutex);
            } else {
                ESP_LOGW(TAG, "Failed to acquire mutex for event delivery");
            }
        }
    }
}

/* ==========================================================================
 * USB Event Subscriber (AC9)
 * ========================================================================== */

/**
 * @brief USB event subscriber callback
 *
 * Formats events using format_event() and sends to USB host.
 */
static void usb_event_subscriber(const Event* event, void* ctx)
{
    (void)ctx;

    char buf[LIMIT_RESPONSE_MAX_LENGTH];

    esp_err_t err = format_event(buf, sizeof(buf), event);
    if (err == ESP_OK) {
        // Send via USB (non-blocking)
        usb_cdc_send_line(buf);
    } else {
        ESP_LOGW(TAG, "Failed to format event type %d: %d", (int)event->type, err);
    }
}

/* ==========================================================================
 * Utility Functions
 * ========================================================================== */

uint32_t event_get_queue_high_watermark(void)
{
    return s_queue_high_watermark;
}
