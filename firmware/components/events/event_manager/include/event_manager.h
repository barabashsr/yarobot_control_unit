/**
 * @file event_manager.h
 * @brief Event publish/subscribe system for asynchronous communication
 * @author YaRobot Team
 * @date 2025
 *
 * @note Provides a pub/sub event system that decouples publishers from
 *       subscribers. Events are delivered asynchronously via a FreeRTOS
 *       queue and processor task.
 *
 * @details
 * - Uses Event and EventType from response_formatter.h (shared definitions)
 * - Events delivered to subscribers within 5ms of publish
 * - ISR-safe publish variant available
 * - Thread-safe subscriber management
 */

#ifndef EVENT_MANAGER_H
#define EVENT_MANAGER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "response_formatter.h"  // Reuse Event and EventType definitions

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup event_manager Event Manager
 * @brief Publish/subscribe event system
 * @{
 */

/**
 * @brief Event callback function pointer
 *
 * @param[in] event Pointer to the event being delivered
 * @param[in] ctx User-provided context pointer from subscribe
 */
typedef void (*EventCallback)(const Event* event, void* ctx);

/**
 * @brief Initialize the event manager
 *
 * Creates the event queue, subscriber storage, and processor task.
 * Must be called before any other event_manager functions.
 * Publishes EVT_BOOT event after initialization.
 *
 * @return
 *      - ESP_OK: Initialization successful
 *      - ESP_ERR_NO_MEM: Failed to allocate resources
 *      - ESP_ERR_INVALID_STATE: Already initialized
 */
esp_err_t event_manager_init(void);

/**
 * @brief Subscribe to events of a specific type
 *
 * Registers a callback to be invoked when events of the specified
 * type are published. The callback is invoked from the event
 * processor task context.
 *
 * @param[in] type Event type to subscribe to
 * @param[in] callback Function to call when event occurs
 * @param[in] ctx User context pointer passed to callback
 *
 * @return
 *      - ESP_OK: Subscription successful
 *      - ESP_ERR_INVALID_ARG: callback is NULL or type invalid
 *      - ESP_ERR_NO_MEM: Maximum subscribers reached
 *      - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t event_subscribe(EventType type, EventCallback callback, void* ctx);

/**
 * @brief Unsubscribe from events
 *
 * Removes a previously registered callback. After this call,
 * the callback will not be invoked for new events.
 *
 * @param[in] type Event type to unsubscribe from
 * @param[in] callback Callback function to remove
 *
 * @return
 *      - ESP_OK: Unsubscription successful
 *      - ESP_ERR_INVALID_ARG: callback is NULL or type invalid
 *      - ESP_ERR_NOT_FOUND: Callback not registered
 *      - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t event_unsubscribe(EventType type, EventCallback callback);

/**
 * @brief Publish an event
 *
 * Queues an event for delivery to all subscribers. The event is
 * copied to the queue, so the caller's event can be on the stack.
 * Non-blocking - returns immediately after queuing.
 *
 * If the queue is full, logs ERR_EVENT_OVERFLOW warning and drops
 * the oldest event to make room.
 *
 * @param[in] event Event to publish (copied to queue)
 *
 * @return
 *      - ESP_OK: Event queued successfully
 *      - ESP_ERR_INVALID_ARG: event is NULL
 *      - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t event_publish(const Event* event);

/**
 * @brief Publish an event from ISR context
 *
 * ISR-safe variant of event_publish(). Uses xQueueSendFromISR()
 * for non-blocking operation from interrupt handlers.
 *
 * @param[in] event Event to publish (copied to queue)
 * @param[out] woken Set to pdTRUE if higher priority task was woken
 *
 * @return
 *      - ESP_OK: Event queued successfully
 *      - ESP_ERR_INVALID_ARG: event or woken is NULL
 *      - ESP_ERR_INVALID_STATE: Not initialized or queue full
 *
 * @note Caller should call portYIELD_FROM_ISR(*woken) after return
 */
esp_err_t event_publish_from_isr(const Event* event, BaseType_t* woken);

/**
 * @brief Get event queue high watermark
 *
 * Returns the maximum number of events that were queued at any time.
 * Useful for tuning queue depth.
 *
 * @return High watermark count
 */
uint32_t event_get_queue_high_watermark(void);

/** @} */ // end event_manager

#ifdef __cplusplus
}
#endif

#endif // EVENT_MANAGER_H
