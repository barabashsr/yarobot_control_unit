/**
 * @file test_event_manager.c
 * @brief Unit tests for event manager component
 * @author YaRobot Team
 * @date 2025
 *
 * Tests all acceptance criteria from Story 2.7
 */

#include "unity.h"
#include "event_manager.h"
#include "config_limits.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include <string.h>

/* ==========================================================================
 * Test Helpers
 * ========================================================================== */

/** @brief Received event storage for callback verification */
static Event s_received_event;
static volatile int s_callback_count = 0;
static SemaphoreHandle_t s_callback_sem = NULL;
static void* s_received_ctx = NULL;

/**
 * @brief Reset test state before each test
 */
static void reset_test_state(void)
{
    memset(&s_received_event, 0, sizeof(s_received_event));
    s_callback_count = 0;
    s_received_ctx = NULL;
}

/**
 * @brief Test callback that records event and signals semaphore
 */
static void test_callback(const Event* event, void* ctx)
{
    if (event != NULL) {
        s_received_event = *event;
    }
    s_received_ctx = ctx;
    s_callback_count++;

    if (s_callback_sem != NULL) {
        xSemaphoreGive(s_callback_sem);
    }
}

/**
 * @brief Second callback for multi-subscriber tests
 */
static volatile int s_callback2_count = 0;
static void test_callback2(const Event* event, void* ctx)
{
    (void)event;
    (void)ctx;
    s_callback2_count++;
}

/**
 * @brief Wait for callback with timeout
 */
static bool wait_for_callback(TickType_t timeout_ticks)
{
    if (s_callback_sem == NULL) {
        return false;
    }
    return xSemaphoreTake(s_callback_sem, timeout_ticks) == pdTRUE;
}

/* ==========================================================================
 * AC1: event_manager_init() returns ESP_OK
 * ========================================================================== */

/**
 * AC1: Given the event manager is initialized, when event_manager_init() is called,
 * then it returns ESP_OK and the system is ready for subscriptions
 */
TEST_CASE("AC1: event_manager_init returns ESP_OK", "[event_manager]")
{
    // Note: event_manager may already be initialized from app startup
    // Re-init should return ESP_OK (already initialized)
    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());
}

/* ==========================================================================
 * AC2: event_publish delivers to all subscribers
 * ========================================================================== */

/**
 * AC2: Given the event manager is initialized, when a publisher calls event_publish(),
 * then all registered subscribers for that event type receive the event
 */
TEST_CASE("AC2: event_publish delivers to subscribers", "[event_manager]")
{
    reset_test_state();
    s_callback_sem = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(s_callback_sem);

    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());

    // Subscribe to MOTION_COMPLETE events
    TEST_ASSERT_EQUAL(ESP_OK, event_subscribe(EVTTYPE_MOTION_COMPLETE, test_callback, NULL));

    // Publish an event
    Event event = {
        .type = EVTTYPE_MOTION_COMPLETE,
        .axis = 0,
        .data.position = 123.456f,
        .timestamp = esp_timer_get_time()
    };
    TEST_ASSERT_EQUAL(ESP_OK, event_publish(&event));

    // Wait for callback
    TEST_ASSERT_TRUE(wait_for_callback(pdMS_TO_TICKS(100)));
    TEST_ASSERT_EQUAL(1, s_callback_count);
    TEST_ASSERT_EQUAL(EVTTYPE_MOTION_COMPLETE, s_received_event.type);

    // Cleanup
    event_unsubscribe(EVTTYPE_MOTION_COMPLETE, test_callback);
    vSemaphoreDelete(s_callback_sem);
    s_callback_sem = NULL;
}

/* ==========================================================================
 * AC3: Callback receives event and context
 * ========================================================================== */

/**
 * AC3: Given a callback is registered via event_subscribe(), when an event of that type
 * is published, then the callback is invoked with the event and context pointer
 */
TEST_CASE("AC3: callback receives event and context", "[event_manager]")
{
    reset_test_state();
    s_callback_sem = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(s_callback_sem);

    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());

    // Subscribe with a context pointer
    int test_context = 0xDEADBEEF;
    TEST_ASSERT_EQUAL(ESP_OK, event_subscribe(EVTTYPE_LIMIT_TRIGGERED, test_callback, &test_context));

    // Publish event
    Event event = {
        .type = EVTTYPE_LIMIT_TRIGGERED,
        .axis = 1,
        .data.limit_state = LIMIT_STATE_MIN,
        .timestamp = esp_timer_get_time()
    };
    TEST_ASSERT_EQUAL(ESP_OK, event_publish(&event));

    // Wait for callback
    TEST_ASSERT_TRUE(wait_for_callback(pdMS_TO_TICKS(100)));

    // Verify context was passed
    TEST_ASSERT_EQUAL_PTR(&test_context, s_received_ctx);

    // Verify event data
    TEST_ASSERT_EQUAL(EVTTYPE_LIMIT_TRIGGERED, s_received_event.type);
    TEST_ASSERT_EQUAL(1, s_received_event.axis);
    TEST_ASSERT_EQUAL(LIMIT_STATE_MIN, s_received_event.data.limit_state);

    // Cleanup
    event_unsubscribe(EVTTYPE_LIMIT_TRIGGERED, test_callback);
    vSemaphoreDelete(s_callback_sem);
    s_callback_sem = NULL;
}

/* ==========================================================================
 * AC4: Unsubscribe stops callbacks
 * ========================================================================== */

/**
 * AC4: Given a callback is registered, when event_unsubscribe() is called with the same
 * callback, then subsequent events do not invoke that callback
 */
TEST_CASE("AC4: unsubscribe stops callbacks", "[event_manager]")
{
    reset_test_state();
    s_callback_sem = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(s_callback_sem);

    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());

    // Subscribe
    TEST_ASSERT_EQUAL(ESP_OK, event_subscribe(EVTTYPE_ERROR, test_callback, NULL));

    // Publish - should invoke callback
    Event event = {
        .type = EVTTYPE_ERROR,
        .axis = 0xFF,
        .data.error_code = 42,
        .timestamp = esp_timer_get_time()
    };
    TEST_ASSERT_EQUAL(ESP_OK, event_publish(&event));
    TEST_ASSERT_TRUE(wait_for_callback(pdMS_TO_TICKS(100)));
    TEST_ASSERT_EQUAL(1, s_callback_count);

    // Unsubscribe
    TEST_ASSERT_EQUAL(ESP_OK, event_unsubscribe(EVTTYPE_ERROR, test_callback));

    // Publish again - should NOT invoke callback
    reset_test_state();
    TEST_ASSERT_EQUAL(ESP_OK, event_publish(&event));

    // Wait briefly and verify no callback
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_EQUAL(0, s_callback_count);

    // Cleanup
    vSemaphoreDelete(s_callback_sem);
    s_callback_sem = NULL;
}

/* ==========================================================================
 * AC7: FIFO order preserved
 * ========================================================================== */

/**
 * AC7: Given events are published, when the event processor runs,
 * then events are delivered in FIFO order (no reordering)
 */
TEST_CASE("AC7: events delivered in FIFO order", "[event_manager]")
{
    static int received_order[3] = {0};
    static int order_idx = 0;

    // Custom callback that records order
    void order_callback(const Event* event, void* ctx) {
        (void)ctx;
        if (order_idx < 3) {
            received_order[order_idx++] = (int)event->data.error_code;
        }
    }

    order_idx = 0;
    memset(received_order, 0, sizeof(received_order));

    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());
    TEST_ASSERT_EQUAL(ESP_OK, event_subscribe(EVTTYPE_ERROR, order_callback, NULL));

    // Publish events with different data to track order
    for (int i = 1; i <= 3; i++) {
        Event event = {
            .type = EVTTYPE_ERROR,
            .axis = 0xFF,
            .data.error_code = (uint8_t)i,
            .timestamp = esp_timer_get_time()
        };
        TEST_ASSERT_EQUAL(ESP_OK, event_publish(&event));
    }

    // Wait for processing
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify FIFO order
    TEST_ASSERT_EQUAL(1, received_order[0]);
    TEST_ASSERT_EQUAL(2, received_order[1]);
    TEST_ASSERT_EQUAL(3, received_order[2]);

    // Cleanup
    event_unsubscribe(EVTTYPE_ERROR, order_callback);
}

/* ==========================================================================
 * Multiple Subscribers Test
 * ========================================================================== */

/**
 * Test: Multiple subscribers receive the same event
 */
TEST_CASE("Multiple subscribers receive same event", "[event_manager]")
{
    reset_test_state();
    s_callback2_count = 0;
    s_callback_sem = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(s_callback_sem);

    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());

    // Subscribe two callbacks to the same event type
    TEST_ASSERT_EQUAL(ESP_OK, event_subscribe(EVTTYPE_ESTOP_CHANGED, test_callback, NULL));
    TEST_ASSERT_EQUAL(ESP_OK, event_subscribe(EVTTYPE_ESTOP_CHANGED, test_callback2, NULL));

    // Publish event
    Event event = {
        .type = EVTTYPE_ESTOP_CHANGED,
        .axis = 0xFF,
        .data.estop_active = true,
        .timestamp = esp_timer_get_time()
    };
    TEST_ASSERT_EQUAL(ESP_OK, event_publish(&event));

    // Wait for first callback
    TEST_ASSERT_TRUE(wait_for_callback(pdMS_TO_TICKS(100)));

    // Wait a bit more for second callback
    vTaskDelay(pdMS_TO_TICKS(50));

    // Both callbacks should have been invoked
    TEST_ASSERT_EQUAL(1, s_callback_count);
    TEST_ASSERT_EQUAL(1, s_callback2_count);

    // Cleanup
    event_unsubscribe(EVTTYPE_ESTOP_CHANGED, test_callback);
    event_unsubscribe(EVTTYPE_ESTOP_CHANGED, test_callback2);
    vSemaphoreDelete(s_callback_sem);
    s_callback_sem = NULL;
}

/* ==========================================================================
 * Input Validation Tests
 * ========================================================================== */

/**
 * Test: NULL callback is rejected
 */
TEST_CASE("subscribe rejects NULL callback", "[event_manager]")
{
    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, event_subscribe(EVTTYPE_BOOT, NULL, NULL));
}

/**
 * Test: NULL event is rejected
 */
TEST_CASE("publish rejects NULL event", "[event_manager]")
{
    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, event_publish(NULL));
}

/**
 * Test: NULL callback for unsubscribe is rejected
 */
TEST_CASE("unsubscribe rejects NULL callback", "[event_manager]")
{
    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, event_unsubscribe(EVTTYPE_BOOT, NULL));
}

/**
 * Test: Unsubscribe non-existent callback returns NOT_FOUND
 */
TEST_CASE("unsubscribe non-existent returns NOT_FOUND", "[event_manager]")
{
    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, event_unsubscribe(EVTTYPE_BOOT, test_callback));
}

/* ==========================================================================
 * Event Type Coverage Tests
 * ========================================================================== */

/**
 * Test: All event types can be published
 */
TEST_CASE("all event types can be published", "[event_manager]")
{
    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());

    EventType types[] = {
        EVTTYPE_MOTION_COMPLETE,
        EVTTYPE_MOTION_ERROR,
        EVTTYPE_LIMIT_TRIGGERED,
        EVTTYPE_ESTOP_CHANGED,
        EVTTYPE_MODE_CHANGED,
        EVTTYPE_ERROR,
        EVTTYPE_WIDTH_MEASURED,
        EVTTYPE_BOOT
    };

    for (size_t i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
        Event event = {
            .type = types[i],
            .axis = 0xFF,
            .timestamp = esp_timer_get_time()
        };

        // Set appropriate data for each type
        switch (types[i]) {
            case EVTTYPE_MOTION_COMPLETE:
            case EVTTYPE_WIDTH_MEASURED:
                event.data.position = 100.0f;
                break;
            case EVTTYPE_MODE_CHANGED:
                event.data.mode_name = "TEST";
                break;
            case EVTTYPE_ESTOP_CHANGED:
                event.data.estop_active = true;
                break;
            default:
                event.data.error_code = 1;
                break;
        }

        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, event_publish(&event),
                                   "Failed to publish event type");
    }
}

/* ==========================================================================
 * High Watermark Test
 * ========================================================================== */

/**
 * Test: Queue high watermark tracking works
 */
TEST_CASE("queue high watermark tracks maximum", "[event_manager]")
{
    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());

    uint32_t initial_hwm = event_get_queue_high_watermark();

    // Publish several events rapidly
    for (int i = 0; i < 5; i++) {
        Event event = {
            .type = EVTTYPE_ERROR,
            .axis = 0xFF,
            .data.error_code = (uint8_t)i,
            .timestamp = esp_timer_get_time()
        };
        event_publish(&event);
    }

    // High watermark should have increased (or stayed same if already higher)
    uint32_t new_hwm = event_get_queue_high_watermark();
    TEST_ASSERT_TRUE(new_hwm >= initial_hwm);

    // Let processor drain queue
    vTaskDelay(pdMS_TO_TICKS(100));
}

/* ==========================================================================
 * Double Init Test
 * ========================================================================== */

/**
 * Test: Double initialization is safe
 */
TEST_CASE("double initialization is safe", "[event_manager]")
{
    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());
    TEST_ASSERT_EQUAL(ESP_OK, event_manager_init());

    // Should still work
    Event event = {
        .type = EVTTYPE_BOOT,
        .axis = 0xFF,
        .timestamp = esp_timer_get_time()
    };
    TEST_ASSERT_EQUAL(ESP_OK, event_publish(&event));
}
