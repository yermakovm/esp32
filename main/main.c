/**
 * Traffic light simulator for ESP32-S3 (ESP-IDF v5.4, standard timer driver).
 *
 * Design:
 *  - A hardware timer group (TIMER_GROUP_0 / TIMER_0) fires an interrupt
 *    every TICK_MS in auto-reload mode.
 *  - The ISR is short and fast: it re-enables the interrupt, advances the
 *    tick counter and pokes the main loop via a queue. No logging, no
 *    GPIO, no other I/O inside the ISR.
 *  - The main loop owns everything else: it consumes ticks, detects state
 *    expiry, logs transitions, updates the LEDs and drives the green
 *    blink by comparing elapsed time (esp_timer_get_time, i.e. millis)
 *    against the blink period.
 *
 * State sequence (cyclic, starts at RED like in real life):
 *  RED (5 s) -> RED+YELLOW (2 s) -> GREEN (5 s) -> GREEN BLINK (2 s)
 *  -> YELLOW (3 s) -> RED ...
 */

#include <assert.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "traffic";

/* ---------------------------- LED wiring ---------------------------- */

#define LED_GREEN  GPIO_NUM_4
#define LED_YELLOW GPIO_NUM_5
#define LED_RED    GPIO_NUM_6

/* --------------------------- State machine -------------------------- */

typedef enum {
    STATE_RED,
    STATE_RED_YELLOW,
    STATE_GREEN,
    STATE_GREEN_BLINK,
    STATE_YELLOW,
    STATE_MAX
} traffic_state_t;

static const struct {
    const char *name;
    uint32_t duration_ms;
} state_table[STATE_MAX] = {
    [STATE_RED]         = { "RED",          5000 },
    [STATE_RED_YELLOW]  = { "RED + YELLOW", 2000 },
    [STATE_GREEN]       = { "GREEN",        5000 },
    [STATE_GREEN_BLINK] = { "GREEN BLINK",  2000 },
    [STATE_YELLOW]      = { "YELLOW",       3000 },
};

/* ------------------------- Timing constants ------------------------- */

#define TICK_MS           100   /* timer interrupt period       */
#define BLINK_TOGGLE_MS   500   /* green on/off period: 2 Hz    */

/* ------------------------------ Context ----------------------------- */

static volatile traffic_state_t s_state = STATE_RED;   /* start RED, like IRL */
static volatile uint32_t s_ticks_in_state = 0;  /* ticks spent in s_state */
static bool s_green_on = false;                    /* green LED blink phase */
static int64_t s_last_blink_us = 0;                /* when green last toggled */
static QueueHandle_t s_tick_queue;                 /* ISR -> main loop       */

/* ------------------------- LED state apply -------------------------- */

static void apply_leds(traffic_state_t state)
{
    switch (state) {
    case STATE_RED:
        gpio_set_level(LED_GREEN, 0);
        gpio_set_level(LED_YELLOW, 0);
        gpio_set_level(LED_RED, 1);
        break;
    case STATE_RED_YELLOW:
        gpio_set_level(LED_GREEN, 0);
        gpio_set_level(LED_YELLOW, 1);
        gpio_set_level(LED_RED, 1);
        break;
    case STATE_GREEN:
        s_green_on = true;
        gpio_set_level(LED_GREEN, 1);
        gpio_set_level(LED_YELLOW, 0);
        gpio_set_level(LED_RED, 0);
        break;
    case STATE_GREEN_BLINK:
        s_green_on = true;
        gpio_set_level(LED_GREEN, 1);
        gpio_set_level(LED_YELLOW, 0);
        gpio_set_level(LED_RED, 0);
        s_last_blink_us = esp_timer_get_time();
        break;
    case STATE_YELLOW:
        gpio_set_level(LED_GREEN, 0);
        gpio_set_level(LED_YELLOW, 1);
        gpio_set_level(LED_RED, 0);
        break;
    default:
        break;
    }
}

/* --------------------------- Timer ISR ------------------------------ */
/*
 * Keep this short: re-enable the interrupt, advance the tick counter,
 * wake the main loop. No logging, no GPIO, no other I/O.
 */
static bool IRAM_ATTR timer_isr(void *arg)
{
    (void)arg;

    /* Re-arm the auto-reload timer interrupt */
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    s_ticks_in_state++;

    /* Wake the main loop; the payload is not used */
    uint32_t dummy = 0;
    xQueueSendFromISR(s_tick_queue, &dummy, NULL);

    return true;
}

/* ------------------------ Blink handling ---------------------------- */

static void handle_blink(void)
{
    if (s_state != STATE_GREEN_BLINK) {
        return;
    }

    int64_t now = esp_timer_get_time();
    if (now - s_last_blink_us < (int64_t)BLINK_TOGGLE_MS * 1000) {
        return;
    }

    s_last_blink_us = now;
    s_green_on = !s_green_on;
    gpio_set_level(LED_GREEN, s_green_on);
}

/* --------------------------- GPIO setup ----------------------------- */

static void configure_leds(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << LED_GREEN) | (1ULL << LED_YELLOW) | (1ULL << LED_RED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

/* --------------------------- Timer setup ---------------------------- */

static esp_err_t configure_timer(void)
{
    const timer_config_t tcfg = {
        .divider = 100,                          /* 160 MHz / 100 = 1.6 MHz */
        .auto_reload = false,
        .counter_dir = TIMER_COUNT_UP,
    };
    esp_err_t err = timer_init(TIMER_GROUP_0, TIMER_0, &tcfg);
    if (err != ESP_OK) {
        return err;
    }

    /* ticks per tick: (160 MHz / divider) * TICK_MS / 1000 */
    uint64_t period = (160000000ULL / tcfg.divider) * TICK_MS / 1000;
    err = timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, period);
    if (err != ESP_OK) {
        return err;
    }

    err = timer_set_auto_reload(TIMER_GROUP_0, TIMER_0, TIMER_AUTORELOAD_EN);
    if (err != ESP_OK) {
        return err;
    }

    err = timer_set_alarm(TIMER_GROUP_0, TIMER_0, TIMER_ALARM_EN);
    if (err != ESP_OK) {
        return err;
    }

    err = timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_isr, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }

    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    return timer_start(TIMER_GROUP_0, TIMER_0);
}

/* ----------------------------- app_main ----------------------------- */

void app_main(void)
{
    s_tick_queue = xQueueCreate(8, sizeof(uint32_t));
    assert(s_tick_queue != NULL);

    configure_leds();

    esp_err_t err = configure_timer();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to start timer: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "traffic light started");
    ESP_LOGI(TAG, "state: %s", state_table[STATE_RED].name);
    apply_leds(STATE_RED);

    for (;;) {
        uint32_t dummy;
        xQueueReceive(s_tick_queue, &dummy, portMAX_DELAY);

        /* Drive the green blink from elapsed time in the main loop */
        handle_blink();

        uint32_t ticks = s_ticks_in_state;
        if (ticks * TICK_MS >= state_table[s_state].duration_ms) {
            traffic_state_t next = (traffic_state_t)((s_state + 1) % STATE_MAX);

            /* Log the transition in the main loop */
            ESP_LOGI(TAG, "state: %s -> %s",
                     state_table[s_state].name, state_table[next].name);

            s_ticks_in_state = 0;
            s_state = next;
            apply_leds(next);
        }
    }
}
