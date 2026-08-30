/**
 * Encoder-based Safe Vault
 * 
 * Rotary encoder controls digit input (0-9)
 * Button press submits current digit and moves to next
 * Button press on 4th digit submits the PIN attempt
 * Maximum 3 failed attempts trigger alarm and block input
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

static const char *TAG = "safe_vault";

// PIN Configuration

#define VAULT_PIN_CODE_LEN    4
static const uint8_t VAULT_PIN_CODE[VAULT_PIN_CODE_LEN] = {1, 2, 3, 4}; // Default: 1234

// GPIO Pin Definitions

#define ENCODER_SW_PIN        GPIO_NUM_10   // Button (active low)
#define ENCODER_DI_PIN        GPIO_NUM_11   // Data pin
#define ENCODER_CLK_PIN       GPIO_NUM_12   // Clock pin

#define BUZZER_PIN            GPIO_NUM_5    // Buzzer
#define GREEN_LED_PIN         GPIO_NUM_6    // Success LED
#define RED_LED_PIN           GPIO_NUM_7    // Alarm LED

#define BUZZER_LEDC_CHANNEL   LEDC_CHANNEL_0
#define BUZZER_LEDC_TIMER     LEDC_TIMER_0
#define BUZZER_LEDC_MODE      LEDC_LOW_SPEED_MODE

typedef enum {
    STATE_INPUT,        // Accepting encoder input
    STATE_BLOCKED,      // After 3 failed attempts, blocked until reset
    STATE_UNLOCKED      // Successful unlock
} vault_state_t;

typedef enum {
    DIRECTION_NONE = 0,
    DIRECTION_CW,       // Clockwise - increment
    DIRECTION_CCW       // Counter-clockwise - decrement
} encoder_dir_t;

static vault_state_t    s_vault_state   = STATE_INPUT;
static uint8_t          s_current_digit = 0;
static uint8_t          s_digit_index   = 0;
static uint8_t          s_attempt_code[VAULT_PIN_CODE_LEN];
static uint8_t          s_failed_tries  = 0;
static encoder_dir_t    s_last_direction = DIRECTION_NONE;

static void log_pin_attempt(const uint8_t* code, size_t len)
{
    char buf[32] = {0};
    size_t pos = 0;
    for (size_t i = 0; i < len && pos < sizeof(buf) - 2; i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%d", code[i]);
    }
    ESP_LOGI(TAG, "PIN attempt: [%s]", buf);
}

static void log_current_input(void)
{
    char entered[VAULT_PIN_CODE_LEN + 1] = {0};
    for (int i = 0; i < s_digit_index; i++) {
        entered[i] = '0' + s_attempt_code[i];
    }
    entered[s_digit_index] = '_';
    entered[s_digit_index + 1] = '\0';
    
    ESP_LOGI(TAG, "Current input: [%s]  Digit #%d=%d  Direction: %s",
             entered, s_digit_index + 1, s_current_digit,
             s_last_direction == DIRECTION_CW ? "CW (+)" :
             s_last_direction == DIRECTION_CCW ? "CCW (-)" : "NONE");
}

static void green_led_on(void)
{
    gpio_set_level(GREEN_LED_PIN, 1);
    gpio_set_level(RED_LED_PIN, 0);
}

static void red_led_on(void)
{
    gpio_set_level(RED_LED_PIN, 1);
    gpio_set_level(GREEN_LED_PIN, 0);
}

static void leds_off(void)
{
    gpio_set_level(GREEN_LED_PIN, 0);
    gpio_set_level(RED_LED_PIN, 0);
}

typedef struct {
    uint32_t freq;
    uint32_t duration_ms;
} tone_t;

// Success melody: rising cheerful tones (C5→E5→G5→C6)
static const tone_t SUCCESS_MELODY[] = {
    {523, 120},
    {659, 120},
    {784, 120},
    {1047, 300},
};
static const size_t SUCCESS_MELODY_LEN = sizeof(SUCCESS_MELODY) / sizeof(tone_t);

// Failure/alarm melody: descending ominous tones
static const tone_t ALARM_MELODY[] = {
    {440, 200},
    {330, 200},
    {220, 300},
    {165, 400},
};
static const size_t ALARM_MELODY_LEN = sizeof(ALARM_MELODY) / sizeof(tone_t);

static void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms)
{
    if (freq_hz == 0) {
    
        ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
        ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
        return;
    }


    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq_hz);
    

    uint32_t duty = (1 << LEDC_TIMER_13_BIT) / 2; // 50% of max
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, duty);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);


    vTaskDelay(pdMS_TO_TICKS(duration_ms));


    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

static void buzzer_init(void)
{

    ledc_timer_config_t timer_cfg = {
        .speed_mode = BUZZER_LEDC_MODE,
        .timer_num  = BUZZER_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz    = 440,              // Default frequency
        .clk_cfg    = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));


    ledc_channel_config_t chan_cfg = {
        .gpio_num   = BUZZER_PIN,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel    = BUZZER_LEDC_CHANNEL,
        .timer_sel  = BUZZER_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan_cfg));


    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

static void play_success_melody(void)
{
    ESP_LOGI(TAG, "Playing SUCCESS melody");
    for (size_t i = 0; i < SUCCESS_MELODY_LEN; i++) {
        buzzer_tone(SUCCESS_MELODY[i].freq, SUCCESS_MELODY[i].duration_ms);
        vTaskDelay(pdMS_TO_TICKS(30)); // Small gap between notes
    }
}

static void play_alarm_melody(void)
{
    ESP_LOGI(TAG, "Playing ALARM melody");
    for (size_t i = 0; i < ALARM_MELODY_LEN; i++) {
        buzzer_tone(ALARM_MELODY[i].freq, ALARM_MELODY[i].duration_ms);
        if (i < ALARM_MELODY_LEN - 1) {
            vTaskDelay(pdMS_TO_TICKS(50)); // Gap between notes
        }
    }
}

static void play_digit_beep(void)
{
    buzzer_tone(880, 30); // A5 short beep
}

static void play_button_beep(void)
{
    buzzer_tone(1200, 50); // High click
}

static void verify_and_apply_pin(void);
static void reset_current_input(void)
{
    s_current_digit = 0;
    s_digit_index   = 0;
    s_last_direction = DIRECTION_NONE;
    ESP_LOGI(TAG, "Input reset - starting new PIN entry");
}

// Verify the entered PIN (assumes s_digit_index == VAULT_PIN_CODE_LEN)
static void verify_and_apply_pin(void)
{
    log_pin_attempt(s_attempt_code, VAULT_PIN_CODE_LEN);

    bool correct = (memcmp(s_attempt_code, VAULT_PIN_CODE, VAULT_PIN_CODE_LEN) == 0);

    if (correct) {
        ESP_LOGI(TAG, "*** CORRECT PIN - VAULT UNLOCKED! ***");
        s_vault_state = STATE_UNLOCKED;
        green_led_on();
        play_success_melody();
    } else {
        s_failed_tries++;
        ESP_LOGW(TAG, "*** WRONG PIN - Attempt %d of 3 ***", s_failed_tries);

        // Blink red LED once after every failed attempt
        gpio_set_level(RED_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(RED_LED_PIN, 0);

        if (s_failed_tries >= 3) {
            ESP_LOGE(TAG, "*** ALARM! 3 failed attempts - SYSTEM BLOCKED! ***");
            s_vault_state = STATE_BLOCKED;
            red_led_on();
            play_alarm_melody();
        } else {
            ESP_LOGI(TAG, "Try again - %d attempts remaining", 3 - s_failed_tries);
            // Failure beep
            buzzer_tone(200, 300);
        }
    }

    // Reset attempt
    reset_current_input();
}

static void handle_encoder_rotation(encoder_dir_t direction)
{
    if (s_vault_state == STATE_BLOCKED || s_vault_state == STATE_UNLOCKED) {
        ESP_LOGW(TAG, "Input blocked - system in %s state",
                 s_vault_state == STATE_BLOCKED ? "BLOCKED" : "UNLOCKED");
        return;
    }

    if (direction == DIRECTION_NONE) return;

    // Guard: all 4 digits entered - no more input until button submits
    if (s_digit_index >= VAULT_PIN_CODE_LEN) {
        ESP_LOGW(TAG, "All 4 digits entered, press button to submit");
        return;
    }

    // Direction change: commit digit
    if (s_last_direction != DIRECTION_NONE && direction != s_last_direction) {
        if (s_digit_index < VAULT_PIN_CODE_LEN) {
            s_attempt_code[s_digit_index] = s_current_digit;
            ESP_LOGI(TAG, "Direction change: digit[%d]=%d committed",
                     s_digit_index, s_current_digit);
            s_digit_index++;
            s_current_digit = 0;
            play_digit_beep();
        }

        // Auto-submit on 4th digit
        if (s_digit_index >= VAULT_PIN_CODE_LEN) {
            verify_and_apply_pin();
            return;
        }
    }

    if (direction == DIRECTION_CW) {
        s_current_digit = (s_current_digit + 1) % 10;
        ESP_LOGI(TAG, "Encoder CW: digit=%d", s_current_digit);
    } else if (direction == DIRECTION_CCW) {
        s_current_digit = (s_current_digit == 0) ? 9 : (s_current_digit - 1);
        ESP_LOGI(TAG, "Encoder CCW: digit=%d", s_current_digit);
    }

    s_last_direction = direction;
    log_current_input();
}

static void handle_button_press(void)
{
    ESP_LOGI(TAG, "Button pressed - resetting current attempt");
    
    if (s_vault_state == STATE_BLOCKED) {
        ESP_LOGW(TAG, "System blocked - 3 failed attempts already");
        return;
    }

    if (s_vault_state == STATE_UNLOCKED) {
        ESP_LOGI(TAG, "System already unlocked - resetting to locked state");
        s_vault_state = STATE_INPUT;
        s_failed_tries = 0;
        leds_off();
        reset_current_input();
        return;
    }

    play_button_beep();

    // If we already have a complete PIN (from last direction change), verify it
    if (s_digit_index >= VAULT_PIN_CODE_LEN) {
        verify_and_apply_pin();
        return;
    }

    // Reset current input
    reset_current_input();
}

// Encoder polling (timer-based, no GPIO interrupt for simplicity)

static int s_last_clk = -1;
static int s_last_di = -1;

static void encoder_poll_cb(void *arg)
{
    int clk = gpio_get_level(ENCODER_CLK_PIN);
    int di  = gpio_get_level(ENCODER_DI_PIN);

    if (s_last_clk == -1) {
        s_last_clk = clk;
        s_last_di = di;
        return;
    }

    // Detect falling edge on CLK (active low encoder)
    if (s_last_clk == 1 && clk == 0) {
        encoder_dir_t dir = (di == 0) ? DIRECTION_CW : DIRECTION_CCW;
        handle_encoder_rotation(dir);
    }

    s_last_clk = clk;
    s_last_di = di;
}

static volatile uint32_t s_last_button_time = 0;
#define BUTTON_DEBOUNCE_MS  300

static TaskHandle_t s_button_task_handle = NULL;

static void IRAM_ATTR button_isr(void *arg)
{
    // Button debounce + level check
    if (gpio_get_level(ENCODER_SW_PIN) != 0) {
        return;
    }

    uint32_t now = xTaskGetTickCountFromISR() * (1000 / configTICK_RATE_HZ);
    if (now - s_last_button_time > BUTTON_DEBOUNCE_MS) {
        s_last_button_time = now;
        BaseType_t higher_priority_woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_button_task_handle, &higher_priority_woken);
        if (higher_priority_woken) {
            portYIELD_FROM_ISR();
        }
    }
}

static void button_task(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        handle_button_press();
    }
}

static void gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENCODER_SW_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << ENCODER_DI_PIN) | (1ULL << ENCODER_CLK_PIN);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);


    io_conf.pin_bit_mask = (1ULL << GREEN_LED_PIN) | (1ULL << RED_LED_PIN);
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Buzzer pin is handled by LEDC in buzzer_init(), not here
    // (LEDC channel config takes ownership of the pin)


    leds_off();


    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_SW_PIN, button_isr, NULL);
}

void app_main(void)
{

    ESP_LOGI(TAG, "Safe Vault Starting...");
    ESP_LOGI(TAG, "PIN Code: %d%d%d%d", VAULT_PIN_CODE[0], VAULT_PIN_CODE[1], 
             VAULT_PIN_CODE[2], VAULT_PIN_CODE[3]);
    ESP_LOGI(TAG, "GPIO Config:");
    ESP_LOGI(TAG, "  Encoder SW:  GPIO%d", ENCODER_SW_PIN);
    ESP_LOGI(TAG, "  Encoder DI:  GPIO%d", ENCODER_DI_PIN);
    ESP_LOGI(TAG, "  Encoder CLK: GPIO%d", ENCODER_CLK_PIN);
    ESP_LOGI(TAG, "  Buzzer:      GPIO%d", BUZZER_PIN);
    ESP_LOGI(TAG, "  Green LED:   GPIO%d", GREEN_LED_PIN);
    ESP_LOGI(TAG, "  Red LED:     GPIO%d", RED_LED_PIN);

    // Init buzzer
    buzzer_init();

    // Create button task before ISR
    TaskHandle_t button_handle;
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, &button_handle);
    s_button_task_handle = button_handle;

    // Init GPIO + ISR
    gpio_init();

    // Encoder poll timer
    const esp_timer_create_args_t poll_args = {
        .callback = encoder_poll_cb,
        .name     = "encoder_poll",
    };
    static esp_timer_handle_t s_poll_timer;
    ESP_ERROR_CHECK(esp_timer_create(&poll_args, &s_poll_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_poll_timer, 1000)); // 1ms polling

    ESP_LOGI(TAG, "Safe Vault initialized - waiting for input...");
    ESP_LOGI(TAG, "Rotate encoder to set digits (0-9)");
    ESP_LOGI(TAG, "Press button after each digit or to submit full PIN");
    ESP_LOGI(TAG, "Default PIN: %d%d%d%d", VAULT_PIN_CODE[0], VAULT_PIN_CODE[1], 
             VAULT_PIN_CODE[2], VAULT_PIN_CODE[3]);
}