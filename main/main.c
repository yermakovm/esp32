#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#define ADC_UNIT          ADC_UNIT_1
#define ADC_CHANNEL       ADC_CHANNEL_9   // GPIO10 on ADC_UNIT_1

#define LED_GPIO          GPIO_NUM_5      // LED, active-high, PWM
#define MOTOR_GPIO        GPIO_NUM_4      // DC motor (via driver/MOSFET), PWM

#define PWM_TIMER         LEDC_TIMER_0
#define PWM_MODE          LEDC_LOW_SPEED_MODE
#define PWM_FREQ_HZ       (10000)         // 10 kHz: above most motor PWM drive ranges
#define PWM_RES           LEDC_TIMER_12_BIT
#define PWM_MAX_DUTY      ((1 << 12) - 1) // 4095
#define LED_CHANNEL       LEDC_CHANNEL_0
#define MOTOR_CHANNEL     LEDC_CHANNEL_1

#define READ_MS           100             // update LED + motor every 100 ms
#define SMA_SIZE          8               // moving average window (samples)
#define ADC_BITWIDTH      12
#define ADC_MIN_RAW       100             // pot endpoints: ignore the noisy edges
#define ADC_MAX_RAW       3600            // ~1.65 V through the 10k/10k divider (DB_6)
#define PWM_FLOOR_DUTY    700             // below this the motor only buzzes/stalls (set 0 to disable)
#define PRINT_EVERY       5               // status line cadence (x READ_MS)

static const char *TAG = "pot_pwm";

typedef struct {
    int buf[SMA_SIZE];   // circular buffer
    int idx;             // next slot to overwrite (oldest when full)
    int count;           // filled samples (<= SMA_SIZE)
    int sum;             // running sum of the window
} sma_t;

static void sma_init(sma_t *s)
{
    memset(s, 0, sizeof(*s));
}

/* Push one sample, return the current mean of the window. */
static int sma_push(sma_t *s, int sample)
{
    if (s->count < SMA_SIZE) {
        s->buf[s->idx] = sample;
        s->sum += sample;
        s->count++;
    } else {
        s->sum -= s->buf[s->idx];   // drop oldest
        s->buf[s->idx] = sample;
        s->sum += sample;
    }
    s->idx = (s->idx + 1) % SMA_SIZE;
    return s->sum / s->count;
}

static int clamp_i(int v, int lo, int hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

/* Map the pot's usable range (ADC_MIN_RAW..ADC_MAX_RAW) onto 0..PWM_MAX_DUTY. */
static int duty_from_raw(int raw)
{
    int span = ADC_MAX_RAW - ADC_MIN_RAW;
    int d = ((clamp_i(raw, ADC_MIN_RAW, ADC_MAX_RAW) - ADC_MIN_RAW) * PWM_MAX_DUTY) / span;
    return clamp_i(d, 0, PWM_MAX_DUTY);
}

/* ============================================================
 * PWM: one timer, two channels, one shared duty value
 * ============================================================ */
static void pwm_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    const struct {
        ledc_channel_t ch;
        gpio_num_t     gpio;
    } chans[] = {
        { LED_CHANNEL,   LED_GPIO   },
        { MOTOR_CHANNEL, MOTOR_GPIO },
    };

    for (size_t i = 0; i < sizeof(chans) / sizeof(chans[0]); i++) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = chans[i].gpio,
            .speed_mode = PWM_MODE,
            .channel    = chans[i].ch,
            .timer_sel  = PWM_TIMER,   // both channels share the timer -> same frequency
            .duty       = 0,           // start off
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    }
}

static void pwm_set_both(uint32_t duty)
{
    const ledc_channel_t chans[] = { LED_CHANNEL, MOTOR_CHANNEL };

    for (size_t i = 0; i < sizeof(chans) / sizeof(chans[0]); i++) {
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, chans[i], duty));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, chans[i]));
    }
}

static void pot_pwm_task(void *arg)
{
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH,
.atten = ADC_ATTEN_DB_6,    // 10k/10k divider halves 0..3.3 V -> ~0..1.65 V (DB_6 range)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg));

    sma_t sma;
    sma_init(&sma);

    /* Prime the filter so the first loop iteration isn't a jump from 0. */
    for (int i = 0; i < SMA_SIZE; i++) {
        int raw;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw));
        sma_push(&sma, raw);
        vTaskDelay(pdMS_TO_TICKS(READ_MS));
    }

    uint32_t last_duty = 0;
    int loop = 0;

    while (1) {
        int raw;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw));
        int filtered = sma_push(&sma, raw);

        uint32_t duty = (uint32_t)duty_from_raw(filtered);
        if (duty > 0 && duty < PWM_FLOOR_DUTY) {
            duty = PWM_FLOOR_DUTY;
        }

        if (duty != last_duty) {
            pwm_set_both(duty);
            last_duty = duty;
        }

        if (++loop % PRINT_EVERY == 0) {
            printf("raw=%-4d filtered=%-4d | duty=%-4u/%-4d (%2u%%) -> LED GPIO%d + MOTOR GPIO%d\n",
                   raw, filtered, (unsigned)duty, PWM_MAX_DUTY,
                   (unsigned)(duty * 100 / PWM_MAX_DUTY), LED_GPIO, MOTOR_GPIO);
        }

        vTaskDelay(pdMS_TO_TICKS(READ_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 potentiometer -> LED + DC motor (PWM)");
ESP_LOGI(TAG, "ADC: unit1 CH9 (GPIO10), oneshot, %d-bit, 6dB (10k/10k divider)", ADC_BITWIDTH);
    ESP_LOGI(TAG, "PWM: %d Hz (10 kHz = above audible motor drive), 12-bit duty, "
             "LED=GPIO%d ch%d, MOTOR=GPIO%d ch%d (share timer)",
             PWM_FREQ_HZ, LED_GPIO, LED_CHANNEL, MOTOR_GPIO, MOTOR_CHANNEL);
    ESP_LOGI(TAG, "loop=%d ms, SMA=%d, duty floor=%d/%d",
             READ_MS, SMA_SIZE, PWM_FLOOR_DUTY, PWM_MAX_DUTY);

    /* Start both outputs low before the PWM peripheral takes over the pins. */
    gpio_reset_pin(LED_GPIO);
    gpio_reset_pin(MOTOR_GPIO);

    pwm_init();

    xTaskCreate(pot_pwm_task, "pot_pwm", 4096, NULL, 5, NULL);
}
