#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "driver/gpio.h"

/* ============================================================
 * CONFIGURATION
 * ============================================================ */
#define ADC_CHANNEL     ADC_CHANNEL_9   // GPIO10 on ADC_UNIT_1 (unchanged from pot setup)
#define LED_GPIO        GPIO_NUM_5      // LED on GPIO5, active-high

#define READ_MS         100             // sample interval
#define SMA_SIZE        10              // simple moving average window (samples)
#define CALIB_SAMPLES   10              // init measurements -> baseline (== SMA_SIZE, primes the filter)
#define HYST_STEP       200             // hysteresis band half-width (raw counts, baseline +/- 200)
#define PRINT_EVERY     5               // status line cadence (x READ_MS)

#define ADC_BITWIDTH    12              // ESP32-S3 supports 12-bit

/* LDR voltage divider at the ADC input (same 10k/10k divider as the old pot).
 * Measured behavior on the bench:
 *
 *   bright -> HIGH raw counts (~3600)
 *   dark   -> LOW  raw counts (~0)
 *
 * => DARK means a LOW reading, so the LED turns ON in the dark.
 */

static const char *TAG = "ldr_led";

/* ============================================================
 * SIMPLE MOVING AVERAGE (SMA) FILTER
 * Circular buffer of the last SMA_SIZE samples; output is the
 * mean of the window. Running-sum keeps each push O(1).
 * ============================================================ */
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

/* ============================================================
 * LED
 * ============================================================ */
static void led_set(bool on)
{
    gpio_set_level(LED_GPIO, on ? 1 : 0);
}

/* ============================================================
 * TASK: calibrate, then filter LDR + hysteresis-driven LED
 * ============================================================ */
static void ldr_led_task(void *arg)
{
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN_DB_6,   // 0–~1.75V usable (matches 10k/10k divider)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg));

    sma_t sma;
    sma_init(&sma);

    int baseline = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        int raw;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw));
        baseline += raw;
        sma_push(&sma, raw);
        vTaskDelay(pdMS_TO_TICKS(READ_MS));
    }
    baseline /= CALIB_SAMPLES;

    const int max_raw = (1 << ADC_BITWIDTH) - 1;
    int thr_dark  = baseline - HYST_STEP;
    int thr_light = baseline + HYST_STEP;
    if (thr_dark < 0)         thr_dark  = 0;
    if (thr_light > max_raw)  thr_light = max_raw;

     * If you calibrate in near-saturated light (raw ~3600),
     * thr_light lands above the divider's max output and the LED
     * can turn ON but never OFF again. */

    ESP_LOGI(TAG, "calibrated: baseline=%d  dark_on<=%d  light_off>=%d",
             baseline, thr_dark, thr_light);

    bool led_on = false;
    int loop = 0;

    while (1) {
        int raw;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw));
        int filtered = sma_push(&sma, raw);

        if (!led_on && filtered <= thr_dark) {
            led_on = true;
            led_set(true);
            ESP_LOGI(TAG, "dark  (filtered=%d <= %d) -> LED ON", filtered, thr_dark);
        } else if (led_on && filtered >= thr_light) {
            led_on = false;
            led_set(false);
            ESP_LOGI(TAG, "light (filtered=%d >= %d) -> LED OFF", filtered, thr_light);
        }

        if (++loop % PRINT_EVERY == 0) {
            printf("raw=%-4d filtered=%-4d | dark_on<=%d light_off>=%d | LED=%s\n",
                   raw, filtered, thr_dark, thr_light, led_on ? "ON " : "off");
        }

        vTaskDelay(pdMS_TO_TICKS(READ_MS));
    }
}

/* ============================================================
 * ENTRY POINT
 * ============================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 LDR light sensor -> LED");
    ESP_LOGI(TAG, "ADC: CH9 (GPIO10), oneshot, %d-bit, 6dB", ADC_BITWIDTH);
    ESP_LOGI(TAG, "LED: GPIO%d (on in dark)  SMA=%d  hysteresis=+/-%d",
             LED_GPIO, SMA_SIZE, HYST_STEP);

    /* --- LED GPIO (output, active-high, start off) --- */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    led_set(false);

    xTaskCreate(ldr_led_task, "ldr_led", 4096, NULL, 5, NULL);
}
