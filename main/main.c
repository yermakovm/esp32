#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

static const char *TAG = "PhotoRelay";

// Photoresistor on GPIO5 -> ADC1_CHANNEL4 on ESP32-S3
#define LDR_ADC_UNIT      ADC_UNIT_1
#define LDR_ADC_CHANNEL   ADC_CHANNEL_4
#define LDR_ATTEN         ADC_ATTEN_DB_11   // full 0V..3.3V range, 0..4095

// Relay (via transistor) on GPIO10
#define RELAY_GPIO        GPIO_NUM_10

#define CALIB_SAMPLES     32      // median of these = median light level at boot
#define CALIB_PERIOD_MS   20      // sample spacing during calibration
#define SAMPLE_PERIOD_MS  50      // steady-state sample spacing

#define ADC_RAW_MAX       4095

static adc_oneshot_unit_handle_t adc_handle;

// Simple insertion sort (fine for small N)
static void sort16(const uint16_t *in, uint16_t *out, int n)
{
    for (int i = 0; i < n; i++)
        out[i] = in[i];
    for (int i = 1; i < n; i++) {
        uint16_t key = out[i];
        int j = i - 1;
        while (j >= 0 && out[j] > key) {
            out[j + 1] = out[j];
            j--;
        }
        out[j + 1] = key;
    }
}

// Take CALIB_SAMPLES readings and return the median as the reference level
static uint32_t calibrate_median(void)
{
    static uint16_t samples[32];
    static uint16_t sorted[32];

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        int raw = 0;
        if (adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &raw) != ESP_OK || raw < 0)
            raw = 0;
        samples[i] = (uint16_t)raw;
        vTaskDelay(pdMS_TO_TICKS(CALIB_PERIOD_MS));
    }
    sort16(samples, sorted, CALIB_SAMPLES);

    int n = CALIB_SAMPLES;
    uint32_t median = (n % 2) ? sorted[n / 2] : (sorted[n / 2 - 1] + sorted[n / 2]) / 2;
    ESP_LOGI(TAG, "Calibration median = %lu", (unsigned long)median);
    return median;
}

void app_main(void)
{
    // --- ADC setup (12-bit) ---
    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = LDR_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = LDR_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL, &chan_cfg));

    // --- Relay output setup ---
    const gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << RELAY_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));
    gpio_set_level(RELAY_GPIO, 0);   // start with relay off

    // --- Initial calibration: dynamic threshold = 2 x median ---
    uint32_t median   = calibrate_median();
    uint32_t threshold = median * 2;
    if (threshold > ADC_RAW_MAX)
        threshold = ADC_RAW_MAX;
    ESP_LOGI(TAG, "Dynamic threshold = %lu", (unsigned long)threshold);

    // --- Main loop ---
    while (1) {
        int raw = 0;
        if (adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &raw) == ESP_OK) {
            if ((uint32_t)raw > threshold) {
                gpio_set_level(RELAY_GPIO, 1);   // brighter than median x2: close relay
            } else if ((uint32_t)raw < threshold) {
                gpio_set_level(RELAY_GPIO, 0);   // darker: open relay
            }
            // raw == threshold: leave output unchanged
            ESP_LOGI(TAG, "ADC GPIO5 = %d (threshold %lu)", raw, (unsigned long)threshold);
        } else {
            ESP_LOGW(TAG, "ADC read failed, retrying");
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
