#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

/* ============================================================
 * CONFIGURATION
 * ============================================================ */
#define ADC_CHANNEL     ADC_CHANNEL_9   // GPIO10 on ADC_UNIT_1

#define READ_MS         100              // sample interval
#define SAMPLES_PER_PRINT 5              // average N samples -> print every N*READ_MS

/* Full-scale voltage at the pin for the configured attenuation.
 * ADC_ATTEN_DB_6 saturates at ~1750 mV (NOT the 3.3 V rail!).
 * Used only by the naive manual estimate; calibration knows better. */
#define ADC_FS_MV       1750
#define VREF_MV         3300             // 3.3V reference

/* External voltage divider on the ADC input:
 * pot wiper --[10k]-- ADC pin --[10k]-- GND
 * Scales 0–3.3V down to 0–1.65V (inside the accurate 6 dB range).
 * Multiply readings back up by DIV_RATIO. */
#define DIV_R_TOP       10000.0f         // wiper -> ADC pin
#define DIV_R_BOT       10000.0f         // ADC pin -> GND
#define DIV_RATIO       ((DIV_R_TOP + DIV_R_BOT) / DIV_R_BOT)  /* = 2.0 */

/* ADC bit width — ESP32-S3 supports 12-bit */
#define ADC_BITWIDTH    12

static const char *TAG = "adc_single";

/* ============================================================
 * HELPER: raw → voltage (simple linear conversion)
 * ============================================================ */
static inline float raw_to_voltage_mv(int raw_val, int ref_mv, int bits)
{
    return (raw_val * (float)ref_mv) / ((1 << bits) - 1);
}

/* ============================================================
 * DATA STRUCTURE — one reading
 * ============================================================ */
typedef struct {
    int   raw;              // raw ADC counts (0–4095)
    float manual_mv;        // linear conversion from VREF
    float calibrated_mv;    // ESP-IDF curve-fitting calibration
    float error_pct;        // |calibrated − manual| / calibrated × 100
} adc_reading_t;

/* ============================================================
 * PRINT HEADER
 * ============================================================ */
static void print_header(void)
{
    printf("\n");
    printf("%-8s | %-14s | %-18s | %-9s\n",
           "Raw", "V_manual(mV)", "V_calibrated(mV)", "Error(%)");
    printf("---------|----------------|--------------------|----------\n");
}

/* ============================================================
 * PRINT ONE ROW
 * ============================================================ */
static void print_row(const adc_reading_t *r)
{
    printf("%-8d | %-14.2f | %-18.2f | %+9.3f\n",
           r->raw,
           r->manual_mv,
           r->calibrated_mv,
           r->error_pct);
}

/* ============================================================
 * TASK: read ADC1 (raw + calibrated)
 * ============================================================ */
static void adc_read_task(void *arg)
{
    /* --- Init ADC unit --- */
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    /* --- Config channel --- */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN_DB_6,    // 0–~1.75V usable (better accuracy than 12 dB)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg));

    /* --- Calibration handle --- */
    adc_cali_handle_t cali = NULL;
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_6,
        .bitwidth = ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali));

    /* --- Main loop: sample every READ_MS, print average of
     * SAMPLES_PER_PRINT readings (every N*READ_MS ms) --- */
    adc_reading_t r;
    bool use_calib = (cali != NULL);

    printf("\n*** ADC1 READING (avg of %d, %s) ***\n",
           SAMPLES_PER_PRINT,
           use_calib ? "raw + calibrated" : "calibration unavailable");
    print_header();

    while (1) {
        /* Accumulate SAMPLES_PER_PRINT samples */
        uint32_t raw_sum = 0;
        for (int i = 0; i < SAMPLES_PER_PRINT; i++) {
            int raw;
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw));
            raw_sum += raw;
            vTaskDelay(pdMS_TO_TICKS(READ_MS));
        }
        r.raw = raw_sum / SAMPLES_PER_PRINT;

        /* Manual voltage: naive linear estimate from the attenuation's
         * full-scale, corrected for external divider */
        r.manual_mv = raw_to_voltage_mv(r.raw, ADC_FS_MV, ADC_BITWIDTH) * DIV_RATIO;

        /* Calibrated voltage: curve-fitting, already in mV, corrected too */
        if (use_calib) {
            int mv;
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali, r.raw, &mv));
            r.calibrated_mv = mv * DIV_RATIO;
        } else {
            r.calibrated_mv = r.manual_mv;
        }

        /* Relative error between the two estimates, as % of calibrated.
         * Guard against division by zero at 0V. */
        r.error_pct = (fabsf(r.calibrated_mv) > 1e-6f)
            ? (r.calibrated_mv - r.manual_mv) / r.calibrated_mv * 100.0f
            : 0.0f;

        print_row(&r);
        vTaskDelay(pdMS_TO_TICKS(READ_MS));
    }
}

/* ============================================================
 * ENTRY POINT
 * ============================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 Single ADC Potentiometer Reading");
    ESP_LOGI(TAG, "ADC Channel: CH9 (GPIO10)");
    ESP_LOGI(TAG, "ADC FS: %d mV @6dB  |  Bitwidth: %d  |  Sample: %d ms  |  Print: avg of %d (%d ms)",
             ADC_FS_MV, ADC_BITWIDTH, READ_MS, SAMPLES_PER_PRINT, READ_MS * SAMPLES_PER_PRINT);
    ESP_LOGI(TAG, "Divider: %.0fk/%.0fk (x%.4f)",
             DIV_R_TOP / 1000.0f, DIV_R_BOT / 1000.0f, DIV_RATIO);

    xTaskCreate(adc_read_task, "adc_read", 4096, NULL, 5, NULL);
}
