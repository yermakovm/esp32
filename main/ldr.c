#include "ldr.h"
#include "config.h"

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_log.h>

static const char *TAG = "LDR";

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;

esp_err_t ldr_init(void)
{
    // Initialize ADC unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = LDR_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // Configure ADC channel
    adc_oneshot_chan_cfg_t chan_config = {
        .atten  = LDR_ADC_ATTEN,
        .bitwidth = LDR_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL, &chan_config));

    // Initialize calibration (curve fitting)
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = LDR_ADC_UNIT,
        .chan    = LDR_ADC_CHANNEL,
        .bitwidth = LDR_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));

    ESP_LOGI(TAG, "initialized on ADC%d_CH%d", LDR_ADC_UNIT, LDR_ADC_CHANNEL);
    return ESP_OK;
}

bool ldr_read_mv(int *out_mv)
{
    int raw = 0;
    if (adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &raw) != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed");
        return false;
    }
    if (adc_cali_raw_to_voltage(cali_handle, raw, out_mv) != ESP_OK) {
        ESP_LOGE(TAG, "calibration conversion failed");
        return false;
    }
    ESP_LOGD(TAG, "raw=%d mv=%d", raw, *out_mv);
    return true;
}