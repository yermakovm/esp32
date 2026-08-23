#include "servo.h"
#include "config.h"

#include <stdint.h>
#include <driver/ledc.h>
#include <esp_log.h>

static const char *TAG = "SERVO";

static int val_clamp(int value, int val_min, int val_max)
{
    if (value < val_min) return val_min;
    if (value > val_max) return val_max;
    return value;
}

esp_err_t servo_init(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = SERVO_UNIT,
        .duty_resolution  = SERVO_RESOLUTION,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = SERVO_CHANNEL,
        .timer_sel      = SERVO_UNIT,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_PIN,
        .duty           = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    ESP_LOGI(TAG, "initialized on GPIO%d", SERVO_PIN);
    return ESP_OK;
}

esp_err_t servo_set_us(int pulse_us)
{
    pulse_us = val_clamp(pulse_us, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);

    // duty = (max_duty / period_us) * pulse_us
    uint32_t duty = (uint32_t)SERVO_MAX_DUTY * pulse_us / SERVO_PERIOD_US;
    return ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty);
}

esp_err_t servo_set_angle(int angle_deg)
{
    angle_deg = val_clamp(angle_deg, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);

    int pulse_us = SERVO_MIN_PULSE_US +
                   (angle_deg - SERVO_MIN_ANGLE_DEG) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) /
                   (SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG);
    esp_err_t err = servo_set_us(pulse_us);
    if (err == ESP_OK) {
        ESP_LOGD(TAG, "angle=%d us=%d", angle_deg, pulse_us);
    }
    return err;
}