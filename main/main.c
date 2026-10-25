#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_log.h"

#define LED_PWM_GPIO        4
#define LDR_ADC_GPIO        5
#define BUTTON_GPIO         GPIO_NUM_0

#define LEDC_DUTY_BITS      LEDC_TIMER_10_BIT
#define LEDC_PWM_FREQ_HZ    5000
#define PWM_MAX_DUTY        ((1 << LEDC_DUTY_BITS) - 1)

static const int MODE_PERCENT[] = { 100, 75, 50, 25 };
#define MODE_COUNT            (int)(sizeof(MODE_PERCENT) / sizeof(MODE_PERCENT[0]))

#define AUTO_CALIBRATE        1
#define CALI_SETTLE_MS        800
#define CALI_SAMPLES          50

#define MV_DARK_REF           300
#define MV_FULL_REF           2900
#define MIN_SPAN_MV           150

#define PID_KP                1000.0f
#define PID_KI                300.0f
#define PID_KD                50.0f
#define PID_DT_S              0.01f
#define PID_PERIOD_US         10000
#define PID_D_ALPHA           0.8f

#define ADC_ATTEN             ADC_ATTEN_DB_12
#define ADC_FILTER_ALPHA      0.4f

static const char *TAG = "ldr_led_pid";

static void led_pwm_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_DUTY_BITS,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = LEDC_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t chan_cfg = {
        .gpio_num   = LED_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan_cfg));
}

static void led_set_duty(uint32_t duty)
{
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t         cali_handle = NULL;
static adc_channel_t             ldr_chan = 0;
static bool                      cali_ok = false;

static void ldr_adc_init(void)
{
    adc_unit_t unit;
    adc_channel_t chan;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(LDR_ADC_GPIO, &unit, &chan));
    ldr_chan = chan;
    ESP_LOGI(TAG, "LDR on GPIO%d -> %s_CH%d", LDR_ADC_GPIO,
             unit == ADC_UNIT_1 ? "ADC1" : "ADC2", chan);

    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, chan, &chan_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = chan,
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle);
    cali_ok = (err == ESP_OK);
    if (!cali_ok) {
        ESP_LOGW(TAG, "ADC calibration unavailable (%s), using raw*3300/4095",
                 esp_err_to_name(err));
    }
}

static int ldr_read_mv(int *out_raw)
{
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ldr_chan, &raw));
    if (out_raw) *out_raw = raw;

    int mv = 0;
    if (cali_ok) {
        adc_cali_raw_to_voltage(cali_handle, raw, &mv);
    } else {
        mv = (int)((float)raw * 3300.0f / 4095.0f);
    }
    return mv;
}

typedef struct {
    float kp, ki, kd;
    float dt;
    float integral, integral_min, integral_max;
    float out_min, out_max;
    float prev_error;
    float d_filtered;
    float d_alpha;
    bool  enable_i;
    bool  enable_d;
} pid_t;

static void pid_init(pid_t *p, float kp, float ki, float kd, float dt, float out_min, float out_max)
{
    p->kp = kp; p->ki = ki; p->kd = kd; p->dt = dt;
    p->integral = 0.0f;
    p->out_min = out_min; p->out_max = out_max;
    p->integral_min = (ki > 0.0f) ? out_min / ki : 0.0f;
    p->integral_max = (ki > 0.0f) ? out_max / ki : 0.0f;
    p->prev_error = 0.0f;
    p->d_filtered = 0.0f;
    p->d_alpha = PID_D_ALPHA;
    p->enable_i = true;
    p->enable_d = (kd > 0.0f);
}

static float pid_compute(pid_t *p, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    float p_term = p->kp * error;

    float i_term = 0.0f;
    if (p->enable_i) {
        p->integral += error * p->dt;
        if (p->integral > p->integral_max) p->integral = p->integral_max;
        if (p->integral < p->integral_min) p->integral = p->integral_min;
        i_term = p->ki * p->integral;
    }

    float d_term = 0.0f;
    if (p->enable_d) {
        float d_raw = (error - p->prev_error) / p->dt;
        p->d_filtered = p->d_alpha * p->d_filtered + (1.0f - p->d_alpha) * d_raw;
        d_term = p->kd * p->d_filtered;
    }

    p->prev_error = error;

    float out = p_term + i_term + d_term;
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;
    return out;
}

static void pid_reset(pid_t *p)
{
    p->integral   = 0.0f;
    p->d_filtered = 0.0f;
}

static int mv_dark = MV_DARK_REF;
static int mv_full = MV_FULL_REF;
static int mv_span = MV_FULL_REF - MV_DARK_REF;

static int average_mv_at_duty(uint32_t duty, int samples)
{
    led_set_duty(duty);
    vTaskDelay(pdMS_TO_TICKS(CALI_SETTLE_MS));

    float acc = 0.0f;
    for (int i = 0; i < samples; i++) {
        int mv = ldr_read_mv(NULL);
        acc = (i == 0) ? (float)mv : (ADC_FILTER_ALPHA * mv + (1.0f - ADC_FILTER_ALPHA) * acc);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return (int)(acc + 0.5f);
}

static void calibrate_range(void)
{
#if AUTO_CALIBRATE
    ESP_LOGI(TAG, "calibrating: LED off ...");
    mv_dark = average_mv_at_duty(0, CALI_SAMPLES);

    ESP_LOGI(TAG, "calibrating: LED 100%% ...");
    mv_full = average_mv_at_duty(PWM_MAX_DUTY, CALI_SAMPLES);

    led_set_duty(0);
#else
    ESP_LOGI(TAG, "using fixed reference: %d / %d mV", MV_DARK_REF, MV_FULL_REF);
#endif

    mv_span = mv_full - mv_dark;
    ESP_LOGI(TAG, "reference: dark=%d mV, full=%d mV, span=%d mV", mv_dark, mv_full, mv_span);

    if (mv_span < MIN_SPAN_MV) {
        ESP_LOGE(TAG, "span too small (<%d mV): LED light does not reach the LDR", MIN_SPAN_MV);
        mv_full = mv_dark + MIN_SPAN_MV;
        mv_span = MIN_SPAN_MV;
    }
}

static int mode_index = 0;

static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

static bool check_button_and_advance_mode(void)
{
    static bool prev_pressed = false;
    static int64_t last_change_us = 0;

    bool pressed = (gpio_get_level(BUTTON_GPIO) == 0);
    int64_t now_us = esp_timer_get_time();

    bool changed = false;
    if (pressed && !prev_pressed && (now_us - last_change_us > 50000)) {
        mode_index = (mode_index + 1) % MODE_COUNT;
        last_change_us = now_us;
        changed = true;
        ESP_LOGI(TAG, "mode -> %d%%", MODE_PERCENT[mode_index]);
    }
    prev_pressed = pressed;
    return changed;
}

static int mode_target_mv(void)
{
    return mv_dark + (int)((float)mv_span * (float)MODE_PERCENT[mode_index] / 100.0f + 0.5f);
}

static TaskHandle_t pid_task_handle = NULL;

static bool IRAM_ATTR timer_alarm_cb(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    vTaskNotifyGiveFromISR(pid_task_handle, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static void pid_timer_init(void)
{
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = { .on_alarm = timer_alarm_cb };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = PID_PERIOD_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

void app_main(void)
{
    pid_task_handle = xTaskGetCurrentTaskHandle();

    led_pwm_init();
    ldr_adc_init();
    button_init();

    calibrate_range();
    pid_timer_init();

    pid_t light_pid;
    pid_init(&light_pid, PID_KP, PID_KI, PID_KD, PID_DT_S,
             0.0f, (float)PWM_MAX_DUTY);

    float filt_mv = 0.0f;
    bool  filt_init = false;
    int   target_mv = mode_target_mv();

    ESP_LOGI(TAG, "mode %d%% -> target %d mV (span %d mV)",
             MODE_PERCENT[mode_index], target_mv, mv_span);
    pid_reset(&light_pid);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (check_button_and_advance_mode()) {
            target_mv = mode_target_mv();
            pid_reset(&light_pid);
        }

        int raw = 0;
        int mv  = ldr_read_mv(&raw);

        filt_mv = filt_init ? (ADC_FILTER_ALPHA * mv + (1.0f - ADC_FILTER_ALPHA) * filt_mv)
                            : (float)mv;
        filt_init = true;

        float pv_norm     = (filt_mv - (float)mv_dark) / (float)mv_span;
        float target_norm = (float)(target_mv - mv_dark) / (float)mv_span;

        float output = pid_compute(&light_pid, target_norm, pv_norm);
        led_set_duty((uint32_t)output);

        printf("t_ms:%lld|mode:%d|target_mv:%d|pv_mv:%d|error_mV:%d|duty:%d\n",
               (long long)(esp_timer_get_time() / 1000),
               MODE_PERCENT[mode_index], target_mv, (int)filt_mv,
               target_mv - (int)filt_mv, (int)output);
    }
}
