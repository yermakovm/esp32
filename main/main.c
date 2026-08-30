#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "servo_pot";

#define TICK_MS           25              // one ADC sample + duty update

/* --- potentiometer on GPIO5 = ADC1 channel 4 --- */
#define POT_GPIO          GPIO_NUM_5    // ADC1_CHANNEL_4, no PWM/leDC on it
#define POT_ADC_UNIT      ADC_UNIT_1
#define POT_ADC_CHANNEL   ADC_CHANNEL_4   // GPIO5 on ADC_UNIT_1
#define POT_ATTEN         ADC_ATTEN_DB_12
#define ADC_BITWIDTH      ADC_BITWIDTH_12
#define ADC_BITS          12
#define ADC_RAW_MAX       ((1 << ADC_BITS) - 1)         // 4095

#define POT_RAW_MIN       0
#define POT_RAW_MAX       ADC_RAW_MAX

#define POT_INVERT        0

/* --- SG90 on GPIO10: 20 ms period (50 Hz), 0.5 ms = 0 deg, 2.5 ms = 180 deg --- */
#define SERVO_GPIO        GPIO_NUM_10
#define SERVO_MODE        LEDC_LOW_SPEED_MODE
#define SERVO_TIMER       LEDC_TIMER_0
#define SERVO_CHANNEL     LEDC_CHANNEL_0
#define SERVO_FREQ_HZ     50              // datasheet: 20 ms period
#define SERVO_RES         LEDC_TIMER_14_BIT
#define SERVO_RES_BITS    14
#define SERVO_PERIOD_US   (1000000 / SERVO_FREQ_HZ)     // 20000 us
#define SERVO_PULSE_MIN_US 500            // 0 deg  = leftmost / CCW end stop
#define SERVO_PULSE_MAX_US 2500           // 180 deg = rightmost / CW end stop
#define SERVO_ANGLE_MAX_X10 1800          // 180.0 deg, kept in tenths of degree

/* Ignore ADC jitter smaller than this, otherwise a stationary servo buzzes
 * around its position: 5 = 0.5 deg. */
#define ANGLE_DEADBAND_X10  5

/* Moving average over the pot samples: SMA_SIZE * TICK_MS = 200 ms window. */
#define SMA_SIZE          8

/* Print a status line every N ticks (0 = never). */
#define HEARTBEAT_TICKS   40              // 1 s

static int clamp_i(int v, int lo, int hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static int angle_x10_from_raw(int raw)
{
#if POT_INVERT
    raw = (POT_RAW_MIN + POT_RAW_MAX) - raw;
#endif
    raw = clamp_i(raw, POT_RAW_MIN, POT_RAW_MAX);
    return ((raw - POT_RAW_MIN) * SERVO_ANGLE_MAX_X10) / (POT_RAW_MAX - POT_RAW_MIN);
}

static int pulse_us_from_angle_x10(int angle_x10)
{
    angle_x10 = clamp_i(angle_x10, 0, SERVO_ANGLE_MAX_X10);
    return SERVO_PULSE_MIN_US +
           (angle_x10 * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)) / SERVO_ANGLE_MAX_X10;
}

static uint32_t duty_from_pulse_us(int pulse_us)
{
    return (uint32_t)(((int64_t)pulse_us << SERVO_RES_BITS) / SERVO_PERIOD_US);
}

static adc_oneshot_unit_handle_t s_adc;

typedef struct {
    int buf[SMA_SIZE];   // circular buffer
    int idx;             // next slot to overwrite (oldest when full)
    int count;           // filled samples (<= SMA_SIZE)
    int sum;             // running sum of the window
    int mean;            // mean of the current window
} sma_t;

static sma_t s_pot;

static int sma_push(sma_t *s, int sample)
{
    if (s->count < SMA_SIZE) {
        s->buf[s->idx] = sample;
        s->sum += sample;
        s->count++;
    } else {
        s->sum -= s->buf[s->idx];     // drop oldest
        s->buf[s->idx] = sample;
        s->sum += sample;
    }
    s->idx = (s->idx + 1) % SMA_SIZE;
    s->mean = s->sum / s->count;
    return s->mean;
}

static void pot_init(void)
{
    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = POT_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH,
        .atten    = POT_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, POT_ADC_CHANNEL, &chan_cfg));
}

static int pot_read(void)
{
    int raw = 0;
    esp_err_t err = adc_oneshot_read(s_adc, POT_ADC_CHANNEL, &raw);
    if (err != ESP_OK) {                 // never abort() from a timer task
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(err));
        return s_pot.mean;               // keep the last good value
    }
    return raw;
}

static int pot_prime(void)
{
    int mean = 0;
    for (size_t i = 0; i < SMA_SIZE; i++) {
        mean = sma_push(&s_pot, pot_read());
    }
    return mean;
}

static int      s_angle_x10;             // angle applied, tenths of degree
static int      s_pulse_us;              // pulse currently on the pin
static uint32_t s_duty;                  // its LEDC duty counts

static void servo_init(int angle_x10)
{
    gpio_reset_pin(SERVO_GPIO);          // release the pin from any other role

    const ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_MODE,
        .timer_num       = SERVO_TIMER,
        .duty_resolution = SERVO_RES,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    const ledc_channel_config_t chan_cfg = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel    = SERVO_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .duty       = duty_from_pulse_us(pulse_us_from_angle_x10(angle_x10)),
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan_cfg));

    s_pulse_us = pulse_us_from_angle_x10(angle_x10);
    s_duty     = chan_cfg.duty;
    s_angle_x10 = angle_x10;
}

static void servo_apply(int angle_x10)
{
    s_pulse_us = pulse_us_from_angle_x10(angle_x10);
    s_duty     = duty_from_pulse_us(s_pulse_us);

    esp_err_t err = ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, s_duty);
    if (err == ESP_OK) {
        err = ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "servo update failed: %s", esp_err_to_name(err));
    }
}

static void log_angle(const char *why)
{
    ESP_LOGI(TAG, "angle=%d.%d deg from leftmost (raw %d, pulse %d us, duty %" PRIu32 ") %s",
             s_angle_x10 / 10, s_angle_x10 % 10, s_pot.mean, s_pulse_us, s_duty,
             why ? why : "");
}

static void tick_cb(void *arg)
{
    static uint32_t s_tick;

    s_tick++;

    int raw  = pot_read();
    int mean = sma_push(&s_pot, raw);
    int next = angle_x10_from_raw(mean);

    if (next - s_angle_x10 > ANGLE_DEADBAND_X10 ||
        s_angle_x10 - next > ANGLE_DEADBAND_X10) {
        s_angle_x10 = next;
        servo_apply(s_angle_x10);
        log_angle("moved");
    }

#if HEARTBEAT_TICKS
    if (s_tick % HEARTBEAT_TICKS == 0) {
        log_angle("alive");
    }
#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 pot -> SG90 1:1: pot=GPIO%d (ADC1 CH%d, 12 dB, %d bit), "
             "servo=GPIO%d (%d Hz, %d-bit duty, %d..%d us = 0..180 deg)",
             POT_GPIO, POT_ADC_CHANNEL, ADC_BITS, SERVO_GPIO,
             SERVO_FREQ_HZ, SERVO_RES_BITS, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
    ESP_LOGI(TAG, "tick=%d ms, averaging=%d samples (%d ms), deadband=0.%d deg",
             TICK_MS, SMA_SIZE, TICK_MS * SMA_SIZE, ANGLE_DEADBAND_X10);

    pot_init();

    /* Initial servo position = where the pot is right now, so the servo is
     * powered up already at the knob, then the tick takes over. */
    int mean = pot_prime();
    servo_init(angle_x10_from_raw(mean));
    log_angle("initial");

    const esp_timer_create_args_t timer_args = {
        .callback = tick_cb,
        .name     = "servo_tick",
    };
    static esp_timer_handle_t s_timer;   // lives for the whole run
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_timer, (uint64_t)TICK_MS * 1000));

    ESP_LOGI(TAG, "running: turn the pot, the servo follows 1:1");
    /* app_main returns: nothing to poll here, the timer task drives everything. */
}
