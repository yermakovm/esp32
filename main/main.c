#include <stdint.h>
#include <stddef.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "ldr.h"
#include "servo.h"
#include "config.h"

static const char *TAG = "Mini Project";

// Simple moving average over the last SMA_SIZE samples.
typedef struct {
    int16_t buf[SMA_SIZE];
    size_t  idx;
} sma_t;

static int sma_update(sma_t *s, int sample)
{
    int sum = 0;
    for (size_t i = 0; i < SMA_SIZE; i++) {
        sum += s->buf[i];
    }
    s->buf[s->idx] = (int16_t)sample;
    s->idx = (s->idx + 1) % SMA_SIZE;
    return sum / SMA_SIZE;
}

// Map a voltage range to the servo's full angle range.
static int mv_to_angle(int mv)
{
    int clamped = mv;
    if (clamped < LDR_CAL_MIN_MV) clamped = LDR_CAL_MIN_MV;
    if (clamped > LDR_CAL_MAX_MV) clamped = LDR_CAL_MAX_MV;
    return SERVO_MIN_ANGLE_DEG + (clamped - LDR_CAL_MIN_MV) * (SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG) / (LDR_CAL_MAX_MV - LDR_CAL_MIN_MV);
}

// The superloop phases. Add new cases here as the project grows
// (e.g. a calibration phase, an auto-range phase, etc.).
typedef enum {
    PHASE_RUNNING,
} phase_t;

// Static state for the superloop.
static sma_t s_sma = {0};

static void superloop_tick(phase_t *p)
{

    switch (*p) {
    case PHASE_RUNNING: {
        int mv;
        if (!ldr_read_mv(&mv)) {
            // Transient ADC error: skip this tick, keep running.
            return;
        }
        int smoothed = sma_update(&s_sma, mv);
        servo_set_angle(mv_to_angle(smoothed));
        break;
    }
    }
}

void app_main(void)
{
    ldr_init();
    servo_init();

    phase_t phase = PHASE_RUNNING;

    while (1) {
        superloop_tick(&phase);
        vTaskDelay(pdMS_TO_TICKS(SUPERLOOP_DELAY_MS));
    }
}
