/**
 * Lesson 4.6 — DMA: ADC1 continuous mode (GDMA) -> processing -> UART (ESP32-S3)
 * Source practice: practice_dma_adc_uart_esp-idf.md (Parts 2, 3, 5, 6)
 *
 * Chain: ADC1 samples continuously into a ring of DMA frames (no CPU polling)
 *        -> ISR wakes adc_read_task -> statistics (min/max/vpp/avg)
 *        -> uart_write_bytes() on UART0 (interrupt + ring buffer, non-blocking).
 *
 * Wiring:
 *   Potentiometer wiper -> GPIO4 (ADC1_CH3), ends -> 3V3 and GND  (never above 3.3 V!)
 *   Console/UART: Micro-USB port -> on-board USB-UART bridge -> UART0 (GPIO43 TX / GPIO44 RX
 *   on DevKitC-1, these are the chip defaults, so no uart_set_pin() call is needed).
 *   GPIO17/18 (U1TXD/U1RXD) stay free for the STM32 link of lesson 4.1.
 *
 * Tasks from the practice:
 *   2.1  LOG_FRAME_EDGES=1 logs first/last raw value of each DMA frame (temporary debug)
 *   5.1  read buffer is allocated with MALLOC_CAP_DMA (see dma_buffer note below)
 *   6.1  per-frame line: raw min/max/avg/vpp + volts, several times per second
 *   6.2  rolling window (AGG_WINDOW_MS) accumulates min/max/avg/vpp over several frames
 *        -> the optional "console oscilloscope"
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_continuous.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = "adc_dma_uart";

/* ----- ADC1 continuous (P2M DMA), practice Part 2 ----- */
#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_3      /* GPIO4 on ESP32-S3 = ADC1_CH3 */
#define ADC_ATTEN       ADC_ATTEN_DB_12    /* full range, ~0-3.3 V */
#define ADC_BIT_WIDTH   ADC_BITWIDTH_12
#define SAMPLE_FREQ_HZ  20000              /* plenty for a potentiometer */

#define FRAME_SIZE      256   /* bytes in ONE ready block  == conv_frame_size == I2S dma_buf_len */
#define POOL_FRAMES     4     /* blocks kept in the ring at once == I2S dma_buf_count            */
                              /* FRAME_SIZE is a multiple of SOC_ADC_DIGI_DATA_BYTES_PER_CONV (4) */

/* FRAME_SIZE/4 = 64 samples per frame -> 20000/64 ~= 312 frames/s.
 * Printing every frame would push ~312*60 B/s > 115200 baud capacity (~11.5 KB/s)
 * and would eventually block in uart_write_bytes(). So the per-frame line is
 * throttled: ~10 lines/s, which is what task 6.1 asks for. */
#define FRAME_PRINT_INTERVAL_MS 100
#define AGG_WINDOW_MS           1000   /* task 6.2: oscilloscope-style window */
#define LOG_FRAME_EDGES         1      /* task 2.1: first/last raw of each printed frame, 0 to silence */

/* ----- UART (practice Part 3) ----- */
#define UART_PORT       UART_NUM_0       /* on-board USB-UART bridge = console */
#define UART_BAUD       115200
#define UART_RX_BUF     0                /* TX only here */
#define UART_TX_BUF     1024             /* MUST be > 0: non-blocking uart_write_bytes()
                                          * (with 0 the call blocks on the 128-byte HW FIFO) */

/* ADC full-scale for the raw -> volt conversion (12 bit, ATTEN_DB_12) */
#define ADC_FULL_SCALE_RAW  4095.0f
#define ADC_FULL_SCALE_V    3.3f

static adc_continuous_handle_t s_adc_handle;
static TaskHandle_t            s_adc_task_handle;
static uint8_t                *s_dma_buf;          /* MALLOC_CAP_DMA, Part 5 */
static volatile uint32_t       s_pool_overflow_cnt;

/* ------------------------------------------------------------------ */
/* UART init (Part 3)                                                  */
/* ------------------------------------------------------------------ */
static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    /* UART0 default pins are used (GPIO43/44 on ESP32-S3, wired to the USB-UART
     * bridge on DevKitC-1). For another port, e.g. UART1 on GPIO17/18:
     *   ESP_ERROR_CHECK(uart_set_pin(UART_PORT, 17, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
     */
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF, UART_TX_BUF, 0, NULL, 0));
}

/* ------------------------------------------------------------------ */
/* ADC continuous + DMA init (Part 2)                                  */
/* ------------------------------------------------------------------ */

/* Runs in ISR context (GDMA done interrupt). Analogue of HAL_ADC_ConvCpltCallback on STM32:
 * only wakes the consumer task, never processes data here. */
static bool IRAM_ATTR adc_conv_done_cb(adc_continuous_handle_t handle,
                                       const adc_continuous_evt_data_t *edata, void *user_data)
{
    (void)handle; (void)edata; (void)user_data;
    BaseType_t must_yield = pdFALSE;
    vTaskNotifyGiveFromISR(s_adc_task_handle, &must_yield);
    return must_yield == pdTRUE;
}

/* Fires when the whole ring (POOL_FRAMES frames) is full of unread data:
 * this is the ESP-IDF version of "CPU is too slow, Circular mode overwrote data". */
static bool IRAM_ATTR adc_pool_ovf_cb(adc_continuous_handle_t handle,
                                      const adc_continuous_evt_data_t *edata, void *user_data)
{
    (void)handle; (void)edata; (void)user_data;
    s_pool_overflow_cnt++;
    return false;
}

static esp_err_t adc_dma_init(void)
{
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = FRAME_SIZE * POOL_FRAMES,  /* ring capacity: dma_buf_count * dma_buf_len */
        .conv_frame_size    = FRAME_SIZE,                /* size of ONE ready block: on_conv_done fires here */
        .flags = { .flush_pool = 0 },                    /* 0 = keep oldest frames, drop newest when full */
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &s_adc_handle));

    adc_digi_pattern_config_t pattern[1] = {
        {
            .atten     = ADC_ATTEN,
            .channel   = ADC_CHANNEL,
            .unit      = ADC_UNIT,
            .bit_width = ADC_BIT_WIDTH,
        },
    };
    adc_continuous_config_t dig_cfg = {
        .pattern_num   = 1,                 /* >1: several channels interleaved in one DMA stream */
        .adc_pattern   = pattern,
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode     = ADC_CONV_SINGLE_UNIT_1,
        .format        = ADC_DIGI_OUTPUT_FORMAT_TYPE2,   /* TYPE2 = ESP32-S3 output format */
    };
    esp_err_t err = adc_continuous_config(s_adc_handle, &dig_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_config failed: %s (sample_freq_hz out of range?)",
                 esp_err_to_name(err));
        return err;
    }

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_conv_done_cb,
        .on_pool_ovf  = adc_pool_ovf_cb,
    };
    /* v5.x signature takes user_data as the 3rd argument (passed to every callback) */
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(s_adc_handle, &cbs, NULL));
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Processing + UART output (Part 6)                                   */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t vmin;
    uint32_t vmax;
    uint32_t vavg;
    uint32_t vpp;
    uint32_t samples;
} stats_t;

/* Rolling window (task 6.2): accumulates several consecutive frames. */
typedef struct {
    uint32_t vmin;
    uint32_t vmax;
    uint64_t sum;
    uint32_t samples;
    uint32_t frames;
    uint32_t start_tick;
} window_t;

static void stats_from_frame(adc_digi_output_data_t *data, int count, stats_t *out)
{
    uint32_t sum = 0;
    uint32_t vmin = 0xFFFFFFFFu, vmax = 0;

    for (int i = 0; i < count; i++) {
        /* TYPE2 carries the channel id together with the raw value — needed when
         * pattern_num > 1, because the driver interleaves channels in one stream. */
        uint32_t raw = data[i].type2.data;
        sum += raw;
        if (raw < vmin) vmin = raw;
        if (raw > vmax) vmax = raw;
    }

    out->vmin    = vmin;
    out->vmax    = vmax;
    out->vavg    = sum / (uint32_t)count;
    out->vpp     = vmax - vmin;
    out->samples = (uint32_t)count;
}

static void window_reset(window_t *w, uint32_t now_tick)
{
    w->vmin = 0xFFFFFFFFu;
    w->vmax = 0;
    w->sum = 0;
    w->samples = 0;
    w->frames = 0;
    w->start_tick = now_tick;
}

static void window_accumulate(window_t *w, const stats_t *f)
{
    if (f->vmin < w->vmin) w->vmin = f->vmin;
    if (f->vmax > w->vmax) w->vmax = f->vmax;
    w->sum += (uint64_t)f->vavg * f->samples;   /* per-frame sum is 32 bit, window sum is 64 bit */
    w->samples += f->samples;
    w->frames++;
}

static void window_report(window_t *w, uint32_t now_tick)
{
    if (w->samples == 0) return;

    uint32_t avg = (uint32_t)(w->sum / w->samples);
    uint32_t vpp = w->vmax - w->vmin;
    uint32_t dur_ms = (uint32_t)(now_tick - w->start_tick) * (1000 / configTICK_RATE_HZ);
    float v_avg = (avg / ADC_FULL_SCALE_RAW) * ADC_FULL_SCALE_V;

    char line[128];
    int n = snprintf(line, sizeof(line),
                     "win %3lu ms (%2lu frames, %4lu smp): min=%4lu max=%4lu avg=%4lu vpp=%4lu | avg=%.2fV\r\n",
                     (unsigned long)dur_ms, (unsigned long)w->frames, (unsigned long)w->samples,
                     (unsigned long)w->vmin, (unsigned long)w->vmax, (unsigned long)avg,
                     (unsigned long)vpp, v_avg);
    if (n > 0) {
        uart_write_bytes(UART_PORT, line, (size_t)n);   /* copies into the driver ring, returns at once */
    }
}

/* Split of the practice's process_and_send(): stats_from_frame() does the math,
 * send_frame_line() does the UART output. Split because the window (task 6.2)
 * needs every frame, while the console line is throttled (see FRAME_PRINT_INTERVAL_MS). */
static void send_frame_line(const stats_t *f)
{
    float v_avg = (f->vavg / ADC_FULL_SCALE_RAW) * ADC_FULL_SCALE_V;

    char line[96];
    int n = snprintf(line, sizeof(line),
                     "raw: min=%lu max=%lu avg=%lu vpp=%lu | avg=%.2fV\r\n",
                     (unsigned long)f->vmin, (unsigned long)f->vmax,
                     (unsigned long)f->vavg, (unsigned long)f->vpp, v_avg);
    if (n > 0) {
        uart_write_bytes(UART_PORT, line, (size_t)n);
    }
}

/* ------------------------------------------------------------------ */
/* Consumer task (analogue of the STM32 while(1) + volatile flags)     */
/* ------------------------------------------------------------------ */
static void adc_read_task(void *arg)
{
    (void)arg;
    uint32_t bytes_read = 0;
    const uint32_t print_period = pdMS_TO_TICKS(FRAME_PRINT_INTERVAL_MS);
    const uint32_t window_len   = pdMS_TO_TICKS(AGG_WINDOW_MS);
    window_t win;
    uint32_t last_print_tick = 0;

    ESP_ERROR_CHECK(adc_continuous_start(s_adc_handle));
    window_reset(&win, xTaskGetTickCount());

    while (1) {
        /* Sleep until adc_conv_done_cb wakes us — the CPU is free while the
         * ADC + GDMA move samples in the background. No readiness polling at all. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* timeout 0: never block, take the frame that GDMA already finished. */
        esp_err_t err = adc_continuous_read(s_adc_handle, s_dma_buf, FRAME_SIZE, &bytes_read, 0);
        if (err == ESP_ERR_TIMEOUT) {
            continue;                       /* frame not complete yet, keep waiting */
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_continuous_read: %s (pool overflow %lu)",
                     esp_err_to_name(err), (unsigned long)s_pool_overflow_cnt);
            continue;
        }

        int count = (int)(bytes_read / sizeof(adc_digi_output_data_t));
        if (count <= 0) {
            continue;
        }

        adc_digi_output_data_t *data = (adc_digi_output_data_t *)s_dma_buf;

        stats_t f;
        stats_from_frame(data, count, &f);
        window_accumulate(&win, &f);

        uint32_t now = xTaskGetTickCount();

        if (now - last_print_tick >= print_period) {
            last_print_tick = now;
            send_frame_line(&f);                  /* task 6.1 */
            /* Task 2.1: first/last raw value of this DMA block (throttled, else the
             * console gets ~312 lines/s). Set LOG_FRAME_EDGES to 0 to silence. */
            if (LOG_FRAME_EDGES) {
                ESP_LOGI(TAG, "frame: first=%lu last=%lu (%d smp)",
                         (unsigned long)data[0].type2.data,
                         (unsigned long)data[count - 1].type2.data, count);
            }
        }

        if (now - win.start_tick >= window_len) {
            window_report(&win, now);           /* task 6.2 */
            window_reset(&win, now);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */
void app_main(void)
{
    ESP_LOGI(TAG, "ADC1 continuous (DMA) -> stats -> UART0 @%d", UART_BAUD);
    ESP_LOGI(TAG, "ADC%d_CH%d (GPIO4) atten=%d width=%d fsampling=%d Hz, frame=%d B x %d pool",
             ADC_UNIT + 1, ADC_CHANNEL, ADC_ATTEN, ADC_BIT_WIDTH, SAMPLE_FREQ_HZ,
             FRAME_SIZE, POOL_FRAMES);

    /* Part 5: DMA-capable memory. Without MALLOC_CAP_DMA the buffer may land in
     * PSRAM; the driver would then silently memcpy it into internal SRAM before
     * the transfer — works, but the CPU pays for the copy (hidden slowdown). */
    s_dma_buf = heap_caps_malloc(FRAME_SIZE, MALLOC_CAP_DMA);
    if (s_dma_buf == NULL) {
        ESP_LOGE(TAG, "no DMA-capable memory for %d bytes", FRAME_SIZE);
        return;
    }

    uart_init();

    if (adc_dma_init() != ESP_OK) {
        heap_caps_free(s_dma_buf);
        s_dma_buf = NULL;
        return;
    }

    if (xTaskCreate(adc_read_task, "adc_read_task", 4096, NULL, 5, &s_adc_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "failed to create adc_read_task");
        ESP_ERROR_CHECK(adc_continuous_deinit(s_adc_handle));
        heap_caps_free(s_dma_buf);
        s_dma_buf = NULL;
        return;
    }

    ESP_LOGI(TAG, "free DMA-capable heap: %u bytes",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA));
    /* app_main() returns on purpose: adc_read_task keeps running under the
     * scheduler, there is no "main while(1)" here (unlike bare-metal STM32). */
    ESP_LOGI(TAG, "ready: turn the potentiometer on GPIO4, watch console lines");
}
