/**
 * ESP32-S3 <-> STM32F411 UART link (lesson)
 *
 * Bidirectional "toggle the LED" demo over UART:
 *   - Button press on ESP32  -> sends TOGGLE frame -> STM32 flips its onboard LED
 *   - Valid frame from STM32 -> ESP32 flips its ONBOARD RGB LED
 *
 * Physical wiring (both boards powered from USB, cross the TX/RX!):
 *   ESP32-S3 GPIO17 (UART1_TX)  --->  STM32 PA3 (USART2_RX)
 *   ESP32-S3 GPIO18 (UART1_RX)  <---  STM32 PA2 (USART2_TX)
 *   ESP32-S3 GND                -----  STM32 GND
 *   ESP32-S3 user button GPIO10 -> tact switch -> GND   (already wired on DevKitC-1)
 *
 * Onboard RGB LED:
 *   - ESP32-S3-DevKitC-1 (v1.0): WS2812 on GPIO48
 *   - ESP32-S3-DevKitC-1 (v1.1): solder jumper; otherwise GPIO38
 *   - Some boards have a simple LED there (just toggle the pin)
 *   The code below drives WS2812 protocol via GPIO bit-banging using
 *   cycle-accurate rom delays.
 *
 * Frame protocol (identical in both directions):
 *   byte0 = header 0xA5
 *   byte1 = command (CMD_TOGGLE_LEDS = 0x01)
 *   byte2 = header XOR command  (cheap checksum; wrong -> frame discarded)
 *   Baud: 115200 8N1
 *
 * NOTE on BOOT/RST buttons: not used here. RST/EN is a hardware reset pin.
 * ESP32-S3 GPIO0 ("BOOT") is a strapping pin: pressing it with RST drops into
 * download mode. We use the existing wired button on GPIO10 instead.
 */

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"
#include "esp_cpu.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"

static const char *TAG = "uart_link";

/* ----- Pin map ----- */
#define BTN_PIN            GPIO_NUM_10
#define ONBOARD_RGB_PIN    GPIO_NUM_48   /* WS2812 data on DevKitC-1, v1.0 */
#define UART_PORT          UART_NUM_1
#define UART_TX_PIN        GPIO_NUM_17   /* -> STM32 PA3 (USART2_RX)  */
#define UART_RX_PIN        GPIO_NUM_18   /* <- STM32 PA2 (USART2_TX) — matches wiring doc above! */
#define UART_BAUD          115200

/* ----- Protocol ----- */
#define FRAME_HEADER       0xA5U
#define CMD_TOGGLE_LEDS    0x01U
#define FRAME_LEN          3U

/* ----- WS2812 / RGB LED timing (bit-banged via cycle counter) ----- */
/* esp_rom_delay_us() has 1 µs resolution but WS2812 needs ~350/900 ns
 * edges — and the old code set bit0/bit1 timings identical, so every bit
 * looked the same to the LED. Instead we busy-wait on esp_cpu_get_cycle_count()
 * against absolute deadlines (overhead of the loop itself is absorbed).
 * The counter resolution is >= 40 MHz (>= 25 ns), calibrated at init.
 * WS2812-B spec: T1H≈900ns T1L≈350ns, T0H≈380ns T0L~>800ns, bit≈1.25µs. */
#define WS2812_RESET_US       60U  /* >50µs reset */

static uint32_t s_ticks_per_us;
static uint32_t s_bit1_high_t, s_bit0_high_t, s_bit_total_t;

#define RGB_OUT_REG_SEL  ((ONBOARD_RGB_PIN) < 32 ? GPIO_OUT_REG : GPIO_OUT1_REG)
#define RGB_OUT_BIT       (1U << ((ONBOARD_RGB_PIN) & 31U))

static inline void rgb_pin_level(int lvl)
{
    /* Direct register write: gpio_set_level() is too slow (~µs) for bit-banging */
    if (lvl) { SET_PERI_REG_MASK(RGB_OUT_REG_SEL, RGB_OUT_BIT); }
    else     { CLEAR_PERI_REG_MASK(RGB_OUT_REG_SEL, RGB_OUT_BIT); }
}

static void ws2812_timing_init(void)
{
    uint32_t c0 = esp_cpu_get_cycle_count();
    esp_rom_delay_us(1000);
    s_ticks_per_us = (esp_cpu_get_cycle_count() - c0) / 1000;
    if (s_ticks_per_us < 1) s_ticks_per_us = 40; /* fallback: 40 MHz XTAL */

    /* Targets per WS2812-B: T1H=0.8µs, T0H=0.4µs, bit period=1.3µs */
    s_bit1_high_t = ((8u * s_ticks_per_us) + 5u) / 10u;
    s_bit0_high_t = ((4u * s_ticks_per_us) + 5u) / 10u;
    if (s_bit0_high_t < 1) s_bit0_high_t = 1;
    s_bit_total_t = ((13u * s_ticks_per_us) + 5u) / 10u;
}

static inline void ws2812_send_bit(uint32_t high_t)
{
    uint32_t start = esp_cpu_get_cycle_count();
    rgb_pin_level(1);
    while ((uint32_t)(esp_cpu_get_cycle_count() - start) < high_t) { }
    rgb_pin_level(0);
    while ((uint32_t)(esp_cpu_get_cycle_count() - start) < s_bit_total_t) { }
}

/* Send 24-bit GRB color to WS2812 (GRB order, MSB first per channel). */
static portMUX_TYPE s_led_mux = portMUX_INITIALIZER_UNLOCKED;

static void ws2812_send_color(uint8_t g, uint8_t r, uint8_t b)
{
    /* Spinlock + disabled interrupts: makes the whole ~30 µs frame atomic on
     * both cores, so rx_task and status_task can never interleave bits and no
     * ISR can stretch a bit past the WS2812 reset timeout mid-frame. */
    portENTER_CRITICAL(&s_led_mux);
    for (int i = 7; i >= 0; i--) ws2812_send_bit(((g >> i) & 1) ? s_bit1_high_t : s_bit0_high_t);
    for (int i = 7; i >= 0; i--) ws2812_send_bit(((r >> i) & 1) ? s_bit1_high_t : s_bit0_high_t);
    for (int i = 7; i >= 0; i--) ws2812_send_bit(((b >> i) & 1) ? s_bit1_high_t : s_bit0_high_t);
    rgb_pin_level(0);
    esp_rom_delay_us(WS2812_RESET_US); /* latch / reset */
    portEXIT_CRITICAL(&s_led_mux);
}

/* LED states */
static bool s_led_on = false;     /* "green" state (logical toggle from STM32) */

static void ws2812_apply_state(void)
{
    if (s_led_on) ws2812_send_color(50, 0, 0);        /* green */
    else         ws2812_send_color(0, 0, 0);
}

/* Frame parser state */
static uint8_t  s_rx_buf[FRAME_LEN];
static uint8_t  s_rx_pos = 0;

/* Activity flag for status LED flash */
static volatile bool s_activity_pulse = false;

/* Button debounce */
#define BUTTON_DEBOUNCE_MS 300
static volatile uint32_t s_last_button_time = 0;

/* --------------------------------------------------------- */

static void uart_link_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 256, 0, 0, NULL, 0));
}

static void send_toggle_frame(void)
{
    uint8_t frame[FRAME_LEN] = {
        FRAME_HEADER,
        CMD_TOGGLE_LEDS,
        FRAME_HEADER ^ CMD_TOGGLE_LEDS,
    };
    uart_write_bytes(UART_PORT, (const char *)frame, sizeof(frame));
    s_activity_pulse = true;
    ESP_LOGI(TAG, "TX: toggle request -> STM32");
}

/* Parse single byte, returns true if a valid cmd was parsed */
static bool parse_byte(uint8_t b)
{
    if (s_rx_pos == 0) {
        if (b == FRAME_HEADER) {
            s_rx_buf[s_rx_pos++] = b;
        }
        return false;
    }

    s_rx_buf[s_rx_pos++] = b;

    if (s_rx_pos < FRAME_LEN) {
        return false;
    }

    uint8_t crc_calc = s_rx_buf[0] ^ s_rx_buf[1];
    s_rx_pos = 0;

    if (s_rx_buf[2] != crc_calc) {
        ESP_LOGW(TAG, "Bad checksum: recv 0x%02X, expect 0x%02X", s_rx_buf[2], crc_calc);
        return false;
    }
    s_activity_pulse = true;
    return true;
}

static void handle_cmd(uint8_t cmd)
{
    if (cmd == CMD_TOGGLE_LEDS) {
        s_led_on = !s_led_on;
        ws2812_apply_state();
        ESP_LOGI(TAG, "RX: toggle from STM32 -> LED %s", s_led_on ? "ON" : "OFF");
    } else {
        ESP_LOGW(TAG, "RX: unknown cmd 0x%02X", cmd);
    }
}

/* ---- Tasks ---- */

static void rx_task(void *arg)
{
    (void)arg;
    uint8_t byte;

    while (1) {
        int len = uart_read_bytes(UART_PORT, &byte, 1, pdMS_TO_TICKS(50));
        if (len > 0) {
            if (parse_byte(byte)) {
                handle_cmd(s_rx_buf[1]);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

static void status_task(void *arg)
{
    (void)arg;
    while (1) {
        if (s_activity_pulse) {
            /* Brief RED flash: note arg order is (g, r, b)! */
            ws2812_send_color(0, 50, 0);
            vTaskDelay(pdMS_TO_TICKS(15));
            ws2812_apply_state();
            s_activity_pulse = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ---- Button ISR -- */

static TaskHandle_t s_btn_task = NULL;   /* handle for the ISR to notify */

static void IRAM_ATTR button_isr(void *arg)
{
    (void)arg;
    if (gpio_get_level(BTN_PIN) != 0) return;   /* not pressed */

    uint32_t now = xTaskGetTickCountFromISR() * (1000 / configTICK_RATE_HZ);
    if (now - s_last_button_time > BUTTON_DEBOUNCE_MS) {
        s_last_button_time = now;
        if (s_btn_task == NULL) return;  /* task not created; nothing to wake */
        BaseType_t hp = pdFALSE;
        vTaskNotifyGiveFromISR(s_btn_task, &hp);
        if (hp) portYIELD_FROM_ISR();
    }
}

static void button_task(void *arg)
{
    (void)arg;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        send_toggle_frame();
    }
}

/* ---- GPIO init ---- */

static void gpio_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BTN_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io);

    io.pin_bit_mask = (1ULL << ONBOARD_RGB_PIN);
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);

    /* Reset: ensure line low (WS2812 idle is low) */
    gpio_set_level(ONBOARD_RGB_PIN, 0);
}

/* ---- Main ---- */

void app_main(void)
{
    ESP_LOGI(TAG, "UART link starting (ESP32-S3 <-> STM32F411)");
    ESP_LOGI(TAG, "GPIO: BTN=%d RGB=%d UART1 TX=%d RX=%d @%d",
             BTN_PIN, ONBOARD_RGB_PIN, UART_TX_PIN, UART_RX_PIN, UART_BAUD);

    s_led_on = false;

    gpio_init();
    ws2812_timing_init();   /* must run before any ws2812_send_color() */
    uart_link_init();

    /* Initial LED state = off (green LED dark) */
    ws2812_apply_state();

    if (xTaskCreate(rx_task, "rx_task", 4096, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create rx_task");
    }
    if (xTaskCreate(status_task, "status_task", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create status_task");
    }
    if (xTaskCreate(button_task, "btn_task", 4096, NULL, 10, &s_btn_task) != pdPASS ||
        s_btn_task == NULL) {
        ESP_LOGE(TAG, "failed to create btn_task");
        s_btn_task = NULL;
    }

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_PIN, button_isr, NULL);

    ESP_LOGI(TAG, "free internal heap: %u bytes", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    ESP_LOGI(TAG, "Ready: press button to toggle STM32 LED; green LED shows toggle state");
}