#include <stdbool.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "button";

#define BTN_PIN GPIO_NUM_5

/*
 * Select the debounce method to use:
 *   1 - raw interrupt, count in ISR (original main.cpp)
 *   2 - interrupt + time-based debounce (200 ms, original main.cpp.2)
 *   3 - interrupt sets a flag, count in the main loop (original main.cpp.3)
 *   4 - no interrupts, polled button state machine (original main.cpp.4)
 */
#ifndef DEBOUNCE_METHOD
#define DEBOUNCE_METHOD 4
#endif

static int btnPressCounter = 0;

#if DEBOUNCE_METHOD == 1

static void IRAM_ATTR handleButtonPress(void *arg)
{
  btnPressCounter++;
  ESP_LOGI(TAG, "Press %d", btnPressCounter);
}

static void setup(void)
{
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << BTN_PIN),
    .mode = GPIO_MODE_INPUT,
    .intr_type = GPIO_INTR_NEGEDGE,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
  };
  gpio_config(&io_conf);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(BTN_PIN, handleButtonPress, NULL);
}

#elif DEBOUNCE_METHOD == 2

static int lastBtnPressTime = 0;

static void IRAM_ATTR handleButtonPress(void *arg)
{
  if (xTaskGetTickCountFromISR() - lastBtnPressTime < pdMS_TO_TICKS(200)) {
    return;
  }
  lastBtnPressTime = xTaskGetTickCountFromISR();
  btnPressCounter++;
  ESP_LOGI(TAG, "Press %d", btnPressCounter);
}

static void setup(void)
{
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << BTN_PIN),
    .mode = GPIO_MODE_INPUT,
    .intr_type = GPIO_INTR_NEGEDGE,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
  };
  gpio_config(&io_conf);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(BTN_PIN, handleButtonPress, NULL);
}

#elif DEBOUNCE_METHOD == 3

static bool btnPressed = false;

static void IRAM_ATTR handleButtonPress(void *arg)
{
  btnPressed = true;
}

static void setup(void)
{
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << BTN_PIN),
    .mode = GPIO_MODE_INPUT,
    .intr_type = GPIO_INTR_NEGEDGE,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
  };
  gpio_config(&io_conf);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(BTN_PIN, handleButtonPress, NULL);
}

#elif DEBOUNCE_METHOD == 4

#define DEBOUNCE_DELAY_MS 50
enum ButtonState {
  BUTTON_RELEASED = 0,
  BUTTON_PRESSED = 1,
  BUTTON_HELD = 2,
  BUTTON_IDLE = 3,
};

static enum ButtonState buttonState = BUTTON_RELEASED;
static int lastPressedTime = 0;

static void setup(void)
{
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << BTN_PIN),
    .mode = GPIO_MODE_INPUT,
    .intr_type = GPIO_INTR_DISABLE,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
  };
  gpio_config(&io_conf);
}

#else
#error "DEBOUNCE_METHOD must be 1, 2, 3 or 4"
#endif

void app_main(void)
{
  setup();
  ESP_LOGI(TAG, "Started, debounce method: %d, pin: %" PRId32,
           DEBOUNCE_METHOD, (int32_t)BTN_PIN);

#if DEBOUNCE_METHOD == 3
  for (;;) {
    if (!btnPressed) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    btnPressCounter++;
    btnPressed = false;
    ESP_LOGI(TAG, "Press %d", btnPressCounter);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
#elif DEBOUNCE_METHOD == 4
  for (;;) {
    int currentTime = xTaskGetTickCount();
    bool btnPinHigh = gpio_get_level(BTN_PIN) == 1;

    switch (buttonState) {
    case BUTTON_HELD:
      // short press
      if (!btnPinHigh) {
        btnPressCounter++;
        ESP_LOGI(TAG, "Button pressed: %d", btnPressCounter);
        buttonState = BUTTON_RELEASED;
      }
      break;

    case BUTTON_PRESSED:
      if (!btnPinHigh) {
        buttonState = BUTTON_RELEASED;
        break;
      }
      if (currentTime - lastPressedTime >= pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)) {
        buttonState = BUTTON_HELD;
      }
      break;

    case BUTTON_RELEASED:
      if (currentTime - lastPressedTime >= pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)) {
        buttonState = BUTTON_IDLE;
      }
      break;

    case BUTTON_IDLE:
      if (btnPinHigh) {
        buttonState = BUTTON_PRESSED;
        lastPressedTime = currentTime;
      }
      break;

    default:
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
#else
  for (;;) {
    vTaskDelay(portMAX_DELAY);
  }
#endif
}
