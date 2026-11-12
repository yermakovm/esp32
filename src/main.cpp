#include <FreeRTOS.h>
#include <task.h>

#include <driver/gpio.h>
#include <esp_log.h>

// GPIO_NUM_* instead of plain ints: the gpio_* API takes gpio_num_t.
#define YELLOW_LED_PIN GPIO_NUM_2
#define RED_LED_PIN GPIO_NUM_35
#define BLUE_LED_PIN GPIO_NUM_38

static const char *TAG = "blink";

struct Led {
  gpio_num_t pin;
  uint32_t periodMs; // ms between toggles
};

static const Led leds[] = {
    {YELLOW_LED_PIN, 500},
    {RED_LED_PIN, 200},
    {BLUE_LED_PIN, 1000},
};

static void blinkTask(void *arg) {
  const Led *led = static_cast<const Led *>(arg);
  bool on = false;
  while (1) {
    on = !on;
    gpio_set_level(led->pin, on);
    vTaskDelay(pdMS_TO_TICKS(led->periodMs));
  }
}

void setup() {
  for (const Led &led : leds) {
    gpio_set_direction(led.pin, GPIO_MODE_OUTPUT);
    gpio_set_level(led.pin, 0);
  }

  for (const Led &led : leds) {
    char name[16];
    snprintf(name, sizeof(name), "blink%d", led.pin);
    if (xTaskCreatePinnedToCore(blinkTask, name, 4096, (void *)&led, 1, nullptr,
                                0) != pdPASS) {
      ESP_LOGE(TAG, "failed to create task for pin %d", led.pin);
    }
  }
}

void loop() {
  // All work runs in the blink tasks; do not busy-spin here.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
