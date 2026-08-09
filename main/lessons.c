#include "config.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

static esp_timer_handle_t hourly_timer = NULL;
static esp_timer_handle_t motor_off_timer = NULL;
static bool motor_on_pending = false;

static const char *TAG = "motor";

static void motor_off_callback(void *arg) {
  gpio_set_level(MOTOR_PIN, 1);
  ESP_LOGI(TAG, "Motor turned OFF (15 min done)");
}

static void hourly_timer_callback(void *arg) {
  gpio_set_level(MOTOR_PIN, 0);
  ESP_LOGI(TAG, "Motor turned ON (hourly trigger)");
  motor_on_pending = true;
}

void setup(void) {
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << MOTOR_PIN),
    .mode = GPIO_MODE_OUTPUT,
    .intr_type = GPIO_INTR_DISABLE,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
  };
  gpio_config(&io_conf);
  gpio_set_level(MOTOR_PIN, 1);

  const esp_timer_create_args_t hourly_args = {
    .callback = &hourly_timer_callback,
    .arg = NULL,
    .name = "hourly_motor_timer"
  };
  esp_timer_create(&hourly_args, &hourly_timer);

  const esp_timer_create_args_t motor_off_args = {
    .callback = &motor_off_callback,
    .arg = NULL,
    .name = "motor_off_timer"
  };
  esp_timer_create(&motor_off_args, &motor_off_timer);

  int64_t wake_us = (int64_t)WAKE_UP_INTERVAL_S * 1000000LL;
  esp_timer_start_periodic(hourly_timer, wake_us);
  ESP_LOGI(TAG, "Hourly motor timer started (every %d s), motor ON for %d s",
           WAKE_UP_INTERVAL_S, WORK_INTERVAL_S);
}


void app_main(void) {
  setup();
  int tick_count = 0;
  while (1) {
    if (motor_on_pending) {
      motor_on_pending = false;

      int64_t work_us = (int64_t)WORK_INTERVAL_S * 1000000LL;
      esp_timer_start_once(motor_off_timer, work_us);
      ESP_LOGI(TAG, "Scheduled motor OFF after %d s", WORK_INTERVAL_S);
    }
    tick_count++;
    if (tick_count % 3600 == 0) {
      ESP_LOGI(TAG, "Tick: 1 hour elapsed, motor pin=%d", gpio_get_level(MOTOR_PIN));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
