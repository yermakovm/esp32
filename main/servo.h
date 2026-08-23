#pragma once

#include <esp_err.h>

esp_err_t servo_init(void);

// Set servo position by angle in degrees (clamped to the configured range).
esp_err_t servo_set_angle(int angle_deg);

// Set servo position by PWM pulse width in microseconds.
esp_err_t servo_set_us(int pulse_us);