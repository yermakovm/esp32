#pragma once

#include <esp_err.h>
#include <stdbool.h>

// Read the LDR voltage in millivolts (calibrated raw -> mV).
// Returns true on success, false on ADC/calibration read error.
bool ldr_read_mv(int *out_mv);

esp_err_t ldr_init(void);