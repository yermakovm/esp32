#pragma once

// Superloop Configuration
#define SUPERLOOP_DELAY_MS      100     // Main loop delay in milliseconds

// Signal Processing Configuration
#define SMA_SIZE                8       // Moving average buffer size

// LDR / ADC Configuration
#define LDR_ADC_CHANNEL         ADC_CHANNEL_8   // GPIO 9
#define LDR_ADC_UNIT            ADC_UNIT_1
#define LDR_ADC_ATTEN           ADC_ATTEN_DB_12
#define LDR_ADC_BITWIDTH        ADC_BITWIDTH_12

// Calibration: voltage range that maps to servo motion (in millivolts)
#define LDR_CAL_MIN_MV          50      // Minimum light level (dark)
#define LDR_CAL_MAX_MV          3000    // Maximum light level (bright)

// Servo Configuration
#define SERVO_PIN               14
#define SERVO_FREQ_HZ           50      // PWM frequency (standard servo)
#define SERVO_RESOLUTION        LEDC_TIMER_12_BIT
#define SERVO_UNIT              LEDC_TIMER_0
#define SERVO_CHANNEL           LEDC_CHANNEL_0

// Servo angle range (degrees)
#define SERVO_MIN_ANGLE_DEG     10
#define SERVO_MAX_ANGLE_DEG     170

// Servo pulse width range (microseconds)
#define SERVO_MIN_PULSE_US      500     // Corresponds to MIN_ANGLE_DEG
#define SERVO_MAX_PULSE_US      2500    // Corresponds to MAX_ANGLE_DEG

// Derived Constants (computed from above)
#define SERVO_PERIOD_US         (1000000 / SERVO_FREQ_HZ)
#define SERVO_MAX_DUTY          ((1 << SERVO_RESOLUTION) - 1)
