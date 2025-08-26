
#pragma once
// ==== Adjust for your carrier board ====

// Motor driver (differential drive)
#define PIN_ML_PWM 14
#define PIN_ML_IN1 12
#define PIN_ML_IN2 13

#define PIN_MR_PWM 2     // NOTE: GPIO2 is also ESP32-CAM onboard flash LED; remap if needed
#define PIN_MR_IN1 15
#define PIN_MR_IN2 16     // If GPIO16 is not available, choose another PWM-capable IO

// Status LED (optional)
#define PIN_STATUS_LED 4
