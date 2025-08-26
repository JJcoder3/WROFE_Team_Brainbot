
#include "motion.h"

void motion_init() {
  pinMode(PIN_ML_IN1, OUTPUT);
  pinMode(PIN_ML_IN2, OUTPUT);
  pinMode(PIN_MR_IN1, OUTPUT);
  pinMode(PIN_MR_IN2, OUTPUT);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(PIN_ML_PWM, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(PIN_MR_PWM, 1);
}

void set_motor(int pwm_pin, int in1, int in2, int value) {
  bool fwd = value >= 0;
  int mag = abs(value);
  if (mag > 255) mag = 255;
  digitalWrite(in1, fwd ? HIGH : LOW);
  digitalWrite(in2, fwd ? LOW : HIGH);
  if (pwm_pin == PIN_ML_PWM) ledcWrite(0, mag);
  else if (pwm_pin == PIN_MR_PWM) ledcWrite(1, mag);
}

void drive_differential(int base, int bias) {
  int left = base - bias;
  int right = base + bias;
  set_motor(PIN_ML_PWM, PIN_ML_IN1, PIN_ML_IN2, left);
  set_motor(PIN_MR_PWM, PIN_MR_IN1, PIN_MR_IN2, right);
}

void stop_all() {
  set_motor(PIN_ML_PWM, PIN_ML_IN1, PIN_ML_IN2, 0);
  set_motor(PIN_MR_PWM, PIN_MR_IN1, PIN_MR_IN2, 0);
}
