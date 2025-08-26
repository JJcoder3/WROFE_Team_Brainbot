
#pragma once
#include <Arduino.h>
#include "pins.h"

void motion_init();
void set_motor(int pwm_pin, int in1, int in2, int value);
void drive_differential(int base, int bias); // base 0..255, bias -255..255
void stop_all();
