
#pragma once
#include <Arduino.h>
#include "vision.h"
#include "control.h"
#include "parking.h"

enum Mode { MODE_OPEN=0, MODE_OBSTACLE=1 };

struct Runtime {
  Mode mode = MODE_OBSTACLE;
  VisionParams vp;
  PID pid;
  int base_speed=110;
  SideBias bias=SIDE_NONE;
  int laps=0;
  uint32_t last_lap_ms=0;
  bool parking_triggered=false;
  ParkingState park;
};
