
#pragma once
#include <Arduino.h>

enum ParkingPhase {
  PARK_IDLE=0,
  PARK_PHASE1,
  PARK_PHASE2,
  PARK_PHASE3,
  PARK_DONE
};

struct ParkingState {
  ParkingPhase phase=PARK_IDLE;
  uint32_t phase_start=0;
  bool lot_on_right=true;
  void begin(bool on_right);
  bool step(int base_speed, int* out_bias); // returns true when still parking
};
