
#include "parking.h"

void ParkingState::begin(bool on_right) {
  lot_on_right = on_right;
  phase = PARK_PHASE1;
  phase_start = millis();
}

bool ParkingState::step(int base_speed, int* out_bias) {
  // Simple 3-phase parallel parking: reverse with steer, then counter-steer, then straighten
  if (phase == PARK_DONE || phase == PARK_IDLE) return false;
  uint32_t t = millis() - phase_start;
  int dir = lot_on_right ? 1 : -1; // steer direction
  if (phase == PARK_PHASE1) {
    *out_bias = 180 * dir; // strong steer
    if (t > 1200) { phase = PARK_PHASE2; phase_start = millis(); }
    return true;
  } else if (phase == PARK_PHASE2) {
    *out_bias = -160 * dir;
    if (t > 900) { phase = PARK_PHASE3; phase_start = millis(); }
    return true;
  } else if (phase == PARK_PHASE3) {
    *out_bias = 0;
    if (t > 700) { phase = PARK_DONE; }
    return true;
  }
  return false;
}
