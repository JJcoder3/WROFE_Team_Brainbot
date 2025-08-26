
#pragma once
#include <Arduino.h>
#include "esp_camera.h"

struct VisionParams {
  uint8_t line_thresh=90;
  bool line_invert=false;
  int roi_rows[3] = {80,100,120};
  // Color gates
  int red_r_min=150, red_g_max=80, red_b_max=80;
  int green_g_min=150, green_r_max=110, green_b_max=110;
  int magenta_r_min=140, magenta_b_min=140, magenta_g_max=80;
};

enum SideBias { SIDE_NONE=0, SIDE_LEFT=1, SIDE_RIGHT=2 };

bool camera_init();
void vision_load_params(VisionParams& vp);
bool get_frame_grayscale(uint8_t** buf, int* w, int* h); // returns mono8 via luma from RGB565
int compute_line_error(const VisionParams& vp, int imgw, int imgh, uint8_t* gray, int* quality_out);
SideBias detect_pillar_bias(const VisionParams& vp, int imgw, int imgh, uint8_t* gray, camera_fb_t* fb);
bool detect_startline_cross(const VisionParams& vp, int imgw, int imgh, uint8_t* gray, int* strength);
bool detect_parking_magenta(const VisionParams& vp, camera_fb_t* fb, bool* on_right);
