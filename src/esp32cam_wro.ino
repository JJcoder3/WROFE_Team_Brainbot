
#include <Arduino.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include "pins.h"
#include "motion.h"
#include "vision.h"
#include "control.h"
#include "state.h"
#include "parking.h"

Runtime RT;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  motion_init();
  if (!camera_init()) {
    Serial.println("Camera init failed");
    while(true){ delay(1000); }
  }
  vision_load_params(RT.vp);
  RT.pid.setGains(0.9f, 0.0f, 0.12f, 200.0f);
  RT.mode = MODE_OBSTACLE; // change to MODE_OPEN for open challenge
  RT.base_speed = (RT.mode==MODE_OPEN) ? 120 : 100;
  RT.last_lap_ms = millis();
}

void loop() {
  // Acquire frame (RGB565)
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;

  // Build grayscale buffer once (luma) for line & start-line
  int pixels = fb->width * fb->height;
  static std::vector<uint8_t> gray;
  gray.resize(pixels);
  uint16_t* src = (uint16_t*)fb->buf;
  for (int i=0;i<pixels;i++) {
    uint16_t rgb = src[i];
    uint8_t r = ((rgb >> 11) & 0x1F) << 3;
    uint8_t g = ((rgb >> 5) & 0x3F) << 2;
    uint8_t b = (rgb & 0x1F) << 3;
    gray[i] = (uint8_t)((r*30 + g*59 + b*11)/100);
  }

  // Compute line error
  int quality=0;
  int err = compute_line_error(RT.vp, fb->width, fb->height, gray.data(), &quality);

  // Pillar bias (obstacle mode)
  if (RT.mode==MODE_OBSTACLE && !RT.parking_triggered) {
    SideBias b = detect_pillar_bias(RT.vp, fb->width, fb->height, gray.data(), fb);
    if (b != SIDE_NONE) RT.bias = b;
  }

  // Start-line / lap counting
  int strength=0;
  bool cross = detect_startline_cross(RT.vp, fb->width, fb->height, gray.data(), &strength);
  uint32_t now = millis();
  if (cross && (now - RT.last_lap_ms) > 5000) { // debounce
    if (RT.laps==0) {
      // First crossing considered as starting line arm
      RT.last_lap_ms = now;
    } else {
      RT.laps += 1;
      RT.last_lap_ms = now;
      Serial.printf("Lap %d\\n", RT.laps);
    }
  }

  // Parking trigger after 3 laps in obstacle mode
  if (RT.mode==MODE_OBSTACLE && RT.laps >= 3 && !RT.parking_triggered) {
    bool on_right=true;
    if (detect_parking_magenta(RT.vp, fb, &on_right)) {
      RT.parking_triggered = true;
      RT.park.begin(on_right);
      Serial.println("Parking lot detected, starting parking maneuver.");
    }
  }

  // Control
  int bias_cmd = 0;
  if (RT.parking_triggered) {
    // Execute parking phases with slow speed
    int out_bias=0;
    bool active = RT.park.step(80, &out_bias);
    bias_cmd = out_bias;
    RT.base_speed = 80;
    if (!active) {
      stop_all();
      Serial.println("Parking done.");
      while(true){ delay(1000); } // stop forever
    }
  } else {
    float dt = 0.033f; // approx 30 Hz
    float steer = RT.pid.step((float)err, dt);
    // Apply pillar side bias
    int side = 0;
    if (RT.bias == SIDE_RIGHT) side = 20;
    else if (RT.bias == SIDE_LEFT) side = -20;
    bias_cmd = (int)steer + side;
  }

  // Drive
  drive_differential(RT.base_speed, bias_cmd);

  // Telemetry
  static uint32_t lastPrint=0;
  if (now - lastPrint > 200) {
    lastPrint = now;
    Serial.printf("e=%d q=%d bias=%d laps=%d parking=%d\\n", err, quality, bias_cmd, RT.laps, (int)RT.parking_triggered);
  }

  esp_camera_fb_return(fb);
}
