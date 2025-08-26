
#pragma once
#include <Arduino.h>

struct PID {
  float Kp, Ki, Kd;
  float iacc = 0;
  float last_err = 0;
  float i_limit = 200;
  void setGains(float kp, float ki, float kd, float ilimit) { Kp=kp; Ki=ki; Kd=kd; i_limit=ilimit; }
  float step(float err, float dt) {
    iacc += err * dt;
    if (iacc > i_limit) iacc = i_limit;
    if (iacc < -i_limit) iacc = -i_limit;
    float d = (err - last_err) / dt;
    last_err = err;
    return Kp*err + Ki*iacc + Kd*d;
  }
};
