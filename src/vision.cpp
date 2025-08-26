
#include "vision.h"
#include <FS.h>
#include "SPIFFS.h"
#include "camera_pins.h"

static bool s_cam_ready=false;

bool camera_init() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QQVGA;
  config.pixel_format = PIXFORMAT_RGB565;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  s_cam_ready = (err == ESP_OK);
  return s_cam_ready;
}

static uint8_t luma_from_rgb565(uint16_t rgb) {
  // Extract RGB565
  uint8_t r = ((rgb >> 11) & 0x1F) << 3;
  uint8_t g = ((rgb >> 5) & 0x3F) << 2;
  uint8_t b = (rgb & 0x1F) << 3;
  // Luma approximation
  return (uint8_t)((r*30 + g*59 + b*11)/100);
}

void vision_load_params(VisionParams& vp) {
  if (!SPIFFS.begin(true)) return;
  File f = SPIFFS.open("/vision_params.json", "r");
  if (!f) return;
  // Minimal JSON parse (naive). Recommend ArduinoJson if available.
  String s = f.readString();
  auto find_val = [&](const char* key)->int {
    int i = s.indexOf(key);
    if (i<0) return INT32_MIN;
    i = s.indexOf(':', i);
    if (i<0) return INT32_MIN;
    int j=i+1;
    while (j < (int)s.length() && (s[j]==' '||s[j]=='\t')) j++;
    int k=j;
    while (k < (int)s.length() && (isdigit(s[k])||s[k]=='-' )) k++;
    return s.substring(j,k).toInt();
  };
  int v;
  if ((v=find_val("binary_threshold"))!=INT32_MIN) vp.line_thresh=v;
  if (s.indexOf('"')>=0) {
    int idx = s.indexOf(""invert"");
    if (idx>=0) {
      int t = s.indexOf("true", idx);
      if (t>=0 && t-idx < 40) vp.line_invert=true;
      int fidx = s.indexOf("false", idx);
      if (fidx>=0 && fidx-idx < 40) vp.line_invert=false;
    }
  }
}

bool get_frame_grayscale(uint8_t** out, int* w, int* h) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return false;
  int pixels = fb->width * fb->height;
  static std::vector<uint8_t> gray;
  gray.resize(pixels);
  uint16_t* src = (uint16_t*)fb->buf;
  for (int i=0;i<pixels;i++) gray[i] = luma_from_rgb565(src[i]);
  *out = gray.data();
  *w = fb->width;
  *h = fb->height;
  esp_camera_fb_return(fb);
  return true;
}

int compute_line_error(const VisionParams& vp, int imgw, int imgh, uint8_t* gray, int* quality_out) {
  // Center-of-mass over last ROI row
  int row = vp.roi_rows[2];
  if (row >= imgh) row = imgh - 1;
  int sum = 0, count = 0;
  for (int x=0;x<imgw;x++) {
    uint8_t v = gray[row*imgw + x];
    bool on = vp.line_invert ? (v > vp.line_thresh) : (v < vp.line_thresh);
    if (on) { sum += x; count++; }
  }
  if (quality_out) *quality_out = count;
  if (count == 0) return 0;
  float cx = (float)sum / (float)count;
  float center = (imgw-1)*0.5f;
  return (int)(cx - center); // negative => line left of center
}

static int count_in_roi(uint8_t* buf, int imgw, int imgh, int y0, int y1, std::function<bool(uint8_t,uint8_t,uint8_t)> gate, camera_fb_t* fb) {
  // Re-scan the original RGB565 to classify colors (more robust than gray)
  uint16_t* src = (uint16_t*)fb->buf;
  int cnt=0;
  for (int y=y0; y<y1 && y<imgh; ++y) {
    for (int x=0; x<imgw; ++x) {
      uint16_t rgb=src[y*imgw+x];
      uint8_t r=((rgb>>11)&0x1F)<<3, g=((rgb>>5)&0x3F)<<2, b=(rgb&0x1F)<<3;
      if (gate(r,g,b)) cnt++;
    }
  }
  return cnt;
}

SideBias detect_pillar_bias(const VisionParams& vp, int imgw, int imgh, uint8_t* gray, camera_fb_t* fb) {
  int y0 = imgh/3, y1 = imgh/2; // horizon band
  auto is_red = [&](uint8_t r,uint8_t g,uint8_t b){ return r>=vp.red_r_min && g<=vp.red_g_max && b<=vp.red_b_max; };
  auto is_green = [&](uint8_t r,uint8_t g,uint8_t b){ return g>=vp.green_g_min && r<=vp.green_r_max && b<=vp.green_b_max; };
  int left_red=0, right_red=0, left_green=0, right_green=0;
  uint16_t* src = (uint16_t*)fb->buf;
  for (int y=y0; y<y1 && y<imgh; ++y) {
    for (int x=0; x<imgw; ++x) {
      uint16_t rgb=src[y*imgw+x];
      uint8_t r=((rgb>>11)&0x1F)<<3, g=((rgb>>5)&0x3F)<<2, b=(rgb&0x1F)<<3;
      if (is_red(r,g,b)) { if (x < imgw/2) left_red++; else right_red++; }
      if (is_green(r,g,b)) { if (x < imgw/2) left_green++; else right_green++; }
    }
  }
  // Decide dominant color and side
  int red_total = left_red + right_red;
  int green_total = left_green + right_green;
  if (red_total < 50 && green_total < 50) return SIDE_NONE;
  if (red_total >= green_total) {
    // Red pillar => keep RIGHT side -> bias to right (positive)
    return SIDE_RIGHT;
  } else {
    // Green pillar => keep LEFT side -> bias to left (negative)
    return SIDE_LEFT;
  }
}

bool detect_startline_cross(const VisionParams& vp, int imgw, int imgh, uint8_t* gray, int* strength) {
  // Look for a sudden brightness delta across a horizontal band near bottom
  int y = imgh - 10;
  int delta_sum=0;
  for (int x=1; x<imgw; ++x) {
    int d = (int)gray[y*imgw+x] - (int)gray[y*imgw + x-1];
    delta_sum += abs(d);
  }
  if (strength) *strength = delta_sum;
  return delta_sum > (vp.line_thresh + 100); // heuristic
}

bool detect_parking_magenta(const VisionParams& vp, camera_fb_t* fb, bool* on_right) {
  uint16_t* src = (uint16_t*)fb->buf;
  int w = fb->width, h = fb->height;
  int y0 = h/2, y1 = h; // lower half
  int left=0, right=0;
  for (int y=y0; y<y1; ++y) {
    for (int x=0; x<w; ++x) {
      uint16_t rgb=src[y*w+x];
      uint8_t r=((rgb>>11)&0x1F)<<3, g=((rgb>>5)&0x3F)<<2, b=(rgb&0x1F)<<3;
      bool is_mag = (r>=vp.magenta_r_min && b>=vp.magenta_b_min && g<=vp.magenta_g_max);
      if (is_mag) { if (x < w/2) left++; else right++; }
    }
  }
  if (left+right < 100) return false;
  if (on_right) *on_right = (right >= left);
  return true;
}
