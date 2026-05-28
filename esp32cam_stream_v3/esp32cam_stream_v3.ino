/*
 * ============================================================
 *  esp32cam_stream_v3.ino
 *  ESP32-CAM (AI-Thinker) — Stream + Fresh Capture cho PC fuzzy
 *
 *  Triet ly: ESP chi lam "sensor node" — capture JPEG, serve qua HTTP.
 *  PC chiu trach nhiem ROI selection + fuzzy logic + finger counting.
 *
 *  Endpoints:
 *    GET /          : trang index don gian, link toi stream/capture
 *    GET /stream    : MJPEG stream lien tuc (cho demo realtime)
 *    GET /capture   : 1 JPEG tuoi (chup lai moi request) — PC pull moi 500ms
 *    GET /info      : JSON metadata (fps, frame_size, uptime)
 *
 *  Khac biet so voi esp32cam_stream.ino (giu nguyen lam tham khao):
 *    - Bo on-device fuzzy edge detection (PC lam het)
 *    - /capture tra fresh shot (chu khong phai "qualified capture" cache 10s)
 *    - Frame size QVGA (320x240) thay vi QQVGA — PC fuzzy can resolution hon
 *    - Pixel format JPEG truc tiep (khong qua frame2jpg) — giam CPU ESP
 *    - ae_level=2, gainceiling=32X, brightness=1 — sang hon, khong can LED
 * ============================================================
 */

#include "esp_camera.h"
#include <Arduino.h>
#include <WiFi.h>
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ── CAU HINH WI-FI ──────────────────────────────────────
const char* WIFI_SSID = "Dat Dat";
const char* WIFI_PASS = "12345678";

// ── PIN MAP (AI-Thinker ESP32-CAM) ─────────────────────
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ── CAU HINH CAMERA ────────────────────────────
constexpr framesize_t kFrameSize = FRAMESIZE_QVGA;       // 320x240
constexpr pixformat_t kPixelFormat = PIXFORMAT_JPEG;     // JPEG truc tiep
constexpr uint32_t kXclkHz = 20000000;
constexpr int kJpegQuality = 12;                         // 0..63 (thap = chat luong cao)

#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=frame"
#define STREAM_BOUNDARY     "\r\n--frame\r\n"
#define STREAM_PART         "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

namespace {

SemaphoreHandle_t gCamMutex = nullptr;
uint32_t gFrameCounter = 0;
uint32_t gLastCaptureMs = 0;

bool initCamera() {
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
  config.xclk_freq_hz = kXclkHz;
  config.pixel_format = kPixelFormat;
  config.frame_size = kFrameSize;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.jpeg_quality = kJpegQuality;
  config.fb_count = psramFound() ? 2 : 1;
  config.grab_mode = CAMERA_GRAB_LATEST;

  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[LOI] Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    // Tinh chinh cho ung dung nhan dien tay: sang vua, contrast cao
    sensor->set_brightness(sensor, 1);
    sensor->set_contrast(sensor, 2);
    sensor->set_saturation(sensor, 0);
    sensor->set_gain_ctrl(sensor, 1);
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_whitebal(sensor, 1);
    sensor->set_awb_gain(sensor, 1);
    sensor->set_ae_level(sensor, 0);       // co the dieu chinh runtime qua /control
    sensor->set_gainceiling(sensor, GAINCEILING_32X);
    sensor->set_hmirror(sensor, 0);
    sensor->set_vflip(sensor, 0);
  }

  Serial.println("[OK] Camera OV2640 san sang (QVGA 320x240 JPEG)");
  return true;
}

esp_err_t capture_handler(httpd_req_t* req) {
  if (xSemaphoreTake(gCamMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    return httpd_resp_send(req, "Camera busy", HTTPD_RESP_USE_STRLEN);
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (fb == nullptr) {
    xSemaphoreGive(gCamMutex);
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_send(req, "Capture failed", HTTPD_RESP_USE_STRLEN);
  }

  gLastCaptureMs = millis();
  ++gFrameCounter;

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  char hbuf[24];
  snprintf(hbuf, sizeof(hbuf), "%lu", static_cast<unsigned long>(gLastCaptureMs));
  httpd_resp_set_hdr(req, "X-Capture-Ms", hbuf);

  const esp_err_t res = httpd_resp_send(
      req, reinterpret_cast<const char*>(fb->buf), fb->len);

  esp_camera_fb_return(fb);
  xSemaphoreGive(gCamMutex);
  return res;
}

esp_err_t stream_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char partBuf[128];
  esp_err_t res = ESP_OK;

  while (true) {
    if (xSemaphoreTake(gCamMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (fb == nullptr) {
      xSemaphoreGive(gCamMutex);
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      const size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, partBuf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(
          req, reinterpret_cast<const char*>(fb->buf), fb->len);
    }

    esp_camera_fb_return(fb);
    xSemaphoreGive(gCamMutex);

    if (res != ESP_OK) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(20));   // ~50 fps cap, thuc te ~10-15 fps
  }
  return res;
}

// Helper: parse query string "var=X&val=Y"
bool getQueryParam(httpd_req_t* req, const char* key, char* out, size_t outLen) {
  size_t qlen = httpd_req_get_url_query_len(req);
  if (qlen == 0 || qlen >= 256) return false;
  char query[256];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
  return httpd_query_key_value(query, key, out, outLen) == ESP_OK;
}

esp_err_t control_handler(httpd_req_t* req) {
  char var[16], val[16];
  if (!getQueryParam(req, "var", var, sizeof(var)) ||
      !getQueryParam(req, "val", val, sizeof(val))) {
    httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_send(req, "Need var=X&val=Y", HTTPD_RESP_USE_STRLEN);
  }
  int v = atoi(val);
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_send(req, "no sensor", HTTPD_RESP_USE_STRLEN);
  }

  int rc = -1;
  if      (strcmp(var, "brightness") == 0) rc = s->set_brightness(s, v);
  else if (strcmp(var, "contrast")   == 0) rc = s->set_contrast(s, v);
  else if (strcmp(var, "ae_level")   == 0) rc = s->set_ae_level(s, v);
  else if (strcmp(var, "aec_value")  == 0) rc = s->set_aec_value(s, v);
  else if (strcmp(var, "aec")        == 0) rc = s->set_exposure_ctrl(s, v);
  else if (strcmp(var, "agc")        == 0) rc = s->set_gain_ctrl(s, v);
  else if (strcmp(var, "agc_gain")   == 0) rc = s->set_agc_gain(s, v);
  else if (strcmp(var, "gainceiling")== 0) rc = s->set_gainceiling(s, (gainceiling_t)v);
  else if (strcmp(var, "hmirror")    == 0) rc = s->set_hmirror(s, v);
  else if (strcmp(var, "vflip")      == 0) rc = s->set_vflip(s, v);
  else {
    httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_send(req, "unknown var", HTTPD_RESP_USE_STRLEN);
  }

  char body[64];
  snprintf(body, sizeof(body), "{\"var\":\"%s\",\"val\":%d,\"rc\":%d}", var, v, rc);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

esp_err_t info_handler(httpd_req_t* req) {
  char body[256];
  snprintf(
      body, sizeof(body),
      "{\"frame_size\":\"QVGA_320x240\",\"pixel_format\":\"JPEG\","
      "\"jpeg_quality\":%d,\"frame_counter\":%lu,\"last_capture_ms\":%lu,"
      "\"uptime_ms\":%lu,\"free_heap\":%lu}",
      kJpegQuality,
      static_cast<unsigned long>(gFrameCounter),
      static_cast<unsigned long>(gLastCaptureMs),
      static_cast<unsigned long>(millis()),
      static_cast<unsigned long>(ESP.getFreeHeap()));
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

esp_err_t index_handler(httpd_req_t* req) {
  const char* html =
      "<!doctype html><html><head><meta charset='utf-8'>"
      "<title>ESP32-CAM v3 (PC Fuzzy)</title>"
      "<style>body{font-family:sans-serif;background:#222;color:#eee;padding:20px}"
      "img{max-width:640px;border:2px solid #444}a{color:#7af}</style></head>"
      "<body><h1>ESP32-CAM v3 — Dumb Streamer</h1>"
      "<p>ESP chi capture + serve. PC chay fuzzy logic.</p>"
      "<ul>"
      "<li><a href='/stream'>/stream</a> — MJPEG realtime</li>"
      "<li><a href='/capture'>/capture</a> — 1 JPEG tuoi</li>"
      "<li><a href='/info'>/info</a> — metadata JSON</li>"
      "</ul>"
      "<img src='/stream' alt='stream'>"
      "</body></html>";
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

void startServer() {
  httpd_handle_t server = nullptr;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 5;
  config.stack_size = 8192;

  if (httpd_start(&server, &config) != ESP_OK) {
    Serial.println("[LOI] Khong khoi dong duoc HTTP server!");
    return;
  }

  httpd_uri_t uris[] = {
      {.uri = "/",        .method = HTTP_GET, .handler = index_handler,   .user_ctx = nullptr},
      {.uri = "/stream",  .method = HTTP_GET, .handler = stream_handler,  .user_ctx = nullptr},
      {.uri = "/capture", .method = HTTP_GET, .handler = capture_handler, .user_ctx = nullptr},
      {.uri = "/info",    .method = HTTP_GET, .handler = info_handler,    .user_ctx = nullptr},
      {.uri = "/control", .method = HTTP_GET, .handler = control_handler, .user_ctx = nullptr},
  };
  for (auto& u : uris) {
    httpd_register_uri_handler(server, &u);
  }
  Serial.println("[OK] HTTP server da khoi dong");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32-CAM Stream v3 (Dumb Streamer) ===");

  gCamMutex = xSemaphoreCreateMutex();
  if (gCamMutex == nullptr) {
    Serial.println("[LOI] Khong tao duoc mutex!");
    while (true) delay(1000);
  }

  if (!initCamera()) {
    while (true) delay(1000);
  }

  Serial.printf("Dang ket noi Wi-Fi: %s ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 30) {
    delay(500);
    Serial.print(".");
    ++retry;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[LOI] Khong ket noi duoc Wi-Fi! Restart...");
    delay(2000);
    ESP.restart();
  }

  Serial.println("\n[OK] Wi-Fi da ket noi");
  Serial.print("[IP] ");
  Serial.println(WiFi.localIP());
  Serial.printf("[URL] Stream : http://%s/stream\n", WiFi.localIP().toString().c_str());
  Serial.printf("[URL] Capture: http://%s/capture\n", WiFi.localIP().toString().c_str());
  Serial.printf("[URL] Info   : http://%s/info\n", WiFi.localIP().toString().c_str());

  startServer();
}

void loop() {
  // HTTP server chay tren task rieng — loop chi idle
  delay(1000);
}
