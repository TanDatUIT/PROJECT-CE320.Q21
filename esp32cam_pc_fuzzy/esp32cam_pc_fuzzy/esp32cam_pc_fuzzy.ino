/*
 * ESP32-CAM PC FUZZY stream project
 *
 * ESP32 chi phu trach camera:
 * - /capture: tra ve 1 JPEG
 * - :81/stream : MJPEG stream de xem tren browser
 *
 * PC Python se xu ly fuzzy/AI nang hon.
 */

#include "esp_camera.h"
#include "esp_http_server.h"
#include <Arduino.h>
#include <WiFi.h>

// Sua theo Wi-Fi dang dung.
const char* WIFI_SSID = "Bich Tram";
const char* WIFI_PASS = "99999999";

// Pin map AI-Thinker ESP32-CAM
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

#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=frame"
#define STREAM_BOUNDARY     "\r\n--frame\r\n"
#define STREAM_PART         "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

namespace {

httpd_handle_t gCaptureHttpd = nullptr;
httpd_handle_t gStreamHttpd = nullptr;
int gBrightness = 0;
int gContrast = 1;
int gSaturation = 0;
int gQuality = 14;

esp_err_t captureHandler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  const esp_err_t res = httpd_resp_send(req, reinterpret_cast<const char*>(fb->buf), fb->len);
  esp_camera_fb_return(fb);
  return res;
}

esp_err_t streamHandler(httpd_req_t* req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char part[64];
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("ERR,camera_fb_get_failed");
      return ESP_FAIL;
    }

    const size_t partLen = snprintf(part, sizeof(part), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, part, partLen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, reinterpret_cast<const char*>(fb->buf), fb->len);
    }
    esp_camera_fb_return(fb);

    if (res != ESP_OK) {
      break;
    }
    delay(25);
  }

  return res;
}

esp_err_t rootHandler(httpd_req_t* req) {
  const char* html =
      "<!doctype html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>ESP32-CAM PC FUZZY</title>"
      "<style>body{font-family:Arial;margin:24px;background:#f5f7fa;color:#111}"
      "img{max-width:100%;border:1px solid #bbb}code{background:#eee;padding:2px 4px}</style>"
      "</head><body><h2>ESP32-CAM PC FUZZY</h2>"
      "<p>Use <code>/capture</code> for PC Python, <code>:81/stream</code> for browser.</p>"
      "<p><a href='/capture'>Capture</a> | <a id='streamLink'>Stream</a></p>"
      "<img id='streamImg'>"
      "<script>"
      "const u='http://'+location.hostname+':81/stream';"
      "document.getElementById('streamLink').href=u;"
      "document.getElementById('streamImg').src=u;"
      "</script></body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

esp_err_t controlHandler(httpd_req_t* req) {
  char query[160] = {0};
  char value[16] = {0};
  bool changed = false;
  sensor_t* s = esp_camera_sensor_get();

  if (s && httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    if (httpd_query_key_value(query, "brightness", value, sizeof(value)) == ESP_OK) {
      const int brightness = constrain(atoi(value), -2, 2);
      s->set_brightness(s, brightness);
      gBrightness = brightness;
      changed = true;
    }
    if (httpd_query_key_value(query, "contrast", value, sizeof(value)) == ESP_OK) {
      const int contrast = constrain(atoi(value), -2, 2);
      s->set_contrast(s, contrast);
      gContrast = contrast;
      changed = true;
    }
    if (httpd_query_key_value(query, "saturation", value, sizeof(value)) == ESP_OK) {
      const int saturation = constrain(atoi(value), -2, 2);
      s->set_saturation(s, saturation);
      gSaturation = saturation;
      changed = true;
    }
    if (httpd_query_key_value(query, "quality", value, sizeof(value)) == ESP_OK) {
      const int quality = constrain(atoi(value), 10, 30);
      s->set_quality(s, quality);
      gQuality = quality;
      changed = true;
    }
  }

  if (changed) {
    Serial.printf(
        "CONFIG,brightness=%d,contrast=%d,saturation=%d,quality=%d\n",
        gBrightness,
        gContrast,
        gSaturation,
        gQuality);
  }

  char body[160];
  snprintf(
      body,
      sizeof(body),
      "{\"ok\":true,\"changed\":%s,\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,\"quality\":%d}",
      changed ? "true" : "false",
      gBrightness,
      gContrast,
      gSaturation,
      gQuality);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

bool startCamera() {
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
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = psramFound() ? FRAMESIZE_QVGA : FRAMESIZE_QQVGA;
  config.jpeg_quality = gQuality;
  config.fb_count = psramFound() ? 2 : 1;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("ERR,camera_init_failed,0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, gBrightness);
    s->set_contrast(s, gContrast);
    s->set_saturation(s, gSaturation);
    s->set_quality(s, gQuality);
    s->set_framesize(s, psramFound() ? FRAMESIZE_QVGA : FRAMESIZE_QQVGA);
  }
  return true;
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi connecting to %s", WIFI_SSID);

  const uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERR,wifi_connect_failed");
    return false;
  }

  Serial.print("WiFi OK, IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("PC capture URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/capture");
  Serial.print("Browser stream: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/stream");
  return true;
}

bool startHttpServer() {
  httpd_config_t captureConfig = HTTPD_DEFAULT_CONFIG();
  captureConfig.server_port = 80;
  captureConfig.ctrl_port = 32768;
  captureConfig.max_uri_handlers = 4;

  if (httpd_start(&gCaptureHttpd, &captureConfig) != ESP_OK) {
    Serial.println("ERR,capture_httpd_start_failed");
    return false;
  }

  const httpd_uri_t rootUri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = rootHandler,
      .user_ctx = nullptr,
  };
  const httpd_uri_t captureUri = {
      .uri = "/capture",
      .method = HTTP_GET,
      .handler = captureHandler,
      .user_ctx = nullptr,
  };
  const httpd_uri_t controlUri = {
      .uri = "/control",
      .method = HTTP_GET,
      .handler = controlHandler,
      .user_ctx = nullptr,
  };
  httpd_register_uri_handler(gCaptureHttpd, &rootUri);
  httpd_register_uri_handler(gCaptureHttpd, &captureUri);
  httpd_register_uri_handler(gCaptureHttpd, &controlUri);

  httpd_config_t streamConfig = HTTPD_DEFAULT_CONFIG();
  streamConfig.server_port = 81;
  streamConfig.ctrl_port = 32769;
  streamConfig.max_uri_handlers = 1;

  if (httpd_start(&gStreamHttpd, &streamConfig) != ESP_OK) {
    Serial.println("ERR,stream_httpd_start_failed");
    return false;
  }

  const httpd_uri_t streamUri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = streamHandler,
      .user_ctx = nullptr,
  };
  httpd_register_uri_handler(gStreamHttpd, &streamUri);
  return true;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  delay(1000);
  Serial.println();
  Serial.println("ESP32-CAM PC FUZZY boot");

  if (!startCamera()) {
    return;
  }
  if (!connectWiFi()) {
    return;
  }
  if (!startHttpServer()) {
    return;
  }

  Serial.println("READY");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WARN,wifi_disconnected");
    WiFi.reconnect();
    delay(2000);
    return;
  }
  delay(1000);
}
