/*
 * ============================================================
 *  esp32cam_stream.ino
 *  Nạp vào ESP32-CAM bằng Arduino IDE
 *
 *  Chức năng:
 *  - Stream MJPEG qua Wi-Fi HTTP
 *  - Chạy fuzzy edge detection trực tiếp trên ESP32-CAM
 *  - Xuất chỉ số qua Serial và endpoint /metrics
 * ============================================================
 */

#include "esp_camera.h"
#include "img_converters.h"
#include <Arduino.h>
#include <WiFi.h>
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdlib.h>

// ── CẤU HÌNH WI-FI ──────────────────────────────────────
const char* WIFI_SSID = "Bich Tram";
const char* WIFI_PASS = "99999999";

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

// ── CẤU HÌNH CAMERA + FUZZY ────────────────────────────
constexpr framesize_t kFrameSize = FRAMESIZE_QQVGA;
constexpr pixformat_t kPixelFormat = PIXFORMAT_GRAYSCALE;
constexpr uint32_t kXclkHz = 20000000;
constexpr int kMaxWidth = 160;
constexpr int kJpegQuality = 18;
constexpr uint32_t kLoopDelayMs = 10;
constexpr uint32_t kAnalyzeIntervalMs = 200;
constexpr uint32_t kSummaryWindowMs = 10000;
constexpr uint32_t kQualifiedCaptureIntervalMs = 10000;
constexpr float kEdgeThreshold = 0.32f;
constexpr float kHandEdgeDensityThreshold = 0.035f;
constexpr float kEdgeDensityAlpha = 0.25f;
constexpr float kMeanStrengthAlpha = 0.20f;
constexpr int kMinHandWidth = 18;
constexpr int kMinHandHeight = 18;
constexpr uint8_t kHandOnFrames = 3;
constexpr uint8_t kHandOffFrames = 5;
constexpr uint8_t kFingerHistorySize = 5;

#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=frame"
#define STREAM_BOUNDARY     "\r\n--frame\r\n"
#define STREAM_PART         "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

namespace {

struct FuzzyMetrics {
  float edgeDensity;
  float meanStrength;
  bool handDetected;
  int fingerCount;
  int minX;
  int minY;
  int maxX;
  int maxY;
  uint32_t frameTimeMs;
  uint32_t frameCounter;
  uint32_t lastUpdateMs;
  uint16_t frameWidth;
  uint16_t frameHeight;
};

struct WindowStats {
  uint32_t startMs;
  uint32_t sampleCount;
  uint32_t handDetectedCount;
  float edgeDensitySum;
  float meanStrengthSum;
  uint32_t fingerBins[6];
  bool captureOccurred;
  uint32_t captureMs;
  FuzzyMetrics captureMetrics;
};

uint8_t linePrev[kMaxWidth];
uint8_t lineCurr[kMaxWidth];
uint8_t lineNext[kMaxWidth];
uint16_t colEdgeCount[kMaxWidth];
uint16_t colEdgeCountTop[kMaxWidth];
uint16_t colEdgeCountSmooth[kMaxWidth];

SemaphoreHandle_t gDataMutex = nullptr;
FuzzyMetrics gMetrics = {0.0f, 0.0f, false, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0};
float gEdgeDensityFiltered = 0.0f;
float gMeanStrengthFiltered = 0.0f;
bool gFilterPrimed = false;
bool gHandLatched = false;
uint8_t gHandOnStreak = 0;
uint8_t gHandOffStreak = 0;
uint8_t* gLatestJpeg = nullptr;
size_t gLatestJpegLen = 0;
uint8_t* gQualifiedJpeg = nullptr;
size_t gQualifiedJpegLen = 0;
uint32_t gLastQualifiedCaptureMs = 0;
FuzzyMetrics gQualifiedCaptureMetrics = {0.0f, 0.0f, false, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0};
uint8_t gFingerHistory[kFingerHistorySize] = {0};
uint8_t gFingerHistoryCount = 0;
uint8_t gFingerHistoryIndex = 0;
WindowStats gWindowStats = {0, 0, 0, 0.0f, 0.0f, {0, 0, 0, 0, 0, 0}, false, 0, {0.0f, 0.0f, false, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0}};
uint32_t gLastAnalyzeMs = 0;

float triangle(float x, float a, float b, float c) {
  if (x <= a || x >= c) {
    return 0.0f;
  }
  if (x == b) {
    return 1.0f;
  }
  if (x < b) {
    return (x - a) / (b - a);
  }
  return (c - x) / (c - b);
}

float trapLeft(float x, float a, float b) {
  if (x <= a) {
    return 1.0f;
  }
  if (x >= b) {
    return 0.0f;
  }
  return (b - x) / (b - a);
}

float trapRight(float x, float a, float b) {
  if (x <= a) {
    return 0.0f;
  }
  if (x >= b) {
    return 1.0f;
  }
  return (x - a) / (b - a);
}

void loadGrayLine(const camera_fb_t* fb, int y, uint8_t* out) {
  const uint8_t* row = fb->buf + (y * fb->width);
  for (int x = 0; x < fb->width; ++x) {
    out[x] = row[x];
  }
}

float fuzzyEdgeFromDelta(uint8_t center, const uint8_t* up, const uint8_t* mid, const uint8_t* down, int x) {
  const int d1 = abs(static_cast<int>(center) - static_cast<int>(up[x - 1]));
  const int d2 = abs(static_cast<int>(center) - static_cast<int>(up[x]));
  const int d3 = abs(static_cast<int>(center) - static_cast<int>(up[x + 1]));
  const int d4 = abs(static_cast<int>(center) - static_cast<int>(mid[x - 1]));
  const int d5 = abs(static_cast<int>(center) - static_cast<int>(mid[x + 1]));
  const int d6 = abs(static_cast<int>(center) - static_cast<int>(down[x - 1]));
  const int d7 = abs(static_cast<int>(center) - static_cast<int>(down[x]));
  const int d8 = abs(static_cast<int>(center) - static_cast<int>(down[x + 1]));

  const float delta = static_cast<float>(
      max(max(max(d1, d2), max(d3, d4)), max(max(d5, d6), max(d7, d8))));
  const float muLow = trapLeft(delta, 15.0f, 40.0f);
  const float muMed = triangle(delta, 25.0f, 70.0f, 115.0f);
  const float muHigh = trapRight(delta, 90.0f, 170.0f);

  const float numerator = muLow * 0.10f + muMed * 0.55f + muHigh * 1.00f;
  const float denominator = muLow + muMed + muHigh + 1e-6f;
  return numerator / denominator;
}

void smoothHistogram(const uint16_t* src, uint16_t* dst, int x0, int x1) {
  for (int x = x0; x <= x1; ++x) {
    const uint16_t left = src[max(x0, x - 1)];
    const uint16_t mid = src[x];
    const uint16_t right = src[min(x1, x + 1)];
    dst[x] = static_cast<uint16_t>((left + (2 * mid) + right) / 4);
  }
}

void resetFingerHistory() {
  memset(gFingerHistory, 0, sizeof(gFingerHistory));
  gFingerHistoryCount = 0;
  gFingerHistoryIndex = 0;
}

uint8_t stabilizeFingerCount(uint8_t rawCount) {
  gFingerHistory[gFingerHistoryIndex] = rawCount;
  gFingerHistoryIndex = (gFingerHistoryIndex + 1) % kFingerHistorySize;
  if (gFingerHistoryCount < kFingerHistorySize) {
    ++gFingerHistoryCount;
  }

  uint8_t bins[6] = {0};
  for (uint8_t i = 0; i < gFingerHistoryCount; ++i) {
    const uint8_t value = gFingerHistory[i] > 5 ? 5 : gFingerHistory[i];
    ++bins[value];
  }

  uint8_t bestValue = 0;
  uint8_t bestCount = 0;
  for (uint8_t value = 0; value <= 5; ++value) {
    if (bins[value] > bestCount) {
      bestCount = bins[value];
      bestValue = value;
    }
  }
  return bestValue;
}

int estimateFingerCount(const uint16_t* histogram, int x0, int x1, int minPeakGap, int minPeakHeight, int minPeakProminence) {
  int peaks = 0;
  int lastPeakX = -minPeakGap;
  const int halfWindow = max(2, minPeakGap / 2);

  for (int x = x0 + halfWindow; x <= x1 - halfWindow; ++x) {
    const int mid = histogram[x];
    if (mid < minPeakHeight) {
      continue;
    }

    bool isLocalMax = true;
    int valley = mid;
    for (int k = x - halfWindow; k <= x + halfWindow; ++k) {
      if (k == x) {
        continue;
      }
      if (histogram[k] > mid) {
        isLocalMax = false;
        break;
      }
      valley = min(valley, static_cast<int>(histogram[k]));
    }

    if (!isLocalMax) {
      continue;
    }

    if ((mid - valley) < minPeakProminence) {
      continue;
    }

    if ((x - lastPeakX) >= minPeakGap) {
      ++peaks;
      lastPeakX = x;
    }
  }

  if (peaks > 5) {
    peaks = 5;
  }
  return peaks;
}

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
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[LỖI] Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    sensor->set_brightness(sensor, 1);
    sensor->set_contrast(sensor, 2);
    sensor->set_saturation(sensor, -1);
    sensor->set_gain_ctrl(sensor, 1);
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_whitebal(sensor, 1);
    sensor->set_hmirror(sensor, 0);
    sensor->set_vflip(sensor, 0);
  }

  Serial.println("[OK] Camera OV2640 sẵn sàng (160x120 GRAYSCALE)");
  return true;
}

bool frameToJpeg(camera_fb_t* fb, uint8_t** jpgBuf, size_t* jpgLen) {
  if (fb == nullptr || jpgBuf == nullptr || jpgLen == nullptr) {
    return false;
  }

  if (fb->format == PIXFORMAT_JPEG) {
    *jpgBuf = fb->buf;
    *jpgLen = fb->len;
    return true;
  }

  return frame2jpg(fb, kJpegQuality, jpgBuf, jpgLen);
}

bool copyLatestJpeg(uint8_t** outBuf, size_t* outLen) {
  if (outBuf == nullptr || outLen == nullptr || gDataMutex == nullptr) {
    return false;
  }

  if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    return false;
  }

  if (gLatestJpeg == nullptr || gLatestJpegLen == 0) {
    xSemaphoreGive(gDataMutex);
    return false;
  }

  uint8_t* copy = static_cast<uint8_t*>(malloc(gLatestJpegLen));
  if (copy == nullptr) {
    xSemaphoreGive(gDataMutex);
    return false;
  }

  memcpy(copy, gLatestJpeg, gLatestJpegLen);
  *outBuf = copy;
  *outLen = gLatestJpegLen;
  xSemaphoreGive(gDataMutex);
  return true;
}

void storeLatestJpeg(uint8_t* jpgBuf, size_t jpgLen) {
  if (jpgBuf == nullptr || jpgLen == 0 || gDataMutex == nullptr) {
    return;
  }

  if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    free(jpgBuf);
    return;
  }

  if (gLatestJpeg != nullptr) {
    free(gLatestJpeg);
  }
  gLatestJpeg = jpgBuf;
  gLatestJpegLen = jpgLen;
  xSemaphoreGive(gDataMutex);
}

void storeQualifiedJpeg(uint8_t* jpgBuf, size_t jpgLen, uint32_t captureMs, const FuzzyMetrics& captureMetrics) {
  if (jpgBuf == nullptr || jpgLen == 0 || gDataMutex == nullptr) {
    return;
  }

  if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    free(jpgBuf);
    return;
  }

  if (gQualifiedJpeg != nullptr) {
    free(gQualifiedJpeg);
  }
  gQualifiedJpeg = jpgBuf;
  gQualifiedJpegLen = jpgLen;
  gLastQualifiedCaptureMs = captureMs;
  gQualifiedCaptureMetrics = captureMetrics;
  xSemaphoreGive(gDataMutex);
}

FuzzyMetrics snapshotMetrics() {
  FuzzyMetrics snapshot = gMetrics;
  if (gDataMutex == nullptr) {
    return snapshot;
  }

  if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return snapshot;
  }
  snapshot = gMetrics;
  xSemaphoreGive(gDataMutex);
  return snapshot;
}

bool copyQualifiedJpeg(uint8_t** outBuf, size_t* outLen, uint32_t* captureMs) {
  if (outBuf == nullptr || outLen == nullptr || gDataMutex == nullptr) {
    return false;
  }

  if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    return false;
  }

  if (gQualifiedJpeg == nullptr || gQualifiedJpegLen == 0) {
    xSemaphoreGive(gDataMutex);
    return false;
  }

  uint8_t* copy = static_cast<uint8_t*>(malloc(gQualifiedJpegLen));
  if (copy == nullptr) {
    xSemaphoreGive(gDataMutex);
    return false;
  }

  memcpy(copy, gQualifiedJpeg, gQualifiedJpegLen);
  *outBuf = copy;
  *outLen = gQualifiedJpegLen;
  if (captureMs != nullptr) {
    *captureMs = gLastQualifiedCaptureMs;
  }
  xSemaphoreGive(gDataMutex);
  return true;
}

FuzzyMetrics snapshotQualifiedCaptureMetrics() {
  FuzzyMetrics snapshot = gQualifiedCaptureMetrics;
  if (gDataMutex == nullptr) {
    return snapshot;
  }

  if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return snapshot;
  }
  snapshot = gQualifiedCaptureMetrics;
  xSemaphoreGive(gDataMutex);
  return snapshot;
}

void resetWindowStats(uint32_t nowMs) {
  gWindowStats.startMs = nowMs;
  gWindowStats.sampleCount = 0;
  gWindowStats.handDetectedCount = 0;
  gWindowStats.edgeDensitySum = 0.0f;
  gWindowStats.meanStrengthSum = 0.0f;
  memset(gWindowStats.fingerBins, 0, sizeof(gWindowStats.fingerBins));
  gWindowStats.captureOccurred = false;
  gWindowStats.captureMs = 0;
  gWindowStats.captureMetrics = {0.0f, 0.0f, false, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0};
}

void updateWindowStats(const FuzzyMetrics& metrics) {
  if (gWindowStats.startMs == 0) {
    resetWindowStats(metrics.lastUpdateMs);
  }

  ++gWindowStats.sampleCount;
  if (metrics.handDetected) {
    ++gWindowStats.handDetectedCount;
  }
  gWindowStats.edgeDensitySum += metrics.edgeDensity;
  gWindowStats.meanStrengthSum += metrics.meanStrength;
  const int fingerBin = constrain(metrics.fingerCount, 0, 5);
  ++gWindowStats.fingerBins[fingerBin];
}

uint8_t dominantFingerCountFromWindow() {
  uint8_t bestValue = 0;
  uint32_t bestCount = 0;
  for (uint8_t value = 0; value <= 5; ++value) {
    if (gWindowStats.fingerBins[value] > bestCount) {
      bestCount = gWindowStats.fingerBins[value];
      bestValue = value;
    }
  }
  return bestValue;
}

void printWindowSummary(uint32_t nowMs) {
  if (gWindowStats.sampleCount == 0) {
    return;
  }

  const float avgEdge = gWindowStats.edgeDensitySum / gWindowStats.sampleCount;
  const float avgMean = gWindowStats.meanStrengthSum / gWindowStats.sampleCount;
  const float handRatio = static_cast<float>(gWindowStats.handDetectedCount) / gWindowStats.sampleCount;
  const uint8_t dominantFinger = dominantFingerCountFromWindow();

  Serial.printf(
      "WINDOW_LOG,start_ms=%lu,end_ms=%lu,samples=%lu,avg_edge=%.6f,avg_mean=%.6f,hand_ratio=%.4f,dominant_fingers=%u,capture=%s,capture_ms=%lu\n",
      static_cast<unsigned long>(gWindowStats.startMs),
      static_cast<unsigned long>(nowMs),
      static_cast<unsigned long>(gWindowStats.sampleCount),
      avgEdge,
      avgMean,
      handRatio,
      dominantFinger,
      gWindowStats.captureOccurred ? "yes" : "no",
      static_cast<unsigned long>(gWindowStats.captureMs));
}

void printCaptureLog(const FuzzyMetrics& metrics, uint32_t captureMs) {
  Serial.printf(
      "CAPTURE_LOG,time_ms=%lu,edge_density=%.6f,mean_strength=%.6f,hand=%d,fingers=%d,bbox=%d,%d,%d,%d,frame_time_ms=%lu,frame=%lu\n",
      static_cast<unsigned long>(captureMs),
      metrics.edgeDensity,
      metrics.meanStrength,
      metrics.handDetected ? 1 : 0,
      metrics.fingerCount,
      metrics.minX,
      metrics.minY,
      metrics.maxX,
      metrics.maxY,
      static_cast<unsigned long>(metrics.frameTimeMs),
      static_cast<unsigned long>(metrics.frameCounter));
}

void updateMetricsFromFrame(camera_fb_t* fb, uint32_t frameTimeMs) {
  if (fb == nullptr || fb->format != PIXFORMAT_GRAYSCALE || fb->width > kMaxWidth || fb->height < 3) {
    return;
  }

  loadGrayLine(fb, 0, linePrev);
  loadGrayLine(fb, 1, lineCurr);

  const int roiX0 = fb->width / 4;
  const int roiX1 = fb->width - roiX0;
  const int roiY0 = fb->height / 5;
  const int roiY1 = fb->height - roiY0;

  uint32_t edgeCount = 0;
  uint32_t roiPixels = 0;
  float sumStrength = 0.0f;
  int minX = fb->width;
  int minY = fb->height;
  int maxX = -1;
  int maxY = -1;
  memset(colEdgeCount, 0, sizeof(colEdgeCount));
  memset(colEdgeCountTop, 0, sizeof(colEdgeCountTop));
  const int upperBandY = roiY0 + ((roiY1 - roiY0) * 11) / 20;

  for (int y = 1; y < fb->height - 1; ++y) {
    loadGrayLine(fb, y + 1, lineNext);

    if (y >= roiY0 && y < roiY1) {
      for (int x = max(1, roiX0); x < min(static_cast<int>(fb->width) - 1, roiX1); ++x) {
        const float edgeStrength = fuzzyEdgeFromDelta(lineCurr[x], linePrev, lineCurr, lineNext, x);
        sumStrength += edgeStrength;
        ++roiPixels;

        if (edgeStrength >= kEdgeThreshold) {
          ++edgeCount;
          ++colEdgeCount[x];
          if (y <= upperBandY) {
            ++colEdgeCountTop[x];
          }
          if (x < minX) minX = x;
          if (x > maxX) maxX = x;
          if (y < minY) minY = y;
          if (y > maxY) maxY = y;
        }
      }
    }

    memcpy(linePrev, lineCurr, fb->width);
    memcpy(lineCurr, lineNext, fb->width);
  }

  const float edgeDensity = roiPixels > 0 ? static_cast<float>(edgeCount) / roiPixels : 0.0f;
  const float meanStrength = roiPixels > 0 ? sumStrength / roiPixels : 0.0f;

  if (!gFilterPrimed) {
    gEdgeDensityFiltered = edgeDensity;
    gMeanStrengthFiltered = meanStrength;
    gFilterPrimed = true;
  } else {
    gEdgeDensityFiltered =
        (1.0f - kEdgeDensityAlpha) * gEdgeDensityFiltered + kEdgeDensityAlpha * edgeDensity;
    gMeanStrengthFiltered =
        (1.0f - kMeanStrengthAlpha) * gMeanStrengthFiltered + kMeanStrengthAlpha * meanStrength;
  }

  const int handWidth = maxX >= minX ? (maxX - minX + 1) : 0;
  const int handHeight = maxY >= minY ? (maxY - minY + 1) : 0;
  const bool handCandidate =
      gEdgeDensityFiltered >= kHandEdgeDensityThreshold &&
      maxX >= 0 &&
      handWidth >= kMinHandWidth &&
      handHeight >= kMinHandHeight;

  if (handCandidate) {
    gHandOffStreak = 0;
    if (gHandOnStreak < 255) {
      ++gHandOnStreak;
    }
    if (gHandOnStreak >= kHandOnFrames) {
      gHandLatched = true;
    }
  } else {
    gHandOnStreak = 0;
    if (gHandOffStreak < 255) {
      ++gHandOffStreak;
    }
    if (gHandOffStreak >= kHandOffFrames) {
      gHandLatched = false;
    }
  }

  const bool handDetected = gHandLatched;
  int fingerCount = 0;

  if (handDetected) {
    const int dynamicPeakGap = max(8, handWidth / 7);
    const int dynamicPeakHeight = max(4, handHeight / 9);
    const int dynamicPeakProminence = max(3, handHeight / 10);

    smoothHistogram(colEdgeCountTop, colEdgeCountSmooth, minX, maxX);

    for (int x = minX; x <= maxX; ++x) {
      if (colEdgeCountSmooth[x] < dynamicPeakHeight) {
        colEdgeCountSmooth[x] = 0;
      }
    }

    fingerCount = estimateFingerCount(
        colEdgeCountSmooth,
        minX,
        maxX,
        dynamicPeakGap,
        dynamicPeakHeight,
        dynamicPeakProminence);
    fingerCount = stabilizeFingerCount(static_cast<uint8_t>(fingerCount));
  } else {
    resetFingerHistory();
  }

  if (gDataMutex != nullptr && xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    gMetrics.edgeDensity = gEdgeDensityFiltered;
    gMetrics.meanStrength = gMeanStrengthFiltered;
    gMetrics.handDetected = handDetected;
    gMetrics.fingerCount = fingerCount;
    gMetrics.minX = handDetected ? minX : -1;
    gMetrics.minY = handDetected ? minY : -1;
    gMetrics.maxX = handDetected ? maxX : -1;
    gMetrics.maxY = handDetected ? maxY : -1;
    gMetrics.frameTimeMs = frameTimeMs;
    gMetrics.frameCounter += 1;
    gMetrics.lastUpdateMs = millis();
    gMetrics.frameWidth = fb->width;
    gMetrics.frameHeight = fb->height;
    xSemaphoreGive(gDataMutex);
  }
}

void printMetricsToSerial() {
  const FuzzyMetrics metrics = snapshotMetrics();
  Serial.printf(
      "edge_density=%.6f mean_strength=%.6f hand=%s fingers=%d bbox=%d,%d,%d,%d time_ms=%lu frame=%lu\n",
      metrics.edgeDensity,
      metrics.meanStrength,
      metrics.handDetected ? "yes" : "no",
      metrics.fingerCount,
      metrics.minX,
      metrics.minY,
      metrics.maxX,
      metrics.maxY,
      static_cast<unsigned long>(metrics.frameTimeMs),
      static_cast<unsigned long>(metrics.frameCounter));
}

esp_err_t capture_handler(httpd_req_t* req) {
  uint8_t* jpgBuf = nullptr;
  size_t jpgLen = 0;
  uint32_t captureMs = 0;
  if (!copyQualifiedJpeg(&jpgBuf, &jpgLen, &captureMs)) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "Qualified capture not ready", HTTPD_RESP_USE_STRLEN);
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char captureHeader[24];
  snprintf(captureHeader, sizeof(captureHeader), "%lu", static_cast<unsigned long>(captureMs));
  httpd_resp_set_hdr(req, "X-Capture-Ms", captureHeader);
  const esp_err_t res = httpd_resp_send(req, reinterpret_cast<const char*>(jpgBuf), jpgLen);
  free(jpgBuf);
  return res;
}

esp_err_t metrics_handler(httpd_req_t* req) {
  const FuzzyMetrics metrics = snapshotMetrics();
  const FuzzyMetrics captureMetrics = snapshotQualifiedCaptureMetrics();
  char body[512];
  snprintf(
      body,
      sizeof(body),
      "{\"edge_density\":%.6f,\"mean_strength\":%.6f,\"hand_detected\":%s,"
      "\"finger_count\":%d,\"min_x\":%d,\"min_y\":%d,\"max_x\":%d,\"max_y\":%d,"
      "\"frame_time_ms\":%lu,\"frame_counter\":%lu,\"last_update_ms\":%lu,"
      "\"frame_width\":%u,\"frame_height\":%u,\"capture_ready\":%s,\"last_capture_ms\":%lu,"
      "\"capture_finger_count\":%d,\"capture_hand_detected\":%s,"
      "\"capture_min_x\":%d,\"capture_min_y\":%d,\"capture_max_x\":%d,\"capture_max_y\":%d}",
      metrics.edgeDensity,
      metrics.meanStrength,
      metrics.handDetected ? "true" : "false",
      metrics.fingerCount,
      metrics.minX,
      metrics.minY,
      metrics.maxX,
      metrics.maxY,
      static_cast<unsigned long>(metrics.frameTimeMs),
      static_cast<unsigned long>(metrics.frameCounter),
      static_cast<unsigned long>(metrics.lastUpdateMs),
      metrics.frameWidth,
      metrics.frameHeight,
      gLastQualifiedCaptureMs > 0 ? "true" : "false",
      static_cast<unsigned long>(gLastQualifiedCaptureMs),
      captureMetrics.fingerCount,
      captureMetrics.handDetected ? "true" : "false",
      captureMetrics.minX,
      captureMetrics.minY,
      captureMetrics.maxX,
      captureMetrics.maxY);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

esp_err_t index_handler(httpd_req_t* req) {
  const char* html =
      "<!doctype html>"
      "<html>"
      "<head>"
      "<meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>ESP32-CAM Stream + Fuzzy</title>"
      "<style>"
      "body{font-family:Arial,sans-serif;background:#f2f4f7;color:#111;margin:0;padding:20px;}"
      ".wrap{max-width:1200px;margin:0 auto;}"
      ".card{background:#fff;border-radius:16px;padding:16px;box-shadow:0 10px 30px rgba(0,0,0,.08);margin-bottom:16px;}"
      ".stage{position:relative;width:100%;max-width:1040px;}"
      "img{width:100%;border-radius:12px;background:#000;display:block;}"
      ".bbox{position:absolute;border:5px solid #20b15a;border-radius:14px;box-sizing:border-box;pointer-events:none;display:none;box-shadow:0 0 0 2px rgba(255,255,255,.45) inset;}"
      ".badge{position:absolute;left:14px;top:14px;background:rgba(0,0,0,.72);color:#fff;padding:12px 16px;border-radius:12px;font-size:18px;font-weight:700;}"
      ".two-up{display:grid;grid-template-columns:2fr 1fr;gap:16px;align-items:start;}"
      ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;margin-top:16px;}"
      ".metric{background:#f7f9fc;border-radius:12px;padding:12px;}"
      ".label{font-size:12px;color:#666;text-transform:uppercase;margin-bottom:6px;}"
      ".value{font-size:22px;font-weight:700;}"
      ".small{font-size:14px;}"
      ".capture-box{width:100%;min-height:220px;display:flex;align-items:center;justify-content:center;background:#eef2f7;border-radius:12px;overflow:hidden;}"
      ".capture-box img{border-radius:0;}"
      ".capture-empty{padding:24px;color:#667085;text-align:center;font-size:15px;}"
      "a{color:#0b57d0;text-decoration:none;}"
      "@media (max-width:900px){.two-up{grid-template-columns:1fr;}}"
      "</style>"
      "</head>"
      "<body>"
      "<div class='wrap'>"
      "<div class='card'>"
      "<h1>ESP32-CAM Stream + Fuzzy</h1>"
      "<p class='small'>Stream: <a href='/stream' target='_blank'>/stream</a> | "
      "Capture: <a href='/capture' target='_blank'>/capture</a> | "
      "Metrics: <a href='/metrics' target='_blank'>/metrics</a></p>"
      "<div class='two-up'>"
      "<div class='stage'>"
      "<img id='stream' src='/stream' alt='ESP32-CAM stream'>"
      "<div class='bbox' id='bbox_overlay'></div>"
      "<div class='badge' id='status_badge'>Waiting...</div>"
      "</div>"
      "<div>"
      "<h2>Qualified Capture</h2>"
      "<div class='capture-box'>"
      "<div class='capture-empty' id='capture_empty'>Chưa có ảnh xác nhận. Ảnh chỉ được lưu mỗi 10 giây khi phát hiện hand hoặc fingers.</div>"
      "<img id='capture_img' src='' alt='Qualified capture' style='display:none;'>"
      "</div>"
      "<p class='small' id='capture_info'>Capture theo điều kiện phát hiện.</p>"
      "<p class='small' id='capture_analysis'>Phân tích capture sẽ hiện ở đây.</p>"
      "</div>"
      "</div>"
      "<div class='card'>"
      "<h2>Fuzzy Metrics</h2>"
      "<div class='grid'>"
      "<div class='metric'><div class='label'>Edge Density</div><div class='value' id='edge_density'>-</div></div>"
      "<div class='metric'><div class='label'>Mean Strength</div><div class='value' id='mean_strength'>-</div></div>"
      "<div class='metric'><div class='label'>Hand</div><div class='value' id='hand_detected'>-</div></div>"
      "<div class='metric'><div class='label'>Fingers</div><div class='value' id='finger_count'>-</div></div>"
      "<div class='metric'><div class='label'>BBox</div><div class='value small' id='bbox'>-</div></div>"
      "<div class='metric'><div class='label'>Frame Time</div><div class='value' id='frame_time_ms'>-</div></div>"
      "</div>"
      "</div>"
      "</div>"
      "<script>"
      "let lastCaptureMs=0;"
      "async function updateMetrics(){"
      "try{"
      "const r=await fetch('/metrics');"
      "const m=await r.json();"
      "document.getElementById('edge_density').textContent=Number(m.edge_density).toFixed(4);"
      "document.getElementById('mean_strength').textContent=Number(m.mean_strength).toFixed(4);"
      "document.getElementById('hand_detected').textContent=m.hand_detected?'YES':'NO';"
      "document.getElementById('finger_count').textContent=m.finger_count;"
      "document.getElementById('bbox').textContent=[m.min_x,m.min_y,m.max_x,m.max_y].join(', ');"
      "document.getElementById('frame_time_ms').textContent=m.frame_time_ms+' ms';"
      "const badge=document.getElementById('status_badge');"
      "badge.textContent='Hand: '+(m.hand_detected?'YES':'NO')+' | Fingers: '+m.finger_count;"
      "badge.style.background=m.hand_detected?'rgba(32,177,90,.82)':'rgba(0,0,0,.68)';"
      "const box=document.getElementById('bbox_overlay');"
      "if(m.hand_detected&&m.frame_width>0&&m.frame_height>0&&m.max_x>=m.min_x&&m.max_y>=m.min_y){"
      "box.style.display='block';"
      "box.style.left=(m.min_x*100/m.frame_width)+'%';"
      "box.style.top=(m.min_y*100/m.frame_height)+'%';"
      "box.style.width=((m.max_x-m.min_x+1)*100/m.frame_width)+'%';"
      "box.style.height=((m.max_y-m.min_y+1)*100/m.frame_height)+'%';"
      "}else{box.style.display='none';}"
      "const capInfo=document.getElementById('capture_info');"
      "if(m.capture_ready){"
      "capInfo.textContent='Capture xác nhận lúc: '+m.last_capture_ms+' ms';"
      "document.getElementById('capture_analysis').textContent='Capture hand: '+(m.capture_hand_detected?'YES':'NO')+' | fingers: '+m.capture_finger_count+' | bbox: '+[m.capture_min_x,m.capture_min_y,m.capture_max_x,m.capture_max_y].join(', ');"
      "if(m.last_capture_ms!==lastCaptureMs){"
      "lastCaptureMs=m.last_capture_ms;"
      "document.getElementById('capture_img').src='/capture?t='+m.last_capture_ms;"
      "document.getElementById('capture_img').style.display='block';"
      "document.getElementById('capture_empty').style.display='none';"
      "}"
      "}else{"
      "capInfo.textContent='Chưa có capture xác nhận trong phiên này.';"
      "document.getElementById('capture_analysis').textContent='Phân tích capture sẽ hiện ở đây.';"
      "}"
      "}catch(e){}"
      "}"
      "updateMetrics();"
      "setInterval(updateMetrics, 800);"
      "</script>"
      "</body>"
      "</html>";

  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

esp_err_t stream_handler(httpd_req_t* req) {
  esp_err_t res = ESP_OK;
  char partBuf[128];

  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    uint8_t* jpgBuf = nullptr;
    size_t jpgLen = 0;
    if (!copyLatestJpeg(&jpgBuf, &jpgLen)) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      const size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, jpgLen);
      res = httpd_resp_send_chunk(req, partBuf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, reinterpret_cast<const char*>(jpgBuf), jpgLen);
    }

    free(jpgBuf);

    if (res != ESP_OK) {
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  return res;
}

void startServer() {
  httpd_handle_t server = nullptr;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 6;

  if (httpd_start(&server, &config) != ESP_OK) {
    Serial.println("[LỖI] Không khởi động được HTTP server!");
    return;
  }

  httpd_uri_t streamUri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = nullptr};
  httpd_register_uri_handler(server, &streamUri);

  httpd_uri_t indexUri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = index_handler,
      .user_ctx = nullptr};
  httpd_register_uri_handler(server, &indexUri);

  httpd_uri_t captureUri = {
      .uri = "/capture",
      .method = HTTP_GET,
      .handler = capture_handler,
      .user_ctx = nullptr};
  httpd_register_uri_handler(server, &captureUri);

  httpd_uri_t metricsUri = {
      .uri = "/metrics",
      .method = HTTP_GET,
      .handler = metrics_handler,
      .user_ctx = nullptr};
  httpd_register_uri_handler(server, &metricsUri);

  Serial.println("[OK] HTTP Server đã khởi động");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32-CAM Stream + Fuzzy Server ===");

  gDataMutex = xSemaphoreCreateMutex();
  if (gDataMutex == nullptr) {
    Serial.println("[LỖI] Không tạo được data mutex!");
    while (true) {
      delay(1000);
    }
  }

  if (!initCamera()) {
    while (true) {
      delay(1000);
    }
  }

  Serial.print("LOGCSV_HEADER,");
  Serial.println("window_start_ms,window_end_ms,samples,avg_edge,avg_mean,hand_ratio,dominant_fingers,capture,capture_ms");

  Serial.printf("Đang kết nối Wi-Fi: %s ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    ++retry;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[LỖI] Không kết nối được Wi-Fi! Restart...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("\n[OK] Wi-Fi đã kết nối!");
  Serial.print("[IP] Địa chỉ IP: ");
  Serial.println(WiFi.localIP());
  Serial.printf("[URL] Stream : http://%s/stream\n", WiFi.localIP().toString().c_str());
  Serial.printf("[URL] Capture: http://%s/capture\n", WiFi.localIP().toString().c_str());
  Serial.printf("[URL] Metrics: http://%s/metrics\n", WiFi.localIP().toString().c_str());

  startServer();
}

void loop() {
  const uint32_t t0 = millis();
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb == nullptr) {
    Serial.println("[LỖI] Camera capture failed");
    delay(500);
    return;
  }

  if (fb->format != PIXFORMAT_GRAYSCALE || fb->width > kMaxWidth || fb->height < 3) {
    Serial.printf("[LỖI] Unexpected frame format/size: format=%d %ux%u\n", fb->format, fb->width, fb->height);
    esp_camera_fb_return(fb);
    delay(500);
    return;
  }

  const uint32_t nowMs = millis();
  const bool shouldAnalyze = (gLastAnalyzeMs == 0) || ((nowMs - gLastAnalyzeMs) >= kAnalyzeIntervalMs);
  if (shouldAnalyze) {
    updateMetricsFromFrame(fb, nowMs - t0);
    gLastAnalyzeMs = nowMs;
  }
  uint8_t* jpgBuf = nullptr;
  size_t jpgLen = 0;
  if (frameToJpeg(fb, &jpgBuf, &jpgLen)) {
    if (fb->format == PIXFORMAT_JPEG) {
      uint8_t* copy = static_cast<uint8_t*>(malloc(jpgLen));
      if (copy != nullptr) {
        memcpy(copy, jpgBuf, jpgLen);
        storeLatestJpeg(copy, jpgLen);
      }
    } else {
      storeLatestJpeg(jpgBuf, jpgLen);
      jpgBuf = nullptr;
    }
  }
  esp_camera_fb_return(fb);

  const FuzzyMetrics metrics = snapshotMetrics();
  if (shouldAnalyze) {
    updateWindowStats(metrics);
  }
  if (shouldAnalyze &&
      (metrics.handDetected || metrics.fingerCount > 0) &&
      (gLastQualifiedCaptureMs == 0 || (nowMs - gLastQualifiedCaptureMs) >= kQualifiedCaptureIntervalMs)) {
    uint8_t* qualifiedCopy = nullptr;
    size_t qualifiedLen = 0;
    if (copyLatestJpeg(&qualifiedCopy, &qualifiedLen)) {
      storeQualifiedJpeg(qualifiedCopy, qualifiedLen, nowMs, metrics);
      gWindowStats.captureOccurred = true;
      gWindowStats.captureMs = nowMs;
      gWindowStats.captureMetrics = metrics;
      printCaptureLog(metrics, nowMs);
    }
  }

  if (shouldAnalyze && gWindowStats.startMs > 0 && (nowMs - gWindowStats.startMs) >= kSummaryWindowMs) {
    printWindowSummary(nowMs);
    Serial.printf(
        "LOGCSV,%lu,%lu,%lu,%.6f,%.6f,%.4f,%u,%d,%lu\n",
        static_cast<unsigned long>(gWindowStats.startMs),
        static_cast<unsigned long>(nowMs),
        static_cast<unsigned long>(gWindowStats.sampleCount),
        gWindowStats.sampleCount > 0 ? (gWindowStats.edgeDensitySum / gWindowStats.sampleCount) : 0.0f,
        gWindowStats.sampleCount > 0 ? (gWindowStats.meanStrengthSum / gWindowStats.sampleCount) : 0.0f,
        gWindowStats.sampleCount > 0 ? (static_cast<float>(gWindowStats.handDetectedCount) / gWindowStats.sampleCount) : 0.0f,
        dominantFingerCountFromWindow(),
        gWindowStats.captureOccurred ? 1 : 0,
        static_cast<unsigned long>(gWindowStats.captureMs));
    resetWindowStats(nowMs);
  }

  delay(kLoopDelayMs);
}
