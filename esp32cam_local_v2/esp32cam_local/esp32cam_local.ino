/*
 * ============================================================
 *  esp32cam_local.ino
 *  Chạy local trên ESP32-CAM, không Wi-Fi, không HTTP stream
 *
 *  Chức năng:
 *  - Chụp frame grayscale trực tiếp từ camera
 *  - Ưu tiên nhận diện coarse gesture: none / fist / open
 *  - Dùng edge + foreground blob để tăng độ ổn định nhận diện
 *  - Chỉ dump capture qua Serial khi được yêu cầu SNAP
 *  - Tổng hợp kết quả theo cửa sổ 10 giây
 *  - In log qua Serial Monitor 115200
 * ============================================================
 */

#include "esp_camera.h"
#include "img_converters.h"
#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <math.h>
#include <stdlib.h>

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
constexpr framesize_t kFrameSize = FRAMESIZE_QVGA;
constexpr pixformat_t kPixelFormat = PIXFORMAT_GRAYSCALE;
constexpr uint32_t kXclkHz = 20000000;
constexpr int kMaxWidth = 320;
constexpr int kJpegQuality = 18;
constexpr int kDefaultBrightness = 0;
constexpr int kDefaultContrast = 2;
constexpr uint32_t kLoopDelayMs = 10;
constexpr uint32_t kAnalyzeIntervalMs = 200;
constexpr uint32_t kSummaryWindowMs = 10000;
constexpr uint32_t kDebugStatusIntervalMs = 2000;
constexpr int kFlashLedGpio = 4;
constexpr int kFlashLedChannel = 7;       // LEDC channel rieng, khac channel camera (0)
constexpr int kFlashLedFreqHz = 5000;
constexpr int kFlashLedResBits = 8;
constexpr int kFlashCaptureDuty = 32;     // ~12% — sang vua, khong choi
constexpr uint32_t kFlashWarmupMs = 120;  // doi AE on dinh sau khi bat LED
constexpr float kOpenPalmEdgeDensityThreshold = 0.085f;
constexpr float kEdgeDensityAlpha = 0.25f;
constexpr float kMeanStrengthAlpha = 0.20f;
constexpr int kMinHandWidth = 18;
constexpr int kMinHandHeight = 18;
constexpr uint8_t kHandOnFrames = 3;
constexpr uint8_t kHandOffFrames = 5;
constexpr uint8_t kFingerHistorySize = 5;
constexpr uint8_t kOpenPalmMinRawPeaks = 2;
constexpr bool kAlwaysDumpCaptureBase64 = true;
constexpr bool kEnableSdCaptureStorage = false;
constexpr char kCaptureDir[] = "/fuzzy_captures";
constexpr char kCaptureLogPath[] = "/fuzzy_capture_log.csv";
constexpr float kBinaryAreaMinRatio = 0.035f;
constexpr float kBinaryAreaMaxRatio = 0.70f;
constexpr uint8_t kGrayThresholdMargin = 18;

namespace {

enum GestureClass : uint8_t {
  kGestureNone = 0,
  kGestureFist = 1,
  kGestureOpen = 5,
};

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

struct BinaryCandidate {
  uint32_t pixelCount;
  uint32_t centerCount;
  int minX;
  int minY;
  int maxX;
  int maxY;
};

uint8_t linePrev[kMaxWidth];
uint8_t lineCurr[kMaxWidth];
uint8_t lineNext[kMaxWidth];
uint16_t colEdgeCount[kMaxWidth];
uint16_t colEdgeCountTop[kMaxWidth];
uint16_t colEdgeCountSmooth[kMaxWidth];
uint16_t colDarkCountTop[kMaxWidth];
uint16_t colBrightCountTop[kMaxWidth];
String gSerialCommandBuffer;

FuzzyMetrics gMetrics = {0.0f, 0.0f, false, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0};
float gEdgeDensityFiltered = 0.0f;
float gMeanStrengthFiltered = 0.0f;
bool gFilterPrimed = false;
bool gHandLatched = false;
uint8_t gHandOnStreak = 0;
uint8_t gHandOffStreak = 0;
uint8_t gFingerHistory[kFingerHistorySize] = {0};
uint8_t gFingerHistoryCount = 0;
uint8_t gFingerHistoryIndex = 0;
WindowStats gWindowStats = {0, 0, 0, 0.0f, 0.0f, {0, 0, 0, 0, 0, 0}, false, 0, {0.0f, 0.0f, false, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0}};
uint32_t gLastAnalyzeMs = 0;
uint32_t gLastPeriodicCaptureMs = 0;
bool gSdReady = false;
uint32_t gNextCaptureIndex = 1;
uint32_t gLastDebugStatusMs = 0;
int gCaptureJpegQuality = kJpegQuality;
int gBrightness = kDefaultBrightness;
int gContrast = kDefaultContrast;
float gEdgeThresholdDark = 0.26f;
float gEdgeThresholdBright = 0.20f;
float gHandEdgeDensityThreshold = 0.012f;
uint32_t gCaptureIntervalMs = 10000;  // mac dinh 10s, set qua SET capture_ms=30000 cho 30s
bool gCaptureRequested = false;

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

uint8_t clampToByte(int value) {
  return static_cast<uint8_t>(constrain(value, 0, 255));
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

float adaptiveEdgeThreshold(float meanStrength) {
  const float meanClamped = constrain(meanStrength, 0.10f, 0.18f);
  const float t = (meanClamped - 0.10f) / 0.08f;
  return gEdgeThresholdDark + ((gEdgeThresholdBright - gEdgeThresholdDark) * t);
}

void smoothHistogram(const uint16_t* src, uint16_t* dst, int x0, int x1) {
  for (int x = x0; x <= x1; ++x) {
    const uint16_t left = src[max(x0, x - 1)];
    const uint16_t mid = src[x];
    const uint16_t right = src[min(x1, x + 1)];
    dst[x] = static_cast<uint16_t>((left + (2 * mid) + right) / 4);
  }
}

void resetBinaryCandidate(BinaryCandidate* candidate) {
  candidate->pixelCount = 0;
  candidate->centerCount = 0;
  candidate->minX = kMaxWidth;
  candidate->minY = 32767;
  candidate->maxX = -1;
  candidate->maxY = -1;
}

void updateBinaryCandidate(BinaryCandidate* candidate, int x, int y, bool inCenterBand) {
  ++candidate->pixelCount;
  if (inCenterBand) {
    ++candidate->centerCount;
  }
  if (x < candidate->minX) candidate->minX = x;
  if (x > candidate->maxX) candidate->maxX = x;
  if (y < candidate->minY) candidate->minY = y;
  if (y > candidate->maxY) candidate->maxY = y;
}

float candidateAreaRatio(const BinaryCandidate& candidate, uint32_t roiPixels) {
  if (roiPixels == 0) {
    return 0.0f;
  }
  return static_cast<float>(candidate.pixelCount) / static_cast<float>(roiPixels);
}

bool candidateIsUsable(const BinaryCandidate& candidate, uint32_t roiPixels) {
  if (candidate.maxX < candidate.minX || candidate.maxY < candidate.minY) {
    return false;
  }
  const int width = candidate.maxX - candidate.minX + 1;
  const int height = candidate.maxY - candidate.minY + 1;
  const float areaRatio = candidateAreaRatio(candidate, roiPixels);
  return width >= kMinHandWidth &&
         height >= kMinHandHeight &&
         areaRatio >= kBinaryAreaMinRatio &&
         areaRatio <= kBinaryAreaMaxRatio;
}

float candidateScore(const BinaryCandidate& candidate, uint32_t roiPixels) {
  if (!candidateIsUsable(candidate, roiPixels)) {
    return -1.0f;
  }

  const int width = candidate.maxX - candidate.minX + 1;
  const int height = candidate.maxY - candidate.minY + 1;
  const float areaRatio = candidateAreaRatio(candidate, roiPixels);
  const float centerRatio =
      candidate.pixelCount > 0 ? static_cast<float>(candidate.centerCount) / candidate.pixelCount : 0.0f;
  const float fillRatio =
      static_cast<float>(candidate.pixelCount) / static_cast<float>(max(1, width * height));
  const float targetArea = 0.18f;
  const float areaScore = 1.0f - min(1.0f, fabsf(areaRatio - targetArea) / targetArea);
  const float fillPenalty = max(0.0f, fillRatio - 0.88f);
  return (centerRatio * 1.6f) + areaScore - fillPenalty;
}

uint8_t computeOtsuThreshold(const uint32_t* histogram, uint32_t totalPixels) {
  if (totalPixels == 0) {
    return 96;
  }

  uint64_t sum = 0;
  for (int i = 0; i < 256; ++i) {
    sum += static_cast<uint64_t>(i) * histogram[i];
  }

  uint64_t sumBackground = 0;
  uint32_t weightBackground = 0;
  float maxVariance = -1.0f;
  uint8_t bestThreshold = 96;

  for (int t = 0; t < 256; ++t) {
    weightBackground += histogram[t];
    if (weightBackground == 0) {
      continue;
    }

    const uint32_t weightForeground = totalPixels - weightBackground;
    if (weightForeground == 0) {
      break;
    }

    sumBackground += static_cast<uint64_t>(t) * histogram[t];
    const float meanBackground = static_cast<float>(sumBackground) / weightBackground;
    const float meanForeground = static_cast<float>(sum - sumBackground) / weightForeground;
    const float variance =
        static_cast<float>(weightBackground) * static_cast<float>(weightForeground) *
        (meanBackground - meanForeground) * (meanBackground - meanForeground);

    if (variance > maxVariance) {
      maxVariance = variance;
      bestThreshold = static_cast<uint8_t>(t);
    }
  }

  return bestThreshold;
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

bool isOpenPalmCandidate(
    int minX,
    int minY,
    int maxX,
    int maxY,
    int roiX0,
    int roiX1,
    int roiY0,
    int roiY1,
    float edgeDensity,
    int rawFingerCount) {
  if (rawFingerCount < kOpenPalmMinRawPeaks || maxX < minX || maxY < minY) {
    return false;
  }

  const int handWidth = maxX - minX + 1;
  const int handHeight = maxY - minY + 1;
  const int roiWidth = roiX1 - roiX0;
  const int roiHeight = roiY1 - roiY0;

  return edgeDensity >= kOpenPalmEdgeDensityThreshold &&
         handWidth >= ((roiWidth * 3) / 4) &&
         handHeight >= ((roiHeight * 3) / 4) &&
         minY <= (roiY0 + (roiHeight / 5)) &&
         maxY >= (roiY0 + ((roiHeight * 7) / 10));
}

GestureClass classifySimpleGesture(
    bool handDetected,
    int minX,
    int minY,
    int maxX,
    int maxY,
    int roiX0,
    int roiX1,
    int roiY0,
    int roiY1,
    float edgeDensity) {
  if (!handDetected || maxX < minX || maxY < minY) {
    return kGestureNone;
  }

  const int handWidth = maxX - minX + 1;
  const int handHeight = maxY - minY + 1;
  const int roiWidth = max(1, roiX1 - roiX0);
  const int roiHeight = max(1, roiY1 - roiY0);
  const float areaRatio =
      static_cast<float>(handWidth * handHeight) / static_cast<float>(roiWidth * roiHeight);

  if (edgeDensity >= 0.080f &&
      areaRatio >= 0.220f &&
      handWidth >= (roiWidth / 2) &&
      handHeight >= (roiHeight / 2)) {
    return kGestureOpen;
  }

  return kGestureFist;
}

const char* gestureName(GestureClass gesture) {
  if (gesture == kGestureOpen) {
    return "open";
  }
  if (gesture == kGestureFist) {
    return "fist";
  }
  return "none";
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
    sensor->set_brightness(sensor, gBrightness);
    sensor->set_contrast(sensor, gContrast);
    sensor->set_saturation(sensor, -1);
    sensor->set_gain_ctrl(sensor, 1);
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_aec2(sensor, 1);
    sensor->set_ae_level(sensor, 1);
    sensor->set_gainceiling(sensor, GAINCEILING_32X);
    sensor->set_whitebal(sensor, 1);
    sensor->set_hmirror(sensor, 1);
    sensor->set_vflip(sensor, 1);
  }

  Serial.println("[OK] Camera OV2640 sẵn sàng (320x240 GRAYSCALE)");
  return true;
}

void printConfigStatus() {
  Serial.printf(
      "CONFIG_STATUS,quality=%d,brightness=%d,contrast=%d,edge_dark=%.3f,edge_bright=%.3f,hand_threshold=%.3f,capture_ms=%lu\n",
      gCaptureJpegQuality,
      gBrightness,
      gContrast,
      gEdgeThresholdDark,
      gEdgeThresholdBright,
      gHandEdgeDensityThreshold,
      static_cast<unsigned long>(gCaptureIntervalMs));
}

void applySensorConfig() {
  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    Serial.println("[WARN] Sensor not ready for config update");
    return;
  }

  sensor->set_brightness(sensor, gBrightness);
  sensor->set_contrast(sensor, gContrast);
  sensor->set_quality(sensor, gCaptureJpegQuality);
  printConfigStatus();
}

void applyConfigCommand(const String& line) {
  int quality = gCaptureJpegQuality;
  int brightness = gBrightness;
  int contrast = gContrast;
  float edgeDark = gEdgeThresholdDark;
  float edgeBright = gEdgeThresholdBright;
  float handThreshold = gHandEdgeDensityThreshold;
  uint32_t captureIntervalMs = gCaptureIntervalMs;

  int start = 0;
  while (start < line.length()) {
    int end = line.indexOf(' ', start);
    if (end < 0) {
      end = line.length();
    }

    const String token = line.substring(start, end);
    const int eq = token.indexOf('=');
    if (eq > 0) {
      const String key = token.substring(0, eq);
      const String valueStr = token.substring(eq + 1);
      const int value = valueStr.toInt();

      if (key == "quality") {
        quality = constrain(value, 10, 40);
      } else if (key == "brightness") {
        brightness = constrain(value, -2, 2);
      } else if (key == "contrast") {
        contrast = constrain(value, -2, 2);
      } else if (key == "edge_dark") {
        edgeDark = constrain(valueStr.toFloat(), 0.05f, 0.80f);
      } else if (key == "edge_bright") {
        edgeBright = constrain(valueStr.toFloat(), 0.05f, 0.80f);
      } else if (key == "hand_threshold") {
        handThreshold = constrain(valueStr.toFloat(), 0.005f, 0.200f);
      } else if (key == "capture_ms") {
        captureIntervalMs = static_cast<uint32_t>(constrain(value, 0, 60000));
      }
    }

    start = end + 1;
  }

  gCaptureJpegQuality = quality;
  gBrightness = brightness;
  gContrast = contrast;
  gEdgeThresholdDark = edgeDark;
  gEdgeThresholdBright = edgeBright;
  gHandEdgeDensityThreshold = handThreshold;
  gCaptureIntervalMs = captureIntervalMs;
  applySensorConfig();
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      gSerialCommandBuffer.trim();
      if (gSerialCommandBuffer.startsWith("SET ")) {
        applyConfigCommand(gSerialCommandBuffer.substring(4));
      } else if (gSerialCommandBuffer == "GET_CONFIG") {
        printConfigStatus();
      } else if (gSerialCommandBuffer == "SNAP") {
        gCaptureRequested = true;
        Serial.println("SNAP_ACK");
      }
      gSerialCommandBuffer = "";
      continue;
    }

    if (gSerialCommandBuffer.length() < 220) {
      gSerialCommandBuffer += ch;
    } else {
      gSerialCommandBuffer = "";
    }
  }
}

void turnOffFlashLed() {
  pinMode(kFlashLedGpio, OUTPUT);
  digitalWrite(kFlashLedGpio, LOW);
}

void initFlashLedPwm() {
  // ESP32 Arduino core 3.x API: ledcAttach(pin, freq, resolution_bits)
  ledcAttach(kFlashLedGpio, kFlashLedFreqHz, kFlashLedResBits);
  ledcWrite(kFlashLedGpio, 0);
}

void flashLedOn() {
  ledcWrite(kFlashLedGpio, kFlashCaptureDuty);
}

void flashLedOff() {
  ledcWrite(kFlashLedGpio, 0);
}

// Khi sap capture: bo fb hien tai, bat LED, doi warmup, lay fb moi sang ro
camera_fb_t* grabFlashCaptureFrame(camera_fb_t* currentFb) {
  if (currentFb != nullptr) {
    esp_camera_fb_return(currentFb);
  }
  flashLedOn();
  delay(kFlashWarmupMs);
  // Bo 1-2 frame stale tu queue de AE adapt
  camera_fb_t* tmp = esp_camera_fb_get();
  if (tmp != nullptr) {
    esp_camera_fb_return(tmp);
  }
  camera_fb_t* fresh = esp_camera_fb_get();
  return fresh;
}

void makeCapturePath(uint32_t index, char* out, size_t outLen) {
  snprintf(out, outLen, "%s/capture_%06lu.jpg", kCaptureDir, static_cast<unsigned long>(index));
}

uint32_t findNextCaptureIndex() {
  char path[64];
  for (uint32_t index = 1; index < 1000000; ++index) {
    makeCapturePath(index, path, sizeof(path));
    if (!SD_MMC.exists(path)) {
      return index;
    }
  }
  return 1;
}

bool initCaptureStorage() {
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  delay(200);

  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[WARN] SD_MMC 1-bit mount failed, retrying 4-bit mode");
    SD_MMC.end();
    delay(200);

    if (!SD_MMC.begin("/sdcard", false)) {
      Serial.println("[WARN] SD_MMC mount failed, capture image saving disabled");
      Serial.println("[HINT] Kiem tra the microSD FAT32, tiep xuc the, va tranh dung LED flash GPIO4 khi mount SD");
      return false;
    }
  }

  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("[WARN] No SD card attached, capture image saving disabled");
    return false;
  }

  if (!SD_MMC.exists(kCaptureDir)) {
    SD_MMC.mkdir(kCaptureDir);
  }

  if (!SD_MMC.exists(kCaptureLogPath)) {
    File logFile = SD_MMC.open(kCaptureLogPath, FILE_WRITE);
    if (logFile) {
      logFile.println("capture_ms,path,edge_density,mean_strength,hand_detected,finger_count,min_x,min_y,max_x,max_y,frame_time_ms,frame_counter");
      logFile.close();
    }
  }

  gNextCaptureIndex = findNextCaptureIndex();
  Serial.printf("[OK] SD capture storage ready, next=%lu\n", static_cast<unsigned long>(gNextCaptureIndex));
  return true;
}

void appendCaptureCsv(const char* path, const FuzzyMetrics& metrics, uint32_t captureMs) {
  if (!gSdReady || path == nullptr) {
    return;
  }

  File logFile = SD_MMC.open(kCaptureLogPath, FILE_APPEND);
  if (!logFile) {
    Serial.println("[WARN] Cannot append capture CSV");
    return;
  }

  logFile.printf(
      "%lu,%s,%.6f,%.6f,%d,%d,%d,%d,%d,%d,%lu,%lu\n",
      static_cast<unsigned long>(captureMs),
      path,
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
  logFile.close();
}

bool saveCaptureImage(camera_fb_t* fb, const FuzzyMetrics& metrics, uint32_t captureMs, char* savedPath, size_t savedPathLen) {
  if (!gSdReady || fb == nullptr || savedPath == nullptr || savedPathLen == 0) {
    return false;
  }

  uint8_t* jpgBuf = nullptr;
  size_t jpgLen = 0;
  if (!frame2jpg(fb, gCaptureJpegQuality, &jpgBuf, &jpgLen)) {
    Serial.println("[WARN] Capture JPEG encode failed");
    return false;
  }

  makeCapturePath(gNextCaptureIndex, savedPath, savedPathLen);
  File imageFile = SD_MMC.open(savedPath, FILE_WRITE);
  if (!imageFile) {
    Serial.printf("[WARN] Cannot create capture image: %s\n", savedPath);
    free(jpgBuf);
    return false;
  }

  const size_t written = imageFile.write(jpgBuf, jpgLen);
  imageFile.close();
  free(jpgBuf);

  if (written != jpgLen) {
    Serial.printf("[WARN] Capture image write incomplete: %s\n", savedPath);
    return false;
  }

  appendCaptureCsv(savedPath, metrics, captureMs);
  ++gNextCaptureIndex;
  return true;
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

void updateWindowStats(const FuzzyMetrics& metrics, uint32_t nowMs) {
  if (gWindowStats.startMs == 0) {
    resetWindowStats(nowMs);
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

void printBase64Chunk(const uint8_t* data, size_t len) {
  static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  char out[5];
  out[4] = '\0';

  for (size_t i = 0; i < len; i += 3) {
    const uint32_t a = data[i];
    const uint32_t b = (i + 1 < len) ? data[i + 1] : 0;
    const uint32_t c = (i + 2 < len) ? data[i + 2] : 0;
    const uint32_t triple = (a << 16) | (b << 8) | c;

    out[0] = alphabet[(triple >> 18) & 0x3F];
    out[1] = alphabet[(triple >> 12) & 0x3F];
    out[2] = (i + 1 < len) ? alphabet[(triple >> 6) & 0x3F] : '=';
    out[3] = (i + 2 < len) ? alphabet[triple & 0x3F] : '=';
    Serial.print(out);
  }
}

void printCaptureBase64(camera_fb_t* fb, uint32_t captureMs) {
  if (fb == nullptr) {
    return;
  }

  uint8_t* jpgBuf = nullptr;
  size_t jpgLen = 0;
  if (!frame2jpg(fb, gCaptureJpegQuality, &jpgBuf, &jpgLen)) {
    Serial.println("[WARN] Capture JPEG encode failed for serial base64");
    return;
  }

  Serial.printf("CAPTURE_JPG_BASE64_BEGIN,time_ms=%lu,len=%lu\n",
                static_cast<unsigned long>(captureMs),
                static_cast<unsigned long>(jpgLen));
  printBase64Chunk(jpgBuf, jpgLen);
  Serial.println();
  Serial.println("CAPTURE_JPG_BASE64_END");
  free(jpgBuf);
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
      "WINDOW_LOG,start_ms=%lu,end_ms=%lu,samples=%lu,avg_edge=%.6f,avg_mean=%.6f,hand_ratio=%.4f,dominant_gesture=%s,gesture_code=%u,capture=%s,capture_ms=%lu\n",
      static_cast<unsigned long>(gWindowStats.startMs),
      static_cast<unsigned long>(nowMs),
      static_cast<unsigned long>(gWindowStats.sampleCount),
      avgEdge,
      avgMean,
      handRatio,
      gestureName(static_cast<GestureClass>(dominantFinger)),
      dominantFinger,
      gWindowStats.captureOccurred ? "yes" : "no",
      static_cast<unsigned long>(gWindowStats.captureMs));
}

void printCaptureLog(const FuzzyMetrics& metrics, uint32_t captureMs, const char* imagePath) {
  Serial.printf(
      "CAPTURE_LOG,time_ms=%lu,path=%s,edge_density=%.6f,mean_strength=%.6f,hand=%d,fingers=%d,gesture=%s,bbox=%d,%d,%d,%d,frame_time_ms=%lu,frame=%lu\n",
      static_cast<unsigned long>(captureMs),
      imagePath != nullptr ? imagePath : "none",
      metrics.edgeDensity,
      metrics.meanStrength,
      metrics.handDetected ? 1 : 0,
      metrics.fingerCount,
      gestureName(static_cast<GestureClass>(metrics.fingerCount)),
      metrics.minX,
      metrics.minY,
      metrics.maxX,
      metrics.maxY,
      static_cast<unsigned long>(metrics.frameTimeMs),
      static_cast<unsigned long>(metrics.frameCounter));
}

void printDebugStatus(const FuzzyMetrics& metrics, uint32_t nowMs) {
  if ((nowMs - gLastDebugStatusMs) < kDebugStatusIntervalMs) {
    return;
  }
  gLastDebugStatusMs = nowMs;

  Serial.printf(
      "DEBUG_STATUS,time_ms=%lu,edge_density=%.6f,mean_strength=%.6f,hand=%d,gesture=%s,code=%d,bbox=%d,%d,%d,%d\n",
      static_cast<unsigned long>(nowMs),
      metrics.edgeDensity,
      metrics.meanStrength,
      metrics.handDetected ? 1 : 0,
      gestureName(static_cast<GestureClass>(metrics.fingerCount)),
      metrics.fingerCount,
      metrics.minX,
      metrics.minY,
      metrics.maxX,
      metrics.maxY);
}

void updateMetricsFromFrame(camera_fb_t* fb, uint32_t frameTimeMs) {
  if (fb == nullptr || fb->format != PIXFORMAT_GRAYSCALE || fb->width > kMaxWidth || fb->height < 3) {
    return;
  }

  const int roiX0 = fb->width / 5;
  const int roiX1 = fb->width - roiX0;
  const int roiY0 = fb->height / 6;
  const int roiY1 = fb->height - roiY0;
  const int centerX0 = roiX0 + ((roiX1 - roiX0) / 4);
  const int centerX1 = roiX1 - ((roiX1 - roiX0) / 4);
  const int centerY0 = roiY0 + ((roiY1 - roiY0) / 4);
  const int centerY1 = roiY1 - ((roiY1 - roiY0) / 4);
  const int upperBandY = roiY0 + ((roiY1 - roiY0) * 11) / 20;

  uint32_t grayHistogram[256] = {0};
  uint64_t graySum = 0;
  uint32_t grayPixels = 0;
  for (int y = roiY0; y < roiY1; ++y) {
    const uint8_t* row = fb->buf + (y * fb->width);
    for (int x = roiX0; x < roiX1; ++x) {
      const uint8_t gray = row[x];
      ++grayHistogram[gray];
      graySum += gray;
      ++grayPixels;
    }
  }

  const uint8_t otsuThreshold = computeOtsuThreshold(grayHistogram, grayPixels);
  const uint8_t darkThreshold = clampToByte(static_cast<int>(otsuThreshold) - kGrayThresholdMargin);
  const uint8_t brightThreshold = clampToByte(static_cast<int>(otsuThreshold) + (kGrayThresholdMargin / 2));

  loadGrayLine(fb, 0, linePrev);
  loadGrayLine(fb, 1, lineCurr);
  const float edgeThresholdAdaptive = adaptiveEdgeThreshold(gMeanStrengthFiltered);

  uint32_t edgeCount = 0;
  uint32_t roiPixels = 0;
  float sumStrength = 0.0f;
  BinaryCandidate darkCandidate;
  BinaryCandidate brightCandidate;
  resetBinaryCandidate(&darkCandidate);
  resetBinaryCandidate(&brightCandidate);
  memset(colEdgeCount, 0, sizeof(colEdgeCount));
  memset(colEdgeCountTop, 0, sizeof(colEdgeCountTop));
  memset(colDarkCountTop, 0, sizeof(colDarkCountTop));
  memset(colBrightCountTop, 0, sizeof(colBrightCountTop));

  for (int y = 1; y < fb->height - 1; ++y) {
    loadGrayLine(fb, y + 1, lineNext);

    if (y >= roiY0 && y < roiY1) {
      for (int x = max(1, roiX0); x < min(static_cast<int>(fb->width) - 1, roiX1); ++x) {
        const float edgeStrength = fuzzyEdgeFromDelta(lineCurr[x], linePrev, lineCurr, lineNext, x);
        const bool inCenterBand = x >= centerX0 && x < centerX1 && y >= centerY0 && y < centerY1;
        const bool darkForeground = lineCurr[x] <= darkThreshold;
        const bool brightForeground = lineCurr[x] >= brightThreshold;
        sumStrength += edgeStrength;
        ++roiPixels;

        if (edgeStrength >= edgeThresholdAdaptive) {
          ++edgeCount;
          ++colEdgeCount[x];
          if (y <= upperBandY) {
            ++colEdgeCountTop[x];
          }
        }

        if (darkForeground) {
          updateBinaryCandidate(&darkCandidate, x, y, inCenterBand);
          if (y <= upperBandY) {
            ++colDarkCountTop[x];
          }
        }

        if (brightForeground) {
          updateBinaryCandidate(&brightCandidate, x, y, inCenterBand);
          if (y <= upperBandY) {
            ++colBrightCountTop[x];
          }
        }
      }
    }

    memcpy(linePrev, lineCurr, fb->width);
    memcpy(lineCurr, lineNext, fb->width);
  }

  const float edgeDensity = roiPixels > 0 ? static_cast<float>(edgeCount) / roiPixels : 0.0f;
  const float meanStrength = roiPixels > 0 ? sumStrength / roiPixels : 0.0f;
  const float meanGray = grayPixels > 0 ? static_cast<float>(graySum) / grayPixels : 0.0f;

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

  const bool useDarkCandidate =
      candidateScore(darkCandidate, roiPixels) >= candidateScore(brightCandidate, roiPixels);
  const BinaryCandidate& chosenCandidate = useDarkCandidate ? darkCandidate : brightCandidate;
  const uint16_t* chosenTopHistogram = useDarkCandidate ? colDarkCountTop : colBrightCountTop;
  const float chosenAreaRatio = candidateAreaRatio(chosenCandidate, roiPixels);
  const int minX = chosenCandidate.minX;
  const int minY = chosenCandidate.minY;
  const int maxX = chosenCandidate.maxX;
  const int maxY = chosenCandidate.maxY;
  const int handWidth = maxX >= minX ? (maxX - minX + 1) : 0;
  const int handHeight = maxY >= minY ? (maxY - minY + 1) : 0;
  const float bboxFillRatio =
      (handWidth > 0 && handHeight > 0)
          ? static_cast<float>(chosenCandidate.pixelCount) / static_cast<float>(handWidth * handHeight)
          : 0.0f;
  const bool handCandidate =
      maxX >= 0 &&
      handWidth >= kMinHandWidth &&
      handHeight >= kMinHandHeight &&
      (handHeight * 10) >= (handWidth * 9) &&
      chosenAreaRatio >= kBinaryAreaMinRatio &&
      bboxFillRatio >= 0.18f &&
      (gEdgeDensityFiltered >= gHandEdgeDensityThreshold || fabsf(meanGray - static_cast<float>(otsuThreshold)) >= 18.0f);

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
  int rawFingerCount = 0;
  GestureClass gesture = kGestureNone;

  if (handDetected && maxX >= minX && maxY >= minY) {
    const int dynamicPeakGap = max(10, handWidth / 8);
    const int dynamicPeakHeight = max(5, handHeight / 10);
    const int dynamicPeakProminence = max(4, handHeight / 10);

    smoothHistogram(chosenTopHistogram, colEdgeCountSmooth, minX, maxX);
    for (int x = minX; x <= maxX; ++x) {
      if (colEdgeCountSmooth[x] < dynamicPeakHeight) {
        colEdgeCountSmooth[x] = 0;
      }
    }

    rawFingerCount = estimateFingerCount(
        colEdgeCountSmooth,
        minX,
        maxX,
        dynamicPeakGap,
        dynamicPeakHeight,
        dynamicPeakProminence);
    stabilizeFingerCount(static_cast<uint8_t>(rawFingerCount));

    if (isOpenPalmCandidate(
            minX,
            minY,
            maxX,
            maxY,
            roiX0,
            roiX1,
            roiY0,
            roiY1,
            gEdgeDensityFiltered,
            rawFingerCount)) {
      gesture = kGestureOpen;
    }
  } else {
    resetFingerHistory();
  }

  if (gesture != kGestureOpen) {
    const int roiWidth = max(1, roiX1 - roiX0);
    const int roiHeight = max(1, roiY1 - roiY0);
    const float bboxAreaRatio =
        static_cast<float>(max(1, handWidth) * max(1, handHeight)) / static_cast<float>(roiWidth * roiHeight);
    if (handDetected &&
        rawFingerCount >= 2 &&
        chosenAreaRatio >= 0.10f &&
        chosenAreaRatio <= 0.38f &&
        bboxAreaRatio >= 0.28f &&
        bboxFillRatio <= 0.64f &&
        handWidth >= ((roiWidth * 2) / 3) &&
        handHeight >= (roiHeight / 2) &&
        minY <= (roiY0 + (roiHeight / 3))) {
      gesture = kGestureOpen;
    } else {
      gesture = handDetected ? kGestureFist : kGestureNone;
    }
  }

  gMetrics.edgeDensity = gEdgeDensityFiltered;
  gMetrics.meanStrength = gMeanStrengthFiltered;
  gMetrics.handDetected = handDetected;
  gMetrics.fingerCount = static_cast<int>(gesture);
  const bool hasCurrentBox = maxX >= minX && maxY >= minY;
  gMetrics.minX = (handDetected && hasCurrentBox) ? minX : -1;
  gMetrics.minY = (handDetected && hasCurrentBox) ? minY : -1;
  gMetrics.maxX = (handDetected && hasCurrentBox) ? maxX : -1;
  gMetrics.maxY = (handDetected && hasCurrentBox) ? maxY : -1;
  gMetrics.frameTimeMs = frameTimeMs;
  gMetrics.frameCounter += 1;
  gMetrics.lastUpdateMs = millis();
  gMetrics.frameWidth = fb->width;
  gMetrics.frameHeight = fb->height;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32-CAM Local Fuzzy Analyzer ===");

  if (!initCamera()) {
    while (true) {
      delay(1000);
    }
  }

  initFlashLedPwm();
  if (kEnableSdCaptureStorage) {
    gSdReady = initCaptureStorage();
  } else {
    gSdReady = false;
    Serial.println("[INFO] SD capture storage disabled, capture image only when SNAP is requested");
  }

  Serial.print("LOGCSV_HEADER,");
  Serial.println("window_start_ms,window_end_ms,samples,avg_edge,avg_mean,hand_ratio,gesture_code,capture,capture_ms");
  printConfigStatus();
}

void loop() {
  handleSerialCommands();
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
    updateWindowStats(gMetrics, nowMs);
    printDebugStatus(gMetrics, nowMs);

    const bool shouldCapture =
        gCaptureRequested ||
        (gCaptureIntervalMs > 0 &&
         (gLastPeriodicCaptureMs == 0 || (nowMs - gLastPeriodicCaptureMs) >= gCaptureIntervalMs));
    if (shouldCapture) {
      gCaptureRequested = false;
      gLastPeriodicCaptureMs = nowMs;
      gWindowStats.captureOccurred = true;
      gWindowStats.captureMs = nowMs;
      gWindowStats.captureMetrics = gMetrics;

      // Bat LED, lay frame moi sang ro de gui ve PC
      camera_fb_t* flashFb = grabFlashCaptureFrame(fb);
      fb = nullptr;  // da return trong grabFlashCaptureFrame
      const uint32_t captureMs = millis();

      char capturePath[64] = "none";
      if (flashFb != nullptr) {
        if (saveCaptureImage(flashFb, gMetrics, captureMs, capturePath, sizeof(capturePath))) {
          Serial.printf("CAPTURE_FILE,%s\n", capturePath);
        }
        printCaptureLog(gMetrics, captureMs, capturePath);
        if (kAlwaysDumpCaptureBase64 || strcmp(capturePath, "none") == 0) {
          printCaptureBase64(flashFb, captureMs);
        }
        esp_camera_fb_return(flashFb);
      } else {
        Serial.println("[WARN] Flash capture fb null, skip dump");
        printCaptureLog(gMetrics, captureMs, capturePath);
      }
      flashLedOff();
    }

    if (gWindowStats.startMs > 0 && (nowMs - gWindowStats.startMs) >= kSummaryWindowMs) {
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
  }

  if (fb != nullptr) {
    esp_camera_fb_return(fb);
  }
  delay(kLoopDelayMs);
}
