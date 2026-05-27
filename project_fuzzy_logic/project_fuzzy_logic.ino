#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"
#include "FS.h"
#include "SD_MMC.h"

// Chan ket noi camera AI Thinker ESP32-CAM.
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

namespace {
constexpr framesize_t kFrameSize = FRAMESIZE_QQVGA;
constexpr pixformat_t kPixelFormat = PIXFORMAT_GRAYSCALE;
constexpr uint32_t kXclkHz = 20000000;
constexpr int kMaxWidth = 160;
constexpr int kJpegQuality = 18;
constexpr int kDefaultBrightness = -1;
constexpr int kDefaultContrast = 2;
constexpr float kDefaultEdgeThresholdDark = 0.32f;
constexpr float kDefaultEdgeThresholdBright = 0.24f;
constexpr float kDefaultHandEdgeDensityThreshold = 0.035f;
constexpr float kOpenPalmEdgeDensityThreshold = 0.12f;
constexpr uint32_t kLoopDelayMs = 40;
constexpr uint32_t kTelemetryIntervalMs = 200;
constexpr uint32_t kDefaultSnapshotIntervalMs = 0;
constexpr bool kEnableSdLogging = false;
constexpr char kLogFilePath[] = "/fuzzy_log.csv";
constexpr char kLogHeader[] =
    "time_ms,edge_density,mean_strength,hand_detected,finger_count,min_x,min_y,max_x,max_y,frame_time_ms";
constexpr float kEdgeDensityAlpha = 0.25f;
constexpr float kMeanStrengthAlpha = 0.20f;
constexpr int kMinHandWidth = 18;
constexpr int kMinHandHeight = 18;
constexpr uint8_t kHandOnFrames = 3;
constexpr uint8_t kHandOffFrames = 5;
constexpr uint8_t kFingerHistorySize = 5;
constexpr uint8_t kOpenPalmMinRawPeaks = 3;

enum GestureClass : uint8_t {
  kGestureNone = 0,
  kGestureFist = 1,
  kGestureOpen = 5,
};

uint8_t linePrev[kMaxWidth];
uint8_t lineCurr[kMaxWidth];
uint8_t lineNext[kMaxWidth];
uint16_t colEdgeCount[kMaxWidth];
uint16_t colEdgeCountTop[kMaxWidth];
uint16_t colEdgeCountSmooth[kMaxWidth];
bool gSdReady = false;
bool gLogHeaderReady = false;
float gEdgeDensityFiltered = 0.0f;
float gMeanStrengthFiltered = 0.0f;
bool gFilterPrimed = false;
bool gHandLatched = false;
uint8_t gHandOnStreak = 0;
uint8_t gHandOffStreak = 0;
uint8_t gFingerHistory[kFingerHistorySize] = {0};
uint8_t gFingerHistoryCount = 0;
uint8_t gFingerHistoryIndex = 0;
String gSerialCommandBuffer;
int gBrightness = kDefaultBrightness;
int gContrast = kDefaultContrast;
int gCaptureJpegQuality = kJpegQuality;
float gEdgeThresholdDark = kDefaultEdgeThresholdDark;
float gEdgeThresholdBright = kDefaultEdgeThresholdBright;
float gHandEdgeDensityThreshold = kDefaultHandEdgeDensityThreshold;
uint32_t gSnapshotIntervalMs = kDefaultSnapshotIntervalMs;
uint32_t gLastSnapshotMs = 0;
uint32_t gLastTelemetryMs = 0;
bool gSnapshotRequested = false;

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

float fuzzyEdgeFromDelta(
    uint8_t center,
    const uint8_t* up,
    const uint8_t* mid,
    const uint8_t* down,
    int x) {
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
  config.jpeg_quality = gCaptureJpegQuality;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
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
    sensor->set_ae_level(sensor, -2);
    sensor->set_gainceiling(sensor, GAINCEILING_8X);
    sensor->set_whitebal(sensor, 1);
    sensor->set_hmirror(sensor, 0);
    sensor->set_vflip(sensor, 0);
  }

  Serial.println("Camera ready: 160x120 GRAYSCALE");
  return true;
}

void printConfigStatus() {
  Serial.printf(
      "CONFIG_STATUS,quality=%d,brightness=%d,contrast=%d,edge_dark=%.3f,edge_bright=%.3f,hand_threshold=%.3f,snapshot_ms=%lu\n",
      gCaptureJpegQuality,
      gBrightness,
      gContrast,
      gEdgeThresholdDark,
      gEdgeThresholdBright,
      gHandEdgeDensityThreshold,
      static_cast<unsigned long>(gSnapshotIntervalMs));
}

void applySensorConfig() {
  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    Serial.println("Sensor not ready for config update");
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
  uint32_t snapshotIntervalMs = gSnapshotIntervalMs;

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
      const float value = valueStr.toFloat();

      if (key == "quality") {
        quality = constrain(static_cast<int>(value), 10, 40);
      } else if (key == "brightness") {
        brightness = constrain(static_cast<int>(value), -2, 2);
      } else if (key == "contrast") {
        contrast = constrain(static_cast<int>(value), -2, 2);
      } else if (key == "edge_dark") {
        edgeDark = constrain(value, 0.18f, 0.55f);
      } else if (key == "edge_bright") {
        edgeBright = constrain(value, 0.16f, 0.50f);
      } else if (key == "hand_threshold") {
        handThreshold = constrain(value, 0.010f, 0.120f);
      } else if (key == "snapshot_ms") {
        snapshotIntervalMs = static_cast<uint32_t>(constrain(value, 0.0f, 60000.0f));
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
  gSnapshotIntervalMs = snapshotIntervalMs;
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
        gSnapshotRequested = true;
        Serial.println("CAPTURE_STATUS,requested");
      }
      gSerialCommandBuffer = "";
      continue;
    }

    if (gSerialCommandBuffer.length() < 160) {
      gSerialCommandBuffer += ch;
    } else {
      gSerialCommandBuffer = "";
    }
  }
}

bool initLogStorage() {
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD_MMC mount failed");
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return false;
  }

  return true;
}

bool ensureLogHeader() {
  if (!gSdReady) {
    return false;
  }
  if (gLogHeaderReady) {
    return true;
  }

  if (!SD_MMC.exists(kLogFilePath)) {
    File file = SD_MMC.open(kLogFilePath, FILE_WRITE);
    if (!file) {
      Serial.println("Cannot create log file");
      return false;
    }
    file.println(kLogHeader);
    file.close();
  }

  gLogHeaderReady = true;
  return true;
}

void emitSerialCsvLine(
    uint32_t nowMs,
    float edgeDensity,
    float meanStrength,
    bool handLikelyPresent,
    int fingerCount,
    int minX,
    int minY,
    int maxX,
    int maxY,
    uint32_t frameTimeMs) {
  char line[192];
  snprintf(
      line,
      sizeof(line),
      "LOGCSV,%lu,%.6f,%.6f,%d,%d,%d,%d,%d,%d,%lu",
      static_cast<unsigned long>(nowMs),
      edgeDensity,
      meanStrength,
      handLikelyPresent ? 1 : 0,
      fingerCount,
      minX,
      minY,
      maxX,
      maxY,
      static_cast<unsigned long>(frameTimeMs));
  Serial.println(line);
}

void appendLogLine(
    uint32_t nowMs,
    float edgeDensity,
    float meanStrength,
    bool handLikelyPresent,
    int fingerCount,
    int minX,
    int minY,
    int maxX,
    int maxY,
    uint32_t frameTimeMs) {
  if (!ensureLogHeader()) {
    return;
  }

  File file = SD_MMC.open(kLogFilePath, FILE_APPEND);
  if (!file) {
    Serial.println("Cannot append log file");
    return;
  }

  char line[192];
  snprintf(
      line,
      sizeof(line),
      "%lu,%.6f,%.6f,%d,%d,%d,%d,%d,%d,%lu",
      static_cast<unsigned long>(nowMs),
      edgeDensity,
      meanStrength,
      handLikelyPresent ? 1 : 0,
      fingerCount,
      minX,
      minY,
      maxX,
      maxY,
      static_cast<unsigned long>(frameTimeMs));
  file.println(line);
  file.close();
}

int estimateFingerCount(
    const uint16_t* histogram,
    int x0,
    int x1,
    int minPeakGap,
    int minPeakHeight,
    int minPeakProminence) {
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

    if (!isLocalMax || (mid - valley) < minPeakProminence) {
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

void printBase64Chunk(const uint8_t* data, size_t len) {
  static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  char out[5];
  out[4] = '\0';
  uint16_t lineChars = 0;

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
    lineChars += 4;
    if (lineChars >= 128) {
      Serial.println();
      lineChars = 0;
    }
  }
}

void printCaptureBase64(camera_fb_t* fb, uint32_t captureMs) {
  if (fb == nullptr) {
    return;
  }

  uint8_t* jpgBuf = nullptr;
  size_t jpgLen = 0;
  if (!frame2jpg(fb, gCaptureJpegQuality, &jpgBuf, &jpgLen)) {
    Serial.println("CAPTURE_STATUS,error=jpeg_encode_failed");
    return;
  }

  Serial.printf(
      "CAPTURE_STATUS,begin,time_ms=%lu,len=%lu\n",
      static_cast<unsigned long>(captureMs),
      static_cast<unsigned long>(jpgLen));
  Serial.printf(
      "CAPTURE_JPG_BASE64_BEGIN,time_ms=%lu,len=%lu\n",
      static_cast<unsigned long>(captureMs),
      static_cast<unsigned long>(jpgLen));
  printBase64Chunk(jpgBuf, jpgLen);
  Serial.println();
  Serial.println("CAPTURE_JPG_BASE64_END");
  Serial.println("CAPTURE_STATUS,done");
  free(jpgBuf);
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32-CAM fuzzy delta edge detector");
  Serial.printf("PSRAM found: %s\n", psramFound() ? "yes" : "no");
  Serial.print("LOGCSV_HEADER,");
  Serial.println(kLogHeader);

  if (kEnableSdLogging) {
    gSdReady = initLogStorage();
  } else {
    gSdReady = false;
    Serial.println("SD logging: disabled, PC dashboard still writes CSV");
  }
  Serial.printf("SD logging: %s\n", gSdReady ? "enabled" : "disabled");

  if (!initCamera()) {
    Serial.println("Camera setup failed, stopping.");
    while (true) {
      delay(1000);
    }
  }
  printConfigStatus();
}

void loop() {
  handleSerialCommands();
  const uint32_t t0 = millis();
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb == nullptr) {
    Serial.println("Camera capture failed");
    delay(500);
    return;
  }

  if (fb->format != PIXFORMAT_GRAYSCALE || fb->width > kMaxWidth || fb->height < 3) {
    Serial.printf(
        "Unexpected frame format/size: format=%d %ux%u\n",
        fb->format,
        fb->width,
        fb->height);
    esp_camera_fb_return(fb);
    delay(500);
    return;
  }

  loadGrayLine(fb, 0, linePrev);
  loadGrayLine(fb, 1, lineCurr);

  const int roiX0 = fb->width / 4;
  const int roiX1 = fb->width - roiX0;
  const int roiY0 = fb->height / 5;
  const int roiY1 = fb->height - roiY0;
  const int upperBandY = roiY0 + ((roiY1 - roiY0) * 11) / 20;
  const float edgeThresholdAdaptive = adaptiveEdgeThreshold(gMeanStrengthFiltered);

  uint32_t edgeCount = 0;
  uint32_t roiPixels = 0;
  float sumStrength = 0.0f;
  int minX = fb->width;
  int minY = fb->height;
  int maxX = -1;
  int maxY = -1;
  memset(colEdgeCount, 0, sizeof(colEdgeCount));
  memset(colEdgeCountTop, 0, sizeof(colEdgeCountTop));

  for (int y = 1; y < fb->height - 1; ++y) {
    loadGrayLine(fb, y + 1, lineNext);

    if (y >= roiY0 && y < roiY1) {
      for (int x = max(1, roiX0); x < min(static_cast<int>(fb->width) - 1, roiX1); ++x) {
        const float edgeStrength = fuzzyEdgeFromDelta(lineCurr[x], linePrev, lineCurr, lineNext, x);
        sumStrength += edgeStrength;
        ++roiPixels;

        if (edgeStrength >= edgeThresholdAdaptive) {
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
      gEdgeDensityFiltered >= gHandEdgeDensityThreshold &&
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

  const bool handLikelyPresent = gHandLatched;
  int rawFingerCount = 0;
  GestureClass gesture = kGestureNone;

  if (handLikelyPresent && maxX >= minX && maxY >= minY) {
    const int dynamicPeakGap = max(8, handWidth / 7);
    const int dynamicPeakHeight = max(4, handHeight / 9);
    const int dynamicPeakProminence = max(3, handHeight / 10);

    smoothHistogram(colEdgeCountTop, colEdgeCountSmooth, minX, maxX);
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
    gesture = classifySimpleGesture(
        handLikelyPresent,
        minX,
        minY,
        maxX,
        maxY,
        roiX0,
        roiX1,
        roiY0,
        roiY1,
        gEdgeDensityFiltered);
  }
  const int gestureCode = static_cast<int>(gesture);

  const uint32_t nowMs = millis();
  if (gSnapshotRequested ||
      (gSnapshotIntervalMs > 0 &&
       (gLastSnapshotMs == 0 || (nowMs - gLastSnapshotMs) >= gSnapshotIntervalMs))) {
    gSnapshotRequested = false;
    gLastSnapshotMs = nowMs;
    printCaptureBase64(fb, nowMs);
  }
  esp_camera_fb_return(fb);

  const bool shouldEmitTelemetry =
      gLastTelemetryMs == 0 || (nowMs - gLastTelemetryMs) >= kTelemetryIntervalMs;
  if (shouldEmitTelemetry) {
    gLastTelemetryMs = nowMs;
    Serial.printf(
        "edge_density=%.6f mean_strength=%.6f threshold=%.3f hand=%s gesture=%s code=%d raw_peaks=%d bbox=%d,%d,%d,%d time_ms=%lu\n",
        gEdgeDensityFiltered,
        gMeanStrengthFiltered,
        edgeThresholdAdaptive,
        handLikelyPresent ? "yes" : "no",
        gestureName(gesture),
        gestureCode,
        rawFingerCount,
        handLikelyPresent ? minX : -1,
        handLikelyPresent ? minY : -1,
        handLikelyPresent ? maxX : -1,
        handLikelyPresent ? maxY : -1,
        static_cast<unsigned long>(nowMs - t0));

    emitSerialCsvLine(
        nowMs,
        gEdgeDensityFiltered,
        gMeanStrengthFiltered,
        handLikelyPresent,
        gestureCode,
        handLikelyPresent ? minX : -1,
        handLikelyPresent ? minY : -1,
        handLikelyPresent ? maxX : -1,
        handLikelyPresent ? maxY : -1,
        nowMs - t0);

    appendLogLine(
        nowMs,
        gEdgeDensityFiltered,
        gMeanStrengthFiltered,
        handLikelyPresent,
        gestureCode,
        handLikelyPresent ? minX : -1,
        handLikelyPresent ? minY : -1,
        handLikelyPresent ? maxX : -1,
        handLikelyPresent ? maxY : -1,
        nowMs - t0);
  }

  delay(kLoopDelayMs);
}
