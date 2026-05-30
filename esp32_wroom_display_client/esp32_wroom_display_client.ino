/*
 * ESP32-WROOM-32 display client for ESP32-CAM PC FUZZY.
 *
 * Flow:
 *   ESP32-CAM -> PC Python server -> this ESP32-WROOM -> LCD/OLED display
 *
 * Configure WIFI_SSID, WIFI_PASS, and DISPLAY_API_URL below.
 * DISPLAY_API_URL must use the real PC LAN IP, not 127.0.0.1.
 */

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <Wire.h>

#define DISPLAY_OLED 1
#define DISPLAY_LCD  2

// ===== User config =====
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// Example: "http://192.168.1.10:5000/api/display"
const char* DISPLAY_API_URL = "http://192.168.1.10:5000/api/display";

// Choose DISPLAY_OLED or DISPLAY_LCD.
#define DISPLAY_TYPE DISPLAY_OLED

constexpr uint32_t POLL_INTERVAL_MS = 700;
constexpr uint32_t WIFI_RETRY_MS = 10000;
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;

// OLED I2C SSD1306: GND, VDD, SCK/SCL, SDA. Common address 0x3C.
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr int OLED_WIDTH = 128;
constexpr int OLED_HEIGHT = 64;

// LCD 16x2 I2C backpack: GND, VCC, SDA, SCL. Common address 0x27 or 0x3F.
constexpr uint8_t LCD_ADDR = 0x27;
constexpr int LCD_COLS = 16;
constexpr int LCD_ROWS = 2;
// =======================

#if DISPLAY_TYPE == DISPLAY_OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
#elif DISPLAY_TYPE == DISPLAY_LCD
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
#else
  #error "DISPLAY_TYPE must be DISPLAY_OLED or DISPLAY_LCD"
#endif

struct DisplayData {
  bool ok = false;
  bool running = false;
  String error = "";
  String gesture = "none";
  int fingers = 0;
  float confidence = 0.0f;
  String mode = "";
  uint32_t frameCount = 0;
  uint32_t ageMs = 0;
};

uint32_t gLastPollMs = 0;
uint32_t gLastWifiTryMs = 0;

String upperCopy(String value) {
  value.toUpperCase();
  return value;
}

String fitText(String value, size_t maxLen) {
  value.trim();
  if (value.length() <= maxLen) {
    return value;
  }
  if (maxLen <= 1) {
    return value.substring(0, maxLen);
  }
  return value.substring(0, maxLen - 1) + "~";
}

bool jsonBool(const String& json, const String& key, bool fallback) {
  const String token = "\"" + key + "\":";
  int p = json.indexOf(token);
  if (p < 0) return fallback;
  p += token.length();
  while (p < (int)json.length() && isspace(json[p])) ++p;
  if (json.substring(p, p + 4) == "true") return true;
  if (json.substring(p, p + 5) == "false") return false;
  return fallback;
}

float jsonNumber(const String& json, const String& key, float fallback) {
  const String token = "\"" + key + "\":";
  int p = json.indexOf(token);
  if (p < 0) return fallback;
  p += token.length();
  while (p < (int)json.length() && isspace(json[p])) ++p;
  int e = p;
  while (e < (int)json.length()) {
    const char c = json[e];
    if (!isdigit(c) && c != '-' && c != '+' && c != '.') break;
    ++e;
  }
  if (e <= p) return fallback;
  return json.substring(p, e).toFloat();
}

String jsonString(const String& json, const String& key, const String& fallback) {
  const String token = "\"" + key + "\":";
  int p = json.indexOf(token);
  if (p < 0) return fallback;
  p += token.length();
  while (p < (int)json.length() && isspace(json[p])) ++p;
  if (p >= (int)json.length() || json[p] != '"') return fallback;
  ++p;
  String out;
  while (p < (int)json.length()) {
    const char c = json[p++];
    if (c == '"') break;
    if (c == '\\' && p < (int)json.length()) {
      out += json[p++];
    } else {
      out += c;
    }
  }
  return out;
}

void drawBoot(const String& line1, const String& line2 = "") {
#if DISPLAY_TYPE == DISPLAY_OLED
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ESP32 Display");
  display.println(line1);
  if (line2.length()) display.println(line2);
  display.display();
#else
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(fitText(line1, 16));
  lcd.setCursor(0, 1);
  lcd.print(fitText(line2, 16));
#endif
}

void drawData(const DisplayData& data) {
  const int confPct = constrain((int)roundf(data.confidence * 100.0f), 0, 100);
  const String gesture = upperCopy(data.gesture.length() ? data.gesture : "none");

#if DISPLAY_TYPE == DISPLAY_OLED
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("WiFi ");
  display.print(WiFi.localIP());

  display.setTextSize(2);
  display.setCursor(0, 14);
  display.print(fitText(gesture, 10));

  display.setTextSize(1);
  display.setCursor(0, 38);
  display.print("fingers: ");
  display.print(data.fingers);
  display.print("  conf: ");
  display.print(confPct);
  display.println("%");

  display.setCursor(0, 50);
  if (!data.ok) {
    display.print("ERR ");
    display.print(fitText(data.error.length() ? data.error : "waiting", 18));
  } else {
    display.print("mode ");
    display.print(fitText(data.mode, 15));
  }
  display.display();
#else
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(fitText(gesture + " f=" + String(data.fingers), 10));
  lcd.print(" ");
  lcd.print(confPct);
  lcd.print("%");

  lcd.setCursor(0, 1);
  if (!data.ok) {
    lcd.print(fitText(data.error.length() ? data.error : "waiting", 16));
  } else {
    lcd.print("frame ");
    lcd.print(data.frameCount);
  }
#endif
}

bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;

  const uint32_t now = millis();
  if (now - gLastWifiTryMs < WIFI_RETRY_MS) return false;
  gLastWifiTryMs = now;

  drawBoot("WiFi connecting", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  const uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < WIFI_RETRY_MS) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    drawBoot("WiFi OK", WiFi.localIP().toString());
    return true;
  }

  drawBoot("WiFi failed", "retrying...");
  return false;
}

bool fetchDisplayData(DisplayData& out) {
  if (WiFi.status() != WL_CONNECTED) {
    out.ok = false;
    out.error = "wifi lost";
    return false;
  }

  HTTPClient http;
  http.setTimeout(2500);
  if (!http.begin(DISPLAY_API_URL)) {
    out.ok = false;
    out.error = "bad url";
    return false;
  }

  const int code = http.GET();
  if (code != HTTP_CODE_OK) {
    out.ok = false;
    out.error = "http " + String(code);
    http.end();
    return false;
  }

  const String body = http.getString();
  http.end();

  out.ok = jsonBool(body, "ok", false);
  out.running = jsonBool(body, "running", false);
  out.error = jsonString(body, "error", "");
  out.gesture = jsonString(body, "gesture", "none");
  out.fingers = (int)jsonNumber(body, "fingers", 0);
  out.confidence = jsonNumber(body, "confidence", 0.0f);
  out.mode = jsonString(body, "mode", "");
  out.frameCount = (uint32_t)jsonNumber(body, "frame_count", 0);
  out.ageMs = (uint32_t)jsonNumber(body, "age_ms", 0);
  return true;
}

void setupDisplay() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
#if DISPLAY_TYPE == DISPLAY_OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (true) delay(1000);
  }
  display.clearDisplay();
  display.display();
#else
  lcd.init();
  lcd.backlight();
#endif
}

void setup() {
  Serial.begin(115200);
  delay(300);
  setupDisplay();
  drawBoot("Booting...", "display client");
  connectWiFi();
}

void loop() {
  if (!connectWiFi()) {
    delay(250);
    return;
  }

  const uint32_t now = millis();
  if (now - gLastPollMs < POLL_INTERVAL_MS) {
    delay(20);
    return;
  }
  gLastPollMs = now;

  DisplayData data;
  fetchDisplayData(data);
  drawData(data);

  Serial.printf(
      "gesture=%s fingers=%d conf=%.2f ok=%d err=%s\n",
      data.gesture.c_str(), data.fingers, data.confidence, data.ok ? 1 : 0,
      data.error.c_str());
}
