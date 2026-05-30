/*
 * ESP32-WROOM-32 OLED SSD1306 I2C test.
 *
 * OLED pins on your module: GND, VDD, SCK, SDA
 *   OLED GND -> ESP32 GND
 *   OLED VDD -> ESP32 3V3
 *   OLED SCK -> ESP32 GPIO22
 *   OLED SDA -> ESP32 GPIO21
 *
 * Arduino IDE libraries:
 *   - Adafruit SSD1306
 *   - Adafruit GFX Library
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr uint8_t OLED_ADDR = 0x3C;  // Try 0x3D if scanner reports 0x3D.
constexpr int OLED_WIDTH = 128;
constexpr int OLED_HEIGHT = 64;

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

void drawFrame(uint32_t counter) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ESP32-WROOM-32");
  display.println("OLED SSD1306 OK");

  display.setTextSize(2);
  display.setCursor(0, 24);
  display.print("CNT ");
  display.println(counter);

  display.setTextSize(1);
  display.setCursor(0, 52);
  display.print("SDA21 SCL22 0x");
  display.print(OLED_ADDR, HEX);

  const int barWidth = (counter * 8) % OLED_WIDTH;
  display.drawRect(0, 45, OLED_WIDTH, 5, SSD1306_WHITE);
  display.fillRect(0, 45, barWidth, 5, SSD1306_WHITE);
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("ESP32-WROOM OLED SSD1306 test");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed. Check address 0x3C/0x3D, wiring, and power.");
    while (true) {
      delay(1000);
    }
  }

  display.clearDisplay();
  display.display();
  Serial.println("OLED init OK");
}

void loop() {
  static uint32_t counter = 0;
  drawFrame(counter++);
  Serial.printf("OLED counter=%lu\n", static_cast<unsigned long>(counter));
  delay(1000);
}
