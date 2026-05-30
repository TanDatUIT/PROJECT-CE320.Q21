/*
 * ESP32-WROOM-32 LCD 16x2 I2C test.
 *
 * LCD I2C backpack pins: GND, VCC, SDA, SCL
 *   LCD GND -> ESP32 GND
 *   LCD VCC -> 5V or 3V3 depending on module
 *   LCD SDA -> ESP32 GPIO21
 *   LCD SCL -> ESP32 GPIO22
 *
 * Warning: many LCD backpacks use 5V pull-ups on SDA/SCL.
 * ESP32 GPIO is 3.3V. A level shifter is safest.
 *
 * Arduino IDE library:
 *   - LiquidCrystal I2C
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr uint8_t LCD_ADDR = 0x27;  // Try 0x3F if scanner reports 0x3F.
constexpr int LCD_COLS = 16;
constexpr int LCD_ROWS = 2;

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

void printFixedLine(uint8_t row, const String& text) {
  lcd.setCursor(0, row);
  String out = text;
  if (out.length() > LCD_COLS) {
    out = out.substring(0, LCD_COLS);
  }
  while (out.length() < LCD_COLS) {
    out += ' ';
  }
  lcd.print(out);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("ESP32-WROOM LCD 16x2 I2C test");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  lcd.init();
  lcd.backlight();

  printFixedLine(0, "ESP32-WROOM OK");
  printFixedLine(1, "LCD I2C 0x" + String(LCD_ADDR, HEX));
  Serial.println("LCD init sent. If screen is blank, adjust contrast potentiometer.");
  delay(2000);
}

void loop() {
  static uint32_t counter = 0;
  printFixedLine(0, "LCD 16x2 TEST");
  printFixedLine(1, "Count: " + String(counter++));
  Serial.printf("LCD counter=%lu\n", static_cast<unsigned long>(counter));
  delay(1000);
}
