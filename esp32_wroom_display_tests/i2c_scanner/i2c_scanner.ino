/*
 * ESP32-WROOM-32 I2C scanner.
 *
 * Use this first to find OLED/LCD I2C address.
 * Wiring default:
 *   SDA -> GPIO21
 *   SCL/SCK -> GPIO22
 *   GND -> GND
 *   VCC/VDD -> 3V3 for OLED, LCD may need 5V depending on module.
 */

#include <Arduino.h>
#include <Wire.h>

constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr uint32_t I2C_CLOCK_HZ = 100000;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("ESP32-WROOM I2C scanner");
  Serial.printf("SDA=%d SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
}

void loop() {
  int count = 0;
  Serial.println("Scanning I2C bus...");

  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    const uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("Found device at 0x%02X\n", addr);
      ++count;
    }
  }

  if (count == 0) {
    Serial.println("No I2C device found. Check wiring, power, SDA/SCL pins.");
  } else {
    Serial.printf("Done. Found %d device(s).\n", count);
    Serial.println("Common: OLED SSD1306 = 0x3C/0x3D, LCD16x2 = 0x27/0x3F");
  }
  Serial.println();
  delay(3000);
}
