#include <Wire.h>

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Инициализация I²C:
  // первый аргумент — SDA, второй — SCL
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println();
  Serial.println("I2C scanner started");
  Serial.printf("SDA = GPIO %d, SCL = GPIO %d\n", SDA_PIN, SCL_PIN);
}

void loop() {
  uint8_t error;
  int devicesFound = 0;

  Serial.println("\nScanning I2C bus...");

  // Адреса I²C — 7-битные: от 0x01 до 0x7E.
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("Device found at 0x%02X", address);

      if (address == 0x68 || address == 0x69) {
        Serial.print("  <-- possible ICM-20948");
      }

      Serial.println();
      devicesFound++;
    } else if (error == 4) {
      Serial.printf("Unknown I2C error at 0x%02X\n", address);
    }
  }

  if (devicesFound == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.printf("Scan complete: %d device(s) found\n", devicesFound);
  }

  delay(3000);
}