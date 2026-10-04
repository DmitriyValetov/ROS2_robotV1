/***********************************************************************
 *  Кто на самом деле стоит на модуле IMU?
 *
 *  Скетч для случая «WHO_AM_I вернул не то, что ждали». Ничего не
 *  настраивает и не пишет в датчик — только читает идентификаторы,
 *  чтобы понять, какой чип распаян.
 *
 *  Зачем: у ICM-20948 и у ICM-42688-P РАЗНАЯ карта регистров.
 *  У ICM-20948 WHO_AM_I лежит в регистре 0x00 (банк 0), у 42688 — в 0x75.
 *  Поэтому старый код, читающий 0x00, получает у 42688 случайный мусор
 *  (например 0xBD), и это НЕ значит, что датчик неисправен.
 *
 *  Подключение (ESP32 DevKit):
 *    - SDA -> GPIO 21
 *    - SCL -> GPIO 22
 *    - VCC -> 3.3V  (модули ICM не терпят 5 В на логике!)
 *    - GND -> GND
 *
 *  Что смотреть в мониторе порта (115200):
 *    "ICM-42688-P"  -> у вас 6-осевой датчик, нужен драйвер ICM-42688
 *    "ICM-20948"    -> датчик тот, ищите проблему в своём коде
 *    "неизвестный ID" -> пришлите строку, разберём
 *
 *  (C) 2025
 ***********************************************************************/

#include <Wire.h>
#include <string.h>

#define IMU_ADDR_LOW   0x68   // AD0 = GND
#define IMU_ADDR_HIGH  0x69   // AD0 = VCC

// --- регистры ICM-20948 (9 осей, с магнитометром) ---
#define ICM20948_REG_BANK_SEL 0x7F
#define ICM20948_WHO_AM_I     0x00   // в банке 0
#define ICM20948_ID           0xEA

// --- регистры ICM-42688-P / ICM-42686-P (6 осей) ---
#define ICM42688_WHO_AM_I     0x75   // банк 0, отдельного селектора банков не требует
#define ICM42688_ID           0x47
#define ICM42686_ID           0x44

// --- регистры семейства MPU-6000/6050/6500/9250 (6 или 9 осей) ---
// У них WHO_AM_I тоже лежит в 0x75, поэтому проверяется тем же чтением.
#define MPU_WHO_AM_I          0x75
#define MPU_ID_6000           0x68   // MPU-6000
#define MPU_ID_6050           0x68   // MPU-6050 (тот же ID)
#define MPU_ID_6500           0x70   // MPU-6500
#define MPU_ID_9250           0x71   // MPU-9250 (9 осей, с магнитометром AK8963)
#define MPU_ID_9255           0x73   // MPU-9255
#define MPU_ID_ICM20648       0xE1   // ICM-20648
#define MPU_ID_ICM20602       0x12   // ICM-20602

bool readReg(uint8_t addr, uint8_t reg, uint8_t *value)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) {
    return false;
  }
  *value = Wire.read();
  return true;
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

const char *identify(uint8_t addr)
{
  uint8_t id20948 = 0, id42688 = 0;
  bool ok20948 = readReg(addr, ICM20948_WHO_AM_I, &id20948);
  bool ok42688 = readReg(addr, ICM42688_WHO_AM_I, &id42688);

  Serial.printf("Модуль 0x%02X:\n", addr);
  Serial.printf("  регистр 0x00 (ICM-20948, банк 0) = %s\n",
                ok20948 ? String("0x" + String(id20948, HEX)).c_str() : "нет ответа");
  Serial.printf("  регистр 0x75 (ICM-42688 / MPU)    = %s\n",
                ok42688 ? String("0x" + String(id42688, HEX)).c_str() : "нет ответа");

  if (ok42688) {
    switch (id42688) {
      case ICM42688_ID:
        Serial.println("  → ЭТО ICM-42688-P (6 осей, без магнитометра)");
        return "ICM-42688-P";
      case ICM42686_ID:
        Serial.println("  → ЭТО ICM-42686-P (6 осей, без магнитометра)");
        return "ICM-42686-P";
      case MPU_ID_6500:
        Serial.println("  → ЭТО MPU-6500 (6 осей, без магнитометра)");
        return "MPU-6500";
      case MPU_ID_9250:
        Serial.println("  → ЭТО MPU-9250 (9 осей, магнитометр AK8963)");
        return "MPU-9250";
      case MPU_ID_9255:
        Serial.println("  → ЭТО MPU-9255 (9 осей)");
        return "MPU-9255";
      case MPU_ID_ICM20648:
        Serial.println("  → ЭТО ICM-20648 (6 осей)");
        return "ICM-20648";
      case MPU_ID_ICM20602:
        Serial.println("  → ЭТО ICM-20602 (6 осей)");
        return "ICM-20602";
      case MPU_ID_6000:
        Serial.println("  → ЭТО MPU-6000/MPU-6050 (6 осей)");
        return "MPU-6050";
      default:
        Serial.printf("  → неизвестный ID 0x%02X\n", id42688);
        return "unknown";
    }
  }
  if (ok20948 && id20948 == ICM20948_ID) {
    // у настоящего 20948 в регистре 0x75 обычно 0x00 или 0xFF
    Serial.println("  → ЭТО ICM-20948 (9 осей)");
    return "ICM-20948";
  }
  Serial.println("  → не отвечает");
  return "unknown";
}

void scanBus()
{
  Serial.println("Сканирование шины I2C:");
  uint8_t found = 0;
  for (uint8_t addr = 0x03; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  устройство по адресу 0x%02X\n", addr);
      found++;
    }
  }
  if (!found) {
    Serial.println("  пусто: проверьте питание (3.3 В!), SDA=21, SCL=22 и подтяжки");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1500);   // чтобы ESP32 успел поднять USB-CDC и вы не потеряли вывод
  Wire.begin(21, 22, 400000);
  delay(100);

  Serial.println();
  Serial.println("=== Определение модели IMU ===");

  scanBus();

  Serial.println();
  const char *chipLow  = identify(IMU_ADDR_LOW);
  const char *chipHigh = identify(IMU_ADDR_HIGH);

  Serial.println();
  if (strcmp(chipLow, "unknown") == 0 && strcmp(chipHigh, "unknown") == 0) {
    Serial.println("ИТОГ: датчик не опознан. Пришлите вывод выше.");
  } else if (strcmp(chipLow, "ICM-20948") == 0 || strcmp(chipHigh, "ICM-20948") == 0) {
    Serial.println("ИТОГ: на модуле ICM-20948 — железо совпадает с курсом.");
  } else if (strcmp(chipLow, "MPU-6500") == 0 || strcmp(chipHigh, "MPU-6500") == 0) {
    Serial.println("ИТОГ: на модуле MPU-6500 (6 осей, без магнитометра).");
    Serial.println("      Это НЕ ICM-20948: другая карта регистров.");
    Serial.println("      Рабочий скетч: test_imu_mpu6500.ino");
  } else if (strcmp(chipLow, "MPU-9250") == 0 || strcmp(chipHigh, "MPU-9250") == 0) {
    Serial.println("ИТОГ: на модуле MPU-9250 (9 осей, есть магнитометр AK8963).");
    Serial.println("      Рабочий скетч: test_imu_mpu6500.ino (6 осей).");
  } else {
    Serial.println("ИТОГ: на модуле 6-осевой InvenSense (ICM-42688-P/42686-P).");
    Serial.println("      Код и библиотеки для ICM-20948 с ним НЕ работают:");
    Serial.println("      другая карта регистров, другой WHO_AM_I, нет магнитометра.");
    Serial.println("      Рабочий скетч: test_imu_42688.ino");
  }
}

void loop()
{
  delay(10000);
  Serial.println("--- повтор ---");
  identify(IMU_ADDR_LOW);
  identify(IMU_ADDR_HIGH);
}
