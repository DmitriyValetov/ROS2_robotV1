/***********************************************************************
 *  Просмотр показаний MPU-6500 в мониторе порта.
 *
 *  Задача скетча — просто показать, что датчик живой и что он выдаёт.
 *  Никакой калибровки, фильтров и команд: прошили, открыли монитор
 *  порта на 115200 и смотрите.
 *
 *  Подключение (ESP32 DevKit):
 *    - SDA -> GPIO 21
 *    - SCL -> GPIO 22
 *    - VCC -> 3.3V
 *    - GND -> GND
 *
 *  Что означает вывод:
 *    Accel (g)  — ускорение по осям, в покое одна из осей даёт ±1.00 g
 *                 (та, что смотрит вниз или вверх — видно по знаку)
 *    Gyro (dps) — угловая скорость, град/с; в покое около нуля,
 *                 у этого чипа смещение нуля до нескольких dps — норма
 *    pitch/roll — углы наклона, посчитанные по вектору гравитации
 *    T          — температура самого чипа (греется сам, 40+ °C нормально)
 *
 *  Как проверить за 10 секунд:
 *    1. Положите модуль плашмя на стол — по оси Z должно быть ≈±1.00 g,
 *       по X и Y — около нуля.
 *    2. Поднимите на ребро — «единица» переедет на другую ось.
 *    3. Поверните модуль рукой — гироскоп покажет десятки и сотни dps
 *       по оси вращения.
 *
 *  Если нужно ловить удары и резкие движения, поднимите шкалу
 *  акселерометра до ±16g: REG_ACCEL_CONFIG = 0x18, ACCEL_LSB_PER_G = 2048.
 *
 *  (C) 2025
 ***********************************************************************/

#include <Wire.h>

#define IMU_ADDR         0x68   // AD0 = GND (при AD0 = VCC будет 0x69)

// Регистры (карта MPU-6000/6050/6500/9250)
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B   // далее 14 байт: AccX..Z, T, GyroX..Z
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define PWR_WAKE         0x01   // снять SLEEP, тактирование от PLL по X-гироскопу
#define ACCEL_FS_2G      0x00   // AFS_SEL = 0
#define GYRO_FS_250DPS   0x00   // FS_SEL = 0

// Чувствительность для выбранных шкал: сырые единицы на g и на °/с
const float ACCEL_LSB_PER_G   = 16384.0;   // ±2g
const float GYRO_LSB_PER_DPS  = 131.0;     // ±250 dps

// --- низкоуровневый доступ к регистрам ---
void writeReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg)
{
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0xFF;
  }
  if (Wire.requestFrom(IMU_ADDR, (uint8_t)1) != 1) {
    return 0xFF;
  }
  return Wire.read();
}

// Все 14 байт данных читаем одним запросом — так они гарантированно из одного замера
bool readAll(int16_t *accel, int16_t *gyro, int16_t *temp)
{
  uint8_t b[14];
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(IMU_ADDR, (uint8_t)14) != 14) {
    return false;
  }
  for (uint8_t i = 0; i < 14; i++) {
    b[i] = Wire.read();
  }

  // Данные идут старшим байтом вперёд
  accel[0] = (int16_t)((b[0] << 8) | b[1]);
  accel[1] = (int16_t)((b[2] << 8) | b[3]);
  accel[2] = (int16_t)((b[4] << 8) | b[5]);
  *temp    = (int16_t)((b[6] << 8) | b[7]);
  gyro[0]  = (int16_t)((b[8] << 8) | b[9]);
  gyro[1]  = (int16_t)((b[10] << 8) | b[11]);
  gyro[2]  = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);   // чтобы не потерять первые строки при открытии монитора

  Wire.begin(21, 22, 400000);
  delay(100);

  Serial.println();
  Serial.println("=== MPU-6500: показания датчика ===");

  uint8_t id = readReg(REG_WHO_AM_I);
  Serial.printf("WHO_AM_I = 0x%02X ", id);
  if (id == 0x70)      Serial.println("(MPU-6500)");
  else if (id == 0x71) Serial.println("(MPU-9250)");
  else if (id == 0x68) Serial.println("(MPU-6050/6000)");
  else                 Serial.println("(неизвестный ID - проверьте подключение)");

  writeReg(REG_PWR_MGMT_1, 0x80);   // сброс
  delay(100);
  writeReg(REG_PWR_MGMT_1, PWR_WAKE);
  delay(50);
  writeReg(REG_ACCEL_CONFIG, ACCEL_FS_2G);
  writeReg(0x1B, GYRO_FS_250DPS);   // REG_GYRO_CONFIG
  delay(50);

  Serial.println("Шкалы: акселерометр +/-2g, гироскоп +/-250 dps");
  Serial.println("Формат: t,мс | Accel X,Y,Z (g) | Gyro X,Y,Z (dps) | pitch,roll | T (C)");
  Serial.println("В покое на плоской поверхности одна из осей Accel дает ~1.00 g");
  Serial.println();
}

void loop()
{
  int16_t accel[3], gyro[3], tempRaw;

  if (!readAll(accel, gyro, &tempRaw)) {
    Serial.println("Ошибка чтения по I2C: проверьте SDA=21, SCL=22, питание 3.3 В");
    delay(1000);
    return;
  }

  float ax = accel[0] / ACCEL_LSB_PER_G;
  float ay = accel[1] / ACCEL_LSB_PER_G;
  float az = accel[2] / ACCEL_LSB_PER_G;
  float gx = gyro[0] / GYRO_LSB_PER_DPS;
  float gy = gyro[1] / GYRO_LSB_PER_DPS;
  float gz = gyro[2] / GYRO_LSB_PER_DPS;
  float tempC = tempRaw / 340.0 + 36.53;

  // Углы по вектору гравитации. atan2 сохраняет знак и различает
  // «плашмя» и «вверх дном» — по acos этого не видно.
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float roll  = atan2(ay, az) * 180.0 / PI;

  Serial.printf("%6lu | Accel (g) X=%+.3f Y=%+.3f Z=%+.3f | Gyro (dps) X=%+.2f Y=%+.2f Z=%+.2f | "
                "pitch=%+.1f roll=%+.1f | T=%.1f C\n",
                (unsigned long)millis(), ax, ay, az, gx, gy, gz, pitch, roll, tempC);

  delay(100);   // ~10 строк в секунду, удобно читать глазами
}
