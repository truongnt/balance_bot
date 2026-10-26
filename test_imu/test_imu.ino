/*
 * TEST IMU — Xác định đúng trục MPU-6050
 * Mở Serial Monitor 115200 baud
 * Nghiêng robot về phía TRƯỚC và xem cột nào thay đổi nhiều nhất
 */
#include <Wire.h>
#define MPU_ADDR 0x68

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
  Serial.println(F("ax\tay\taz\t|\tgx\tgy\tgz"));
  Serial.println(F("---\t---\t---\t|\t---\t---\t---"));
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  int16_t ax = Wire.read()<<8|Wire.read();
  int16_t ay = Wire.read()<<8|Wire.read();
  int16_t az = Wire.read()<<8|Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = Wire.read()<<8|Wire.read();
  int16_t gy = Wire.read()<<8|Wire.read();
  int16_t gz = Wire.read()<<8|Wire.read();

  // Chia 100 cho dễ đọc
  Serial.print(ax/100); Serial.print(F("\t"));
  Serial.print(ay/100); Serial.print(F("\t"));
  Serial.print(az/100); Serial.print(F("\t|\t"));
  Serial.print(gx/100); Serial.print(F("\t"));
  Serial.print(gy/100); Serial.print(F("\t"));
  Serial.println(gz/100);

  delay(200);
}
