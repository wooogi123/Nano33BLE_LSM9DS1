/*
   Nano33BLE_IMU readRawGyroscope

   This example reads gyroscope raw datas.

   Created to 26 Nov 2019
 */

#include <Nano33BLE_IMU.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to started IMU");
    while (1);
  }

  Serial.println("Gyroscope(dps)");
  Serial.println("X\tY\tZ");
}

void loop() {
  int16_t x, y, z;
  if (IMU.gyroscopeAvailable()) {
    IMU.readRawGyroscope(x, y, z);
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.println(z);
  }
}
