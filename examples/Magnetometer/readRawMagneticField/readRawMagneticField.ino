/*
   Nano33BLE_IMU readRawMagneticField

   This example reads magnetometer raw datas.

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

  Serial.println("Magnetometer(gauss)");
  Serial.println("X\tY\tZ");
}

void loop() {
  int16_t x, y, z;
  if (IMU.magneticFieldAvailable()) {
    IMU.readRawMagneticField(x, y, z);
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.println(z);
  }
}
