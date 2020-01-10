/*
   Nano33BLE_IMU readAccelGyro

   This exmaple reads accelerometer & gyroscope values.

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

  Serial.println("Accelerometer(g),\tGyroscope(dps)");
  Serial.println("AX\tAY\tAZ\t\tGX\tGY\tGZ");
}

void loop() {
  float accel[3] = { 0, 0, 0 };
  float gyro[3] = { 0, 0, 0 };
  if (IMU.accelgyroAvailable()) {
    IMU.readAccelGyro(accel, gyro, 3, 3);
    for (int i = 0; i < 3; ++i) {
      Serial.print(accel[i]);
      Serial.print("\t");
    }
    Serial.print("\t");
    for (int i = 0; i < 3; ++i) {
      Serial.print(gyro[i]);
      Serial.print("\t");
    }
    Serial.println();
  }
}
