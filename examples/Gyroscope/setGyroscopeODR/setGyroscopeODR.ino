/*
   Nano33BLE_IMU setGyroscopeODR

   This example change gyroscope output data rate(ODR).
   Gyroscope ODR can be choose 14.9, 59.5, 119, 238, 476, 952

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

  Serial.print("Gyroscope ODR: ");
  Serial.println(IMU.gyroscopeSampleRate());
}

void loop() {
  if (IMU.gyroscopeAvailable()) {
    IMU.setGyroscopeODR("952");
    Serial.print("Gyroscope ODR: ");
    Serial.println(IMU.gyroscopeSampleRate());
  }
}
