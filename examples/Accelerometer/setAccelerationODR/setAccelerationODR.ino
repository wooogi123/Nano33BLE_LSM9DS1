/*
   Nano33BLE_IMU setAccelerationODR

   This example change accelerometer output data rate(ODR).
   Accelerometer ODR can be choose 10, 50, 119, 238, 476, 952

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

  Serial.print("Accelerometer ODR: ");
  Serial.println(IMU.accelerationSampleRate());
}

void loop() {
  if (IMU.accelerationAvailable()) {
    IMU.setAccelerationODR("952");
    Serial.print("Accelerometer ODR: ");
    Serial.println(IMU.accelerationSampleRate());
  }
}
