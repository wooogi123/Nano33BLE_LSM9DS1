/*
   Nano33BLE_IMU setAccelerationScale

   This example change accelerometer scale.
   Accelerometer scale can be choose 2, 4, 8, 16.

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

  Serial.print("Accelerometer scale: ");
  Serial.println(IMU.accelerationScale());
}

void loop() {
  if (IMU.accelerationAvailable()) {
    IMU.setAccelerationScale(2);
    Serial.print("Accelerometer scale: ");
    Serial.println(IMU.accelerationScale());
  }
}
