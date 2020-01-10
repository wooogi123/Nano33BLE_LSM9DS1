/*
   Nano33BLE_IMU setGyroscopeScale

   This example change gyroscope scale.
   Gyroscope scale can be choose 245, 500, 2000

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

  Serial.print("Gyroscope scale: ");
  Serial.println(IMU.gyroscopeScale());
}

void loop() {
  if (IMU.gyroscopeAvailable()) {
    IMU.setGyroscopeScale(245);
    Serial.print("Gyroscope scale: ");
    Serial.println(IMU.gyroscopeScale());
  }
}
