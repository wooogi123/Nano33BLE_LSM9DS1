/*
   Nano33BLE_IMU setMagneticFieldODR

   This example change magnetometer output data rate(ODR).
   Magnetometer ODR can be choose 0.625, 1.25, 2.5, 5, 10, 20, 40, 80

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

  Serial.print("Magnetometer ODR: ");
  Serial.println(IMU.magneticFieldSampleRate());
}

void loop() {
  if (IMU.magneticFieldAvailable()) {
    IMU.setMagneticFieldODR("80");
    Serial.print("Magnetometer ODR: ");
    Serial.println(IMU.magneticFieldSampleRate());
  }
}
