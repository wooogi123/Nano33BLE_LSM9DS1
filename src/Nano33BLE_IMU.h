/*
 * Nano33BLE_IMU.h
 *
 * Environment specifics:
 *     IDE: Arduino 1.8.10
 *     Platform: Arduino Nano 33 BLE
 */

#ifndef Nano33BLE_IMU_H
#define Nano33BLE_IMU_H

#include <Arduino.h>
#include <Wire.h>

typedef struct {
  int scale;
  int odr;
  int bw;
  float sensitivity;
} conf;

class Nano33BLE_IMU {
public:
  Nano33BLE_IMU(TwoWire & wire);
  ~Nano33BLE_IMU();

  int begin();
  void end();

  int readAcceleration(float & x, float & y, float & z);
  int readAcceleration(float arr[], int length);
  int readRawAcceleration(int16_t & x, int16_t & y, int16_t & z);
  int readRawAcceleration(int16_t arr[], int length);
  int accelerationAvailable();
  int setAccelerationScale(int scale); // 2, 4, 8, 16
  int setAccelerationScale(const char scale[]);
  int setAccelerationODR(const char odr[]); // 10, 50, 119, 238, 476, 952
  int accelerationScale();
  float accelerationSampleRate();
  float accelerationSensitivity();

  int readGyroscope(float & x, float & y, float & z);
  int readGyroscope(float arr[], int length);
  int readRawGyroscope(int16_t & x, int16_t & y, int16_t & z);
  int readRawGyroscope(int16_t arr[], int length);
  int gyroscopeAvailable();
  int setGyroscopeScale(int scale); // 245, 500, 2000
  int setGyroscopeScale(const char scale[]);
  int setGyroscopeODR(const char odr[]); // 14.9, 59.5, 119, 238, 476, 952
  int gyroscopeScale();
  float gyroscopeSampleRate();
  float gyroscopeSensitivity();

  int readMagneticField(float & x, float & y, float & z);
  int readMagneticField(float arr[], int length);
  int readRawMagneticField(int16_t & x, int16_t & y, int16_t & z);
  int readRawMagneticField(int16_t arr[], int length);
  int magneticFieldAvailable();
//  int setMagneticFieldScale(int scale); // 4, 8, 12, 16
//  int setMagneticFieldScale(const char scale[]);
  int setMagneticFieldODR(const char odr[]); // 0.625, 1.25, 2.5, 5, 10, 20, 40, 80
  int magneticFieldScale();
  float magneticFieldSampleRate();
  float magneticFieldSensitivity();


  int readAccelGyro(float & ax, float & ay, float & az,
                    float & gx, float & gy, float & gz);
  int readAccelGyro(float accel[], float gyro[], int ac_len, int g_len);
  int readAccelGyro(float arr[], int length);
  int accelgyroAvailable();

private:
  int readRegister(uint8_t slaveAddress, uint8_t address);
  int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t * data, size_t length);
  int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);

private:
  TwoWire * _wire;
  conf accel_conf;
  conf gyro_conf;
  conf mag_conf;
};

#ifdef ARDUINO_ARDUINO_NANO33BLE 
extern Nano33BLE_IMU IMU;
#endif

#endif
