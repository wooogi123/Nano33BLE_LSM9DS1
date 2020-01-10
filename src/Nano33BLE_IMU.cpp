/*
 * Nano33BLE_IMU.cpp
 *
 * Environment specifics:
 *     IDE: Arduino 1.8.10
 *     Platform: Arduino Nano 33 BLE
 */

#include "Nano33BLE_IMU.h"
#include "LSM9DS1_Registers.h"

Nano33BLE_IMU::Nano33BLE_IMU(TwoWire & wire) : _wire(&wire) {
}

Nano33BLE_IMU::~Nano33BLE_IMU() {
}

int Nano33BLE_IMU::begin() {
  _wire->begin();

  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG8, 0x05);
  writeRegister(LSM9DS1_M_ADDR, CTRL_REG2_M, 0x0C);

  delay(10);

  if (readRegister(LSM9DS1_AG_ADDR, WHO_AM_I) != 0x68) {
    end();
    return 0;
  }

  if (readRegister(LSM9DS1_M_ADDR, WHO_AM_I) != 0x3D) {
    end();
    return 0;
  }

  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG6_XL, 0x70); // 119 Hz, 4G
  accel_conf.scale = 0x02;
  accel_conf.odr = 0x03;
  accel_conf.sensitivity = 0.000122;
  accel_conf.bw = 0;

  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG1_G, 0x78); // 119 Hz, 2000 dps, 16 Hz BW
  gyro_conf.scale = 0x03;
  gyro_conf.odr = 0x03;
  gyro_conf.sensitivity = 0.07;
  accel_conf.bw = 0;

  writeRegister(LSM9DS1_M_ADDR, CTRL_REG1_M, 0xB4); // Temperature compensation enable, medium performance, 20 Hz
  writeRegister(LSM9DS1_M_ADDR, CTRL_REG2_M, 0x00); // 4 Gauss
  writeRegister(LSM9DS1_M_ADDR, CTRL_REG3_M, 0x00); // Continuous conversion mode
  mag_conf.scale = 0x00;
  mag_conf.odr = 0x05;
  mag_conf.sensitivity = 0.00014;
  mag_conf.bw = 0;

  return 1;
}

void Nano33BLE_IMU::end() {
  writeRegister(LSM9DS1_M_ADDR, CTRL_REG3_M, 0x03);
  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG1_G, 0x00);
  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG6_XL, 0x00);

  _wire->end();
}

int Nano33BLE_IMU::readAcceleration(float & x, float & y, float & z) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_XL, (uint8_t *) data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }
  
  x = data[0] * accel_conf.sensitivity;
  y = data[1] * accel_conf.sensitivity;
  z = data[2] * accel_conf.sensitivity;

  return 1;
}

int Nano33BLE_IMU::readAcceleration(float arr[], int length) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_XL, (uint8_t *) data, sizeof(data))) {
    arr[0] = NAN;
    arr[1] = NAN;
    arr[2] = NAN;

    return 0;
  }

  arr[0] = data[0] * accel_conf.sensitivity;
  arr[1] = data[1] * accel_conf.sensitivity;
  arr[2] = data[2] * accel_conf.sensitivity;

  return 1;
}

int Nano33BLE_IMU::readRawAcceleration(int16_t & x, int16_t & y, int16_t & z) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_XL, (uint8_t *) data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0];
  y = data[1];
  z = data[2];

  return 1;
}

int Nano33BLE_IMU::readRawAcceleration(int16_t arr[], int length) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_XL, (uint8_t *) data, sizeof(data))) {
    arr[0] = NAN;
    arr[1] = NAN;
    arr[2] = NAN;

    return 0;
  }

  arr[0] = data[0];
  arr[1] = data[1];
  arr[2] = data[2];

  return 1;
}

int Nano33BLE_IMU::accelerationAvailable() {
  if (readRegister(LSM9DS1_AG_ADDR, STATUS_REG_0) & 0x01) {
    return 1;
  }

  return 0;
}

int Nano33BLE_IMU::setAccelerationScale(int scale) {
  uint8_t temp = (accel_conf.odr << 5) + accel_conf.bw;

  if (scale == 2) {
    accel_conf.scale = 0x00;
    accel_conf.sensitivity = 0.000061;
    temp += 0x00;
  } else if (scale == 16) {
    accel_conf.scale = 0x01;
    accel_conf.sensitivity = 0.000732;
    temp += 0x08;
  } else if (scale == 4) {
    accel_conf.scale = 0x02;
    accel_conf.sensitivity = 0.000122;
    temp += 0x10;
  } else if (scale == 8){
    accel_conf.scale = 0x03;
    accel_conf.sensitivity = 0.000244;
    temp += 0x18;
  } else {
    return 1;
  }

  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG6_XL, temp);
  return 0;
}

int Nano33BLE_IMU::setAccelerationScale(const char scale[]) {
  uint8_t temp = (accel_conf.odr << 5) + accel_conf.bw;

  if (scale == "2") {
    accel_conf.scale = 0x00;
    accel_conf.sensitivity = 0.000061;
    temp += 0x00;
  } else if (scale == "16") {
    accel_conf.scale = 0x01;
    accel_conf.sensitivity = 0.000732;
    temp += 0x08;
  } else if (scale == "4") {
    accel_conf.scale = 0x02;
    accel_conf.sensitivity = 0.000122;
    temp += 0x10;
  } else if (scale == "8") {
    accel_conf.scale = 0x03;
    accel_conf.sensitivity = 0.000244;
    temp += 0x18;
  } else {
    return 1;
  }

  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG6_XL, temp);
  return 0;
}

int Nano33BLE_IMU::setAccelerationODR(const char odr[]) {
  uint8_t temp = (accel_conf.scale << 3) + accel_conf.bw;

  if (odr == "10") {
    accel_conf.odr = 0x01;
    temp += 0x20;
  } else if (odr == "50") {
    accel_conf.odr = 0x02;
    temp += 0x40;
  } else if (odr == "119") {
    accel_conf.odr = 0x03;
    temp += 0x60;
  } else if (odr == "238") {
    accel_conf.odr = 0x04;
    temp += 0x80;
  } else if (odr == "476") {
    accel_conf.odr = 0x05;
    temp += 0xA0;
  } else if (odr == "952") {
    accel_conf.odr = 0x06;
    temp += 0xC0;
  } else {
    return 1;
  }

  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG6_XL, temp);
  return 0;
}

int Nano33BLE_IMU::accelerationScale() {
  uint8_t temp = readRegister(LSM9DS1_AG_ADDR, CTRL_REG6_XL);
  temp -= (accel_conf.odr << 5) + accel_conf.bw;

  if (temp == 0x00) {
    return 2;
  } else if (temp == 0x08) {
    return 16;
  } else if (temp == 0x10) {
    return 4;
  } else if (temp == 0x18) {
    return 8;
  } else {
    return 0;
  }
}

float Nano33BLE_IMU::accelerationSampleRate() {
  uint8_t temp = readRegister(LSM9DS1_AG_ADDR, CTRL_REG6_XL) >> 5;

  if (temp == 0x01) {
    return 10.F;
  } else if (temp == 0x02) {
    return 50.F;
  } else if (temp == 0x03) {
    return 119.F;
  } else if (temp == 0x04) {
    return 238.F;
  } else if (temp == 0x05) {
    return 476.F;
  } else if (temp == 0x06) {
    return 952.F;
  } else {
    return 0;
  }
}

float Nano33BLE_IMU::accelerationSensitivity() {
  return accel_conf.sensitivity;
}

int Nano33BLE_IMU::readGyroscope(float & x, float & y, float & z) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_G, (uint8_t *) data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * gyro_conf.sensitivity;
  y = data[1] * gyro_conf.sensitivity;
  z = data[2] * gyro_conf.sensitivity;

  return 1;
}

int Nano33BLE_IMU::readGyroscope(float arr[], int length) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_G, (uint8_t *) data, sizeof(data))) {
    arr[0] = NAN;
    arr[1] = NAN;
    arr[2] = NAN;

    return 0;
  }

  arr[0] = data[0] * gyro_conf.sensitivity;
  arr[1] = data[1] * gyro_conf.sensitivity;
  arr[2] = data[2] * gyro_conf.sensitivity;

  return 1;
}

int Nano33BLE_IMU::readRawGyroscope(int16_t & x, int16_t & y, int16_t & z) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_G, (uint8_t *) data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0];
  y = data[1];
  z = data[2];

  return 1;
}

int Nano33BLE_IMU::readRawGyroscope(int16_t arr[], int length) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_G, (uint8_t *) data, sizeof(data))) {
    arr[0] = NAN;
    arr[1] = NAN;
    arr[2] = NAN;

    return 0;
  }

  arr[0] = data[0];
  arr[1] = data[1];
  arr[2] = data[2];

  return 1;
}

int Nano33BLE_IMU::gyroscopeAvailable() {
  if (readRegister(LSM9DS1_AG_ADDR, STATUS_REG_0) & 0x02) {
    return 1;
  }

  return 0;
}

int Nano33BLE_IMU::setGyroscopeScale(int scale) {
  uint8_t temp = (gyro_conf.odr << 5) + gyro_conf.bw;

  if (scale == 245) {
    gyro_conf.scale = 0x00;
    gyro_conf.sensitivity = 0.00875;
    temp += 0x00;
  } else if (scale == 500) {
    gyro_conf.scale = 0x01;
    gyro_conf.sensitivity = 0.0175;
    temp += 0x08;
  } else if (scale == 2000) {
    gyro_conf.scale = 0x03;
    gyro_conf.sensitivity = 0.07;
    temp += 0x18;
  } else {
    return 1;
  }

  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG1_G, temp);
  return 0;
}

int Nano33BLE_IMU::setGyroscopeScale(const char scale[]) {
  uint8_t temp = (gyro_conf.odr << 5) + gyro_conf.bw;

  if (scale == "245") {
    gyro_conf.scale = 0x00;
    gyro_conf.sensitivity = 0.00875;
    temp += 0x00;
  } else if (scale == "500") {
    gyro_conf.scale = 0x01;
    gyro_conf.sensitivity = 0.0175;
    temp += 0x08;
  } else if (scale == "2000") {
    gyro_conf.scale = 0x03;
    gyro_conf.sensitivity = 0.07;
    temp += 0x18;
  } else {
    return 1;
  }

  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG1_G, temp);
  return 0;
}

int Nano33BLE_IMU::setGyroscopeODR(const char odr[]) {
  uint8_t temp = (gyro_conf.scale << 3) + gyro_conf.bw;

  if (odr == "14.9") {
    gyro_conf.odr = 0x01;
    temp += 0x20;
  } else if (odr == "59.5") {
    gyro_conf.odr = 0x02;
    temp += 0x40;
  } else if (odr == "119") {
    gyro_conf.odr = 0x03;
    temp += 0x60;
  } else if (odr == "238") {
    gyro_conf.odr = 0x04;
    temp += 0x80;
  } else if (odr == "476") {
    gyro_conf.odr = 0x05;
    temp += 0xA0;
  } else if (odr == "952") {
    gyro_conf.odr = 0x06;
    temp += 0xC0;
  } else {
    return 1;
  }

  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG1_G, temp);
  return 0;
}

int Nano33BLE_IMU::gyroscopeScale() {
  uint8_t temp = readRegister(LSM9DS1_AG_ADDR, CTRL_REG1_G);
  temp -= (gyro_conf.odr << 5) + gyro_conf.bw;

  if (temp == 0x00) {
    return 245;
  } else if (temp == 0x08) {
    return 500;
  } else if (temp == 0x18) {
    return 2000;
  } else {
    return 0;
  }
}

float Nano33BLE_IMU::gyroscopeSampleRate() {
  uint8_t temp = readRegister(LSM9DS1_AG_ADDR, CTRL_REG1_G) >> 5;

  if (temp == 0x01) {
    return 14.9F;
  } else if (temp == 0x02) {
    return 59.5F;
  } else if (temp == 0x03) {
    return 119.F;
  } else if (temp == 0x04) {
    return 238.F;
  } else if (temp == 0x05) {
    return 476.F;
  } else if (temp == 0x06) {
    return 952.F;
  } else {
    return 0;
  }
}

float Nano33BLE_IMU::gyroscopeSensitivity() {
  return gyro_conf.sensitivity;
}

int Nano33BLE_IMU::readMagneticField(float & x, float & y, float & z) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_M_ADDR, OUT_X_L_M, (uint8_t *) data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * mag_conf.sensitivity;
  y = data[1] * mag_conf.sensitivity;
  z = data[2] * mag_conf.sensitivity;

  return 1;
}

int Nano33BLE_IMU::readMagneticField(float arr[], int length) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_M_ADDR, OUT_X_L_M, (uint8_t *) data, sizeof(data))) {
    arr[0] = NAN;
    arr[1] = NAN;
    arr[2] = NAN;

    return 0;
  }

  arr[0] = data[0] * mag_conf.sensitivity;
  arr[1] = data[1] * mag_conf.sensitivity;
  arr[2] = data[2] * mag_conf.sensitivity;

  return 1;
}

int Nano33BLE_IMU::readRawMagneticField(int16_t & x, int16_t & y, int16_t & z) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_M_ADDR, OUT_X_L_M, (uint8_t *) data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0];
  y = data[1];
  z = data[2];

  return 1;
}

int Nano33BLE_IMU::readRawMagneticField(int16_t arr[], int length) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_M_ADDR, OUT_X_L_M, (uint8_t *) data, sizeof(data))) {
    arr[0] = NAN;
    arr[1] = NAN;
    arr[2] = NAN;

    return 0;
  }

  arr[0] = data[0];
  arr[1] = data[1];
  arr[2] = data[2];

  return 1;
}

int Nano33BLE_IMU::magneticFieldAvailable() {
  if (readRegister(LSM9DS1_M_ADDR, STATUS_REG_M) & 0x08) {
    return 1;
  }

  return 0;
}

/*
int Nano33BLE_IMU::setMagneticFieldScale(int scale) {
  uint8_t temp = 0x00;

  if (scale == 4) {
    mag_conf.scale = 0x00;
    mag_conf.sensitivity = 0.00014;
  } else if (scale == 8) {
    mag_conf.scale = 0x01;
    mag_conf.sensitivity = 0.00029;
    temp += 0x20;
  } else if (scale == 12) {
    mag_conf.scale = 0x02;
    mag_conf.sensitivity = 0.00043;
    temp += 0x40;
  } else if (scale == 16) {
    mag_conf.scale = 0x03;
    mag_conf.sensitivity = 0.00058;
    temp += 0x60;
  } else {
    return 1;
  }

  writeRegister(LSM9DS1_M_ADDR, CTRL_REG2_M, temp);
  return temp;
}

int Nano33BLE_IMU::setMagneticFieldScale(const char scale[]) {
  uint8_t temp = 0x00;

  if (scale == "4") {
    mag_conf.scale = 0x00;
    mag_conf.sensitivity = 0.00014;
  } else if (scale == "8") {
    mag_conf.scale = 0x01;
    mag_conf.sensitivity = 0.00029;
    temp += 0x20;
  } else if (scale == "12") {
    mag_conf.scale = 0x02;
    mag_conf.sensitivity = 0.00043;
    temp += 0x40;
  } else if (scale == "16") {
    mag_conf.scale = 0x03;
    mag_conf.sensitivity = 0.00058;
    temp += 0x60;
  } else {
    return 1;
  }

  writeRegister(LSM9DS1_M_ADDR, CTRL_REG2_M, temp);
  return temp;
}
*/

int Nano33BLE_IMU::setMagneticFieldODR(const char odr[]) {
  uint8_t temp = 0x00;

  if (odr == "0.625") {
    mag_conf.odr = 0x00;
    temp += 0x00;
  } else if (odr == "1.25") {
    mag_conf.odr = 0x01;
    temp += 0x04;
  } else if (odr == "2.5") {
    mag_conf.odr = 0x02;
    temp += 0x08;
  } else if (odr == "5") {
    mag_conf.odr = 0x03;
    temp += 0x0C;
  } else if (odr == "10") {
    mag_conf.odr = 0x04;
    temp += 0x10;
  } else if (odr == "20") {
    mag_conf.odr = 0x05;
    temp += 0x14;
  } else if (odr == "40") {
    mag_conf.odr = 0x06;
    temp += 0x18;
  } else if (odr == "80") {
    mag_conf.odr = 0x07;
    temp += 0x1C;
  } else {
    return 1;
  }

  writeRegister(LSM9DS1_M_ADDR, CTRL_REG1_M, temp);
  return 0;
}

int Nano33BLE_IMU::magneticFieldScale() {
  uint8_t temp = readRegister(LSM9DS1_AG_ADDR, CTRL_REG2_M) >> 5;

  if (temp == 0x00) {
    return 4;
  } else if (temp == 0x01) {
    return 8;
  } else if (temp == 0x02) {
    return 12;
  } else if (temp == 0x03) {
    return 16;
  } else {
    return 0;
  }
}

float Nano33BLE_IMU::magneticFieldSampleRate() {
  uint8_t temp = readRegister(LSM9DS1_M_ADDR, CTRL_REG1_M) >> 2;

  if (temp == 0x00) {
    return 0.625F;
  } else if (temp == 0x01) {
    return 1.25F;
  } else if (temp == 0x02) {
    return 2.5F;
  } else if (temp == 0x03) {
    return 5.F;
  } else if (temp == 0x04) {
    return 10.F;
  } else if (temp == 0x05) {
    return 20.F;
  } else if (temp == 0x06) {
    return 40.F;
  } else if (temp == 0x07) {
    return 80.F;
  } else {
    return 0;
  }
}

float Nano33BLE_IMU::magneticFieldSensitivity() {
  return mag_conf.sensitivity;
}

int Nano33BLE_IMU::readAccelGyro(float & ax, float & ay, float & az,
                                 float & gx, float & gy, float & gz) {
  ax = NAN;
  ay = NAN;
  az = NAN;

  gx = NAN;
  gy = NAN;
  gz = NAN;

  int16_t data[3];

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_XL, (uint8_t *) data, sizeof(data))) {
    return 0;
  }

  ax = data[0] * accel_conf.sensitivity;
  ay = data[1] * accel_conf.sensitivity;
  az = data[2] * accel_conf.sensitivity;

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_G, (uint8_t *) data, sizeof(data))) {
    return 0;
  }

  gx = data[0] * gyro_conf.sensitivity;
  gy = data[1] * gyro_conf.sensitivity;
  gz = data[2] * gyro_conf.sensitivity;

  return 1;
}

int Nano33BLE_IMU::readAccelGyro(float accel[], float gyro[], int ac_len, int g_len) {
  for (int i = 0; i < ac_len; ++i) {
    accel[i] = NAN;
  }

  for (int i = 0; i < g_len; ++i) {
    gyro[i] = NAN;
  }

  int16_t data[3];

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_XL, (uint8_t *) data, sizeof(data))) {
    return 0;
  }

  accel[0] = data[0] * accel_conf.sensitivity;
  accel[1] = data[1] * accel_conf.sensitivity;
  accel[2] = data[2] * accel_conf.sensitivity;

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_G, (uint8_t *) data, sizeof(data))) {
    return 0;
  }

  gyro[0] = data[0] * gyro_conf.sensitivity;
  gyro[1] = data[1] * gyro_conf.sensitivity;
  gyro[2] = data[2] * gyro_conf.sensitivity;

  return 1;
}

int Nano33BLE_IMU::readAccelGyro(float arr[], int length) {
  for (int i = 0; i < length; ++i) {
    arr[i] = NAN;
  }

  int16_t data[3];

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_XL, (uint8_t *) data, sizeof(data))) {
    return 0;
  }

  arr[0] = data[0] * accel_conf.sensitivity;
  arr[1] = data[1] * accel_conf.sensitivity;
  arr[2] = data[2] * accel_conf.sensitivity;

  if (!readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_G, (uint8_t *) data, sizeof(data))) {
    return 0;
  }

  arr[3] = data[0] * gyro_conf.sensitivity;
  arr[4] = data[1] * gyro_conf.sensitivity;
  arr[5] = data[2] * gyro_conf.sensitivity;

  return 1;
}

int Nano33BLE_IMU::accelgyroAvailable() {
  uint8_t ac_odr = readRegister(LSM9DS1_AG_ADDR, CTRL_REG6_XL) >> 5;
  uint8_t gy_odr = readRegister(LSM9DS1_AG_ADDR, CTRL_REG1_G) >> 5;

  if (ac_odr < 0x03) {
    return 1;
  }

  if (gy_odr < 0x03) {
    return 1;
  }

  if (ac_odr != gy_odr) {
    return 1;
  }

  if (readRegister(LSM9DS1_AG_ADDR, STATUS_REG_0) & 0x03) {
    return 1;
  }

  return 0;
}

int Nano33BLE_IMU::readRegister(uint8_t slaveAddress, uint8_t address) {
  _wire->beginTransmission(slaveAddress);
  _wire->write(address);

  if (_wire->endTransmission() != 0) {
    return -1;
  }

  if (_wire->requestFrom(slaveAddress, 1) != 1) {
    return -1;
  }

  return _wire->read();
}

int Nano33BLE_IMU::readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t * data, size_t length) {
  _wire->beginTransmission(slaveAddress);
  _wire->write(0x80 | address);

  if (_wire->endTransmission(false) != 0) {
    return -1;
  }

  if (_wire->requestFrom(slaveAddress, length) != length) {
    return 0;
  }

  for (size_t i = 0; i < length; i++) {
    *data++ = _wire->read();
  }

  return 1;
}

int Nano33BLE_IMU::writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value) {
  _wire->beginTransmission(slaveAddress);
  _wire->write(address);
  _wire->write(value);

  if (_wire->endTransmission() != 0) {
    return 0;
  }

  return 1;
}

#ifdef ARDUINO_ARDUINO_NANO33BLE
Nano33BLE_IMU IMU(Wire1);
#endif
