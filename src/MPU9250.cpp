/*
BSD 3-Clause License

Copyright (c) 2025, Jai Willems

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "MPU9250.h"

void MPU9250::setup() {
    Wire.begin();

    writeByte(MPU9250_I2C_ADDRESS, PWR_MGMT_1, 0x00);
    delay(100);

    writeByte(MPU9250_I2C_ADDRESS, SMPLRT_DIV, 0x09);
    delay(10);

    writeByte(MPU9250_I2C_ADDRESS, CONFIG, 0x03);
    delay(10);

    writeByte(MPU9250_I2C_ADDRESS, GYRO_CONFIG, 0x00);
    delay(10);

    writeByte(MPU9250_I2C_ADDRESS, ACCEL_CONFIG, 0x00);
    delay(10);

    writeByte(MPU9250_I2C_ADDRESS, INT_PIN_CFG, 0x02);
    delay(10);

    writeByte(AK8963_I2C_ADDRESS, CNTL1, 0x00);
    delay(10);

    writeByte(AK8963_I2C_ADDRESS, CNTL1, 0x16);
    delay(10);
}

void MPU9250::calibrateAccel() {
    int samples = NUMBER_OF_CALIBRATION_SAMPLES;
    for (int i = 0; i < samples; i++) {
        Vector3D data = getRawAccel();
        _axOffset += data.x;
        _ayOffset += data.y;
        _azOffset += data.z;
    }

    _axOffset /= samples;
    _ayOffset /= samples;
    _azOffset /= samples;
}

float MPU9250::getPitch() {
    Vector3D a = getRawAccel();
    return atan2(a.x, sqrt(a.y * a.y + a.z * a.z)) * 180.0 / PI;
}

float MPU9250::getRoll() {
    Vector3D a = getAccel();
    return atan2(a.y, a.z) * 180.0 / PI;
}

Vector3D MPU9250::getAccel() {
    Vector3D data = getRawAccel();

    Vector3D calibratedData;
    calibratedData.x = (data.x - _axOffset) / ACCEL_SENSITIVITY_FACTOR;
    calibratedData.y = (data.y - _ayOffset) / ACCEL_SENSITIVITY_FACTOR;
    calibratedData.z = (data.z - _azOffset) / ACCEL_SENSITIVITY_FACTOR;

    return calibratedData;
}

Vector3D MPU9250::getRawAccel() {
    uint8_t buffer[6];
    readBytes(MPU9250_I2C_ADDRESS, ACCEL_XOUT_H, 6, buffer);

    Vector3D data;
    data.x = buffer[0] << 8 | buffer[1];
    data.y = buffer[2] << 8 | buffer[3];
    data.z = buffer[4] << 8 | buffer[5];

    return data;
}

float MPU9250::readGyroX() {
    return readValue(GYRO_XOUT_H, true) / GYRO_SENSITIVITY_FACTOR;
}

float MPU9250::readGyroY() {
    return readValue(GYRO_YOUT_H, true) / GYRO_SENSITIVITY_FACTOR;
}

float MPU9250::readGyroZ() {
    return readValue(GYRO_ZOUT_H, true) / GYRO_SENSITIVITY_FACTOR;
}

int16_t MPU9250::readMagX() {
    return readValue(MAG_XOUT_L, false);
}

int16_t MPU9250::readMagY() {
    return readValue(MAG_YOUT_L, false);
}

int16_t MPU9250::readMagZ() {
    return readValue(MAG_ZOUT_L, false);
}

void MPU9250::writeByte(
    uint8_t i2cAddress,
    uint8_t registerAddress,
    int16_t data
) {
    Wire.beginTransmission(i2cAddress);
    Wire.write(registerAddress);
    Wire.write(data);
    Wire.endTransmission(true);
}

void MPU9250::readBytes(
    uint8_t i2cAddress,
    uint8_t registerAddress,
    uint8_t numberOfBytes,
    uint8_t* destination
) {
    Wire.beginTransmission(i2cAddress);
    Wire.write(registerAddress);
    Wire.endTransmission(false);
    Wire.requestFrom(i2cAddress, numberOfBytes);
    
    for (int i = 0; i < numberOfBytes; i++) {
        destination[i] = Wire.read();
    }
}

int16_t MPU9250::readValue(
    uint8_t registerAddress,
    bool highByteFirst
) {
    request2ByteData(
        registerAddress
    );
    
    if (highByteFirst) {
        return Wire.read() << 8 | Wire.read();
    } else {
        return Wire.read() |Wire.read() << 8;
    }
}

void MPU9250::request2ByteData(
    uint8_t registerAddress
) {
    Wire.beginTransmission(MPU9250_I2C_ADDRESS);
    Wire.write(registerAddress);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_I2C_ADDRESS, 2, true);
}
