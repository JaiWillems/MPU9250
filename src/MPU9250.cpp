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

Vector3D NULL_VECTOR_OFFSET = {
    .x = 0.0,
    .y = 0.0,
    .z = 0.0
};
Matrix3x3 NULL_MATRIX_OFFSET = {
    .m11 = 1.0,
    .m12 = 0.0,
    .m13 = 0.0,
    .m21 = 0.0,
    .m22 = 1.0,
    .m23 = 0.0,
    .m31 = 0.0,
    .m32 = 0.0,
    .m33 = 1.0
};

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

    _aOffset = NULL_VECTOR_OFFSET;
    _gOffset = NULL_VECTOR_OFFSET;
    _mHardIronOffset = NULL_VECTOR_OFFSET;
    _mSoftIronOffset = NULL_MATRIX_OFFSET;
}

void MPU9250::calibrateAccelGyro() {
    int samples = NUMBER_OF_CALIBRATION_SAMPLES;
    for (int i = 0; i < samples; i++) {
        Vector3D accelData = getRawAccel();
        _aOffset.x += accelData.x;
        _aOffset.y += accelData.y;
        _aOffset.z += accelData.z;

        Vector3D gyroData = getRawGyro();
        _gOffset.x += gyroData.x;
        _gOffset.y += gyroData.y;
        _gOffset.z += gyroData.z;
    }

    _aOffset.x /= samples;
    _aOffset.y /= samples;
    _aOffset.z /= samples;

    // Subtract 1G so gravity vector is not calibrated out.
    _aOffset.z -= 1;

    _gOffset.x /= samples;
    _gOffset.y /= samples;
    _gOffset.z /= samples;
}

void MPU9250::setMagOffsets(
    Vector3D hardIronOffset,
    Matrix3x3 softIronOffset
) {
    _mHardIronOffset = hardIronOffset;
    _mSoftIronOffset = softIronOffset;
}

// TODO: Add tilt-compensation of move to a quaternion compass.
float MPU9250::getYaw() {
    Vector3D m = getMag();
    return atan2(-m.y, m.x) * 180.0 / PI;
}

float MPU9250::getPitch() {
    Vector3D a = getAccel();
    return atan2(-a.x, sqrt(a.y * a.y + a.z * a.z)) * 180.0 / PI;
}

float MPU9250::getRoll() {
    Vector3D a = getAccel();
    return atan2(a.y, sqrt(a.x * a.x + a.z * a.z)) * 180.0 / PI;
}

Vector3D MPU9250::getAccel() {
    Vector3D data = getRawAccel();

    Vector3D calibratedData;
    calibratedData.x = data.x - _aOffset.x;
    calibratedData.y = data.y - _aOffset.y;
    calibratedData.z = data.z - _aOffset.z;

    return calibratedData;
}

Vector3D MPU9250::getRawAccel() {
    uint8_t buffer[6];
    readBytes(MPU9250_I2C_ADDRESS, ACCEL_XOUT_H, 6, buffer);

    Vector3D data;
    data.x = (buffer[0] << 8 | buffer[1]) / ACCEL_SENSITIVITY_FACTOR;
    data.y = (buffer[2] << 8 | buffer[3]) / ACCEL_SENSITIVITY_FACTOR;
    data.z = (buffer[4] << 8 | buffer[5]) / ACCEL_SENSITIVITY_FACTOR;

    return data;
}

Vector3D MPU9250::getGyro() {
    Vector3D data = getRawGyro();

    Vector3D calibratedData;
    calibratedData.x = data.x - _gOffset.x;
    calibratedData.y = data.y - _gOffset.y;
    calibratedData.z = data.z - _gOffset.z;

    return calibratedData;
}

Vector3D MPU9250::getRawGyro() {
    uint8_t buffer[6];
    readBytes(MPU9250_I2C_ADDRESS, GYRO_XOUT_H, 6, buffer);

    Vector3D data;
    data.x = (buffer[0] << 8 | buffer[1]) / GYRO_SENSITIVITY_FACTOR;
    data.y = (buffer[2] << 8 | buffer[3]) / GYRO_SENSITIVITY_FACTOR;
    data.z = (buffer[4] << 8 | buffer[5]) / GYRO_SENSITIVITY_FACTOR;

    return data;
}

Vector3D MPU9250::getMag() {
    Vector3D data = getRawMag();
    float x = data.x - _mHardIronOffset.x;
    float y = data.y - _mHardIronOffset.y;
    float z = data.z - _mHardIronOffset.z;

    Matrix3x3 s = _mSoftIronOffset;

    Vector3D calibratedData;
    calibratedData.x = s.m11 * x + s.m12 * y + s.m13 * z;
    calibratedData.y = s.m21 * x + s.m22 * y + s.m23 * z;
    calibratedData.z = s.m31 * x + s.m32 * y + s.m33 * z;

    return calibratedData;
}

Vector3D MPU9250::getRawMag() {
    uint8_t buffer[7];
    readBytes(AK8963_I2C_ADDRESS, MAG_XOUT_L, 7, buffer);

    // Magnetometer values in AK8963 frame.
    float x = MAG_SCALING_FACTOR * (buffer[0] | buffer[1] << 8);
    float y = MAG_SCALING_FACTOR * (buffer[2] | buffer[3] << 8);
    float z = MAG_SCALING_FACTOR * (buffer[4] | buffer[5] << 8);

    // Magnetometer values in MPU9250 frame.
    Vector3D data;
    data.x = y; // AK8963 y-axis is aligned to MPU9250 x-axis.
    data.y = x; // AK8963 x-axis is aligned to MPU9250 y-axis.
    data.z = -z; // AK8963 z-axis is opposite to MPU9250 z-axis.

    // TODO: Check overflow byte (byte 7, register 0x09).

    return data;
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
