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

void MPU9250::calibrateAccelGyro() {
    int samples = NUMBER_OF_CALIBRATION_SAMPLES;
    for (int i = 0; i < samples; i++) {
        Vector3D accelData = getRawAccel();
        _axOffset += accelData.x;
        _ayOffset += accelData.y;
        _azOffset += accelData.z;

        Vector3D gyroData = getRawGyro();
        _gxOffset += gyroData.x;
        _gyOffset += gyroData.y;
        _gzOffset += gyroData.z;
    }

    _axOffset /= samples;
    _ayOffset /= samples;
    _azOffset /= samples;

    // Add 1G for gravity vector.
    _azOffset -= ACCEL_SENSITIVITY_FACTOR;

    _gxOffset /= samples;
    _gyOffset /= samples;
    _gzOffset /= samples;
}

void MPU9250::calibrateMag() {
    uint16_t samples = 0;
    unsigned long startTime = millis();
    while (millis() - startTime < MAG_CALIBRATION_TIME_MS) {
        Vector3D data = getRawMag();

        _mxMean += data.x;
        if (data.x < _mxMin) _mxMin = data.x;
        if (data.x > _mxMax) _mxMax = data.x;

        _myMean += data.y;
        if (data.y < _myMin) _myMin = data.y;
        if (data.y > _myMax) _myMax = data.y;

        _mzMean += data.z;
        if (data.z < _mzMin) _mzMin = data.z;
        if (data.z > _mzMax) _mzMax = data.z;

        samples += 1;

        delay(10);
    }

    _mxMean /= samples;
    _myMean /= samples;
    _mzMean /= samples;
}

// TODO: Add tilt-compensation of move to a quaternion compass.
float MPU9250::getYaw() {
    Vector3D m = getMag();
    return atan2(m.y, m.x) * 180.0 / PI;
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

Vector3D MPU9250::getGyro() {
    Vector3D data = getRawGyro();

    Vector3D calibratedData;
    calibratedData.x = (data.x - _gxOffset) / GYRO_SENSITIVITY_FACTOR;
    calibratedData.y = (data.y - _gyOffset) / GYRO_SENSITIVITY_FACTOR;
    calibratedData.z = (data.z - _gzOffset) / GYRO_SENSITIVITY_FACTOR;

    return calibratedData;
}

Vector3D MPU9250::getRawGyro() {
    uint8_t buffer[6];
    readBytes(MPU9250_I2C_ADDRESS, GYRO_XOUT_H, 6, buffer);

    Vector3D data;
    data.x = buffer[0] << 8 | buffer[1];
    data.y = buffer[2] << 8 | buffer[3];
    data.z = buffer[4] << 8 | buffer[5];

    return data;
}

Vector3D MPU9250::getMag() {
    Vector3D data = getRawMag();

    Vector3D calibratedData;
    calibratedData.x = 2 * MAG_SCALING_FACTOR * (data.x - _mxMean) / (_mxMax - _mxMin);
    calibratedData.y = 2 * MAG_SCALING_FACTOR * (data.y - _myMean) / (_myMax - _myMin);
    calibratedData.z = 2 * MAG_SCALING_FACTOR * (data.z - _mzMean) / (_mzMax - _mzMin);

    return calibratedData;
}

Vector3D MPU9250::getRawMag() {
    uint8_t buffer[7];
    readBytes(AK8963_I2C_ADDRESS, MAG_XOUT_L, 7, buffer);

    Vector3D data;
    data.x = buffer[0] | buffer[1] << 8;
    data.y = buffer[2] | buffer[3] << 8;
    data.z = buffer[4] | buffer[5] << 8;

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
