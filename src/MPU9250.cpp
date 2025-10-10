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
    writeToRegister(PWR_MGMT_1, 0x00);

    writeToRegister(ACCEL_CONFIG, 0x00);
    writeToRegister(GYRO_CONFIG, 0x00);
    writeToRegister(CONFIG, 0x03);
    writeToRegister(SMPLRT_DIV, 0x09);
}

void MPU9250::calibrateAccelGyro() {
    _axOffset = readValue(ACCEL_XOUT_H, true);
    _ayOffset = readValue(ACCEL_YOUT_H, true);
    _azOffset = readValue(ACCEL_ZOUT_H, true);

    _gxOffset = readValue(GYRO_XOUT_H, true);
    _gyOffset = readValue(GYRO_YOUT_H, true);
    _gzOffset = readValue(GYRO_ZOUT_H, true);
}

float MPU9250::readAccelX() {
    return (readValue(ACCEL_XOUT_H, true) - _axOffset) / ACCEL_SENSITIVITY_FACTOR;
}

float MPU9250::readAccelY() {
    return (readValue(ACCEL_YOUT_H, true) - _ayOffset) / ACCEL_SENSITIVITY_FACTOR;
}

float MPU9250::readAccelZ() {
    return (readValue(ACCEL_ZOUT_H, true) - _azOffset) / ACCEL_SENSITIVITY_FACTOR;
}

float MPU9250::readGyroX() {
    return (readValue(GYRO_XOUT_H, true) - _gxOffset) / GYRO_SENSITIVITY_FACTOR;
}

float MPU9250::readGyroY() {
    return (readValue(GYRO_YOUT_H, true) - _gyOffset) / GYRO_SENSITIVITY_FACTOR;
}

float MPU9250::readGyroZ() {
    return (readValue(GYRO_ZOUT_H, true) - _gzOffset) / GYRO_SENSITIVITY_FACTOR;
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

void MPU9250::writeToRegister(
    uint8_t registerAddress,
    int16_t data
) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(registerAddress);
    Wire.write(data);
    Wire.endTransmission(true);
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
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(registerAddress);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 2, true);
}
