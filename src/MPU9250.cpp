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
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission(true);
}

int16_t MPU9250::readRawAccelX() {
    return readValueHighByteFirst(
        ACCEL_XOUT_H
    );
}

int16_t MPU9250::readRawAccelY() {
    return readValueHighByteFirst(
        ACCEL_YOUT_H
    );
}

int16_t MPU9250::readRawAccelZ() {
    return readValueHighByteFirst(
        ACCEL_ZOUT_H
    );
}

int16_t MPU9250::readRawGyroX() {
    return readValueHighByteFirst(
        GYRO_XOUT_H
    );
}
int16_t MPU9250::readRawGyroY() {
    return readValueHighByteFirst(
        GYRO_YOUT_H
    );
}

int16_t MPU9250::readRawGyroZ() {
    return readValueHighByteFirst(
        GYRO_ZOUT_H
    );
}

int16_t MPU9250::readRawMagX() {
    return readValueLowByteFirst(
        MAG_XOUT_L
    );
}

int16_t MPU9250::readRawMagY() {
    return readValueLowByteFirst(
        MAG_YOUT_L
    );
}

int16_t MPU9250::readRawMagZ() {
    return readValueLowByteFirst(
        MAG_ZOUT_L
    );
}

int16_t MPU9250::readValueHighByteFirst(
    uint8_t highByteRegister
) {
    request2ByteData(
        highByteRegister
    );
    return Wire.read() <<8 | Wire.read();
}

int16_t MPU9250::readValueLowByteFirst(
    uint8_t lowByteRegister
) {
    request2ByteData(
        lowByteRegister
    );
    return Wire.read() | Wire.read() << 8;
}

void MPU9250::request2ByteData(
    uint8_t registerAddress
) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(registerAddress);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 2, true);
}
