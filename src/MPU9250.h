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

#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDRESS 0x68
#define PWR_MGMT_1 0x6B

#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define CONFIG 0x1A
#define SMPLRT_DIV 0x19

#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F

#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

#define MAG_XOUT_L 0x03
#define MAG_YOUT_L 0x05
#define MAG_ZOUT_L 0x07

#define ACCEL_SENSITIVITY_FACTOR 16384.0
#define GYRO_SENSITIVITY_FACTOR 131.0

#define NUMBER_OF_CALIBRATION_SAMPLES 100;

class MPU9250 {
    public:
        void setup();
        void calibrateAccel();
        float getPitch();
        float getRoll();
        float readAccelX();
        float readAccelY();
        float readAccelZ();
        float readGyroX();
        float readGyroY();
        float readGyroZ();
        int16_t readMagX();
        int16_t readMagY();
        int16_t readMagZ();
    private:
        int16_t _axOffset;
        int16_t _ayOffset;
        int16_t _azOffset;
        void writeToRegister(
            uint8_t registerAddress,
            int16_t data
        );
        int16_t readValue(
            uint8_t registerAddress,
            bool highByteFirst
        );
        void request2ByteData(
            uint8_t registerAddress
        );
};
