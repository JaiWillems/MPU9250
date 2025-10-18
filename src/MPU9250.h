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

#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"
#include "Wire.h"

/*
* MPU9250 + AK8963 register addresses.
*/

#define MPU9250_I2C_ADDRESS 0x68
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_SMPLRT_DIV 0x19
#define MPU9250_CONFIG 0x1A
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_ACCEL_YOUT_H 0x3D
#define MPU9250_ACCEL_ZOUT_H 0x3F
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_GYRO_YOUT_H 0x45
#define MPU9250_GYRO_ZOUT_H 0x47

#define AK8963_I2C_ADDRESS 0x0C
#define AK8963_CNTL1 0x0A
#define AK8963_MAG_XOUT_L 0x03
#define AK8963_MAG_YOUT_L 0x05
#define AK8963_MAG_ZOUT_L 0x07

/*
* Macros.
*/

#define ACCEL_SENSITIVITY_FACTOR 16384.0
#define GYRO_SENSITIVITY_FACTOR 131.0
#define MAG_SCALING_FACTOR 0.14994
#define NUMBER_OF_CALIBRATION_SAMPLES 100
#define DEG_TO_RAD 0.01745
#define RAD_TO_DEG 57.29578

/*
* Custom datatypes.
*/

struct Vector3D {
    float x, y, z;
};

struct Matrix3x3 {
   float m11, m12, m13;
   float m21, m22, m23;
   float m31, m32, m33;
};

struct Attitude {
   float yaw, pitch, roll;
};

/*
* MPU9250 class.
*/

class MPU9250 {
    public:
        void begin();
        void calibrateAccelGyro();
        void setMagnetometerOffsets(
            Vector3D hardIronOffset,
            Matrix3x3 softIronOffset
        );
        Attitude getYawPitchRoll();
        Vector3D readAccelerometer();
        Vector3D readRawAccelerometer();
        Vector3D readGyroscope();
        Vector3D readRawGyroscope();
        Vector3D readMagnetometer();
        Vector3D readRawMagnetometer();
    private:
        Vector3D _aOffset;
        Vector3D _gOffset;
        Vector3D _mHardIronOffset;
        Matrix3x3 _mSoftIronOffset;
        float getPitch(Vector3D accel);
        float getRoll(Vector3D accel);
        float getYaw(
            Vector3D mag,
            float pitch,
            float roll
        );
        void writeByte(
            uint8_t i2cAddr,
            uint8_t registerAddr,
            int16_t data
        );
        void readBytes(
            uint8_t i2cAddr,
            uint8_t registerAddr,
            uint8_t numberOfBytes,
            uint8_t* destination
        );
};

#endif
