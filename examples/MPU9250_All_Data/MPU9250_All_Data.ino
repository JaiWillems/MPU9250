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

// Refer to the calibration\README.md file in the GitHub repository for an
// explanation on how to determine the magnetometer hard and soft iron offsets.
Vector3D HARD_IRON_OFFSET = {
    .x = 26.01,
    .y = 13.85,
    .z = 10.03
};
Matrix3x3 SOFT_IRON_OFFSET = {
    .m11 = 1.004,
    .m12 = 0.010,
    .m13 = -0.002,
    .m21 = 0.010,
    .m22 = 1.011,
    .m23 = -0.001,
    .m31 = -0.002,
    .m32 = -0.001,
    .m33 = 0.984
};

MPU9250 mpu;

void setup() {
    Serial.begin(115200);

    mpu.begin();
    mpu.calibrateAccelGyro();
    mpu.setMagnetometerOffsets(
        HARD_IRON_OFFSET,
        SOFT_IRON_OFFSET
    );
}

void loop() {
    Attitude attitude = mpu.getYawPitchRoll();

    Serial.print("Attitude (°) Yaw, Pitch, Roll:\t");
    Serial.print(attitude.yaw);
    Serial.print("\t");
    Serial.print(attitude.pitch);
    Serial.print("\t");
    Serial.println(attitude.roll);

    Vector3D accelerometer = mpu.readAccelerometer();

    Serial.print("Accelerometer (G) X, Y, Z:\t");
    Serial.print(accelerometer.x);
    Serial.print("\t");
    Serial.print(accelerometer.y);
    Serial.print("\t");
    Serial.println(accelerometer.z);


    Vector3D gyroscope = mpu.readGyroscope();

    Serial.print("Gyroscope (°/s) X, Y, Z:\t");
    Serial.print(gyroscope.x);
    Serial.print("\t");
    Serial.print(gyroscope.y);
    Serial.print("\t");
    Serial.println(gyroscope.z);

    Vector3D magnetometer = mpu.readMagnetometer();

    Serial.print("Magnetometer (µT) X, Y, Z:\t");
    Serial.print(gyroscope.x);
    Serial.print("\t");
    Serial.print(gyroscope.y);
    Serial.print("\t");
    Serial.println(gyroscope.z);

    Serial.println();

    delay(500);
}
