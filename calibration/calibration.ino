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
#include "Types.h"

MPU9250 mpu;

void setup() {
	// Baud rate used by the Motion Sensor Calibration Tool.
    Serial.begin(115200);

    mpu.setup();
}

void loop() {
	Vector3D a = mpu.getAccel();
	Vector3D g = mpu.getGyro();
	Vector3D m = mpu.getMag();

	// Serial format required by the Motion Sensor Calibration Tool.
    Serial.print("Raw:");
	Serial.print(int(a.x));
	Serial.print(',');
	Serial.print(int(a.y));
	Serial.print(',');
	Serial.print(int(a.z));
	Serial.print(',');
	Serial.print(int(g.x));
	Serial.print(',');
	Serial.print(int(g.y));
	Serial.print(',');
	Serial.print(int(g.z));
	Serial.print(',');
	Serial.print(int(10 * m.x)); // Multiply by 10 to capture decimal precision.
	Serial.print(',');
	Serial.print(int(10 * m.y)); // Multiply by 10 to capture decimal precision.
	Serial.print(',');
	Serial.print(int(10 * m.z)); // Multiply by 10 to capture decimal precision.
	Serial.println();

    delay(10);
}
