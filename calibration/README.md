# Magnetometer Calibration

Magnetometers measure the Earths local magnetic field vector and are highly sensitive to local disturbances. Calibration of the AK8963 magnetometer, attached to the MPU9250 IMU, will be performed using the Motion Sensor Calibration Tool (MotionCal) available for download from the [PRJC website](https://www.pjrc.com/store/prop_shield.html).

## Scope & Limitations

This calibration procedure will account for internal time-invariant hard and soft iron disturbances, therefore, addressing upset due to the drone's physical layout and Electro-Magnetic Interference (EMI) signatures present during calibration. Recalibration is required if the drone's hardware or software configuration changes.

The calibration does not account for time-variant disturbances which may be introduced in different flight regeimes from changes to EMI patterns. A potential source of these disturbances are the four Brushless DC (BLDC) motors which will have varying voltage requirements for different types of flight.

## Procedure

To calibrate the magnetometer:
1. Run the `Calibration.ino` script.
1. Ensure the Serial Monitor and Serial Plotter are closed in the Arduino IDE.
1. Open up the MotionCal tool.
1. In the top left of the tool, select the port attached to the microcontroller.
1. Rotate the magnetometer through all orientations until the "Gaps" metric at the bottom center of the tool drops below 1%.
1. Configure the magnetometer offsets in the Arduino project as seen below.

> Note: The "Magnetic Offset" and "Magnetic Mapping" values in the top right of the tool are the hard iron offset vector and soft iron offset matrix, respectively.

```cpp
#include "MPU9250.h"
#include "Types.h"

MPU9250 mpu;

// Example magnetic offset data.
Vector3D HARD_IRON_OFFSET = {
    .x = 14.38,
    .y = 26.21,
    .z = -8.81
};
// Example magnetic mapping data.
Matrix3x3 SOFT_IRON_OFFSET = {
    .m11 = 1.007,
    .m12 = 0.006,
    .m13 = -0.005,
    .m21 = 0.006,
    .m22 = 1.007,
    .m23 = -0.008,
    .m31 = -0.005,
    .m32 = -0.008,
    .m33 = 0.986
};

void setup() {
    mpu.setup();
    mpu.calibrateAccelGyro();
    mpu.setMagOffsets(
        HARD_IRON_OFFSET,
        SOFT_IRON_OFFSET
    );
}
```
