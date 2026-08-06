#include <stdio.h>
#include <unistd.h>
#include "libbno055-linux/bno055_c.h"

int main(void) {
    printf("Initializing BNO055 via C API...\n");

    // Create I2C handle (Address 0x28, /dev/i2c-1)
    bno055_handle_t imu = bno055_create_i2c(0x28, "/dev/i2c-1");
    if (!imu) {
        fprintf(stderr, "Failed to create BNO055 handle.\n");
        return 1;
    }

    // Initialize in NDOF fusion mode
    if (!bno055_begin(imu, BNO055_OPMODE_NDOF)) {
        fprintf(stderr, "Sensor initialization failed (Is hardware connected?).\n");
        bno055_destroy(imu);
        return 1;
    }

    printf("BNO055 Initialized Successfully!\n");

    // Read orientation and accelerometer data
    bno055_quaternion_t q;
    bno055_vector3_t accel;
    bno055_calibration_status_t calib;

    for (int i = 0; i < 5; ++i) {
        if (bno055_get_quaternion(imu, &q)) {
            bno055_vector3_t euler = bno055_to_euler_degrees(&q);
            printf("Euler (deg): Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", euler.x, euler.y, euler.z);
        }

        if (bno055_get_accelerometer(imu, &accel)) {
            printf("Accel (m/s^2): X=%.2f, Y=%.2f, Z=%.2f\n", accel.x, accel.y, accel.z);
        }

        if (bno055_get_calibration_status(imu, &calib)) {
            printf("Calibration: SYS=%d, GYRO=%d, ACCEL=%d, MAG=%d\n",
                   calib.sys, calib.gyro, calib.accel, calib.mag);
        }

        printf("----------------------------------------\n");
        usleep(200000); // 200ms
    }

    bno055_destroy(imu);
    return 0;
}
