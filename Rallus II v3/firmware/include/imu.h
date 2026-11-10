#pragma once
#include <stdbool.h>
#include <stdint.h>

// One raw IMU sample. Gyro in dps, accel in g. Bias is NOT applied here;
// control.c owns the bias and subtracts it.
typedef struct {
    float gx, gy, gz;   // dps
    float ax, ay, az;   // g
    bool  valid;        // false if the SPI transaction failed
} imu_sample_t;

bool imu_init(void);
bool imu_read(imu_sample_t *out);
