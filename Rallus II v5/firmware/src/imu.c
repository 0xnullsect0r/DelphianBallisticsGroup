#include "imu.h"
#include "config.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <string.h>

// ICM-42688-P, SPI mode 0, on SPI0. Register map from the datasheet
// (MPU-ICM-42688-P DS-000347). Verify against your board at first bench
// bring-up: 'i' command prints the WHO_AM_I check result at boot.

#define ICM_ADDR_BANK_SEL        0x76
#define ICM_ADDR_WHO_AM_I        0x75
#define ICM_VAL_WHO_AM_I         0x47
#define ICM_ADDR_PWR_MGMT0       0x4E   // bank 0
#define ICM_ADDR_DEVICE_CONFIG   0x11   // bank 0
#define ICM_ADDR_ACCEL_DATAX1    0x1F   // bank 0, 12 bytes through gyro Z
#define ICM_ADDR_GYRO_CONFIG0    0x4F   // bank 0: FS_SEL[7:5], ODR[3:0]
#define ICM_ADDR_ACCEL_CONFIG0   0x50   // bank 0: FS_SEL[7:5], ODR[3:0]
#define ICM_ODR_1KHZ             0x06

// Sensitivities for the configured ranges: gyro +/-2000 dps, accel +/-16 g.
// +/-16 g is the LARGEST range the ICM-42688-P has (the +/-32 g part is the
// ICM-42686-P). The motor peaks ~18 g, so az clips for a fraction of the
// burn; v_hat reads a few m/s low, which only nudges the gain schedule.
#define GYRO_LSB_PER_DPS         16.4f
#define ACCEL_LSB_PER_G          2048.0f

// Mount remap: chip axes -> body axes, body +Z = up the rocket (toward the
// nose), so body gz is the roll rate and body az reads +1 g on the pad.
// Selected by IMU_UP_AXIS in config.h; each case is a proper rotation.
static void remap3(float *v) {
    float x = v[0], y = v[1], z = v[2];
#if   IMU_UP_AXIS == 1      // chip +X toward the nose
    v[0] = y;  v[1] = z;  v[2] = x;
#elif IMU_UP_AXIS == -1     // chip -X toward the nose
    v[0] = z;  v[1] = y;  v[2] = -x;
#elif IMU_UP_AXIS == 2      // chip +Y toward the nose
    v[0] = z;  v[1] = x;  v[2] = y;
#elif IMU_UP_AXIS == -2     // chip -Y toward the nose
    v[0] = x;  v[1] = z;  v[2] = -y;
#elif IMU_UP_AXIS == 3      // chip +Z toward the nose
    (void)x; (void)y; (void)z;
#elif IMU_UP_AXIS == -3     // chip -Z toward the nose
    v[0] = y;  v[1] = x;  v[2] = -z;
#else
#error "IMU_UP_AXIS must be +/-1 (X), +/-2 (Y) or +/-3 (Z)"
#endif
}

static void imu_remap(float *g, float *a) {
    remap3(g);
    remap3(a);
}

static void imu_cs(bool high) { gpio_put(PIN_CS_IMU, high); }

static void imu_wreg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { (uint8_t)(reg & 0x7Fu), val };
    imu_cs(false);
    spi_write_blocking(spi0, buf, 2);
    imu_cs(true);
}

static uint8_t imu_rreg(uint8_t reg) {
    // Two bytes: address out, then the data clocks back on the second.
    uint8_t tx[2] = { (uint8_t)(reg | 0x80u), 0 };
    uint8_t rx[2] = { 0, 0 };
    imu_cs(false);
    spi_write_read_blocking(spi0, tx, rx, 2);
    imu_cs(true);
    return rx[1];
}

static void imu_burst(uint8_t reg, uint8_t *dst, uint len) {
    uint8_t tx[13] = { (uint8_t)(reg | 0x80u) };
    uint8_t rx[13];
    imu_cs(false);
    spi_write_read_blocking(spi0, tx, rx, len + 1);
    imu_cs(true);
    memcpy(dst, rx + 1, len);
}

static void imu_bank(uint8_t n) { imu_wreg(ICM_ADDR_BANK_SEL, n); }

bool imu_init(void) {
    // Soft reset, then wait for the boot.
    imu_bank(0);
    imu_wreg(ICM_ADDR_DEVICE_CONFIG, 0x01);
    sleep_ms(50);

    for (int tries = 0; tries < 10; tries++) {
        if (imu_rreg(ICM_ADDR_WHO_AM_I) == ICM_VAL_WHO_AM_I) break;
        sleep_ms(10);
        if (tries == 9) return false;
    }

    // Power: gyro LN, accel LN.
    imu_bank(0);
    imu_wreg(ICM_ADDR_PWR_MGMT0, 0x0F);
    sleep_ms(50);

    // Gyro +/-2000 dps (FS_SEL 0), accel +/-16 g (FS_SEL 0), both 1 kHz ODR.
    imu_wreg(ICM_ADDR_GYRO_CONFIG0,  (0x00u << 5) | ICM_ODR_1KHZ);
    imu_wreg(ICM_ADDR_ACCEL_CONFIG0, (0x00u << 5) | ICM_ODR_1KHZ);
    sleep_ms(20);

    // Read back: a config that did not stick means a bad bus, not a flyable IMU.
    if (imu_rreg(ICM_ADDR_GYRO_CONFIG0)  != ICM_ODR_1KHZ) return false;
    if (imu_rreg(ICM_ADDR_ACCEL_CONFIG0) != ICM_ODR_1KHZ) return false;
    return true;
}

bool imu_read(imu_sample_t *out) {
    uint8_t raw[12];
    imu_burst(ICM_ADDR_ACCEL_DATAX1, raw, 12);

    // Plausibility: a disconnected/busy bus reads back all zeros or all 0xFF.
    bool all_zero = true, all_ff = true;
    for (int i = 0; i < 12; i++) {
        if (raw[i] != 0x00) all_zero = false;
        if (raw[i] != 0xFF) all_ff = false;
    }
    if (all_zero || all_ff) {
        memset(out, 0, sizeof *out);   // never hand stack garbage downstream
        out->valid = false;
        return false;
    }

    // Big endian 16-bit pairs: accel X/Y/Z then gyro X/Y/Z.
    float ax = (int16_t)((raw[0] << 8) | raw[1])  / ACCEL_LSB_PER_G;
    float ay = (int16_t)((raw[2] << 8) | raw[3])  / ACCEL_LSB_PER_G;
    float az = (int16_t)((raw[4] << 8) | raw[5])  / ACCEL_LSB_PER_G;
    float gx = (int16_t)((raw[6] << 8) | raw[7])  / GYRO_LSB_PER_DPS;
    float gy = (int16_t)((raw[8] << 8) | raw[9])  / GYRO_LSB_PER_DPS;
    float gz = (int16_t)((raw[10] << 8) | raw[11]) / GYRO_LSB_PER_DPS;

    float g[3] = { gx, gy, gz };
    float a[3] = { ax, ay, az };
    imu_remap(g, a);

    out->gx = g[0]; out->gy = g[1]; out->gz = g[2];
    out->ax = a[0]; out->ay = a[1]; out->az = a[2];
    out->valid = true;
    return true;
}
