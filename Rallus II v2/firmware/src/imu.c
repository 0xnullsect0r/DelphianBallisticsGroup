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
#define ICM_ADDR_GYRO_CONFIG0    0x01   // bank 2
#define ICM_ADDR_ACCEL_CONFIG0   0x03   // bank 2

// Sensitivities for the configured ranges: gyro +/-2000 dps, accel +/-32 g.
#define GYRO_LSB_PER_DPS         16.4f
#define ACCEL_LSB_PER_G          1024.0f

// Mount remap: change here if the Pico sits in the sled rotated. Default:
// chip +Z (top face of the Pico) points up the rocket axis, so chip gz is
// the body roll rate.
static void imu_remap(float *g, float *a) {
    (void)g; (void)a;  // identity
}

static void imu_cs(bool high) { gpio_put(PIN_CS_IMU, high); }

static void imu_wreg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { (uint8_t)(reg & 0x7Fu), val };
    imu_cs(false);
    spi_write_blocking(spi0, buf, 2);
    imu_cs(true);
}

static uint8_t imu_rreg(uint8_t reg) {
    uint8_t tx[1] = { (uint8_t)(reg | 0x80u) };
    uint8_t rx[1] = { 0 };
    imu_cs(false);
    spi_write_read_blocking(spi0, tx, rx, 1);
    imu_cs(true);
    return rx[0];
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

    imu_bank(2);
    // ODR 1 kHz (0b101 << 5), UI filter bypass (0b000 << 2), gyro +/-2000 dps (0b00).
    imu_wreg(ICM_ADDR_GYRO_CONFIG0, (0x05u << 5) | (0x00u << 2) | 0x00u);
    // ODR 1 kHz, filter bypass, accel +/-32 g (0b10).
    imu_wreg(ICM_ADDR_ACCEL_CONFIG0, (0x05u << 5) | (0x00u << 2) | 0x02u);
    imu_bank(0);
    sleep_ms(20);
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
