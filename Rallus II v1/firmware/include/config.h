#pragma once

// ---------------------------------------------------------------------------
// Pin map. Also the authoritative wiring table (mirrored in README.md).
// SPI0 belongs to core 0 (control ISR, IMU only).
// SPI1 belongs to core 1 (MS5611 + W25Q128).
// ---------------------------------------------------------------------------
#define PIN_SPI0_SCK    2
#define PIN_SPI0_MOSI   3
#define PIN_SPI0_MISO   4
#define PIN_CS_IMU      5

#define PIN_SPI1_SCK    6
#define PIN_SPI1_MOSI   7
#define PIN_SPI1_MISO   8
#define PIN_CS_BARO     9
#define PIN_CS_FLASH    10

// Servo outputs, 50 Hz hardware PWM. Fin order around the can: 0,1,2,3.
#define PIN_SERVO0      11
#define PIN_SERVO1      12
#define PIN_SERVO2      14
#define PIN_SERVO3      15

// Arm interlock: RBF pin + master switch in series to GND when SAFE.
// Internal pull-up, pin reads high only when both switch and RBF are open.
#define PIN_ARM         16

// Battery sense: 10k/2k divider from 2S LiPo to GP26 (ADC0).
#define PIN_VBAT        26
#define VBAT_DIV_R_TOP  10000.0f
#define VBAT_DIV_R_BOT  2000.0f
#define ADC_REF_MV      3300.0f

#define PIN_LED         25

// ---------------------------------------------------------------------------
// Rates. Control ISR at 1 kHz (IMU ODR 1 kHz, no averaging aliasing games),
// log every 4th cycle = 250 Hz.
// ---------------------------------------------------------------------------
#define CTRL_HZ         1000
#define CTRL_DT         (1.0f / CTRL_HZ)
#define CTRL_PERIOD_MS  1
#define LOG_DIV         4

// ---------------------------------------------------------------------------
// Flight state thresholds. Counters are derived from the millisecond values.
// ---------------------------------------------------------------------------
#define ARM_STILL_DPS       20.0f    // every axis must stay below this to arm
#define ARM_STILL_MS        3000
#define ARM_MAX_BIAS_DPS    50.0f    // refuse to fly with a crazy bias
#define ARM_MIN_VBAT_MV     6800
#define PAD_TIMEOUT_MS      300000
#define LIFTOFF_G           3.0f
#define LIFTOFF_MS          40
#define BURNOUT_G           0.5f
#define BURNOUT_MS          100
#define APOGEE_V_MPS        5.0f
#define APOGEE_BARO_DROP_M  2.0f
#define BARO_STALE_MS       500
#define LANDED_BAND_M       0.3f
#define LANDED_MS           5000
#define POST_LANDED_LOG_MS  10000
#define AUTH_V_MIN          15.0f    // fins driven only inside this window
#define AUTH_V_MAX          150.0f
#define G_MPS2              9.81f
#define V_HAT_MAX           300.0f

// ---------------------------------------------------------------------------
// Roll control. Starting gains come from sim/roll_sim.py; retune after M3.
// ROLL_DIR flips the fin response sign: set it during the ground spin test.
// ---------------------------------------------------------------------------
#define ROLL_KP             0.15f    // deg deflection per deg/s of rate error
#define ROLL_KI             0.02f    // deg per (deg/s * s)
#define ROLL_SCHED_VREF     25.0f    // full gain at/below, scaled (vref/v)^2 above
#define ROLL_MAX_DEG        15.0f
#define ROLL_MAX_RATE_DEG_S 600.0f   // commanded deflection slew limit
#define ROLL_DIR            1.0f

// ---------------------------------------------------------------------------
// Servos
// ---------------------------------------------------------------------------
#define SERVO_US_MIN        1000
#define SERVO_US_MAX        2000
#define SERVO_US_CENTER     1500
#define SERVO_US_PER_DEG    11.111f  // 1000 us across 90 deg

// ---------------------------------------------------------------------------
// Flash log
// ---------------------------------------------------------------------------
#define LOG_MAGIC           0x314E5641u  // "AVN1" little endian
#define LOG_VERSION         1u
#define FLASH_LOG_BASE      0x00001000u // sector 0 is reserved for the header
#define FLASH_SIZE_BYTES    (16u * 1024u * 1024u)
