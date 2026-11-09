#pragma once
#include <stdint.h>

typedef enum {
    ST_PADIDLE = 0,
    ST_ARMED,
    ST_BOOST,
    ST_COAST,
    ST_APOGEE,
    ST_DESCENT,
    ST_LANDED,
} flight_state_t;

// Fault / status flag bits, recorded with every log sample.
#define FF_LOGQ_DROP    (1u << 0)   // log queue overflowed, samples lost
#define FF_IMU_ERR      (1u << 1)   // IMU SPI failure in the last 200 ms
#define FF_BARO_STALE   (1u << 2)   // no fresh baro data for >500 ms
#define FF_FLASH_ERR    (1u << 3)   // flash log write failure / full
#define FF_WDT_RESET    (1u << 4)   // previous boot ended in watchdog reset
#define FF_CORE1_HB     (1u << 5)   // core 1 heartbeat lost
#define FF_ARMED        (1u << 6)   // currently armed
#define FF_CAL          (1u << 7)   // gyro bias calibration valid

// One flight log sample. 36 bytes, packed.
typedef struct __attribute__((packed)) {
    uint32_t t_ms;                   // ms since boot
    int16_t  ax_mg, ay_mg, az_mg;    // accelerometer, mg
    int16_t  gx_dps10, gy_dps10, gz_dps10; // gyro, dps * 10
    int32_t  v_cms;                  // v_hat airspeed estimate, cm/s
    uint32_t d1;                     // MS5611 pressure raw
    uint32_t d2;                     // MS5611 temperature raw
    int16_t  delta_cdeg;             // commanded roll deflection, deg * 100
    uint16_t batt_mv;                // servo rail battery, mV
    uint8_t  state;                  // flight_state_t
    uint8_t  flags;                  // FF_ bits
    uint16_t reserved;
} log_record_t;

_Static_assert(sizeof(log_record_t) == 36, "log record must be 36 bytes");

// Latest baro sample, produced on core 1.
typedef struct {
    uint32_t d1, d2;      // raw counts
    float    press_pa;    // compensated
    float    alt_m;       // standard atmosphere altitude
    uint32_t t_ms;        // when it was taken
    uint32_t seq;         // bumped on every update, staleness detection
} baro_shared_t;

// Snapshot of the control side for the USB CLI, produced on core 0.
typedef struct {
    uint8_t  state;
    int16_t  gz_dps10;
    float    v_mps;
    int16_t  delta_cdeg;
    uint16_t batt_mv;
    uint16_t flags;
} status_shared_t;
