#pragma once
#include "datadefs.h"
#include "pico/critical_section.h"
#include <stdbool.h>
#include <stdint.h>

// Everything shared between the two cores. Producers copy whole structs under
// a critical section; both sides only ever hold the lock for a few us.
typedef struct {
    // core 1 -> core 0
    critical_section_t baro_cs;
    baro_shared_t      baro;
    // core 0 -> core 1 (CLI display)
    critical_section_t status_cs;
    status_shared_t    status;
    // core 1 heartbeat counter, bumped every loop iteration
    volatile uint32_t  hb;
    // core 1 faults, ORed into the record flags by core 0
    volatile uint16_t  faults_core1;
    // core 1 -> core 0: last battery ADC reading
    volatile uint16_t  vbat_mv;
    // init results, written by core 0 before core 1 launches, read by the CLI
    volatile bool      imu_ok;
    volatile bool      baro_ok;
    volatile bool      flash_ok;
    volatile bool      wdt_reboot;
    // set by the USB CLI (core 1): core 0 stops touching the servos
    volatile bool      bench;
    // set by core 0 state machine: stop pushing new log records
    volatile bool      log_hold;
} shared_sys_t;

extern shared_sys_t g_sys;

void shared_init(void);

void publish_baro(uint32_t d1, uint32_t d2, float press_pa, float alt_m, uint32_t t_ms);
void read_baro(baro_shared_t *out);

void publish_status(uint8_t state, int16_t gz_dps10, float v_mps,
                    int16_t delta_cdeg, uint16_t batt_mv, uint16_t flags);
void read_status(status_shared_t *out);
