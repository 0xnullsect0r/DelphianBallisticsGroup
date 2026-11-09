#include "state.h"
#include "config.h"
#include "control.h"
#include "shared.h"
#include "pico/stdlib.h"
#include <math.h>

static flight_state_t st = ST_PADIDLE;

// Threshold counters, converted from milliseconds at compile time.
#define LIFTOFF_N   ((LIFTOFF_MS)     / (CTRL_PERIOD_MS))
#define BURNOUT_N   ((BURNOUT_MS)     / (CTRL_PERIOD_MS))

static uint32_t liftoff_cnt;
static uint32_t burnout_cnt;

static uint32_t quiet_since_ms;    // start of the current stillness run
static bool     quiet_run;
static uint32_t armed_at_ms;

static float    max_alt = -1e9f;
static uint32_t apogee_at_ms;
static float    alt_at_check;
static uint32_t landed_check_ms;
static uint32_t landed_at_ms;

static bool     baro_sticky_stale;

static bool arm_pin_set(void) { return gpio_get(PIN_ARM); }

static bool still(const imu_sample_t *s) {
    float m = sqrtf(s->gx * s->gx + s->gy * s->gy + s->gz * s->gz);
    return m < ARM_STILL_DPS;
}

flight_state_t state_get(void) { return st; }

void state_init(void) {
    st = ST_PADIDLE;
    liftoff_cnt = burnout_cnt = 0;
    quiet_run = false;
    max_alt = -1e9f;
    baro_sticky_stale = false;
    g_sys.log_hold = false;
}

flight_state_t state_step(const imu_sample_t *raw, uint32_t t_ms) {
    baro_shared_t baro;
    read_baro(&baro);
    bool baro_fresh = (t_ms - baro.t_ms) < BARO_STALE_MS;
    if (!baro_fresh) baro_sticky_stale = true;

    float alt = baro.alt_m;
    float v_hat = ctrl_v_hat();

    switch (st) {
    case ST_PADIDLE:
        if (arm_pin_set()) {
            if (g_sys.vbat_mv >= ARM_MIN_VBAT_MV && raw->valid) {
                if (!quiet_run) {
                    quiet_run = true;
                    quiet_since_ms = t_ms;
                    ctrl_bias_begin();
                }
                ctrl_bias_sample(raw->gx, raw->gy, raw->gz);
                if (!still(raw)) {
                    // Movement restarts the window.
                    quiet_run = false;
                } else if (t_ms - quiet_since_ms >= ARM_STILL_MS) {
                    if (ctrl_bias_commit()) {
                        st = ST_ARMED;
                        armed_at_ms = t_ms;
                        max_alt = alt;
                        liftoff_cnt = burnout_cnt = 0;
                    } else {
                        quiet_run = false;   // absurd bias, try again
                    }
                }
            }
        } else {
            quiet_run = false;
            ctrl_reset();
        }
        break;

    case ST_ARMED:
        if (!arm_pin_set()) {
            st = ST_PADIDLE;
            ctrl_reset();
            break;
        }
        if (t_ms - armed_at_ms > PAD_TIMEOUT_MS) {
            st = ST_PADIDLE;
            ctrl_reset();
            break;
        }
        if (raw->az > LIFTOFF_G) {
            if (++liftoff_cnt >= LIFTOFF_N) { st = ST_BOOST; burnout_cnt = 0; }
        } else {
            liftoff_cnt = 0;
        }
        break;

    case ST_BOOST:
        if (alt > max_alt) max_alt = alt;
        if (raw->az < BURNOUT_G) {
            if (++burnout_cnt >= BURNOUT_N) { st = ST_COAST; burnout_cnt = 0; }
        } else {
            burnout_cnt = 0;
        }
        break;

    case ST_COAST:
        if (alt > max_alt) max_alt = alt;
        if (v_hat < APOGEE_V_MPS ||
            (baro_fresh && alt < max_alt - APOGEE_BARO_DROP_M)) {
            st = ST_APOGEE;
            apogee_at_ms = t_ms;
        }
        break;

    case ST_APOGEE:
        if (t_ms - apogee_at_ms >= 250) {
            st = ST_DESCENT;
            alt_at_check = alt;
            landed_check_ms = t_ms;
        }
        break;

    case ST_DESCENT:
        if (t_ms - landed_check_ms >= LANDED_MS) {
            if (fabsf(alt - alt_at_check) < LANDED_BAND_M) {
                st = ST_LANDED;
                landed_at_ms = t_ms;
            } else {
                alt_at_check = alt;
                landed_check_ms = t_ms;
            }
        }
        break;

    case ST_LANDED:
        if (t_ms - landed_at_ms >= POST_LANDED_LOG_MS)
            g_sys.log_hold = true;
        break;
    }

    return st;
}

bool state_baro_stale(void) { return baro_sticky_stale; }
