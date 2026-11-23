#include "state.h"
#include "config.h"
#include "control.h"
#include "pyro.h"
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
static float    pad_alt;
static float    v_peak;
static uint32_t liftoff_at_ms;
static uint32_t coast_at_ms;
static uint32_t baro_seq_seen;
static uint32_t baro_drop_cnt;
static uint32_t last_climb_ms;     // last time the baro made a new maximum
static float    alt_hist[3];       // median-of-3 spike filter
static float    alt_f;

static float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return a > b ? a : b;
}
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

    // Median of the last three baro samples: a single wild reading (ejection
    // gas, a pressure transient, a flipped bit) never reaches max_alt.
    static uint32_t filt_seq;
    if (baro.seq != filt_seq) {
        filt_seq = baro.seq;
        alt_hist[0] = alt_hist[1];
        alt_hist[1] = alt_hist[2];
        alt_hist[2] = baro.alt_m;
        alt_f = (baro.seq < 3) ? baro.alt_m
                               : median3(alt_hist[0], alt_hist[1], alt_hist[2]);
    }
    float alt = alt_f;
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
                        pad_alt = alt;
                        v_peak = 0.0f;
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
        // Track the pad altitude while waiting (weather drifts the baro).
        if (baro_fresh) { pad_alt = alt; max_alt = alt; }
        if (raw->valid && raw->az > LIFTOFF_G) {
            if (++liftoff_cnt >= LIFTOFF_N) {
                st = ST_BOOST;
                burnout_cnt = 0;
                liftoff_at_ms = t_ms;
            }
        } else {
            liftoff_cnt = 0;
        }
        break;

    case ST_BOOST:
        if (alt > max_alt) { max_alt = alt; last_climb_ms = t_ms; }
        if (v_hat > v_peak) v_peak = v_hat;
        if (raw->valid && raw->az < BURNOUT_G) {
            if (++burnout_cnt >= BURNOUT_N) {
                st = ST_COAST;
                burnout_cnt = 0;
                coast_at_ms = t_ms;
                baro_drop_cnt = 0;
                baro_seq_seen = baro.seq;
            }
        } else {
            burnout_cnt = 0;
        }
        break;

    case ST_COAST:
        if (alt > max_alt) { max_alt = alt; last_climb_ms = t_ms; }
        if (v_hat > v_peak) v_peak = v_hat;
        // Baro vote: the drop must hold across several NEW samples, so one
        // noisy reading or a pressure transient cannot call apogee.
        if (baro.seq != baro_seq_seen) {
            baro_seq_seen = baro.seq;
            if (alt < max_alt - APOGEE_BARO_DROP_M) baro_drop_cnt++;
            else                                    baro_drop_cnt = 0;
        }
        // Two sensors have to agree before the bolts go:
        //  - baro says "descending" AND the inertial speed is at least low
        //    (v_hat only ever errs low: accel clip, off-vertical flight), or
        //  - inertial says "stopped" AND the baro is not still climbing
        //    (or is dead / vent blocked), or
        //  - the coast failsafe timer ran out.
        bool baro_apogee = baro_fresh &&
                           baro_drop_cnt >= APOGEE_BARO_SAMPLES &&
                           v_hat < APOGEE_BARO_VMAX;
        bool inert_apogee = v_hat < APOGEE_V_MPS &&
                            (!baro_fresh ||
                             t_ms - last_climb_ms >= APOGEE_NOCLIMB_MS);
        if (baro_apogee || inert_apogee ||
            t_ms - coast_at_ms >= COAST_TIMEOUT_MS) {
            st = ST_APOGEE;
            apogee_at_ms = t_ms;
            // v5: the Pico fires the separation charge at its own apogee
            // detection, but only if this was plausibly a real flight. The
            // altimeter stays in the loop as backup either way.
            bool flew = (t_ms - liftoff_at_ms >= PYRO_LOCKOUT_MS) &&
                        ((max_alt - pad_alt >= PYRO_MIN_ALT_M) ||
                         (v_peak >= PYRO_MIN_VPEAK));
            if (flew) {
                pyro_fire(t_ms);
                g_sys.events |= EV_PYRO_FIRED;
            } else {
                // Not a flight (bump, drop, dud): flag it and go back to the
                // pad sequence so the rocket re-arms itself instead of
                // sitting in LANDED, deaf to the real launch.
                g_sys.events |= EV_PYRO_INHIBIT;
                st = ST_PADIDLE;
                quiet_run = false;
                ctrl_reset();
            }
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
