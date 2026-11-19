#include "control.h"
#include "config.h"
#include "servos.h"
#include "shared.h"
#include <math.h>

static float    bias[3];        // dps
static bool     bias_valid;

static float    bias_acc[3];
static uint32_t bias_n;

static float    v_hat;          // m/s
static float    integ;          // deg
static float    prev_delta;     // deg, for slew limiting

void ctrl_reset(void) {
    v_hat      = 0.0f;
    integ      = 0.0f;
    prev_delta = 0.0f;
}

void ctrl_bias_begin(void) {
    bias_acc[0] = bias_acc[1] = bias_acc[2] = 0.0f;
    bias_n = 0;
}

void ctrl_bias_sample(float gx, float gy, float gz) {
    bias_acc[0] += gx;
    bias_acc[1] += gy;
    bias_acc[2] += gz;
    bias_n++;
}

bool ctrl_bias_commit(void) {
    if (bias_n == 0) return false;
    float n = (float)bias_n;
    float b0 = bias_acc[0] / n;
    float b1 = bias_acc[1] / n;
    float b2 = bias_acc[2] / n;
    float mag = sqrtf(b0 * b0 + b1 * b1 + b2 * b2);
    if (mag > ARM_MAX_BIAS_DPS) return false;   // moving rocket, refuse to arm
    bias[0] = b0;
    bias[1] = b1;
    bias[2] = b2;
    bias_valid = true;
    return true;
}

bool ctrl_bias_valid(void) { return bias_valid; }

void ctrl_get_bias(float *gx, float *gy, float *gz) {
    *gx = bias[0];
    *gy = bias[1];
    *gz = bias[2];
}

float ctrl_v_hat(void) { return v_hat; }

// Deflection ceiling vs airspeed: full travel low and slow, tapering to a
// floor through the transonic band (see config.h).
static float delta_cap(float v) {
    if (v <= DEFLECT_FULL_V) return ROLL_MAX_DEG;
    if (v >= DEFLECT_MIN_V)  return DEFLECT_MIN_DEG;
    float f = (v - DEFLECT_FULL_V) / (DEFLECT_MIN_V - DEFLECT_FULL_V);
    return ROLL_MAX_DEG + f * (DEFLECT_MIN_DEG - ROLL_MAX_DEG);
}

int16_t ctrl_step(flight_state_t st, const imu_sample_t *raw, uint32_t t_ms) {
    (void)t_ms;

    // Bias-corrected sensors. On an implausible sample, hold neutral: better
    // fixed fins than fins driven by garbage.
    float gz = raw->gz - bias[2];
    float az_g = raw->az;

    // v_hat: proper axial accel minus gravity, clamped. Vertical-flight
    // approximation, good enough for gain scheduling and apogee cross-check.
    v_hat += (az_g * G_MPS2 - G_MPS2) * CTRL_DT;
    if (v_hat < 0.0f)    v_hat = 0.0f;
    if (v_hat > V_HAT_MAX) v_hat = V_HAT_MAX;

    bool authority = (st == ST_BOOST || st == ST_COAST)
                  && raw->valid
                  && v_hat >= AUTH_V_MIN && v_hat <= AUTH_V_MAX;

    float delta = 0.0f;
    if (authority) {
        float e = -gz;   // target roll rate is zero
        float sched = (v_hat > ROLL_SCHED_VREF)
                    ? (ROLL_SCHED_VREF * ROLL_SCHED_VREF) / (v_hat * v_hat)
                    : 1.0f;
        float kp = ROLL_KP * sched;
        float ki = ROLL_KI * sched;
        float cap = delta_cap(v_hat);

        float unsat = kp * e + integ;
        // Conditional anti-windup: only integrate when not railing.
        if (fabsf(unsat) <= cap)
            integ += ki * e * CTRL_DT;

        delta = kp * e + integ;
        if (delta >  cap)  delta =  cap;
        if (delta < -cap) delta = -cap;
    } else {
        integ = 0.0f;
        delta = 0.0f;
    }

    delta *= ROLL_DIR;

    // Slew limit the command so the linkage never snaps across full range.
    float max_step = ROLL_MAX_RATE_DEG_S * CTRL_DT;
    if (delta - prev_delta >  max_step) delta = prev_delta + max_step;
    if (delta - prev_delta < -max_step) delta = prev_delta - max_step;
    prev_delta = delta;

    if (!g_sys.bench)
        servos_write_roll(delta);

    return (int16_t)(delta * 100.0f);
}
