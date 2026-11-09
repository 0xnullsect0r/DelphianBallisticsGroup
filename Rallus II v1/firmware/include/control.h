#pragma once
#include "datadefs.h"
#include "imu.h"
#include <stdbool.h>
#include <stdint.h>

// Roll controller + v_hat estimator. Runs once per control tick on core 0.

void  ctrl_reset(void);   // zero integrator, v_hat, slew state (keeps bias)

// Bias calibration, driven by the state machine during the stillness window.
void  ctrl_bias_begin(void);
void  ctrl_bias_sample(float gx, float gy, float gz);
bool  ctrl_bias_commit(void);   // false if the collected bias is implausible
bool  ctrl_bias_valid(void);

int16_t ctrl_step(flight_state_t st, const imu_sample_t *raw, uint32_t t_ms);
float   ctrl_v_hat(void);
