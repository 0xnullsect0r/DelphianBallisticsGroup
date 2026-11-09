#pragma once
#include "datadefs.h"
#include "imu.h"
#include <stdbool.h>
#include <stdint.h>

void          state_init(void);
flight_state_t state_step(const imu_sample_t *raw, uint32_t t_ms);
flight_state_t state_get(void);
bool          state_baro_stale(void);
