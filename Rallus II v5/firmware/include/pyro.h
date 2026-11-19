#pragma once
#include <stdbool.h>
#include <stdint.h>

// v5 separation-charge channel. The Pico fires the explosive bolts at its
// own apogee detection; the commercial altimeter remains connected as a
// backup on a later setting.

void pyro_init(void);            // FET off, ADC channel ready
bool pyro_cont_ok(void);         // true = e-match continuity reads good
void pyro_fire(uint32_t now_ms); // start the fire pulse (core 0, apogee)
void pyro_tick(uint32_t now_ms); // end the pulse when it expires (core 1)
