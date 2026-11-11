#pragma once
#include <stdbool.h>
#include <stdint.h>

void servos_init(void);
void servos_center(void);
// Differential roll command: positive delta_deg tilts fins 0,2 positive and
// fins 1,3 negative. Clamped to ROLL_MAX_DEG and the pulse limits.
void servos_write_roll(float delta_deg);
// Direct pulse write for bench tests (us).
void servos_write_us(uint8_t i, uint16_t us);
