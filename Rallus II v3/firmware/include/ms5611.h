#pragma once
#include <stdbool.h>
#include <stdint.h>

bool ms5611_init(void);              // reset, PROM read, CRC check, report via status
bool ms5611_prom_crc_ok(void);
// Non-blocking conversion state machine. Call from the core 1 loop as fast
// as convenient; conversions take 10 ms each, full P/T pair at ~50 Hz.
void ms5611_tick(uint32_t now_ms);
