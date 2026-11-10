#pragma once
#include <stdbool.h>

// MAX31855 K-type thermocouple amplifier on SPI1 (read-only device).
// Probe bonds to the nosecone skin near the tip (stagnation-region temp);
// a JST disconnect at the nose shoulder lets the cone come off. Temperature
// goes into every log record.

bool tc_init(void);
// Returns false on a fault frame (open/shorted probe); *c holds degC when ok.
bool tc_read(float *c);
