#pragma once
#include <stdint.h>

// Implemented in main.c, used by the USB CLI (core 1).

// Snapshot bias + gains into the flash header. Called at every ARMED entry.
void avionics_log_meta_update(void);
// Flush + persist the flash header (post-flight or after an aborted flight).
void avionics_finalize_log(void);
