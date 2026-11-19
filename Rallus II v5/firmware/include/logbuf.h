#pragma once
#include "datadefs.h"
#include <stdbool.h>
#include <stdint.h>

#define LOGQ_BITS   11
#define LOGQ_SIZE   (1u << LOGQ_BITS)   // 2048 records, 8 s of buffering at 250 Hz

void     logq_init(void);
bool     logq_push(const log_record_t *rec);   // producer (core 0), false if full
bool     logq_pop(log_record_t *out);          // consumer (core 1)
uint32_t logq_count(void);
uint32_t logq_dropped(void);
