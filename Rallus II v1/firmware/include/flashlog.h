#pragma once
#include "datadefs.h"
#include <stdbool.h>
#include <stdint.h>

// W25Q128 flight log on SPI1, written by core 1.
// Layout: sector 0 = header, records start at FLASH_LOG_BASE, 36 bytes each,
// pages packed sequentially, sectors erased one ahead of the write pointer.

bool     flashlog_init(void);
bool     flashlog_ok(void);
bool     flashlog_append(const log_record_t *r);
void     flashlog_flush(void);      // program the partial page out
void     flashlog_finalize(void);   // flush + persist the header
uint32_t flashlog_count(void);
uint32_t flashlog_used_bytes(void);
bool     flashlog_read(uint32_t idx, log_record_t *out);
bool     flashlog_erase_used(void); // erase the used span, reset to empty
