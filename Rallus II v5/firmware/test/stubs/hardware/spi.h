#pragma once
#include <stdint.h>
#include <stddef.h>
typedef struct spi_inst spi_inst_t;
extern spi_inst_t *spi1;
int spi_write_blocking(spi_inst_t*, const uint8_t*, size_t);
int spi_read_blocking(spi_inst_t*, uint8_t, uint8_t*, size_t);
int spi_write_read_blocking(spi_inst_t*, const uint8_t*, uint8_t*, size_t);
