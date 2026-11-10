#include "tc.h"
#include "config.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdint.h>

// MAX31855: read-only SPI. 32-bit frame:
//   D31..D18  14-bit thermocouple temp, 0.25 C/bit, two's complement
//   D16       fault flag
//   D2/D1/D0  SCV / SCG / OC fault codes
// Conversion rate is ~5 Hz internally; polling slower than that is fine.

bool tc_init(void) {
    gpio_init(PIN_CS_TC);
    gpio_set_dir(PIN_CS_TC, GPIO_OUT);
    gpio_put(PIN_CS_TC, 1);
    return true;
}

bool tc_read(float *c) {
    uint8_t tx[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
    uint8_t rx[4];
    gpio_put(PIN_CS_TC, false);
    spi_write_read_blocking(spi1, tx, rx, 4);
    gpio_put(PIN_CS_TC, true);

    uint32_t frame = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16)
                   | ((uint32_t)rx[2] << 8) | rx[3];

    if (frame & (1u << 16)) return false;   // any fault: open/short/GND

    // Arithmetic right shift sign-extends the 14-bit value (gcc semantics,
    // used deliberately).
    int32_t raw = (int32_t)frame >> 18;
    *c = (float)raw * 0.25f;
    return true;
}
