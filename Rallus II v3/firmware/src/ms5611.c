#include "ms5611.h"
#include "config.h"
#include "shared.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <math.h>
#include <string.h>

// MS5611-01BA on SPI1, mode 0. Datasheet compensation, first and second
// order (the sub -1500 season correction is omitted: never that cold in
// flight, noted for completeness).

#define MS_CMD_RESET        0x1E
#define MS_CMD_CONV_D1_4096 0x48
#define MS_CMD_CONV_D2_4096 0x58
#define MS_CMD_ADC_READ     0x00
#define MS_CMD_PROM(n)      (0xA0u | ((uint16_t)(n) << 1))
#define MS_CONV_MS          10   // OSR 4096 conversion time, 9.04 ms max

static uint16_t prom[8];   // C1..C6 live in prom[1]..prom[6]
static bool     present;
static bool     crc_ok;

typedef enum { MS_START, MS_WAIT_D1, MS_START_D2, MS_WAIT_D2 } ms_phase_t;
static ms_phase_t phase = MS_START;
static uint32_t   ready_ms;
static uint32_t   d1_raw, d2_raw;

static void cs(bool high) { gpio_put(PIN_CS_BARO, high); }

static void ms_cmd(uint8_t c) {
    cs(false);
    spi_write_blocking(spi1, &c, 1);
    cs(true);
}

static uint32_t ms_read24(void) {
    uint8_t tx[4] = { MS_CMD_ADC_READ, 0xFF, 0xFF, 0xFF };
    uint8_t rx[4];
    cs(false);
    spi_write_read_blocking(spi1, tx, rx, 4);
    cs(true);
    return ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
}

static uint16_t ms_prom_read(int n) {
    uint8_t tx[3] = { (uint8_t)MS_CMD_PROM(n), 0xFF, 0xFF };
    uint8_t rx[3];
    cs(false);
    spi_write_read_blocking(spi1, tx, rx, 3);
    cs(true);
    return ((uint16_t)rx[1] << 8) | rx[2];
}

// AN520 CRC-4. The nibble is stored in bits 15..12 of PROM word 7. If this
// ever mismatches on a good sensor (some clones botch the PROM image), the
// real sanity check is pressure reading within a few hPa of the local
// weather station at the first bench run.
static uint8_t ms_crc4(const uint16_t p[8]) {
    uint16_t q[8];
    memcpy(q, p, sizeof q);
    q[7] &= 0x0FFF;   // remove the stored CRC from the data
    uint16_t n_rem = 0;
    for (int cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 1) n_rem ^= (uint16_t)(q[cnt >> 1] & 0x00FF);
        else              n_rem ^= (uint16_t)(q[cnt >> 1] >> 8);
        for (int bit = 8; bit > 0; bit--) {
            if (n_rem & 0x8000) n_rem = (uint16_t)((n_rem << 1) ^ 0x3000);
            else                n_rem = (uint16_t)(n_rem << 1);
        }
    }
    return (uint8_t)((n_rem >> 12) & 0x000F);
}

static void ms_compensate(uint32_t d1, uint32_t d2,
                          float *press_pa, float *temp_c) {
    int64_t dt = (int64_t)d2 - ((int64_t)prom[5] << 8);
    int64_t temp = 2000 + ((dt * (int64_t)prom[6]) >> 23);
    int64_t off  = ((int64_t)prom[2] << 16) + ((dt * (int64_t)prom[4]) >> 7);
    int64_t sens = ((int64_t)prom[1] << 15) + ((dt * (int64_t)prom[3]) >> 8);
    int64_t p    = (((int64_t)d1 * sens) >> 21) - off;
    p >>= 15;

    if (temp < 2000) {
        int64_t t2    = (dt * dt) >> 31;
        int64_t off2  = (61 * (temp - 2000) * (temp - 2000)) >> 4;
        int64_t sens2 = 2 * (temp - 2000) * (temp - 2000);
        // temp < -1500 (-150 degC) extra correction intentionally omitted.
        temp -= t2;
        off  -= off2;
        sens -= sens2;
    }

    *press_pa = (float)p;
    *temp_c   = (float)temp * 0.01f;
}

bool ms5611_prom_crc_ok(void) { return crc_ok; }

bool ms5611_init(void) {
    present = false;
    crc_ok = false;

    ms_cmd(MS_CMD_RESET);
    sleep_ms(5);

    for (int i = 0; i < 8; i++) prom[i] = ms_prom_read(i);

    // Zero words across the board means no chip on the bus.
    bool any = false;
    for (int i = 0; i < 8; i++) if (prom[i] != 0) any = true;
    if (!any) return false;
    present = true;

    crc_ok = (ms_crc4(prom) == (uint8_t)((prom[7] >> 12) & 0x000F));

    // Prime one conversion so the shared struct has data quickly.
    ms_cmd(MS_CMD_CONV_D1_4096);
    phase = MS_WAIT_D1;
    ready_ms = to_ms_since_boot(get_absolute_time()) + MS_CONV_MS;
    return true;
}

void ms5611_tick(uint32_t now_ms) {
    if (!present) return;

    switch (phase) {
    case MS_WAIT_D1:
        if ((int32_t)(now_ms - ready_ms) < 0) break;
        d1_raw = ms_read24();
        ms_cmd(MS_CMD_CONV_D2_4096);
        phase = MS_WAIT_D2;
        ready_ms = now_ms + MS_CONV_MS;
        break;

    case MS_WAIT_D2:
        if ((int32_t)(now_ms - ready_ms) < 0) break;
        d2_raw = ms_read24();
        {
            float press, temp_c;
            ms_compensate(d1_raw, d2_raw, &press, &temp_c);
            float alt = 44330.0f * (1.0f - powf(press / 101325.0f, 0.190295f));
            publish_baro(d1_raw, d2_raw, press, alt, now_ms);
        }
        ms_cmd(MS_CMD_CONV_D1_4096);
        phase = MS_WAIT_D1;
        ready_ms = now_ms + MS_CONV_MS;
        break;

    default:
        ms_cmd(MS_CMD_CONV_D1_4096);
        phase = MS_WAIT_D1;
        ready_ms = now_ms + MS_CONV_MS;
        break;
    }
}
