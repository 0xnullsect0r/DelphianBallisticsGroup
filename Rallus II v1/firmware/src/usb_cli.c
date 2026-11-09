#include "usb_cli.h"
#include "config.h"
#include "flashlog.h"
#include "logbuf.h"
#include "ms5611.h"
#include "servos.h"
#include "shared.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include <stdio.h>

static const char *state_name[7] = {
    "PADIDLE", "ARMED", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED"
};

static bool     streaming;
static uint32_t last_stream_ms;

static uint32_t crc32_update(uint32_t c, const uint8_t *p, uint32_t n) {
    c = ~c;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1)));
    }
    return ~c;
}

static void print_menu(void) {
    printf("\nfin-roll avionics\n");
    printf("  h  help\n");
    printf("  s  status snapshot\n");
    printf("  i  toggle 10Hz live stream\n");
    printf("  w  bench servo sweep (PADIDLE only)\n");
    printf("  d  dump flight log (binary)\n");
    printf("  e  erase flight log\n");
    printf("  f  finalize log header (after a flight without clean landing)\n");
    printf("  b  battery\n");
    printf("  x  reboot to BOOTSEL\n");
}

static void print_status(void) {
    status_shared_t st;
    read_status(&st);
    baro_shared_t b;
    read_baro(&b);
    printf("state=%s wz=%.1fdps v=%.1fm/s d=%+.2fdeg vb=%umV\n",
           state_name[st.state & 7],
           st.gz_dps10 * 0.1f, st.v_mps, st.delta_cdeg * 0.01f, st.batt_mv);
    printf("baro seq=%lu p=%.0fPa alt=%.1fm  log=%lu recs, q=%lu, dropped=%lu, flash=%s promcrc=%s\n",
           (unsigned long)b.seq, b.press_pa, b.alt_m,
           (unsigned long)flashlog_count(), (unsigned long)logq_count(),
           (unsigned long)logq_dropped(),
           flashlog_ok() ? "ok" : "ERR",
           ms5611_prom_crc_ok() ? "ok" : "bad");
}

static void bench_sweep(void) {
    status_shared_t st;
    read_status(&st);
    if (st.state != ST_PADIDLE) {
        printf("refused: bench tests only in PADIDLE\n");
        return;
    }
    g_sys.bench = true;
    printf("sweep +10, -10, center\n");
    for (float a = 0.0f; a <= 10.0f; a += 1.0f) { servos_write_roll(a);  sleep_ms(40); }
    for (float a = 10.0f; a >= -10.0f; a -= 1.0f) { servos_write_roll(a); sleep_ms(40); }
    for (float a = -10.0f; a <= 0.0f; a += 1.0f) { servos_write_roll(a); sleep_ms(40); }
    servos_write_roll(0.0f);
    g_sys.bench = false;
    printf("sweep done\n");
}

static void cmd_dump(void) {
    uint32_t n = flashlog_count();
    printf("!DUMP %lu\n", (unsigned long)n);

    uint32_t hdr[3] = { LOG_MAGIC, LOG_VERSION, n };
    fwrite(hdr, 4, 3, stdout);
    uint32_t crc = crc32_update(0, (const uint8_t *)hdr, 12);

    log_record_t rec;
    for (uint32_t i = 0; i < n; i++) {
        if (!flashlog_read(i, &rec)) {
            printf("\n!READERR at %lu\n", (unsigned long)i);
            return;
        }
        fwrite(&rec, sizeof rec, 1, stdout);
        crc = crc32_update(crc, (const uint8_t *)&rec, sizeof rec);
    }

    fwrite(&crc, 4, 1, stdout);
    fflush(stdout);
    printf("\n!END crc=%08lx n=%lu\n", (unsigned long)crc, (unsigned long)n);
}

void usb_cli_init(void) {
    printf("\n=== fin-roll avionics, build %s %s ===\n", __DATE__, __TIME__);
    printf("imu=%s baro=%s (prom crc %s) flash=%s prev-reset=%s\n",
           g_sys.imu_ok ? "ok" : "ERR",
           g_sys.baro_ok ? "ok" : "ERR",
           ms5611_prom_crc_ok() ? "ok" : "bad",
           g_sys.flash_ok ? "ok" : "ERR",
           g_sys.wdt_reboot ? "WDT" : "power/other");
    printf("flash log: %lu records, %lu KB used\n",
           (unsigned long)flashlog_count(),
           (unsigned long)(flashlog_used_bytes() / 1024));
    print_menu();
}

void usb_cli_poll(uint32_t now_ms) {
    if (streaming && now_ms - last_stream_ms >= 100) {
        last_stream_ms = now_ms;
        status_shared_t st;
        read_status(&st);
        printf("st=%s wz=%.1f v=%.1f d=%+.2f vb=%u fl=0x%04x\n",
               state_name[st.state & 7],
               st.gz_dps10 * 0.1f, st.v_mps, st.delta_cdeg * 0.01f,
               st.batt_mv, st.flags);
    }

    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) return;

    switch (c) {
    case 'h': print_menu(); break;
    case 's': print_status(); break;
    case 'i': streaming = !streaming; break;
    case 'w': bench_sweep(); break;
    case 'd': cmd_dump(); break;
    case 'e':
        printf("erasing...\n");
        if (flashlog_erase_used()) printf("erase done\n");
        else printf("erase FAILED\n");
        break;
    case 'f':
        flashlog_finalize();
        printf("finalized at %lu records\n", (unsigned long)flashlog_count());
        break;
    case 'b': printf("vbatt %u mV\n", g_sys.vbat_mv); break;
    case 'x':
        printf("rebooting to BOOTSEL\n");
        sleep_ms(100);
        reset_usb_boot(0, 0);
        break;
    default: break;
    }
}
