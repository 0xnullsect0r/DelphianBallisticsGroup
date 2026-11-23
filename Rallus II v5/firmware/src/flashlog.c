#include "flashlog.h"
#include "config.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

#define FL_CMD_WREN       0x06
#define FL_CMD_RDSR1      0x05
#define FL_CMD_JEDEC_ID   0x9F
#define FL_CMD_READ       0x03
#define FL_CMD_PAGE_PROG  0x02
#define FL_CMD_SECT_ERASE 0x20

#define FL_JEDEX_MFR      0xEF   // Winbond
#define FL_JEDEC_MEM_16M  0x4018 // type 0x40, capacity 0x18 (16 MB)

// 24-byte header type (log_meta_t) lives in datadefs.h; shared with the
// ground decode tool.
static log_meta_t meta;
static uint32_t   meta_ckpt_offset;   // fl_offset at last header checkpoint
#define META_CKPT_BYTES (256u * 1024u)

static bool     ok;
static bool     full;
static uint32_t fl_count;
static uint32_t fl_offset;    // next absolute byte to write
static uint32_t page_addr;    // address of the first buffered byte
static uint32_t page_len;
static uint8_t  page_buf[256];

static void cs(bool high) { gpio_put(PIN_CS_FLASH, high); }

static void fl_cmd(uint8_t c) {
    cs(false);
    spi_write_blocking(spi1, &c, 1);
    cs(true);
}

static void fl_wren(void) { fl_cmd(FL_CMD_WREN); }

static uint8_t fl_rdsr1(void) {
    uint8_t tx[2] = { FL_CMD_RDSR1, 0xFF };
    uint8_t rx[2];
    cs(false);
    spi_write_read_blocking(spi1, tx, rx, 2);
    cs(true);
    return rx[1];
}

// Busy-wait for write/erase completion. Typical page program is <1 ms,
// sector erase ~45 ms; runs on core 1 where stalling the baro tick briefly
// is acceptable (log queue holds ~2 s of data at 1 kHz).
static bool fl_wait_busy(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (fl_rdsr1() & 0x01) {
        if (to_ms_since_boot(get_absolute_time()) - start > timeout_ms)
            return false;
        tight_loop_contents();
    }
    return true;
}

static void fl_read_bytes(uint32_t addr, uint8_t *dst, uint32_t len) {
    uint8_t tx[4] = { FL_CMD_READ, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    cs(false);
    spi_write_blocking(spi1, tx, 4);
    spi_read_blocking(spi1, 0xFF, dst, len);
    cs(true);
}

static bool fl_program_page(void) {
    if (page_len == 0) return true;
    fl_wren();
    uint8_t tx[4] = { FL_CMD_PAGE_PROG, (uint8_t)(page_addr >> 16), (uint8_t)(page_addr >> 8), (uint8_t)page_addr };
    cs(false);
    spi_write_blocking(spi1, tx, 4);
    spi_write_blocking(spi1, page_buf, page_len);
    cs(true);
    if (!fl_wait_busy(100)) return false;
    page_len = 0;
    return true;
}

static bool fl_erase_sector(uint32_t addr) {
    fl_wren();
    uint8_t tx[4] = { FL_CMD_SECT_ERASE, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    cs(false);
    spi_write_blocking(spi1, tx, 4);
    cs(true);
    return fl_wait_busy(500);
}

static bool fl_write_header(void) {
    if (!fl_erase_sector(0)) return false;
    meta.magic   = LOG_MAGIC;
    meta.version = LOG_VERSION;
    meta.records = fl_count;
    fl_wren();
    uint8_t tx[4] = { FL_CMD_PAGE_PROG, 0, 0, 0 };
    cs(false);
    spi_write_blocking(spi1, tx, 4);
    spi_write_blocking(spi1, (const uint8_t *)&meta, sizeof meta);
    cs(true);
    return fl_wait_busy(100);
}

void flashlog_set_meta(const log_meta_t *m) {
    meta = *m;
    meta.records = fl_count;
}

void flashlog_get_meta(log_meta_t *out) { *out = meta; }

bool flashlog_ok(void) { return ok; }

bool flashlog_init(void) {
    ok = false;
    full = false;
    fl_count = 0;
    fl_offset = FLASH_LOG_BASE;
    page_len = 0;

    // Release deep power-down then read the JEDEC ID.
    fl_cmd(0xAB);
    sleep_ms(1);
    uint8_t tx[4] = { FL_CMD_JEDEC_ID, 0xFF, 0xFF, 0xFF };
    uint8_t rx[4];
    cs(false);
    spi_write_read_blocking(spi1, tx, rx, 4);
    cs(true);
    uint32_t jedec = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
    if (jedec != ((uint32_t)FL_JEDEX_MFR << 16 | FL_JEDEC_MEM_16M)) return false;

    fl_read_bytes(0, (uint8_t *)&meta, sizeof meta);
    if (meta.magic == LOG_MAGIC && meta.version == LOG_VERSION &&
        meta.records <= (FLASH_SIZE_BYTES - FLASH_LOG_BASE) / sizeof(log_record_t)) {
        fl_count = meta.records;
        // The header count lags the data by up to one checkpoint (power cut,
        // watchdog reset in flight). Walk forward to the first blank record
        // so nothing is lost and nothing gets programmed over old data.
        uint32_t max_n = (FLASH_SIZE_BYTES - FLASH_LOG_BASE) / sizeof(log_record_t);
        while (fl_count < max_n) {
            uint8_t raw[sizeof(log_record_t)];
            fl_read_bytes(FLASH_LOG_BASE + fl_count * sizeof(log_record_t),
                          raw, sizeof raw);
            bool blank = true;
            for (uint32_t i = 0; i < sizeof raw; i++)
                if (raw[i] != 0xFF) { blank = false; break; }
            if (blank) break;
            fl_count++;
        }
    } else {
        memset(&meta, 0, sizeof meta);
        fl_count = 0;
        if (!fl_write_header()) return false;
    }
    fl_offset = FLASH_LOG_BASE + fl_count * sizeof(log_record_t);
    meta_ckpt_offset = fl_offset;
    ok = true;
    return true;
}

// True if the 4 KB sector at addr is already erased. ~3.5 ms at 10 MHz
// versus 45-400 ms for an erase: with the log erased on the ground ('e'),
// no erase ever has to run in flight and the 1 kHz stream cannot back up.
static bool fl_sector_blank(uint32_t addr) {
    uint8_t tmp[256];
    for (uint32_t off = 0; off < 4096; off += sizeof tmp) {
        fl_read_bytes(addr + off, tmp, sizeof tmp);
        for (uint32_t i = 0; i < sizeof tmp; i++)
            if (tmp[i] != 0xFF) return false;
    }
    return true;
}

// Byte-stream writer. A page program must never cross a 256-byte page
// boundary (the chip wraps inside the page and overwrites its start), and
// records are 38 bytes, so they are split across pages here.
static bool fl_put(const uint8_t *src, uint32_t len) {
    while (len) {
        if (page_len == 0) {
            page_addr = fl_offset;
            if ((page_addr & 0xFFF) == 0 && !fl_sector_blank(page_addr)) {
                if (!fl_erase_sector(page_addr)) return false;
            }
        }
        uint32_t room = 256 - ((page_addr + page_len) & 0xFF);
        uint32_t n = len < room ? len : room;
        memcpy(page_buf + page_len, src, n);
        page_len  += n;
        fl_offset += n;
        src += n;
        len -= n;
        if (((page_addr + page_len) & 0xFF) == 0) {
            if (!fl_program_page()) return false;
        }
    }
    return true;
}

bool flashlog_append(const log_record_t *r) {
    if (!ok) return false;
    if (full) return false;
    if (fl_offset + sizeof(log_record_t) > FLASH_SIZE_BYTES) {
        full = true;
        return false;
    }

    if (!fl_put((const uint8_t *)r, sizeof *r)) { ok = false; return false; }
    fl_count++;

    // Checkpoint the header every 256 KB so a power loss mid-flight costs at
    // most ~7k records of the count (and those are recovered by the scan at
    // the next boot). The 45 ms sector erase blocks this core briefly; the
    // log queue rides it out.
    if (fl_offset - meta_ckpt_offset >= META_CKPT_BYTES) {
        meta_ckpt_offset = fl_offset;
        if (!fl_write_header()) { ok = false; return false; }
    }
    return true;
}

void flashlog_flush(void) {
    if (ok && page_len > 0) {
        if (!fl_program_page()) ok = false;
    }
}

void flashlog_finalize(void) {
    flashlog_flush();
    if (ok) {
        if (!fl_write_header()) ok = false;
    }
}

uint32_t flashlog_count(void)     { return fl_count; }
uint32_t flashlog_used_bytes(void) { return ok ? fl_offset : 0; }

bool flashlog_read(uint32_t idx, log_record_t *out) {
    if (!ok || idx >= fl_count) return false;
    fl_read_bytes(FLASH_LOG_BASE + idx * sizeof(log_record_t),
                  (uint8_t *)out, sizeof *out);
    return true;
}

bool flashlog_erase_used(void) {
    if (!ok) return false;
    flashlog_flush();
    uint32_t end = fl_offset | 0xFFF;   // erase through the end of the last sector
    uint32_t done = 0;
    uint32_t total = (end - FLASH_LOG_BASE) / 4096;
    for (uint32_t a = FLASH_LOG_BASE; a <= end; a += 4096) {
        if (!fl_erase_sector(a)) { ok = false; return false; }
        if (++done % 128 == 0)
            printf("erase %lu/%lu sectors\n", done, total);
    }
    fl_count = 0;
    fl_offset = FLASH_LOG_BASE;
    page_len = 0;
    full = false;
    return fl_write_header();
}
