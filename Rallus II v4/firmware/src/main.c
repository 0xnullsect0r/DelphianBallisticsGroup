#include "config.h"
#include "control.h"
#include "datadefs.h"
#include "flashlog.h"
#include "imu.h"
#include "logbuf.h"
#include "main.h"
#include "ms5611.h"
#include "servos.h"
#include "shared.h"
#include "state.h"
#include "tc.h"
#include "usb_cli.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static repeating_timer_t ctrl_timer;
static uint32_t imu_err_streak;
static uint16_t flags_core0;
static uint32_t log_div_cnt;
static flight_state_t log_last_st = ST_PADIDLE;
static uint32_t descent_entry_ms;

static uint16_t read_vbat(void) {
    adc_select_input(0);
    uint16_t raw = adc_read();
    float mv = (float)raw * ADC_REF_MV / 4095.0f
             * (VBAT_DIV_R_TOP + VBAT_DIV_R_BOT) / VBAT_DIV_R_BOT;
    return (uint16_t)mv;
}

static void led_update(uint32_t now_ms) {
    status_shared_t st;
    read_status(&st);
    bool on;
    switch (st.state & 7) {
    case ST_PADIDLE: on = (now_ms / 500) % 2 == 0; break;          // slow
    case ST_ARMED:   on = (now_ms / 125) % 2 == 0; break;          // fast
    case ST_LANDED:  on = (now_ms / 250) % 4 != 3; break;          // double-ish
    default:         on = true; break;                             // flight: solid
    }
    gpio_put(PIN_LED, on);
}

// Snapshot what the controller knows into the flash header: gyro bias,
// gains, cal status. Called on every entry into ARMED; handed to core 1 via
// the shared section, which owns all flash access.
void avionics_log_meta_update(void) {
    log_meta_t m;
    memset(&m, 0, sizeof m);
    float bx, by, bz;
    ctrl_get_bias(&bx, &by, &bz);
    m.bias_gx10 = (int16_t)(bx * 10.0f);
    m.bias_gy10 = (int16_t)(by * 10.0f);
    m.bias_gz10 = (int16_t)(bz * 10.0f);
    m.kp_x10000 = (uint16_t)(ROLL_KP * 10000.0f);
    m.ki_x10000 = (uint16_t)(ROLL_KI * 10000.0f);
    m.meta_flags = ctrl_bias_valid() ? LOG_META_BIAS_VALID : 0;
    request_log_meta(&m);
}

void avionics_finalize_log(void) {
    flashlog_finalize();
}

// ---------------------------------------------------------------------------
// Core 1: baro, flash log, USB, LED. Never touches SPI0 or the servos.
// ---------------------------------------------------------------------------
static void core1_main(void) {
    g_sys.baro_ok = ms5611_init();
    g_sys.flash_ok = flashlog_init();
    if (!g_sys.flash_ok) g_sys.faults_core1 |= FF_FLASH_ERR;
    tc_init();

    usb_cli_init();

    uint32_t last_adc = 0;
    uint32_t last_tc = 0;
    uint32_t last_flush = 0;
    bool finalized = false;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        g_sys.hb++;

        ms5611_tick(now);

        if (now - last_adc >= 100) {
            last_adc = now;
            g_sys.vbat_mv = read_vbat();
        }

        // Bay thermocouple: the MAX31855 converts at ~5 Hz internally,
        // polling at 10 Hz just picks up fresh frames.
        if (now - last_tc >= 100) {
            last_tc = now;
            float tc_c;
            g_sys.tc_c10 = tc_read(&tc_c) ? (int16_t)(tc_c * 10.0f)
                                          : (int16_t)INT16_MIN;
        }

        bool wrote = false;
        log_record_t rec;
        while (logq_pop(&rec)) {
            if (!flashlog_append(&rec)) g_sys.faults_core1 |= FF_FLASH_ERR;
            wrote = true;
        }
        if (wrote && now - last_flush >= 100) {
            last_flush = now;
            flashlog_flush();
        }
        log_meta_t meta_req;
        if (take_log_meta(&meta_req))
            flashlog_set_meta(&meta_req);
        if (g_sys.log_hold && !finalized) {
            avionics_finalize_log();
            finalized = true;
        }

        usb_cli_poll(now);
        led_update(now);
    }
}

// Phase-aware log divider: every control cycle (1 kHz) through flight and
// the first 30 s of descent (separation + chute events), 100 Hz for the rest
// of descent, 50 Hz armed on the pad, 10 Hz idle. Full mission ~3.5 MB.
static uint32_t log_divider(flight_state_t st, uint32_t now) {
    switch (st) {
    case ST_PADIDLE: return CTRL_HZ / LOG_HZ_PAD;
    case ST_ARMED:   return CTRL_HZ / LOG_HZ_ARMED;
    case ST_BOOST:
    case ST_COAST:
    case ST_APOGEE:  return 1;
    default:         // DESCENT, LANDED
        return (now - descent_entry_ms < LOG_DESCENT_HIRATE_MS)
             ? 1 : CTRL_HZ / LOG_HZ_DESCENT;
    }
}

// ---------------------------------------------------------------------------
// Core 0 control tick at 1 kHz. Reads the IMU, runs the state machine and
// the roll controller, drives the servos, queues log records at the
// phase-aware rate. This ISR is the only place the watchdog is fed.
// ---------------------------------------------------------------------------
static bool ctrl_tick(repeating_timer_t *rt) {
    (void)rt;
    watchdog_update();

    uint32_t now = to_ms_since_boot(get_absolute_time());

    imu_sample_t s;
    imu_read(&s);
    if (!s.valid) imu_err_streak++;
    else          imu_err_streak = 0;

    flight_state_t st = state_step(&s, now);
    int16_t delta_cdeg = ctrl_step(st, &s, now);

    if (st != log_last_st) {
        if (st == ST_DESCENT) descent_entry_ms = now;
        if (st == ST_ARMED)   avionics_log_meta_update();
        log_last_st = st;
        log_div_cnt = 0;
    }

    uint16_t flags = flags_core0 | g_sys.faults_core1;
    if (logq_dropped())                  flags |= FF_LOGQ_DROP;
    if (imu_err_streak > 200)            flags |= FF_IMU_ERR;
    if (state_baro_stale())              flags |= FF_BARO_STALE;
    if (st >= ST_ARMED)                  flags |= FF_ARMED;
    if (ctrl_bias_valid())               flags |= FF_CAL;

    if (!g_sys.log_hold) {
        if (++log_div_cnt >= log_divider(st, now)) {
            log_div_cnt = 0;
            baro_shared_t b;
            read_baro(&b);
            log_record_t rec;
            memset(&rec, 0, sizeof rec);
            rec.t_ms = now;
            rec.ax_mg = (int16_t)(s.ax * 1000.0f);
            rec.ay_mg = (int16_t)(s.ay * 1000.0f);
            rec.az_mg = (int16_t)(s.az * 1000.0f);
            rec.gx_dps10 = (int16_t)(s.gx * 10.0f);
            rec.gy_dps10 = (int16_t)(s.gy * 10.0f);
            rec.gz_dps10 = (int16_t)(s.gz * 10.0f);
            rec.v_cms = (int32_t)(ctrl_v_hat() * 100.0f);
            rec.d1 = b.d1;
            rec.d2 = b.d2;
            rec.delta_cdeg = delta_cdeg;
            rec.batt_mv = g_sys.vbat_mv;
            rec.state = (uint8_t)st;
            rec.flags = flags;
            rec.tc_c10 = g_sys.tc_c10;
            logq_push(&rec);
        }
    }

    publish_status((uint8_t)st, (int16_t)(s.gz * 10.0f), ctrl_v_hat(),
                   delta_cdeg, g_sys.vbat_mv, flags);
    return true;
}

int main(void) {
    // CS pins high before bus init so nothing glitches into a command.
    const uint cs_pins[4] = { PIN_CS_IMU, PIN_CS_BARO, PIN_CS_FLASH, PIN_CS_TC };
    for (int i = 0; i < 4; i++) {
        gpio_init(cs_pins[i]);
        gpio_set_dir(cs_pins[i], GPIO_OUT);
        gpio_put(cs_pins[i], 1);
    }

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    spi_init(spi0, 10 * 1000 * 1000);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI0_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI0_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI0_SCK,  GPIO_FUNC_SPI);

    spi_init(spi1, 10 * 1000 * 1000);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI1_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI1_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI1_SCK,  GPIO_FUNC_SPI);

    stdio_init_all();

    shared_init();
    logq_init();
    state_init();
    ctrl_reset();
    servos_init();

    adc_init();
    adc_gpio_init(PIN_VBAT);

    gpio_init(PIN_ARM);
    gpio_set_dir(PIN_ARM, GPIO_IN);
    gpio_pull_up(PIN_ARM);

    g_sys.imu_ok = imu_init();
    if (!g_sys.imu_ok) flags_core0 |= FF_IMU_ERR;
    if (watchdog_caused_reboot()) {
        g_sys.wdt_reboot = true;
        flags_core0 |= FF_WDT_RESET;
    }

    multicore_launch_core1(core1_main);

    // The control ISR is the sole watchdog feeder: a hung control loop means
    // a full reset, not a silently dead rocket.
    watchdog_enable(1500, true);
    add_repeating_timer_us(-(1000000 / CTRL_HZ), ctrl_tick, NULL, &ctrl_timer);

    uint32_t last_hb = 0;
    uint32_t last_hb_ms = to_ms_since_boot(get_absolute_time());
    while (true) {
        tight_loop_contents();
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (g_sys.hb != last_hb) {
            last_hb = g_sys.hb;
            last_hb_ms = now;
            flags_core0 &= (uint16_t)~FF_CORE1_HB;
        } else if (now - last_hb_ms > 2000) {
            flags_core0 |= FF_CORE1_HB;
        }
    }
}
