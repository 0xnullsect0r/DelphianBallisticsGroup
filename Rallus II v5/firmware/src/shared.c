#include "shared.h"
#include "imu.h"
#include <string.h>

shared_sys_t g_sys;

void shared_init(void) {
    memset((void *)&g_sys, 0, sizeof g_sys);
    critical_section_init(&g_sys.baro_cs);
    critical_section_init(&g_sys.status_cs);
    critical_section_init(&g_sys.meta_cs);
}

void request_log_meta(const log_meta_t *m) {
    critical_section_enter_blocking(&g_sys.meta_cs);
    g_sys.meta_req = *m;
    g_sys.meta_seq++;
    critical_section_exit(&g_sys.meta_cs);
}

bool take_log_meta(log_meta_t *out) {
    static uint32_t seen;
    bool fresh = false;
    critical_section_enter_blocking(&g_sys.meta_cs);
    if (g_sys.meta_seq != seen) {
        seen = g_sys.meta_seq;
        *out = g_sys.meta_req;
        fresh = true;
    }
    critical_section_exit(&g_sys.meta_cs);
    return fresh;
}

void publish_baro(uint32_t d1, uint32_t d2, float press_pa, float alt_m, uint32_t t_ms) {
    critical_section_enter_blocking(&g_sys.baro_cs);
    g_sys.baro.d1 = d1;
    g_sys.baro.d2 = d2;
    g_sys.baro.press_pa = press_pa;
    g_sys.baro.alt_m = alt_m;
    g_sys.baro.t_ms = t_ms;
    g_sys.baro.seq++;
    critical_section_exit(&g_sys.baro_cs);
}

void read_baro(baro_shared_t *out) {
    critical_section_enter_blocking(&g_sys.baro_cs);
    *out = g_sys.baro;
    critical_section_exit(&g_sys.baro_cs);
}

void publish_status_full(uint8_t state, const imu_sample_t *raw,
                         float v_mps, int16_t delta_cdeg,
                         uint16_t batt_mv, uint16_t flags, uint32_t t_ms) {
    critical_section_enter_blocking(&g_sys.status_cs);
    g_sys.status.state = state;
    g_sys.status.flags = flags;
    g_sys.status.t_ms = t_ms;
    g_sys.status.ax_mg = (int16_t)(raw->ax * 1000.0f);
    g_sys.status.ay_mg = (int16_t)(raw->ay * 1000.0f);
    g_sys.status.az_mg = (int16_t)(raw->az * 1000.0f);
    g_sys.status.gx_dps10 = (int16_t)(raw->gx * 10.0f);
    g_sys.status.gy_dps10 = (int16_t)(raw->gy * 10.0f);
    g_sys.status.gz_dps10 = (int16_t)(raw->gz * 10.0f);
    g_sys.status.v_mps = v_mps;
    g_sys.status.delta_cdeg = delta_cdeg;
    g_sys.status.batt_mv = batt_mv;
    critical_section_exit(&g_sys.status_cs);
}

void read_status(status_shared_t *out) {
    critical_section_enter_blocking(&g_sys.status_cs);
    *out = g_sys.status;
    critical_section_exit(&g_sys.status_cs);
}
