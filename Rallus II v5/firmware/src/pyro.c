#include "pyro.h"
#include "config.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"

// Fire path: 2S -> e-match -> N-FET (GP18) -> GND. Sense path: 2S -> match
// -> 10k -> GND with a 47k/10k tap to ADC1. Sense current ~0.8 mA, far
// below any no-fire rating; match present puts ~1.5 V on the tap, an open
// circuit puts ~0 V.

// Fire pulse deadline. Set by core 0 (pyro_fire, at apogee), cleared by
// core 1 (pyro_tick) after the FET turns off. A stale nonzero deadline
// only ever turns the FET off, never on, so the sharing is safe.
static volatile uint32_t fire_deadline_ms;

void pyro_init(void) {
    gpio_init(PIN_PYRO_FET);
    gpio_set_dir(PIN_PYRO_FET, GPIO_OUT);
    gpio_put(PIN_PYRO_FET, false);
    adc_gpio_init(PIN_PYRO_CONT);
}

bool pyro_cont_ok(void) {
    adc_select_input(1);   // ADC1 = GP27
    uint16_t raw = adc_read();
    return raw > (uint16_t)(PYRO_CONT_ADC_MIN * 4095.0f);
}

void pyro_fire(uint32_t now_ms) {
    gpio_put(PIN_PYRO_FET, true);
    fire_deadline_ms = now_ms + PYRO_PULSE_MS;
}

void pyro_tick(uint32_t now_ms) {
    uint32_t d = fire_deadline_ms;
    if (d != 0 && (int32_t)(now_ms - d) >= 0) {
        gpio_put(PIN_PYRO_FET, false);
        fire_deadline_ms = 0;
    }
}
