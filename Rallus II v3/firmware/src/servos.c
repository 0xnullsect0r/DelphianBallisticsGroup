#include "servos.h"
#include "config.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

// Fin order around the can. Roll pairs: 0,2 same sign, 1,3 opposite.
static const uint8_t  servo_pin[4]  = { PIN_SERVO0, PIN_SERVO1, PIN_SERVO2, PIN_SERVO3 };
static const float    fin_sign[4]   = { +1.0f, -1.0f, +1.0f, -1.0f };
// Bench-calibrated center trim, us per servo. Adjust after linkage assembly.
static const uint16_t center_trim[4] = { 0, 0, 0, 0 };

// 50 Hz from a 1 MHz timebase: sys clock 150 MHz / clkdiv 150 = 1 MHz,
// wrap 20000 - 1 = 20 ms period, 1 us per count.
void servos_init(void) {
    for (int i = 0; i < 4; i++) {
        gpio_set_function(servo_pin[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(servo_pin[i]);
        uint chan  = pwm_gpio_to_channel(servo_pin[i]);
        pwm_set_clkdiv_int_frac(slice, 150, 0);
        pwm_set_wrap(slice, 20000 - 1);
        pwm_set_chan_level(slice, chan, SERVO_US_CENTER + center_trim[i]);
        pwm_set_enabled(slice, true);
    }
}

void servos_write_us(uint8_t i, uint16_t us) {
    if (i > 3) return;
    if (us < SERVO_US_MIN) us = SERVO_US_MIN;
    if (us > SERVO_US_MAX) us = SERVO_US_MAX;
    pwm_set_chan_level(pwm_gpio_to_slice_num(servo_pin[i]),
                       pwm_gpio_to_channel(servo_pin[i]), us);
}

void servos_center(void) {
    for (int i = 0; i < 4; i++)
        servos_write_us(i, SERVO_US_CENTER + center_trim[i]);
}

void servos_write_roll(float delta_deg) {
    if (delta_deg > ROLL_MAX_DEG)  delta_deg = ROLL_MAX_DEG;
    if (delta_deg < -ROLL_MAX_DEG) delta_deg = -ROLL_MAX_DEG;
    for (int i = 0; i < 4; i++) {
        float us = (float)(SERVO_US_CENTER + center_trim[i])
                 + fin_sign[i] * delta_deg * SERVO_US_PER_DEG;
        servos_write_us(i, (uint16_t)(us + 0.5f));
    }
}
