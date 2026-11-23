#pragma once

// ---------------------------------------------------------------------------
// Pin map. Also the authoritative wiring table (mirrored in README.md).
// SPI0 belongs to core 0 (control ISR, IMU only).
// SPI1 belongs to core 1 (MS5611 + W25Q128 + MAX31855).
// ---------------------------------------------------------------------------
#define PIN_SPI0_SCK    2
#define PIN_SPI0_MOSI   3
#define PIN_SPI0_MISO   4
#define PIN_CS_IMU      5

#define PIN_SPI1_SCK    6
#define PIN_SPI1_MOSI   7
#define PIN_SPI1_MISO   8
#define PIN_CS_BARO     9
#define PIN_CS_FLASH    10
#define PIN_CS_TC       17   // MAX31855, K-probe on the nosecone skin

// Servo outputs, 50 Hz hardware PWM. Fin order around the can: 0,1,2,3.
#define PIN_SERVO0      11
#define PIN_SERVO1      12
#define PIN_SERVO2      14
#define PIN_SERVO3      15

// Arm interlock: RBF jack and master switch each short this pin to GND when
// SAFE, wired in PARALLEL. Internal pull-up: reads high (armed) only when
// the switch is on AND the RBF pin is pulled.
#define PIN_ARM         16

// Battery sense: 10k/2k divider from 2S LiPo to GP26 (ADC0).
#define PIN_VBAT        26
#define VBAT_DIV_R_TOP  10000.0f
#define VBAT_DIV_R_BOT  2000.0f
#define ADC_REF_MV      3300.0f

#define PIN_LED         25

// IMU breakout orientation: which CHIP axis points up the rocket (toward the
// nose). +/-1 = X, +/-2 = Y, +/-3 = Z. A GY breakout lying flat on the blade
// has its Z axis radial, so the nose-ward axis is X or Y (read the silkscreen
// arrows). Bench check: nose up on the table, CLI 'i' must show az = +1.0 g.
#define IMU_UP_AXIS     1

// ---------------------------------------------------------------------------
// v5: Pi link + separation charge (the Pico now fires the explosive bolts;
// the commercial altimeter stays in the loop as backup on a later setting).
// UART0 to the Pi Zero 2 W at 115200: telemetry out, GPS lines in.
// ---------------------------------------------------------------------------
#define PIN_LINK_TX     0            // UART0 TX -> Pi RX
#define PIN_LINK_RX     1            // UART0 RX <- Pi TX
#define LINK_BAUD       115200

// Pyro channel: low-side MOSFET on the e-match/bolt charge, plus a
// high-impedance continuity divider (0.8 mA sense current, far below any
// no-fire rating) into ADC1.
#define PIN_PYRO_FET    18
#define PIN_PYRO_CONT   27           // ADC1
#define PYRO_PULSE_MS   1000         // fire duration at apogee
#define PYRO_CONT_ADC_MIN  0.25f     // fraction of full scale = match present
// Fire inhibit. The charge may only fire if the flight looks like a flight:
// long enough since liftoff AND (real altitude gained OR real speed reached).
// Without this a 3 g handling bump on an armed rocket walks the state
// machine straight to "apogee" and fires the bolts in someone's hands.
#define PYRO_LOCKOUT_MS    12000     // min time liftoff -> fire (7 s burn, sim apogee 20-26 s)
#define PYRO_MIN_ALT_M     150.0f    // baro gain above the pad, or...
#define PYRO_MIN_VPEAK     100.0f    // ...peak v_hat, m/s

// ---------------------------------------------------------------------------
// Rates. Control ISR at 1 kHz (IMU ODR 1 kHz, no averaging aliasing games).
// Log rate is phase aware: every control cycle during flight and for 30 s
// into descent (separation + chute events), then 100 Hz, low rates on pad.
// Full flight budget: ~3.5 MB of the 16 MB chip.
// ---------------------------------------------------------------------------
#define CTRL_HZ         1000
#define CTRL_DT         (1.0f / CTRL_HZ)
#define CTRL_PERIOD_MS  1
#define LOG_HZ_PAD      10
#define LOG_HZ_ARMED    50
#define LOG_HZ_DESCENT  100
#define LOG_DESCENT_HIRATE_MS 30000

// ---------------------------------------------------------------------------
// Flight state thresholds. Counters are derived from the millisecond values.
// ---------------------------------------------------------------------------
#define ARM_STILL_DPS       20.0f    // every axis must stay below this to arm
#define ARM_STILL_MS        3000
#define ARM_MAX_BIAS_DPS    50.0f    // refuse to fly with a crazy bias
#define ARM_MIN_VBAT_MV     6800
#define PAD_TIMEOUT_MS      300000
// Liftoff: the real motor (openMotor on Motors.ric: K179, 7.2 s burn)
// only pushes 3.7 g at ignition, so the old 3 g threshold sat right on the
// thrust level. 2 g held for 100 ms; false triggers from handling are made
// harmless by the pyro lockout below, not by this threshold.
#define LIFTOFF_G           2.0f
#define LIFTOFF_MS          100
#define BURNOUT_G           0.5f
#define BURNOUT_MS          100
#define APOGEE_V_MPS        5.0f
#define APOGEE_BARO_DROP_M  2.0f
#define APOGEE_BARO_SAMPLES 5        // consecutive fresh baro samples below the drop line
#define APOGEE_BARO_VMAX    40.0f    // baro apogee only believed below this v_hat
#define APOGEE_NOCLIMB_MS   1500     // v_hat apogee only if baro made no new max this long
#define COAST_TIMEOUT_MS    60000    // failsafe: no apogee seen by now -> call it
#define BARO_STALE_MS       500
#define LANDED_BAND_M       0.3f
#define LANDED_MS           5000
#define POST_LANDED_LOG_MS  10000
// Authority window. Real motor in OpenRocket: ~13 m/s off the rail,
// ~180 m/s / M0.53 at burnout (up to ~250 m/s with a tighter throat).
// AUTH_V_MIN is below rail-exit speed on purpose: control comes alive as
// soon as there is any airspeed worth using.
#define AUTH_V_MIN          10.0f    // canards driven only inside this window
#define AUTH_V_MAX          275.0f
#define G_MPS2              9.81f
#define V_HAT_MAX           320.0f

// ---------------------------------------------------------------------------
// Roll control. Starting gains come from sim/roll_sim.py; retune after M3.
// ROLL_DIR flips the fin response sign: set it during the ground spin test.
// ---------------------------------------------------------------------------
// 2026-11-22: retuned in sim/roll_sim.py against booster-fin cant (0.2 deg
// on all four) with aerodynamic roll damping. The old 0.15 / 0.02 left
// ~220 deg/s of spin: the integrator was 50x too slow to trim the cant out.
// 0.3 / 1.0 holds < 5 deg/s and stays well damped with the canards 2x or
// 0.5x as effective as modeled, Ixx off by 35 %, and a slow servo.
#define ROLL_KP             0.30f    // deg deflection per deg/s of rate error
#define ROLL_KI             1.00f    // deg per (deg/s * s)
#define ROLL_SCHED_VREF     25.0f    // full gain at/below, scaled (vref/v)^2 above
#define ROLL_MAX_DEG        15.0f
#define ROLL_MAX_RATE_DEG_S 600.0f   // commanded deflection slew limit
#define ROLL_DIR            1.0f

// Transonic deflection taper: full ROLL_MAX_DEG at/below DEFLECT_FULL_V,
// shrinking linearly to DEFLECT_MIN_DEG at/above DEFLECT_MIN_V. Keeps hinge
// loads and flutter margin sane through M0.78; the Q-scheduled gains shrink
// commands further on their own.
#define DEFLECT_FULL_V      100.0f
#define DEFLECT_MIN_V       250.0f
#define DEFLECT_MIN_DEG     3.0f

// ---------------------------------------------------------------------------
// Servos
// ---------------------------------------------------------------------------
#define SERVO_US_MIN        1000
#define SERVO_US_MAX        2000
#define SERVO_US_CENTER     1500
#define SERVO_US_PER_DEG    11.111f  // 1000 us across 90 deg

// ---------------------------------------------------------------------------
// Flash log
// ---------------------------------------------------------------------------
#define LOG_MAGIC           0x314E5641u  // "AVN1" little endian
#define LOG_VERSION         4u           // v4: events byte (pyro + gps) added
#define FLASH_LOG_BASE      0x00001000u // sector 0 is reserved for the header
#define FLASH_SIZE_BYTES    (16u * 1024u * 1024u)
