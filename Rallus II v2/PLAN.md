# Rallus II v2 — Canard Roll Control Avionics

Active canard control for Rallus II, 2 in airframe, CTI J-class solid motor
(fixed impulse, no throttle). Phase 1: roll control (anti-spin). Phase 2:
boost-phase attitude stabilization (pitch/yaw). Hardware is 4-canard capable
from the start so phase 2 is a firmware update, not an airframe rebuild.

v1 (`../Rallus II v1/`) is the frozen archive: original OpenRocket file and
the aft-fin-control revision of this plan. Everything active lives here.

## Vehicle (from Rallus Mk.II V1.ork)

Two-stage model of the mid-break airframe. Separation at apogee — in the
real rocket the halves are held by explosive bolts and split at the break;
each half recovers on its own 24 in ripstop chute.

| | |
|---|---|
| Total length | 1.17 m (nose cone 282 + upper tube 305 + coupler 25 + booster 559 mm) |
| Diameter | 2.0 in (50.8 mm OD) |
| Upper section | PLA conical nose (362 g) + aluminum tube, 2 mm wall, 305 mm |
| Booster | steel tube, 3 mm wall, 559 mm + 4 steel fins (141 mm chord, 50 mm span) |
| Motor | Custom propellant (EX), modeled in ORK as the closest commercial equivalent: CTI 1115J530-15A, 38 mm, ~2.1 s burn. EX at organized launches = TRA EX rules, L2-equivalent standing |
| Liftoff mass | ~4.25 kg (est. from ORK components incl. 0.2 kg avionics bay) |
| Rail | 1010 buttons, 3 m rail, 33.8 m/s off the rail |

OpenRocket sim (V1 file): apogee **2485 m**, max velocity **263 m/s (M0.78,
transonic)**, max accel **19.3 g**, time to apogee 21.6 s.

**Control section = upper half only.** The break at the coupler means the
canards, servos, avionics stack, and battery all live in the aluminum upper
tube, forward of the split, and fly with the nose on their own chute. The
booster is passive.

**Room**: the ORK reserves a 0.27 m long, ~49 mm diameter payload volume in
the upper tube. That is the avionics bay — genuinely a lot of room for a
Pico 2 stack, 4 micro servos, and a 2S pack.

## Decisions

1. **Scope:** Roll control first on 4-servo / 4-canard hardware. Attitude
   control later.
2. **Recovery + separation:** A commercial altimeter fires the charges and
   the explosive-bolt system. This board is control + logging only and rides
   the upper section. It never touches pyro unless that decision changes.
3. **Motor:** Custom propellant, our own load. The ORK and the sim carry a
   CTI 1115J530 thrust curve as the closest commercial equivalent — when the
   real static-test data exists, replace the curve and re-run everything.
   Fixed impulse either way; control authority is purely aerodynamic.
4. **Telemetry:** Onboard logging only. No radio in phase 1.
5. **Flight computer:** Raspberry Pi Pico 2 (RP2350, dual Cortex-M33 @
   150 MHz, FPU, 520 KB SRAM) for ALL phases. Native 3.3 V (no level
   shifters). Pico 2 W is a +$2 option covering future WiFi telemetry.
6. **Toolchain:** Pico SDK, C/C++, CMake, UF2 over USB. No Arduino framework.
7. **IMU:** ICM-42688-P (gyro +/-2000 dps, accel +/-32 g, SPI). The 19.3 g
   boost is why: a 16 g part would clip. Baro: MS5611 (SPI).

## Physics constraints

- Solid motor = fixed burn. The only control input is canard deflection.
- Canard force scales with Q = 0.5*rho*v^2. Below ~15 m/s: no authority.
  At 263 m/s: Q ~ 42 kPa, hinge loads get real, flutter margin shrinks.
- **Transonic (M0.78 max):** canard effectiveness wanders near Mach 1 and
  roll reversal is possible at large deflections. Handled two ways: Q-squared
  gain scheduling (deflections shrink ~100x at burnout vs 25 m/s) and a hard
  deflection taper in firmware — full 15 deg below 100 m/s, linearly down to
  3 deg by 250 m/s (`DEFLECT_*` in config.h).
- Forward canards shift CP forward and eat static margin. V2.ork models the
  4 canards; **run its stability sim before cutting metal** — static margin
  must stay >= 1.5 cal with canards at neutral. The booster's big steel
  fins dominate CP, and the V1 layout already had 2 fins up there, so the
  margin should hold, but verify.
- Gyro range +/-2000 dps = 5.5 rev/s. Control arms at liftoff detection,
  not burnout.
- Nothing fin-based holds the rocket up at apogee. Attitude control (phase
  2) is boost/coast weathercock reduction only.

## System architecture

```
                 2S LiPo
                    |
              5V/3A BEC ----+---- servo rail (4x MG90S)  (+1000uF cap)
                            |
                          VSYS (Pico 2 onboard regulator makes 3.3V)

   Pico 2 (RP2350) — mounted in the UPPER tube, USB axis along the body
     core 0 @ 1 kHz: ICM-42688-P sample -> state machine -> roll PI ->
                      deflection taper -> servo PWM
     core 1: MS5611 poll ~25 Hz, flash log writer, USB CDC CLI, LED, ADC

   SPI0 (IMU only, core 0):      SPI1 (baro + flash, core 1):
     GP2  SCK                      GP6  SCK
     GP3  TX/MOSI                  GP7  TX/MOSI
     GP4  RX/MISO                  GP8  RX/MISO
     GP5  CS_IMU                   GP9  CS_BARO
                                   GP10 CS_FLASH
   Servos (PWM 50 Hz): GP11, GP12, GP14, GP15
   Arm input (RBF pin, pull-up, to GND when inserted): GP16
   Battery sense (10k/2k divider off 2S): GP26/ADC0
   Status LED: GP25 (onboard)
```

Roll mixing (phase 1): canards 0,2 deflect +delta, 1,3 deflect -delta.
Pitch/yaw mixing (phase 2): collective pairs added on top of the roll term.

## Hardware

### Flight computer
Pico 2. Core 0 runs the 1 kHz control ISR: IMU read, state machine, PI,
taper, servo write, watchdog feed. Core 1: baro, flash, USB, LED. Hardware
watchdog fed only by the control ISR. Reset reason logged at boot.

### IMU
ICM-42688-P: +/-32 g (19.3 g peak + vibration headroom), +/-2000 dps gyro,
~$9 GY breakout at 3.3 V. 1 kHz ODR. Hard-mounted, sensor-side filtering.

### Baro
MS5611-01BA on SPI1, full datasheet compensation. The bay vents through a
1/8 in hole drilled in the aluminum upper tube (metal tube = no leakage
through the wall, the vent is mandatory).

### Logging
W25Q128 SPI NOR flash, 16 MB, on SPI1. 250 Hz x 36 B records. Flight log
never touches the Pico's boot flash.

### Servos + canards
4x MG90S-class digital metal-gear (~2.2 kg-cm). Canards: 20 mm chord,
10 mm span freeform (the ORK V2 fin set — brass, 1 mm thick, same shape as
the V1 nose fins). At max Q a 3 deg deflection loads a canard hinge around
5 mN·m — orders of magnitude inside MG90S margin. Hinge moment check at
15 deg / 100 m/s still passes; recheck if canard geometry changes.

### Power
2S 450 mAh LiPo. 5V/3A switching BEC -> servo rail + Pico VSYS. 1000 uF
across the servo rail. Bench test: all four servos stalled while logging.

## Mechanical

- Canard fin can at the TOP of the upper tube, just below the nose cone
  shoulder: 4 canards on 3 mm shafts through the aluminum wall, each driven
  directly by its servo. OpenSCAD in `hardware/` once fin can design starts.
- Avionics sled directly behind the canard bay in the 0.27 m payload volume:
  4 servos forward around the perimeter, Pico 2 + MS5611 + flash stacked
  center, battery aft of the electronics (mass forward helps phase 2
  anyway).
- Servos mounted so launch load is perpendicular to the output axis;
  threadlock the horns; tight shaft fits (wobble = flutter).
- Everything forward of the mid-break coupler. The altimeter and its battery
  ride wherever the recovery layout wants them (also upper section).

## Software

Firmware in `firmware/`, Pico SDK C. File map and wiring in
`firmware/README.md`.

### State machine
```
PADIDLE -> ARMED -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED
```
- ARMED: RBF pulled + 3 s stillness (gyro bias cal) + battery >= 6.8 V.
  5-min pad timeout.
- LIFTOFF: >3 g axial sustained 40 ms. BOOST->COAST: <0.5 g sustained
  100 ms. APOGEE: v_hat < 5 m/s or baro 2 m below max — this roughly
  coincides with the explosive-bolt separation event; the accel spike it
  produces lands after control has already been disabled.
- Canards driven ONLY in BOOST/COAST with v_hat in [15, 275] m/s. Every
  other state: exact center.
- Watchdog fed only in the control ISR. IMU failure >200 ms: neutral +
  fault logged.

### Roll control law (phase 1)
PI on roll-rate error, gains scheduled (v_ref/v)^2 above 25 m/s, deflection
tapered 15 -> 3 deg across 100-250 m/s, command slew limited, anti-windup.
Starting gains from `sim/roll_sim.py` (now carrying real Rallus II numbers).

### Phase 2 (attitude)
Gyro integration + accel gravity correction (gated off during boost),
pitch/yaw PID -> collective canard commands. Canards up front is the
classic layout for this. Same Pico 2.

### Log record (36 B packed)
t, ax/ay/az mg, gx/gy/gz dps*10, v_hat cm/s, MS5611 D1, D2, delta deg*100,
battery mV, state, fault flags.

## Simulation

`sim/roll_sim.py`: 1-DOF roll dynamics with Rallus II numbers (4.1 kg,
J530 thrust, CD 0.5, Ixx ~0.0015 kg·m², 4 canards at r=30 mm). Same
discrete PI + taper + servo lag as firmware. Post-flight: replay logged
v(t), compare commanded vs actual roll rate.

## Test campaign (do not skip steps)

1. **Bench:** banner IDs, baro vs local weather, servo sweep + trim, 4-servo
   stall brownout test.
2. **Sim:** gains from `roll_sim.py`.
3. **Spin test:** hand-spin the upper section, canards counter-rotate the
   right way; fix `ROLL_DIR` if not.
4. **Shaker/handling:** RBF logic, arm timeouts, log capture.
5. **Locked-canard flight:** full stack flies passive, log validates
   v_hat vs ORK sim (263 m/s, 19.3 g, 21.6 s apogee).
6. **Roll-active flight.** Compare, retune.
7. **Phase 2 attitude flights** after two clean roll flights.

## OpenRocket TODO

- `openrocket/Rallus Mk.II V2.ork` — V1 revised: 2 nose fins replaced by 4
  canards (same geometry, 90 deg spacing), avionics bay modeled as a real
  0.2 kg mass at the top of the upper tube, stale V1 flight data stripped,
  sim timestep tightened to 0.01 s for transonic runs. **Open it, run the
  stability + flight sims, record**: static margin with canards neutral,
  CG/CP, Q(t), hinge moments at 15 deg / 100 m/s and 3 deg / 263 m/s.
- Confirm margin >= 1.5 cal. If canards push CP too far forward, shrink
  span or grow the booster fins.
- Feed the sim results back into `sim/roll_sim.py` (IXX especially).

## Risks

| Risk | Mitigation |
|---|---|
| Canards shift CP forward, destabilize | V2.ork stability sim before cutting; margin >= 1.5 cal |
| Transonic flutter / roll reversal | Deflection taper to 3 deg, thick low-AR brass canards, gains Q-scheduled |
| Servo rail brownout during boost | Dedicated BEC, 1000 uF, 4-servo stall bench test |
| Separation shock at apogee | Control already disabled by APOGEE; accel spike is post-authority |
| Gyro saturation before control engages | Arm at liftoff, not burnout |
| Vibration aliasing IMU | Hard mount, 1 kHz ODR, sensor-side filtering |
| Hung control loop | Watchdog fed only by control ISR |
| Metal tube blocks baro | 1/8 in vent hole, mandatory |
| Wrong canard sign convention | Ground spin test |
| Custom motor curve differs from CTI stand-in | Static test the real grain, swap the thrust curve, re-run ORK + sim before flight |
| EX motor at launch | TRA EX rules apply — fly on the EX range, L2-equivalent standing |

## Milestones

- M0: plan + BOM done, parts ordered
- M1: firmware builds (done), bench suite green
- M2: V2.ork sims validated, canard fin can + sled built, spin test passed
- M3: locked-canard flight, full log recovered
- M4: first roll-active flight
- M5: phase 2 attitude control
