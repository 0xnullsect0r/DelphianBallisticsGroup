# Active Fin Control Avionics (working name)

Model rocket, 2 in airframe, solid motor (fixed impulse, no throttle), fin-based active control.
Phase 1: roll control (anti-spin). Phase 2: boost-phase attitude stabilization (pitch/yaw).
Hardware is built 4-fin capable from the start so phase 2 is a firmware update, not an
airframe rebuild.

## Decisions

1. **Scope:** Roll control first on 4-servo / 4-fin hardware. Attitude control later.
2. **Recovery:** A commercial altimeter fires the charges. This board is control + logging only.
   Pyro channels can be added in a later board rev if wanted.
3. **Motor:** Unknown until the OpenRocket file lands. Hardware sized for composite F/G class.
   Solid propellant: thrust curve is fixed, control authority is purely aerodynamic.
4. **Telemetry:** Onboard logging only. No radio in phase 1.
5. **Flight computer:** Raspberry Pi Pico 2 (RP2350, dual Cortex-M33 @ 150MHz, FPU, 520KB SRAM)
   for ALL phases. Native 3.3V (no level shifters), dual core, more than enough headroom for
   phase 2 attitude estimation. Pico 2 W is a +$2 option that would cover future WiFi telemetry.
6. **Toolchain:** Pico SDK, C/C++, CMake, UF2 over USB. No Arduino framework.
7. **IMU:** ICM-42688-P (gyro +/-2000 dps, accel +/-32g, SPI). Baro: MS5611 (SPI, exact
   documented compensation, standard in amateur rocketry).

## Physics constraints (why the plan looks like this)

- Solid motor = fixed burn. No throttle, no abort. The only control input is fin deflection.
- Fins make control force proportional to dynamic pressure Q = 0.5 * rho * v^2. Below roughly
  15 m/s there is no meaningful authority. Nothing fin-based can hold the rocket up at apogee.
- Active attitude control therefore only works rail-clear to late coast, and only if velocity
  off the rail is healthy. Target v_off_rail >= 20 m/s (check in OpenRocket, drives motor pick).
- Gyro range +/-2000 deg/s = 5.5 rev/s. If the rocket rolls faster than that before control
  engages, the gyro saturates and the controller is blind. Control arms at liftoff detection,
  not burnout.
- The rocket must be statically stable with fins at neutral. Control fins are sized as fins
  first, control surfaces second.

## System architecture

```
                 2S LiPo
                    |
              5V/3A BEC ----+---- servo rail (4x MG90S)  (+1000uF cap)
                            |
                          VSYS (Pico 2 onboard regulator makes 3.3V)

   Pico 2 (RP2350)
     core 0 @ 250Hz: ICM-42688-P sample -> state machine -> roll PI -> servo PWM
     core 1: MS5611 poll ~25Hz, flash log writer, USB CDC command line, LED, battery ADC

   SPI0 (IMU only, core 0):      SPI1 (baro + flash, core 1):
     GP2  SCK                      GP6  SCK
     GP3  TX/MOSI                  GP7  TX/MOSI
     GP4  RX/MISO                  GP8  RX/MISO
     GP5  CS_IMU                   GP9  CS_BARO
                                   GP10 CS_FLASH
   Servos (PWM 50Hz): GP11, GP12, GP14, GP15
   Arm input (RBF pin, pull-up, to GND when inserted): GP16
   Battery sense (divider off 2S): GP26/ADC0
   Status LED: GP25 (onboard)
```

Separate SPI buses per core so the control ISR never contends with logging. Core-to-core
traffic is single-producer/single-consumer queues and small critical sections.

Roll mixing (phase 1): fins 0,2 deflect +delta, fins 1,3 deflect -delta.
Pitch/yaw mixing (phase 2): collective pairs added on top of the roll term.

## Hardware

### Flight computer
Pico 2. Core 0 runs the 250Hz control ISR: IMU read, state machine, PI, servo write, watchdog
feed. Core 1 runs everything else: baro, flash, USB, LED. Hardware watchdog fed only by the
control ISR, so a hung control loop resets the board instead of flying blind. Reset reason is
checked and logged at boot (watchdog / power-on).

### IMU
ICM-42688-P: +/-32g accel (won't clip on a 10-15g boost; +/-16g parts like the MPU-6050 will,
and MPU-6050 clones are garbage), +/-2000dps gyro, low noise, ~$9 GY breakout running direct
at 3.3V. Sampled at 1kHz ODR, averaged 4:1 down to 250Hz (cheap anti-aliasing). Hard-mounted,
using the sensor-side config plus the 4:1 average, no foam.

### Baro
MS5611-01BA on SPI1. Full first- and second-order datasheet compensation in firmware.
D1/D2 raw values also go in the log so pressure can be re-derived on the ground.
Vent hole in the avionics bay (1/8 in).

### Logging
W25Q128 SPI NOR flash, 16MB, on SPI1. Flight log never touches the Pico's boot flash.
Full-rate logging: every control cycle (250Hz, 36B/record = 9KB/s, ~31 min capacity).
Records flow core 0 -> core 1 through a 2048-entry SPSC ring (8s of buffering), core 1 packs
pages, erases sectors ahead of the write pointer, and programs 256B pages. Header sector keeps
the record count, updated at sector boundaries and at flight finalize.

### Servos
4x MG90S-class digital, metal gear, ~2.2 kg-cm. Driven by hardware PWM at 50Hz, 1us
resolution, 3.3V pulse into a 5V servo (fine). Digital servos can go 100-200Hz later by
changing the PWM period, that is a phase 2 latency win. Vibration survival: mount so launch
load is perpendicular to the servo output axis, no slop in the coupling, threadlock the horns.
Slew limiter in firmware caps command rate.

### Power
2S 450mAh LiPo. 5V/3A switching BEC feeds the servo rail and Pico VSYS. 1000uF electrolytic
across the servo rail. Servo brownout during boost is the #1 way these projects die, so the
bench test is all four servos stalling simultaneously while logging.
Master arm: RBF pin + slide switch in series. Software refuses ARM unless still and battery
>= 6.8V, and disarms after 5 minutes on the pad.

## Mechanical

- Fin can at the aft end, 3D printed (PETG or CF-nylon), designed in OpenSCAD in `hardware/`.
- 4 fins on 3mm fiberglass or CF shafts through the wall, each shaft driven directly by its
  servo (direct drive preferred over pushrods: no backlash, fewer failure modes, flutter
  resistance). Max deflection +/-15 deg initially, hard stops at ~25 deg.
- Fin size and placement from the OpenRocket model when it lands. Static margin target 1.5-2
  calibers with fins neutral.
- Avionics sled inside the 2 in tube: 4 servos around the perimeter, Pico 2 + MS5611 + flash
  stacked center, battery forward, IMU hard-mounted mid-sled.
- Shaft pass-through holes tight on the shaft. Wobble = play = flutter.

## Software

Firmware lives in `firmware/`, Pico SDK C. Full file map and wiring table in
`firmware/README.md`.

### State machine
```
PADIDLE -> ARMED -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED
```
- ARMED requires: RBF pulled, gyro stillness calibration passed (750 consecutive quiet
  samples, 3s), battery >= 6.8V. 5-minute pad timeout returns to PADIDLE.
- LIFTOFF: axial accel > 3g sustained 10 samples (40ms).
- BOOST -> COAST: axial accel < 0.5g sustained 25 samples.
- APOGEE: v_hat < 5 m/s or baro altitude fallen 2m below max.
- Fins are driven ONLY in BOOST and COAST while v_hat is in the authority window
  (~15-150 m/s). Every other state: servos commanded to exact center.
- Watchdog fed only inside the control ISR. IMU SPI failure > 200ms: fins neutral + fault.
  Every transition and fault is captured by the 250Hz log.

### Roll control law (phase 1)
- omega_z from gyro, PI on rate error (target 0), no D (differentiating a gyro is noise).
- Gain scheduling: fin effectiveness scales with v^2, so gains scale (v_ref/v)^2 above 25 m/s
  to keep loop gain roughly constant and prevent overcorrection near burnout.
- v_hat = integral(axial accel - g) dt, clamped >= 0, zeroed on pad. Documented approximation,
  good enough for gain scheduling; apogee detection cross-checks with baro.
- Anti-windup: integral frozen while output saturates.

### Attitude control (phase 2, design now, build later)
- Gyro integration + accel gravity-vector correction (complementary filter). During boost the
  accel measures thrust, not gravity, so the correction is gated to off-rail and late coast.
- Pitch/yaw PID -> collective fin commands added to the roll channel.
- Runs on the same Pico 2: FPU + core 1 spare capacity cover it.

### Log record (36B packed)
t, ax/ay/az (mg), gx/gy/gz (dps x10), v_hat (cm/s), MS5611 D1, D2, delta_cmd (deg x100),
battery (mV), state, fault flags, reserved.

## Simulation

`sim/roll_sim.py`: 1-DOF roll dynamics + the same discrete PI the firmware runs (250Hz,
20ms servo lag, gyro noise). Params at top are placeholders marked TODO until the OpenRocket
file lands. Post-flight, the same model replays logged v(t) to compare commanded vs actual.

## Test campaign (do not skip steps)

1. **Bench:** boot, USB CLI, IMU WHO_AM_I, MS5611 PROM CRC, flash JEDEC ID, servo sweep,
   centers trimmed, brownout test (all 4 servos stalled while logging).
2. **Sim:** gains from `roll_sim.py` pasted into config.h.
3. **Spin test:** USB CLI live view, spin the bay by hand, fins visibly counter-rotate the
   right way. Sign conventions die here or 200m up, pick here.
4. **Locked-fin flight:** everything flies and logs, fins locked center. Validates data
   pipeline, gives real roll-rate history.
5. **Roll-active flight:** mid motor (E/F). Compare log to sim, retune.
6. **Phase 2 attitude flights:** only after two clean roll-active flights.

## OpenRocket TODO (when the file lands)

- Drop the .ork in `openrocket/`. Extract: mass, CG/CP with fins neutral, stability calibers,
  velocity off rail, max Q and Q(t), burnout velocity, roll inertia estimate, motor options.
- Confirm v_off_rail >= 20 m/s on the chosen motor. If not, longer rail or bigger motor.
- Size fins: static margin 1.5-2 cal with fins neutral, hinge moments within servo rating at
  max Q at 15 deg deflection.
- Check CP shift with deflected fins (approximate, OR can't do this directly).
- Feed real numbers into `sim/roll_sim.py` and re-tune gains.

## Risks

| Risk | Mitigation |
|---|---|
| Servo rail brownout during boost | Dedicated BEC, 1000uF cap, 4-servo stall bench test |
| Gyro saturation >5.5 rev/s before control engages | Arm control at liftoff detection, not burnout |
| Fin flutter from linkage play | Direct drive, tight shaft fits, hard stops |
| Static instability (fins too small at neutral) | Size from OpenRocket before cutting anything |
| Vibration aliasing IMU | Hard mount, 1kHz ODR, 4:1 averaging in firmware |
| Hung control loop | Watchdog fed only by control ISR, reboot reason logged |
| Servo/BEC brownout resets MCU | Reset-reason logged, fins safe-center at boot |
| Wrong sign convention on fins | Spin test on the ground, step 3 |
| Servo gear failure under launch G | Perpendicular mounting, metal gears, spares |

## Milestones

- M0: plan + BOM done, parts ordered
- M1: firmware builds, bench suite green (IMU/baro/flash IDs, servo sweep, stall test) ~~in progress~~
- M2: fin can + sled built (after OpenRocket sizing), spin test passed
- M3: locked-fin flight, full log recovered
- M4: first roll-active flight
- M5: phase 2 attitude control
