# Rallus II v3 — Canard Roll Control Avionics

Active canard control for Rallus II, 2 in airframe, custom EX J-class solid
motor (fixed impulse, no throttle). Phase 1: roll control (anti-spin).
Phase 2: boost-phase attitude stabilization (pitch/yaw). Hardware is
4-canard capable from the start so phase 2 is a firmware update, not an
airframe rebuild.

v1 and v2 are frozen archives. Everything active lives here.

## v3 changes (2026-11-09)

Everything from v2 carries over (canard sizing, fin optimization, sim round
1 fixes). v3 adds the physical build-out:

1. **Dedicated bay hardware** (`hardware/`, OpenSCAD, all render-validated):
   canard servo ring, HERO5 Session camera cage, avionics sled (Pico 2
   standoffs, 3-slot breakout caddy, BEC posts, battery cradle, switch/RFB
   ring, thermocouple probe clip), and a 2-piece drill template for the
   aluminum tube. Layout table + angular map in `hardware/README.md`.
2. **Nosecone thermocouple**: MAX31855 (in the sled caddy) + K-type probe
   bonded to the PLA nosecone skin ~50 mm from the tip on the 270 deg
   meridian — stagnation-region aero heating, the number that says whether
   the PLA is happy at M0.73. Read at 10 Hz on SPI1, temperature in every
   log record (`tc_c10`, degC x 10, INT16_MIN = probe fault). Mount: 2 mm
   pocket for the bead, alu tape over the bead + 50 mm of lead laid
   chordwise, kapton over that and every 30 mm down the cone, 2-pin JST
   disconnect at the nose shoulder so the cone still comes off for USB
   access. Log format v3.
3. **Camera bay**: GoPro HERO5 Session just below the canard bay, lens
   straight out the side, 26x26 mm window in the tube taped over with clear
   packaging tape (2 layers, fresh each flight). The camera bay is NOT
   pressure-sealed; the avionics sled bay above it is the sealed volume for
   the MS5611 (vent at 270 deg stays open — only the window is taped).
4. **ORK V3**: the single 0.2 kg "Avionics Bay" placeholder became three
   mass objects at real stations — Canard Servo Bay 65 g @ 0.30 m, Camera
   Bay 90 g @ 0.34 m (Session 74.5 g + cage), Avionics Sled 145 g @ 0.39 m
   (includes the MAX31855 + probe). Net +95 g, all forward of the old
   placeholder's CG: liftoff 4.67 kg, margins IMPROVE to 2.48 / 2.84 cal
   (loaded/burnout), ~1.97 est at M0.78. Same aero and canard authority
   ratios as v2.

Upper-tube station budget: canard bay 0-38, camera bay 38-84, avionics sled
84-178, chute 178-305 mm. Angular map: canards 45/135/225/315, window 90,
switch+RFB 180, vent 270 (deg, 0 = rail button plane).

## Vehicle (Rallus Mk.II V3.ork)

Two-stage model of the mid-break airframe. Separation at apogee — in the
real rocket the halves are held by explosive bolts and split at the break;
each half recovers under its own chute (24 in upper, 30 in booster).

| | |
|---|---|
| Total length | 1.17 m (nose 282 + upper tube 305 + coupler 25 + booster 559 mm) |
| Diameter | 2.0 in (50.8 mm OD) |
| Upper section | PLA conical nose (362 g) + aluminum tube, 2 mm wall, 305 mm |
| Booster | steel tube, 3 mm wall, 559 mm + 4 steel fins, 141 mm root, 65 mm span |
| Canards | 4x brass, 35 mm root / 15.4 mm tip / 28 mm span, 35 deg LE sweep, 1.5 mm |
| Motor | Custom EX propellant, modeled as CTI 1115J530-15A (~530 N, ~2.1 s) |
| Liftoff mass | ~4.67 kg (component sum incl. camera + thermocouple bays) |

V1 sim reference: apogee 2485 m, 263 m/s (M0.78) burnout, 19.3 g, 33.8 m/s
off a 3 m rail. V2 is slightly draggier (bigger surfaces); re-sim pending.

**Control section = upper half only.** The break at the coupler means the
canards, servos, avionics, and battery live in the aluminum upper tube,
forward of the split. The booster is passive.

## Feasibility & optimization pass (2026-11-09, `sim/stability.py`)

Numbers from a Barrowman + mass-model analysis of the ORK; final authority
is an OpenRocket run (checklist at the bottom).

### Verdicts

| Quantity | As received (V1) | V2 revised | Status |
|---|---|---|---|
| Canard roll authority vs 0.5 deg cant @ 15 deg | 3.1 | 13.2 | fixed (was borderline) |
| Canard roll authority vs 0.5 deg cant @ 3 deg (transonic cap) | 0.63 | 2.63 | **fixed: was infeasible** |
| Static margin loaded / burnout | 3.5 / 3.9 cal | 2.35 / 2.70 cal | traded for canards, healthy |
| Static margin at M0.78 (est) | 2.6 cal | 1.84 cal | verify in OR, target >= 1.5 |
| Roll accel authority, 35 / 100 / 263 m/s | - | 1.6k / 2.5k / 15.5k deg/s^2 | massive rate damping margin |
| Canard hinge moment, worst case | - | ~0.053 N·m | 4x inside MG90S (0.22 N·m) |
| Booster half descent | 10.5 m/s (24 in) | ~7.9 m/s (30 in) | fixed |
| Upper half descent | ~6 m/s | unchanged | ok |
| TWR at liftoff | 12:1 | ~12:1 | ok |
| Gyro saturation risk | - | uncontrolled 0.5 deg cant = 3900 deg/s^2 spin-up at burnout | control live from liftoff covers it |

### What changed and why

1. **Canards 2x(20x10 mm, 1 mm) -> 4x(35/15.4/28 mm, 35 deg sweep, 1.5 mm
   brass, 7.06 cm^2 each).** The drawn canards were control-authority-starved
   at the transonic taper: a single booster fin canted half a degree
   (realistic build tolerance) produced 1.6x more roll torque than the
   canards could counter at their 3 deg burnout limit -> integrator rails,
   residual torque spins the rocket. 7.06 cm^2 restores a 2.6x margin at the
   cap and 13x at full deflection. Sized by sweeping geometry against the
   A/D ratio AND the static-margin constraint simultaneously (`--canard-sweep`).
2. **Booster fin span 50 -> 65 mm** (root 141 mm and LE sweep angle kept,
   tip chord shrinks to 55.5 mm, area 63.8 cm^2 each, +47 g total). Big
   canards shift CP forward; the span buys the transonic margin back
   (est 1.06 -> 1.84 cal at M0.78). Steel, 1.5 mm, airfoil section: no
   flutter concern at this span and speed.
3. **Booster chute 24 -> 30 in.** Booster half is ~3.1 kg after burnout;
   24 in lands it at 10.5 m/s, 30 in at 7.9 m/s.
4. **Avionics bay modeled as a real 0.2 kg** at the canard bay station
   (Pico stack + 4 MG90S + 2S 450 + BEC + sled + shafts).
5. **Sim timestep 0.05 -> 0.01 s**, stale V1 flight data stripped.

### Reviewed and deliberately unchanged

- **Nose cone**: conical PLA, fineness 5.6. An ogive or von Karman would
  shave a few percent CD at M0.78, but it is printed hardware with a decal
  and 362 g of forward mass the stability margin wants. Revisit only if the
  re-sim shows marginal apogee.
- **Upper tube 2 mm alu wall**: overbuilt for pressure, right for carrying
  canard shaft loads and the mid-break loads path. Keep.
- **Booster 3 mm steel tube**: heavy (1.98 kg) but it is the airframe as
  designed and margin is healthy. Not worth fighting.
- **Paper transition at the break**: left as-is in the ORK (mass
  negligible), but the REAL break hardware must be an aluminum/steel coupler
  with the bolt carriers. Estimated 150-250 g at x~0.6 m moves CG forward,
  which IMPROVES margin ~0.2 cal. Add to the ORK once the coupler design
  exists.
- **Rail buttons**: 1010 pair, one per section, correct for the 3 m rail.
- **Motor mount**: 38 mm, flush overhang. Matches the CTI stand-in; swap to
  the measured EX curve after static testing.
- **Chute deploy**: apogee + 2 s both halves (altimeter's job). V1 sim
  deployed at 17.9 m/s; check opening loads on the re-sim.
- **Cant angles**: 0 deg on canards and booster fins (symmetric, as they
  should be; the controller exists precisely because we do not cant).

## Decisions

1. **Scope:** Roll control first, attitude control later.
2. **Recovery + separation:** Commercial altimeter fires charges and bolts.
   This board is control + logging only, rides the upper section.
3. **Motor:** Custom EX propellant, CTI 1115J530 stand-in curve in sims.
4. **Telemetry:** Onboard logging only in phase 1.
5. **Flight computer:** Pico 2 (RP2350) for all phases, Pico SDK C.
6. **IMU:** ICM-42688-P (32 g — mandatory, 19.3 g boost), MS5611 baro.

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
   Arm input (RBF pin, pull-up): GP16
   Battery sense (10k/2k divider): GP26/ADC0
   Status LED: GP25
```

Roll mixing (phase 1): canards 0,2 +delta, 1,3 -delta. Phase 2 adds
collective pitch/yaw on top.

## Hardware

- **Pico 2:** core 0 = 1 kHz control ISR (sole watchdog feeder), core 1 =
  baro/flash/USB/LED. Reset reason logged.
- **ICM-42688-P:** +/-32 g, +/-2000 dps, SPI0, 1 kHz ODR, hard-mounted.
- **MS5611:** SPI1, full datasheet compensation, 1/8 in vent hole in the
  aluminum tube (mandatory, metal blocks pressure).
- **W25Q128:** 16 MB log, 250 Hz x 36 B records, boot flash untouched.
- **Servos:** 4x MG90S. Hinge moment worst case ~0.053 N·m (263 m/s, 3 deg)
  vs 0.22 N·m rating. Mount launch-load perpendicular to output axes.
- **Power:** 2S 450 mAh, 5V/3A BEC, 1000 uF. Bench 4-servo stall test.

## Mechanical

- Canard fin can at the top of the upper tube, just below the nose shoulder:
  4 canards on 3 mm shafts through the wall, direct drive. OpenSCAD in
  `hardware/` when the fin can design starts.
- Avionics sled behind the canard bay in the 0.27 m payload volume: servos
  forward, Pico stack center, battery aft.
- Everything forward of the mid-break coupler.

## Software

Firmware in `firmware/` (builds clean). State machine PADIDLE -> ARMED ->
BOOST -> COAST -> APOGEE -> DESCENT -> LANDED; canards driven only in
BOOST/COAST with v_hat in [15, 275] m/s; deflection taper 15 -> 3 deg across
100-250 m/s; PI with (v_ref/v)^2 gain scheduling; watchdog fed only by the
control ISR; USB CLI (h/s/i/w/d/e/f/b/x).

**Logging (the post-mortem guarantee):** every IMU sample, state, servo
command, battery reading, baro pair, and bay temperature is recorded. 1 kHz
during flight and the first 30 s of descent (separation + chute events),
100 Hz for the rest of descent, 50 Hz armed, 10 Hz idle. Full mission
~3.5 MB of 16 MB. Flash header captures gyro bias + gains at arming,
checkpoints every 256 KB in flight, so even a crash leaves a complete,
decodable record of what the controller saw and did (`tools/dump_log.py` ->
CSV with meta; log format v3 with `tc_c10`).

## Simulation

- `sim/stability.py` — parses the ORK, mass model + Barrowman CP + canard
  authority sizing. Rerun after any geometry change:
  `python3 sim/stability.py openrocket/"Rallus Mk.II V2.ork" --canard-sweep`
- `sim/roll_sim.py` — 1-DOF roll sim, V2 geometry, mirrors the firmware PI
  + taper. Current placeholder-gain result: worst roll ~35 deg/s in boost,
  ~3 deg/s by 10 s. Retune with `--sweep` after OR data lands.
- `firmware/tools/dump_log.py` — flight log -> CSV.

## Test campaign

1. Bench: IDs, baro sanity, servo sweep, 4-servo stall brownout test.
2. Sim gains from roll_sim.
3. Spin test (sets ROLL_DIR).
4. Locked-canard flight, full log.
5. Roll-active flight.
6. Phase 2 attitude flights.

## Sim handoff — run these, bring back the numbers

`openrocket/Rallus Mk.II V2.ork` carries three ready simulations (Flight
simulations tab, "Run simulations" runs all three): **Calm (0.5 m/s)**,
**Crosswind 2 m/s**, **Crosswind 5 m/s gusty**. All use the CTI 1115J530
stand-in motor, 3 m rail, 0.01 s timestep, launch-into-wind.

Manual checks before running:

1. **Motor**: if the sim warns the motor can not be found (database digest),
   reselect Cesaroni 1115J530-15A on the booster motor mount, or the closest
   J (~1100 Ns) and note which.
2. **Launch site**: inherited conditions say Cape Canaveral, 0 m MSL. Set
   your real field elevation/coordinates in each sim's launch site tab, or
   say so and results get read as sea level.

Numbers to bring back (per sim, Calm + Crosswind 2 m/s at minimum):

- apogee, max velocity, max Mach, max acceleration, time to apogee
- velocity at launch rod clearance (events list or exported CSV)
- landing/deployment velocities per stage (two ground hits, two deployments)
- any warnings (yellow icon -> read them out)
- stability margin in calibers, loaded and burnout (main window CG/CP readout,
  toggle motor config)
- best: full CSV export per sim (plot dialog -> Table tab -> export). With
  "Center of pressure" and "CG location" curves added to the plot, the CSV
  carries margin vs time; I take it from there.

## Sim results, round 1 (2026-11-09, three runs x two stage branches)

| Metric | Estimate | OpenRocket | Verdict |
|---|---|---|---|
| Apogee | ~2400-2500 m | 2236-2249 m | ok (V2 is draggier than V1) |
| Max velocity / Mach | 263 / M0.78 | 246.4 / **M0.73** | ok |
| Max acceleration | 19.3 g | 18.2 g | ok (32 g IMU still right) |
| Off rail (3 m) | ~34 m/s | **32.5 m/s @ 0.20 s** | ok, well above 20 |
| Margin at burnout | 2.70 cal | **2.45 cal** (M0.72) | pass |
| Margin min during boost | est 1.84 worst | **1.89 cal at rail exit** | PASS, never below 1.5 |
| Roll inertia at burnout | est 0.0031 | **0.0020 kg·m²** | authority ~1.5x better than estimated |
| Margin after separation | - | nonsense (-7 cal, AoA >100 deg) | expected, Barrowman invalid post-split |

Findings and actions taken from round 1:

1. **Deployment at 20.8 m/s** (the warning you saw): apogee+2 s is too late.
   Both chute delays set to 1.0 s in the ORK -> deploys at ~11 m/s. Real
   config: altimeter fires the bolts at apogee detect, 1 s is inflation lag.
2. **Upper half lands at 11-12 m/s** on a 24 in chute: nose-section chute
   grown to 30 in, same as the booster (both halves now ~8-9 m/s, and you
   only stock one chute size).
3. **Motor ejection at t=8.1 s (246 m/s, near max Q)** appeared in the sim
   from the stand-in motor's 6 s delay. OPERATIONAL RULE: the real EX motor
   must be **plugged / no ejection charge**. An ejection event at max Q would
   fire the mid-break at the worst possible moment. Altimeter + bolts are the
   only deployment path. (ORK motor delay set to 600 s so the event stops
   appearing in sims.)

After these edits: re-run the three sims; expect deploy ~11 m/s, both ground
hits ~8-9 m/s, everything else unchanged. `sim/analyze_flight.py` digests the
exports (`python3 sim/analyze_flight.py ../something*.csv`).

`sim/roll_sim.py` now carries OR-measured numbers (Ixx 0.0020, mass 4.29,
CD calibrated to 246 m/s burnout). Current gains: worst roll ~40 deg/s in
boost, -4 deg/s at 10 s. Sweep suggests Ki 0.05 settles faster; leaving
0.15/0.02 as the flight-one starting point.

## Risks

| Risk | Mitigation |
|---|---|
| Transonic margin thinner than my estimate | OR sim gate: >= 1.5 cal everywhere; else booster span 70 mm or canards 30x24 |
| Booster fin cant eats canard authority | A/D 2.63 at the 3 deg cap vs 0.5 deg cant; jig the fin alignment, 0.2 deg is achievable |
| Transonic flutter / roll reversal | 3 deg hard cap at speed, swept low-AR brass, Q-scheduled gains |
| Servo rail brownout | BEC + 1000 uF + stall test |
| Separation shock at apogee | Control disabled by APOGEE state before the event |
| Gyro saturation before control engages | Arm at liftoff detection |
| Metal tube blocks baro | 1/8 in vent hole |
| Wrong canard sign | Ground spin test |
| Paper break joint in ORK | Real coupler must be metal; add to ORK after design |
| EX curve differs from CTI stand-in | Static test, swap curve, rerun everything |

## Milestones

- M0: plan + BOM done, parts ordered
- M1: firmware builds (done), bench suite green
- M2: OR sims validated (pending handoff numbers), fin can + sled built, spin test
- M3: locked-canard flight
- M4: first roll-active flight
- M5: phase 2 attitude control
