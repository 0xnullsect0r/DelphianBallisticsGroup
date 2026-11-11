# Rallus II v3 — fin-roll avionics firmware

Pico 2 (RP2350) flight computer for active canard roll control. Rides the
upper section (forward of the explosive-bolt mid-break), drives 4 canards
just below the nose cone. Dual core:

- **core 0** @ 1 kHz: ICM-42688-P sample, flight state machine, roll PI with
  dynamic-pressure gain scheduling and a transonic deflection taper, servo
  PWM, watchdog feed. Nothing else runs here, so control can not be starved.
- **core 1**: MS5611 baro (~50 Hz P/T pairs), W25Q128 flash log writer,
  MAX31855 bay thermocouple (10 Hz), USB CDC command line, LED, battery ADC.

Canards are driven ONLY during BOOST/COAST with v_hat in [15, 275] m/s
(Rallus II burns out around 263 m/s / M0.78; motor is custom propellant,
simmed against the CTI 1115J530 curve as closest equivalent). Deflection is
capped 15 deg below 100 m/s, tapering to 3 deg by 250 m/s. Every other
moment the servos are commanded to exact center. The watchdog is fed only
by the control ISR: a hung control loop resets the whole board.

## Build

Toolchain on Arch:

```
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib cmake
git clone --recursive https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
```

Build (target is `pico2`, set in CMakeLists.txt):

```
cmake -B build -DPICO_SDK_PATH=~/pico-sdk
make -C build -j
```

Flash: hold BOOTSEL while plugging in, copy `build/avionics.uf2` to the
mounted drive. Or with picotool: `picotool load build/avionics.uf2 -fx`.
The CLI command `x` reboots into BOOTSEL without touching the cable.

## Wiring

| Pico 2 GP | Peripheral | Notes |
|---|---|---|
| 2  | ICM-42688-P SCK  | SPI0, IMU only |
| 3  | ICM-42688-P SDI  | MOSI |
| 4  | ICM-42688-P SDO  | MISO |
| 5  | ICM-42688-P CS   | |
| 6  | MS5611 SCLK      | SPI1, baro + flash |
| 7  | MS5611 SDI (MOSI) | MS5611 SDI is data-in |
| 8  | MS5611 SDO (MISO) | |
| 9  | MS5611 CSB       | |
| 10 | W25Q128 CLK      | |
|    | W25Q128 DI <- GP7, DO -> GP8 | shared SPI1 bus |
|    | W25Q128 CS -> GP10 | |
| 17 | MAX31855 CS      | K-type thermocouple, read-only SPI1 device |
|    | MAX31855 CLK <- GP6, DO -> GP8 | module in sled caddy; probe on nosecone skin, JST disconnect at the shoulder |
| 11 | servo 0 signal   | fin 0 |
| 12 | servo 1 signal   | fin 1 |
| 14 | servo 2 signal   | fin 2 |
| 15 | servo 3 signal   | fin 3 |
| 16 | arm interlock    | slide switch + RBF pin in series to GND, pull-up enabled |
| 26 | battery sense    | 10k/2k divider from 2S LiPos, ADC0 |
| 25 | status LED       | onboard |

Power: 2S LiPo -> 5V/3A BEC -> servo rail AND Pico VSYS. 1000uF electrolytic
across the servo rail. Logic and servo grounds tied at one point. Servo
signals at 3.3V into 5V servos are fine.

IMU orientation: mount the Pico flat in the sled inside the UPPER tube
(forward of the mid-break), USB pins toward the nose or tail, so the chip
+Z axis points up the rocket. Chip gz = body roll rate. If mounted
differently, edit `imu_remap()` in `src/imu.c`.

Baro vent: the upper tube is aluminum — drill a 1/8 in vent hole in the
avionics bay or the MS5611 reads trapped pressure.

## Bring-up (test campaign step 1, in order)

1. USB serial (`screen /dev/ttyACM0 115200` or your preferred terminal),
   wait for the banner. Check `imu=ok baro=ok flash=ok`.
2. `s` status: baro pressure should be within a few hPa of the local
   weather. If `promcrc=bad` but pressure is sane, the clone botched the
   PROM CRC word; fine to fly, note it.
3. `w` servo sweep with linkages attached but fins NOT in the airflow.
   Fins should move 0 -> +10 -> -10 -> 0 smoothly.
4. `i` live stream: rotate the bay about its long axis by hand. `wz` should
   track your rotation. Then spin test: spin the bay steadily one way, the
   fins must deflect the SAME direction relative to the body each time you
   spin the SAME way. If the response is backwards, flip `ROLL_DIR` in
   `config.h` to `-1.0f` and retest. Do not fly until this is right.
5. Brownout test: grab all four fins and stall them against the servo while
   streaming. No reboot, no `fl=` fault bits, battery above 6.8 V.
6. Trim: with fins at center (`servos_center` runs at boot), adjust the
   mechanical linkages so fins sit flush. `center_trim[]` in `src/servos.c`
   for electrical trim.

## USB CLI

```
h  help            s  status          i  10Hz live stream
w  servo sweep     d  dump log        e  erase log
f  finalize header b  battery         x  BOOTSEL reboot
```

## Flight log

36-byte records to external W25Q128 (boot flash is never touched), at
phase-aware rates:

| Phase | Rate | Why |
|---|---|---|
| PADIDLE | 10 Hz | boot/pad context |
| ARMED | 50 Hz | bias cal, pad conditions |
| BOOST / COAST / APOGEE | **1 kHz** | every IMU sample and servo command the controller saw |
| DESCENT, first 30 s | 1 kHz | separation event + chute opening |
| DESCENT, after | 100 Hz | ride down |
| LANDED | +10 s, then stop | touchdown capture |

Worst-case full mission ~3.5 MB of the 16 MB chip. The flash header (24 B,
sector 0) stores the flight context: gyro bias at arming, control gains,
cal status. It is written at arming, checkpointed every 256 KB during
flight (a power loss costs at most ~7k records of the index, never the
data), and finalized after landing or via `f`.

Dump and decode (header meta prints to the terminal and lands in the CSV
as a comment row):

```
sudo pacman -S python-pyserial
python3 tools/dump_log.py -o flight.csv
# with the board's MS5611 PROM C1..C6 for independent pressure
# recompensation:
python3 tools/dump_log.py -o flight.csv --prom 46472 43953 28912 27150 32767 28673
```

**Always dump after a flight before erasing or flying again** — a fresh
flight overwrites from the start of flash.

Record layout (`datadefs.h`, little endian, 36 B):

```
u32 t_ms | i16 ax,ay,az mg | i16 gx,gy,gz dps*10 | i32 v_hat cm/s |
u32 d1 | u32 d2 | i16 delta deg*100 | u16 batt mV | u8 state | u8 flags |
i16 tc degC*10 (-32768 = probe fault)
```

Flags: 0x01 logq-drop 0x02 imu-err 0x04 baro-stale 0x08 flash-err
0x10 wdt-reset 0x20 core1-hb 0x40 armed 0x80 bias-cal-ok

State: 0 PADIDLE 1 ARMED 2 BOOST 3 COAST 4 APOGEE 5 DESCENT 6 LANDED

## Tuning

Gains live in `include/config.h` (`ROLL_KP`, `ROLL_KI`). Get starting values
from `../sim/roll_sim.py` (replace the TODO-ORK placeholders with OpenRocket
numbers first), then retune from flight logs after M3/M4.
