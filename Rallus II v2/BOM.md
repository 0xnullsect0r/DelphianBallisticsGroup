# BOM, Rallus II v2 phase 1 (canard roll control, 4 canards)

| Part | Qty | ~$ | Notes |
|---|---|---|---|
| Raspberry Pi Pico 2 | 1 | 5 | RP2350. Pico 2 W +$2 adds WiFi for future telemetry |
| GY-ICM42688-P breakout | 1 | 9 | 3.3V native. 32g range is mandatory on this motor (19.3g peak). Adafruit 4636 ($18) alternative |
| GY-MS5611 baro breakout | 1 | 8 | MS5611-01BA, SPI |
| W25Q128 SPI flash module | 1 | 5 | 16MB log storage |
| MG90S digital servo (metal gear) | 6 | 21 | 4 fly, 2 spares |
| 2S 450mAh LiPo (XT30 or JST) | 1 | 12 | |
| 5V/3A switching BEC | 1 | 7 | Feeds servo rail + Pico VSYS |
| Slide switch + RBF pin/jack | 1 set | 5 | Arm interlock, in series |
| 1000uF electrolytic, decoupling caps, headers, wire | - | 5 | |
| Resistor divider (10k/2k) for battery sense | 1 | 0.5 | 2S -> ADC |
| Brass sheet 1.5mm (canard stock) | 1 | 10 | 4x canards, 35/15.4/28 mm trapezoid per V2.ork |
| Steel sheet 1.5mm (if recutting booster fins) | 1 | 15 | Booster fins grew to 65 mm span; skip if fins already cut at 50 mm and we re-run numbers instead |
| 3mm CF or fiberglass rod (canard shafts) | 4 | 8 | Cut to length after fin can design |
| PETG or CF-nylon filament | - | have? | Fin can + sled prints |
| Commercial altimeter (existing?) | 1 | 0-60 | Fires chutes + explosive bolts. Not this board's job |
| 1/8 in drill bit | 1 | have? | Baro vent hole in the aluminum upper tube |

**Control electronics subtotal: ~$100** (minus altimeter if owned, minus
filament if stocked).

Motor is our own custom propellant (EX) — nothing to buy; the CTI 1115J530
in the OpenRocket file is only the closest-equivalent thrust curve for sims.
When the grain is static-tested, replace the curve and re-run ORK + sim.

Deliberately NOT bought yet: radio modules, smaller servos. Revisit after
M3 flight data and if telemetry gets added.
