# BOM, phase 1 (roll control, 4-fin capable)

| Part | Qty | ~$ | Notes |
|---|---|---|---|
| Raspberry Pi Pico 2 | 1 | 5 | RP2350. Pico 2 W +$2 adds WiFi for future telemetry |
| GY-ICM42688-P breakout | 1 | 9 | 3.3V native now, no shifter needed. Adafruit 4636 ($18) if you want hand-holding |
| GY-MS5611 baro breakout | 1 | 8 | MS5611-01BA, SPI |
| W25Q128 SPI flash module | 1 | 5 | 16MB log storage |
| MG90S digital servo (metal gear) | 6 | 21 | 4 fly, 2 spares |
| 2S 450mAh LiPo (XT30 or JST) | 1 | 12 | |
| 5V/3A switching BEC | 1 | 7 | Feeds servo rail + Pico VSYS |
| Slide switch + RBF pin/jack | 1 set | 5 | Arm interlock, in series |
| 1000uF electrolytic, decoupling caps, headers, wire | - | 5 | |
| Resistor divider (10k/2k) for battery sense | 1 | 0.5 | 2S -> ADC |
| 3mm CF or fiberglass rod (fin shafts) | 4 | 8 | Cut to length after fin sizing |
| Fin stock (3mm ply or G10) | - | 10 | Size from OpenRocket |
| PETG or CF-nylon filament | - | have? | Fin can + sled prints |
| Commercial altimeter (existing?) | 1 | 0-60 | Only if not already owned, handles deployment |

**Control electronics subtotal: ~$95** (minus altimeter if owned, minus filament if stocked).

Deliberately NOT bought yet: smaller servos, SD modules, radio modules. All revisit points
after the OpenRocket file lands and M3 flight data exists.
