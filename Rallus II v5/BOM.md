# BOM, Rallus II v5 (live telemetry: Pi + GPS + LoRa + pyro)

| Part | Qty | ~$ | Notes |
|---|---|---|---|
| Raspberry Pi Pico 2 (with headers) | 1 | 5-8 | RP2350 flight computer |
| Raspberry Pi Zero 2 W (with headers) | 2 | 2x 20-25 | one in the nose (broadcaster), one ground tracker |
| microSD 16-32 GB | 2 | 2x 7 | one per Pi |
| I2C GPS: Adafruit Mini GPS PA1010D (or a u-blox with DDC broken out) | 1 | 30 | rides the Pi's I2C. NOT a BN-880/"M8N + compass" puck: its I2C is the compass only |
| RFM95W LoRa breakout, 915 MHz (Adafruit 3072 or equal) | 2 | 2x 20 | rocket + ground tracker. Antenna = 78 mm wire on each, inside the nose on the rocket |
| N-FET (IRLZ44N / AO3400 class, logic level) + 2-pin screw terminal + resistors 10k x3, 47k, 100R | 1 set | 3 | separation charge channel: gate pulldown 10k + 100R series, continuity divider |
| USB power bank + micro-USB cable | 1 | have? | powers the ground tracker |
| GY-ICM42688-P breakout | 1 | 9 | 3.3V native. Max range is +/-16 g (there is no 32 g mode on this part); the 18 g thrust peak clips briefly, firmware tolerates it |
| GY-MS5611 (GY-63) baro breakout | 1 | 8 | MS5611-01BA, SPI: tie PS to GND |
| W25Q128 SPI flash module | 1 | 5 | 16MB log storage |
| MAX31855 module + K-type probe (long lead) | 1 | 7 | Nosecone skin temp, SPI1, log field tc_c10; JST pair at the nose shoulder |
| Aluminium + kapton tape | - | 3 | Probe bonding on the nosecone (alu over bead, kapton over lead) |
| MG90S digital servo (metal gear) | 6 | 21 | 4 fly, 2 spares |
| 2S 850mAh LiPo, JST (Zeee 54x30x15) | 1 (2-pack) | 21 | Lies in the blade window. ~3.5 h with the Pi on board. A Gens ace 2S 450 (56x31x11) also fits, ~1.8 h |
| 5V/3A switching BEC | 2 | 2x 7 | one for servo rail + Pico VSYS, a SEPARATE one for the Pi (4 stalled servos alone are ~3 A; a sagging rail reboots the Pi for 30 s) |
| Slide switch + RBF pin/jack | 1 set | 5 | Arm interlock, in series, mounts in sled bottom ring |
| 1000uF electrolytic, decoupling caps, headers, wire | - | 5 | |
| Resistor divider (10k/2k) for battery sense | 1 | 0.5 | 2S -> ADC |
| Brass sheet 1.5mm (canard stock) | 1 | 10 | 4x canards, 35/15.4/28 mm trapezoid per the ORK |
| Steel sheet 1.5mm (booster fins) | 1 | 15 | Booster fins are now 141 mm root / 36.4 mm tip / **80 mm span** (OpenRocket needs it for margin on the real motor). Recut if yours are 50 or 65 mm |
| 3mm CF or fiberglass rod (canard shafts) | 4 | 8 | Cut to length after fin can design |
| Lead shot + epoxy (nose ballast) | - | 10 | ~135-150 g cast into the printed ballast cup; finished nose ~280 g |
| 5 mm rod x 50 mm (CF or steel) + 2x 4 mm x 18 mm dowel pins | 1 set | 3 | nose cross pin (locks ballast cup, shock cord anchor) + key pins glued into the cup |
| Resistor kit, 1000 uF 16 V cap, perfboard, 2-pin screw terminals, JST-XH kit, 2x20 pin headers | - | 25 | pyro board, sense dividers, nose plug, Pi headers. See hardware/WIRING.md |
| GoPro HERO5 Session | 1 | owned | Used market ~$40-60 if needed; 37mm cube, verify vs dimensions.scad |
| Clear packaging tape | 1 | have? | 2 fresh layers over the camera window per flight |
| Foam sheet, 1.5mm + velcro strap | - | 3 | Camera shock mount, battery strap |
| M2.5x8 screws (4) + M3x6 screws (4) | 1 set | 2 | Pico standoffs + retention ring clamp |
| PETG or CF-nylon filament | - | have? | Servo ring (CF-nylon preferred), cage, sled, templates |
| Commercial altimeter (existing?) | 1 | 0-60 | Fires chutes + explosive bolts. Not this board's job |
| Drill bits: 3.2, 6mm + needle files | - | have? | Vent, RFB, shafts, window filing |

**Electronics subtotal: ~$230** with both Pis, GPS and radios (minus
altimeter if owned, minus filament if stocked).

Motor is our own propellant: 60/40 KNO3/erythritol, 874 g in six 42.8 x
76.2 mm BATES grains with 20 mm cores, 9.5 mm throat (see PLAN "Motor on
the real propellant"). Propellant ingredients and casting supplies are not
on this list. `openrocket/Rallus_EX_J323_nominal.eng` is the simulated
curve until there is a static test.

Deliberately NOT bought: external SMA whips for the rocket (antennas stay
inside the printed nose), smaller servos.
