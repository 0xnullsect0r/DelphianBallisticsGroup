# Rallus II v5 — wiring

Everything electrical in one place. Pin numbers are the authority in
`firmware/include/config.h`; if this file and that one disagree, the header
wins and this file is wrong.

## Power

```
 2S LiPo (JST) ──┬── BEC A (5 V / 3 A) ──┬── servo rail: 4x MG90S (+ and -)
                 │                       ├── 1000 uF across the rail, at the servos
                 │                       └── Pico VSYS (pin 39)
                 │
                 ├── BEC B (5 V / 3 A) ───── nose plug pin 1 ──> Pi 5V (header pin 2)
                 │
                 ├── 10k ─┬─ 2k ── GND      battery sense
                 │        └────────────────> Pico GP26 (ADC0)
                 │
                 └── pyro channel (below)

 All grounds common: battery -, both BEC outputs -, Pico GND, Pi GND, servo -.
```

Two BECs on purpose. Four stalled MG90S can take all 3 A of one BEC; on a
shared rail that sag reboots the Pi (30 s of no tracking). The pack itself
is not switched: plug the battery in on the pad. The slide switch and the
RBF jack are LOGIC arm inputs, not power switches.

## Arm interlock (Pico GP16, internal pull-up)

```
 GP16 ──┬── slide switch ── GND      closed = SAFE
        └── RBF jack ────── GND      plug inserted = SAFE (use a jack whose
                                     switch contact shorts tip to sleeve, or
                                     a shorted 3.5 mm plug as the RBF pin)
```

In PARALLEL: the pin only floats high (armed) when the switch is in ARM
*and* the RBF plug is pulled. Either one alone keeps it safe.

## Separation charge (pyro) channel

```
 2S + ───────────────┐
                     │
                 [ e-match / bolt charge ]     2-pin screw terminal
                     │
                     ├───────── 10k ───────────── GND     (continuity bleed, ~0.8 mA)
                     │
                     ├── 47k ──┬── 10k ── GND
                     │         └──────────────────> Pico GP27 (ADC1)   ~1.5 V = match present
                     │
                   D │
     GP18 ─ 100R ─ G ┤  IRLZ44N (logic-level N-FET, TO-220: G-D-S left to right, tab = D)
              │    S │
             10k     │
              │      │
             GND    GND
```

- **The 10k from gate to GND is not optional.** While the Pico is in reset
  or BOOTSEL its pins float; without the pulldown the FET can turn on.
- Sense current is 0.8 mA through the match: far below any no-fire rating.
- Build this on a scrap of perfboard, short fat wires on the battery / match
  / FET loop, and test with a 12 V automotive bulb or a 2 ohm resistor in
  place of the match long before a real charge is anywhere near it. The CLI
  `s` command shows `continuity=OK/open`.
- Connect the real charge LAST, on the pad, rocket safed (switch SAFE, RBF
  in), and stand clear before arming. The commercial altimeter stays wired
  to its own charge as the backup.

## Pico 2 signal wiring

| Pico GP (pin) | Goes to |
|---|---|
| GP0 (1) UART0 TX | nose plug pin 3 -> Pi RXD (GPIO15, header pin 10) |
| GP2 (4) / GP3 (5) / GP4 (6) / GP5 (7) | ICM-42688: SCL(SCK) / SDA(SDI) / SDO(AD0) / CS |
| GP6 (9) / GP7 (10) / GP8 (11) | SPI1 SCK / MOSI / MISO, shared by the next three |
| GP9 (12) | GY-63 MS5611 CSB. **Tie its PS pin to GND** (selects SPI) |
| GP10 (14) | W25Q128 CS |
| GP17 (22) | MAX31855 CS (it uses only SCK + MISO) |
| GP11, GP12, GP14, GP15 (15, 16, 19, 20) | servo 0..3 signal |
| GP16 (21) | arm interlock |
| GP18 (24) | pyro FET gate, via 100R |
| GP26 (31) / GP27 (32) | battery sense / pyro continuity |
| 3V3 OUT (36) | 3.3 V to IMU, baro, flash, MAX31855 |
| VSYS (39), GND | from BEC A |

Sensor boards run from the Pico's 3V3 pin, not from 5 V.

## Nose plug (4-pin JST-XH, through the ballast cup conduit)

| Pin | Signal |
|---|---|
| 1 | +5 V from BEC B -> Pi 5V |
| 2 | GND |
| 3 | Pico GP0 TX -> Pi RXD |
| 4 | spare (Pi TXD -> Pico GP1, unused by the firmware) |

The thermocouple keeps its own 2-pin plug at the nose shoulder.

## Inside the nose (Pi Zero 2 W header)

| Pi pin | Goes to |
|---|---|
| 1 (3V3) | GPS 3V3 |
| 17 (3V3) | RFM95 VIN |
| 3 / 5 (SDA / SCL) | GPS SDA / SCL (the Qwiic cable's blue / yellow) |
| 19 / 21 / 23 | RFM95 MOSI / MISO / SCK |
| 26 (CE1) | RFM95 CS (NSS) |
| 11 (GPIO17) | RFM95 RST |
| 10 (RXD) | nose plug pin 3 |
| 2 (5V), 6 (GND) | nose plug pins 1, 2 |
| ANT pad on the RFM95 | 78 mm of solid wire, straight up the nose axis |

The ground tracker is the same minus the GPS and the nose plug, powered from
a USB power bank.
