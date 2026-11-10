# Rallus II v3 — upper-tube bay hardware

Printable parts + drill template for the avionics bay, camera bay, and
canard servo bay. All dimensions live in `dimensions.scad`; **verify every
line marked VERIFY against your actual parts before printing** (servo hole
pitch, Session cube, battery, Pico standoffs, GY breakout boards).

## Assembly order (top of upper tube down)

| Station | Part | Contents |
|---|---|---|
| 0–38 mm | `canard_servo_ring.scad` | 4x MG90S at 45/135/225/315 deg, outputs up, zip-tie retention, shaft bosses at 12 mm |
| 38–84 mm | `camera_cage.scad` + retention plate | HERO5 Session, lens against the tube wall, aperture self-indexes into the window |
| 84–178 mm | `avionics_sled.scad` | deck: Pico 2 (USB up), 3-slot breakout caddy (MS5611 / W25Q128 / MAX31855), BEC posts, TC lead clip; cradle: 2S 450 on velcro; ring: switch + RFB jack |

Nosecone thermocouple: K-probe bead in a 2 mm pocket ~50 mm from the tip,
270 deg meridian, alu tape over bead + 50 mm chordwise, kapton over that and
every 30 mm down the cone, 2-pin JST at the nose shoulder (cone comes off
for USB access). Keep the lead away from the servo wires where it runs
through the canard bay.
| 178+ mm | (existing) | upper chute + shock cord |

## Angular layout (0 deg = rail button plane)

- 45/135/225/315 deg: canard shafts (clear of rail plane and window)
- 90 deg: camera window + lens
- 180 deg: switch slot + RFB jack (arm side)
- 270 deg: baro vent
- 0 deg seam: LED lightpipe (optional)

## Drilling the tube

`drill_template.scad` is a developed (unrolled) view of every hole. Print
BOTH halves at 1:1 (check a calibration cube first), wrap around the tube,
align the 0 deg seam with the rail button plane — pin through the seam
notches and a rail button hole — tape, center-punch, drill:

| Angle | Station | Hole |
|---|---|---|
| 45/135/225/315 | 12 mm | 3.2 mm canard shafts |
| 90 | 60 mm | 26x26 mm window (drill + file) |
| 270 | 130 mm | 3.2 mm baro vent — stays OPEN |
| 180 | 160 mm | 9x5 mm switch slot |
| 180 | 170 mm | 6 mm RFB jack |
| 0 | 140 mm | 3 mm LED lightpipe (optional) |

Window: drill a pilot, nibble to the square, file smooth, deburr inside.
Lens window gets 2 layers of fresh clear packaging tape per flight; the
baro vent stays open — never tape it.

## Print settings

PETG minimum, CF-nylon preferred for the servo ring (vibration + heat).
4 perimeters, 30 % infill, no supports needed on any part as oriented in
the files. The two sled plates print flat; the servo ring and cage print
as modeled.

## Rendering / exporting

```
openscad -o part.stl canard_servo_ring.scad
```

All four parts render clean in OpenSCAD (validated 2026-11-09). One OpenSCAD
foot-gun to remember when editing: parts rendered together must not touch
(coplanar faces collapse in CGAL) — that's why the exploded views keep gaps.
