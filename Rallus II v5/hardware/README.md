# Rallus II v4 — upper-tube bay hardware (printed nose cone)

Printable parts + drill template for the avionics bay, camera bay, and
canard servo bay. All dimensions live in `dimensions.scad`; **verify every
line marked VERIFY against your actual parts before printing** (servo hole
pitch, Session cube, battery, Pico standoffs, GY breakout boards).

## v4: printed nose cone + one-piece slide-in stack

`nosecone.py` (build123d, exports + Bambu profile in `models/`) replaces
the PLA nose with a printed cone that is integral to the stack. The full
upper assembly now slides into the tube as ONE unit, nose first:

  nose (spigot into ring counterbore, 2 key pins into sockets)
   -> canard servo ring (counterbore + pin sockets added in v4)
   -> camera cage -> avionics blade -> retention ring (3 radial screws)

Axial lock: retention ring screws; nose rim seats on the tube end; spigot
bottom seats on the ring counterbore floor. Rotation lock: key pins ->
servo ring -> canard shafts -> tube.

Ballast: cavity (z 6..200) filled with lead shot + epoxy through the base
to ~285 g; assembled nose target ~360 g to preserve the 2.48/2.84 cal
margins (tune fill height, verify on a scale), then glue the base plug
(`nose_base_plug`) into the spigot bore. Shock cord through the base bore.

The cone profile matches the ORK nose exactly, so V3 sims carry over.

SPLIT (256 mm bed limit): the full cone is 299 mm, so it prints as TWO
pieces - nose_base (157 mm, with spigot/pins/cavity) and nose_tip (167 mm,
with a conical sleeve) - glued at the 0.25 mm-clearance sleeve joint with
epoxy BEFORE ballasting. Both print pointy-end-down, no supports.

Print files (`models/`): `rallus_plate_bambu.3mf` = all five parts on one
plate (PETG settings); per-part Bambu projects with the right material per
part: `nose_base_bambu`, `nose_tip_bambu` (PETG), `avionics_blade_bambu`
(PETG), `retention_ring_bambu` (PA-CF, hardened nozzle). Project settings
mirror a real Bambu-saved 3MF (Application + BambuStudio:3mfVersion
metadata, model_settings.config object/plate schema, slice_info header).

## v5: telemetry nose (Pi + GPS + LoRa live in the nose cone)

The metal tube blocks radio, so the whole broadcaster stack rides in the
printed nose cone instead of the blade (blade unchanged except a standoff
axis fix). `telemetry_carriage.py` = a chassis that drops into the nose
cavity onto three new printed ledges (nosecone.py v5): Pi Zero 2 W
vertical on standoffs (plate tilted 5.2 deg to follow the cone), RFM95W
LoRa on the back face, GPS patch on a shelf looking up through the
plastic. Two 6.5 mm holes through the nose wall at z=31 (90/270 deg)
carry SMA bulkheads for the LoRa and WiFi antennas, which sit just above
the tube rim - completely clear of the metal.

**Rev 2026-11-22 (supersedes the paragraph above where they differ).**
No antenna holes, no poured ballast, two ledges instead of three, key pins
on the ballast cup. Assembly, nose held tip-down:

1. Glue `nose_tip` into `nose_base` (sleeve joint, epoxy).
2. Bench: glue two 4 mm x 18 mm dowels into the sockets in the underside of
   `nose_ballast_cup`. Plug the centre conduit and the cross tunnel with
   tape, fill the cup to the brim with lead shot + epoxy (~135-150 g),
   cure, weigh.
3. Bolt Pi / LoRa / GPS to `telemetry_carriage`, solder the 78 mm antenna
   wire, plug in the harness. Slide the carriage into the nose from the
   base, Pi faces toward the 90/270 deg sides (the long side of the stack
   runs along the cross-pin axis), until the disc stops under the ledges.
4. Thread the harness through the cup's conduit, push the cup in behind
   the disc, line the tunnel up with the two holes in the nose wall, push
   the 5 mm x 50 mm cross pin through, flush both sides, a wrap of tape or
   a dab of RTV to keep it.
5. Tie the shock cord to the cross pin inside the conduit. Key pins go
   into the servo ring sockets as the nose seats.

Pull the cross pin and it all comes back out (SD card, repairs).

Prints: PETG; everything is in models/ with Bambu profiles
(rallus_plate_bambu.3mf carries all six parts: nose base, nose tip,
ballast cup, telemetry carriage, blade, retention ring).

## Slide-in avionics mount (PRIMARY electronics carrier)

`avionics_slide_mount.py` (build123d, exports in `models/`) supersedes
`avionics_sled.scad` as the electronics carrier. Two printed parts:

- **avionics_blade** — 44 x 4 x 154 mm diametral blade. Pico 2 on standoffs
  (USB forward, +Y face); four open pockets on the -Y face: MS5611,
  W25Q128, MAX31855 and the ICM-42688 IMU (boards fixed with VHB or epoxy;
  the IMU must be rigid); 33 x 59 mm battery window (flat 2S pack lies in
  it, two velcro straps in the rail notches), BEC zip posts and the RFB jack
  tab (+Y face aft), switch bracket (knob pokes -X through the tube slot),
  TC wire hooks, shock-cord hole at the aft end.
- **retention_ring** — clamps the blade via a key slot (1 M3 radial screw)
  and locks in the tube with 3 radial M3 screws at 0/120/240 deg.

Assembly: wire everything on the bench -> drop the blade in radially at the
sled bay -> rotate vertical -> slide forward until it seats on the camera
cage -> ring onto the blade aft end, key slot up -> tighten 3 wall screws +
1 blade screw -> shock cord through the blade's aft hole, knot forward.
Hardware: 4x M2.5x8 (Pico), 4x M3x6 (ring), velcro strap, 2-pin JST at the
nose shoulder (thermocouple).

Tube stations (drill template updated to match): switch slot 218.5 mm,
RFB jack 220 mm, vent 130 mm, window 60 mm, canard shafts 12 mm.

Print: PETG minimum, CF-nylon preferred for the ring; blade as exported
(pocket side up, standoffs down: needs supports under the blade); ring flat as exported; no supports needed (bridges < 10 mm).

Print settings and material profiles are EMBEDDED in the Bambu project
files: `models/avionics_blade_bambu.3mf` (PETG, 250/70) and
`models/retention_ring_bambu.3mf` (PA-CF, 280/70) — layer 0.2, 4 walls,
30/35 % grid infill. Open in Bambu Studio, answer the load-project dialog
with "apply all settings", pick your printer. PA-CF needs a hardened
nozzle and dry filament; switch the ring to PETG temps if you print it in
PETG. `models/all_parts.3mf` stays a plain plate (no settings).

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
| 180 | 218.5 mm | 9x5 mm switch slot |
| 180 | 220 mm | 6 mm RFB jack |
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
