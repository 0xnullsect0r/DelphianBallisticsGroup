"""Rallus II v5 — telemetry carriage ("broadcaster nose" chassis).

Modeled FLAT (plate in the XZ plane... concretely: everything is plain
boxes and unrotated Z cylinders, which is both the cleanest CAD and the
print orientation). In the rocket the part stands up: the disc seats on
the nose cone's three printed ledges and the plate rises along the nose
axis carrying:

  +Z face (up in print):  Pi Zero 2 W, vertical, 4x M2.5 standoffs
  -Z face (down in print): RFM95W LoRa module, 4 zip posts
  far end from disc:      GPS patch shelf (horizontal in use)

Flat-frame mapping: X along the rocket axis (disc end at X=PLATE_L),
Y across, Z out of the plate. Nose transform used for the assembly check:
nose_z = (PLATE_L + DISC_T) + 26 - X, nose_x = Z, nose_y = Y.

Ballast (lead + epoxy, ~215 g) is poured below the ledges first; the wire
harness exits through the disc slot.

Run:
    python <skill>/scripts/build.py telemetry_carriage.py -o models \
        --section Y=0 --material PETG
"""
from build123d import *
from printlib import *

# ---------------------------------------------------------------- parameters
# Pi Zero 2 W (VERIFY hole pitch against the board)
PI_L, PI_W = 65.0, 30.0
PI_HOLE_PITCH_L = 58.0
PI_HOLE_PITCH_W = 23.0
PI_HOLE_D = 2.6             # M2.5 self-tap
PI_STANDOFF_H = 5.0

# RFM95W breakout (VERIFY size)
LORA_L, LORA_W = 26.0, 26.0

# GPS patch module (VERIFY)
GPS_W = 25.0

# Structure (flat frame)
PLATE_T = 3.0
PLATE_W = 30.0
PLATE_L = 68.0              # X 0..68; Pi zone X 6..71 via standoff inset
DISC_D = 40.0
DISC_T = 3.0
EMBED = 1.2
GPS_SHELF_X = 46.0          # shelf center along X; nose z ~ 45
FACE = PLATE_T / 2          # +Z face of the plate


def post(x, y, z0, d, h, hole_d):
    """Plain Z-axis post growing +Z from z0, base embedded, hole through."""
    body = Pos(x, y, z0 - EMBED) * Cylinder(
        d / 2, h + EMBED, align=(Align.CENTER, Align.CENTER, Align.MIN))
    hole = Pos(x, y, z0 - EMBED - 1.0) * Cylinder(
        hole_d / 2, h + 2 * EMBED + 3,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    return body - hole


# ---------------------------------------------------------------- body
# The cone narrows as you go up: the head (plate + everything on it) is
# tilted TILT deg so the Pi's top corners follow the wall instead of
# poking through it. The disc stays flat and seats on the ledges.
TILT = 5.2

# Tapered plate: half-width 15 at the base narrowing to 13 at the top,
# tracking the cone so the tilted -Z edge clears the wall too.
PLATE_HW_BASE = PLATE_W / 2
PLATE_HW_TOP = 12.6
with BuildPart() as car:
    with BuildSketch(Plane.XY) as pl:
        with BuildLine():
            Polyline((0, -PLATE_HW_BASE), (PLATE_L, -PLATE_HW_TOP),
                     (PLATE_L, PLATE_HW_TOP), (0, PLATE_HW_BASE), close=True)
        make_face()
    extrude(amount=PLATE_T)
head = Pos(0, 0, -PLATE_T / 2) * car.part

# Pi standoffs on the +Z face: rows 3.5 mm from each board end.
for dx in (6.0 + 3.5, 6.0 + PI_L - 3.5):
    for dy in (-PI_HOLE_PITCH_W / 2, +PI_HOLE_PITCH_W / 2):
        head += post(dx, dy, FACE, 6.5, PI_STANDOFF_H, PI_HOLE_D)

# LoRa zip posts on the -Z face. Rot(180,0,0) mirrors a +Z post through
# the plate plane (x kept, y flips - harmless for symmetric pairs).
for dx in (12.0, 12.0 + LORA_L * 0.8):
    for dy in (-(LORA_W / 2 - 3.0), +(LORA_W / 2 - 3.0)):
        head += Rot(180, 0, 0) * post(dx, dy, FACE, 7.0, 6.0, 3.4)

# GPS shelf: corner-cut plate standing off the -Z face, straddling the
# plate. Corner cuts in the sketch (no oversized chamfers on thin plates).
SW = (GPS_W + 2) / 2
CUT = 4.0
with BuildPart() as shelf_b:
    with BuildSketch(Plane.XY) as sh:
        with BuildLine():
            Polyline((GPS_SHELF_X - SW + CUT, -SW), (GPS_SHELF_X + SW - CUT, -SW),
                     (GPS_SHELF_X + SW, -SW + CUT), (GPS_SHELF_X + SW, SW - CUT),
                     (GPS_SHELF_X + SW - CUT, SW), (GPS_SHELF_X - SW + CUT, SW),
                     (GPS_SHELF_X - SW, SW - CUT), (GPS_SHELF_X - SW, -SW + CUT),
                     close=True)
        make_face()
    extrude(amount=2.0)
head += Pos(0, 0, -FACE - 3.0) * shelf_b.part              # 2 mm gap to plate

# Two small posts pinning the shelf to the -Z plate face.
head += Rot(180, 0, 0) * post(GPS_SHELF_X - SW + 6, 0, FACE, 5.0, 2.5, 2.6)
head += Rot(180, 0, 0) * post(GPS_SHELF_X + SW - 6, 0, FACE, 5.0, 2.5, 2.6)

# Tilt the head about the plate base (flat X=0, z=0), then unite with the
# flat disc.
head = Rot(0, TILT, 0) * head

# Disc at the X=0 end (bottom in use), flat, ROUND (a square disc's
# corners cannot follow the bore).
carriage = Pos(-DISC_T / 2 + 0.8, 0, 0) * Rot(0, 90, 0) * Cylinder(
    DISC_D / 2, DISC_T, align=(Align.CENTER, Align.CENTER, Align.CENTER))
carriage += head
# Wire slot + lightening holes in the disc.
carriage -= Pos(-DISC_T - 1, 0, 0) * Box(DISC_T + 2, 10, 8)
for a in (0, 120, 240):
    carriage -= Pos(-DISC_T / 2, 0, 0) * Rot(0, 0, a) * Pos(-9, 14, 0) * \
        Box(DISC_T + 2, 6.5, 6.5, align=(Align.CENTER, Align.CENTER, Align.CENTER))

carriage = carriage.clean()

# ---------------------------------------------------------------- print orientation
# Plate face down, all posts up: already the flat frame, just drop to bed.
printed = Pos(0, 0, -carriage.bounding_box().min.Z) * carriage

results = {"telemetry_carriage": printed}

# ---------------------------------------------------------------- assembly
# Stand it up in the nose. Rot(0,-90,0) maps flat (x,y,z) -> (-z, y, x):
# nose_z = flat X + 29 puts the disc bottom at z 26 (the ledges), plate
# rising to z ~97. Pi face (+flat z) ends up at -nose x; symmetric anyway.
assembly_shape = Pos(0, 0, 29.0) * Rot(0, -90, 0) * carriage

# Tapered cavity dummy: r 22.4 at z 6 down to r 12 at z 140.
cavity_wall = Pos(0, 0, 6) * (
    Cylinder(23.0, 134, align=(Align.CENTER, Align.CENTER, Align.MIN)) -
    Cone(22.4, 12.0, 134, align=(Align.CENTER, Align.CENTER, Align.MIN)))

# Pi board dummy, parallel to the tilted face, in the nose frame.
pi_flat = Rot(0, TILT, 0) * Pos(6.0 + PI_L / 2, 0, FACE + PI_STANDOFF_H + 0.8) * \
    Box(PI_L, PI_W, 1.6, align=(Align.CENTER, Align.CENTER, Align.CENTER))
pi_in_nose = Pos(0, 0, 29.0) * Rot(0, -90, 0) * pi_flat

assembly = {
    "carriage": assembly_shape,
    "nose_cavity_wall": cavity_wall,
    "pi_dummy": pi_in_nose,
}
