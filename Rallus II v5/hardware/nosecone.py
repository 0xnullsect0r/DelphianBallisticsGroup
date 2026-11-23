"""Rallus II v4 — printed nose cone, integral to the slide-in stack.

Conical, 282 mm, base r 25.4 (matches the ORK nose exactly, so V3 sims and
static margins carry over). The stack assembles nose-first:
nose (spigot + key pins) -> canard servo ring (counterbore + pin sockets)
-> camera cage -> avionics blade -> retention ring (3 radial screws into
the tube). The cone's own base rim seats on the tube end; pull-out and
rotation are blocked by the pins and the retention ring.

Ballast (rev 2026-11-22): NOT poured in place. A printed cup
(`nose_ballast_cup`) is filled with lead shot + epoxy on the bench, slides
into the base under the telemetry carriage and is locked by a 5 mm cross
pin; pull the pin to get everything back out. Target finished nose mass
~280-315 g (see PLAN "v5 audit"); verify on a scale and re-run
sim/stability.py --nose with the real numbers.

Print orientation: tip down (as exported after the flip), PETG or ASA,
0.2 mm layers, 4 walls, 10% infill, NO supports (cone overhang is ~5 deg).
Shock cord: one Ø5 hole through the solid base at r 12.

Run:
    python <skill>/scripts/build.py nosecone.py -o models \
        --section X=0 --material PETG
"""
from math import radians, cos, sin

from build123d import *
from printlib import *

# ---------------------------------------------------------------- parameters
# Outer cone (matches Rallus Mk.II ORK nose cone)
CONE_L = 282.0
CONE_R = 25.4            # base radius = tube OD
TIP_R = 1.5              # blunt tip

# Tube interface (upper tube: ID 46.8)
TUBE_ID = 46.8
SPIGOT_D = TUBE_ID - 0.4 # slip fit into the tube ID
SPIGOT_WALL = 2.4
SPIGOT_DEPTH = 5.0       # seats in the servo ring counterbore

# Key pins: anti-rotation + anti-pullout into the canard servo ring
PIN_D = 4.0
PIN_LEN = 12.0
PIN_R = 17.0             # radius from the axis; angles 0/180
PIN_COUNT = 2

# Structure
SHELL = 2.0              # cone wall
BASE_SOLID = 6.0         # solid ring at the base
CAVITY_TOP = 200.0       # solid tip above this
BAFFLE_Z = 120.0         # internal ring for shell stiffness
BAFFLE_ID = 12.0
BAFFLE_T = 2.4
CORD_HOLE_D = 5.0
CORD_HOLE_R = 12.0



def r_out(z):
    """Outer cone radius at height z."""
    return CONE_R + (TIP_R - CONE_R) * z / CONE_L


def r_in(z):
    return r_out(z) - SHELL


# ---------------------------------------------------------------- nose pieces
# The 299 mm cone does not fit a 256 mm build volume in one piece: split at
# SPLIT_Z into base + tip with a glued conical sleeve joint. The tip's
# sleeve drops into the base cavity with 0.25 mm clearance and 25 mm of
# glue length; the joint sits where the cone is still thick-walled.
SPLIT_Z = 140.0
SLEEVE_LEN = 25.0
SLEEVE_CLEAR = 0.25

def sleeve_r_out(z):
    return r_in(z) - SLEEVE_CLEAR          # sleeve OD against base cavity

# Base piece: spigot + pins + cord hole + cavity up to the split plane.
base_pts = [
    (SPIGOT_D / 2, -SPIGOT_DEPTH),                    # spigot outer bottom
    (SPIGOT_D / 2, 0.0),                              # spigot outer top
    (CONE_R, 0.0),                                    # base rim (seats on tube)
    (r_out(SPLIT_Z), SPLIT_Z),                        # cone to split plane
    (r_in(SPLIT_Z), SPLIT_Z),                         # split face (annulus)
    (r_in(BASE_SOLID), BASE_SOLID),                   # cavity wall down
    (SPIGOT_D / 2 - SPIGOT_WALL, 0.0),                # spigot inner top
    (SPIGOT_D / 2 - SPIGOT_WALL, -SPIGOT_DEPTH),      # spigot inner bottom
]

with BuildPart() as base_builder:
    with BuildSketch(Plane.XZ) as prof:
        with BuildLine():
            Polyline(*[(r, z) for r, z in base_pts], close=True)
        make_face()
    revolve(axis=Axis.Z)

nose_base = base_builder.part

# v5 (rev 2026-11-22): the telemetry carriage lives in the cavity. Loading
# order is everything-from-the-base: carriage first (its disc stops UNDER
# two ledges), then the removable ballast cup, then one cross pin through
# the nose wall and the cup locks the lot.
#
# Two ledges only, at 0/180 deg (the +/-X sides). The loaded carriage is an
# 18 x 30 mm slab in plan (plate + Pi + LoRa, long side along Y); ledges on
# the X sides are the only place it can slide past. Three ledges at 120 deg
# (the first v5 layout) blocked the Pi corners.
LEDGE_Z = 23.0           # ledge centre; underside (disc seat) at 21.5
LEDGE_T = 3.0
LEDGE_W = 14.0
LEDGE_SEAT_Z = LEDGE_Z - LEDGE_T / 2
for ang in (0, 180):
    # Buried well into the sloped shell wall (no tangent slivers): inner
    # face reaches r 16.5 to catch the r 20 carriage disc.
    # Trimmed to stay 0.4 mm inside the outer skin (the box corners would
    # otherwise break through the cone as bumps).
    ledge = Rot(0, 0, ang) * Pos(20.0, 0, LEDGE_Z) * Box(7.0, LEDGE_W, LEDGE_T)
    nose_base += ledge & (Pos(0, 0, LEDGE_Z) *
                          Cylinder(r_out(LEDGE_Z + LEDGE_T / 2) - 0.4, LEDGE_T + 2))

# No antenna holes: the LoRa antenna is a 78 mm wire inside the nose and the
# Pi uses its on-board WiFi antenna. (Whips in the airstream just ahead of
# the canards were a bad idea; the first v5 revision had two SMA holes.)

# Key pins (anti-rotation into the canard servo ring sockets at r 17):
# these used to be modeled here, hanging in the open spigot bore attached
# to nothing - they exported as two loose cylinders. They now live on the
# ballast cup, which closes the bore and is locked to the nose by the
# cross pin (see below).

# Cross-pin bore along Y (90/270 deg, clear of the key pins on X) through
# both sides of the base at z = 3. A 5 mm
# pin (CF rod / dowel, cut flush, 50 mm) goes through the nose wall, through
# the ballast cup's tunnel, and out the other side: it retains the cup and
# is the shock-cord anchor (tie the cord to the pin inside the conduit).
PIN_Z = BASE_SOLID / 2
nose_base = cut(nose_base, Pos(0, 0, PIN_Z) *
                Rot(90, 0, 0) * Cylinder(CORD_HOLE_D / 2, CONE_R * 2 + 4),
                "cross pin bore")

nose_base = nose_base.clean()

# Tip piece: cone from split plane to tip, solid above CAVITY_TOP, with a
# skirt reaching into the base cavity (0.25 mm clearance) and a solid web
# spanning the split plane +3 mm that joins skirt, shell, and cavity wall
# into one boundary (no self-intersecting horizontals: the two z=SPLIT
# segments cover disjoint radii).
WEB_Z = 3.0
tip_pts = [
    (sleeve_r_out(SPLIT_Z - SLEEVE_LEN), SPLIT_Z - SLEEVE_LEN),  # skirt OD bottom
    (sleeve_r_out(SPLIT_Z), SPLIT_Z),                            # skirt OD top
    (r_out(SPLIT_Z), SPLIT_Z),                                   # split face (outer)
    (TIP_R, CONE_L),                                             # cone surface up
    (0.0, CONE_L),                                               # tip face
    (0.0, CAVITY_TOP),                                           # solid tip underside
    (r_in(CAVITY_TOP), CAVITY_TOP),                              # cavity top
    (r_in(SPLIT_Z + WEB_Z), SPLIT_Z + WEB_Z),                    # cavity wall to web
    (0.0, SPLIT_Z + WEB_Z),                                      # web top face
    (0.0, SPLIT_Z),                                              # axis through web
    (sleeve_r_out(SPLIT_Z) - SHELL, SPLIT_Z),                    # skirt ID top
    (sleeve_r_out(SPLIT_Z - SLEEVE_LEN) - SHELL, SPLIT_Z - SLEEVE_LEN),  # skirt ID bottom
]

with BuildPart() as tip_builder:
    with BuildSketch(Plane.XZ) as prof:
        with BuildLine():
            Polyline(*[(r, z) for r, z in tip_pts], close=True)
        make_face()
    revolve(axis=Axis.Z)

nose_tip = tip_builder.part.clean()

# Assembled nose (for the interface checks and renders).
nose = nose_base + nose_tip

# ---------------------------------------------------------------- ballast cup
# Replaces the poured-in-place ballast and the glued base plug. A printed
# cup, filled with lead shot + epoxy ON THE BENCH, slides up the spigot bore
# under the carriage disc and is locked by the cross pin. Pull the pin and
# everything comes back out: the Pi and its SD card stay serviceable.
#   - centre conduit: the nose harness (and its plug) passes through
#   - cross tunnel at PIN_Z: sleeve for the 5 mm pin, sealed from the lead
# Nose frame: floor at z = -SPIGOT_DEPTH (flush with the spigot end), rim
# just under the carriage disc.
CARRIAGE_DISC_T = 3.0                      # telemetry_carriage.DISC_T
CUP_BORE_R = SPIGOT_D / 2 - SPIGOT_WALL    # spigot bore radius, 20.8
CUP_R = CUP_BORE_R - 0.3                   # slip fit
CUP_Z0 = -SPIGOT_DEPTH
CUP_Z1 = LEDGE_SEAT_Z - CARRIAGE_DISC_T - 0.3
CUP_H = CUP_Z1 - CUP_Z0
CUP_WALL = 1.2
CUP_FLOOR = 1.6
CONDUIT_ID = 13.0                          # passes a 4-pin JST-XH plug
CONDUIT_OD = CONDUIT_ID + 2 * CUP_WALL
TUNNEL_ID = CORD_HOLE_D + 0.4
TUNNEL_OD = TUNNEL_ID + 2 * CUP_WALL

_cyl = lambda r, h: Cylinder(r, h, align=(Align.CENTER, Align.CENTER, Align.MIN))
cup = _cyl(CUP_R, CUP_H) - Pos(0, 0, CUP_FLOOR) * _cyl(CUP_R - CUP_WALL, CUP_H)
cup += _cyl(CONDUIT_OD / 2, CUP_H)                                  # conduit wall
_tunnel_z = PIN_Z - CUP_Z0
_tunnel = Pos(0, 0, _tunnel_z) * Rot(90, 0, 0) * Cylinder(TUNNEL_OD / 2, 2 * CUP_R - 1.0)
# Tunnel sleeve, with a web down to the floor so it prints without support.
cup += _tunnel & _cyl(CUP_R - 0.2, CUP_H)
cup += Pos(0, 0, CUP_FLOOR / 2 + _tunnel_z / 2) * Box(TUNNEL_OD * 0.7, 2 * CUP_R - 2.0, _tunnel_z) \
    & _cyl(CUP_R - 0.2, CUP_H)
# Key-pin sockets: two bosses inside the cup at r PIN_R (0/180 deg) with a
# blind hole from the underside. Glue in 4 mm x 18 mm dowels (steel dowel
# pin, CF rod, or a cut M4 screw shank); 12 mm sticks out into the servo
# ring sockets.
KEY_SOCKET_DEPTH = 9.0
for sx in (1, -1):
    cup += (Pos(sx * PIN_R, 0, 0) * _cyl(PIN_D / 2 + 1.8, CUP_FLOOR + KEY_SOCKET_DEPTH + 1.2)) \
        & _cyl(CUP_R - 0.2, CUP_H)
    cup = cut(cup, Pos(sx * PIN_R, 0, -1) * _cyl(PIN_D / 2 + 0.05, KEY_SOCKET_DEPTH + 1),
              "key pin socket")
cup = cut(cup, Pos(0, 0, -1) * _cyl(CONDUIT_ID / 2, CUP_H + 2), "harness conduit")
cup = cut(cup, Pos(0, 0, _tunnel_z) * Rot(90, 0, 0) * Cylinder(TUNNEL_ID / 2, 2 * CUP_R + 4),
          "pin tunnel")
cup = cup.clean()

# Lead fill volume (for the mass budget printed at build time).
_fill = (Pos(0, 0, CUP_FLOOR) * _cyl(CUP_R - CUP_WALL, CUP_H - CUP_FLOOR)) - cup \
    - _cyl(CONDUIT_OD / 2, CUP_H + 1) \
    - Pos(0, 0, _tunnel_z) * Rot(90, 0, 0) * Cylinder(TUNNEL_OD / 2, 2 * CUP_R + 4)
BALLAST_CM3 = _fill.volume / 1000.0
print(f"ballast cup: {CUP_H:.1f} mm tall, fill volume {BALLAST_CM3:.1f} cm3 "
      f"-> ~{BALLAST_CM3 * 6.8:.0f} g lead shot + epoxy (6.8 g/cm3), "
      f"~{BALLAST_CM3 * 7.6:.0f} g if tamped fine shot")

# ---------------------------------------------------------------- print orientation
# Both cone pieces print "pointy end down" (5 deg overhang, features on
# top, no supports). Base piece: split-plane face on the bed, pins up.
# Tip piece: tip on the bed, sleeve up.
printed_base = Rot(180, 0, 0) * nose_base
printed_base = Pos(0, 0, -printed_base.bounding_box().min.Z) * printed_base
printed_tip = Rot(180, 0, 0) * nose_tip
printed_tip = Pos(0, 0, -printed_tip.bounding_box().min.Z) * printed_tip

# ---------------------------------------------------------------- outputs
results = {
    "nose_base": printed_base,
    "nose_tip": printed_tip,
    "nose_ballast_cup": cup,          # prints floor down, as modeled
}

# Assembly in the use frame: nose over the tube shell and dummies of the
# canard servo ring as designed (counterbored top + key pin sockets).
assembly = {
    "nose": nose,
    "ballast_cup": Pos(0, 0, CUP_Z0) * cup,
    "key_pins": Pos(PIN_R, 0, CUP_Z0 - PIN_LEN) * _cyl(PIN_D / 2 - 0.05, PIN_LEN + KEY_SOCKET_DEPTH - 0.2)
                + Pos(-PIN_R, 0, CUP_Z0 - PIN_LEN) * _cyl(PIN_D / 2 - 0.05, PIN_LEN + KEY_SOCKET_DEPTH - 0.2),
    "cross_pin": Pos(0, 0, PIN_Z) * Rot(90, 0, 0) * Cylinder(CORD_HOLE_D / 2 - 0.15, 50.0),
    "carriage_disc": Pos(0, 0, LEDGE_SEAT_Z - CARRIAGE_DISC_T) * (
        Cylinder(20.0, CARRIAGE_DISC_T, align=(Align.CENTER, Align.CENTER, Align.MIN)) -
        Pos(0, 0, -1) * Cylinder(6.5, CARRIAGE_DISC_T + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))),
    # tube ends AT the cone seating plane
    "tube": Pos(0, 0, -60) * ((Cylinder(25.4, 60, align=(Align.CENTER, Align.CENTER, Align.MIN))) -
                              (Cylinder(23.4, 60, align=(Align.CENTER, Align.CENTER, Align.MIN)))),
    # ring counterbore floor: annulus the spigot bottom seats on
    "ring_floor": Pos(0, 0, -8) * ((Cylinder(23.2, 3, align=(Align.CENTER, Align.CENTER, Align.MIN))) -
                                   (Cylinder(20.8, 3, align=(Align.CENTER, Align.CENTER, Align.MIN)))),
    # ring body below the floor, pin sockets drilled 12 deep from the
    # counterbore floor (stations 5..17 from the ring top)
    "ring_body": Pos(0, 0, -38) * ((Cylinder(23.2, 30, align=(Align.CENTER, Align.CENTER, Align.MIN)) -
                                    Cylinder(4.0, 30, align=(Align.CENTER, Align.CENTER, Align.MIN)) -
                                    Pos(17, 0, 27) * Cylinder(2.1, 12) -
                                    Pos(-17, 0, 27) * Cylinder(2.1, 12))),
}
