"""Rallus II v4 — printed nose cone, integral to the slide-in stack.

Conical, 282 mm, base r 25.4 (matches the ORK nose exactly, so V3 sims and
static margins carry over). The stack assembles nose-first:
nose (spigot + key pins) -> canard servo ring (counterbore + pin sockets)
-> camera cage -> avionics blade -> retention ring (3 radial screws into
the tube). The cone's own base rim seats on the tube end; pull-out and
rotation are blocked by the pins and the retention ring.

Ballast: the cavity (z 6..200, open at the base) is filled with lead shot
+ epoxy through the base, then the printed base plug is glued into the
spigot bore. Target assembled mass ~360 g (print ~75 g + ~285 g ballast)
to preserve the CG and 2.48/2.84 cal margins from the V3 sims. Mass is
tunable by ballast fill height; verify on a scale before flight.

Print orientation: tip down (as exported after the flip), PETG or ASA,
0.2 mm layers, 4 walls, 10% infill, NO supports (cone overhang is ~5 deg).
Shock cord: one Ø5 hole through the solid base at r 12.

Run:
    python <skill>/scripts/build.py nosecone.py -o models \
        --section X=0 --material PETG
"""
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

# Base plug (separate part, glued in after ballasting)
PLUG_T = 4.0
PLUG_D = SPIGOT_D - 2 * SPIGOT_WALL - 0.6


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

# Internal baffle ring, kept clear of the sleeve zone (SPLIT - SLEEVE_LEN).
BAFFLE_Z = 60.0
r_baf = r_out(BAFFLE_Z) - SHELL - 0.6
nose_base += Pos(0, 0, BAFFLE_Z - BAFFLE_T / 2) * (
    Cylinder(r_baf, BAFFLE_T, align=(Align.CENTER, Align.CENTER, Align.MIN)) -
    Pos(0, 0, -1) * Cylinder(BAFFLE_ID / 2, BAFFLE_T + 2,
                             align=(Align.CENTER, Align.CENTER, Align.MIN)))

# Key pins at 0/180 deg.
for i in range(PIN_COUNT):
    x = PIN_R * (1 if i == 0 else -1)
    nose_base += Pos(x, 0, -SPIGOT_DEPTH - PIN_LEN + PIN_LEN / 2) * \
        Cylinder(PIN_D / 2, PIN_LEN)

# Shock cord hole: one through-bore along X across the base annulus.
nose_base = cut(nose_base, Pos(0, 0, BASE_SOLID / 2) *
                Rot(0, 90, 0) * Cylinder(CORD_HOLE_D / 2, CONE_R * 2 + 4),
                "shock cord hole")

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

# ---------------------------------------------------------------- base plug
plug = Cylinder(PLUG_D / 2, PLUG_T,
                align=(Align.CENTER, Align.CENTER, Align.MIN))

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
    "nose_base_plug": plug,
}

# Assembly in the use frame: nose over the tube shell and dummies of the
# canard servo ring as designed (counterbored top + key pin sockets).
assembly = {
    "nose": nose,
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
