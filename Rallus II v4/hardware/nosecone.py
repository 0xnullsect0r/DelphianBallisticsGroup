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


# ---------------------------------------------------------------- nose shell
# Closed revolve profile in the XZ plane (r, z).
pts = [
    (SPIGOT_D / 2, -SPIGOT_DEPTH),                    # spigot outer bottom
    (SPIGOT_D / 2, 0.0),                              # spigot outer top
    (CONE_R, 0.0),                                    # base rim (seats on tube)
    (TIP_R, CONE_L),                                  # cone surface to tip
    (0.0, CONE_L),                                    # tip face
    (0.0, CAVITY_TOP),                                # axis down to cavity top
    (r_out(CAVITY_TOP) - SHELL, CAVITY_TOP),          # inner cone up begins
    (r_out(BASE_SOLID) - SHELL, BASE_SOLID),          # inner cone at base ring
    (SPIGOT_D / 2 - SPIGOT_WALL, 0.0),                # spigot inner top
    (SPIGOT_D / 2 - SPIGOT_WALL, -SPIGOT_DEPTH),      # spigot inner bottom
]

with BuildPart() as nose_builder:
    with BuildSketch(Plane.XZ) as prof:
        with BuildLine():
            Polyline(*[(r, z) for r, z in pts], close=True)
        make_face()
    revolve(axis=Axis.Z)

nose = nose_builder.part

# Internal baffle ring.
r_baf = r_out(BAFFLE_Z) - SHELL - 0.6
nose += Pos(0, 0, BAFFLE_Z - BAFFLE_T / 2) * (
    Cylinder(r_baf, BAFFLE_T, align=(Align.CENTER, Align.CENTER, Align.MIN)) -
    Pos(0, 0, -1) * Cylinder(BAFFLE_ID / 2, BAFFLE_T + 2,
                             align=(Align.CENTER, Align.CENTER, Align.MIN)))

# Key pins at 0/180 deg.
for i in range(PIN_COUNT):
    a = 180.0 * i
    x = PIN_R * (1 if i == 0 else -1)
    nose += Pos(x, 0, -SPIGOT_DEPTH - PIN_LEN + PIN_LEN / 2) * \
        Cylinder(PIN_D / 2, PIN_LEN)

# Shock cord hole: one through-bore along X across the base annulus.
nose = cut(nose, Pos(0, 0, BASE_SOLID / 2) *
           Rot(0, 90, 0) * Cylinder(CORD_HOLE_D / 2, CONE_R * 2 + 4),
           "shock cord hole")

nose = nose.clean()

# ---------------------------------------------------------------- base plug
plug = Cylinder(PLUG_D / 2, PLUG_T,
                align=(Align.CENTER, Align.CENTER, Align.MIN))

# ---------------------------------------------------------------- print orientation
# Tip down: the cone overhang is a gentle ~5 deg, spigot and pins print
# on top, nothing needs supports.
printed_nose = Rot(180, 0, 0) * nose
printed_nose = Pos(0, 0, -printed_nose.bounding_box().min.Z) * printed_nose

# ---------------------------------------------------------------- outputs
results = {
    "nosecone": printed_nose,
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
