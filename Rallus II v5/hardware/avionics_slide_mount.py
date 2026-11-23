"""Rallus II v3 — slide-in avionics mount for the 46.8 mm ID upper tube.

A diametral blade carrying every piece of avionics, dropped in radially at
the sled bay, rotated vertical, slid forward until it seats against the
camera cage, then locked by a set-screw retention ring at the aft end.

Print orientation: blade flat (+Y face up, tray lips and bracket edges are
short bridges); ring flat as modeled. Material: PETG minimum, CF-nylon
preferred. All dimensions in mm.

Layout (blade-local z, 0 = forward end = seats on camera cage; tube station
= 84 + z):
  +Y face: Pico 2 on standoffs (USB forward) z 2-53
           BEC zip posts z 120-150 (diagonal pair)
           RFB jack tab z 130-143 (tube hole at station ~220)
  -Y face: MS5611 + W25Q128 trays side-by-side z 0-30
           MAX31855 tray (-X) + ICM-42688 IMU tray (+X), short, z 31-55
           switch bracket z 122-150 (knob -X, tube slot at station ~218.5)
  through: battery window z 60-119, 33 wide: a flat 2S pack lies IN the
           window on the blade plane, two velcro straps round blade + pack
           in the rail notches; TC wire hooks on the rails

Everything on the blade stays inside the tube circle (r 23.4) with margin:
deepest corners are the tray shells at r ~22.8.

Run:
    python <skill>/scripts/build.py avionics_slide_mount.py -o models \
        --section Y=0 --section X=0 --material PETG
"""
from math import radians, cos, sin

from build123d import *
from printlib import *

# ---------------------------------------------------------------- parameters
# Tube (upper section, aluminum)
TUBE_ID = 46.8             # measured; blade and ring fit inside this
TUBE_R = TUBE_ID / 2
FIT = 0.4                  # diametral slip clearance

# Blade
BLADE_W = 44.0             # across, leaves ~1.4 mm radial clearance each side
BLADE_T = 4.0
# rev 2026-11-22: +22 mm. The battery window grew from 37 to 59 mm so a real
# flat 2S pack fits (the old 20x30x35 "pack" does not exist); everything
# aft of the window moved back by AFT_SHIFT.
AFT_SHIFT = 22.0
BAY_LEN = 132.0 + AFT_SHIFT  # blade length; forward end = assembly datum
EMBED = 1.2                # how far appendage bases sink into the blade

# Pico 2 (VERIFY holes against the board)
PICO_L, PICO_W = 51.0, 21.0
PICO_HOLE_PITCH_Z = 47.0   # mounting holes along the board length
PICO_HOLE_INSET = 2.0      # hole row distance from each board end
PICO_HOLE_PITCH_X = 20.3
STANDOFF_H = 6.0
STANDOFF_D = 6.5
PCB_HOLE_D = 2.6           # M2.5 self-tap

# Breakout trays (GY modules vary; VERIFY board sizes)
TRAY_CAV_W = 16.6          # cavity width  (X); two shells + gap fit the chord
TRAY_CAV_L = 26.6          # cavity length (Z), MS5611 / W25Q128
TRAY_CAV_L_SHORT = 20.0    # MAX31855 module is small
TRAY_DEPTH = 6.5           # board sits this deep, open toward -Y
TRAY_LIP = 2.5             # front lip height retaining the board
TRAY_WALL = 1.8
TRAY_FLOOR = 1.2

# Battery: flat 2S pack lying IN the window, centred on the blade plane.
# Envelope covers Zeee 2S 850 mAh (54 x 30 x 15, 46 g, ~3.5 h of Pi + avionics)
# and Gens ace 2S 450 mAh (56 x 31 x 11, 28 g, ~1.8 h). Up to 57 x 32 x 17.
BAT_X, BAT_Y, BAT_Z = 32.0, 17.0, 57.0     # width, thickness, length
WIN_X = BAT_X + 1.0        # pass-through window in the blade
WIN_Z0, WIN_Z1 = 60.0, 60.0 + BAT_Z + 2.0
STRAP_W, STRAP_NOTCH = 14.0, 2.0           # velcro strap notches in the rails
VELCRO_SLOT_L, VELCRO_SLOT_W = 25.0, 3.6

# BEC
BEC_L = 28.0
POST_D, POST_H = 8.0, 10.0
ZIP_HOLE_D = 3.4

# Switch (slide, knob toward -X) and RFB 3.5 mm jack
SWITCH_L, SWITCH_W, SWITCH_H = 21.0, 11.0, 8.0   # body, length along Z
SWITCH_WALL = 2.4
JACK_HOLE_D = 6.2
JACK_TAB_X = 12.0          # how far the jack tab reaches toward -X
TC_HOOK_D = 5.0

# Retention ring
RING_OD = TUBE_ID - FIT
RING_H = 8.0
RING_WALL = 3.4
RING_ID = RING_OD - 2 * RING_WALL
KEY_SLOT_CLEAR = 0.4       # blade slot in the ring, per side
CLAMP_HOLE_D = 2.6         # M3 self-tap, radial
CORD_HOLE_D = 8.0          # shock cord hole through the blade aft end

# derived
FACE_P, FACE_N = BLADE_T / 2, -BLADE_T / 2   # blade faces


# ---------------------------------------------------------------- helpers
def standoff(x, z):
    """Boss growing out of the +Y face, base embedded, hole fully through.

    Axis fix (2026-11-19): the cylinder axis must be Y (out of the face).
    Rot(-90,0,0) maps the cylinder's +Z to +Y; MIN aligns on the height
    slot so it grows away from the face.
    """
    h = EMBED + STANDOFF_H + 1.0
    body = Pos(x, FACE_P - EMBED, z) * Rot(-90, 0, 0) * Cylinder(
        STANDOFF_D / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN))
    hole = Pos(x, FACE_P - EMBED - 1.0, z) * Rot(-90, 0, 0) * Cylinder(
        PCB_HOLE_D / 2, h + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return body - hole


def post(x, z, d, h, hole_d):
    """Generic zip/post boss on the +Y face (axis Y, embedded base)."""
    body = Pos(x, FACE_P - EMBED, z) * Rot(-90, 0, 0) * Cylinder(
        d / 2, EMBED + h + 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    hole = Pos(x, FACE_P - EMBED - 1.0, z) * Rot(-90, 0, 0) * Cylinder(
        hole_d / 2, EMBED + h + 3.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return body - hole


def tray_shell(cx, z0, cav_l):
    """Board tray shell on the -Y face, overlapping into the blade."""
    shell_l = cav_l + 2 * TRAY_WALL
    shell_h = TRAY_DEPTH + TRAY_FLOOR + TRAY_WALL
    return Pos(cx, FACE_N - shell_h / 2 + EMBED, z0 + shell_l / 2 - TRAY_WALL) * \
        Box(TRAY_CAV_W + 2 * TRAY_WALL, shell_h, shell_l)


def tray_cavity(cx, z0, cav_l):
    """Open-top pocket: walls all round, the blade face is the floor.

    rev 2026-11-22: the old cavity was cut 5 mm too far toward +Y. It went
    clean through the blade (severing the Pico standoffs above it) and left
    the -Y side closed, so a board could only go in through the blade from
    under the Pico. Now the pocket stops at the blade face and is open on
    -Y: drop the board in, fix it with VHB tape or a dab of epoxy.
    """
    shell_l = cav_l + 2 * TRAY_WALL
    shell_h = TRAY_DEPTH + TRAY_FLOOR + TRAY_WALL
    # Same z centre as tray_shell, so there is a wall at BOTH ends.
    return Pos(cx, FACE_N - (shell_h + 2) / 2, z0 + shell_l / 2 - TRAY_WALL) * \
        Box(TRAY_CAV_W, shell_h + 2, cav_l)


# ---------------------------------------------------------------- blade
with BuildPart() as blade_builder:
    Box(BLADE_W, BLADE_T, BAY_LEN,
        align=(Align.CENTER, Align.CENTER, Align.MIN))

blade = blade_builder.part

# Battery pass-through window (blade continues above/below as shelf + strip).
blade -= Pos(0, 0, (WIN_Z0 + WIN_Z1) / 2) * Box(WIN_X, BLADE_T + 2, WIN_Z1 - WIN_Z0)

# Pico standoffs, +Y face: hole rows 2 mm from each board end, USB forward.
Z_PICO0 = 2.0
for zx in (Z_PICO0 + PICO_HOLE_INSET,
           Z_PICO0 + PICO_L - PICO_HOLE_INSET):
    for x in (-PICO_HOLE_PITCH_X / 2, +PICO_HOLE_PITCH_X / 2):
        blade += standoff(x, zx)

# Breakout trays, -Y face: two side-by-side forward, short one aft of them.
for cx in (-9.5, +9.5):
    blade += tray_shell(cx, 2.0, TRAY_CAV_L)
    blade -= tray_cavity(cx, 2.0, TRAY_CAV_L)
# rev 2026-11-22: the short row is now TWO trays. -X: MAX31855. +X: the
# ICM-42688 IMU, which had no mount at all. The IMU must not be able to
# move: bed it in its tray with epoxy or VHB tape, component side out, and
# set IMU_UP_AXIS in firmware/include/config.h to the chip axis that points
# at the nose (forward = toward z 0 here).
for cx in (-9.5, +9.5):
    blade += tray_shell(cx, 33.0, TRAY_CAV_L_SHORT)
    blade -= tray_cavity(cx, 33.0, TRAY_CAV_L_SHORT)

# BEC zip posts, +Y face aft, diagonal pair.
for dx, dz in ((-7.0, 102.0 + AFT_SHIFT), (7.0, 124.0 + AFT_SHIFT)):
    blade += post(dx, dz, POST_D, POST_H, ZIP_HOLE_D)

# Velcro slots flanking the battery window (clear of all brackets).
for zs in (WIN_Z0 - 2.5, WIN_Z1 + 2.0):
    blade -= Pos(0, 0, zs) * Box(VELCRO_SLOT_L, BLADE_T + 2, VELCRO_SLOT_W)

# TC wire hooks on remaining +Y blade material beside the window.
# (On the 5.5 mm rails left beside the wider window.)
RAIL_CX = (BLADE_W / 2 + WIN_X / 2) / 2
for hx in (-RAIL_CX, RAIL_CX):
    for hz in (WIN_Z0 + 4.0, WIN_Z1 - 4.0):
        blade += post(hx, hz, TC_HOOK_D, 5.0, 3.0)

# Strap notches: two velcro straps go right round blade + pack; the notches
# keep them inside the tube bore and stop them sliding.
for nz in (WIN_Z0 + 18.0, WIN_Z1 - 18.0):
    for sx in (-1, 1):
        blade -= Pos(sx * BLADE_W / 2, 0, nz) * Box(2 * STRAP_NOTCH, BLADE_T + 2, STRAP_W)

# Switch bracket, -Y face, knob toward -X through the blade edge. Kept
# inside the tube circle: worst corner ~(15.4, -12.8) -> r 19.9.
SW_Z0 = 102.0 + AFT_SHIFT
SW_CX = -8.5
SW_SHELL_W = SWITCH_W + 2 * SWITCH_WALL          # 15.8
SW_SHELL_H = 14.3                                # blade center .. below body
blade += Pos(SW_CX, FACE_N - 12.8 + SW_SHELL_H / 2, SW_Z0 + SWITCH_L / 2) * \
    Box(SW_SHELL_W, SW_SHELL_H, SWITCH_L + 2 * SWITCH_WALL)
# Switch body pocket, open only on the -Y side (stops at the blade face).
blade -= Pos(SW_CX, FACE_N - (SWITCH_H + 0.4) / 2, SW_Z0 + SWITCH_L / 2) * \
    Box(SWITCH_W + 0.6, SWITCH_H + 0.4, SWITCH_L + 0.6)
# Knob/lever clearance through bracket wall + blade edge toward -X.
blade -= Pos(-BLADE_W / 2 - 2.0, -6.0, SW_Z0 + SWITCH_L / 2) * \
    Box(10.0, 14.0, 12.0)

# RFB jack tab, +Y face, bore along -X toward the tube wall.
JK_Z = 114.5 + AFT_SHIFT
jack_tab = Pos(-BLADE_W / 2 + JACK_TAB_X / 2 + 2.0, FACE_P + 3.0, JK_Z) * \
    Box(JACK_TAB_X, 6.0 + EMBED, 14.0)
jack_tab -= Pos(-BLADE_W / 2 - 1.0, FACE_P + 3.0, JK_Z) * Rot(0, 90, 0) * \
    Cylinder(JACK_HOLE_D / 2, JACK_TAB_X + 4)
blade += jack_tab

# Shock cord hole through the blade, aft end (loads into the ring key
# slot). Kept clear of the end face: a tangent hole makes a degenerate
# edge that breaks mesh export.
blade -= Pos(0, 0, BAY_LEN - 6.5) * Rot(90, 0, 0) * Cylinder(
    CORD_HOLE_D / 2, BLADE_T + 2)

blade = blade.clean()

# ---------------------------------------------------------------- retention ring
ring = Cylinder(RING_OD / 2, RING_H,
                align=(Align.CENTER, Align.CENTER, Align.MIN)) - \
       Pos(0, 0, -1) * Cylinder(RING_ID / 2, RING_H + 2,
                                align=(Align.CENTER, Align.CENTER, Align.MIN))

# Key slot for the blade, cut through the ring wall on the forward face.
ring -= Pos(0, 0, -1) * Box(BLADE_W + 2 * KEY_SLOT_CLEAR,
                            BLADE_T + 2 * KEY_SLOT_CLEAR, 12.0)

# Wall-clamp tabs at 0/120/240 deg: thickened sectors whose outer face is
# 0.1 mm inside the tube ID, each with a radial M3 self-tap hole; the screw
# tip passes the tab face and presses the tube wall.
TAB_R_OUT = TUBE_R - 0.1
TAB_TANG = 10.0
TAB_R_IN = RING_ID / 2 - 1.0
tab_trim = Cylinder(TAB_R_OUT, RING_H + 2,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
for ang in (0, 120, 240):
    tab = Rot(0, 0, ang) * Pos((TAB_R_IN + TAB_R_OUT) / 2, 0, RING_H / 2) * \
        Box(TAB_R_OUT - TAB_R_IN, TAB_TANG, RING_H)
    ring += tab & tab_trim          # trim the tab to the circle
    ring = cut(ring, Rot(0, 0, ang) * Pos(TAB_R_OUT - 1.0, 0, RING_H / 2) *
               Rot(0, 90, 0) * Cylinder(CLAMP_HOLE_D / 2,
                                        TAB_R_OUT - TAB_R_IN + 2), f"clamp {ang}")

# Blade clamp screw from the OD into the key slot, at 90 deg.
ring = cut(ring, Rot(0, 0, 90) * Pos(TAB_R_OUT - 1.0, 0, RING_H / 2) *
           Rot(0, 90, 0) * Cylinder(3.2 / 2, RING_WALL + 4), "blade clamp screw")

ring = elephant_foot(ring, 0.3)

# ---------------------------------------------------------------- print orientation
# Blade: lay flat, +Y face up. Ring: flat as modeled.
printed_blade = Rot(-90, 0, 0) * blade
printed_blade = Pos(0, 0, -printed_blade.bounding_box().min.Z) * printed_blade

# ---------------------------------------------------------------- outputs
results = {
    "avionics_blade": printed_blade,
    "retention_ring": ring,
}

# Assembly in the use frame: parts, tube, battery, Pico board.
assembly = {
    "blade": blade,
    "ring": Pos(0, 0, BAY_LEN) * ring,
    "tube": Pos(0, 0, -30) * ((Cylinder(25.4, 220, align=(Align.CENTER, Align.CENTER, Align.MIN))) -
                              (Cylinder(23.4, 220, align=(Align.CENTER, Align.CENTER, Align.MIN)))),
    "battery": Pos(0, 0, (WIN_Z0 + WIN_Z1) / 2) * Box(BAT_X - 0.8, BAT_Y - 0.8, BAT_Z - 0.8),
    "pico": Pos(0, FACE_P + STANDOFF_H + 0.8, Z_PICO0 + PICO_L / 2) * Box(PICO_W, 1.6, PICO_L),
}
