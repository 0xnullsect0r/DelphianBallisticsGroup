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
           BEC zip posts z 98-128 (diagonal pair)
           RFB jack tab z 108-121 (tube hole at station ~199)
  -Y face: MS5611 + W25Q128 trays side-by-side z 2-32
           MAX31855 tray (short) z 33-57
           switch bracket z 102-128 (knob -X, tube slot at station ~197)
  through: battery window z 60-97 (battery strapped across the blade),
           velcro slots z 57.5 / 99, TC wire hooks x +/-16 z 64 / 93

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
BAY_LEN = 132.0            # blade length; forward end = assembly datum
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

# Battery: 2S 450 mAh (VERIFY pack dims before printing)
BAT_X, BAT_Y, BAT_Z = 20.0, 30.0, 35.0
WIN_X = BAT_X + 3.0        # pass-through window in the blade
WIN_Z0, WIN_Z1 = 60.0, 97.0
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
    """Boss on the +Y face, base embedded in the blade, hole fully through."""
    h = EMBED + STANDOFF_H + 1.0
    body = Pos(x, FACE_P - EMBED, z) * Cylinder(
        STANDOFF_D / 2, h, align=(Align.CENTER, Align.MIN, Align.CENTER))
    hole = Pos(x, FACE_P - EMBED - 1.0, z) * Cylinder(
        PCB_HOLE_D / 2, h + 2, align=(Align.CENTER, Align.MIN, Align.CENTER))
    return body - hole


def post(x, z, d, h, hole_d):
    """Generic zip/post boss on the +Y face, embedded base, through hole."""
    body = Pos(x, FACE_P - EMBED, z) * Cylinder(
        d / 2, EMBED + h + 1.0, align=(Align.CENTER, Align.MIN, Align.CENTER))
    hole = Pos(x, FACE_P - EMBED - 1.0, z) * Cylinder(
        hole_d / 2, EMBED + h + 3.0, align=(Align.CENTER, Align.MIN, Align.CENTER))
    return body - hole


def tray_shell(cx, z0, cav_l):
    """Board tray shell on the -Y face, overlapping into the blade."""
    shell_l = cav_l + 2 * TRAY_WALL
    shell_h = TRAY_DEPTH + TRAY_FLOOR + TRAY_WALL
    return Pos(cx, FACE_N - shell_h / 2 + EMBED, z0 + shell_l / 2 - TRAY_WALL) * \
        Box(TRAY_CAV_W + 2 * TRAY_WALL, shell_h, shell_l)


def tray_cavity(cx, z0, cav_l):
    shell_l = cav_l + 2 * TRAY_WALL
    cav = Pos(cx, FACE_N + EMBED - 0.5, z0 + shell_l / 2) * \
        Box(TRAY_CAV_W, TRAY_DEPTH + EMBED + 1, cav_l)
    mouth = Pos(cx, FACE_N - TRAY_DEPTH - 3, z0 + shell_l / 2) * \
        Box(TRAY_CAV_W - 2 * TRAY_LIP, 8, cav_l)
    return cav + mouth


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
blade += tray_shell(0.0, 33.0, TRAY_CAV_L_SHORT)
blade -= tray_cavity(0.0, 33.0, TRAY_CAV_L_SHORT)

# BEC zip posts, +Y face aft, diagonal pair.
for dx, dz in ((-7.0, 102.0), (7.0, 124.0)):
    blade += post(dx, dz, POST_D, POST_H, ZIP_HOLE_D)

# Velcro slots flanking the battery window (clear of all brackets).
for zs in (WIN_Z0 - 2.5, WIN_Z1 + 2.0):
    blade -= Pos(0, 0, zs) * Box(VELCRO_SLOT_L, BLADE_T + 2, VELCRO_SLOT_W)

# TC wire hooks on remaining +Y blade material beside the window.
for hx in (-16.0, 16.0):
    for hz in (WIN_Z0 + 4.0, WIN_Z1 - 4.0):
        blade += post(hx, hz, TC_HOOK_D, 5.0, 3.6)

# Switch bracket, -Y face, knob toward -X through the blade edge. Kept
# inside the tube circle: worst corner ~(15.4, -12.8) -> r 19.9.
SW_Z0 = 102.0
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
JK_Z = 114.5
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
    "tube": Pos(0, 0, -30) * ((Cylinder(25.4, 200, align=(Align.CENTER, Align.CENTER, Align.MIN))) -
                              (Cylinder(23.4, 200, align=(Align.CENTER, Align.CENTER, Align.MIN)))),
    "battery": Pos(0, 0, (WIN_Z0 + WIN_Z1) / 2) * Box(BAT_X - 0.8, BAT_Y - 0.8, BAT_Z - 0.8),
    "pico": Pos(0, FACE_P + STANDOFF_H + 0.8, Z_PICO0 + PICO_L / 2) * Box(PICO_W, 1.6, PICO_L),
}
