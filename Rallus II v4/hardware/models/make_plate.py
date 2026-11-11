"""Plate all five Rallus v4 printed parts into one 3MF.

Loads the per-part STLs (already in their individual print orientations),
lays them out in one row with gaps, and exports as a single plate plus a
layout PNG. Everything fits a 256x256 Bambu bed.

Run:
    python <skill>/scripts/build.py make_plate.py -o models
"""

from pathlib import Path

from build123d import *
from printlib import *

HERE = Path(__file__).parent
PARTS = ["nose_base", "nose_tip", "nose_base_plug", "avionics_blade", "retention_ring"]
GAP = 8.0

results = {}
x_cursor = 0.0
for name in PARTS:
    shape = import_stl(str(HERE / f"{name}.stl"))
    bb = shape.bounding_box()
    results[name] = Pos(x_cursor - bb.min.X, -bb.min.Y, -bb.min.Z) * shape
    x_cursor += (bb.max.X - bb.min.X) + GAP

# No assembly check: parts are pre-oriented STLs kept 8 mm apart on purpose.
