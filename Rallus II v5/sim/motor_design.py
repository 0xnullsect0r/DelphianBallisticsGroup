#!/usr/bin/env python3
"""Rallus II EX motor design on 60/40 KNO3/erythritol, using openMotor's own
engine (motorlib).

Propellant inputs
  T, M, k    sim/kner_thermo.py for 60/40 at 4 MPa: 1349 K, 33.97, 1.1585.
             T is entered at 95 % of ideal, the convention of openMotor's
             own Nakka KNSB/KNDX presets (combustion efficiency).
  density    1700 kg/m^3 = 95 % of the 1785 ideal for 60/40 (hand cast).
  burn rate  Gudnason 2010 strand-burner data for 65/35 KNER, fitted:
             r = 2.782 mm/s * P[MPa]^0.416 (21 points, 4.6 % rms scatter).
             No 60/40 data exists, so it is bracketed:
               slow 0.60x, nominal 0.80x, fast 1.10x of the 65/35 law.
             Measure it (strand burns) and use the throat table below.

Design limits: fast-case peak pressure <= 8 MPa, mass flux <= 1406
kg/m^2/s (the file's erosive limit), port/throat >= 2, nominal thrust/weight
>= 5 on the 5.16 kg liftoff mass, no openMotor alerts in any case.

Usage:
    python3 sim/motor_design.py --openmotor ~/openMotor            # cases + throat table
    python3 sim/motor_design.py --openmotor ~/openMotor --eng-dir openrocket
"""

import argparse
import math
import os
import sys

A65 = 2.782e-3 / (1e6) ** 0.416     # m/s/Pa^n, Gudnason 65/35 KNER fit
N65 = 0.416
CASES = {"slow": 0.60, "nominal": 0.80, "fast": 1.10}
PROP = dict(density=1700.0, t=0.95 * 1349.0, m=33.97, k=1.1585)
DRY_KG = 4.286          # OpenRocket liftoff mass minus propellant (incl. 0.25 kg motor hardware)
HARDWARE_KG = 0.25

# The design
N_GRAINS, GRAIN_OD, STACK = 6, 42.8, 457.2      # mm
CORE, THROAT, EXIT = 20.0, 9.5, 23.0            # mm


def motor_dict(afac, n=N_GRAINS, core=CORE, throat=THROAT, exit_d=EXIT, max_p=8.0e6):
    length = STACK / n / 1000
    grains = [{"type": "BATES", "properties": {
        "diameter": GRAIN_OD / 1000, "length": length, "coreDiameter": core / 1000,
        "inhibitedEnds": "Top" if i == 0 else "Neither"}} for i in range(n)]
    return {
        "config": {"ambPressure": 101325.0, "burnoutThrustThres": 0.1, "burnoutWebThres": 2.54e-05,
                   "flowSeparationWarnPercent": 0.05, "mapDim": 750, "maxMachNumber": 0.7,
                   "maxMassFlux": 1406.4697609001405, "maxPressure": max_p, "minPortThroat": 2.0,
                   "sepPressureRatio": 0.4, "timestep": 0.01},
        "grains": grains,
        "nozzle": {"convAngle": 45.0, "divAngle": 15.0, "efficiency": 0.88, "erosionCoeff": 0.0,
                   "exit": exit_d / 1000, "slagCoeff": 0.0, "throat": throat / 1000,
                   "throatLength": 0.0031750064},
        "propellant": {"name": "KNER 60/40 (est.)", "density": PROP["density"],
                       "tabs": [{"a": A65 * afac, "n": N65, "k": PROP["k"], "m": PROP["m"],
                                 "t": PROP["t"], "minPressure": 0.0, "maxPressure": 10342500.0}]},
    }


def simulate(om_path, d):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from openmotor_run import load_motor  # sets up motorlib (and its stub) on the path
    import yaml
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".ric", delete=False) as f:
        yaml.safe_dump({"data": d}, f)
        path = f.name
    try:
        r = load_motor(path, om_path).runSimulation()
    finally:
        os.unlink(path)
    ch = r.channels
    m0 = DRY_KG + r.getPropellantMass()
    return r, dict(des=r.getDesignation(), I=r.getImpulse(), burn=r.getBurnTime(),
                   F0=ch["force"].getData()[1], Fpk=ch["force"].getMax(),
                   tw=ch["force"].getData()[1] / (m0 * 9.80665),
                   Pmax=r.getMaxPressure() / 1e6, G=r.getPeakMassFlux(),
                   prop=r.getPropellantMass(), alerts=[a.description for a in r.alerts])


def write_eng(path, r, s, tag):
    t, F = r.channels["time"].getData(), r.channels["force"].getData()
    step = max(1, len(t) // 32)
    pts = [(t[i], F[i]) for i in range(step, len(t), step)]
    if pts[-1][1] != 0:
        pts.append((t[-1] + 0.01, 0.0))
    with open(path, "w") as o:
        o.write(f"; Rallus II EX motor: KNO3/erythritol 60/40, {N_GRAINS}x BATES {GRAIN_OD}x"
                f"{STACK/N_GRAINS:.1f} mm, {CORE:.0f} mm core, {THROAT} mm throat\n")
        o.write(f"; openMotor (motorlib), {tag} burn-rate case ({CASES[tag]:.2f}x Gudnason 65/35 KNER)\n")
        o.write("; ESTIMATE - replace with static-test data\n")
        o.write(f"{s['des']} 43 457 P {s['prop']:.3f} {s['prop'] + HARDWARE_KG:.3f} DBG\n")
        for a, b in pts:
            o.write(f"   {a:.3f} {max(b, 0):.1f}\n")


def main():
    import warnings
    warnings.simplefilter("ignore")
    ap = argparse.ArgumentParser()
    ap.add_argument("--openmotor", default="~/openMotor")
    ap.add_argument("--eng-dir", help="write slow/nominal/fast .eng files here")
    args = ap.parse_args()
    om = os.path.expanduser(args.openmotor)

    print(f"Design: {N_GRAINS} x BATES {GRAIN_OD} x {STACK/N_GRAINS:.1f} mm, core {CORE} mm, "
          f"throat {THROAT} mm, exit {EXIT} mm, 60/40 KNER at {PROP['density']:.0f} kg/m^3\n")
    for tag, f in CASES.items():
        r, s = simulate(om, motor_dict(f))
        print(f"  {tag:8s} {s['des']:6s} {s['I']:5.0f} N s  burn {s['burn']:.2f} s  "
              f"T/W {s['tw']:.1f}  peak {s['Fpk']:.0f} N  Pc max {s['Pmax']:.2f} MPa  "
              f"mass flux {s['G']:.0f}  prop {s['prop']*1000:.0f} g  alerts: {'; '.join(s['alerts']) or '-'}")
        if args.eng_dir:
            p = os.path.join(args.eng_dir, f"Rallus_EX_{s['des']}_{tag}.eng")
            write_eng(p, r, s, tag)
            print(f"           wrote {p}")

    print("\nAfter strand tests: measured rate at 3.5 MPa -> throat/exit that keep the "
          "same ~3.8 MPa design pressure")
    for f in (0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2):
        th, ex = THROAT * math.sqrt(f / 0.8), EXIT * math.sqrt(f / 0.8)
        _, s = simulate(om, motor_dict(f, throat=th, exit_d=ex))
        print(f"  {A65*f*(3.5e6)**N65*1000:4.2f} mm/s ({f:.1f}x)  throat {th:5.2f} mm  exit {ex:4.1f} mm  "
              f"-> {s['des']:6s} T/W {s['tw']:.1f}  Pc max {s['Pmax']:.2f} MPa  "
              f"alerts: {'; '.join(s['alerts']) or '-'}")


if __name__ == "__main__":
    main()
