#!/usr/bin/env python3
"""Analyze OpenRocket CSV exports: one file per stage branch, two per sim.

Extracts the full flight card per simulation: apogee, max velocity/Mach/
acceleration, off-rail velocity, stability margin through flight (min and
at burnout), deployment and landing velocities, roll inertia, and events.

Usage:
    python3 analyze_flight.py ../something*.csv
    python3 analyze_flight.py FILE [FILE ...] --json card.json
"""

import argparse
import csv
import json
import math
import sys

RAIL_LEN_M = 3.0     # 3 m launch rail
BODY_D_M = 0.0508


def read_export(path):
    """Return (sim_name, columns, rows, events). rows = list of dicts."""
    name = None
    header = None
    rows = []
    events = []

    with open(path, newline="") as f:
        for line in f:
            if line.startswith("#"):
                text = line.lstrip("# ").strip()
                if text.endswith("(Up to date)") or "(Out of date)" in text:
                    name = text.split(" (Up to date)")[0].split(" (Out of date)")[0]
                elif text.startswith("Event "):
                    events.append(text)
                elif text.startswith("Time (s)"):
                    header = [h.strip() for h in line.lstrip("#").strip().split(",")]
                continue
            if not line.strip():
                continue
            if header is None:
                raise ValueError(f"{path}: data before header")
            vals = line.split(",")
            rows.append({header[i]: float(vals[i]) for i in range(len(header))
                         if vals[i].strip() != ""})
    if header is None or not rows:
        raise ValueError(f"{path}: no data")
    return name, rows, events


def col(row, *substrs):
    """First column whose name contains all substrs."""
    for k in row:
        if all(s.lower() in k.lower() for s in substrs):
            return k
    return None


def analyze(path):
    name, rows, events = read_export(path)
    r0 = rows[0]

    def series(*substrs):
        k = col(r0, *substrs)
        if k is None:
            return None, f"missing column {'+'.join(substrs)}"
        return k, [r[k] for r in rows]

    t_k, t = series("time")
    alt_k, alt = series("altitude (m)")          # not "above sea level"
    v_k, v = series("total velocity")
    mach_k, mach = series("mach")
    acc_k, acc = series("total acceleration")
    m_k, mass = series("mass")
    sm_k, sm = series("stability margin")
    cgp_k, _ = series("cg location")
    ixx_k, ixx = series("rotational moment")
    roll_k, roll = series("roll rate")
    thrust_k, thrust = series("thrust")

    # Off-rail: first sample at/above rail tip.
    off_rail_v = None
    off_rail_t = None
    for i, a in enumerate(alt):
        if a >= RAIL_LEN_M:
            off_rail_v = v[i]
            off_rail_t = t[i]
            break

    apogee_i = max(range(len(alt)), key=lambda i: alt[i])
    burnout_i = None
    for i, th in enumerate(thrust):
        if th <= 0.0 and i > 0:
            burnout_i = i
            break

    sm_valid = [x for x in sm if not math.isnan(x)]
    min_sm = min(sm_valid) if sm_valid else float("nan")
    max_sm = max(sm_valid) if sm_valid else float("nan")
    sm_burnout = sm[burnout_i] if burnout_i else None
    mach_burnout = mach[burnout_i] if burnout_i else None
    sm_boost = [x for x in sm[:burnout_i + 1] if not math.isnan(x)] if burnout_i else sm_valid
    sm_min_while_boost = min(sm_boost) if sm_boost else float("nan")

    # Deployment / landing from events + data around them.
    def event_time(prefix):
        for e in events:
            if e.startswith(prefix):
                try:
                    return float(e.split("t=")[1].split()[0])
                except (IndexError, ValueError):
                    return None
        return None

    t_dep = event_time("RECOVERY_DEVICE_DEPLOYMENT")
    t_land = event_time("GROUND_HIT")
    v_dep = None
    v_land = None
    if t_dep is not None:
        for i, tt in enumerate(t):
            if tt >= t_dep:
                v_dep = v[i]
                break
    if t_land is not None:
        for i, tt in enumerate(t):
            if tt >= t_land:
                v_land = v[i]
                break
    if v_land is None and t_land is None:
        # No event line: take end-of-data velocity as landing.
        v_land = v[-1]

    # Branch ends in the air (separated stage still descending at data end)?
    ended_in_air = alt[-1] > 5.0

    card = {
        "file": path.split("/")[-1],
        "sim": name,
        "t_end": t[-1],
        "apogee_m": max(alt),
        "t_apogee": t[apogee_i],
        "max_v": max(v),
        "max_mach": max(mach),
        "max_acc_g": max(acc) / 9.81,
        "off_rail_v": off_rail_v,
        "off_rail_t": off_rail_t,
        "burnout_t": t[burnout_i] if burnout_i else None,
        "v_burnout": v[burnout_i] if burnout_i else None,
        "mach_burnout": mach_burnout,
        "sm_min": min_sm,
        "sm_max": max_sm,
        "sm_min_boost": sm_min_while_boost,
        "sm_burnout": sm_burnout,
        "mass0_g": mass[0],
        "mass_burnout_g": mass[burnout_i] if burnout_i else None,
        "ixx_burnout_kgm2": ixx[burnout_i] if burnout_i else None,
        "max_roll_rate": max(abs(x) for x in roll),
        "v_deploy": v_dep,
        "v_landing": v_land,
        "alt_end": alt[-1],
        "ended_in_air": ended_in_air,
        "events": events,
    }
    return card


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="+")
    ap.add_argument("--json", help="also write the cards to this file")
    args = ap.parse_args()

    cards = [analyze(p) for p in args.csvs]

    hdr = (f"{'file':14s} {'apog m':>7s} {'tAp s':>6s} {'maxV':>6s} {'Mach':>5s} "
           f"{'acc g':>6s} {'railV':>6s} {'smMin':>6s} {'smB/O':>6s} "
           f"{'M@B/O':>5s} {'Ixx':>7s} {'vDep':>5s} {'vLand':>6s}")
    # Ixx column: rotational (roll) moment of inertia at burnout, kg m^2
    print(hdr)
    print("-" * len(hdr))
    for c in cards:
        print(f"{c['file']:14s} {c['apogee_m']:7.0f} {c['t_apogee']:6.1f} "
              f"{c['max_v']:6.1f} {c['max_mach']:5.2f} {c['max_acc_g']:6.1f} "
              f"{(c['off_rail_v'] or 0):6.1f} {c['sm_min']:6.2f} "
              f"{(c['sm_burnout'] or 0):6.2f} {(c['mach_burnout'] or 0):5.2f} "
              f"{(c['ixx_burnout_kgm2'] or 0):7.4f} "
              f"{(c['v_deploy'] or 0):5.1f} {(c['v_landing'] or 0):6.1f}")

    print()
    for c in cards:
        print(f"{c['file']}: mass0 {c['mass0_g']:.0f} g, "
              f"roll max {c['max_roll_rate']:.1f} deg/s, "
              f"ended_in_air={c['ended_in_air']}")
        for e in c["events"]:
            print(f"    {e}")

    if args.json:
        with open(args.json, "w") as f:
            json.dump(cards, f, indent=1)
        print(f"\nwrote {args.json}")


if __name__ == "__main__":
    main()
