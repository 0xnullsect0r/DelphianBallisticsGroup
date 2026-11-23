#!/usr/bin/env python3
"""1-D vertical flight estimate for Rallus II on the REAL motor (Motors.ric).

Thrust curve: the .eng exported from openMotor (openmotor_run.py), or the
motor_sim.py estimate when given the .ric; dry mass from the ORK via
stability.py's mass model (or --dry-kg). Reports the numbers the plan and
the firmware thresholds depend on: thrust-to-weight, rail-exit speed, max
speed / Mach / g, burnout and apogee times.

Vertical, no wind, constant CD: good to roughly 10 %. OpenRocket with the
exported .eng is the authority; this is the fast cross-check.

Usage:
    python3 sim/flight_sim.py Motors.ric --dry-kg 4.30
    python3 sim/flight_sim.py Motors.ric --dry-kg 4.30 --rail 3.0 --core-mm 19
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import motor_sim  # noqa: E402

G = 9.80665
AREA = math.pi * 0.0254 ** 2


def rho_air(h):
    return 1.225 * math.exp(-h / 8500.0)


def fly(curve, prop_kg, dry_kg, rail_m=3.0, cd=0.45, site_m=0.0, dt=0.002):
    """dry_kg = everything except propellant. Returns a dict of results."""
    burn = curve[-1][0]
    step = curve[1][0] - curve[0][0]
    total_imp = sum(c[1] for c in curve) * step

    def thrust(t):
        i = int(t / step)
        return curve[i][1] if i < len(curve) else 0.0

    t = v = h = 0.0
    spent = 0.0
    out = {"rail_v": None, "vmax": 0.0, "gmax": 0.0, "g_first": None,
           "g_min_boost": 1e9}
    m0 = dry_kg + prop_kg
    out["twr0"] = thrust(0.05) / (m0 * G)
    out["twr_avg"] = (total_imp / burn) / (m0 * G)
    while True:
        thr = thrust(t)
        spent += thr * dt
        m = dry_kg + prop_kg * max(0.0, 1 - spent / total_imp)
        drag = 0.5 * rho_air(site_m + h) * cd * AREA * v * abs(v)
        a = (thr - drag) / m - G
        if h <= 0 and a < 0 and t < burn:
            a = 0.0                                   # still sitting on the pad
        sensed_g = (thr - drag) / m / G               # what the IMU z axis reads
        if t < burn:
            out["gmax"] = max(out["gmax"], sensed_g)
            if 0.2 < t < burn * 0.85:
                out["g_min_boost"] = min(out["g_min_boost"], sensed_g)
            if out["g_first"] is None and t >= 0.1:
                out["g_first"] = sensed_g
        v += a * dt
        h += v * dt
        t += dt
        if out["rail_v"] is None and h >= rail_m:
            out["rail_v"], out["rail_t"] = v, t
        if v > out["vmax"]:
            out["vmax"], out["vmax_h"] = v, h
        if abs(t - burn) < dt / 2:
            out["burnout_v"], out["burnout_h"] = v, h
        if t > burn and v <= 0:
            break
    out.update(apogee=h, t_apogee=t, burn=burn, m0=m0,
               mach=out["vmax"] / (340.3 - 0.004 * (site_m + out["vmax_h"])))
    return out


def load_eng(path, step=0.002):
    """RASP .eng -> (curve on a fixed time step, propellant kg, name)."""
    rows = [l.split() for l in open(path) if l.strip() and not l.startswith(";")]
    head, pts = rows[0], [(0.0, float(rows[1][1]))] + [(float(a), float(b)) for a, b in rows[1:]]
    curve, t, i = [], 0.0, 0
    while t <= pts[-1][0]:
        while pts[i + 1][0] < t:
            i += 1
        (t0, f0), (t1, f1) = pts[i], pts[i + 1]
        curve.append((t, f0 + (f1 - f0) * (t - t0) / (t1 - t0), 0.0, 0.0))
        t += step
    curve.append((t, 0.0, 0.0, 0.0))
    return curve, float(head[4]), head[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ric", help="thrust curve: a .eng exported by "
                    "openmotor_run.py (preferred), or the .ric itself to use "
                    "the built-in motor_sim.py estimate")
    ap.add_argument("--dry-kg", type=float, required=True,
                    help="liftoff mass minus propellant, kg")
    ap.add_argument("--rail", type=float, default=3.0,
                    help="guided travel, m (rail length minus the distance "
                         "from the rail base to the upper button)")
    ap.add_argument("--cd", type=float, default=0.45)
    ap.add_argument("--site-m", type=float, default=0.0, help="launch site MSL, m")
    ap.add_argument("--core-mm", type=float, help="what-if: BATES core diameter")
    ap.add_argument("--throat-mm", type=float, help="what-if: nozzle throat")
    args = ap.parse_args()

    if args.ric.lower().endswith(".eng"):
        if args.core_mm or args.throat_mm:
            sys.exit("--core-mm / --throat-mm need the .ric (they re-run motor_sim)")
        curve, prop, name = load_eng(args.ric)
        step = curve[1][0] - curve[0][0]
        imp = sum(c[1] for c in curve) * step
        s = {"pc_max": 0.0}
        m = {"p_max": 1.0}
        print(f"motor      {name} (from {os.path.basename(args.ric)})  {imp:.0f} N s, "
              f"{curve[-1][0]:.1f} s burn, peak {max(c[1] for c in curve):.0f} N, "
              f"prop {prop*1000:.0f} g")
    else:
        m = motor_sim.load_ric(args.ric)
        if args.core_mm:
            for g in m["grains"]:
                g["core"] = args.core_mm / 1000
        if args.throat_mm:
            m["throat"] = args.throat_mm / 1000
        curve, prop, _ = motor_sim.simulate(m)
        s = motor_sim.summarize(curve, prop)
        print(f"motor      {s['class']} (motor_sim estimate)  {s['impulse']:.0f} N s, "
              f"{s['burn']:.1f} s burn, peak {s['peak']:.0f} N, "
              f"Pc max {s['pc_max']/1e6:.1f} MPa, prop {prop*1000:.0f} g")
    r = fly(curve, prop, args.dry_kg, args.rail, args.cd, args.site_m)

    print(f"liftoff    {r['m0']:.2f} kg   thrust/weight {r['twr0']:.1f} at ignition, "
          f"{r['twr_avg']:.1f} average")
    print(f"rail exit  {r['rail_v']:.1f} m/s after {args.rail:.1f} m ({r['rail_t']:.2f} s)")
    print(f"boost      IMU reads {r['g_first']:.1f} g at liftoff, max {r['gmax']:.1f} g, "
          f"min {r['g_min_boost']:.1f} g mid-burn")
    print(f"burnout    {r['burnout_v']:.0f} m/s at {r['burnout_h']:.0f} m, t = {r['burn']:.1f} s")
    print(f"max speed  {r['vmax']:.0f} m/s  (Mach {r['mach']:.2f})")
    print(f"apogee     {r['apogee']:.0f} m AGL at t = {r['t_apogee']:.1f} s")
    warn = []
    if r["twr0"] < 5.0:
        warn.append(f"thrust/weight {r['twr0']:.1f} is under the 5:1 rule of thumb")
    if r["rail_v"] < 15.0:
        warn.append(f"rail exit {r['rail_v']:.1f} m/s is under 15 m/s: marginal "
                    "stability off the rail, calm days only or a longer rail")
    if s["pc_max"] > m["p_max"]:
        warn.append("chamber pressure exceeds the limit set in the .ric")
    for w in warn:
        print("WARNING:", w)
    return r


if __name__ == "__main__":
    main()
