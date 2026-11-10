#!/usr/bin/env python3
"""Static stability + canard authority analysis, straight from the .ork.

Parses the OpenRocket file (zipped XML), rebuilds the component tree with
absolute x positions, computes:

  - component masses and CG (loaded + burnout), with motor mass sensitivity
  - Barrowman CP, static margin in calibers, transonic margin estimate
  - canard roll authority vs booster-fin misalignment torque (A/D ratio),
    sweepable over canard size so the geometry gets picked before anyone
    cuts brass

Approximations vs a real OpenRocket run: classic low-AR Barrowman fin CNalpha
with body interference, fin CP at planform centroid, no body-lift terms,
compressibility as a ~25% fin effectiveness penalty at M0.78. Expect agreement
within roughly half a caliber of OpenRocket; the final gate is still opening
the .ork and running its sims.

Usage:
    python3 stability.py "openrocket/Rallus Mk.II V2.ork"
    python3 stability.py FILE --motor-mass 1.5 --prop-mass 0.60
    python3 stability.py FILE --canard-sweep
    python3 stability.py FILE --canard 35,18,24   # root mm, tip mm, span mm
"""

import argparse
import io
import math
import zipfile
import xml.etree.ElementTree as ET

G = 9.81


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def read_ork_xml(path):
    with zipfile.ZipFile(path) as z:
        return ET.parse(io.BytesIO(z.read("rocket.ork")))


def fnum(el, tag, default=0.0):
    t = el.findtext(tag)
    return float(t) if t not in (None, "auto", "") else default


def radius_of(el, tag):
    t = el.findtext(tag) or "0"
    return float(t.replace("auto ", ""))


def mat_density(el):
    m = el.find("material")
    return float(m.get("density", "0")) if m is not None else 0.0


def poly_area_centroid(points):
    """Shoelace area and centroid, meters."""
    a = cx = cy = 0.0
    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        cross = x1 * y2 - x2 * y1
        a += cross
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    a *= 0.5
    if abs(a) < 1e-12:
        return 0.0, (0.0, 0.0)
    return abs(a), (cx / (6 * a), cy / (6 * a))


class Part:
    def __init__(self, name, kind, mass, cg, aero=None, span=0.0):
        self.name, self.kind = name, kind
        self.mass, self.cg = mass, cg
        self.aero = aero or {}
        self.span = span


def fin_aero(points, n_fins, body_r, x_origin, s_ref):
    """Barrowman fin set: CNalpha per rad referenced to S_ref, plus geometry.

    Fin points x grows toward the nose from the finset origin (verified
    against both fin sets in this design).
    """
    area, (cx, cy) = poly_area_centroid(points)
    span = max(p[1] for p in points)
    if span < 1e-6 or area < 1e-9:
        return None
    ar = 2.0 * span ** 2 / area
    xs_root = sorted(p[0] for p in points if p[1] < 1e-9)
    xs_tip = sorted(p[0] for p in points if p[1] > span - 1e-9)
    mid_root = (xs_root[0] + xs_root[-1]) / 2
    mid_tip = (xs_tip[0] + xs_tip[-1]) / 2
    tan_sweep = abs(mid_tip - mid_root) / span
    denom = 2.0 + math.sqrt(4.0 + ar ** 2 * (1.0 + tan_sweep ** 2))
    cn_fin = 2.0 * math.pi * ar / denom
    tau = 1.0 + body_r / (body_r + span)
    return {
        "cn": n_fins * cn_fin * (area / s_ref) * tau,
        "cn_per_fin": cn_fin * (area / s_ref) * tau,
        "x_cp": x_origin - cx,
        "area": area,
        "span": span,
        "ar": ar,
        "n": n_fins,
    }


# ---------------------------------------------------------------------------
# parser
# ---------------------------------------------------------------------------
def parse_rocket(path, motor_mass, canard_override=None, booster_override=None):
    root = read_ork_xml(path).getroot()
    rocket = root.find("rocket")

    # Reference radius: max body radius in the file.
    ref_r = 0.0
    for el in root.iter():
        if el.tag in ("bodytube", "nosecone", "transition"):
            for t in ("radius", "aftradius", "foreradius"):
                v = el.findtext(t)
                if v:
                    ref_r = max(ref_r, float(v.replace("auto ", "")))
    s_ref = math.pi * ref_r ** 2

    parts = []
    x = 0.0  # absolute position of the current stacking point (nose tip = 0)

    for stage in rocket.findall("./subcomponents/stage"):
        for comp in stage.findall("./subcomponents/*"):
            tag, name = comp.tag, comp.findtext("name", "?")
            length = fnum(comp, "length")
            rho = mat_density(comp)

            if tag == "nosecone":
                r_out = radius_of(comp, "aftradius")
                om = comp.findtext("overridemass")
                m = float(om) if om else (1 / 3) * math.pi * r_out ** 2 * length * rho
                cp = x + length * (2 / 3)
                parts.append(Part(name, "nose", m, x + length * 0.6,
                                  {"cn": 2.0, "x_cp": cp}))
                x += length

            elif tag in ("bodytube", "transition"):
                r_out = radius_of(comp, "aftradius") if tag == "transition" \
                    else radius_of(comp, "radius")
                thick = fnum(comp, "thickness")
                r_in = max(r_out - thick, 0.0)
                m = math.pi * (r_out ** 2 - r_in ** 2) * length * rho
                tube_top = x
                parts.append(Part(name, "bodytube", m, x + length / 2))
                if tag == "transition":
                    fore = radius_of(comp, "foreradius")
                    if abs(r_out - fore) > 1e-6:
                        cn = 2.0 * (r_out ** 2 - fore ** 2) / ref_r ** 2
                        parts[-1].aero = {"cn": cn, "x_cp": x + length / 2}
                x += length
                tube_bottom = x

                for sub in comp.findall("./subcomponents/*"):
                    parse_sub(sub, tube_top, tube_bottom, r_out, r_in,
                              ref_r, s_ref, parts, canard_override,
                              booster_override)

            else:
                # Fins/masses directly under a stage (shouldn't happen here).
                pass

    # Motor: flush with the aft end of the airframe, 0.5 m long.
    tail = x
    motor_len = 0.5
    parts.append(Part("motor (loaded)", "motor", motor_mass,
                      tail - motor_len / 2))
    return parts, ref_r


def parse_sub(sub, tube_top, tube_bottom, r_out, r_in, ref_r, s_ref,
              parts, canard_override=None, booster_override=None):
    tag, name = sub.tag, sub.findtext("name", "?")
    off_el = sub.find("axialoffset")
    method = off_el.get("method", "top") if off_el is not None else "top"
    off = fnum(sub, "axialoffset")
    if method == "top":
        x0 = tube_top + off
    elif method == "bottom":
        x0 = tube_bottom + off          # negative off = up from bottom edge
    else:
        x0 = (tube_top + tube_bottom) / 2 + off

    if tag in ("freeformfinset", "trapezoidfinset"):
        n = int(sub.findtext("fincount", "3"))
        rho = mat_density(sub)
        thick = fnum(sub, "thickness")
        if tag == "trapezoidfinset":
            pts = [(0, 0), (0, fnum(sub, "height")),
                   (fnum(sub, "tipchord"), fnum(sub, "height")),
                   (fnum(sub, "rootchord"), 0)]
        else:
            pts = [(float(p.get("x")), float(p.get("y")))
                   for p in sub.findall("./finpoints/point")]
        if canard_override and "canard" in name.lower():
            root, tip, span = canard_override
            sweep = span * math.tan(math.radians(35))
            pts = [(0, 0), (sweep, span), (root, span), (root, 0)]
        elif booster_override and tag == "freeformfinset" \
                and "canard" not in name.lower():
            # Keep the V1 planform family: root chord fixed, LE sweep angle
            # held at the V1 ratio (65.8 mm over 50 mm), span scaled.
            root, span = booster_override
            sweep = span * (0.0658 / 0.050)
            pts = [(0, 0), (sweep, span), (root, span), (root, 0)]
        area, (cx, cy) = poly_area_centroid(pts)
        m = n * area * thick * rho
        a = fin_aero(pts, n, r_out, x0, s_ref)
        parts.append(Part(name, "finset", m, x0 - cx, a, span=a["span"]))
        return

    if tag in ("masscomponent", "shockcord"):
        parts.append(Part(name, "mass", fnum(sub, "mass"), x0))
        return

    if tag == "parachute":
        dia = fnum(sub, "diameter")
        rho_cloth = mat_density(sub)
        m = math.pi / 4 * dia ** 2 * rho_cloth + 0.03
        parts.append(Part(name, "chute", m, x0))
        return

    if tag == "bulkhead":
        m = math.pi * r_in ** 2 * fnum(sub, "length") * mat_density(sub)
        parts.append(Part(name, "mass", m, x0))
        return

    if tag == "railbutton":
        om = sub.findtext("overridemass")
        parts.append(Part(name, "mass", float(om) if om else 0.003, x0))
        return

    # motormount carries no mass here; the motor is added by the caller.


# ---------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------
def totals(parts):
    cn = sum(p.aero.get("cn", 0.0) for p in parts)
    cp = sum(p.aero["cn"] * p.aero["x_cp"] for p in parts if p.aero) / cn
    return cn, cp


def cg(parts, prop_mass, burnout=False):
    live = []
    for p in parts:
        if p.kind == "motor":
            live.append((p.mass - (prop_mass if burnout else 0.0), p.cg))
        else:
            live.append((p.mass, p.cg))
    m = sum(w for w, _ in live)
    return m, sum(w * x for w, x in live) / m


def estimate_ixx(parts, ref_r):
    ix = 0.0
    for p in parts:
        if p.kind == "finset":
            ix += p.mass * (ref_r + 0.6 * p.span) ** 2
        elif p.kind == "nosecone":
            ix += p.mass * (ref_r / 2) ** 2
        elif p.kind == "bodytube":
            ix += p.mass * (0.9 * ref_r) ** 2
        else:
            ix += p.mass * (0.7 * ref_r) ** 2
    return ix


def canard_set(parts):
    cs = [p for p in parts if p.kind == "finset" and "canard" in p.name.lower()]
    return cs[0] if cs else None


def booster_set(parts):
    bs = [p for p in parts if p.kind == "finset" and "canard" not in p.name.lower()]
    return bs[0] if bs else None


def report(parts, ref_r, prop_mass, label):
    body_d = 2 * ref_r
    cn, cp = totals(parts)
    m0, cg0 = cg(parts, prop_mass)
    mb, cgb = cg(parts, prop_mass, burnout=True)
    ixx = estimate_ixx(parts, ref_r)

    # Transonic: fins lose ~25% effectiveness at M0.78, nose unchanged.
    fin_cn = sum(p.aero["cn"] for p in parts if p.kind == "finset")
    fin_cpx = sum(p.aero["cn"] * p.aero["x_cp"] for p in parts
                  if p.kind == "finset") / fin_cn
    nose_cn = sum(p.aero["cn"] for p in parts if p.kind == "nose")
    nose_cpx = sum(p.aero["cn"] * p.aero["x_cp"] for p in parts
                   if p.kind == "nosecone") / nose_cn
    cp_t = (nose_cn * nose_cpx + 0.75 * fin_cn * fin_cpx) / \
           (nose_cn + 0.75 * fin_cn)

    print(f"\n=== {label} ===")
    print(f"{'component':26s} {'mass g':>8s} {'CG m':>6s} {'CN/rad':>7s} {'CP m':>6s}")
    for p in sorted(parts, key=lambda q: q.cg):
        a = f"{p.aero['cn']:7.2f} {p.aero['x_cp']:6.3f}" if p.aero else ""
        print(f"{p.name:26s} {p.mass*1000:8.1f} {p.cg:6.3f} {a}")

    print(f"\nloaded {m0:6.3f} kg CG {cg0:.3f} m | burnout {mb:6.3f} kg CG {cgb:.3f} m")
    print(f"CNalpha {cn:6.2f}/rad  CP {cp:.3f} m")
    print(f"margin: {((cp-cg0)/body_d):5.2f} cal loaded, "
          f"{((cp-cgb)/body_d):5.2f} cal burnout, "
          f"~{((cp_t-cg0)/body_d):5.2f} cal at M0.78 (est)")
    print(f"Ixx estimate {ixx*1000:.1f} g m^2")

    c, b = canard_set(parts), booster_set(parts)
    if c and b:
        r_c = ref_r + c.span / 2
        r_b = ref_r + (2 / 3) * b.span
        print(f"\ncanards  {c.aero['area']*1e4:5.2f} cm2 x{c.aero['n']}  "
              f"AR {c.aero['ar']:.2f}  CNalpha {c.aero['cn']:5.2f}/rad")
        print(f"booster  {b.aero['area']*1e4:5.1f} cm2 x{b.aero['n']}  "
              f"AR {b.aero['ar']:.2f}  CNalpha {b.aero['cn']:5.2f}/rad")
        for eps in (0.2, 0.5):
            dist = b.aero["cn_per_fin"] * math.radians(eps) * r_b
            row = []
            for delta in (15.0, 5.0, 3.0):
                auth = c.aero["cn"] * math.radians(delta) * r_c
                row.append(f"{delta:3.0f}d:{auth/dist:5.2f}")
            print(f"  A/D vs {eps:.1f} deg cant   " + "  ".join(row))
        s_ref = math.pi * ref_r ** 2
        for v, rho in ((35.0, 1.225), (100.0, 1.17), (263.0, 1.03)):
            q = 0.5 * rho * v * v
            delta = 15.0 if v < 100 else 3.0
            auth = c.aero["cn"] * math.radians(delta) * r_c * q * s_ref
            print(f"  v={v:5.0f} m/s  q={q/1000:5.1f} kPa  "
                  f"roll accel at {delta:.0f} deg: {math.degrees(auth/ixx):6.0f} deg/s^2")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ork")
    ap.add_argument("--motor-mass", type=float, default=1.30)
    ap.add_argument("--prop-mass", type=float, default=0.55)
    ap.add_argument("--canard", help="root,tip,span mm, e.g. 35,18,24")
    ap.add_argument("--booster", help="root,span mm, e.g. 141,60")
    ap.add_argument("--canard-sweep", action="store_true")
    args = ap.parse_args()

    override = None
    if args.canard:
        r, t, s = (float(v) / 1000 for v in args.canard.split(","))
        override = (r, t, s)
    booster = None
    if args.booster:
        r, s = (float(v) / 1000 for v in args.booster.split(","))
        booster = (r, s)

    parts, ref_r = parse_rocket(args.ork, args.motor_mass, override, booster)
    label = args.ork.split("/")[-1]
    if override:
        label += f" + canards {override[0]*1000:.0f}x{override[1]*1000:.0f}x{override[2]*1000:.0f} mm"
    if booster:
        label += f" + booster fins {booster[0]*1000:.0f}x{booster[1]*1000:.0f} mm"
    report(parts, ref_r, args.prop_mass, label)

    if args.canard_sweep:
        print("\n=== canard sweep (root x span mm; margin at burnout CG) ===")
        print(f"{'geom':>12s} {'area cm2':>8s} {'margin':>7s} {'A/D .5d15':>10s} {'A/D .5d3':>9s}")
        for root in (20, 25, 30, 35, 40, 45):
            for span in (12, 16, 20, 24, 28, 32):
                ovr = (root / 1000, root * 0.5 / 1000, span / 1000)
                ps, rr = parse_rocket(args.ork, args.motor_mass, ovr)
                cn, cp = totals(ps)
                _, cgb = cg(ps, args.prop_mass, burnout=True)
                cal = (cp - cgb) / (2 * rr)
                c, b = canard_set(ps), booster_set(ps)
                r_c = rr + c.span / 2
                r_b = rr + (2 / 3) * b.span
                dist = b.aero["cn_per_fin"] * math.radians(0.5) * r_b
                auth15 = c.aero["cn"] * math.radians(15) * r_c
                print(f"{root:5.0f}x{span:4.0f} {c.aero['area']*1e4:8.1f} "
                      f"{cal:7.2f} {auth15/dist:10.2f} {auth15/dist*0.2:9.2f}")


if __name__ == "__main__":
    main()
