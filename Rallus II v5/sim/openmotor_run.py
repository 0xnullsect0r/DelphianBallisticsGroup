#!/usr/bin/env python3
"""Run Motors.ric through openMotor's OWN simulation engine (motorlib) and
write the thrust curve everything else in this repo uses.

This is the authority for the motor. sim/motor_sim.py is only a
dependency-free cross-check (it agrees within ~3 %).

Setup (once):
    git clone https://github.com/reilleya/openMotor ~/openMotor
    pip install numpy scipy scikit-image scikit-fmm pyyaml

Usage:
    python3 sim/openmotor_run.py Motors.ric --openmotor ~/openMotor \
        --eng openrocket/Rallus_EX.eng [--hardware-g 250]

The .eng total mass = propellant + --hardware-g (nozzle, forward closure,
liner). The booster tube is the motor case and is already in the ORK.
"""

import argparse
import os
import sys
import types


def load_motor(ric, om_path):
    import yaml
    sys.path.insert(0, os.path.expanduser(om_path))
    # motorlib imports a compiled helper that only FMM (non-BATES) grains
    # use; stand in for it if openMotor was cloned but not built.
    try:
        import mathlib  # noqa: F401
    except ModuleNotFoundError:
        stub = types.ModuleType("mathlib._find_perimeter_cy")
        stub._get_perimeter = lambda *a, **k: (_ for _ in ()).throw(
            NotImplementedError("build openMotor's cython module for FMM grains"))
        sys.modules["mathlib._find_perimeter_cy"] = stub
        for name in [n for n in sys.modules if n.startswith("mathlib")
                     and n != "mathlib._find_perimeter_cy"]:
            del sys.modules[name]

    class Loader(yaml.SafeLoader):
        pass
    Loader.add_multi_constructor(
        "tag:yaml.org,2002:python/",
        lambda loader, suffix, node: loader.construct_sequence(node)
        if isinstance(node, yaml.SequenceNode) else None)
    data = yaml.load(open(ric), Loader=Loader)["data"]
    import motorlib.motor
    return motorlib.motor.Motor(data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ric")
    ap.add_argument("--openmotor", default="~/openMotor",
                    help="path to an openMotor checkout")
    ap.add_argument("--eng", help="write a RASP .eng thrust curve here")
    ap.add_argument("--hardware-g", type=float, default=250.0)
    args = ap.parse_args()

    import warnings
    warnings.simplefilter("ignore")
    motor = load_motor(args.ric, args.openmotor)
    r = motor.runSimulation()
    if not r.success:
        for a in r.alerts:
            print("ALERT:", a.description)
        sys.exit("openMotor simulation failed")
    ch = r.channels
    t, f = ch["time"].getData(), ch["force"].getData()
    prop = r.getPropellantMass()
    print(f"openMotor result  {r.getFullDesignation()}")
    print(f"total impulse     {r.getImpulse():.0f} N s   Isp {r.getISP():.0f} s")
    print(f"burn time         {r.getBurnTime():.2f} s   avg {r.getAverageForce():.0f} N"
          f"   peak {ch['force'].getMax():.0f} N")
    print(f"chamber pressure  max {r.getMaxPressure()/1e6:.2f} MPa, "
          f"avg {r.getAveragePressure()/1e6:.2f} MPa")
    print(f"Kn                initial {r.getInitialKN():.0f}, peak {r.getPeakKN():.0f}")
    print(f"propellant        {prop*1000:.0f} g, grain stack "
          f"{r.getPropellantLength()*1000:.0f} mm x {r.getMaxPropellantDiameter()*1000:.1f} mm")
    print(f"port/throat       {r.getPortRatio():.2f}   peak mass flux "
          f"{r.getPeakMassFlux():.0f} kg/m2/s   peak port Mach {r.getPeakMachNumber():.2f}")
    for a in r.alerts:
        print("ALERT:", a.description)

    if args.eng:
        des = r.getDesignation()
        step = max(1, len(t) // 30)
        pts = [(t[i], f[i]) for i in range(step, len(t), step)]
        if pts[-1][1] != 0.0:
            pts.append((t[-1] + 0.01, 0.0))
        with open(args.eng, "w") as out:
            out.write("; Rallus II EX motor - thrust curve from openMotor (motorlib)\n")
            out.write(f"; source: {os.path.basename(args.ric)}, {r.getFullDesignation()}\n")
            out.write("; simulated, not static-tested\n")
            out.write(f"{des} {r.getMaxPropellantDiameter()*1000:.0f} "
                      f"{r.getPropellantLength()*1000:.0f} P {prop:.3f} "
                      f"{prop + args.hardware_g/1000:.3f} DBG\n")
            for ti, fi in pts:
                out.write(f"   {ti:.3f} {max(fi, 0.0):.1f}\n")
        print(f"wrote {args.eng}")


if __name__ == "__main__":
    main()
