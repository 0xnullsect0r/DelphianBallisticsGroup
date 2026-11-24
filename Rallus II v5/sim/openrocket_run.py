#!/usr/bin/env python3
"""Run the simulations stored in an .ork with OpenRocket itself, no GUI.

OpenRocket is the authority for stability and trajectory; stability.py and
flight_sim.py are quick estimates (stability.py's CP sits ~0.5 cal too far
aft compared with OpenRocket on this airframe).

Needs:  pip install jpype1   and OpenRocket 24.12 installed (jar path below).
The EX motor curve must be where OpenRocket looks for user motors:
    cp openrocket/Rallus_EX_J*.eng ~/.openrocket/ThrustCurves/

Usage:
    python3 sim/openrocket_run.py "openrocket/Rallus Mk.II V5.ork"
"""

import math
import sys

JAR = "/opt/OpenRocket/jar/OpenRocket-24.12.jar"
BODY_D = 0.0508


def main():
    import jpype
    import jpype.imports  # noqa: F401
    jpype.startJVM(jpype.getDefaultJVMPath(), "-Djava.awt.headless=true",
                   classpath=[JAR])
    from info.openrocket.swing.startup import GuiModule
    from info.openrocket.core.plugin import PluginModule
    from info.openrocket.core.startup import Application
    from com.google.inject import Guice
    from java.io import File
    gui, plug = GuiModule(), PluginModule()
    Application.setInjector(Guice.createInjector(gui, plug))
    gui.startLoader()
    from info.openrocket.core.file import GeneralRocketLoader
    from info.openrocket.core.simulation import FlightDataType as T

    doc = GeneralRocketLoader(File(sys.argv[1])).load()
    print(f"{doc.getRocket().getName()}: {doc.getSimulationCount()} simulations")
    for i in range(doc.getSimulationCount()):
        sim = doc.getSimulation(i)
        sim.simulate()
        fd = sim.getSimulatedData()
        b = fd.getBranch(0)
        col = lambda t: list(b.get(t))
        tt, cg, cp, st, thr, m = (col(T.TYPE_TIME), col(T.TYPE_CG_LOCATION),
                                  col(T.TYPE_CP_LOCATION), col(T.TYPE_STABILITY),
                                  col(T.TYPE_THRUST_FORCE), col(T.TYPE_MASS))
        burn = [k for k in range(len(tt)) if thr[k] > 1 and not math.isnan(st[k])]
        print(f"\n== {sim.getName()}")
        print(f"   liftoff mass {m[0]:.2f} kg, peak thrust {max(thr):.0f} N, "
              f"burn {tt[burn[-1]]:.1f} s")
        print(f"   rail exit {fd.getLaunchRodVelocity():.1f} m/s   max {fd.getMaxVelocity():.0f} m/s "
              f"(Mach {fd.getMaxMachNumber():.2f})   max accel {fd.getMaxAcceleration()/9.81:.1f} g")
        print(f"   apogee {fd.getMaxAltitude():.0f} m at {fd.getTimeToApogee():.1f} s   "
              f"deploy at {fd.getDeploymentVelocity():.1f} m/s   ground hit {fd.getGroundHitVelocity():.1f} m/s")
        if burn:
            k2 = min(burn, key=lambda k: abs(tt[k] - 2.0))
            kmin = min(burn, key=lambda k: st[k])
            print(f"   margin: static at liftoff {(cp[k2]-cg[0])/BODY_D:.2f} cal, "
                  f"minimum in flight {st[kmin]:.2f} cal at t={tt[kmin]:.2f} s, "
                  f"burnout {st[burn[-1]]:.2f} cal")
        for w in sim.getSimulatedWarnings():
            print("   warning:", w)


if __name__ == "__main__":
    main()
