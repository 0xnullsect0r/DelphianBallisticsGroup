#!/usr/bin/env python3
"""1-DOF roll dynamics + controller sim for Rallus II v2, matched to the
firmware loop.

Simulates roll rate through the Rallus II boost/coast profile (custom
propellant modeled as its closest commercial equivalent, the CTI 1115J530;
~4.25 kg, transonic), with the same PI + gain scheduling + deflection taper
+ servo lag the firmware runs, and reports the gains for config.h.

Vehicle numbers are extracted from `openrocket/Rallus Mk.II V2.ork`
(component masses incl. the 0.2 kg avionics bay; flight figures from the
V1 sim run). IXX is a computed estimate: refine after running the V2.ork
sims (marked TODO-ORK2).

Usage:
    python3 roll_sim.py                 # run with current params, print stats
    python3 roll_sim.py --plot          # also show plots (needs matplotlib)
    python3 roll_sim.py --sweep         # sweep Kp/Ki, print a tuning table
"""

import argparse
import math
import random

# ----------------------------------------------------------------------------
# Vehicle / motor params, from Rallus Mk.II V1.ork + its sim data.
# ----------------------------------------------------------------------------
MASS_START = 4.56        # kg, loaded (stability.py component sum)
MASS_END = 4.01          # kg, burnout
BURN_TIME = 2.1          # s (1115 Ns / 530 N avg, CTI stand-in curve)
THRUST_N = 530.0         # N average during burn (custom propellant stand-in)
CD = 0.5                 # backs out ~263 m/s burnout vs ORK sim
AREA = math.pi * 0.0254 ** 2   # m^2, 2 in body
RHO = 1.225              # kg/m^3, sea level
G = 9.81

IXX = 0.0031             # kg m^2 roll inertia (stability.py estimate)

# Canards: 35 mm root / 15.4 mm tip / 28 mm span, 35 deg LE sweep, 1.5 mm
# brass (geometry in V2.ork; sizing rationale from stability.py sweep).
FIN_AREA = 0.000706      # m^2 per canard
FIN_CN_ALPHA = 2.5       # normal force slope per rad, AR 2.2 canard
FIN_ARM = 0.0394         # m, canard CP radius from roll axis
N_FINS_ROLL = 4          # all four canards contribute to roll differentially

MISALIGN_DEG = 0.3       # effective fin cant, disturbance torque scales with Q

# ----------------------------------------------------------------------------
# Controller params, mirror of firmware config.h
# ----------------------------------------------------------------------------
KP = 0.15                # deg per deg/s
KI = 0.02                # deg per deg/s*s
SCHED_VREF = 25.0        # m/s
MAX_DELTA = 15.0         # deg
MAX_RATE = 600.0         # deg/s command slew
AUTH_VMIN, AUTH_VMAX = 15.0, 275.0

# Transonic deflection taper, mirror of DEFLECT_* in config.h.
DEFLECT_FULL_V = 100.0
DEFLECT_MIN_V = 250.0
DEFLECT_MIN_DEG = 3.0

# Servo model: first order lag + rate limit (MG90S ~0.10 s/60 deg).
SERVO_TAU = 0.020
SERVO_RATE = 400.0       # deg/s

DT = 0.001               # 1 kHz, matches the firmware control rate
T_END = 20.0


def thrust(t):
    return THRUST_N if t < BURN_TIME else 0.0


def mass(t):
    frac = min(t / BURN_TIME, 1.0) if BURN_TIME > 0 else 1.0
    return MASS_START + frac * (MASS_END - MASS_START)


def q_dyn(v):
    return 0.5 * RHO * v * v


def delta_cap(v):
    """Deflection ceiling vs airspeed, mirrors control.c delta_cap()."""
    if v <= DEFLECT_FULL_V:
        return MAX_DELTA
    if v >= DEFLECT_MIN_V:
        return DEFLECT_MIN_DEG
    f = (v - DEFLECT_FULL_V) / (DEFLECT_MIN_V - DEFLECT_FULL_V)
    return MAX_DELTA + f * (DEFLECT_MIN_DEG - MAX_DELTA)


def roll_torque(q, delta_deg):
    """Control torque from differential canard deflection, N m."""
    return N_FINS_ROLL * q * FIN_AREA * FIN_CN_ALPHA * math.radians(delta_deg) * FIN_ARM


def disturb_torque(q):
    """Misalignment, scales with Q."""
    return q * FIN_AREA * FIN_CN_ALPHA * math.radians(MISALIGN_DEG) * FIN_ARM


def simulate(kp=KP, ki=KI, plot=False):
    v = 0.0
    alt = 0.0
    omega = 0.0           # deg/s roll rate
    integ = 0.0
    delta_cmd = 0.0       # commanded, deg
    delta_act = 0.0       # servo actual, deg
    prev_cmd = 0.0
    gyro_noise = 0.05     # deg/s rms, ICM-42688 typical

    ts, omegas, deltas, vs = [], [], [], []
    max_omega_ctrl = 0.0
    burnout_omega = 0.0

    n = int(T_END / DT)
    for i in range(n):
        t = i * DT

        # Longitudinal: thrust, gravity, drag. Fixed impulse, no throttle.
        m = mass(t)
        drag = q_dyn(v) * CD * AREA if v > 0 else 0.0
        a = (thrust(t) - drag) / m - (G if alt > 0.5 or t > 0.2 else 0.0)
        v = max(v + a * DT, 0.0)
        alt = max(alt + v * DT, 0.0)

        q = q_dyn(v)
        # Control arms at liftoff detection (~40 ms) and gates on the
        # authority window, same as the firmware state machine.
        authority = (t > 0.04) and (AUTH_VMIN <= v <= AUTH_VMAX)

        # Controller, identical structure to ctrl_step().
        omega_meas = omega + random.gauss(0, gyro_noise)
        if authority:
            e = -omega_meas
            sched = (SCHED_VREF ** 2 / (v * v)) if v > SCHED_VREF else 1.0
            cap = delta_cap(v)
            unsat = kp * sched * e + integ
            if abs(unsat) <= cap:
                integ += ki * sched * e * DT
            delta_cmd = max(-cap, min(cap, kp * sched * e + integ))
        else:
            integ = 0.0
            delta_cmd = 0.0

        # Command slew limit.
        step = MAX_RATE * DT
        delta_cmd = max(prev_cmd - step, min(prev_cmd + step, delta_cmd))
        prev_cmd = delta_cmd

        # Servo: first order lag, then rate limit the movement per step.
        delta_next = delta_act + (delta_cmd - delta_act) * DT / SERVO_TAU
        rate_step = SERVO_RATE * DT
        delta_next = max(delta_act - rate_step, min(delta_act + rate_step, delta_next))
        delta_act = delta_next

        # Roll dynamics.
        net = roll_torque(q, delta_act) - disturb_torque(q)
        omega += math.degrees(net / IXX) * DT

        ts.append(t)
        omegas.append(omega)
        deltas.append(delta_act)
        vs.append(v)

        if t >= 0.1:
            max_omega_ctrl = max(max_omega_ctrl, abs(omega))
        if abs(t - BURN_TIME) < DT / 2:
            burnout_omega = omega

    stats = {
        "max_omega": max(abs(w) for w in omegas),
        "max_omega_after_0.1s": max_omega_ctrl,
        "burnout_omega": burnout_omega,
        "omega_at_5s": omegas[int(5.0 / DT)],
        "omega_at_10s": omegas[int(10.0 / DT)],
        "v_max": max(vs),
        "max_delta": max(abs(d) for d in deltas),
    }
    if plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        ax[0].plot(ts, omegas)
        ax[0].set_ylabel("roll rate deg/s")
        ax[1].plot(ts, deltas)
        ax[1].set_ylabel("deflection deg")
        ax[2].plot(ts, vs)
        ax[2].set_ylabel("velocity m/s")
        ax[2].set_xlabel("t s")
        fig.suptitle("Rallus II roll sim")
        plt.show()
    return stats


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--sweep", action="store_true")
    args = ap.parse_args()

    if args.sweep:
        print(f"{'Kp':>6} {'Ki':>6} {'max|w|':>8} {'w@5s':>8} {'w@10s':>8} {'max d':>7}")
        for kp in (0.05, 0.1, 0.15, 0.2, 0.3, 0.5):
            for ki in (0.01, 0.02, 0.05):
                s = simulate(kp, ki)
                print(f"{kp:6.2f} {ki:6.2f} {s['max_omega_after_0.1s']:8.1f} "
                      f"{s['omega_at_5s']:8.1f} {s['omega_at_10s']:8.1f} {s['max_delta']:7.1f}")
    else:
        random.seed(42)
        s = simulate(plot=args.plot)
        print("Rallus II roll sim (V2.ork geometry, IXX a stability.py estimate)")
        for k, v in s.items():
            print(f"  {k:22s} {v:8.1f}")
