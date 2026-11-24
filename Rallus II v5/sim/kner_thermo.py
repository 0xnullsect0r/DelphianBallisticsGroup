#!/usr/bin/env python3
"""Chamber equilibrium for KNO3 / erythritol propellant (Cantera + NASA data).

Gives the openMotor propellant inputs: chamber temperature T, effective
molecular weight M (system mass / gas moles, Nakka's definition) and the
mixture ratio of specific heats k (gas + condensed K2CO3, Nakka's k_mix,
the value openMotor's own sugar presets use).

Validated against Nakka / Gudnason for 65/35 KNER at 6.89 MPa:
    this script  1606 K, M 38.57, k_mix 1.1389, c* 924 m/s
    published    1608 K, M 38.78, k_mix 1.1390, c* 923-925 m/s

Erythritol heat of formation: -885.21 kJ/mol (NIST WebBook, from the solid
heat of combustion, Parks & Manchester 1952). NIST's other value (-910.48,
listed as liquid) gives 1571 K and misses the published 65/35 result.

Setup:  pip install cantera numpy
Usage:  python3 sim/kner_thermo.py [OF ...]       e.g. 0.60 0.65
"""
import cantera as ct, numpy as np, warnings, sys
warnings.simplefilter('ignore')
RU = 8.31446
ELEMS = {'C', 'H', 'O', 'N', 'K'}
_gas_sp = [s for s in ct.Species.list_from_file('nasa_gas.yaml')
           if set(s.composition) <= ELEMS and s.charge == 0]
GAS = ct.Solution(thermo='ideal-gas', species=_gas_sp)
COND_NAMES = ['K2CO3(s)', 'K2CO3(L)', 'KOH(a)', 'KOH(b)', 'KOH(L)', 'C(gr)', 'KCN(s)', 'KCN(L)', 'K2O(s)']
_cond_all = {s.name: s for s in ct.Species.list_from_file('nasa_condensed.yaml')}
# Condensed species need a real molar volume: the NASA file has none, and the
# default makes V*dP at 68 atm a ~900 kJ/mol penalty that keeps them out.
DENS = {'K2CO3': 2290.0, 'KOH': 2044.0, 'C(gr)': 2260.0, 'KCN': 1520.0, 'K2O': 2350.0}
def _with_density(sp):
    d = dict(sp.input_data)
    base = sp.name.split('(')[0] if not sp.name.startswith('C(') else 'C(gr)'
    d['equation-of-state'] = {'model': 'constant-volume', 'density': f"{DENS[base]} kg/m^3"}
    return ct.Species.from_dict(d)
CONDS = [ct.Solution(thermo='fixed-stoichiometry', species=[_with_density(_cond_all[n])])
         for n in COND_NAMES if n in _cond_all]

HF_KNO3 = -494.63e3      # J/mol (NIST)
M_KNO3, M_ER = 101.1032, 122.1198

def reactant(of, hf_er):
    """Per kg: element moles and formation enthalpy."""
    n_kn = of * 1000 / M_KNO3
    n_er = (1 - of) * 1000 / M_ER
    el = {'K': n_kn, 'N': n_kn, 'O': 3 * n_kn + 4 * n_er, 'C': 4 * n_er, 'H': 10 * n_er}
    H = n_kn * HF_KNO3 + n_er * hf_er          # J per kg
    return el, H

def mixture_at(T, P, el):
    GAS.TPX = T, P, 'N2:1'
    mix = ct.Mixture([(GAS, 1.0)] + [(c, 0.0) for c in CONDS])
    mix.T, mix.P = T, P
    # seed moles that carry exactly the element amounts
    seed = {'K': el['K'], 'N2': el['N'] / 2, 'CO': el['C'], 'H2': el['H'] / 2}
    o_left = el['O'] - el['C']
    seed['O2'] = o_left / 2
    moles = np.zeros(mix.n_species)
    for sp, n in seed.items():
        moles[mix.species_index(0, sp)] = n
    mix.species_moles = moles
    mix.T, mix.P = T, P
    mix.equilibrate('TP', solver='vcs', max_iter=500, estimate_equil=-1)
    return mix

def chamber(of, P_pa, hf_er):
    el, H = reactant(of, hf_er)
    lo, hi = 800.0, 3000.0
    for _ in range(50):
        T = 0.5 * (lo + hi)
        mix = mixture_at(T, P_pa, el)
        h = sum(mix.phase_moles(i) * ph.enthalpy_mole / 1000 for i, ph in enumerate(mix_phases(mix)))
        if h > H: hi = T
        else: lo = T
    return T, mix, H

def mix_phases(mix):
    return [GAS] + CONDS

def properties(of, P_pa, hf_er):
    T, mix, _ = chamber(of, P_pa, hf_er)
    phases = mix_phases(mix)
    n_gas = mix.phase_moles(0) / 1000          # kmol -> ... cantera Mixture moles are kmol
    n_gas = mix.phase_moles(0)                 # kmol per kg (we seeded mol, treat consistently)
    # moles we seeded were "per kg" in mol; Cantera treats them as kmol. Scale: masses
    total_mass = sum(mix.phase_moles(i) * ph.mean_molecular_weight for i, ph in enumerate(phases))
    gas_mass = mix.phase_moles(0) * GAS.mean_molecular_weight
    M_eff = total_mass / mix.phase_moles(0)                     # kg/kmol, Nakka's definition
    cp_gas = GAS.cp_mass * gas_mass                              # frozen
    cp_cond = sum(mix.phase_moles(i) * ph.cp_mole for i, ph in enumerate(phases) if i > 0)
    cp_tot = (cp_gas + cp_cond) / total_mass                     # J/kg K
    R_eff = 8314.46 / M_eff
    k_2ph = cp_tot / (cp_tot - R_eff)      # gas + condensed: Nakka's 'k_mix'
    k_gas = GAS.cp_mass / GAS.cv_mass
    # Nakka's "mixture" k for c*: gas-only cp/cv
    cond_frac = 1 - gas_mass / total_mass
    def cstar(k):
        return np.sqrt(8314.46 * T / M_eff) / (np.sqrt(k) * (2 / (k + 1)) ** ((k + 1) / (2 * (k - 1))))
    prods = {}
    for i, ph in enumerate(phases):
        if i == 0:
            for j, sp in enumerate(GAS.species_names):
                n = mix.species_moles[mix.species_index(0, sp)]
                if n / total_mass > 1e-4: prods[sp] = n
        elif mix.phase_moles(i) > 1e-6:
            prods[ph.species_names[0]] = mix.phase_moles(i)
    return dict(T=T, M_eff=M_eff, k_gas=k_gas, k_2ph=k_2ph, cond_frac=cond_frac,
                cstar_gas=cstar(k_gas), cstar_2ph=cstar(k_2ph),
                prods={k: v / total_mass * 100 for k, v in prods.items()})  # mol per 100 g

HF_ERYTHRITOL = -885.21e3   # J/mol, NIST WebBook (solid, from heat of combustion)

if __name__ == '__main__':
    ofs = [float(a) for a in sys.argv[1:]] or [0.65, 0.60]
    for of in ofs:
        for p_mpa in (6.89, 4.0, 2.5):
            p = properties(of, p_mpa * 1e6, HF_ERYTHRITOL)
            k = p['k_2ph']
            cs = np.sqrt(8314.46 * p['T'] / p['M_eff']) / (np.sqrt(k) * (2 / (k + 1)) ** ((k + 1) / (2 * (k - 1))))
            print(f"{of*100:.0f}/{100-of*100:.0f} @ {p_mpa:4.2f} MPa: T {p['T']:.0f} K  M {p['M_eff']:.2f}  "
                  f"k_mix {k:.4f}  condensed {p['cond_frac']*100:.1f}%  c* {cs:.0f} m/s")
        print('   products, mol per 100 g:',
              ', '.join(f"{n} {v:.3f}" for n, v in sorted(p['prods'].items(), key=lambda x: -x[1])))
