#!/usr/bin/env python3
#
#  Analysis of FLL-assisted 3rd-order PLL (doc/update_fll_assisted_pll.md)
#
#  Usage: fll_assisted_pll_ana.py [bw|step|pll|fll|fllnoise|stab|jitter|jerk|all]
#
#  bw      : Table 3  effective noise bandwidth and damping
#  step    : Table 5  acceleration step tolerance
#  pll     : Table 4  max jerk for phase lock (Costas, T = 1 ms)
#  fll     : Table 6  jerk for a given FLL steady-state frequency error
#  fllnoise:          FLL thermal noise (Section 4.2)
#  stab    : Table 8  discrete-time stability limit
#  jitter  : Table 7  Monte Carlo phase jitter without dynamics
#  jerk    : 5.3      Monte Carlo jerk tolerance
#
#  Requires numpy and scipy.
#
import sys, math, random
import numpy as np
from scipy import signal, integrate

# constants --------------------------------------------------------------------
A3, B3, K3 = 1.1, 2.4, 0.7845  # 3rd-order PLL coefficients and Bn = K3 * wp
A2, K2 = 1.414, 0.53           # 2nd-order FLL coefficient and Bn = K2 * wf
LAM_L1 = 299792458.0 / 1575.42e6 # L1 wavelength (m)
G_L1 = 9.80665 / LAM_L1        # 1 g (or 1 g/s) in Hz/s (or Hz/s^2) at L1

# natural frequencies of PLL and FLL --------------------------------------------
def wn(Bp, Bf):
    return Bp / K3, Bf / K2

# denominator of combined closed-loop transfer function (linear model) ---------
def den(Bp, Bf):
    wp, wf = wn(Bp, Bf)
    return [1.0, B3 * wp + A2 * wf, A3 * wp**2 + wf**2, wp**3]

# noise bandwidth of H(s) = num(s) / den(s) (Hz) --------------------------------
def noise_bw(num, d):
    H2 = lambda f: abs(np.polyval(num, 2j * np.pi * f) /
        np.polyval(d, 2j * np.pi * f))**2
    return integrate.quad(H2, 0.0, np.inf, limit=500)[0]

# min damping ratio of complex closed-loop poles ---------------------------------
def min_damping(d):
    z = [-p.real / abs(p) for p in np.roots(d) if abs(p.imag) > 1e-9]
    return min(z) if z else 1.0

# peak of impulse response of 1/den(s) (cyc per Hz/s of accel step) -----------
def step_peak(d):
    t = np.linspace(0.0, 10.0, 40001)
    _, y = signal.impulse(signal.lti([1.0], d), T=t)
    return np.max(np.abs(y))

# PLL thermal noise jitter (deg) --------------------------------------------------
def sigma_pll(Bn, cn0, T):
    c = 10.0**(cn0 / 10.0)
    return math.degrees(math.sqrt(Bn / c * (1.0 + 1.0 / (2.0 * T * c))))

# FLL thermal noise jitter (Hz) ---------------------------------------------------
def sigma_fll(Bn, cn0, T, F=1.0):
    c = 10.0**(cn0 / 10.0)
    return math.sqrt(4.0 * F * Bn / c * (1.0 + 1.0 / (T * c))) / (2.0 * np.pi * T)

# Table 3: effective noise bandwidth and damping ---------------------------------
def table_bw():
    print('Check: Bn(PLL, Bp=5) = %.2f Hz, Bn(FLL, Bf=2) = %.2f Hz' % (
        noise_bw(den(5, 0)[1:], den(5, 0)),
        noise_bw([A2 * wn(0, 2)[1], wn(0, 2)[1]**2],
            [1.0, A2 * wn(0, 2)[1], wn(0, 2)[1]**2])))
    print('| Bp (Hz) | Bf (Hz) | Bn_eff (Hz) | min zeta |')
    for Bp in (5, 10, 15):
        for Bf in (0, 1, 2, 5, 10):
            d = den(Bp, Bf)
            print('| %d | %d | %.2f | %.2f |' % (Bp, Bf, noise_bw(d[1:], d),
                min_damping(d)))
    print('normalized poles of 3rd-order PLL:', np.roots([1.0, B3, A3, 1.0]))

# Table 5: acceleration step tolerance --------------------------------------------
def table_step(theta=0.125):
    print('| Bp (Hz) | Bf (Hz) | k (cyc per Hz/s) | da for %.0f deg (Hz/s) | '
        '(g at L1) |' % (theta * 360))
    for Bp in (5, 10, 15, 20):
        for Bf in (0, 2, 5, 10):
            k = step_peak(den(Bp, Bf))
            print('| %d | %d | %.4f | %.0f | %.2f |' % (Bp, Bf, k, theta / k,
                theta / k / G_L1))

# Table 4: max jerk for phase lock (Costas 45 deg budget) -------------------------
def table_pll(T=0.001, budget=45.0):
    print('| Bp (Hz) | wp | wp^3 | 45 dB-Hz | 40 dB-Hz | 35 dB-Hz |')
    for Bp in (5, 10, 15, 20, 25):
        wp = Bp / K3
        s = '| %d | %.2f | %.0f |' % (Bp, wp, wp**3)
        for cn0 in (45, 40, 35):
            th = max(budget - 3.0 * sigma_pll(Bp, cn0, T), 0.0) / 360.0
            s += ' %.0f (%.1f) |' % (th * wp**3, th * wp**3 / G_L1)
        print(s)

# Table 6: jerk for given FLL steady-state frequency error ------------------------
def table_fll():
    print('| Bf (Hz) | wf | wf^2 | f_e = 50 Hz | f_e = 5 Hz |')
    for Bf in (1, 2, 5, 10, 15):
        wf = Bf / K2
        print('| %d | %.2f | %.1f | %.0f (%.1f) | %.0f (%.2f) |' % (Bf, wf,
            wf**2, 50 * wf**2, 50 * wf**2 / G_L1, 5 * wf**2, 5 * wf**2 / G_L1))

# FLL thermal noise ---------------------------------------------------------------
def table_fllnoise():
    print('| Bf (Hz) | C/N0 | T (s) | sigma_tf (Hz) |')
    for Bf in (1, 2, 5, 10):
        for cn0 in (45, 35, 30):
            for T in (0.001, 0.02):
                print('| %d | %d | %g | %.3f |' % (Bf, cn0, T,
                    sigma_fll(Bf, cn0, T)))

# state transition matrix of implemented discrete loop -------------------------
def loop_matrix(Bp, Bf, T):
    wp, wf = wn(Bp, Bf)
    # state x = [phi_nco, fd, phas_acc, ep_prev], signal = 0
    ep  = np.array([-1.0, -T / 2.0, 0.0, 0.0]) # interval-averaged phase error
    epp = np.array([0.0, 0.0, 0.0, 1.0])
    ef  = (ep - epp) / T
    acc = np.array([0.0, 0.0, 1.0, 0.0]) + (wp**3 * ep + wf**2 * ef) * T
    fd  = (np.array([0.0, 1.0, 0.0, 0.0]) + B3 * wp * (ep - epp) +
        (A3 * wp**2 * ep + A2 * wf * ef) * T + acc * T)
    phi = np.array([1.0, T, 0.0, 0.0])
    return np.array([phi, fd, acc, ep])

# Table 8: discrete-time stability limit ------------------------------------------
def table_stab():
    print('| Bf/Bp | max Bp*T | max Bf*T |')
    for r in (0.0, 0.2, 0.4, 1.0, 2.0):
        for x in np.arange(0.01, 1.0, 0.005):
            if max(abs(np.linalg.eigvals(loop_matrix(x, r * x, 1.0)))) >= 1.0:
                break
        print('| %.1f | %.3f | %s |' % (r, x, '%.3f' % (r * x) if r else '-'))

# Monte Carlo model of implemented loop (returns rms err, slips, max ferr) ----
def sim_loop(Bp, Bf, cn0, T, costas, jerk=0.0, tj=3.0, n=None, seed=1):
    rnd = random.Random(seed)
    n = n or int((2.0 + tj + 3.0) / T)
    wp, wf = wn(Bp, Bf)
    sig = math.sqrt(1.0 / (2.0 * 10.0**(cn0 / 10.0) * T))
    q = 0.5 if costas else 1.0 # phase ambiguity (cyc)
    phi = fd = acc = ep0 = 0.0
    sph = f = a = 0.0
    prev = None
    se, ne, fe = 0.0, 0, 0.0
    for k in range(n):
        t = k * T
        a += (jerk if 2.0 < t < 2.0 + tj else 0.0) * T
        f += a * T
        d = 2.0 * np.pi * ((sph + f * T / 2.0) - (phi + fd * T / 2.0))
        IP = math.cos(d) + rnd.gauss(0.0, sig)
        QP = math.sin(d) + rnd.gauss(0.0, sig)
        sph += f * T
        phi += fd * T
        ep = (math.atan(QP / IP) if costas else math.atan2(QP, IP)) / (2 * np.pi)
        ef = 0.0
        if Bf > 0.0 and prev:
            dot = prev[0] * IP + prev[1] * QP
            cross = prev[0] * QP - prev[1] * IP
            if dot != 0.0:
                ef = (math.atan(cross / dot) if costas else
                    math.atan2(cross, dot)) / (2.0 * np.pi * T)
        acc += (wp**3 * ep + wf**2 * ef) * T
        fd += B3 * wp * (ep - ep0) + (A3 * wp**2 * ep + A2 * wf * ef) * T + \
            acc * T
        ep0 = ep
        prev = (IP, QP)
        fe = max(fe, abs(f - fd))
        if t > 1.0 and jerk == 0.0:
            e = sph - phi
            se += (e - round(e / q) * q)**2
            ne += 1
    rms = math.sqrt(se / ne) * 360.0 if ne else None
    return rms, round(abs(sph - phi) / q) * q, fe

# Table 7: phase jitter without dynamics -------------------------------------------
def table_jitter(Bp=5):
    print('Costas, T = 1 ms (20 s): RMS phase error (deg)')
    print('| C/N0 | Bf = 0 | Bf = 2 | Bf = 5 |')
    for cn0 in (45, 42, 40, 39, 38, 37, 36):
        print('| %d |' % cn0 + ''.join(' %.2f |' % sim_loop(Bp, Bf, cn0, 0.001,
            1, n=20000)[0] for Bf in (0, 2, 5)))
    print('Pilot (atan2), T = 20 ms (200 s): RMS phase error (deg)')
    print('| C/N0 | Bf = 0 | Bf = 2 | Bf = 5 |')
    for cn0 in (40, 35, 30, 27, 25, 23, 21):
        print('| %d |' % cn0 + ''.join(' %.2f |' % sim_loop(Bp, Bf, cn0, 0.02,
            0, n=10000)[0] for Bf in (0, 2, 5)))

# 5.3: jerk tolerance (slips / max freq error) ----------------------------------------
def table_jerk():
    J = (20, 50, 100, 200, 500, 1000, 2000)
    print('Costas, T = 1 ms, 45 dB-Hz: slip (cyc) / max|f_err| (Hz)')
    print('| Bp | Bf |' + ''.join(' %d |' % j for j in J))
    for Bp in (5, 10, 15):
        for Bf in (0, 2, 5, 10):
            print('| %d | %d |' % (Bp, Bf) + ''.join(' %g/%.0f |' %
                sim_loop(Bp, Bf, 45, 0.001, 1, jerk=j)[1:] for j in J))
    print('Pilot (atan2), T = 20 ms, 30 dB-Hz: slip (cyc) / max|f_err| (Hz)')
    print('| Bp | Bf |' + ''.join(' %d |' % j for j in J))
    for Bp in (5, 10):
        for Bf in (0, 2, 5):
            print('| %d | %d |' % (Bp, Bf) + ''.join(' %g/%.0f |' %
                sim_loop(Bp, Bf, 30, 0.02, 0, jerk=j)[1:] for j in J))

# main ------------------------------------------------------------------------------
if __name__ == '__main__':
    funcs = {'bw': table_bw, 'step': table_step, 'pll': table_pll,
        'fll': table_fll, 'fllnoise': table_fllnoise, 'stab': table_stab,
        'jitter': table_jitter, 'jerk': table_jerk}
    args = sys.argv[1:] or ['all']
    for arg in args:
        for name, func in funcs.items():
            if arg in (name, 'all'):
                print('== %s ==' % name)
                func()
