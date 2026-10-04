# FLL-Assisted 3rd-Order PLL for High Dynamics

Date: 2026-10-04

Status: Implemented in `src/sdr_ch.c` v1.27. Verified by linear analysis and
Monte Carlo simulation of the loop equations. Recorded-IF and real dynamic
(vehicle, aircraft, rocket) validation remains pending.

----

## 1. Objective

The original carrier tracking loop of Pocket SDR is a pure 3rd-order PLL. Under
high dynamics (large acceleration steps or jerk) the PLL phase error exceeds the
discriminator range, the loop slips cycles and, because a PLL carries no
frequency information once it has slipped, the frequency estimate runs away and
the channel is lost.

The change adds a 2nd-order FLL as an assist to the 3rd-order PLL (the
"FLL-assisted PLL" of Ward). The FLL keeps the loop frequency-locked when the
PLL temporarily loses phase lock, so phase lock is recovered as soon as the
dynamics relax, instead of losing the channel.

## 2. Original 3rd-order PLL

### 2.1 Code (v1.26)

```c
// PLL (3rd-order, a3=1.1, b3=2.4, Bn=W/0.7845)
double err_phas = (costas ? atan(QP / IP) : atan2(QP, IP)) / DPI;
double W = sdr_b_pll / 0.7845;
ch->trk->phas_acc += W * W * W * err_phas * dt;
ch->fd += 2.4 * W * (err_phas - ch->trk->err_phas) +
    1.1 * W * W * err_phas * dt + ch->trk->phas_acc * dt;
```

- `err_phas` (ep): phase discriminator output (cycles). Costas (`atan`) has a
  linear range of +/-0.25 cycle; the pure PLL (`atan2`, pilots) +/-0.5 cycle.
- `ch->fd`: carrier NCO frequency (Hz). `phas_acc`: acceleration integrator
  (Hz/s).
- `dt`: pre-detection (coherent integration) interval T (s).

### 2.2 Continuous-time model

The update above is the incremental (velocity) form of the NCO frequency

```
f_nco(t) = b3*wp*ep + a3*wp^2 * Int(ep) + wp^3 * Int(Int(ep))
```

so the loop filter is `F(s) = b3*wp + a3*wp^2/s + wp^3/s^2` and, with the NCO
as an integrator (1/s), the closed-loop transfer function is

```
         b3*wp*s^2 + a3*wp^2*s + wp^3
H(s) = ---------------------------------------
       s^3 + b3*wp*s^2 + a3*wp^2*s + wp^3

a3 = 1.1, b3 = 2.4, Bn = 0.7845*wp   (wp = sdr_b_pll / 0.7845)
```

The normalized poles are -2.103 and -0.148 +/- j0.673, i.e. the dominant pair
is lightly damped (zeta = 0.22).

### 2.3 Steady-state errors

The 3rd-order PLL is a type-3 loop: zero steady-state error to a constant
velocity (frequency) and to a constant acceleration (frequency ramp). It has a
constant error to jerk:

```
theta_e = j / wp^3     (cycles)     j: jerk in Hz/s^2 (LOS, carrier units)
```

### 2.4 Limitation

Once `|ep|` exceeds the discriminator range, the error wraps and the loop
receives a wrong-sign correction. All three integrators are driven only by
`ep`, so after a few slips the frequency and acceleration states are
corrupted and the loop diverges (see Section 5: e.g. Bp = 5 Hz loses lock at a
jerk of 50 Hz/s^2, and the frequency error grows to hundreds of Hz).

## 3. FLL-assisted PLL

### 3.1 Structure

```
ep (cyc) --+--> b3*wp -----------------------------------------+
           +--> a3*wp^2 -----------------------+               |
           +--> wp^3 -----+                    |               |
                          v                    v               v
ef (Hz)  --+--> wf^2 --->(+)--> [Int] ------->(+)--> [Int] -->(+)--> f_nco
           +--> a2*wf ---------------------------^
                         acceleration         frequency
                         (phas_acc, Hz/s)     (Hz)
```

The FLL and PLL share the two integrators: the acceleration integrator
(`phas_acc`) is driven by `wp^3*ep + wf^2*ef`, and the frequency integrator by
`a3*wp^2*ep + a2*wf*ef + phas_acc`.

### 3.2 Discriminators

Phase (unchanged):

```
ep = atan(QP/IP) / 2pi            (Costas, data channels)
ep = atan2(QP, IP) / 2pi          (pilot channels, no data)
```

Frequency (cross-product / dot-product of two consecutive prompts, the same
prompts that are given to the PLL):

```
dot   = IP0*IP + QP0*QP
cross = IP0*QP - QP0*IP
ef = atan(cross/dot) / (2pi*dt)   (Costas, linear range +/- 1/(4*dt) Hz)
ef = atan2(cross, dot) / (2pi*dt) (pilot,  linear range +/- 1/(2*dt) Hz)
```

where `(IP0, QP0)` is the previous prompt (`trk->Cf`) and `(IP, QP)` the
current one. `ef > 0` means the signal frequency is higher than the NCO.

### 3.3 Discrete update (as implemented)

```
W  = sdr_b_pll   / 0.7845        (3rd-order PLL, a3 = 1.1, b3 = 2.4)
Wf = sdr_b_fll_a / 0.53          (2nd-order FLL, a2 = 1.414)

phas_acc += (W^3*ep + Wf^2*ef) * dt
fd       += 2.4*W*(ep - ep_prev) + (1.1*W^2*ep + 1.414*Wf*ef) * dt
            + phas_acc * dt
```

With `sdr_b_fll_a = 0` (or `ef = 0`) the update is identical to v1.26.

### 3.4 Continuous model and linear equivalence

The 2nd-order FLL alone has `F_f(s) = a2*wf/s + wf^2/s^2` acting on the
frequency error, with

```
           a2*wf*s + wf^2
H_f(s) = ------------------ ,  a2 = 1.414, Bn = 0.53*wf
         s^2 + a2*wf*s + wf^2
```

and a steady-state frequency error to jerk of

```
f_e = j / wf^2     (Hz)
```

When both discriminators are in their linear range, the frequency error is the
derivative of the phase error (`ef = d(ep)/dt`), and the combined loop is again
a 3rd-order loop with modified coefficients:

```
D(s) = s^3 + (b3*wp + a2*wf)*s^2 + (a3*wp^2 + wf^2)*s + wp^3
```

Consequences:

1. **Small-error (locked) behavior.** The jerk error is still
   `theta_e = j/wp^3`, so the FLL does not raise the steady-state jerk
   tolerance of phase lock. It increases the effective bandwidth and the
   damping (Table 3), which reduces the transient error to acceleration steps.
2. **Large-error behavior (the actual benefit).** When the phase discriminator
   saturates or slips, `ep` becomes noise-like with roughly zero mean, but `ef`
   stays valid while `|f_e| < 1/(4*dt)` (Costas). The loop then behaves as a
   2nd-order FLL and keeps tracking frequency and acceleration, so phase lock
   is recovered when the dynamics relax. The loss-of-frequency-lock condition
   changes from `j/wp^3 > 0.25 cyc` (PLL) to `j/wf^2 > 1/(4*dt) Hz` (FLL).

Table 3. Effective noise bandwidth and damping of the combined loop (linear
model).

| Bp (Hz) | Bf (Hz) | Bn_eff (Hz) | min zeta |
|--------:|--------:|------------:|---------:|
| 5  | 0  | 5.00  | 0.22 |
| 5  | 2  | 6.07  | 0.33 |
| 5  | 5  | 8.41  | 0.79 |
| 5  | 10 | 12.92 | 1.00 |
| 10 | 0  | 10.00 | 0.22 |
| 10 | 2  | 10.90 | 0.25 |
| 10 | 5  | 12.83 | 0.38 |
| 10 | 10 | 16.82 | 0.79 |
| 15 | 2  | 15.83 | 0.23 |
| 15 | 5  | 17.54 | 0.30 |
| 15 | 10 | 21.13 | 0.49 |

### 3.5 FLL-assist gating by C/N0*T

A Monte Carlo test (Section 5.2) showed that the 1 ms Costas frequency
discriminator fails at moderate C/N0: `atan(cross/dot)` wraps when the noisy
phase change between two consecutive prompts exceeds +/-90 deg. The PLL
discriminator wraps only when the phase itself exceeds +/-90 deg, which is far
rarer (about 0.5% per epoch vs. 1e-4 at 35 dB-Hz, T = 1 ms). Each FLL wrap is a
+/-1/(2*dt) Hz impulse into `fd` and `phas_acc` and drives the PLL out of lock.
Averaging cross/dot over several epochs did not fix it.

The FLL term is therefore used only when the predetection SNR is high enough:

```
SNR = C/N0 (dB-Hz) + 10*log10(dt)
FLL assist on if SNR >= 11.0 dB (Costas, THRES_SNR_FLL_C)
                 SNR >=  8.0 dB (non-Costas, THRES_SNR_FLL)
```

| Channel type | dt | FLL assist active for C/N0 >= |
|---|---:|---:|
| Costas, data (e.g. L1C/A, E1B, B1I) | 1 ms | 41 dB-Hz |
| Costas, 4 ms code (E1B/C) | 4 ms | 35 dB-Hz |
| Costas, 10 ms code (L1CD, B1CD) | 10 ms | 31 dB-Hz |
| Pilot, coherent sum (`t_coh`) | 20 ms | 25 dB-Hz |

`ch->cn0` is the filtered tracking C/N0 (updated every 0.5 s), so the gate does
not chatter. Below the threshold the loop is exactly the original PLL.

### 3.6 Implementation details

- `PLL()` stores the prompt it receives in `trk->Cf` and its interval in
  `trk->dt_f`. The FLL term is used only when `dt_f == dt`, so that the first
  update after switching between per-epoch tracking (dt = T) and pilot
  coherent tracking (dt = K*T) is skipped instead of producing a wrong `ef`.
- In the pilot coherent branch `PLL()` is called once per K epochs with the
  coherent sum, so `ef` is computed between consecutive coherent sums
  (`dt = K*T`, `atan2`, range +/-1/(2*dt) Hz).
- `trk->Cf` is cleared in `trk_new()`, `trk_init()` and during the FLL
  pull-in phase (`lock*T <= T_FPULLIN`), so the first PLL update has no FLL
  term (`dot == 0`).
- The 1st-order pull-in FLL (`FLL()`, `b_fll_w`/`b_fll_n`) is unchanged.

## 4. Bandwidth design

### 4.1 Units

Dynamics are expressed in carrier Doppler units along the line of sight:

```
a [Hz/s]   = a [m/s^2]   / lambda
j [Hz/s^2] = j [m/s^3]   / lambda
L1 (lambda = 0.1903 m): 1 g = 51.5 Hz/s, 1 g/s = 51.5 Hz/s^2
L5/E5a (lambda = 0.2548 m): 1 g = 38.5 Hz/s, 1 g/s = 38.5 Hz/s^2
```

### 4.2 Error budgets (rules of thumb)

PLL (Costas, Kaplan/Ward):

```
3*sigma_PLL = 3*sigma_t + theta_e <= 45 deg       (90 deg for atan2 pilots)
sigma_t = (180/pi) * sqrt( Bn/(C/N0) * (1 + 1/(2*T*C/N0)) )   (deg)
theta_e = 360 * j / wp^3                                      (deg)
```

(Oscillator Allan deviation and vibration jitter must also be included for a
real budget; they are not modeled here.)

FLL:

```
3*sigma_FLL = 3*sigma_tf + f_e <= 1/(4*T)         (Hz)
sigma_tf = 1/(2*pi*T) * sqrt( 4*F*Bn/(C/N0) * (1 + 1/(T*C/N0)) )  (Hz)
F = 1 at high C/N0, 2 near threshold
f_e = j / wf^2                                    (Hz)
```

### 4.3 PLL bandwidth from jerk

```
Bp >= 0.7845 * (j / theta_e)^(1/3)     (theta_e in cycles)
```

Table 4. Maximum jerk for phase lock (Costas, T = 1 ms, theta_e = 45 deg -
3*sigma_t). Values in Hz/s^2 (g/s at L1).

| Bp (Hz) | wp | wp^3 | 45 dB-Hz | 40 dB-Hz | 35 dB-Hz |
|---:|---:|---:|---:|---:|---:|
| 5  | 6.37  | 259   | 31 (0.6)    | 30 (0.6)    | 27 (0.5)    |
| 10 | 12.75 | 2071  | 241 (4.7)   | 227 (4.4)   | 199 (3.9)   |
| 15 | 19.12 | 6990  | 801 (15.5)  | 741 (14.4)  | 626 (12.2)  |
| 20 | 25.49 | 16570 | 1871 (36.3) | 1709 (33.2) | 1394 (27.1) |
| 25 | 31.87 | 32362 | 3607 (70.0) | 3254 (63.1) | 2567 (49.8) |

Increasing Bp raises the jerk tolerance as Bp^3 but the thermal jitter only as
sqrt(Bp). Bp is limited by the discrete-time stability (Section 4.6) and by the
C/N0 at which tracking must be kept.

### 4.4 Acceleration steps

A step of acceleration (e.g. engine ignition or cutoff, staging) is a jerk
impulse. Its peak phase error is `k * da` where `k` is the peak of the impulse
response of `1/D(s)`. Table 5 lists `k` and the step `da` that gives a peak of
45 deg (0.125 cycle).

Table 5. Acceleration step tolerance (linear model).

| Bp (Hz) | Bf (Hz) | k (cyc per Hz/s) | da for 45 deg (Hz/s) | (g at L1) |
|---:|---:|---:|---:|---:|
| 5  | 0  | 0.0119 | 10  | 0.20 |
| 5  | 2  | 0.0093 | 13  | 0.26 |
| 5  | 5  | 0.0054 | 23  | 0.45 |
| 5  | 10 | 0.0023 | 55  | 1.07 |
| 10 | 0  | 0.0030 | 42  | 0.81 |
| 10 | 2  | 0.0027 | 47  | 0.90 |
| 10 | 5  | 0.0021 | 58  | 1.13 |
| 10 | 10 | 0.0014 | 92  | 1.79 |
| 15 | 0  | 0.0013 | 94  | 1.83 |
| 15 | 5  | 0.0011 | 115 | 2.23 |
| 15 | 10 | 0.0008 | 152 | 2.95 |
| 20 | 0  | 0.0007 | 168 | 3.25 |
| 20 | 10 | 0.0005 | 233 | 4.53 |

Here the FLL assist helps in the linear regime as well, because it increases
the damping of the lightly damped pole pair.

### 4.5 FLL bandwidth from jerk

The FLL must hold frequency lock under the worst jerk that the PLL cannot
follow:

```
Bf >= 0.53 * sqrt( j / f_e )    with f_e <= 1/(4*T) - 3*sigma_tf (with margin)
```

Table 6. Jerk that gives f_e = 50 Hz (T = 1 ms, range 250 Hz) and f_e = 5 Hz
(pilot T = 20 ms, range 25 Hz). Values in Hz/s^2 (g/s at L1).

| Bf (Hz) | wf | wf^2 | f_e = 50 Hz | f_e = 5 Hz |
|---:|---:|---:|---:|---:|
| 1  | 1.89  | 3.6   | 178 (3.5)     | 18 (0.35)   |
| 2  | 3.77  | 14.2  | 712 (13.8)    | 71 (1.38)   |
| 5  | 9.43  | 89.0  | 4450 (86.4)   | 445 (8.64)  |
| 10 | 18.87 | 356.0 | 17800 (345.4) | 1780 (34.5) |
| 15 | 28.30 | 801.0 | 40050 (777.2) | 4005 (77.7) |

Note that in frequency-lock-only mode (phase not locked) carrier phase and
navigation data are degraded, while Doppler and code tracking continue.

Costs of a larger Bf: higher effective bandwidth and phase jitter (Table 3,
Table 7), and a lower discrete-time stability limit (Table 8).

### 4.6 Discrete-time stability

Spectral radius analysis of the implemented update equations (NCO frequency
applied over the next interval) gives:

Table 8. Maximum stable Bp*T for a given ratio Bf/Bp.

| Bf/Bp | max Bp*T | max Bf*T |
|---:|---:|---:|
| 0   | 0.460 | -     |
| 0.2 | 0.420 | 0.084 |
| 0.4 | 0.375 | 0.150 |
| 1.0 | 0.270 | 0.270 |
| 2.0 | 0.175 | 0.350 |

Keep a margin of at least 2x for good transient behavior. The pilot coherent
branch selects coherent integration only when `sdr_b_pll * dt < 0.4`; this
check does not include `b_fll_a`, so with a large Bf/Bp and `t_coh = 0.02`
check Table 8 (e.g. Bp = 15, Bf = 15, dt = 0.02 gives Bp*T = 0.30 > 0.27,
unstable).

### 4.7 Design procedure

1. Define the worst-case LOS jerk `j` and acceleration step `da` (Hz/s^2,
   Hz/s) for the highest carrier frequency used (largest Doppler per m/s).
2. Choose Bp from Table 4 so that the expected jerk keeps phase lock, and
   check the acceleration step in Table 5.
3. Choose Bf from Table 6 so that the worst jerk (and transients beyond it)
   keeps frequency lock with margin, typically Bf = 0.2 to 0.7 x Bp.
4. Check the jitter at the minimum C/N0 (Section 4.2, Table 7), the
   discrete-time stability (Table 8) and the FLL gate (Section 3.5).

Suggested settings:

| Scenario | b_pll | b_fll_a | Notes |
|---|---:|---:|---|
| Static / pedestrian | 5 | 2 | library default |
| Land vehicle (<= ~1 g/s) | 10 | 2 to 5 | `pocket_trk_default.conf` b_pll |
| Aircraft / launch vehicle (5 to 15 g/s) | 15 to 20 | 5 to 10 | T = 1 ms Costas: FLL active only >= 41 dB-Hz |

## 5. Verification

### 5.1 Method

A Python model of exactly the update equations of Section 3.3 was run with
complex prompts `exp(j*2pi*(phi_sig - phi_nco)) + noise` averaged over each
interval, a constant Doppler plus a jerk applied for 3 s (from t = 2 s), and
random seed 1. The model has no code loop, no data bits, no oscillator noise
and uses the true C/N0 for the gate. Analytic values were checked first:
Bn of the 3rd-order PLL = 5.00 Hz for Bp = 5, Bn of the 2nd-order FLL =
2.00 Hz for Bf = 2.

### 5.2 Phase jitter without dynamics (no gate)

Table 7. RMS phase error (deg), Bp = 5 Hz.

Costas, T = 1 ms (20 s):

| C/N0 | Bf = 0 | Bf = 2 | Bf = 5 |
|---:|---:|---:|---:|
| 45 | 0.66 | 0.74  | 0.90  |
| 42 | 0.94 | 1.06  | 1.28  |
| 40 | 1.21 | 1.36  | 1.63  |
| 39 | 1.37 | 6.50  | 6.68  |
| 38 | 1.57 | 24.70 | 18.85 |
| 36 | 2.06 | 37.83 | 38.77 |

Pilot (atan2), T = 20 ms (200 s):

| C/N0 | Bf = 0 | Bf = 2 | Bf = 5 |
|---:|---:|---:|---:|
| 40 | 1.54  | 1.80  | 2.37  |
| 30 | 4.91  | 5.76  | 7.60  |
| 25 | 9.02  | 10.60 | 14.01 |
| 23 | 11.87 | 15.84 | 19.65 |
| 21 | 15.84 | 27.29 | 29.98 |

The Costas cliff at C/N0*T of about 9 to 10 dB and the gradual pilot
degradation set the gate thresholds of Section 3.5 (11 dB / 8 dB). With the
gate, the jitter is the Bf = 0 column below the threshold.

### 5.3 Jerk tolerance

Result: `slip / max|f_err|` = accumulated cycle slips at the end of the run
(cycles) / maximum frequency error (Hz). A slip count in the thousands means
frequency lock was lost.

Costas, T = 1 ms, 45 dB-Hz (gate on):

| Bp | Bf | 50 | 200 | 500 | 1000 | 2000 Hz/s^2 |
|---:|---:|---|---|---|---|---|
| 5  | 0  | 1323/590 | 6216/2673 | lost | lost | lost |
| 5  | 2  | 4.5/7    | 40.5/18   | 105/40   | 211/76  | 422.5/148 |
| 5  | 10 | 0/3      | 1.5/7     | 4/8      | 8.5/9   | 17/11     |
| 10 | 0  | 0/2      | 0/3       | lost     | lost    | lost      |
| 10 | 5  | 0/3      | 0/4       | 9.5/15   | 30/20   | 65.5/31   |
| 15 | 0  | 0/4      | 0/4       | 0/5      | lost    | lost      |
| 15 | 5  | 0/5      | 0/5       | 0/6      | 0/19    | 58/33     |
| 15 | 10 | 0/6      | 0/6       | 0/6      | 0/9     | 10/22     |

Pilot (atan2), T = 20 ms, 30 dB-Hz (gate on):

| Bp | Bf | 50 | 100 | 200 | 500 | 1000 Hz/s^2 |
|---:|---:|---|---|---|---|---|
| 5  | 0 | 0/4 | lost  | lost | lost  | lost  |
| 5  | 2 | 0/5 | 2/14  | 40/18 | lost | lost  |
| 5  | 5 | 0/6 | 0/9   | 5/14 | 15/33 | 33/62 |
| 10 | 0 | 0/6 | 0/9   | 0/15 | 0/33  | lost  |
| 10 | 5 | 0/8 | 0/11  | 0/17 | 0/35  | lost  |

Observations:

- The pure PLL loses lock close to the jerk predicted by Table 4 (Bp = 10:
  227 to 241 Hz/s^2 predicted, ok at 200 and lost at 500 in simulation).
- With the FLL assist the loop keeps frequency lock far beyond the PLL limit.
  The maximum frequency error follows `j/wf^2` (Bf = 2, j = 2000:
  141 Hz predicted, 148 Hz simulated).
- With the pilot at T = 20 ms the FLL range is only +/-25 Hz, so frequency lock
  is lost once `j/wf^2` approaches 25 Hz (Bf = 2: j ~ 350 Hz/s^2). High
  dynamics should use a short `t_coh` or a larger Bf for pilots.

## 6. Changes

| File | Change |
|---|---|
| `src/sdr_ch.c` | `PLL()`: add 2nd-order FLL assist with C/N0*T gate; add `B_FLL_A`, `THRES_SNR_FLL_C`, `THRES_SNR_FLL`, `sdr_b_fll_a`; reset `Cf`/`dt_f` in `trk_new()`, `trk_init()` and FLL pull-in |
| `src/pocket_sdr.h` | `sdr_trk_t`: add `Cf` (previous prompt) and `dt_f` (its interval) |
| `src/sdr_rcv.c` | `sdr_rcv_setopt()`: add option `b_fll_a` |
| `app/pocket_trk/pocket_trk_default.conf` | add `b_fll_a = 2.0` |
| `doc/command_ref.md` | add `b_fll_a` |

Option:

| Option | Description | Library default | pocket_trk default |
|---|---|---:|---:|
| `b_fll_a` | FLL-assist bandwidth for the 3rd-order PLL (Hz) (0: off) | 2.0 | 2.0 |

## 7. Limitations and future work

- For 1 ms Costas channels the FLL assist is active only at C/N0 >= 41 dB-Hz.
  Extending it to lower C/N0 needs a more robust frequency discriminator (e.g.
  bit-synchronized 10 to 20 ms blocks for GPS L1C/A).
- The gate thresholds are compile-time constants derived from simulation.
- `b_fll_a` is not yet exposed in the Web UI option page
  (`src/sdr_web.c`, `html/js/pages/opts.js`) or the Python GUI
  (`python/sdr_opt.py`).
- The coherent-integration check `sdr_b_pll * dt < 0.4` does not account for
  `b_fll_a` (Section 4.6).
- Validation with recorded IF data and real dynamic platforms is pending.

## 8. References

- E. D. Kaplan and C. J. Hegarty (eds.), Understanding GPS/GNSS: Principles
  and Applications, 3rd ed., Artech House, 2017, Chapter 8 (carrier tracking
  loops, loop filter coefficients, PLL/FLL error budgets).
- P. W. Ward, Performance Comparisons Between FLL, PLL and a Novel
  FLL-Assisted-PLL Carrier Tracking Loop Under RF Interference Conditions,
  ION GPS-98, 1998.
