# Input shaping

Input shaping cancels the **ringing** (ghosting) that the frame of a machine adds to the motion:
after every change of acceleration (a corner, the start or the end of a move) the print head keeps
oscillating at the resonance frequency of the frame, and those oscillations are printed as ripples
next to the features that caused them. Instead of the commanded motion, each axis follows a weighted
sum of delayed copies of it, whose oscillations cancel each other at the frequency the shaper is
tuned to.

In printhor it is the `with-input-shaping` cargo feature (off by default; without it no code is
added) and the `M593` G-code. It shapes the X and Y axes.

## 1. The model: a damped oscillator

Along an axis, the head (position $y$) is modeled as a mass attached to the motor (position $x$) by
the frame, a spring with damping. Its deviation from the motor, $z = y - x$, obeys

$$
\ddot z + 2\zeta\omega\,\dot z + \omega^2 z = -\ddot x
$$

with $\omega = 2\pi f$ the natural angular frequency of the frame and $\zeta$ its damping ratio
(typically $f$ = 20-100 Hz, $\zeta$ = 0.05-0.2 on 3D printers). The oscillation decays at the damped
frequency

$$
\omega_d = \omega\sqrt{1-\zeta^2}, \qquad t_d = \frac{2\pi}{\omega_d} = \frac{1}{f\sqrt{1-\zeta^2}}
$$

($t_d$ is the damped period). Under a constant acceleration $a$ the head settles at the
**quasi-static deflection** $z = -a/\omega^2$; every change of the acceleration starts an
oscillation around it: the ringing.

## 2. Impulse trains that cancel the vibration

An impulse of area $A$ at time $t_j$ excites the oscillator with
$z_j(t) \propto A\, e^{-\zeta\omega (t-t_j)} \sin(\omega_d (t - t_j))$. After the last of $N$
impulses $A_j$ at $t_j$ (with $\sum_j A_j = 1$), the vibration left relative to a single unit
impulse is (Singer & Seering, 1990)

$$
V(\omega, \zeta) = e^{-\zeta\omega t_N}\sqrt{C^2 + S^2}, \qquad
C = \sum_{j} A_j e^{\zeta\omega t_j}\cos(\omega_d t_j), \quad
S = \sum_{j} A_j e^{\zeta\omega t_j}\sin(\omega_d t_j)
$$

An **input shaper** is an impulse train with $V(\omega, \zeta) = 0$ at the frequency of the frame.
Convolving any command with it (by linearity) cancels the vibration of every one of its changes:

$$
x_s(t) = \sum_{j} A_j\, x(t - t_j)
$$

Since $\sum_j A_j = 1$, the shaped motion ends exactly where the commanded one does, $t_N$ later.

**ZV (zero vibration)**, the shortest one (Smith's *posicast*, 1957): two impulses. $C = S = 0$
requires $\omega_d t_2 = \pi$ and $A_1 = A_2 e^{\zeta\omega t_2}$:

$$
t_2 = \frac{t_d}{2}, \qquad \frac{A_2}{A_1} = K = e^{-\zeta\pi/\sqrt{1-\zeta^2}}
$$

The other shapers add impulses to make $V$ also flat around the frequency (robust to tuning errors)
at the price of a longer duration. With $K = e^{-s\,\zeta\pi/\sqrt{1-\zeta^2}}$ (the amplitudes are
normalized by their sum):

| `T` | Shaper | Amplitudes | Delays ($\times t_d$) | $s$ | Notes |
|---|---|---|---|---|---|
| 0 | ZV | $1,\ K$ | $0,\ ½$ | 1 | Marlin's. Shortest; sensitive to a frequency error |
| 1 | EI | $\tfrac{1+v}{4},\ \tfrac{1-v}{2}K,\ \tfrac{1+v}{4}K^2$ | $0,\ ½,\ 1$ | 1 | Extra-insensitive: $V \le v$ = 5% over a wide band |
| 2 | MZV | $1-\tfrac{1}{\sqrt2},\ (\sqrt2-1)K,\ (1-\tfrac{1}{\sqrt2})K^2$ | $0,\ ⅜,\ ¾$ | ¾ | Robust, less smoothing than EI/ZVD |
| 3 | ZVD | $1,\ 2K,\ K^2$ | $0,\ ½,\ 1$ | 1 | Zero vibration and zero derivative at the frequency |

(the formulas of Klipper's `shaper_defs.py`; EI after Singhose *et al.*).

The price of shaping is **smoothing**: the shaped path is a weighted average of the commanded path
over the duration of the shaper, so the corners are slightly rounded. It grows with the variance of
the impulse times,

$$
\sigma^2 = \sum_j A_j (t_j - \bar t)^2, \qquad \bar t = \sum_j A_j t_j
$$

(times the acceleration: a displacement). The shaped axes also lag the commanded motion by the time
centroid $\bar t$.

## 3. Implementation in printhor

`motion/input_shaping.rs` (with the stepper task, `tasks/task_stepper.rs`).

### 3.1 The shapers

`Shaper::new` builds the impulses of the table above from `M593` (`F`, `D`, `T`), with the delays
rounded to whole microseconds (as the motion windows are). The shaped value is computed as

$$
x_s(t) = x(t) + \sum_{j\ge1} A_j\,\bigl(x(t - t_j) - x(t)\bigr)
$$

which is exactly $x$ at rest, whatever the rounding of the amplitudes. The frequency is limited to
$f \ge$ 20 Hz and the damping to $\zeta \le 0.5$: the longest shaper lasts $t_d \le$ 57.7 ms.

### 3.2 The commanded motion and its history

The step generator works in **windows** (micro-segments of `MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY`):
at the end of each window the steps of every axis are computed, and inside it a second-order DDA
ramps the step rate between the velocities at its ends. The interpolator keeps computing the
commanded steps of every axis, as without input shaping: the positions, the deviation corrections
and everything derived from them are unchanged.

The commanded position $R$ (steps, continuous) and velocity of X and Y at each window end are kept
in a history (`ShaperHistory`), in a frame relative to the motor (a `G92` does not move it). A sample
stores the velocity arriving and the velocity leaving it, so that the velocity jump of a corner is
kept exactly. The position at a delayed instant is interpolated between the samples $a$, $b$ around
it with a cubic Hermite polynomial ($h$ the interval, $\tau \in [0,1]$ the fraction of it,
$\Delta = R_b - R_a$):

$$
R(\tau) = R_a + \tau^2(3 - 2\tau)\,\Delta + h\,\bigl(\tau(1-\tau)^2\,v_a + \tau^2(\tau - 1)\,v_b\bigr)
$$

$$
\dot R(\tau) = 6\tau(1-\tau)\,\frac{\Delta}{h} + (1-\tau)(1-3\tau)\,v_a + \tau(3\tau - 2)\,v_b
$$

exact for the cubic phases of the S-curve profile. Windows closer than 2 ms are merged, so 32
samples always cover the longest shaper.

### 3.3 The shaped steps

At the end of every window, for each shaped axis:

1. the shaped target $p = R + \sum_j A_j (R(t - t_j) - R)$ and velocity $\dot p$ are computed from
   the history;
2. the motor goes to the step closest to the target, $m = \lfloor p + ½ \rfloor$, in **either
   direction** (a corner may reverse a shaped axis inside a segment): the steps of the window are
   $m - m_{prev}$, exact (the step executor never drops steps);
3. the step rate ramps between $\dot p$ at the start and at the end of the window (second-order
   DDA), offset so that the window covers exactly its steps.

The steps the interpolator computed for X and Y are replaced by those; the other axes (Z, the
extruder) are not shaped.

### 3.4 Stops, aborts and settings

* **Stops** (the queue runs empty, a dwell `G4`/`M400`, a homing): the commanded motion stopped but
  the shaped one has not. Windows of the shaped axes are queued for the duration of the shaper; the
  last one ends **exactly at the commanded step**, so the motors are where the position says.
* **Abort** (`M410`, alarm): the motors stop where they are, behind the commanded motion; the
  current position is corrected by that lag.
* **`M593`** changes are applied from the next stop. Defaults: ZVD, 40.01 Hz, $\zeta$ 0.15, the shaper
  `s_plot` tunes on `pen-test.gcode` with the motion of the SKR Mini E3 V3 (Marlin's: ZV, 40 Hz,
  $\zeta$ 0.15), overridable at build time (`DEFAULT_INPUT_SHAPING_FREQ_X`, `..._FREQ_Y`, `..._DAMPING_X`,
  `..._DAMPING_Y`, `..._TYPE`).
* **Tool actions** (laser `M3`/`M4`, fans) stay synchronized with the commanded motion: the shaped
  axes lag them by the time centroid of the shaper (a few ms).

### 3.5 `M593`

```
M593 [X] [Y] [F<Hz>] [D<damping>] [T<type>]
```

`F` frequency (0 disables the shaper, otherwise 20 Hz or above), `D` damping ratio (0 to 0.5), `T`
type (0 ZV, 1 EI, 2 MZV, 3 ZVD), of the axes given (`X`, `Y`; both without them). Without values it
reports `M593 X F<f> D<d> T<t>` and `M593 Y ...`. Stored by `M500`, restored by `M501`, reset by
`M502`, reported by `M503`.

## 4. Tuning: measuring the resonance

A shaper must be tuned to the resonance of the frame ($f$, $\zeta$). With an accelerometer, from the
peak of the vibration spectrum (Klipper's `TEST_RESONANCES`); without one, from a ringing test print:
the distance $\lambda$ between ripples printed at speed $v$ gives the damped frequency
$f_d = v / \lambda$.

`s_plot` does it on its model of the frame (`S_PLOT_VIBRATION=<Hz>,<damping>`, 40 Hz and 0.15 by
default) from the run without input shaping, as one would on the ghosting of a print: wherever an
axis cruises undisturbed (no acceleration, no velocity jump since the last change) the head
oscillates freely,
$z(t) = C\,e^{-\zeta\omega t}\sin(\omega_d t + \varphi)$:

* consecutive zero crossings are $t_d/2$ apart: $f_d = 1/(2\,\overline{\Delta t})$;
* the peaks of consecutive half cycles decay by $r = e^{-\pi\zeta/\sqrt{1-\zeta^2}}$ (logarithmic
  decrement $\delta = -\ln r$):

$$
\zeta = \frac{\delta}{\sqrt{\pi^2 + \delta^2}}, \qquad f = \frac{f_d}{\sqrt{1 - \zeta^2}}
$$

(averaged over every half cycle measured, weighted by its amplitude). The type is then chosen with
Klipper's score, a compromise between the vibration left and the smoothing:

$$
\text{score} = \sigma^2 \left(\bar V^{1.5} + 0.2\,\bar V + 0.01\right)
$$

with $\bar V$ the RMS of $V(\omega', \zeta)$ over $\pm10\%$ of the frequency (the frame is not that
constant: the load, the position of the head).

## 5. Seeing it: `s_plot`

Built with `--features s-plot-bin,with-input-shaping`, `s_plot` runs the job twice: without input
shaping (`plot-without-shaping.pdf`, `plot_axes-without-shaping.pdf` and their CSV files) and with
the shaper tuned from the first run (`plot-with-input-shaping.pdf`, `plot_axes-with-input-shaping.pdf`).
`S_PLOT_SHAPER=<type>,<Hz>,<damping>` (or `off`) forces a shaper instead.

The motors are driven as the step executor drives them (the velocity of each window ramps between
the rates at its ends, offset to cover exactly its displacement) and the head ringing is integrated
on the model of the frame. Reported and plotted:

* **ringing**: the oscillation of the head around the quasi-static deflection, $z + \ddot x/\omega^2$
  (the deflection itself, which no shaper removes, is excluded);
* **residual ringing**: the ringing while the motion is steady (constant acceleration, no velocity
  jump) for longer than $1.2/f$: the ghosting of the prints, what input shaping removes. Right after
  a change no shaper can prevent the first oscillation (its first impulse): it only cancels it with
  the next impulses;
* a **zoom** of the ringing around the strongest corner (the same interval in both runs), and the
  vibration the shaper leaves at each frequency, $V(\omega, \zeta)$.

On the `pen-test.gcode` job (40 Hz, $\zeta$ 0.15 frame), the frame identified from the run without
shaping is 40.01 Hz, $\zeta$ 0.150; the shaper chosen (ZVD) reduces the residual ringing by 98.8%
(RMS) and 99.1% (peak), and the excitation of the 25-100 Hz band from 285 to 166 mm/s² RMS.

## 6. Verification

* `motion/input_shaping.rs` tests: the residual vibration of each shaper at its frequency (none for
  ZV, MZV and ZVD; within the 5% of EI), the exactness of the history interpolation (cubics, corner
  velocity jumps, merged samples), the whole pipeline through the real step plan executor (exact
  step counts, tracking of the ideal shaped motion within ½ step, the motors ending exactly at the
  commanded steps) and the ringing left in an oscillator (0.003% to 6.7% of the unshaped one).
* Integration scenario T39 (native simulator): `M593` (defaults, validation, `M503`, `M502`), the
  same square path with and without shaping (the shaped corners are cut, both end on the same
  steps, the position is the one of the steps) and an `M410` halfway through a shaped move.

## References

1. O. J. M. Smith, "Posicast control of damped oscillatory systems," *Proceedings of the IRE*,
   vol. 45, no. 9, pp. 1249-1255, 1957.
2. N. C. Singer and W. P. Seering, "Preshaping command inputs to reduce system vibration," *ASME
   Journal of Dynamic Systems, Measurement, and Control*, vol. 112, no. 1, pp. 76-82, 1990.
3. W. Singhose, W. Seering and N. Singer, "Residual vibration reduction using vector diagrams to
   generate shaped inputs," *ASME Journal of Mechanical Design*, vol. 116, no. 2, pp. 654-659, 1994.
4. W. Singhose, "Command shaping for flexible systems: a review of the first 50 years,"
   *International Journal of Precision Engineering and Manufacturing*, vol. 10, no. 4,
   pp. 153-168, 2009.
5. L. Biagiotti and C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots*,
   Springer, 2008 (the S-curve profiles shaped here).
6. W. T. Thomson and M. D. Dahleh, *Theory of Vibration with Applications*, 5th ed., Prentice Hall,
   1998 (damped free vibration, logarithmic decrement).
7. Klipper documentation, "Resonance Compensation" and "Measuring Resonances",
   https://www.klipper3d.org/Resonance_Compensation.html; shaper definitions in
   `klippy/extras/shaper_defs.py`, https://github.com/Klipper3d/klipper.
8. Marlin firmware documentation, "M593 - ZV Input Shaping",
   https://marlinfw.org/docs/gcode/M593.html.
