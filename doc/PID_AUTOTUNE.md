# PID autotune

The temperature of the hot-end and of the bed is regulated by a PID controller. Its constants depend
on the heater (power, thermal mass, sensor lag): `M303` measures them with Åström and Hägglund's
**relay feedback experiment** and computes the constants with the **Ziegler–Nichols** rules, as
Marlin does (same G-code, same report, comparable constants), with a more robust estimation.

## 1. The controller

The output (heater power, $u \in [0, 255]$) is computed every control period $\Delta t$ (0.5 s)
from the error $e = T_{target} - T$:

$$
u = K_p\, e + K_i \int e\,dt - D_f
$$

* **Derivative on the measurement** (no kick when the target changes), low-pass filtered with a
  first order filter of time constant $\tau_D$:

  $$
  D_f \leftarrow \alpha\, D_f + (1-\alpha)\,K_d\,\frac{T_k - T_{k-1}}{\Delta t}, \qquad
  \alpha = \frac{\tau_D}{\tau_D + \Delta t}
  $$

  $\tau_D$ = 2.4 s is Marlin's filter (`PID_K1` = 0.95 applied every `PID_dT` ≈ 0.128 s:
  $\tau_D = \text{PID\_dT}\cdot K_1/(1-K_1)$), so that the constants tuned for Marlin behave the same
  whatever the control period.
* **Anti-windup**: the integral term is clamped to the output range, $0 \le K_i \int e \le 255$.
* **Functional range**: farther than 10 °C from the target the heater runs at full power (below) or
  off (above), the integral frozen (Marlin's `PID_FUNCTIONAL_RANGE`).

The constants are in physical units (seconds): $K_i$ in output units per °C·s, $K_d$ in output units
per °C/s, as `M301` (hot-end) and `M304` (bed) take them.

## 2. The relay experiment

A heater with a dead time and lags has an **ultimate frequency** $\omega_u$ where its phase lag is
180°, and an **ultimate gain** $K_u$: the proportional gain that keeps it oscillating there. The
Ziegler–Nichols rules compute the PID constants from $K_u$ and the ultimate period
$T_u = 2\pi/\omega_u$.

Instead of raising a proportional gain until the loop oscillates (slow, risky), the heater is
driven by a **relay** around the target (Åström and Hägglund, 1984):

$$
u = \begin{cases} b + d & T < T_{target} \\ b - d & T > T_{target} \end{cases}
$$

The loop settles into a limit cycle at (close to) the ultimate frequency. With the describing
function of the relay (the gain of its fundamental for a sinusoidal input of amplitude $a$),

$$
N(a) = \frac{4d}{\pi a}
$$

the oscillation satisfies $G(j\omega_u)\,N(a) = -1$, so

$$
K_u = \frac{4d}{\pi a}, \qquad T_u = \text{period of the oscillation}
$$

## 3. Implementation in printhor

`hwa/controllers/heater_controller/autotune.rs` runs in the temperature control loop, one step per
reading: it never blocks.

1. **Relay oscillation.** Both $b$ and $d$ start at half power (the first heating is at full power).
   The relay switches when the temperature crosses the target, and not before 2 s in the same state:
   the noise of the readings (or a glitch) can not toggle it. There is no hysteresis: it would move
   the oscillation away from the ultimate point (a bed oscillates by a few tenths of a degree).
2. **Bias balancing** (as Marlin). After each cycle, with $t_h$ and $t_l$ the durations of its
   heating and cooling halves:

   $$
   b \leftarrow b + d\,\frac{t_h - t_l}{t_h + t_l}, \qquad d \leftarrow \min(b,\ 255 - b)
   $$

   ($b$ kept within [20, 235]). The bias moves towards the power that holds the target: the
   oscillation becomes symmetric and $d$ is the largest amplitude the bias allows.
3. **Estimation.** From the third cycle on, each cycle gives an estimate of the ultimate point:
   $T_u$ is the period of the cycle, and $K_u$ the ratio of the **fundamentals** of the relay output
   $u$ and of the temperature deviation $y = T - T_{target}$ over the cycle,

   $$
   U_1 = \int_{cycle} (u - b)\, e^{-j\omega t}\,dt, \qquad
   Y_1 = \int_{cycle} y\, e^{-j\omega t}\,dt, \qquad K_u = \frac{|U_1|}{|Y_1|}
   $$

   (projections accumulated reading by reading at the frequency of the previous cycle: no readings
   are stored; the output is integrated exactly, as it is constant between readings). For a square
   wave of amplitude $d$ and a sinusoidal temperature of amplitude $a$ it is exactly $4d/(\pi a)$;
   unlike Marlin's $4d/(\pi a)$ with $a$ half the distance between the extremes, it is not biased by
   the shape of the temperature (not a sinusoid) and the noise of the readings averages out.
4. **Convergence.** It finishes after at least `C` cycles (at least 3) once the last three
   estimates agree within 15% of their median, or 5 cycles later anyway (reported as not settled).
   The result is the **median of the last five** estimates: one disturbed cycle can not spoil it
   (Marlin takes the last cycle).
5. **Tuning rules** (as Marlin, so that the constants are comparable), with $K_i = K_p/T_i$ and
   $K_d = K_p T_d$:

   | Rule | Used for | $K_p$ | $T_i$ | $T_d$ | $K_i$ | $K_d$ |
   |---|---|---|---|---|---|---|
   | Classic Ziegler–Nichols | hot-ends | $0.6\,K_u$ | $T_u/2$ | $T_u/8$ | $2K_p/T_u$ | $K_p T_u/8$ |
   | "No overshoot" variant | bed | $0.2\,K_u$ | $T_u/2$ | $T_u/3$ | $2K_p/T_u$ | $K_p T_u/3$ |

**Safeguards** (as Marlin): it fails when the temperature exceeds the target by 30 °C, or when a half
cycle (the first heating included) lasts more than 20 minutes. The heater protections (`MAXTEMP`,
`MINTEMP`, heating watch, sensor faults) still apply; only the thermal runaway watch is suspended
while the temperature oscillates on purpose. The heater is left off at the end.

On simulated heaters (heater block, thermistor lag, dead time, noise and quantization of the
readings) the ultimate point is found within 15%: $K_u$ a bit lower and $T_u$ a bit longer than the
true ones, because the relay acts at the readings (on average half a control period after the
crossing). That bias gives gentler constants. With the constants autotuned for the simulated
hot-end, a step to the target overshoots 2.9 °C and settles within 0.2 °C.

## 4. `M303`

```
M303 [E<heater>] [S<temperature>] [C<cycles>] [U[1]]
```

`E` the heater (0 the hot-end, -1 the bed), `S` the target (°C), `C` the minimum number of cycles
(at least 3), `U` (or `U1`) applies the constants found. As Marlin: the temperature is reported every
2 s, and every cycle with its bias, amplitude, extremes and, from the third one, $K_u$, $T_u$ and
the constants; it ends with `PID Autotune finished! ...` and the constants as `#define DEFAULT_Kp`
(or `DEFAULT_bedKp`) lines. The `ok` is sent at the end (the host waits for it). The constants
applied can be stored with `M500`.

## 5. Verification

* `heater_controller/autotune/tests.rs`, on simulated hot-ends and beds (heater block, thermistor
  lag, dead time, noise and quantization):
  `hot_end_autotune_finds_the_ultimate_point` (against the ultimate point of the model),
  `hot_end_pid_from_the_autotune_is_well_damped` (the closed loop with the constants found:
  overshoot and settling), `bed_autotune_uses_the_no_overshoot_rule`,
  `noisy_readings_do_not_spoil_the_result`, `the_bias_balances_a_heater_far_from_half_power`,
  `a_glitch_does_not_toggle_the_relay`, `every_cycle_is_reported_and_the_cycles_are_at_least_c`,
  the safeguards (`a_target_far_below_the_temperature_fails_at_once`,
  `a_heater_that_can_not_reach_the_target_times_out`) and `marlin_tuning_rules`.
* `heater_controller/safety_tests.rs`: `autotune_suspends_the_runaway_detection_but_not_maxtemp`.
* Integration scenario T37 (native simulator): a whole `M303` on the simulated heaters, its report,
  the `U1` application and the heater left off.

## References

1. K. J. Åström and T. Hägglund, "Automatic tuning of simple regulators with specifications on phase
   and amplitude margins," *Automatica*, vol. 20, no. 5, pp. 645-651, 1984.
2. K. J. Åström and T. Hägglund, *PID Controllers: Theory, Design, and Tuning*, 2nd ed., Instrument
   Society of America, 1995.
3. J. G. Ziegler and N. B. Nichols, "Optimum settings for automatic controllers," *Transactions of
   the ASME*, vol. 64, pp. 759-768, 1942.
4. A. Gelb and W. E. Vander Velde, *Multiple-Input Describing Functions and Nonlinear System
   Design*, McGraw-Hill, 1968 (describing function of the relay).
5. K. J. Åström and R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*,
   Princeton University Press, 2008.
6. Marlin firmware documentation, "M303 - PID autotune", https://marlinfw.org/docs/gcode/M303.html.
