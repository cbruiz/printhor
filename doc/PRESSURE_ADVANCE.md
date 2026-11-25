# Pressure advance

Pressure advance (Marlin's *linear advance*) compensates the lag between the extruder motor and the
flow of molten plastic out of the nozzle. Without it, the extrusion falls short when the head
accelerates and is in excess when it brakes: thin starts of the lines, bulging corners, blobs and
gaps at the ends of the lines. The extruder is fed ahead of its nominal position by an amount
proportional to its speed.

In printhor it is the `with-pressure-advance` cargo feature (off by default; it requires
`with-e-axis`) and the `M900` G-code.

## 1. The model: the nozzle as a pressure lag

The filament is compressed (and the melt chamber expands) to build the pressure that pushes the
melt through the nozzle. A first order model of the liquefier (Bellini *et al.*, 2004; Tronvoll
*et al.*, 2019) relates the flow out of the nozzle $q$ to the feed of the filament by the motor
$\dot e$:

$$
q(t) + K\,\dot q(t) = \dot e(t)
$$

with $K$ (seconds) the time constant of the pressure: it grows with the length of the filament path
and its compliance (Bowden tubes: 0.3-1 s; direct drives: 0.02-0.1 s), the viscosity of the melt and
the restriction of the nozzle. For the flow to be the nominal one, $q = \dot e_{nominal}$, the motor
must be fed with

$$
\dot e_{motor} = \dot e + K\,\ddot e \quad\Longrightarrow\quad e_{motor}(t) = e(t) + K\,\dot e(t)
$$

The motor **leads** its nominal position by $K$ times the nominal extruder speed (Klipper's
formulation, the same as Marlin's linear advance 1.5): while accelerating the lead grows (extra
filament builds the pressure), while braking it shrinks and the motor may even retract (the
pressure is relieved).

## 2. Within a segment

A segment of path length $D$ moves the extruder $u_e D$ ($u_e$ is the E component of its unit
vector, the flow rate included) along its motion profile $s(t)$, with path speed $v = \dot s$ and
acceleration $a = \dot v$. In extruder steps ($n_e$ steps per unit), the lead and its rate are

$$
L(t) = \kappa\, v(t), \qquad \dot L(t) = \kappa\, a(t), \qquad \kappa = K\,u_e\,n_e
$$

The motion profiles of printhor are S-curves (jerk limited, see [MOTION_PROFILE.md](MOTION_PROFILE.md)),
so the acceleration is continuous and the motor velocity $u_e n_e v + \kappa a$ has no jumps (a
trapezoidal profile would make it jump at every change of acceleration).

**Which moves are advanced**: as in Marlin and Klipper, the extrusions only, with the extruder moving
forward along with some other axis. Neither the retractions nor the extruder-only moves are.

## 3. Junctions: spreading the lead mismatch

The lead is continuous across a junction when both segments extrude at the same ratio (same $u_e$:
the path speed is continuous). When they do not (an extrusion followed by a travel, a change of the
line width), the lead the motor carries from the previous segment, $L_{carried}$, differs from the
lead of the new one at its start, $\kappa v_0$, by

$$
\Delta = L_{carried} - \kappa\, v_0
$$

Applying it at once would be an instantaneous jump of the motor. It is spread along the segment,
proportionally to the path: the target of the motor, relative to where it is at the start of the
segment, is

$$
p(s, v) = \left(u_e n_e - \frac{\Delta}{D}\right) s + \kappa\,(v - v_0)
$$

continuous at both ends: $p = 0$ at the start, and at the end the motor leads the nominal position by
$\kappa v_{end}$, the lead of the segment. Only the extrusion ratio of the segment changes, by
$-\Delta/D$. Its rate (the target velocity of the motor) is

$$
\dot p = \left(u_e n_e - \frac{\Delta}{D}\right) v + \kappa\, a
$$

## 4. Implementation in printhor

`motion/pressure_advance.rs` (`PressureAdvance`: the state carried by the stepper task;
`ExtruderAdvance`: the generator of one segment).

### 4.1 Step generation

The step generator works in windows (micro-segments): at the end of each window, at path position
$s$ with speed $v$ and acceleration $a$ (from the motion profile):

1. the motor goes to the step closest to the target, $m = \lfloor p(s, v) + ½ \rfloor$; the steps of
   the window are $m - m_{prev}$, **in either direction** (the extruder retracts while braking when
   $\kappa |a| > u_e n_e v$);
2. the step rate ramps between the target rates $\dot p$ at the start and at the end of the window
   (second-order DDA), offset so that the window covers exactly its steps. A window whose target
   reverses inside it moves at a uniform rate. The step counts are exact either way (the step
   executor never drops steps).

The extruder of an advanced segment is moved by this generator; the other axes by the regular
interpolator.

### 4.2 Exactness: what is carried

What the motor is ahead of the nominal extruder position, in steps, is carried from segment to
segment:

$$
L' = m_{end} + L_{start} - u_e n_e s_{end}
$$

the lead $\kappa v_{end}$ plus the rounding residue (within $\pm½$ step). No step is ever lost or
added: when the lead is released the motor is at the nominal position, to the closest step. The
segments that neither are advanced nor carry a lead are moved by the regular interpolator, as
without pressure advance, and their rounding residue is carried the same way (see the stepper task).

### 4.3 Stops

When the motion stops (the queue runs empty, a dwell `G4`/`M400`, a homing) the pressure is relieved:
an extruder-only window of one micro-segment period moves the motor back by the lead,
$\lfloor L + ½ \rfloor$ steps. Not on an abort (`M410`) nor with the steppers parked: the lead is then
kept, and spread along the next move.

### 4.4 `M900`

```
M900 [K<seconds>] [T0]
```

`K` sets the factor (0 to 10; 0, the default, disables it; overridable at build time with
`DEFAULT_PRESSURE_ADVANCE_K`). Without `K` it reports `Advance K=<k>`. Only the extruder 0 (`T` other
than 0: `?T value out of range.`). A new `K` is applied from the next move executed: the change of
lead is spread along it as in §3. Stored by `M500`, restored by `M501`, reset by `M502`, reported by
`M503` (`M900 K<k>`).

### 4.5 Tuning

The usual calibration prints apply (Marlin's K-factor pattern, Klipper's tower): print lines (or a
tower) at alternating slow/fast speeds with increasing $K$ and keep the value whose corners and line
ends are sharpest. Typical values: direct drive 0.02-0.1 s, Bowden 0.3-1 s. A `K` too large makes the
line starts bulge and the ends thin, and the extruder works harder (it reverses at every braking).

## 5. Verification

* `motion/pressure_advance.rs` tests: only the extrusions are advanced; every window emits exactly
  its steps; the motor tracks the nominal position plus the lead within ½ step and the step instants
  follow the target within a tick; the extruder retracts while braking (more with larger $K$) and
  ends at its nominal position; the lead is carried through junctions and spread along a travel; a
  change of extrusion ratio reaches the lead of the new ratio; the release at a stop; an interrupted
  segment keeps its lead consistent.
* `micro_segment_interpolation.rs`: the rounding residue of the segments moved by the interpolator is
  carried across them (segments of a fraction of a step add up to their total).
* Integration scenarios (native simulator): T38 (`M900`, `M503`, `M502`, `M500`/`M501`; the same
  extrusions with and without $K$: 286 backward steps with $K$ = 0.2, none without, the same net
  steps and the same path of the other axes) and T40 (the extruder steps of a run of tiny moves and
  of an arc add up to the nominal extrusion).

## References

1. A. Bellini, S. Güçeri and M. Bertoldi, "Liquefier dynamics in fused deposition," *Journal of
   Manufacturing Science and Engineering*, vol. 126, no. 2, pp. 237-246, 2004.
2. B. N. Turner, R. Strong and S. A. Gold, "A review of melt extrusion additive manufacturing
   processes: I. Process design and modeling," *Rapid Prototyping Journal*, vol. 20, no. 3,
   pp. 192-204, 2014.
3. S. A. Tronvoll, S. Popp, C. W. Elverum and T. Welo, "Investigating pressure advance algorithms for
   filament-based melt extrusion additive manufacturing: theory, practice and simulations," *Rapid
   Prototyping Journal*, vol. 25, no. 5, pp. 830-839, 2019.
4. Klipper documentation, "Pressure advance" and "Kinematics: pressure advance",
   https://www.klipper3d.org/Pressure_Advance.html, https://www.klipper3d.org/Kinematics.html.
5. Marlin firmware documentation, "Linear Advance" and "M900 - Linear Advance Factor",
   https://marlinfw.org/docs/features/lin_advance.html, https://marlinfw.org/docs/gcode/M900.html.
