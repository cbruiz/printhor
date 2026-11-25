![Minimum Rust: 1.99](https://img.shields.io/badge/Minimum%20Rust%20Version-1.99-green.svg)
[![crates.io](https://img.shields.io/crates/v/prinThor.svg)](https://crates.io/crates/prinThor)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer and robotics firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# System architecture low level detail

[System architecture](system_architecture.md)

# For users

[User guide](user_guide.md)

# For makers

TBD

# For developers

[Developers guide](developers_guide.md)

# The features core

* [Hybrid motion profile](MOTION_PROFILE.md): the double S-curve and the polynomial blend every move
  is executed along, and how they chain.
* [Input shaping](INPUT_SHAPING.md): `M593`, feature `with-input-shaping`.
* [Pressure advance](PRESSURE_ADVANCE.md): `M900`, feature `with-pressure-advance`.
* [PID autotune](PID_AUTOTUNE.md): `M303`, the relay experiment and the Ziegler–Nichols rules.

## Physics and mathematics

There are two major steps in the kinematics:
1. Motion plan computation. A Double S-Shape motion profile with 7 segments is calculated constraining to expected constraints (mean velocity, acceleration and jerk) and queued for execution. The queue size is configurable at compile time.
2. Motion plan execution. A dedicated high priority task dequeues each planned move and interpolates the plan at a fixed frequency (the higher the frequency, the finer the motion).
The interpolation algorithm tolerates short unexpected delays.

In order to ease the understanding and to separate the responsibilities as well, each movement is decomposed as:
* Unit direction vector
* Distance (magnitude)

So the plan computation only deals with the distance, which is interpolated as micro-segments, and the advance of each axis is
computed by multiplying the unit direction vector by Delta(t).

# The thermal sensor

[Thermal sensors](thermal_sensors.md)

# Movement queue

TBD
