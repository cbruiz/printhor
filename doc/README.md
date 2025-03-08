![Minimum Rust: 1.85](https://img.shields.io/badge/Minimum%20Rust%20Version-1.85-green.svg)
[![crates.io](https://img.shields.io/crates/v/prinThor.svg)](https://crates.io/crates/prinThor)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# System architecture low level detail

[System architecture](system_architecture.md)

# For users

[User guide](user_guide.md)

# For makers

# For developers

[Developers guide](developers_guide.md)

# The features core

TBD

## Physics and mathematics

There are two major steps in the kinematics:
1. Motion plan computation. A Double S-Shape motion profile with 7 segments is calculated constraining to expected constraints (mean velocity, acceleration and jerk) and queued for execution. The queue size is configurable at compile time.
2. Motion plan execution. A dedicated high priority task, dequeues each planned move, interpolates the plan at fixed frequency (the higher frequency the finest)
The interpolation algorithm is tolerant to unexpected short delays that can happen. A custom experimentation to adjust the interpolation frequency is 

In order to facilitate the understanding and decouple the reponsabilities as well, each movement is decomposed as:
* Unitary director vector
* Distance module

So plan computation is related to the distance module only, which is interpolated as micro-segments and the axis advance is 
computed by multiplying unitary director vector by the Delta(t) 

# The thermal sensor

[[See Thermal Sensors](thermal_sensors.md)]


## Movement queue

TBD
