![Minimum Rust: 1.85](https://img.shields.io/badge/Minimum%20Rust%20Version-1.85-green.svg)
[![crates.io](https://img.shields.io/crates/v/prinThor.svg)](https://crates.io/crates/prinThor)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# The controllers anatomy
Why is that so complex?

\[TLDR;\] Just because!.

In bare-metal, most the peripherals need to be shared and they live forever in nature.
In rust, there is always a lifetime for everything to met the language constraints, so obviously,
in `PrinThor` there is a big bunch of machinery to wrap shared (stateful or stateless) peripherals (a Pin, for instance) or adaptors (a PwmController or Heater).

There are complex Controllers build on top of Controllers. Most meaningful case is the SDCard Controller.

Again, in bare-metal, is quite common that the -physical- SDCard peripheral communicates with the MCU leveraging a SPI Bus, which usually is shared.
So, the construction is like:

```text
StaticAsyncController[SDCardController] <- A clonable and shareable controller which wraps a single reference
    - StaticAsyncController[SDCard] <- A stateful controller (holding open files, etc) 
        - StaticAsyncController[Device] <- The "Device", which can be shared
            - CSPin <- Now yes, the owned instance of the CS Pin
            - StaticAsyncController[SPI] <- Another controller to share same SPI for different controllers
                - SPI <- Now yes, the owned instance

```

So there can be a **single**  `StaticAsyncController[SPI]` constructed by the HWI layer than can be shared by two **diferent** high level controllers like:

* The SDCardController
* Another controller. A DisplayController, for instance

# Architecture

The Work-in-Progress architecture design

![alt text](../design/printhor_motion_high_level_architecture.png "High Level Architecture (motion only as of now)")

# The features core

TBD

## Event bus

## Deferred commands

## State machine


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

## Developers guide

# Troubleshooting
TBD