![Minimum Rust: 1.99](https://img.shields.io/badge/Minimum%20Rust%20Version-1.99-green.svg)
[![crates.io](https://img.shields.io/crates/v/prinThor.svg)](https://crates.io/crates/prinThor)
![CI](https://github.com/cbruiz/printhor/actions/workflows/rust.yml/badge.svg)
[![Coverage Status](https://coveralls.io/repos/github/cbruiz/printhor/badge.svg?branch=main)](https://coveralls.io/github/cbruiz/printhor?branch=main)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Discord Shield](https://discordapp.com/api/guilds/1169965662618259456/widget.png?style=shield)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer and robotics firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# Overview

Printhor is a generic and hardware-agnostic (firmware) framework for FDM printers, CNC machines, engravers and robots, implemented in Rust.

Basically, it's a GCode interpreter capable of processing a stream of GCodes (position and control orders), scheduling them, computing a motion profile on the fly and finally performing the hardware level instructions (step pulses, PWM signals, ... or even data frames broadcast to a dedicated breakout board) in real time.

There are many production-grade firmwares in the community, like grbl, Marlin, RepRapFirmware, etc. Each one has its own approach and guidelines.
This one aims to provide a research environment, not strictly meant for production, but a reliable platform with the following goals:
* Robustness.
* Numerical stability.
* Efficient use of resources and close-to-preemptive multitasking.
  * Leverage async multitasking.
  * Keep busy waits to a minimum.
  * Leverage DMA transfers as much as possible.
  * Leverage the FPU when present.
* Simplicity.
* Clarity and readability.

This means that the main short-term goal is not to develop a production firmware for end users, but to provide an environment in which anyone can test and experiment with any concrete approach, to feed the community with good quality, state-of-the-art or innovative feature increments.

## Documentation (work in progress)

[Documentation and guides](doc/README.md)

## Features
* "Clean" hardware abstraction.
* Vector geometry and linear algebra for kinematics
  * Smooth acceleration and jerk limited motion plan leveraging "Trajectory with Double S Velocity Profile" [1]. Briefly explained at [Plan implementation](printhor/src/bin/motion/motion_profile_traits.rs) and visually explained in [Printhor motion plan. A simplified overview of the velocity integration](https://www.geogebra.org/m/hwpnmhcu) (GeoGebra).
    
    "[1] Biagiotti, L., Melchiorri, C.: Trajectory Planning for Automatic Machines and Robots. Springer, Heidelberg (2008). [DOI:10.1007/978-3-540-85629-0](https://doi.org/10.1007/978-3-540-85629-0)"
  * High precision and deterministic kinematics and computations with configurable resolution. Briefly explained at [Stepper Task](printhor/src/bin/tasks/task_stepper.rs).
  * Simple and efficient cornering algorithm based on pure linear algebra. Foundation implicitly explained at [Printhor naïve cornering algorithm](https://www.geogebra.org/m/ft8svrwd) (GeoGebra).
* Precise thermal control plan with PID. Briefly explained at [Temperature Task](printhor/src/bin/tasks/task_temperature.rs).
* Simple, secure and efficient resource and peripherals sharing.
* Clean and simple async tasks coordination/intercommunication with event based primitives.
* Highly customizable behavior.
* Low-level action distribution leveraging dedicated breakout boards.
* Wide GCode standard coverage.
* Native simulation for development and benchmarking.
* Many others coming.

## Overall status

| Feature | Status | Notes |
|---|---|---|
| Native simulation | Functional | Mocked peripherals, stdin prompt, integration tests |
| I/O (serial, USB, SD card) | Functional | Host line-number protocol (`N`/`M110`); SD listing and print jobs |
| GCode parsing | Functional | See [GCode compliance](#gcode-compliance) for the codes actually supported |
| State and logic | Testing | Event bus, print job control (pause/resume/abort), alarms (`M112`/`M999`) |
| Motion planner | Functional | Double S (jerk-limited) profile, look-ahead chaining, quick stop (`M410`), soft end-stops (`M211`). See [doc/MOTION_PROFILE.md](doc/MOTION_PROFILE.md) |
| Feed rate / flow overrides | Testing | `M220` (with `B`/`R` backup/restore) and `M221`, Marlin semantics; not applied to the firmware retraction |
| Arcs (`G2`/`G3`) and Bézier splines (`G5`) | Testing | `with-arcs` / `with-beziers` features, segmented into linear moves |
| Kinematics | Testing | Cartesian, Core-XY, anthropomorphic; Delta in Draft |
| Homing | Testing | Abortable with `M410`/`M112`; home offsets (`M206`) |
| Thermal control | Testing | PID, autotune (`M303`, see [doc/PID_AUTOTUNE.md](doc/PID_AUTOTUNE.md)), thermal runaway / sensor fault protection |
| Firmware retraction | Testing | `G10`/`G11`, `M207`/`M208` |
| Input shaping | Testing | `with-input-shaping` feature (off by default; no code without it), `M593`: ZV, EI, MZV or ZVD shapers on X and Y, step-exact, the shaped motion completed at the stops. `s_plot` shows its effect (residual ringing of a modeled frame). See [doc/INPUT_SHAPING.md](doc/INPUT_SHAPING.md) |
| Pressure advance | Testing | `with-pressure-advance` feature (off by default), `M900 K`: the extruder leads by `K` times its speed, junction mismatches spread along the move, lead released at the stops; step-exact. See [doc/PRESSURE_ADVANCE.md](doc/PRESSURE_ADVANCE.md) |
| Settings persistence | Testing | `M500`/`M501` (`with-eeprom`), `M502`/`M503` |
| Probing | Testing | `G30` single probe, probe offset (`M851`), deploy/stow (`M401`/`M402`, servo `M280`), sled dock/undock (`G31`/`G32`) |
| Bed leveling | TODO | `G29*` accepted without effect; `with-mesh-leveling` has no implementation yet |
| Laser/CNC | Testing | `with-laser` (laser PWM on the SKR Mini E3, the Nucleo CNC hat, the RP2040 and the simulator; not yet on the MKS Robin Nano). `M3` constant power, off during the rapids (`G0`) and the homing (as GRBL's laser mode); `M4` dynamic power, scaled by the speed relative to the programmed `F` (constant energy per unit of length); `M5` off. Queued with the motion and switched by the step ISR exactly at the junction, without breaking the chaining; off on `M410`/`M112`, never on in dry run. A spindle shares the same PWM (no direction nor RPM control); `M7`/`M8`/`M9` keep the coolant state only (no coolant output yet). With input shaping, the tool actions follow the commanded motion (the shaped axes lag it a few ms) |
| Display (serial TFT) | Testing | BTT TFT (e.g. TFT35) on the display UART with `marlin-compat`: `M115` capabilities, `M503 S0`, `M211`, `M220`/`M221`, `M117` as a status notification. Directly driven LCDs: TODO |
| Instrumentation | Testing | Motion execution metrics (`M800`, `motion-metrics`), MCU operation benchmark (`M801`, `mcu-bench`) |

Assuming the following maturity convention:
 TODO -> Draft -> Incubation -> Testing -> Functional -> Done

# Help wanted
If you are interested in this project and want to collaborate, you are welcome.
A Discord server has been created for informal discussions. Otherwise, GitHub Issues and Pull Requests are preferred.  
[![Discord Banner 4](https://discordapp.com/api/guilds/1169965662618259456/widget.png?style=banner4)](https://discord.gg/VSag6T4KS6)

# Compiling

## Fast track

<a href="https://idx.google.com/import?url=https%3A%2F%2Fgithub.com%2Fcbruiz%2Fprinthor">
  <img
    height="32"
    alt="Open in IDX"
    src="https://cdn.idx.dev/btn/open_dark_32.svg">
</a>

## Step by step

### Checkout

```shell
git clone https://github.com/cbruiz/printhor
cd printhor
git submodule update --init --recursive
```

### Build

The minimal toolset required to build and run is:
* Rust, to compile.
* cargo-binutils, to produce the binary image that you can flash via SD card as usual.
* __[Optionally]__ probe-rs, if you are willing to use an SWD/J-Link debug probe (https://probe.rs)
* __[Optionally]__ cargo-bloat and `cargo size` (from cargo-binutils) are great to analyze the code size.
* __[Optionally]__ A Rust IDE, like [Jetbrains IDE suite](https://www.jetbrains.com/) (IntelliJ, CLion or RustRover) (recommended), Visual Studio Code (also fine), or others

### Prerequisites: Rust and toolchain

This crate requires **Rust >= 1.99**, on the stable channel (selected by `rust-toolchain.toml`).

For official guide, please see https://www.rust-lang.org/tools/install

However, if your OS is Unix-based:
```shell
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup update
```

For microcontrollers (STM32 and RP2040), the target toolchains and cargo-binutils are also needed:

```shell
rustup target add thumbv7em-none-eabi
rustup target add thumbv7em-none-eabihf
rustup target add thumbv6m-none-eabi
rustup component add llvm-tools
cargo install cargo-binutils
```

Optionally, for flashing and debugging bare-metal microcontrollers with a debug probe (the runner configured in `.cargo/config.toml`):

```shell
cargo install probe-rs-tools --locked
```

## Native simulator backend

The framework with a set of mocked peripherals (most of them without any logic).
It provides a command-line GCode prompt on the standard input.

__Note__: The native simulator enables the `with-sd-card` feature by default, which needs the SD card image `data/sdcard.img`.

```shell
RUST_LOG=info cargo run --bin printhor
```

### Integration tests

The native backend has a special feature, `integration-test`, which runs "some kind of" integration tests. It is still
immature.

```shell
RUST_LOG=info cargo run --features integration-test --bin printhor
```

Testing with a GCode sender through socat:

```shell
RUST_LOG=info cargo build --bin printhor
socat pty,link=printhor,rawer EXEC:target/debug/printhor,pty,rawer
```

## Supported boards and specific instructions

| Board                                                                                             | Status            |
|---------------------------------------------------------------------------------------------------|-------------------|
| [SKR Mini E3 V3.0](hwi-boards/printhor-hwi_skr_mini_e3/README.md)                                 | Almost Functional |
| [MKS Robin Nano v3.1](hwi-boards/printhor-hwi_mks_robin_nano/README.md)                           | Almost Functional |
| [Nucleo-f410rb + Arduino CNC Hat v3](hwi-boards/printhor-hwi_nucleo_64_arduino_cnc_hat/README.md) | Almost Functional |
| [Nucleo-l476rg + Arduino CNC Hat v3](hwi-boards/printhor-hwi_nucleo_64_arduino_cnc_hat/README.md) | Almost Functional |
| [Raspberry Pi RP2040](hwi-boards/printhor-hwi_rp_2040/README.md)                                  | Draft             |
| [ESP32-S3-WROOM](hwi-boards/printhor-hwi_esp32/README.md)                                         | Naïve Skeleton    |


## Extra utilities

A simple stand-alone std binary to experiment with motion plan and see what it does (kind of playground):

```shell
cargo run --profile release --bin s_plot --features s-plot-bin
```

### Example output with the current plotting style approach:
As Image:
![alt text](printhor/img/motion_plan.png "Motion Plan")

As Vector:

<object data="printhor/img/motion_plan.pdf" type="application/pdf" width="700px" height="700px">
  <embed src="printhor/img/motion_plan.pdf">
    <p style="text-align: center;">This browser does not support PDFs. Please <a href="printhor/img/motion_plan.pdf">Download the PDF</a> to view it</p>
</object>

There are (currently) two plots:
* Position:
  * A continuous blue curve for the analytic __ideal__ position estimation.
  * A stepped gray curve for the __real__ (discrete) position. 
* Velocity:
  * A continuous green curve for the online derivation of __ideal__ position datapoints.
  * A sloped gray polyline for the online derivation of __real__ (discrete) position datapoints in the sampling interval.

### Example output with the deprecated plotting style:
![alt text](printhor/img/motion_plan_old.png "Motion Plan")

This plot was self-explanatory, but it has been deprecated in favor of the previous one.
We are keeping it because it is clear and useful for a high level understanding.

# TODO
- [ ] Code productivization. Mostly based on https://dl.acm.org/doi/pdf/10.1145/3519941.3535075 and https://jamesmunns.com/blog/fmt-unreasonably-expensive/:
  - [ ] Code size reduction (ongoing: `release-opt` profile, no heap formatting in the reports, see [Controlling the code size](#controlling-the-code-size)).
  - Remove unwrap/panic calls.
    - [x] In the firmware core (`printhor`, `printhor-hwa-common`, `printhor-hwa-utils`): none left outside the tests.
    - [ ] In the board crates: the hardware initialization (`expect`/`panic!` when a peripheral can not be set up) and the `todo!()` of the boilerplate and ESP32 skeletons.
  - Remove unsafe code (a little bit, but there it is).
    - [x] In the firmware core: only the unavoidable ones are left (the `udf` of the panic handler, the `no_mangle` export of the step ISR and the FPU `vsqrt` instruction).
    - [ ] In the board crates: interrupt handlers, register access and linker symbols.
  - [ ] Remove trivial/redundant computation __but not compromising readability__.
- [ ] Display: directly driven LCDs (serial TFTs work in Marlin mode)
- [ ] I/O control
  - [ ] xonxoff
- [ ] exFAT support would be great (FAT with long file names is supported).

# Customization

For a single board, the high-level features (hotend, hotbed, fan, SD card, ...) can be enabled/disabled by cargo feature selection or by directly editing `printhor/Cargo.toml`.
In order to change pins, writing/adapting some code is required, as of now. No configuration file is planned, at least in the short term.

Because of that limitation (strict typing makes it hard to tackle in Rust), a clean code organization is crucial, so that slight customizations are straightforward to make by editing the code.

# Architecture

Printhor is composed of the following architectural blocks:
* embassy-rs, as the hardware and async foundation https://github.com/embassy-rs/embassy
* async-gcode, as the core of the GCode interpretation: a fork (https://github.com/cbruiz/async-gcode, the `libs/async-gcode` submodule) of https://github.com/ithinuel/async-gcode
* printhor-hwa-common (within the project), as the hardware abstraction layer contract and common machinery
* printhor-hwa-utils (within the project), with the mutex strategies and other utilities shared by the HWA and HWI crates
* A set of crates (within the project), one for each hardware/board. Currently:
  * printhor-hwi_native : The native simulator.
  * printhor-hwi_skr_mini_e3 : A BigTreeTech SKR Mini E3 V3.0 (See [datasheets/SKR_MINI_E3-V3.0](datasheets/SKR_MINI_E3-V3.0)).
  * printhor-hwi_mks_robin_nano : A MKS Robin Nano V3.1 (See [datasheets/MKS-ROBIN-NANO-V3.1](datasheets/MKS-ROBIN-NANO-V3.1)).
  * printhor-hwi_nucleo_64_arduino_cnc_hat : A Nucleo-64 development board (currently L476RG or F410RB) with Arduino CNC Shield v3.x (See [datasheets/NUCLEO-L476RG](datasheets/NUCLEO-L476RG) and [datasheets/NUCLEO-F410RB](datasheets/NUCLEO-F410RB)).
  * printhor-hwi_rp_2040 : A Raspberry Pi RP2040 board (See [datasheets/RP2040](datasheets/RP2040)).
  * printhor-hwi_esp32 : An ESP32-S3-WROOM board (skeleton).

In general, traits are intentionally avoided in the HWI layer when not strictly required, in favor of defining a more decoupled and easy to evolve interface based on:
* type aliases
* module exports

# Similar, related software and shout-outs

* https://embassy.dev/ Rust Embassy; The next-generation framework for embedded applications. The most important and core framework, forming the pillars of this project.
* https://github.com/ithinuel/async-gcode This powerful GCode implementation is a fundamental piece of this project.
* https://github.com/nviennot/turbo-resin The same idea as Printhor, but focused on resin printers. The project that inspired and motivated this one.
* https://github.com/marcbone/s_curve Much appreciated know-how. The current motion profile is based on this work, but it was heavily reinterpreted to make this project work. The book referenced by the author of s_curve is a key reading for the goals of the motion plan. Nevertheless, our implementation is still too immature/unproven to ask for a merge.

# GCode compliance

This list reflects the codes the firmware actually parses and executes (`printhor/src/bin/processing`).
Codes are only available when the listed cargo feature is enabled.

Status:
* DONE: Implemented
* DONE*: Implemented with limitations
* Accepted: Acknowledged with `ok`. Only the default/only supported mode exists
* Accepted, no effect: Acknowledged with `ok` for compatibility with slicers, but does nothing yet

Any other code is rejected with an error (not implemented), also when it belongs to a feature that is
not enabled (e.g. `M109` without `with-hot-end`): the command never waits for something that can not
happen, so every feature combination keeps progressing. In a print job, such lines are reported and
skipped. A real fault of a supported command (e.g. a heater safeguard) raises the alarm state instead,
recoverable with `M999`.

| Code | Feature | Description | Status |
|---|---|---|---|
| G | * | List the supported G-codes | DONE |
| G0 | `with-motion` | Rapid move (`X Y Z F`) | DONE |
| G1 | `with-motion` | Linear move (`X Y Z E F`) | DONE |
| G2 / G3 | `with-arcs` | Arc move CW / CCW in the XY plane (`I J` center offset or `R` radius; helical with `Z`) | DONE |
| G4 | `with-motion` | Dwell (`P` ms / `S` s) | DONE |
| G5 | `with-beziers` | Cubic Bézier spline in the XY plane (`I J` = 1st control point relative to the start, `P Q` = 2nd control point relative to the end). Both pairs are required | DONE |
| G10 / G11 | `with-motion` | Firmware retract / recover (needs `with-e-axis`; otherwise accepted without effect) | DONE |
| G17 | `with-motion` | XY plane selection (the only plane supported) | Accepted |
| G21 | `with-motion` | Millimeter units (the only unit supported) | Accepted |
| G28 | `with-motion` | Homing. Z0 is the plane where the Z end-stop (or probe) triggers, the bed; the homing ends 10 mm above it, so `M114` reports `Z:10` (as Marlin's `Z_AFTER_HOMING`), with the probe over the bed center (Marlin's `Z_SAFE_HOMING`). Some axes only (Marlin's semantics: `G28 X Y`, the words are flags): Z is raised first (to 10 mm above the bed when known, never lower), the given X/Y axes are homed and left at their end-stops; `G28 Z` (X and Y known, otherwise `Home XY first`) or `G28 X Y Z` end as the full homing. Moves need every axis homed. Home offsets (`M206`) apply. Abortable with `M410`/`M112` | DONE |
| G29 / G29.1 / G29.2 | `with-motion` | Bed leveling | Accepted, no effect |
| G30 | `with-probe + with-z-axis` | Single Z probe at the current position, or at `X Y` (absolute, where the probe goes: the nozzle goes there minus the probe offset). Waits for the queued moves, requires homing. As the Z sample of the homing: raises Z 10 mm (before the travel and the probe deploy), deploys the probe, lowers Z until it triggers (fast, then the fine approach; down to the homing trigger plane plus 5 mm), stows it and leaves Z 10 mm above where it triggered. Answers `Bed X: <x> Y: <y> Z: <z>` (nozzle Z at the trigger plus the probe Z offset, in the M114 coordinates: a flat bed probes at `Z: 0.00`). Not triggering raises the alarm (`Error:Probing failed`, recoverable with `M999` and `G28`) | DONE |
| G31 / G32 | `with-probe` | Dock / undock the probe sled | DONE |
| G80 | `with-motion` | Cancel canned cycle | Accepted |
| G90 / G91 | `with-motion` | Absolute / relative positioning | DONE |
| G92 | `with-motion` | Set position. Also declares the position as known (manual homing) | DONE |
| G94 | `with-motion` | Units per minute feed rate mode (the only mode supported) | Accepted |
| M | * | List the supported M-codes | DONE |
| M2 | `with-sd-card + with-print-job` | Program end: ends the print job | DONE |
| M3 / M4 / M5 | `with-laser` | Laser (or spindle, same PWM) on / off, `S` 0-255 (full power by default). `M3`: constant power, off during `G0` and homing; `M4`: dynamic power (scaled by the speed relative to `F`). Queued with the motion | DONE |
| M7 / M8 / M9 | * | Mist / flood coolant on, coolant off. Queued with the motion | Partial: state only (no coolant output) |
| M18 | `with-motion` | Disable steppers (alias of `M84`) | DONE |
| M20 | `with-sd-card` | List SD files: `<path> <size>`, those of the subdirectories with their path (4 levels; hidden ones excluded). `L`: also the long path (each entry by its long name, or its short one), Marlin's `M20 L`. `F"<dir>"` lists a directory | DONE |
| M21 | `with-sd-card` | Init SD card | DONE |
| M22 | `with-sd-card` | Release the SD card. Refused while it is in use (print job, file being written) | DONE |
| M23 | `with-sd-card + with-print-job` | Select SD file, named by short or long names (case-insensitive; also every path given to the SD commands). Answers `File opened: <name> Size: <bytes>` | DONE |
| M24 | `with-sd-card + with-print-job` | Start / resume SD print. Error without a selected file; `ok` (no effect) when already printing | DONE |
| M25 | `with-sd-card + with-print-job` | Pause SD print (effective once the planned moves are done). `ok` (no effect) when already paused | DONE |
| M28 / M29 | `with-sd-card` | Write the next lines of the channel to a file (`M28 <name>`) until `M29`. Not allowed while printing | DONE |
| M33 | `with-sd-card` | The long path of a file or directory (`/<long name>/...`, short names where there is no long one) | DONE |
| M27 | `with-sd-card + with-print-job` | SD print status: `SD printing byte <pos>/<size>` or `Not SD printing`. `S<secs>` auto-reports periodically (`S0` off) | DONE* |
| M37 | * | Dry run mode (`S1` on / `S0` off) | DONE |
| M73 | * | Print progress: `P` percentage, `R` remaining minutes. Without words, reports it (the file percentage when the slicer gave none). Stored with `with-print-job` | DONE |
| M79 | * | Software reset | DONE |
| M80 / M81 | `with-ps-on` | ATX power on / off | DONE |
| M82 / M83 | `with-motion` | Absolute / relative extruder mode | DONE |
| M84 | `with-motion` | Disable steppers | DONE |
| M92 | `with-motion` | Set steps per unit | DONE |
| M100 | * | Report memory usage | DONE |
| M104 / M109 | `with-hot-end` | Set / wait for hot-end temperature | DONE |
| M105 | * | Report temperatures (two decimals, built without heap) | DONE |
| M106 / M107 | `with-fan-layer` | Fan on (`S` 0-255, full power by default; `P0` the layer fan, `P1` the extra fan with `with-fan-extra-1`) / layer fan off. Queued with the motion | DONE |
| M110 | * | Set current line number (host protocol) | DONE |
| M112 | * | Emergency stop (heaters off, motion aborted, alarm until `M999`) | DONE |
| M114 | `with-motion` | Report the position of the enabled axes (then `E`) and their step counts | DONE |
| M115 | * | Firmware name (`PrinThor`) and compiled version. With `marlin-compat`, in the Marlin protocol format, plus the `Cap:` list, derived from the enabled features | DONE |
| M117 | * | Display message, sent to the display channel: as a Marlin host notification (`//action:notification`) with `marlin-compat`, as `echo:` otherwise. Empty messages send nothing | DONE |
| M118 | * | Echo message to the host | DONE |
| M119 | `with-motion` | Report endstop status (disabled axes report 0) | DONE |
| M140 / M190 | `with-hot-bed` | Set / wait for bed temperature | DONE |
| M280 | `with-probe` | Servo position (`P0 S<angle>`, 0-180). Without `S`, reports the PWM state | DONE |
| M200 | `with-motion + with-e-axis` | Volumetric extrusion (Marlin's), off by default: `D<diameter>` sets the filament diameter and enables it (`D0` disables it), `S1`/`S0` enables/disables it, `L<mm³/s>` limits the volumetric flow of the extruding moves (0-20, `0` unlimited, also when disabled). Enabled, E words are volumes. Without words, reports `M200 S<0/1> D<diameter> L<limit>`. In `M503`/`M500`, reset by `M502`. `M115` reports `Cap:VOLUMETRIC` as enabled or not | DONE |
| M201 | `with-motion` | Max acceleration per axis | DONE |
| M203 | `with-motion` | Max feed rate per axis | DONE |
| M204 | `with-motion` | Starting acceleration (`P` print, `T` travel) | DONE |
| M205 | * | Jerk settings: accepted but ignored (Marlin semantics differ from the S-curve jerk) | Accepted, no effect |
| M206 | `with-motion` | Home offsets (applied on the next homing, as Marlin) | DONE |
| M207 / M208 | `with-motion + with-e-axis` | Firmware retraction / recover settings | DONE |
| M211 | * | Soft end-stops: `S1`/`S0` enables/disables clipping the moves to the travel of the axes (on by default). Reports the state and the bounds in the Marlin format | DONE |
| M220 | `with-motion` | Feed rate percentage (`S`, 10-255 %), applied to the moves planned from then on (G0/G1, arcs, splines). `B` backs up the current one, `R` restores it (Marlin order: `R`, then `B`, then `S`). Without words, reports `FR:<n>%` | DONE |
| M221 | `with-motion` | Flow percentage of the extruder (`S`, 10-255 %), applied to the E displacement of the moves planned from then on. Only `T0` (single extruder). Without `S`, reports `echo:E0 Flow: <n>%` | DONE |
| M301 / M304 | `with-hot-end / with-hot-bed` | Hot-end / bed PID constants | DONE |
| M302 | `with-hot-end + with-e-axis` | Cold extrusion prevention | DONE |
| M303 | `with-hot-end or with-hot-bed` | PID autotune by relay feedback (`E0` hot-end, `E-1` bed; `S` target, default 150/70; `C` min cycles, 3-20, default 5; `U1` applies the result). Runs in the temperature control loop: the host gets `ok` at its end and can poll temperatures meanwhile. Bias balancing, Fourier estimate of the ultimate gain, finishes once three cycles agree; classic Ziegler–Nichols (hot-end) / "no overshoot" (bed), as Marlin. Reported as Marlin (`bias/d/min/max`, `Ku/Tu`, `Kp/Ki/Kd` per cycle, temperatures every 2 s, `#define DEFAULT_Kp ...`) | DONE |
| M400 | `with-motion` | Wait for the queued moves to finish | DONE |
| M401 / M402 | `with-probe` | Deploy / stow the probe | DONE |
| M410 | `with-motion` | Quick stop: abort the current move and discard the queue | DONE |
| M500 / M501 | `with-eeprom` | Store / restore settings | DONE |
| M502 | * | Restore default settings | DONE |
| M503 | * | Report current settings, one whole line per setting (`S0`: settings only, as sent by the TFTs at boot) | DONE |
| M524 | `with-sd-card` | Abort SD print (interrupts the waits of the job, e.g. heating). `ok` (no effect) when nothing is printing | DONE |
| M800 | `with-motion` | Motion execution metrics (needs `motion-metrics`): ISR load, segments, planner and queue stats. `S0` resets them, `J<mm/s³>` sets the S-curve jerk of the motion axes | DONE |
| M801 | `with-motion` | MCU operation cost benchmark (needs `mcu-bench`) | DONE |
| M851 | `with-motion` | Probe offset (`X Y Z`). Without words, reports `Probe Offset X Y Z` | DONE |
| M862.1 / M862.3 | `with-motion` | Prusa printer checks | Accepted, no effect |
| M593 | `with-motion + with-input-shaping` | Input shaping (Marlin's, plus the type): `F<Hz>` frequency (`0` disables it, else 20 Hz or above), `D<ratio>` damping (0-0.5), `T<type>` 0 ZV, 1 EI, 2 MZV, 3 ZVD, of the axes `X`/`Y` given (both without them). Defaults: ZVD, 40.01 Hz, 0.15 (tuned with `s_plot`; Marlin's: ZV, 40 Hz, 0.15). Without values, reports `M593 X F<f> D<d> T<t>` per axis. Applied from the next stop. In `M503`/`M500`, reset by `M502` | DONE |
| M900 | `with-motion` | Linear advance factor | Accepted, no effect (without `with-pressure-advance`) |
| M900 | `with-motion + with-pressure-advance` | Pressure advance (Marlin's linear advance): `K<seconds>` sets the factor (0-10, `0` disables it, the default), `T0` only. Without `K`, reports `Advance K=<k>`. Applied to the extruding moves (E forward along with other axes) from the next move executed. In `M503`/`M500`, reset by `M502` | DONE |
| M907 | `with-motion` | Motor current | Accepted, no effect |
| M999 | * | Clear the alarm state (homing required afterward) | DONE |

With `grbl-compat`, the GRBL status query (`?`) and `$` commands are also accepted.

Limitations:
* `M27 S` auto-reports are sent between job commands: while a command waits (full motion queue,
  heating) they are delayed. `M27 C` (current file name) is not implemented.
* `M220`/`M221` do not change the moves already queued, nor the firmware retraction (`G10`/`G11`),
  whose length and speeds are absolute (as Marlin).
* Without a step actuator (`with-motion-stepper` or `with-motion-broadcast`), moves are planned and
  tracked but executed virtually (as in dry run), so jobs keep progressing.

### 3D print GCode sample

    M73 P0 R87
    M73 Q0 S87
    M201 X1000 Y1000 Z200 E5000 ; sets maximum accelerations, mm/sec^2
    M203 X200 Y200 Z12 E120 ; sets maximum feedrates, mm / sec
    M204 P1250 R1250 T1500 ; sets acceleration (P, T) and retract acceleration (R), mm/sec^2
    M205 X8.00 Y8.00 Z0.40 E4.50 ; sets the jerk limits, mm/sec
    M205 S0 T0 ; sets the minimum extruding and travel feed rate, mm/sec
    M107
    ;TYPE:Custom
    M862.3 P "MK3" ; printer model check
    M862.1 P0.4 ; nozzle diameter check
    G90 ; use absolute coordinates
    M83 ; extruder relative mode
    M80 ; Power on
    G28 W ; home all without mesh bed level
    G80 ; mesh bed leveling
    G28 X Y; Park
    M104 S235 ; set extruder temp
    M140 S60 ; set bed temp
    M190 S60 ; wait for bed temp
    M109 S235 ; wait for extruder temp
    
    G1 Z0.2 F720
    G1 Y-3 F1000 ; go outside print area
    G92 E0
    G1 X60 E9 F1000 ; intro line
    G1 X100 E12.5 F1000 ; intro line
    
    G92 E0
    M221 S95
    
    ; Don't change E values below. Excessive value can damage the printer.
    M907 E430 ; set extruder motor current
    G21 ; set units to millimeters
    G90 ; use absolute coordinates
    M83 ; use relative distances for extrusion
    ; Filament-specific start gcode
    M900 K0.2
    M107
    ;LAYER_CHANGE
    ;Z:0.2
    ;HEIGHT:0.2
    ;BEFORE_LAYER_CHANGE
    G92 E0.0
    ;0.2
    
    
    G1 E-3.1 F3000
    G1 Z.4 F720
    ;AFTER_LAYER_CHANGE
    ;0.2
    G1 X76.782 Y74.538 F13200
    G1 Z.2 F720
    G1 E3.1 F3000
    M204 P800
    ;TYPE:Skirt/Brim
    ;WIDTH:0.42
    G1 F1200
    G1 X77.609 Y73.879 E.03317
    G1 X90.788 Y67.532 E.45862
    G1 X91.819 Y67.297 E.03317

### Laser GCode sample
    G21
    G90
    M206 X-45. Y-10. Z-20.; Laser offset
    G28; Homing
    G1 F3000. Z7.
    M107;svg#svg5 > g#layer89324 > path#rect2575
    G0 X12.6508 Y109.9950
    M4 S255
    G1 X112.6408 Y109.9950 F3000
    G1 X112.6408 Y10.0050 F3000
    G1 X12.6508 Y10.0050 F3000
    G1 X12.6508 Y109.9950 F3000
    M5
    G4

## Controlling the code size

```bash
brew install bloaty
bloaty target/thumbv7em-none-eabihf/release/printhor -d sections,symbols -n 0 --csv | bloaty-metafile > meta.json
```

Open meta.json with https://esbuild.github.io/analyze/ or any other tool

# Appendix

### Maintaining stack up to date

```bash
cargo install cargo-update
cargo-install-update install-update --all
```
