![Minimum Rust: 1.75](https://img.shields.io/badge/Minimum%20Rust%20Version-1.75-green.svg)
[![crates.io](https://img.shields.io/crates/v/prinThor.svg)](https://crates.io/crates/prinThor)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Discord Shield](https://discordapp.com/api/guilds/1169965662618259456/widget.png?style=shield)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# Overview

There are two base boards supported in this category.
The assumption/requirement is to use any of these generic purpose development board with the Arduino CNC Shield v3 (hat):
![alt text](../../datasheets/NUCLEO-L476RG_CNC_SHIELD_V3/Arduino-CNC-Shield-Pinout-V3.XX.jpeg "Arduino CNC Shield v3")

In these development boards, flash and run can be directly performed with probe-rs just connecting USB as they have a built-in SWD/JTAG interface:

### nucleo-f410rb
Please, note that this board is very limited in terms of flash and memory (48kB SRAM, 128kB flash).
You might not assume that a firwmare not optimized for size (LTO, etc...) will fit in flash.

Note: This target uses flip-link by default, requiring flip-link tool. To change this behavior please check .cargo/config.toml
```shell
cargo install flip-link
```

```shell
DEFMT_LOG=info RUST_BACKTRACE=0 RUSTFLAGS='--cfg board="nucleo64-f410rb"' cargo run --release --no-default-features --features nucleo_64_arduino_cnc_hat,nucleo64-f410rb --target thumbv7em-none-eabihf --bin printhor
```

### nucleo-l476rg
This one is a bit slower but much more RAM and flash. Enough even though with non very optimized firmware and may features

```shell
DEFMT_LOG=info RUST_BACKTRACE=0 RUSTFLAGS='--cfg board="nucleo64-l476rg"' cargo run --release --no-default-features --features nucleo_64_arduino_cnc_hat,nucleo64-l476rg --target thumbv7em-none-eabihf --bin printhor
```

