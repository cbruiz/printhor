![Minimum Rust: 1.79](https://img.shields.io/badge/Minimum%20Rust%20Version-1.79-green.svg)
[![crates.io](https://img.shields.io/crates/v/prinThor.svg)](https://crates.io/crates/prinThor)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Discord Shield](https://discordapp.com/api/guilds/1169965662618259456/widget.png?style=shield)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# Overview

This two boards are quite functional:
* https://biqu.equipment/collections/control-board/products/bigtreetech-skr-mini-e3-v2-0-32-bit-control-board-for-ender-3
* https://biqu.equipment/products/bigtreetech-skr-mini-e3-v2-0-32-bit-control-board-integrated-tmc2209-uart-for-ender-4

### Binary image production (standard with defmt)

The firmware.bin file ready to be uploaded to the SD can be produced with the following commandline:

For SKR Minit E3 V2.0:
```
DEFMT_LOG=info cargo objcopy --release --no-default-features --features skr_mini_e3_v2 --target thumbv7m-none-eabi --bin printhor -- -O binary firmware.bin
```

For SKR Minit E3 V3.0:
```
DEFMT_LOG=info cargo objcopy --release --no-default-features --features skr_mini_e3_v3 --target thumbv6m-none-eabi --bin printhor -- -O binary firmware.bin
```
or

```shell
DEFMT_LOG=info cargo objcopy --release --no-default-features --features skr_mini_e3_v3 --target thumbv6m-none-eabi --bin printhor -- -O binary firmware.bin
```

Firmware size around 196kB as of now with previous settings.

### Minimal-size binary image production

```shell
DEFMT_LOG=off RUST_BACKTRACE=0 cargo objcopy --profile release-opt --no-default-features --features skr_mini_e3_v3 --target thumbv6m-none-eabi --bin printhor -- -O binary firmware.bin
```

Firmware size if 180kB as of now with previous settings.

### Run with JLink/SWD device

DEFMT_LOG=info RUST_BACKTRACE=1 RUSTFLAGS='--cfg board="skr_mini_e3_v3"' cargo run --release --no-default-features --features skr_mini_e3_v3 --target thumbv6m-none-eabi --bin printhor


