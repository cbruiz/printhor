![Minimum Rust: 1.79](https://img.shields.io/badge/Minimum%20Rust%20Version-1.75-green.svg)
[![crates.io](https://img.shields.io/crates/v/prinThor.svg)](https://crates.io/crates/prinThor)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Discord Shield](https://discordapp.com/api/guilds/1169965662618259456/widget.png?style=shield)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# Overview

This board (https://www.makerbase.store/pages/mks-robin-nano-v3-1-intro) is still work in progress

### Binary image production (standard with defmt)

The firmware.bin file ready to be uploaded to the SD can be produced with the following commandline:

```shell
DEFMT_LOG=info cargo objcopy --release --no-default-features --features mks_robin_nano --target thumbv7em-none-eabihf --bin printhor -- -O binary firmware.bin
```

Firmware size if 200kB as of now with previous settings.

### Minimal-size binary image production

```shell
DEFMT_LOG=off RUST_BACKTRACE=0 cargo objcopy --profile release-opt --no-default-features --features mks_robin_nano --target thumbv7em-none-eabihf --bin printhor -- -O binary firmware.bin
```

Firmware size if 164kB as of now with previous settings.

### Run with JLink/SWD device

DEFMT_LOG=info RUST_BACKTRACE=1 RUSTFLAGS='--cfg board="mks_robin_nano"' cargo run --release --no-default-features --features mks_robin_nano --target thumbv7em-none-eabihf --bin printhor
