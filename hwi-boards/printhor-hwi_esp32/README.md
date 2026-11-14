![Minimum Rust: 1.99](https://img.shields.io/badge/Minimum%20Rust%20Version-1.99-green.svg)
[![crates.io](https://img.shields.io/crates/v/prinThor.svg)](https://crates.io/crates/prinThor)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Discord Shield](https://discordapp.com/api/guilds/1169965662618259456/widget.png?style=shield)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer and robotics firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# Overview

Doc WIP: The Printhor HWI for the Espressif ESP32-S3-WROOM. A naïve attempt to bring support for the Xtensa
architecture: printhor cannot be built for this board yet, only this crate.

### Toolchain

The Xtensa targets need the `esp` toolchain, installed with espup (Ref: [The Rust on ESP Book](https://docs.espressif.com/projects/rust/book/)).
The `rust-toolchain.toml` of this crate selects it:

```shell
cargo install espup
espup install --toolchain-version 1.99.0.0
source ~/export-esp.sh
cargo build
```
