![Minimum Rust: 1.85](https://img.shields.io/badge/Minimum%20Rust%20Version-1.85-green.svg)
[![crates.io](https://img.shields.io/crates/v/prinThor.svg)](https://crates.io/crates/prinThor)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Discord Shield](https://discordapp.com/api/guilds/1169965662618259456/widget.png?style=shield)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

A naïve intent to bring support for Xtensa architecture

Instructions (Note: requires nightly toolchain and llvm utilery. Ref: [The Rust on ESP Book](https://docs.esp-rs.org/book/))
```shell
cargo install espup
espup install --toolchain-version 1.85.0.0
source ~/export-esp.sh

[...]

```