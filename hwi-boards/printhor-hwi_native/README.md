![Minimum Rust: 1.79](https://img.shields.io/badge/Minimum%20Rust%20Version-1.79-green.svg)
[![crates.io](https://img.shields.io/crates/v/printhor-hwi_native.svg)](https://crates.io/crates/printhor-hwi_native)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Discord Shield](https://discordapp.com/api/guilds/1169965662618259456/widget.png?style=shield)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# Overview

The framework with a set of mocked peripherals (most of them without any logic).
Provides a commandline GCode prompt on standard input

__Note__: A SDCard image in ./data/ is required to be open if sdcard feature is enabled in native simulator :)

```shell
RUST_LOG=info cargo run --bin printhor
```


