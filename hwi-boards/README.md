![Minimum Rust: 1.99](https://img.shields.io/badge/Minimum%20Rust%20Version-1.99-green.svg)
[![crates.io](https://img.shields.io/crates/v/prinThor.svg)](https://crates.io/crates/prinThor)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Discord Shield](https://discordapp.com/api/guilds/1169965662618259456/widget.png?style=shield)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer and robotics firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# Overview

One HWI (hardware interface) crate per supported board, each implementing the contract of
[printhor-hwa-common](../printhor-hwa-common/README.md):

| Crate                                                                                | Board                                   | Status            |
|--------------------------------------------------------------------------------------|-----------------------------------------|-------------------|
| [printhor-hwi_native](printhor-hwi_native/README.md)                                 | Native simulator                        | Functional        |
| [printhor-hwi_skr_mini_e3](printhor-hwi_skr_mini_e3/README.md)                       | SKR Mini E3 V3.0                        | Almost Functional |
| [printhor-hwi_mks_robin_nano](printhor-hwi_mks_robin_nano/README.md)                 | MKS Robin Nano v3.1                     | Almost Functional |
| [printhor-hwi_nucleo_64_arduino_cnc_hat](printhor-hwi_nucleo_64_arduino_cnc_hat/README.md) | Nucleo-f410rb/l476rg + Arduino CNC Hat v3 | Almost Functional |
| [printhor-hwi_rp_2040](printhor-hwi_rp_2040/README.md)                               | Raspberry Pi RP2040                     | Draft             |
| [printhor-hwi_esp32](printhor-hwi_esp32/README.md)                                   | ESP32-S3-WROOM                          | Naïve Skeleton    |

`printhor-hwi_boilerplate` is a starting point for new boards.
