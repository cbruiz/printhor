![Minimum Rust: 1.99](https://img.shields.io/badge/Minimum%20Rust%20Version-1.99-green.svg)
[![crates.io](https://img.shields.io/crates/v/printhor-hwi_mks_robin_nano.svg)](https://crates.io/crates/printhor-hwi_mks_robin_nano)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Discord Shield](https://discordapp.com/api/guilds/1169965662618259456/widget.png?style=shield)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer and robotics firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# Overview

This board ([MKS Robin Nano V3.1](https://www.makerbase.store/pages/mks-robin-nano-v3-1-intro)) is still a work in progress.

### Binary image production (standard with defmt)

The `firmware.bin` file, ready to be copied to the SD card, can be produced with the following command line:

```shell
DEFMT_LOG=info cargo objcopy --release --no-default-features --features mks_robin_nano --target thumbv7em-none-eabihf --bin printhor -- -O binary firmware.bin
```

The firmware size is about 236kB as of now with these settings.

### FDM printer

The `mks_robin_nano_fdm` profile enables the hot-end, the hot-bed and the fans:

```shell
DEFMT_LOG=info cargo objcopy --release --no-default-features --features mks_robin_nano_fdm --target thumbv7em-none-eabihf --bin printhor -- -O binary firmware.bin
```

Wiring (from the MKS Robin Nano V3.1 pinout):

| Function              | Pin  | Peripheral                  |
|-----------------------|------|-----------------------------|
| Hot-end thermistor    | PC1  | ADC1 (TH1)                  |
| Hot-bed thermistor    | PC0  | ADC1 (TB)                   |
| Hot-end heater (HE0)  | PE5  | TIM9_CH1 (100 Hz)           |
| Hot-bed heater (H-BED)| PA0  | TIM5_CH1 (100 Hz)           |
| Layer fan (FAN2)      | PB1  | TIM3_CH4 (100 Hz)           |
| Extra fan (FAN1)      | PC14 | GPIO, on/off (no timer)     |

**Connect the layer (part cooling) fan to FAN2**: PC14 (FAN1) has no timer channel, so it can only
be switched on and off (`M106 P1` with any power above 0 turns it fully on). FAN1 is suitable for
the hot-end heat-sink fan: it is automatically turned on while the hot-end is above 50ºC.

Thermistors: 100K NTC B3950 with a 4.7K pull-up (`HOT_END_THERM_*`/`HOT_BED_THERM_*`, overridable
in `.cargo/config.toml`). Set the steps per mm of your machine with `M92` and tune the PID constants
with `M303` before printing.

### Settings persistence (EEPROM)

With the `with-eeprom` feature (enabled in the `mks_robin_nano` and `mks_robin_nano_fdm` profiles),
the on-board 24C32 (4KB) EEPROM stores the settings:

| Function   | Pin | Peripheral                                 |
|------------|-----|--------------------------------------------|
| EEPROM SCL | PB6 | I2C1 (400 kHz, DMA1 stream 6 TX / 0 RX)    |
| EEPROM SDA | PB7 | I2C1                                       |

SPI3 (SD card) RX uses DMA1 stream 2, so that stream 0 is free for I2C1.

`M500` stores the steps per unit (`M92`), max feed rates (`M203`), max accelerations (`M201`),
jerk, `M204`, home offsets (`M206`), firmware retraction (`M207`/`M208`), the PID constants
(`M301`/`M304`) and the cold extrusion settings (`M302`). They are restored at boot and with `M501`.
`M502` restores the defaults (in memory: `M500` stores them).

### Minimal-size binary image production

```shell
DEFMT_LOG=off RUST_BACKTRACE=0 cargo objcopy --profile release-opt --no-default-features --features mks_robin_nano --target thumbv7em-none-eabihf --bin printhor -- -O binary firmware.bin
```

The firmware size is about 195kB as of now with these settings.

### Run with a J-Link/SWD probe

```shell
DEFMT_LOG=info RUST_BACKTRACE=1 RUSTFLAGS='--cfg board="mks_robin_nano"' cargo run --release --no-default-features --features mks_robin_nano --target thumbv7em-none-eabihf --bin printhor
```
