![Minimum Rust: 1.99](https://img.shields.io/badge/Minimum%20Rust%20Version-1.99-green.svg)
[![crates.io](https://img.shields.io/crates/v/printhor-hwi_skr_mini_e3.svg)](https://crates.io/crates/printhor-hwi_skr_mini_e3)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Discord Shield](https://discordapp.com/api/guilds/1169965662618259456/widget.png?style=shield)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer and robotics firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# Overview

This board is quite functional:
* [BIGTREETECH SKR Mini E3 V3.0](https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3) (See [datasheets/SKR_MINI_E3-V3.0](../../datasheets/SKR_MINI_E3-V3.0)).

### Binary image production (standard with defmt)

The `firmware.bin` file, ready to be copied to the SD card, can be produced with the following command line:

```shell
DEFMT_LOG=info cargo objcopy --release --no-default-features --features skr_mini_e3_v3 --target thumbv6m-none-eabi --bin printhor -- -O binary firmware.bin
```

The firmware size is about 233kB as of now with these settings.

### FDM printer (stock Creality Ender-3)

The `skr_mini_e3_v3_fdm` profile enables the hot-end, the hot-bed and the layer fan:

```shell
DEFMT_LOG=info cargo objcopy --release --no-default-features --features skr_mini_e3_v3_fdm --target thumbv6m-none-eabi --bin printhor -- -O binary firmware.bin
```

Add `with-fan-extra-1` to also drive FAN1. Wiring (from the board schematic):

| Function             | Pin | Peripheral         |
|----------------------|-----|--------------------|
| Hot-end thermistor   | PA0 | ADC1 (TH0)         |
| Hot-bed thermistor   | PC4 | ADC1 (THB)         |
| Hot-end heater (HE0) | PC8 | TIM3_CH3 (100 Hz)  |
| Hot-bed heater (BED) | PC9 | TIM3_CH4 (100 Hz)  |
| Layer fan (FAN0)     | PC6 | TIM3_CH1 (100 Hz)  |
| Extra fan (FAN1)     | PC7 | TIM3_CH2 (100 Hz)  |

Thermistors: 100K NTC B3950 with a 4.7K pull-up (`HOT_END_THERM_*`/`HOT_BED_THERM_*`, overridable
in `.cargo/config.toml`). Default steps per mm are the stock Ender-3 ones (X/Y 80, Z 400, E 93);
adjust them with `M92`. Tune the PID constants with `M303` before printing.

### Settings persistence (EEPROM)

With the `with-eeprom` feature (enabled in the `skr_mini_e3_v3` and `skr_mini_e3_v3_fdm` profiles),
the on-board AT24C32 (4KB) EEPROM stores the settings:

| Function   | Pin | Peripheral                         |
|------------|-----|------------------------------------|
| EEPROM SCL | PB6 | I2C1 (400 kHz, DMA1 CH4/CH5)       |
| EEPROM SDA | PB7 | I2C1                               |

`M500` stores the steps per unit (`M92`), max feed rates (`M203`), max accelerations (`M201`),
jerk, `M204`, home offsets (`M206`), firmware retraction (`M207`/`M208`), the PID constants
(`M301`/`M304`) and the cold extrusion settings (`M302`). They are restored at boot and with `M501`.
`M502` restores the defaults (in memory: `M500` stores them).

### Minimal-size binary image production

```shell
DEFMT_LOG=off RUST_BACKTRACE=0 cargo objcopy --profile release-opt --no-default-features --features skr_mini_e3_v3 --target thumbv6m-none-eabi --bin printhor -- -O binary firmware.bin
```

The firmware size is about 194kB as of now with these settings.

### Run with a J-Link/SWD probe

```shell
DEFMT_LOG=info RUST_BACKTRACE=1 RUSTFLAGS='--cfg board="skr_mini_e3_v3"' cargo run --release --no-default-features --features skr_mini_e3_v3 --target thumbv6m-none-eabi --bin printhor
```

