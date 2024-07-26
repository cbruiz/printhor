![Minimum Rust: 1.79](https://img.shields.io/badge/Minimum%20Rust%20Version-1.79-green.svg)
[![crates.io](https://img.shields.io/crates/v/prinThor.svg)](https://crates.io/crates/prinThor)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<h3>Printhor: The highly reliable but not necessarily functional 3D printer firmware</h3>

<h5><p align="center"><i>If you are using this product or like the project, please <a href="https://github.com/cbruiz/printhor/stargazers">★</a> this repository to show your support! 🤩</i></p></h5>

# Architecture

The Work-in-Progress architecture design

![alt text](../design/printhor_motion_high_level_architecture.png "High Level Architecture (motion only as of now)")

## Event bus

## Deferred commands

## State machine



## Physics and mathematics

There are two major steps in the kinematics:
1. Motion plan computation. A Double S-Shape motion profile with 7 segments is calculated constraining to expected constraints (mean velocity, acceleration and jerk) and queued for execution. The queue size is configurable at compile time.
2. Motion plan execution. A dedicated high priority task, dequeues each planned move, interpolates the plan at fixed frequency (the higher frequency the finest)
The interpolation algorithm is tolerant to unexpected short delays that can happen. A custom experimentation to adjust the interpolation frequency is 

In order to facilitate the understanding and decouple the reponsabilities as well, each movement is decomposed as:
* Unitary director vector
* Distance module

So plan computation is related to the distance module only, which is interpolated as micro-segments and the axis advance is 
computed by multiplying unitary director vector by the Delta(t) 

# The thermal sensor
A custom thermistor is supported (see configuration below). Calculations are based on Steinhart-Hart equation.
ADC calibration with internal VRef is automatically performed when the MCU supports it, in order to improve accuracy.

# Configuration

TBD

## Temperature calibration

Currently only thermistor support is implemented.
Normally the sensor circuit consist in a pull-up resistor between ADC Pin and +Vref (3.3v in STM32 boards).
The thermistor itself should be easy to determine from manufacturer specs/datasheet, so there are three parameters by each (currently 2 supported sensors):
* The pull-up resistor value, based on the specs:
  * HOT_END_THERM_PULL_UP_RESISTANCE and HOT_BED_THERM_PULL_UP_RESISTANCE
  
    The pull-up resistance of sensor circuit in ohms, with a default value of 4685.0 (~4.7k&#937;)
    This value can be measured connecting a normal resistor (1k&#937; for instance) to the sensor pins and calculating R as:
    ```
    R = (1000 * (3.3 - Vadc)) / Vadc
    ```
    Vadc can be easily measured with a multimeter. For convenience M105 command reports the resistor value (TZ and BZ), example:
    ```
    > M114
    T:X.XXXX /0 T@:0 TZ:1000.00 B:Y.YYYY /0 B@:0 BZ:1000.00
    ```
  * HOT_END_THERM_NOMINAL_RESISTANCE and HOT_BED_THERM_NOMINAL_RESISTANCE

    The nominal (expected) resistance of the NTC thermistor to have at 25ºC, hat can be gotten from manufacturer specs. By default, 100000 (100k&#937;)  
  
  * HOT_END_THERM_BETA and HOT_BED_THERM_BETA.
  
    The &#946; value of hot-end NTC thermistor, that can be gotten frm manufacturer as well. By default: 3950.0

If few words, if the <b>resistance</b> measured by M114 is correct and NTC nominal resistance and beta parameters are also correct, it should be fine.

These settings can be overridden at compile time by setting the proper environment variables. For instance, let's say we have a sensor pull-up resistor of 2K and the thermistor is NTC 10k 3950:
```
HOT_END_THERM_PULL_UP_RESISTANCE=2000 HOT_END_THERM_NOMINAL_RESISTANCE=10000 cargo build [...]
```


## Movement queue

TBD

# Troubleshooting
