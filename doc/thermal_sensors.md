# The thermal sensors
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
      ```text
      R = (1000 * (3.3 - Vadc)) / Vadc
      ```
      Vadc can be easily measured with a multimeter. For convenience M105 command reports the resistor value (TZ and BZ), example:
      ```text
      > M114
      T:X.XXXX /0 T@:0 TZ:1000.00 B:Y.YYYY /0 B@:0 BZ:1000.00
      ```
    * HOT_END_THERM_NOMINAL_RESISTANCE and HOT_BED_THERM_NOMINAL_RESISTANCE

      The nominal (expected) resistance of the NTC thermistor to have at 25ºC, hat can be gotten from manufacturer specs. By default, 100000 (100k&#937;)

    * HOT_END_THERM_BETA and HOT_BED_THERM_BETA.

      The &#946; value of hot-end NTC thermistor, that can be gotten frm manufacturer as well. By default, 3950.0

If few words, if the <b>resistance</b> measured by M114 is correct and NTC nominal resistance and beta parameters are also correct, it should be fine.

These settings can be overridden at compile time by setting the proper environment variables. For instance, let's say we have a sensor pull-up resistor of 2K and the thermistor is NTC 10k 3950:
```text
HOT_END_THERM_PULL_UP_RESISTANCE=2000 HOT_END_THERM_NOMINAL_RESISTANCE=10000 cargo build [...]
```

