# The controllers anatomy
Why is that so complex?

\[TLDR;\] Just because!.

In bare-metal, most the peripherals need to be shared and they live forever in nature.
In rust, there is always a lifetime for everything to met the language constraints, so obviously,
in `PrinThor` there is a big bunch of machinery to wrap shared (stateful or stateless) peripherals (a Pin, for instance) or adaptors (a PwmController or Heater).

There are complex Controllers build on top of Controllers. Most meaningful case is the SDCard Controller.

Again, in bare-metal, is quite common that the -physical- SDCard peripheral communicates with the MCU leveraging a SPI Bus, which usually is shared.
So, the construction is like:

```text
StaticAsyncController[SDCardController] <- A clonable and shareable controller which wraps a single reference
    - StaticAsyncController[SDCard] <- A stateful controller (holding open files, etc) 
        - StaticAsyncController[Device] <- The "Device", which can be shared
            - CSPin <- Now yes, the owned instance of the CS Pin
            - StaticAsyncController[SPI] <- Another controller to share same SPI for different controllers
                - SPI <- Now yes, the owned instance

```

So there can be a **single**  `StaticAsyncController[SPI]` constructed by the HWI layer than can be shared by two **different** high level controllers like:

* The SDCardController
* Another controller. A DisplayController, for instance

# Architecture

The Work-in-Progress architecture design

![alt text](img/printhor_motion_high_level_architecture.png "High Level Architecture (motion only as of now)")

## Event bus

## Deferred commands

## State machine
