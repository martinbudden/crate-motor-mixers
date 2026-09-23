# `motor-mixers` Rust Crate<br>![License: MIT](https://img.shields.io/badge/license-MIT-green) [![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

`motor-mixers` is a Rust crate that implements motor mixing and actuator driving for robotics and arial vehicles.

**Motor mixing** is the process used to translate movement commands (ie throttle, roll, pitch, and yaw)
into individual motor speeds and/or servo angles.

**Actuator driving** is the subsequent step where the outputs from the mixer are serialized into specific electronic signals
(like digital packets or timed pulses) and sent to a motor driver or an Electronic Speed Controller (ESC).

`motor-mixers` supports **PWM** (Pulse-Width Modulation), for DC motors and servos, and **bidirectional Dshot** for ESCs.

This crate is `no_std`, `no alloc`, and the Minimum Supported Rust Version (MSRV) is `Rust 1.89`.

## Dynamic Idle Control

When using **bidirectional Dshot**, `motor-mixers` supports **dynamic idle control**.

This feature protects against ESC desynchronization caused by "windmilling" during aggressive maneuvers,
and increases braking authority by maintaining a minimum RPM floor.

## Motor Saturation, Yaw Jumps and Yaw Washouts

* Yaw Jump this is when, at high throttle, an aggressive yaw manoeuver can cause a multirotor to unexpectedly balloon upwards (or jump).

* Yaw washout is when, at high throttle, an aggressive roll or pitch manoeuver can result in an uncommanded spin.

`motor-mixers` has settings to deal with both these occurrences.

## Hardware supported

`motor-mixers` targets embedded architectures and supports Raspberry Pi Pico, STM32, and ESP32 microcontrollers.

> **⚠️ Note:** This crate is currently under active development.
>
> `PWM` and `Dshot` implementations are provisional.
> Only unidirectional `Dshot` is available on STM32.
> `Dshot` is not yet available on ESP32.

## Mixes available

In all configurations, motor numbering follows the **Betaflight** convention.

All mixes feature built-in output saturation management and yaw-jump compensation.

In the diagrams below:

* **CW** = Clockwise
* **CC** = Counter-Clockwise

### Classic X-configuration quadcopter

Motor rotation is "propellers out" (ie Betaflight "yaw reversed").

```text
       front
 vCC^ 4     2 ^CWv
       \   /
        |X|
       /   \
 ^CWv 3     1 vCC^
```

### Tricopter (3 motors, 1 servo)

```text
    front
  vCC^   ^CWv
    3     2
     \   /
      |Y|
       |
       1
      vCW^
```

### X-configuration hexacopter

Motor rotation is "propellers out" (ie Betaflight "yaw reversed").

```text
        front
  vCC^ 4     2 ^CWv
        \   /
^CWv 6---|*|---5 vCC^
        /   \
  vCC^ 3     1 ^CWv
```

### X-configuration octocopter

Motor directions are the same as Betaflight.

The mix can be configured as a standard X-octocopter, or can use a hybrid mode.

In **hybrid mode** propulsion is split between four large lifting propellers and four small maneuvering propellers.

This gives best of both worlds: high efficiency for hovering/cruising and crisp attitude control due to the lower rotational inertia
of the smaller maneuvering props.

* **Large props:** Motors 1–4
* **Small props:** Motors 5–8

```text
       front
 vCC^ 8     6 ^CWv
 ^CWv 4     2 vCC^
       \   /
        |X|
       /   \
 vCC^ 3     1 ^CWv
 ^CWv 7     5 vCC^
```

## Original implementation

I originally implemented this crate as a C++ library:
[Library-MotorMixers](https://github.com/martinbudden/Library-MotorMixers).

## License

Licensed under either of:

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
