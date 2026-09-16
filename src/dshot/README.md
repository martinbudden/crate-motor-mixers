# Dshot

The [Dshot](https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/) protocol
is based on [W2812B](https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf) (`NeoPixel`) protocol.

See also: [DSHOT - the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/).

See <https://en.wikipedia.org/wiki/Run-length_limited#GCR:_(0,2)_RLL> for details of the GCR encoding.

`Dshot150` means 150 kilobytes/second, `Dshot300` means 300 kilobytes/second

* `T0` is the width of the pulse
* `T1` is the width of gap to the next pulse

WS2812B specification is

```text
    T0H = 400ns +/- 150ns
    T1H = 800ns +/- 150ns
    T0L = 850ns +/- 150ns
    T1L = 450ns +/- 150ns
    TxH+TxL = 1250ns +/- 600ns (T0H + T0L or T1H + T1L)
    W2818B_T0H = 400
    W2818B_T1H = 800
    W2818B_T = 1250
```

`Dshot150` specification is

```text
    T0H = 2500ns (data low pulse width)
    T0L = 4180ns (data low gap width)
    T1H = 5000ns (data high pulse width)
    T1L = 1680ns (data high gap width)
    TxH+TxL = 6680ns  (T0H + T0L or T1H + T1L)
```

`Dshot300` specification is

```text
    T0H = 1250ns (data low pulse width)
    T0L = 2090ns (data low gap width)
    T1H = 2500ns (data high pulse width)
    T1L =  840ns (data high gap width)
    TxH+TxL = 3340ns  (T0H + T0L or T1H + T1L)
```

`Dshot600` specification is

```text
    T0H =  625ns (data low pulse width)
    T0L = 1045ns (data low gap width)
    T1H = 1250ns (data high pulse width)
    T0L =  420ns (data hig gap width)
    TxH+TxL = 1670ns  (T0H + T0L or T1H + T1L)
```

## Comparison

| Protocol | Effective Baud Rate | Frame Duration | Max Theoretical Refresh Rate |
| -------- | ------------------- | -------------- | ---------------------------- |
| Dshot150 |            150 Kbps |      106.7 μ s |                     9.37 kHz |
| Dshot300 |            300 Kbps |       53.3 μ s |                    18.75 kHz |
| Dshot600 |            600 Kbps |       26.7 μ s |                    37.50 kHz |

## Decoding order

```text
[ Microcontroller Pin via Input Capture/PIO ]
                    │
                    ▼
          [ 21-bit Raw Transitions ]
                    │  (Strip leading zero, decode NRZI transitions)
                    ▼
           [ 20-bit GCR Buffer ]
                    │  (Split into 4x 5-bit chunks, apply GCR lookup)
                    ▼
     [ 16-bit DshotTelemetryFrame ]  <-- Where our CRC and EDT/eRPM logic starts!
```

**NRZI** stands for Non-Return-to-Zero, Inverted.

It is a method of mapping digital binary bits (0s and 1s) into physical voltage changes on a wire.
In standard digital communication (like normal `Dshot` commands), a high voltage represents a 1 and a low voltage represents a 0.
This is known as standard **NRZ** (Non-Return-to-Zero).

**NRZI** works differently by focusing on the transitions (edges) rather than the absolute voltage levels:

* A 1 bit forces the signal wire to change state (if it was High, it flips to Low; if it was Low, it flips to High).
* A 0 bit forces the signal wire to stay the same (no change in voltage level).

### Why `DShot` Telemetry uses **GCR** + **NRZI**

Microcontrollers read incoming data by measuring the time between voltage transitions.
If an ESC sent a long string of 0 bits over normal wiring, the voltage line would just sit perfectly flat for a long time.

The microcontroller's internal clock would quickly drift, lose synchronization, and corrupt the incoming data packet.
By combining GCR and NRZI, the `DShot` protocol guarantees a rock-solid connection: GCR ensures that you never have more than two 0 bits in a row in your data stream.

Because there are mostly 1 bits, NRZI forces the physical wire to constantly flip back and forth between High and Low.

These constant flips act like a heartbeat, keeping your RP2040 or RP2350 input capture timers perfectly synchronized with the ESC's transmission clock.

## Capture Mechanism

| Architecture    | Primary Hardware Peripheral | Raw Data Form in RAM                                                                   |
| --------------- | --------------------------- | -------------------------------------------------------------------------------------- |
| RP2040 / RP2350 | PIO + DMA                   | `u32` containing raw bit values                                                        |
| STM32           | Timer Input Capture + DMA   | `[u32; 21]` array of clock timestamps                                                  |
| ESP32           | RMT                         | Array of `RmtPulse` elements specifying the microsecond duration of each high/low peak |
