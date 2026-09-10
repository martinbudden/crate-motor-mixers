# Dshot

The [Dshot](https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/) protocol
is based on [W2812B](https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf) (`NeoPixel`) protocol.

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
| DShot150 |            150 Kbps |      106.7 μ s |                     9.37 kHz |
| DShot300 |            300 Kbps |       53.3 μ s |                    18.75 kHz |
| DShot600 |            600 Kbps |       26.7 μ s |                    37.50 kHz |
