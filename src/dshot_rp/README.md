# RP Dshot

## Decoding

Raspberry Pi microcontrollers use PIO to capture the pin transitions directly as GCR

```text
[ Microcontroller Pin via PIO ]
                    │
                    ▼
             [ 20-bit GCR ]
                    │  (Split into 4x 5-bit chunks, apply GCR lookup)
                    ▼
        [ 16-bit DshotTelemetryFrame ]
```
