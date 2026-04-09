# Wiring guide — ESP32 WROOM 32 + 2× TMC2209

All ESP32 pins used are on the **right side** of the board (USB at top).

## Power

| TMC2209 pin | Connect to |
|---|---|
| VM | 12–24 V motor supply |
| VIO / VCC_IO | 3.3 V (ESP32) |
| GND | Common ground — tie ESP32 GND and PSU GND together with one wire. ESP32 can still be powered separately via USB. |

## UART (shared bus, both drivers)

TMC2209 uses single-wire half-duplex UART on `PDN_UART`. Bridge TX→RX with a 1 kΩ resistor:

```
ESP32 GPIO17 (TX) ── 1kΩ ──┐
                            ├── PDN_UART (driver 1 + driver 2)
ESP32 GPIO16 (RX) ──────────┘
```

## UART address (MS1 / MS2 pins)

| Driver | MS1 | MS2 | Address |
|---|---|---|---|
| Motor 1 | GND | GND | 0 |
| Motor 2 | 3.3 V | GND | 1 |

## Per-driver control pins

Driver 1 uses the **left side** of the ESP32; driver 2 uses the **right side**.
On the right side, UART + driver 2 occupy 6 consecutive pins: RX2(16), TX2(17), STEP(5), DIR(18), EN(19), DIAG(21).

| Signal | Driver 1 (left) | Driver 2 (right) |
|---|---|---|
| STEP | GPIO 25 | GPIO 5 |
| DIR | GPIO 33 | GPIO 18 |
| EN | GPIO 32 | GPIO 19 |
| DIAG | GPIO 34 | GPIO 21 |

## Motor coils

Connect NEMA 17 coils to driver outputs (labeled M1A/M1B/M2A/M2B or OA1/OA2/OB1/OB2). Swap A+/A- or B+/B- if a motor runs the wrong direction — motor 2 inversion is handled in firmware via `driver2.shaft(true)` so physical swapping should not be needed.

## Notes

- **Decoupling caps** — 100 nF ceramic close to each driver's VIO pin; 100 µF electrolytic on VM to absorb motor switching spikes.
- **DIAG pulldown** — optional 10 kΩ pulldown on each DIAG line to prevent spurious stall triggers at startup.
- **GPIO 22/23** are bidirectional (unlike the original GPIO 34/35 which were input-only). Don't configure them as outputs elsewhere in the code.
