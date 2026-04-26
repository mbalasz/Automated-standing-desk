# ESP32 Pinout & Wiring

ESP32 WROOM-32 (30-pin DevKit). USB connector is at the top.

## Side A (left, USB at top)

| Pos | Pin  | Role              | Connector        |
|-----|------|-------------------|------------------|
| 1   | 3V3  | Power             | Display VCC      |
| 2   | GND  | Ground            | Display GND      |
| 3   | 15   | Display DIO       | Display JST      |
| 4   | 2    | Display CLK       | Display JST      |
| 5   | 4    | —                 | (inaccessible)   |
| 6   | 16   | Button UP         | Button 4-pin JST |
| 7   | 17   | Button PRESET 1   | Button 4-pin JST |
| 8   | 5    | Button PRESET 2   | Button 4-pin JST |
| 9   | 18   | Button 5 (future) | Button 5 2-pin JST |
| 10  | 19   | —                 | (inaccessible)   |
| 11  | 21   | —                 | (free)           |
| 12  | RX0  | Serial RX         | —                |
| 13  | TX0  | Serial TX         | —                |
| 14  | 22   | Button 5 GND      | Button 5 2-pin JST |
| 15  | 23   | Button DOWN       | Button 4-pin JST |

### Display (TM1637, 4-pin JST)
| JST wire | ESP32 pin |
|----------|-----------|
| VCC      | 3V3       |
| GND      | GND       |
| CLK      | 2         |
| DIO      | 15        |

### Buttons (5× tactile, common ground)
All buttons share one ground wire.

**4-pin JST** — 4 data wires:
| Wire | Function | ESP32 pin |
|------|----------|-----------|
| 1    | UP       | 16        |
| 2    | DOWN     | 23        |
| 3    | PRESET 1 | 17        |
| 4    | PRESET 2 | 5         |

**2-pin JST** — 5th button + ground:
| Wire | Function        | ESP32 pin |
|------|-----------------|-----------|
| 1    | Common GND      | 22 (software GND, driven LOW) |
| 2    | Button 5 (TBD)  | 18        |

---

## Side B (right, USB at top)

| Pos | Pin | Role          | Connector           |
|-----|-----|---------------|---------------------|
| 1   | VIN | Power in      | —                   |
| 2   | GND | Ground        | —                   |
| 3   | 13  | —             | (free)              |
| 4   | 12  | —             | (inaccessible)      |
| 5   | 14  | —             | (free)              |
| 6   | 27  | —             | (free)              |
| 7   | 26  | Stepper GND   | Stepper 4-pin JST   |
| 8   | 25  | DIR           | Stepper 4-pin JST   |
| 9   | 33  | STEP          | Stepper 4-pin JST   |
| 10  | 32  | ENABLE (MF+)  | Stepper 4-pin JST   |
| 11  | 35  | —             | (input only)        |
| 12  | 34  | —             | (input only)        |
| 13  | VN  | —             | (input only)        |
| 14  | VP  | —             | (input only)        |
| 15  | EN  | Chip enable   | —                   |

### Stepper Driver (DM542Y, 4-pin JST)
| JST wire | ESP32 pin | Driver terminal |
|----------|-----------|-----------------|
| 1        | 26 (GND)  | GND (software, driven LOW) |
| 2        | 25        | DIR−            |
| 3        | 33        | PUL− (STEP)     |
| 4        | 32        | ENA− (MF+)      |

> The stepper driver's GND, DIR, STEP, and ENABLE signals are active-LOW on the DM542Y.
> Power (24V+/24V−) connects directly from the PSU to the driver, not through the ESP32.
