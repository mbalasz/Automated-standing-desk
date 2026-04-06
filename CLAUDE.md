# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Firmware for an automated standing desk controller running on an ESP32 microcontroller, built with PlatformIO and the Arduino framework. The motor is a stepper motor controlled via step/direction signals.

## Commands

```bash
# Build the firmware
pio run

# Upload to ESP32 (ensure device is connected)
pio run --target upload

# Open serial monitor (921600 baud)
pio device monitor

# Build and upload in one step
pio run --target upload && pio device monitor
```

## Architecture

The entire firmware lives in `src/main.cpp` — a single Arduino-style file with `setup()` and `loop()`.

### Height tracking
Desk height is tracked as a step count (`currDeskHeight`, integer) relative to the lowest position (0). The maximum is `MOTOR_MAX_STEPS` (183000). Height is persisted to NVS flash via the `Preferences` library on every stop, and restored on boot.

### Motion state machine
`currMotionState` transitions: `STOPPED → ACCELERATING → RUNNING → DECELERATING → STOPPED`

- **ACCELERATING**: `currSpeedDelayMs` decreases from `maxSpeedDelayMs` (2ms) toward `minSpeedDelayMs` (0.23ms) over `accelerationSteps` (2200) steps.
- **RUNNING**: Full speed until near a boundary.
- **DECELERATING**: Triggered explicitly (button release or boundary proximity). Mirrors acceleration curve in reverse, using `currAccelerationSteps` to know how far to decelerate.
- On stop: a **backtrack** (small reverse movement) is applied to release mechanical tension before disabling the motor.

### Boundary system
`setDeskHeightBoundaries()` constrains movement. For preset-based moves, boundaries are set to the target height so deceleration and stop trigger automatically at the destination.

### Inputs
- **Move up/down buttons**: momentary, active LOW, `INPUT_PULLUP`. Holding moves the desk; releasing triggers deceleration then stop.
- **Preset buttons** (2x): hold >2s to save current height, tap to move to saved height. Stored in NVS under the `"preset"` namespace.
- **Distance sensor** (HC-SR04 on pins 22/23): pins initialized but not actively used in current logic.

### Key constants / tuning parameters
| Variable | Value | Purpose |
|---|---|---|
| `accelerationSteps` | 2200 | Steps to reach full speed |
| `maxSpeedDelayMs` | 2ms | Slowest speed (start/stop) |
| `minSpeedDelayMs` | 0.23ms | Fastest speed |
| `BACKTRACK_STEPS_AFTER_GOING_UP` | 300 | Reverse steps after upward move |
| `BACKTRACK_STEPS_AFTER_GOING_DOWN` | 60 | Reverse steps after downward move |
| `MOTOR_MAX_STEPS` | 183000 | Full travel range in steps |

### Height recalibration
`resetDeskHeightToZero()` exists for one-off recalibration. Call it inside `setup()`, upload, then immediately comment it out and re-upload.
