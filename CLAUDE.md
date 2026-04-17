# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Arduino Mega 2560 firmware for an automated coil/transformer winding machine. Controls two synchronized stepper motors (shaft rotation + layer carriage traverse), a 20×4 LCD menu interface, and an unwind mode with encoder-based turn tracking.

## Build & Upload

This is an **Arduino sketch** — there is no Makefile or build script.

- **IDE**: Arduino IDE or PlatformIO (`pio run` / `pio run -t upload`)
- **Board**: Arduino Mega 2560 @ 16 MHz
- **Required libraries**: GyverStepper2, GyverPlanner, EncButton, AnalogKey, LiquidCrystal, LiquidCrystal_I2C
- **Testing**: Manual only — no automated test framework; verify behaviour on hardware

## Hardware Configuration (config.h)

All hardware-specific constants live in `config.h`. Edit this file when porting to a different shield or motor:

- Pin assignments are hardcoded for **RAMPS 1.4 shield** on Mega
- `STEPPER_Z_MICROSTEPS = 4` (shaft, 1/4 step), `STEPPER_A_MICROSTEPS = 8` (carriage, 1/8 step)
- `THREAD_PITCH = 1000` µm/layer, `UNWIND_PPR = 1024` encoder pulses/rev
- Display: `DISPLAY_I2C 0` (parallel) or `1` (I2C); `LANGUAGE EN` or `RU`

## Architecture

### Motion system
- **GPlanner** (GyverPlanner) coordinates two axes: Z (shaft) and A (layer carriage).
- **Timer1 ISR** (`TIMER1_COMPA_vect`) drives step pulses using an 8.8 fixed-point speed multiplier, enabling real-time speed adjustment without recalculating motion profiles.
- Speed changes during winding apply the multiplier in the ISR; `noInterrupts()`/`interrupts()` guards protect shared motion state.

### Winding engine (`Arduino_winding_machine.ino`)
- `AutoWinding()` — main winding loop: coordinates shaft + carriage, handles pedal start/stop, encoder-based pause/resume, per-layer stopping, and speed adjustment.
- `AutoWindingAll()` — sequences multiple winding profiles (up to 3) with delays between them.
- `UnwindWinding()` — reverse operation; `unwindISR_A` counts quadrature encoder edges via direct register reads (not `digitalRead`) for speed.

### UI
- **Menu system** (`Menu.h`): hierarchical menu with ~30 states enumerated in the main sketch. Modifying menu structure requires re-indexing the `MenuState` enum.
- **Screen** (`Screen.h`): real-time winding status (turns, layers, speed, status text).
- **Input**: rotary encoder (KY-040) via EncButton; stop pedal on `INPUT_PULLUP`; optional 4-direction analog keypad.

### Persistence (EEPROM)
- `Settings` struct at `0x00`, winding profiles (`Winding` × 3 profiles × 3 transformers) at `0x10`, unwind params at `0x80+`.
- Each section has a version byte; mismatch resets to defaults — safe firmware upgrade path.
- Unwind turn counter is flushed every ~3 s for power-loss recovery.
- EEPROM writes are guarded to occur only on value change (100 K write cycle limit).

### Display driver (`LiquidCrystalCyr.h`)
- Wraps `LiquidCrystal`/`LiquidCrystal_I2C` with Cyrillic font support.
- At most 8 custom characters can be loaded into LCD CGRAM at once; the driver caches which glyphs are resident and regenerates only on demand.
- Font bitmaps (5×8) live in PROGMEM in `font.h`.

### Strings / localisation (`strings.h`)
- All UI text is defined as `PROGMEM` string constants.
- Switch `LANGUAGE` in `config.h` between `EN` and `RU` to change locale at compile time.

## Key Constraints

- **Interrupt safety**: any variable shared between the ISR and `loop()` must be `volatile`; wrap multi-byte reads/writes in `noInterrupts()`/`interrupts()`.
- **CGRAM budget**: the LCD supports only 8 user-defined characters simultaneously; adding new Cyrillic characters requires evicting existing ones.
- **Encoder ISR timing**: `unwindISR_A` uses direct port register access — do not replace with `digitalRead()` as it is too slow at high RPM.
- **Menu enum ordering**: `MenuState` values in the main sketch map directly to menu rendering logic; inserting items in the middle breaks the mapping.
