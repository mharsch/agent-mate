# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded Rust firmware for the Raspberry Pi Pico 2 W (RP2350 chip) mounted on a DeskPi PicoMate carrier board. Targets the Cortex-M33 (`thumbv8m.main-none-eabihf`) by default; RISC-V (`riscv32imac-unknown-none-elf`) is also supported via alternate linker scripts.

## Build Commands

```bash
# Build (default Arm/Cortex-M33 target)
cargo build

# Build for RISC-V
cargo build --target riscv32imac-unknown-none-elf

# Flash to device via picotool (device must be in BOOTSEL mode or already running)
cargo run

# Flash and open RTT log viewer via probe-rs
cargo embed
```

`DEFMT_LOG=debug` is set by default in `.cargo/config.toml`. Change it to `info`, `warn`, or `error` to filter log output.

## Architecture

This is a `#![no_std]` / `#![no_main]` bare-metal crate — no OS, no allocator, no standard library.

- **Entry point**: `src/main.rs` — `#[hal::entry] fn main() -> !`
- **HAL**: `rp235x-hal` wraps the PAC (peripheral access crate) for the RP2350. Access peripherals through `hal::pac::Peripherals::take()`.
- **Logging**: `defmt` + `defmt-rtt` for structured, low-overhead logging over RTT (viewed via `cargo embed` or a probe-rs tool).
- **Panic handler**: `panic-probe` — panics are printed over defmt then halt.
- **Linker scripts**: `memory.x` (Arm) and `rp235x_riscv.x` (RISC-V) are copied to `OUT_DIR` by `build.rs` so the linker can find them. Edit `memory.x` if flash/RAM sizes need adjustment (Pico 2 has 4 MiB flash; the script conservatively uses 2 MiB).
- **Boot block**: `IMAGE_DEF` in `.start_block` tells the RP2350 Boot ROM this is a secure executable. `PICOTOOL_ENTRIES` in `.bi_entries` provides metadata readable by `picotool info`.

## Hardware Notes

- Crystal frequency: 12 MHz (`XTAL_FREQ_HZ`). Do not change unless using a different board.
- Default system clock: 125 MHz (set by `init_clocks_and_plls`).
- GP15 is currently used for an external LED on a breadboard (blinked in main loop). Per the PicoMate pinout, GP15 is the RGB LED red channel — see table below.

### DeskPi PicoMate Pin Mapping

| Peripheral | Pins |
|---|---|
| Button | GP26, GND |
| Rotary Encoder | GP6 (A), GP7 (B), GP8 (SW), GND |
| Digital Microphone | GP9 (CLK), GP10 (DAT), GND |
| Accelerometer & Gyroscope | I2C0 — GP4 (SDA), GP5 (SCL) |
| 3-Axis Magnetometer | I2C0 — GP4 (SDA), GP5 (SCL) |
| Temperature & Humidity | I2C0 — GP4 (SDA), GP5 (SCL) |
| RGB LED | GP15 (R), GP16 (G), GP17 (B), GND |
| Buzzer | GP18, GND |
| Digital PIR Sensor | GP19, GND |
| 0.96" OLED 128x64 | I2C1 — GP20 (SDA), GP21 (SCL) |
| Digital Optical Sensor | SPI — GP11 (SCK), GP12 (TX), GP13 (RX), GP14 (CS) |

Accelerometer, Magnetometer, and Temperature/Humidity share the I2C0 bus (GP4/GP5). The OLED uses a separate I2C1 bus (GP20/GP21).

## Flashing Prerequisites

- `picotool` must be installed and on `PATH` (used as the Cargo runner).
- For `cargo embed`: `probe-rs` must be installed; the Pico 2 W must be connected via SWD (not just USB).
