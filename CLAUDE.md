# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Purpose

This is a research documentation repository for **Cascadia Motion Motor Control Unit (MCU)** integration. It consolidates findings about RS-232/CAN communication, firmware flashing, connector pinouts, and system architecture for a BlueBoy EV powertrain project.

## Structure

- `README.md` — project intro and index linking into the reference guide
- `PM100_Reference_Guide.md` — primary technical reference (connectors, communication, software, system architecture)

## Key Technical Facts

**Critical J1 pins for serial/CAN:**
- J1-7 = `/PROG_ENA` (active-low boot mode enable — ground only when flashing)
- J1-12 = `TXD`, J1-35 = `RXD`, J1-22 = `GND` (RS-232)
- J1-11 = `CANA_L`, J1-33 = `CANA_H`
- J1-23 = `CANB_H`, J1-34 = `CANB_L`
- Pins 10, 18, 29, 32 = Reserved — do not connect

**J1 to DB9 RS-232 mapping:** J1-12→DB9 pin 2, J1-35→DB9 pin 3, J1-22→DB9 pin 5

**Termination:** 120 Ω termination is for CAN bus only — RS-232 uses no termination resistors.

**Tool roles:**
- `C2Prog` / `c2p-cli` = firmware flashing only (boot mode required)
- `RMS GUI` = normal service/diagnostics over RS-232 (run mode)
- Generic serial terminals = port testing only, not protocol-aware

**macOS serial device detection:**
```bash
ls /dev/cu.*
screen /dev/cu.usbserial-FTES73H7 115200
```

**Regen path:** Motor U/V/W → Inverter power stage → Inverter DC bus → HV battery (motor does NOT feed battery directly on separate wires)

## Documentation Updates

Add new technical content to `PM100_Reference_Guide.md`. Update `README.md` only if new top-level sections are added. Prefer verified hardware documentation over inferred information; note the distinction where diagrams are conceptual vs. factory-verified schematics.

Official Cascadia Motion documentation (firmware, RMS GUI downloads): `https://www.cascadiamotion.com/documentation`
