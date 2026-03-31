# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Purpose

This is a research documentation repository for **Cascadia Motion Motor Control Unit (MCU)** integration. It consolidates findings about RS-232/CAN communication, firmware flashing, connector pinouts, and system architecture for a BlueBoy EV powertrain project.

## Structure

This repo contains a single primary document: `README.md` — a technical reference covering:

- **C2Prog** firmware flashing workflow (boot mode via `/PROG_ENA`)
- **RS-232 communication** with the Cascadia Motion inverter/MCU
- **35-way AMPSEAL J1 connector** full pinout table
- **RMS GUI** — the Windows service/diagnostics application
- **Regenerative braking** energy flow through the inverter DC bus

## Key Technical Facts

**Critical J1 pins for serial/CAN:**
- J1-7 = `/PROG_ENA` (active-low boot mode enable — ground only when flashing)
- J1-12 = `TXD`, J1-35 = `RXD`, J1-22 = `GND` (RS-232)
- J1-11 = `CANA_L`, J1-33 = `CANA_H`

**J1 to DB9 RS-232 mapping:** J1-12→DB9 pin 2, J1-35→DB9 pin 3, J1-22→DB9 pin 5

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

When adding to this research, update `README.md` directly. Prefer verified hardware documentation over inferred information; note the distinction where diagrams are conceptual vs. factory-verified schematics.
