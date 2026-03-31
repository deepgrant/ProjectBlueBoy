# Cascadia Motion MCU Integration Reference

Technical reference for RS-232/CAN communication, firmware flashing, and connector pinout for the Cascadia Motion inverter/MCU used in the BlueBoy EV powertrain project.

---

## J1 Connector — 35-Way AMPSEAL

### Critical pins

| Signal | Pin | Notes |
|---|---|---|
| `/PROG_ENA` | J1-7 | Active-low boot mode enable — ground **only** when flashing |
| `CANA_L` | J1-11 | CAN channel A low |
| `TXD` | J1-12 | RS-232 transmit (MCU → PC) |
| `GND` | J1-22 | Ground |
| `CANA_H` | J1-33 | CAN channel A high |
| `RXD` | J1-35 | RS-232 receive (PC → MCU) |
| `CANB_H` | J1-23 | CAN channel B high |
| `CANB_L` | J1-34 | CAN channel B low |

### Full pinout

| Pin | Signal | Description |
|---|---|---|
| 1 | XDCR_PWR | +5 V @ 80 mA max, accel pedal power |
| 2 | AGND | Analog ground, accel pedal ground |
| 3 | AIN4 | Analog input 4, 0–5 V full scale |
| 4 | RTD1 | 1000 Ω RTD input |
| 5 | RTD4 | 100 Ω RTD input |
| 6 | RTD5 | 100 Ω RTD input |
| 7 | /PROG_ENA | Serial bootloader enable (active-low) |
| 8 | DIN2 | Digital input — reverse enable switch |
| 9 | DIN5 | Digital input — ignition input |
| 10 | Reserved | Do not connect |
| 11 | CANA_L | CAN channel A low |
| 12 | TXD | RS-232 transmit |
| 13 | AIN1 | Analog input 1, 0–5 V full scale — accel pedal wiper |
| 14 | XDCR_PWR | +5 V @ 80 mA max, spare transducer power |
| 15 | AGND | Analog ground |
| 16 | RTD2 | 1000 Ω RTD input |
| 17 | AGND | Analog ground |
| 18 | Reserved | Do not connect |
| 19 | AGND | Analog ground |
| 20 | DIN3 | Digital input — brake switch |
| 21 | DIN6 | Digital input — start input |
| 22 | GND | Ground |
| 23 | CANB_H | CAN channel B high |
| 24 | AIN2 | Analog input 2, 0–5 V full scale — motor temperature sensor |
| 25 | AIN3 | Analog input 3, 0–5 V full scale — brake pedal |
| 26 | XDCR_PWR | +5 V @ 80 mA max, spare transducer power |
| 27 | RTD3 | 1000 Ω RTD input |
| 28 | XDCR_PWR | +5 V @ 80 mA max, spare transducer power |
| 29 | Reserved | Do not connect |
| 30 | DIN1 | Digital input — forward enable switch |
| 31 | DIN4 | Digital input — regen disable input |
| 32 | Reserved | Do not connect |
| 33 | CANA_H | CAN channel A high |
| 34 | CANB_L | CAN channel B low |
| 35 | RXD | RS-232 receive |

---

## RS-232 Communication

### Signal crossover

| PC / Adapter | Direction | MCU (J1) |
|---|---|---|
| TXD | → | RXD (J1-35) |
| RXD | ← | TXD (J1-12) |
| GND | — | GND (J1-22) |

### J1 to DB9 adapter wiring

| J1 Pin | Signal | DB9 Pin |
|---|---|---|
| J1-12 | TXD | Pin 2 |
| J1-35 | RXD | Pin 3 |
| J1-22 | GND | Pin 5 |

> **Note:** RS-232 uses no termination resistors. 120 Ω termination applies to CAN bus only.

### Verifying the serial connection

**macOS:**
```bash
ls /dev/cu.*
screen /dev/cu.usbserial-FTES73H7 115200
```

- If only `/dev/cu.Bluetooth-Incoming-Port` and `/dev/cu.debug-console` appear, the USB serial adapter is not detected.
- If `screen` opens without error, the Mac can open the port.
- Seeing no text in `screen` does **not** indicate failure — the MCU does not emit plain ASCII on its own.

**Windows 10:**
1. Confirm the adapter appears in Device Manager under *Ports (COM & LPT)*.
2. Run a loopback test by shorting TX to RX on the adapter.
3. Open the COM port with a terminal tool to verify the port is accessible.
4. Use RMS GUI for a real functional communication test.

For any platform: if in doubt, run a loopback test on the adapter before suspecting the MCU wiring.

---

## CAN Communication

Two CAN channels are available on J1:

- **CAN A** — J1-11 (`CANA_L`), J1-33 (`CANA_H`)
- **CAN B** — J1-34 (`CANB_L`), J1-23 (`CANB_H`)

Each channel requires 120 Ω termination resistors at both ends of the bus. RS-232 does not use termination.

---

## Program Mode vs. Normal Run Mode

The `/PROG_ENA` pin (J1-7) controls which mode the MCU enters at power-up:

| `/PROG_ENA` at power-up | Mode |
|---|---|
| Not grounded (floating/high) | Normal run mode |
| Grounded (connected to J1-22) | Boot / programming mode |

Do **not** ground `/PROG_ENA` during normal operation or service — it prevents the MCU from running firmware.

---

## Firmware Flashing — C2Prog

**C2Prog** (also `c2p-cli` / `C2ProgShell.exe`) is the Cascadia Motion firmware flashing tool. It requires the MCU to be in boot mode.

### Flashing workflow

1. Connect RS-232 (J1-12, J1-35, J1-22 → DB9 adapter).
2. Jumper J1-7 (`/PROG_ENA`) to J1-22 (`GND`).
3. Power the unit — it enters boot/programming mode.
4. Select the correct COM port and target in C2Prog.
5. Load the firmware file and flash.
6. Power down, remove the `/PROG_ENA` jumper, and restart normally.

C2Prog is not used for normal diagnostics — it speaks the bootloader protocol only.

---

## Diagnostics — RMS GUI

**RMS GUI** is the Cascadia Motion Windows service and setup application for the inverter. It communicates over RS-232 in normal run mode (not boot mode).

Capabilities:
- Monitor live parameters in real time
- Read and write EEPROM configuration values
- Inspect firmware version and controller status

RMS GUI is the recommended tool for verifying the MCU is alive and communicating. A generic serial terminal can confirm the port opens but cannot interpret the MCU's protocol.

Firmware and RMS GUI downloads: `https://www.cascadiamotion.com/documentation`

---

## System Architecture — Regen Path

The three-phase motor does **not** feed back to the battery on separate direct wires. The physically accurate regen current path is:

```
Motor (U/V/W) → Inverter power stage → Inverter DC bus → HV battery pack
```

Battery charging during regen depends on the inverter, DC bus, contactors, BMS, and battery acceptance — all mediated through the inverter. Diagrams showing a direct motor-to-battery path are conceptual simplifications and not factory-verified schematics.

---

## Service Checklist

Quick reference for bench or field service:

1. Verify the USB-to-RS-232 adapter works (loopback test: short TX to RX).
2. Confirm correct TX/RX crossover and GND in the J1→DB9 wiring.
3. Do **not** ground `/PROG_ENA` (J1-7) unless intentionally entering boot mode.
4. For normal communication and diagnostics: use **RMS GUI**.
5. For firmware flashing only: use **C2Prog** with MCU in boot mode.
6. Key J1 pins: `/PROG_ENA` = 7, `TXD` = 12, `GND` = 22, `CANA_L` = 11, `CANA_H` = 33, `RXD` = 35.
