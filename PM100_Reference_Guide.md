# PM100 Motor Inverter — Engineer's Reference Guide

**Target audience**: Engineers with a solid electrical background who are new to electric vehicle powertrain systems.
This guide covers the Cascadia Motion / Rinehart Motion Systems PM100 AC motor inverter: connectors,
communication interfaces, software tools, and system integration.

---

## Contents

1. [System Architecture Overview](#1-system-architecture-overview)
2. [Power Connections](#2-power-connections)
3. [J1 Connector — 35-Pin AMPSEAL](#3-j1-connector--35-pin-ampseal)
4. [J2 Connector — 23-Pin AMPSEAL](#4-j2-connector--23-pin-ampseal)
5. [Input/Output Electrical Details](#5-inputoutput-electrical-details)
6. [RS-232 Serial Interface](#6-rs-232-serial-interface)
7. [CAN Bus Interface](#7-can-bus-interface)
8. [RMS GUI Software](#8-rms-gui-software)
9. [C2Prog Firmware Flashing](#9-c2prog-firmware-flashing)
10. [Vehicle State Machine (VSM)](#10-vehicle-state-machine-vsm)
11. [Regenerative Braking](#11-regenerative-braking)
12. [Key Configuration Parameters](#12-key-configuration-parameters)
13. [Glossary](#13-glossary)

---

## 1. System Architecture Overview

The PM100 is a **3-phase AC motor inverter** — it converts DC from a high-voltage battery pack into
3-phase AC to drive an electric motor. It also manages vehicle-level logic (enable/disable, pre-charge
sequencing, fault handling) through an internal Vehicle State Machine (VSM).

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        COMPLETE POWERTRAIN OVERVIEW                          │
│                                                                              │
│  ┌─────────────┐  HV DC (100–420V)   ┌────────────────────────────────────┐  │
│  │  HV BATTERY │ ══════════════════> │           PM100 INVERTER           │  │
│  │    PACK     │ <══════════════════ │                                    │  │
│  │             │   regen current     │  ┌──────────┐   ┌───────────────┐  │  │
│  └─────────────┘  flows back through │  │  DC Link │   │  3-Phase      │  │  │
│         │         DC bus (not direct)│  │ ~500 µF  │   │  IGBT Bridge  │  │  │
│         │                            │  └──────────┘   └───────┬───────┘  │  │
│         │                            │                         │          │  │
│  ┌──────┴──────┐                     │  ┌──────────────────┐   │          │  │
│  │  MAIN HV    │                     │  │    TI DSP        │   │          │  │
│  │ CONTACTOR   │                     │  │ (TMS320F28234)   │   │          │  │
│  │  (DC-rated) │                     │  │ Field-Oriented   │   │          │  │
│  └──────┬──────┘                     │  │ Vector Control   │   │          │  │
│         │                            │  └──────────────────┘   │          │  │
│  ┌──────┴──────┐                     └────────────────────────┬┘          │  │
│  │  PRE-CHARGE │                                     Phase A/B/C          │  │
│  │  CIRCUIT    │                                              │           │  │
│  │(R + relay)  │                                      ┌───────┴──────┐    │  │
│  └─────────────┘                                      │  AC MOTOR    │    │  │
│                                                       │(induction or │    │  │
│  ┌─────────────┐  12V logic power                     │ PM sync.)    │    │  │
│  │  12V BATT   │ ═══════════════════════════════════> │              │    │  │
│  │  (ignition) │  J2-8/J2-23 BATT+                    │  + feedback  │    │  │
│  └─────────────┘                                      │  (encoder or │    │  │
│                                                       │  resolver)   │    │  │
│  ┌─────────────┐  CAN A or RS-232                     └──────────────┘    │  │
│  │  VEHICLE    │ <══════════════════════════════════ PM100 J1 connector   │  │
│  │ CONTROLLER  │ ══════════════════════════════════> (torque commands,    │  │
│  │  (optional) │                                      analog pedal, etc.) │  │
│  └─────────────┘                                                          │  │
└──────────────────────────────────────────────────────────────────────────────┘
```

**Why two voltage domains?**

- **HV (high-voltage) bus**: 100–420 VDC, drives the motor, handled only by the power stage.
- **12V logic bus**: powers the DSP, communication, and relay drivers — ordinary automotive wiring.

These are isolated from each other inside the inverter.

---

## 2. Power Connections

### 2.1 DC Power (HV Bus)

The PM100 has two large DC wire ports at the rear: **DC+** and **DC−**.

⚠️ **A pre-charge circuit is mandatory** before closing the main contactor. Without it, the inrush
current as the internal ~500 µF capacitor charges can weld main contactor contacts shut.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        PRE-CHARGE CIRCUIT                                    │
│                                                                              │
│  HV Battery +  ──────┬──────────────────────────┬────────  DC+ to PM100      │
│                      │                          │                            │
│               ┌──────┴──────┐          ┌────────┴────────┐                   │
│               │  PRE-CHARGE │          │   MAIN HV       │                   │
│               │  CONTACTOR  │          │   CONTACTOR     │                   │
│               │  (normally  │          │   (DC-rated!    │                   │
│               │   open)     │          │    AC-only types│                   │
│               └──────┬──────┘          │    will weld)   │                   │
│                      │                 └─────────────────┘                   │
│               ┌──────┴──────┐                                                │
│               │   600 Ω     │  ← limits inrush to ~0.5A at 320V              │
│               │   50 W      │    Peak dissipation: ~171 W for ~0.9s          │
│               │  RESISTOR   │                                                │
│               └──────┬──────┘                                                │
│                      │    ← pre-charge contactor closes first                │
│  HV Battery − ───────┴────────────────────────────────────  DC− to PM100     │
│                                                                              │
│  SEQUENCE: 1) Close pre-charge contactor  →  capacitor charges slowly        │
│            2) Monitor DC bus voltage      →  when near battery voltage...    │
│            3) Close main contactor        →  now DC bus is live              │
│            4) Open pre-charge contactor   →  resistor no longer in circuit   │
└──────────────────────────────────────────────────────────────────────────────┘
```

RMS suggested parts: relay 77-0026, resistor 53-0006 (600 Ω 50 W), fuse 59-0008 (5A 500V),
contactor 77-0025 (Tyco EV200AAANA), main fuse: Bussmann FWP-400A.

### 2.1.1 Contactor Coil Control — PM100 Relay Outputs

The PM100 sequences both contactors automatically through its VSM pre-charge states. Both outputs
are **hi-side (12V) drivers** — the PM100 sources current by applying BATT+ to the output pin.
The contactor coil sits between that pin and GND; it is **not** a ground-switching output.

| Contactor | PM100 Output | Pin | Driver Type |
|-----------|-------------|-----|-------------|
| Pre-charge contactor | RLY1 | J2-21 | Hi-side (sources 12V) |
| Main HV contactor | RLY2 | J2-7 | Hi-side (sources 12V) |

**Coil wiring for each contactor:**

```
J2-8 / J2-23   BATT+ (12V supply into PM100)
      |
  [PM100 internal hi-side switch — RLY1 or RLY2]
      |
J2-21 or J2-7  ────────────  Contactor coil (+) terminal
                             Contactor coil (−) terminal
                                   |
J2-6 / J2-14   GND ────────────────┘
```

- **To close contactor**: PM100 connects output pin to BATT+ → 12V across coil → contactor closes
- **To open contactor**: PM100 opens switch → pin floats → no current → contactor opens

⚠️ **Check coil current**: hi-side drivers are rated **1.5 A continuous**. Verify your chosen
contactor's coil draws less than this at 12V. The Tyco EV200AAANA (RMS p/n 77-0025) is confirmed
compatible.

### 2.2 3-Phase AC Output

Terminals **Phase A, Phase B, Phase C** connect to the motor windings. Order determines rotation
direction. Swapping any two phases reverses spin direction. Use shielded cable with metallic cable
glands into the inverter for EMI suppression.

### 2.3 Wiring Torque Values


| Connection                    | Allen Key | Torque |
| ----------------------------- | --------- | ------ |
| Wiring plug (main DC)         | 12 mm     | 3 Nm   |
| M6 clamp screws (phase wires) | 5 mm      | 5 Nm   |
| M25 cable gland               | 27 mm     | 6 Nm   |


---

## 3. J1 Connector — 35-Pin AMPSEAL

**Connector**: TE Connectivity / AMP AMPSEAL **776164-1** (35-position plug, IP67)
**Crimp contacts**: **770854-1** (16–20 AWG / 0.5–1.25 mm²)

This is the main signal interface connector — all analog inputs, digital inputs, serial communication,
and CAN bus are on J1.

### 3.1 Physical Layout

```
┌──────────────────────────────────────────────────────────────────────────────┐
│              J1 CONNECTOR FACE — LOOKING INTO THE PLUG YOU WIRE UP           │
│              (wire entry is behind this view; mating face shown)             │
│                                                                              │
│   NOTE: Locking latch is at the TOP of the connector body                    │
│                                                                              │
│  Row A ┌────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┐         │
│ (top)  │ 1  │ 2  │ 3  │ 4  │ 5  │ 6  │ 7  │ 8  │ 9  │ 10 │ 11 │ 12 │         │
│        └────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┘         │
│                                                                              │
│  Row B  ┌────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┐        │
│ (mid)   │ 13 │ 14 │ 15 │ 16 │ 17 │ 18 │ 19 │ 20 │ 21 │ 22 │ 23 │ 24 │        │
│         └────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┘        │
│                                                                              │
│  Row C ┌────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┐              │
│ (bot.) │ 25 │ 26 │ 27 │ 28 │ 29 │ 30 │ 31 │ 32 │ 33 │ 34 │ 35 │              │
│        └────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┘              │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Pin Assignments by Function Group

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                   J1 PINS GROUPED BY FUNCTION                                │
│                                                                              │
│  ── POWER SUPPLY (from PM100 to your sensors) ────────────────────────────   │
│  J1-1   XDCR_PWR  +5V @ 80mA max   Accel pedal power                         │
│  J1-14  XDCR_PWR  +5V @ 80mA max   Spare transducer power                    │
│  J1-26  XDCR_PWR  +5V @ 80mA max   Spare transducer power                    │
│  J1-28  XDCR_PWR  +5V @ 80mA max   Spare transducer power                    │
│  ⚠ Total XDCR_PWR current across ALL four pins combined: 80 mA max           │
│                                                                              │
│  ── ANALOG GROUND ────────────────────────────────────────────────────────   │
│  J1-2   AGND      Analog GND for accelerator pedal                           │
│  J1-15  AGND      Analog GND                                                 │
│  J1-17  AGND      Analog GND                                                 │
│  J1-19  AGND      Analog GND                                                 │
│  (Use AGND for signal returns, not J1-22 GND, to reduce noise)               │
│                                                                              │
│  ── POWER GROUND ─────────────────────────────────────────────────────────   │
│  J1-22  GND       Main power/logic ground                                    │
│                                                                              │
│  ── ANALOG INPUTS (0–5V, 12-bit ADC) ────────────────────────────────────    │
│  J1-13  AIN1      Accelerator pedal wiper (primary throttle input)           │
│  J1-24  AIN2      Spare                                                      │
│  J1-25  AIN3      Spare                                                      │
│  J1-3   AIN4      Motor thermistor (requires external pull-up to XDCR_PWR)   │
│                                                                              │
│  ── RTD TEMPERATURE INPUTS ───────────────────────────────────────────────   │
│  J1-4   RTD1      PT1000 (1000 Ω at 0°C)                                     │
│  J1-16  RTD2      PT1000 (1000 Ω at 0°C)                                     │
│  J1-27  RTD3      PT1000 (1000 Ω at 0°C)                                     │
│  J1-5   RTD4      PT100  (100 Ω at 0°C)                                      │
│  J1-6   RTD5      PT100  (100 Ω at 0°C)                                      │
│                                                                              │
│  ── DIGITAL INPUTS — Switch-To-Ground (STG) ──────────────────────────────   │
│  J1-30  DIN1  /FWD_ENA   Forward enable  (ground = forward drive)            │
│  J1-8   DIN2  /REV_ENA   Reverse enable  (ground = reverse drive)            │
│  J1-20  DIN3  /BRAKE_SW  Brake switch    (ground = brake pressed)            │
│  J1-31  DIN4             Unassigned (spare)                                  │
│  (/ prefix = active low: ground the pin to activate the function)            │
│                                                                              │
│  ── DIGITAL INPUTS — Switch-To-Battery (STB) ─────────────────────────────   │
│  J1-9   DIN5             Unassigned (spare STB input)                        │
│  J1-21  DIN6             Unassigned (spare STB input)                        │
│  (STB = apply 12V to activate; input sees 12V when switch closes to BATT+)   │
│                                                                              │
│  ── RS-232 SERIAL COMMUNICATION ──────────────────────────────────────────   │
│  J1-12  TXD    Transmit data  (FROM the PM100 TO your PC)                    │
│  J1-35  RXD    Receive data   (FROM your PC TO the PM100)                    │
│  J1-22  GND    Serial ground  (shared with power ground)                     │
│                                                                              │
│  ── BOOTLOADER ENABLE ────────────────────────────────────────────────────   │
│  J1-7   /PROG_ENA  Active-low — ground only during firmware flashing         │
│                    ⚠ NEVER ground this during normal operation               │
│                                                                              │
│  ── CAN BUS ───────────────────────────────────────────────────────────────  │
│  J1-11  CANA_L  CAN Channel A Low  (primary/only active CAN bus)             │
│  J1-33  CANA_H  CAN Channel A High                                           │
│  J1-34  CANB_L  CAN Channel B Low  (reserved, not active)                    │
│  J1-23  CANB_H  CAN Channel B High (reserved, not active)                    │
│                                                                              │
│  ── RESERVED (DO NOT CONNECT) ────────────────────────────────────────────   │
│  J1-10, J1-18, J1-29, J1-32                                                  │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 3.3 Complete Pin Reference Table


| Pin | Signal    | Dir | Function                  | Notes                                  |
| --- | --------- | --- | ------------------------- | -------------------------------------- |
| 1   | XDCR_PWR  | Out | +5V transducer power      | Accel pedal supply; 80 mA total        |
| 2   | AGND      | —   | Analog ground             | Use for sensor signal returns          |
| 3   | AIN4      | In  | Analog input 4 (0–5V)     | Motor thermistor; needs pull-up        |
| 4   | RTD1      | In  | PT1000 RTD input          | 1000 Ω at 0°C                          |
| 5   | RTD4      | In  | PT100 RTD input           | 100 Ω at 0°C                           |
| 6   | RTD5      | In  | PT100 RTD input           | 100 Ω at 0°C                           |
| 7   | /PROG_ENA | In  | Boot loader enable        | Active LOW; float for normal operation |
| 8   | DIN2      | In  | /REV_ENA — Reverse enable | STG; ground to reverse                 |
| 9   | DIN5      | In  | Spare digital input       | STB; apply 12V to activate             |
| 10  | —         | —   | RESERVED                  | Do NOT connect                         |
| 11  | CANA_L    | I/O | CAN A Low                 | CAN bus differential −                 |
| 12  | TXD       | Out | RS-232 transmit           | Signal out FROM PM100                  |
| 13  | AIN1      | In  | Analog input 1 (0–5V)     | Throttle/accelerator pedal wiper       |
| 14  | XDCR_PWR  | Out | +5V transducer power      | Spare; shares 80 mA budget             |
| 15  | AGND      | —   | Analog ground             |                                        |
| 16  | RTD2      | In  | PT1000 RTD input          |                                        |
| 17  | AGND      | —   | Analog ground             |                                        |
| 18  | —         | —   | RESERVED                  | Do NOT connect                         |
| 19  | AGND      | —   | Analog ground             |                                        |
| 20  | DIN3      | In  | /BRAKE_SW — Brake switch  | STG; ground when brake applied         |
| 21  | DIN6      | In  | Spare digital input       | STB; apply 12V to activate             |
| 22  | GND       | —   | Power/logic ground        | RS-232 GND also references this        |
| 23  | CANB_H    | —   | CAN B High                | Reserved — do not connect              |
| 24  | AIN2      | In  | Analog input 2 (0–5V)     | Spare                                  |
| 25  | AIN3      | In  | Analog input 3 (0–5V)     | Spare                                  |
| 26  | XDCR_PWR  | Out | +5V transducer power      | Spare; shares 80 mA budget             |
| 27  | RTD3      | In  | PT1000 RTD input          |                                        |
| 28  | XDCR_PWR  | Out | +5V transducer power      | Spare; shares 80 mA budget             |
| 29  | —         | —   | RESERVED                  | Do NOT connect                         |
| 30  | DIN1      | In  | /FWD_ENA — Forward enable | STG; ground to drive forward           |
| 31  | DIN4      | In  | Spare digital input       | STG; ground to activate                |
| 32  | —         | —   | RESERVED                  | Do NOT connect                         |
| 33  | CANA_H    | I/O | CAN A High                | CAN bus differential +                 |
| 34  | CANB_L    | —   | CAN B Low                 | Reserved — do not connect              |
| 35  | RXD       | In  | RS-232 receive            | Signal in TO PM100                     |


---

## 4. J2 Connector — 23-Pin AMPSEAL

**Connector**: TE Connectivity / AMP AMPSEAL **770680-1** (23-position plug, IP67)
**Crimp contacts**: **770854-1** (16–20 AWG / 0.5–1.25 mm²) — same as J1

J2 carries: 12V logic power, motor position feedback (resolver or encoder), and relay driver outputs.

### 4.1 Physical Layout

```
┌──────────────────────────────────────────────────────────────────────────────┐
│              J2 CONNECTOR FACE — LOOKING INTO THE PLUG YOU WIRE UP           │
│                                                                              │
│  Row A ┌────┬────┬────┬────┬────┬────┬────┬────┐                             │
│ (top)  │ 1  │ 2  │ 3  │ 4  │ 5  │ 6  │ 7  │ 8  │                             │
│        └────┴────┴────┴────┴────┴────┴────┴────┘                             │
│                                                                              │
│  Row B  ┌────┬────┬────┬────┬────┬────┬────┬────┐                            │
│ (mid)   │ 9  │ 10 │ 11 │ 12 │ 13 │ 14 │ 15 │ 16 │                            │
│         └────┴────┴────┴────┴────┴────┴────┴────┘                            │
│                                                                              │
│  Row C ┌────┬────┬────┬────┬────┬────┬────┐                                  │
│ (bot.) │ 17 │ 18 │ 19 │ 20 │ 21 │ 22 │ 23 │                                  │
│        └────┴────┴────┴────┴────┴────┴────┘                                  │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 Pin Assignments by Function Group

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                   J2 PINS GROUPED BY FUNCTION                                │
│                                                                              │
│  ── 12V LOGIC POWER INPUT (to PM100 from vehicle) ───────────────────────    │
│  J2-8   BATT+   12V ignition power (primary)                                 │
│  J2-23  BATT+   12V ignition power (redundant — wire both for high current)  │
│  J2-6   GND     12V return (primary)                                         │
│  J2-14  GND     12V return (redundant — wire both for high current)          │
│  ⚠ At 14V: ~1.6A draw (not counting relay coil currents)                     │
│                                                                              │
│  ── RELAY DRIVER OUTPUTS (from PM100) ────────────────────────────────────   │
│  J2-21  RLY1   Hi-Side Driver  → Pre-Charge Contactor (pulls to BATT+)       │
│  J2-7   RLY2   Hi-Side Driver  → Main Contactor / Relay (pulls to BATT+)     │
│  J2-15  RLY3   Lo-Side Driver  → OK / Status Indicator (pulls to GND)        │
│  J2-22  RLY4   Lo-Side Driver  → Fault Indicator (pulls to GND)              │
│  Max continuous current: 1.5A for hi-side; refer to datasheet for lo-side    │
│                                                                              │
│  ── ENCODER INPUTS (for induction motors) ───────────────────────────────    │
│  J2-1   XDCR_PWR  +5V encoder power supply                                   │
│  J2-10  GND       Encoder ground                                             │
│  J2-9   ENCA      Encoder Channel A (quadrature)                             │
│  J2-16  ENCB      Encoder Channel B (quadrature)                             │
│  J2-2   ENCZ      Encoder Channel Z (index / one-per-rev pulse)              │
│                                                                              │
│  ── RESOLVER INPUTS (for permanent magnet synchronous motors) ─────────────  │
│  J2-17  EXC    Resolver excitation output (+) — PM100 drives this            │
│  J2-3   /EXC   Resolver excitation return (−) — differential excitation      │
│  J2-11  SIN    Resolver sine winding (+) — position feedback                 │
│  J2-18  /SIN   Resolver sine winding (−) — differential                      │
│  J2-4   COS    Resolver cosine winding (+) — position feedback               │
│  J2-12  /COS   Resolver cosine winding (−) — differential                    │
│  J2-19  GND    Resolver cable shield ground                                  │
│                                                                              │
│  ── RESERVED (DO NOT CONNECT) ────────────────────────────────────────────   │
│  J2-5, J2-13, J2-20                                                          │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 4.3 Complete Pin Reference Table


| Pin | Signal   | Dir | Function                                    |
| --- | -------- | --- | ------------------------------------------- |
| 1   | XDCR_PWR | Out | +5V encoder supply                          |
| 2   | ENCZ     | In  | Encoder index (one pulse per revolution)    |
| 3   | /EXC     | Out | Resolver excitation return (differential −) |
| 4   | COS      | In  | Resolver cosine winding +                   |
| 5   | —        | —   | RESERVED — do not connect                   |
| 6   | GND      | —   | 12V logic power return (primary)            |
| 7   | RLY2     | Out | Hi-side relay driver — Main contactor       |
| 8   | BATT+    | In  | 12V ignition power (primary)                |
| 9   | ENCA     | In  | Encoder channel A                           |
| 10  | GND      | —   | Encoder signal ground                       |
| 11  | SIN      | In  | Resolver sine winding +                     |
| 12  | /COS     | In  | Resolver cosine winding −                   |
| 13  | —        | —   | RESERVED — do not connect                   |
| 14  | GND      | —   | 12V logic power return (redundant)          |
| 15  | RLY3     | Out | Lo-side relay driver — OK indicator         |
| 16  | ENCB     | In  | Encoder channel B                           |
| 17  | EXC      | Out | Resolver excitation output + (12 kHz)       |
| 18  | /SIN     | In  | Resolver sine winding −                     |
| 19  | GND      | —   | Resolver shield ground                      |
| 20  | —        | —   | RESERVED — do not connect                   |
| 21  | RLY1     | Out | Hi-side relay driver — Pre-charge contactor |
| 22  | RLY4     | Out | Lo-side relay driver — Fault indicator      |
| 23  | BATT+    | In  | 12V ignition power (redundant — wire both)  |


### 4.4 Relay Driver Wiring

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                     RELAY DRIVER CIRCUITS                                    │
│                                                                              │
│  HI-SIDE DRIVER (RLY1, RLY2) — PM100 connects load between BATT+ and load    │
│                                                                              │
│       J2-8/J2-23                                                             │
│        BATT+ ──────┬─────────────────────────────────────────────────-─┐     │
│                    │                                                   │     │
│                  [Load+]  e.g. contactor coil positive terminal        │     │
│                  [Load−]                                               │     │
│                    │                                                   │     │
│        J2-21/J2-7──┘  ← PM100 switches this line ON/OFF                │     │
│         RLY1/RLY2        (pulls load+ to BATT+ through switch)         │     │
│        J2-6/J2-14                                                      │     │
│          GND ──────────────────── Contactor coil return ───────────────┘     │
│                                                                              │
│  LO-SIDE DRIVER (RLY3, RLY4) — PM100 connects load between load and GND      │
│                                                                              │
│       J2-8/J2-23                                                             │
│        BATT+ ──────────────────── LED/Indicator positive terminal            │
│                                                                              │
│        J2-15/J2-22                                                           │
│         RLY3/RLY4 ───── LED/Indicator negative terminal ──── to GND via PM   │
│                                   (PM100 switches path to GND)               │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. Input/Output Electrical Details

### 5.1 Analog Inputs (AIN1–AIN4)

Used for continuous voltage signals from sensors (pedal position, temperature, etc.)

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                  ANALOG INPUT INTERNAL CIRCUIT (AIN1–AIN3)                   │
│                                                                              │
│  XDCR_PWR (+5V)                                                              │
│       │                                                                      │
│     300 kΩ   ← weak pull-up keeps the line from floating when unconnected    │
│       │                                                                      │
│  AINx ●───────────────────── to 12-bit ADC on DSP                            │
│       │              │                                                       │
│     1000 pF        4700 pF  ← low-pass RC filter (removes high-freq noise)   │
│       │              │                                                       │
│      GND            GND                                                      │
│                                                                              │
│  Input range: 0 to 5.00 V    Accuracy: ±5% gain, ±50 mV offset               │
│                                                                              │
│  ─────────────────────────────────────────────────────────────────────────   │
│                  AIN4 — MOTOR THERMISTOR WIRING                              │
│                                                                              │
│  XDCR_PWR ───── Rpull ────────── AIN4 ──── to DSP ADC                        │
│   (+5V)     (external resistor,   │                                          │
│              e.g. 2.2 kΩ)         │                                          │
│                              [Thermistor]  ← NTC type; resistance drops      │
│                                   │          as temperature rises            │
│                                  GND                                         │
│                                                                              │
│  As temperature rises → thermistor resistance drops → AIN4 voltage drops     │
└──────────────────────────────────────────────────────────────────────────────┘
```

**Typical accelerator pedal wiring (potentiometer)**:

```
  J1-1 (XDCR_PWR +5V) ──── Pedal pot wiper supply end
  J1-13 (AIN1)         ──── Pedal pot wiper (middle terminal)
  J1-2 (AGND)          ──── Pedal pot return end
```


| Parameter      | Value                                     |
| -------------- | ----------------------------------------- |
| Input range    | 0–5.00 V                                  |
| ADC resolution | 12-bit (1.22 mV/step)                     |
| Gain accuracy  | ±5%                                       |
| Offset         | ±50 mV typical                            |
| Pull-up to +5V | 300 kΩ (weak, mainly for fault detection) |


### 5.2 RTD Temperature Inputs

RTDs (Resistance Temperature Detectors) are precision resistors whose resistance changes predictably
with temperature. The PM100 supports two standard types.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                     RTD INPUT TYPES                                          │
│                                                                              │
│  PT100  (100 Ω at 0°C):  100 Ω + (0.385 Ω/°C) × temp                         │
│    At  25°C ≈ 109.7 Ω    ← J1-5 (RTD4), J1-6 (RTD5)                          │
│    At 100°C ≈ 138.5 Ω                                                        │
│                                                                              │
│  PT1000 (1000 Ω at 0°C): 1000 Ω + (3.85 Ω/°C) × temp                         │
│    At  25°C ≈ 1097 Ω     ← J1-4 (RTD1), J1-16 (RTD2), J1-27 (RTD3)           │
│    At 100°C ≈ 1385 Ω                                                         │
│                                                                              │
│  Accuracy: ±3°C at 25°C, ±3°C additional over measurement range              │
│                                                                              │
│  WIRING: Two-wire connection — one wire to the RTD pin, one to AGND.         │
│  Use twisted pair or shielded wire for best noise rejection.                 │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 5.3 Digital Inputs

The PM100 has two families of digital input, depending on whether the switch connects to ground
or to battery positive.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│            DIGITAL INPUT TYPES — SWITCH-TO-GROUND (STG)                      │
│            DIN1 (J1-30), DIN2 (J1-8), DIN3 (J1-20), DIN4 (J1-31)             │
│                                                                              │
│  INTERNAL TO PM100              EXTERNAL WIRING                              │
│                                                                              │
│  +5V                                                                         │
│   │                                                                          │
│  2.43 kΩ                                                                     │
│   │                                                                          │
│  DINx ●──────────────────────── Switch ──── GND (J1-22)                      │
│   │                               ↑                                          │
│  41.7 kΩ                   Any switch that                                   │
│   │                         grounds this pin                                 │
│  GND                        (relay, pushbutton,                              │
│                              hall-effect, etc.)                              │
│                                                                              │
│  OPEN (switch not closed) → pin pulled HIGH to +5V → input = INACTIVE        │
│  CLOSED (switch to GND) → pin pulled LOW to 0V → input = ACTIVE              │
│                                                                              │
│  Threshold: ON < 0.9V  |  OFF > 4.2V   Max input voltage: 18V                │
│                                                                              │
│ ───────────────────────────────────────────────────────────────────────────  │
│            DIGITAL INPUT TYPES — SWITCH-TO-BATTERY (STB)                     │
│            DIN5 (J1-9), DIN6 (J1-21)                                         │
│                                                                              │
│  INTERNAL TO PM100              EXTERNAL WIRING                              │
│                                                                              │
│  +3.3V                                                                       │
│   │                                                                          │
│  10 kΩ                                                                       │
│   │                                                                          │
│  DINx ●──────────────────────── Switch ──── BATT+ (12V)                      │
│   │                                                                          │
│  4.7 kΩ                                                                      │
│   │                                                                          │
│  10 kΩ                                                                       │
│   │                                                                          │
│  GND                                                                         │
│                                                                              │
│  OPEN (no 12V) → pin pulled LOW → input = INACTIVE                           │
│  CLOSED (12V applied) → pin pulled HIGH → input = ACTIVE                     │
│                                                                              │
│  Threshold: ON > 2.5V  |  OFF < 1.3V   Max input voltage: 18V                │
└──────────────────────────────────────────────────────────────────────────────┘
```

**Default digital input assignments:**


| Pin   | Signal | Default Assignment        | How to Activate |
| ----- | ------ | ------------------------- | --------------- |
| J1-30 | DIN1   | /FWD_ENA — Forward Enable | Ground to J1-22 |
| J1-8  | DIN2   | /REV_ENA — Reverse Enable | Ground to J1-22 |
| J1-20 | DIN3   | /BRAKE_SW — Brake Switch  | Ground to J1-22 |
| J1-31 | DIN4   | Unassigned                | Ground to J1-22 |
| J1-9  | DIN5   | Unassigned                | Apply 12V       |
| J1-21 | DIN6   | Unassigned                | Apply 12V       |


⚠ **Never activate both /FWD_ENA and /REV_ENA simultaneously** — this is a conflict and will
prevent the inverter from running.

### 5.4 Motor Position Feedback

The PM100 uses different feedback sensors depending on motor type:

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                  MOTOR FEEDBACK SENSOR SELECTION                             │
│                                                                              │
│  INDUCTION MOTOR → Incremental Quadrature Encoder                            │
│  ┌─────────────────────────────────────────────────────────────────────┐     │
│  │   PM100 J2         Cable          Encoder                           │     │
│  │   J2-1  XDCR_PWR ─────────────── +5V supply                         │     │
│  │   J2-10 GND      ─────────────── Ground                             │     │
│  │   J2-9  ENCA     ─────────────── Channel A (quadrature square wave) │     │
│  │   J2-16 ENCB     ─────────────── Channel B (quadrature, 90° offset) │     │
│  │   J2-2  ENCZ     ─────────────── Index (1 pulse per revolution)     │     │
│  │                                                                     │     │
│  │  A vs B phase relationship tells the DSP which direction motor spins│     │
│  │  Index Z resets position counter once per revolution                │     │
│  └─────────────────────────────────────────────────────────────────────┘     │
│                                                                              │
│  PM SYNCHRONOUS MOTOR → Resolver (preferred for EVs — robust to vibration)   │
│  ┌─────────────────────────────────────────────────────────────────────┐     │
│  │   PM100 J2         Shielded Cable    Resolver (inside motor)        │     │
│  │   J2-17 EXC   ───────────────────── Reference winding (+)           │     │
│  │   J2-3  /EXC  ───────────────────── Reference winding (−)           │     │
│  │   J2-11 SIN   ───────────────────── Sine output winding (+)         │     │
│  │   J2-18 /SIN  ───────────────────── Sine output winding (−)         │     │
│  │   J2-4  COS   ───────────────────── Cosine output winding (+)       │     │
│  │   J2-12 /COS  ───────────────────── Cosine output winding (−)       │     │
│  │   J2-19 GND   ───────────────────── Cable shield                    │     │
│  │                                                                     │     │
│  │  PM100 excites the reference winding at 12 kHz                      │     │
│  │  SIN and COS outputs vary with rotor angle:                         │     │
│  │    SIN output ∝ sin(rotor angle) × sin(12 kHz)                      │     │
│  │    COS output ∝ cos(rotor angle) × sin(12 kHz)                      │     │
│  │  DSP demodulates these to determine exact rotor position            │     │
│  │  Use SHIELDED twisted pairs for all 3 wire pairs                    │     │
│  └─────────────────────────────────────────────────────────────────────┘     │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 6. RS-232 Serial Interface

RS-232 is a point-to-point serial standard used here for bench service and diagnostics. It is NOT
a network — only one device can be connected to the PM100 at a time.

### 6.1 Serial Parameters


| Parameter      | Value                                  |
| -------------- | -------------------------------------- |
| Baud rate      | **57,600 bps**                         |
| Data bits      | 8                                      |
| Parity         | None                                   |
| Stop bits      | 1                                      |
| Flow control   | None (no RTS/CTS)                      |
| Voltage levels | RS-232 (±12V logic, inverted from TTL) |


### 6.2 Wiring: J1 to PC DB9 (or USB-to-Serial Adapter)

```
┌──────────────────────────────────────────────────────────────────────────────┐
│               RS-232 WIRING: PM100 J1 → PC DB9 FEMALE                        │
│                                                                              │
│   PM100 J1                 Cable (3 wires)             PC / USB Adapter      │
│                                                                              │
│   J1-12  TXD  ──────────────────────────────────────  DB9 Pin 2  RXD         │
│               → signal flows FROM the PM100 TO the PC                        │
│                                                                              │
│   J1-35  RXD  ──────────────────────────────────────  DB9 Pin 3  TXD         │
│               ← signal flows FROM the PC TO the PM100                        │
│                                                                              │
│   J1-22  GND  ──────────────────────────────────────  DB9 Pin 5  GND         │
│               ← common reference                                             │
│                                                                              │
│   ⚠ TX and RX are ALWAYS crossed — the transmitter of one end                │
│     connects to the receiver of the other end.                               │
│     This is called a "null modem" connection.                                │
│                                                                              │
│   ─── FOR FIRMWARE FLASHING ONLY ──────────────────────────────────────      │
│   J1-7  /PROG_ENA  ─────────  J1-22 GND  (temporary jumper only)             │
│               ← ground ONLY while flashing, remove afterward                 │
└──────────────────────────────────────────────────────────────────────────────┘
```

```
┌──────────────────────────────────────────────────────────────────────────────┐
│              DB9 FEMALE SOCKET — PIN LAYOUT                                  │
│              (looking at the face of the socket)                             │
│                                                                              │
│         ┌──────────────────────────────────────────────┐                     │
│        /                                                 \                   │
│       │    (1)     (2)     (3)     (4)     (5)            │                  │
│       │         (6)     (7)     (8)     (9)               │                  │
│        \                                                 /                   │
│         └──────────────────────────────────────────────┘                     │
│                                                                              │
│  Pin  Signal  Function               PM100 Connection                        │
│  ───  ──────  ─────────────────────  ──────────────────────────────────────  │
│   1   DCD     Data Carrier Detect    not connected                           │
│   2   RXD     Receive Data           ← J1-12 TXD  (data from PM100)          │
│   3   TXD     Transmit Data          → J1-35 RXD  (data to PM100)            │
│   4   DTR     Data Terminal Ready    not connected                           │
│   5   GND     Signal Ground          ── J1-22 GND  (common reference)        │
│   6   DSR     Data Set Ready         not connected                           │
│   7   RTS     Request To Send        not connected                           │
│   8   CTS     Clear To Send          not connected                           │
│   9   RI      Ring Indicator         not connected                           │
│                                                                              │
│  Only pins 2, 3, and 5 are wired for PM100 communication.                    │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 6.3 Testing the Serial Connection

**On macOS:**

```bash
# Step 1: List all serial ports — look for your USB-to-serial adapter
ls /dev/cu.*
# Expected when adapter connected: /dev/cu.usbserial-XXXXXXXX
# If only /dev/cu.Bluetooth-Incoming-Port appears: adapter not detected

# Step 2: Open the serial port (use the device name from Step 1)
screen /dev/cu.usbserial-FTES73H7 57600
# Press Ctrl+A then K to exit screen

# Step 3: Loopback test (proves your adapter works, before connecting to PM100)
# Physically short TX to RX pins on the adapter's DB9 connector, then:
screen /dev/cu.usbserial-FTES73H7 57600
# Type characters — they should echo back if loopback is working
```

**On Windows 10:**

> For identifying the COM port and configuring the baud rate, see [§8.1 Connecting on Windows](#81-connecting-on-windows).

1. Identify the COM port and set baud rate to 57600 as described in §8.1.
2. Open RMS GUI and select the COM port → verifies real communication.
3. For adapter testing, short DB9 pins 2–3 and use a terminal (e.g., PuTTY at 57600 8N1).

⚠ **Seeing no text in a plain terminal does NOT mean failure.** The PM100 uses a proprietary protocol
— raw serial output is not plain ASCII. Use RMS GUI for real functional testing.

---

## 7. CAN Bus Interface

CAN (Controller Area Network) is a differential serial bus that supports multiple devices on a single
twisted pair. In an EV, CAN is typically how the vehicle controller commands the inverter.

### 7.1 Physical Layer

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        CAN BUS TOPOLOGY                                      │
│                                                                              │
│  The CAN bus is a single terminated cable run (NOT a star topology).         │
│                                                                              │
│                         120 Ω                     120 Ω                      │
│                    ┌─── term ───┐           ┌─── term ───┐                   │
│                    │            │           │            │                   │
│  ┌──────────┐    ┌─┴────────┐  twisted    ┌─┴────────┐  twisted  ┌────────┐  │
│  │   PM100  │    │  Vehicle │  pair       │  BMS     │  pair     │Battery │  │
│  │          │    │Controller│ ─────────── │          │ ───────── │Monitor │  │
│  │  J1-33   ├────┤  CANH    ├────────────-┤  CANH    ├───────────┤  etc.  │  │
│  │  CANA_H  │    │  CANL    │             │  CANL    │           │        │  │
│  │  J1-11   ├────┤          │             │          │           │        │  │
│  │  CANA_L  │    └──────────┘             └──────────┘           └────────┘  │
│  └──────────┘                                                                │
│                                                                              │
│  KEY RULES:                                                                  │
│  • Termination: exactly TWO 120 Ω resistors, one at each physical end        │
│  • Do NOT add 120 Ω if PM100 software termination is enabled                 │
│  • No star wiring — short stub connections only                              │
│  • Twisted pair required — do not use untwisted wire                         │
│  • Default bus speed: 250 kbps (software selectable: 125/250/500/1000 kbps)  │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 7.2 Message Overview

The PM100 uses CAN 2.0A (standard 11-bit IDs). The **base address** is 0x0A0 by default (adjustable
via EEPROM). All messages are 8 bytes.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                     CAN MESSAGE MAP (default base = 0x0A0)                   │
│                                                                              │
│  DIRECTION: PM100 → VEHICLE (broadcast messages)                             │
│  PM100 continuously transmits these at ~10 Hz without being asked            │
│                                                                              │
│  ID 0x0A0  │ Temperatures #1   │ IGBT/transistor temperatures                │
│  ID 0x0A2  │ Temperatures #2   │ Additional thermal data                     │
│  ID 0x0A4  │ Motor position    │ Flux, rotor position data                   │
│  ID 0x0A6  │ Motor currents    │ IQ and ID feedback and commands             │
│  ID 0x0A8  │ Voltage/current   │ DC bus voltage, DC link current             │
│  ID 0x0AA  │ Internal states   │ VSM state, inverter state                   │
│  ID 0x0AC  │ Fault codes       │ Active fault flags and status bits          │
│                                                                              │
│  DIRECTION: VEHICLE → PM100 (command messages)                               │
│  Send at regular intervals — PM100 will disable if no command received       │
│                                                                              │
│  ID 0x0C0  │ Motor Command     │ Torque, speed, direction, enable            │
│                                                                              │
│  DIRECTION: VEHICLE ↔ PM100 (parameter access)                               │
│  ID 0x0C8  │ Read/Write Request│ Vehicle sends to read or write a parameter  │
│  ID 0x0C9  │ Read/Write Reply  │ PM100 responds with value or ack            │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 7.3 Command Message (0x0C0) — Controlling the Motor

This is the most important message in CAN mode. Send at regular intervals (at least 10 Hz).

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                   CAN COMMAND MESSAGE ID 0x0C0 (8 bytes)                     │
│                                                                              │
│  Byte 0–1  │ Torque Command        │ Signed 16-bit int, units: Nm × 10       │
│            │                       │ e.g. 100 Nm → send 0x03E8               │
│            │                       │ Negative = regen torque request         │
│                                                                              │
│  Byte 2–3  │ Speed Command         │ Signed 16-bit int, units: RPM           │
│            │                       │ Used in speed control mode only         │
│                                                                              │
│  Byte 4    │ Direction + Enable    │ Bit 0: Direction (0=CW, 1=CCW)          │
│            │                       │ Bit 1: Inverter Enable (1=enabled)      │
│            │                       │ Must be 0x01 (fwd) or 0x03 (rev+ena)    │
│                                                                              │
│  Byte 5    │ Relay Control         │ Set to 0x00 for normal torque control   │
│            │                       │ 0x55 activates relay control mode       │
│                                                                              │
│  Byte 6–7  │ Torque Limit          │ Signed 16-bit, Nm × 10                  │
│            │                       │ Set to 0x0000 if not using (fw v1953+)  │
│                                                                              │
│  EXAMPLE — command 50 Nm forward, inverter enabled:                          │
│    Byte 0: 0x01  Byte 1: 0xF4   → Torque = 0x01F4 = 500 → 50.0 Nm            │
│    Byte 2: 0x00  Byte 3: 0x00   → Speed = 0 (unused in torque mode)          │
│    Byte 4: 0x03                  → Direction=fwd, Enable=1                   │
│    Byte 5: 0x00                  → Normal torque control                     │
│    Byte 6: 0x00  Byte 7: 0x00   → No torque limit override                   │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 8. RMS GUI Software

RMS GUI is a Windows application that communicates with the PM100 over RS-232 at **57,600 baud**.
It is the primary service tool for bench testing, configuration, and diagnostics.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        RMS GUI — WHAT IT DOES                                │
│                                                                              │
│  ┌─────────────────┐   57,600 baud RS-232   ┌─────────────────────────┐      │
│  │   Windows PC    │ ◄═══════════════════►  │      PM100 Inverter     │      │
│  │                 │                        │                         │      │
│  │  RMS GUI app    │  ← live parameter data │  broadcasts 20 params   │      │
│  │  ─────────────  │                        │  at 3 Hz in datalog mode│      │
│  │  Memory View    │  ← read EEPROM values  │                         │      │
│  │  Fault Status   │  → write EEPROM values │  reads/writes EEPROM    │      │
│  │  COM port scan  │  ← firmware version    │  on request             │      │
│  │  Save/Load cfg  │                        │                         │      │
│  └─────────────────┘                        └─────────────────────────┘      │
│                                                                              │
│  WHAT RMS GUI SHOWS:                                                         │
│  • DC bus voltage and current                                                │
│  • Motor RPM and torque                                                      │
│  • IGBT and motor temperatures                                               │
│  • IQ / ID current feedback (torque and flux components)                     │
│  • VSM state (see Section 10)                                                │
│  • Active fault codes                                                        │
│  • Firmware version number                                                   │
│                                                                              │
│  WHAT RMS GUI CANNOT DO:                                                     │
│  • Flash firmware (use C2Prog instead)                                       │
│  • Control the motor (use CAN or VSM mode with real switches)                │
│                                                                              │
│  DOWNLOAD: https://www.cascadiamotion.com/documentation                      │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 8.1 Connecting on Windows

**Step 1 — Identify the COM port**

1. Plug in the USB-to-serial adapter.
2. Open **Device Manager** (Win + X → Device Manager).
3. Expand **Ports (COM & LPT)**.
4. Note the COM port assigned to your adapter (e.g. `COM3`).

> If the adapter does not appear under Ports, the driver is not installed. Check the adapter manufacturer's website for a Windows driver.

**Step 2 — Set the baud rate in the port settings**

Windows USB-serial adapters default to **9600 baud**. Change this to match the PM100:

1. In Device Manager, right-click the COM port → **Properties**.
2. Select the **Port Settings** tab.
3. Set **Bits per second** to **57600**.
4. Confirm Data bits = 8, Parity = None, Stop bits = 1, Flow control = None.
5. Click **OK**.

**Step 3 — Configure RMS GUI**

1. Open the `comport.ini` file in the RMS GUI folder.
2. Enter the COM port identifier (e.g. `COM3`) and save.
3. Launch RMS GUI — it should connect and begin displaying live parameters.

---

## 9. C2Prog Firmware Flashing

C2Prog is a third-party tool (codeskin.com) that writes firmware `.hex` files to the PM100's DSP
via RS-232. It uses the TI serial bootloader built into the DSP chip.

### 9.1 DSP Target Selection


| Your Hardware                      | C2Prog Target Setting |
| ---------------------------------- | --------------------- |
| Hardware version starts with "234" | `28234_30MHz`         |
| Floating-point variant (rare)      | `28335_30MHz`         |


The hardware version number is on a sticker or readable through RMS GUI. If unsure, default to
`28234_30MHz`.

### 9.2 Firmware Groups


| Firmware Group | Motor Type Numbers | Use When                |
| -------------- | ------------------ | ----------------------- |
| Group_1        | Motor types 0–59   | Standard configurations |
| Group_2        | Motor types 60+    | Extended motor profiles |


### 9.3 Step-by-Step Flashing Procedure

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                    FIRMWARE FLASHING FLOWCHART                               │
│                                                                              │
│    START                                                                     │
│      │                                                                       │
│      ▼                                                                       │
│  Connect RS-232 cable (J1-12, J1-35, J1-22 to DB9)                           │
│      │                                                                       │
│      ▼                                                                       │
│  Launch C2Prog → Configure Ports → Scan Ports → select your COM port         │
│      │                                                                       │
│      ▼                                                                       │
│  Set Target: 28234_30MHz (or 28335_30MHz if hardware requires)               │
│      │                                                                       │
│      ▼                                                                       │
│  Browse to the firmware .hex file                                            │
│      │                                                                       │
│      ▼                                                                       │
│  ⚠ TURN OFF 12V power to PM100 (remove BATT+ from J2-8/J2-23)                │
│      │                                                                       │
│      ▼                                                                       │
│  ⚠ GROUND /PROG_ENA (connect J1-7 to J1-22 GND with a jumper wire)           │
│      │                                                                       │
│      ▼                                                                       │
│  APPLY 12V power to PM100 → DSP enters bootloader mode                       │
│      │                                                                       │
│      ▼                                                                       │
│  C2Prog begins flashing → watch progress bar                                 │
│      │                                                                       │
│      ▼                                                                       │
│  Click OK when complete                                                      │
│      │                                                                       │
│      ▼                                                                       │
│  ⚠ TURN OFF 12V power again                                                  │
│      │                                                                       │
│      ▼                                                                       │
│  ⚠ REMOVE the /PROG_ENA jumper (unground J1-7)                               │
│      │                                                                       │
│      ▼                                                                       │
│  Apply 12V power normally → PM100 boots new firmware                         │
│      │                                                                       │
│      ▼                                                                       │
│  Verify with RMS GUI: check firmware version in the interface                │
│      │                                                                       │
│      ▼                                                                       │
│    DONE                                                                      │
└──────────────────────────────────────────────────────────────────────────────┘
```

⚠ **Critical warnings**:

- Never leave /PROG_ENA (J1-7) grounded during normal operation — the inverter will not run.
- Do not interrupt 12V power mid-flash — this can corrupt firmware and may require factory recovery.
- Newer command-line alternatives: `c2p-cli` (current) or `C2ProgShell.exe` (legacy) — same pin procedure applies.

---

## 10. Vehicle State Machine (VSM)

The VSM is firmware logic inside the PM100 that manages the drive enable sequence. It protects the
vehicle and inverter from improper state transitions. You can observe the current VSM state in
RMS GUI or via CAN message 0x0AA.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                      VSM STATE MACHINE DIAGRAM                               │
│                                                                              │
│   ┌──────────┐                                                               │
│   │  POWER   │  12V applied to J2-8/J2-23                                    │
│   │   ON     │                                                               │
│   └────┬─────┘                                                               │
│        │                                                                     │
│        ▼                                                                     │
│   ┌──────────┐                                                               │
│   │  START   │  State 0 — POST (Power-On Self Test)                          │
│   │  (Init)  │  Tests internal hardware                                      │
│   └────┬─────┘  POST fault = 2 quick LED blinks                              │
│        │                                                                     │
│        ▼                                                                     │
│   ┌──────────┐                                                               │
│   │PRE-CHARGE│  States 1–3 — RLY1 closes pre-charge contactor                │
│   │ 1, 2, 3  │  Monitors DC bus voltage rise                                 │
│   └────┬─────┘  RLY2 closes main contactor when voltage is sufficient        │
│        │        RLY1 opens pre-charge contactor                              │
│        ▼                                                                     │
│   ┌──────────┐                                                               │
│   │   WAIT   │  State 4 — Watching for enable signals                        │
│   │          │  /FWD_ENA or /REV_ENA not yet active                          │
│   └────┬─────┘                                                               │
│        │  /FWD_ENA or /REV_ENA asserted (grounded)                           │
│        ▼                                                                     │
│   ┌──────────┐                                                               │
│   │  READY   │  State 5 — Inverter enabled, waiting for torque command       │
│   │          │  RLY3 (OK indicator) activates                                │
│   └────┬─────┘                                                               │
│        │  Torque/speed command received (pedal pressed or CAN command)       │
│        ▼                                                                     │
│   ┌──────────┐                                                               │
│   │  MOTOR   │  State 6 — Normal operation                                   │
│   │ RUNNING  │  PWM output active, motor being driven                        │
│   └────┬─────┘                                                               │
│        │  Fault detected (over-temp, over-current, DC fault, etc.)           │
│        ▼                                                                     │
│   ┌──────────┐                                                               │
│   │  FAULT   │  State 7 — All PWM output disabled                            │
│   │          │  RLY4 (fault indicator) activates                             │
│   │          │  Single LED blink pattern                                     │
│   └────┬─────┘                                                               │
│        │  Power cycle required to clear most faults                          │
│        ▼                                                                     │
│   ┌──────────┐                                                               │
│   │ SHUTDOWN │  States 14–15 — Controlled shutdown sequence                  │
│   └──────────┘                                                               │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 11. Regenerative Braking

Regenerative braking converts kinetic energy back into electrical energy when decelerating. It is
a key EV feature but is often misunderstood in terms of the physical current path.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                      REGENERATIVE BRAKING ENERGY FLOW                        │
│                                                                              │
│  MOTORING (normal drive):                                                    │
│                                                                              │
│   Battery  ═══[DC+]══► DC Bus ══► IGBT Bridge ══► Phase A/B/C ══► Motor      │
│   (source)             (500µF)    (PWM inverter)  (3-phase AC)   (load)      │
│                                                                              │
│  REGENERATING (braking / deceleration):                                      │
│                                                                              │
│   Battery  ◄══[DC+]═══ DC Bus ◄══ IGBT Bridge ◄══ Phase A/B/C ◄══ Motor      │
│   (sink)              (500µF)    (operates as    (3-phase AC)   (now a       │
│                                   a rectifier)                  generator)   │
│                                                                              │
│  KEY POINT: The motor does NOT connect directly to the battery.              │
│  ALL energy flows through the inverter's DC bus.                             │
│                                                                              │
│  REGEN PATH REQUIRES:                                                        │
│  1. Main HV contactor closed (DC bus connected to battery)                   │
│  2. Battery BMS accepting charge (not full, temperature OK, etc.)            │
│  3. PM100 regen torque limit parameter allowing it (EEPROM 0x0111)           │
│  4. Negative torque command (CAN mode) or /BRAKE_SW active + pedal (VSM)     │
│                                                                              │
│  THE INVERTER CONTROLS EVERYTHING — the motor has no direct battery path     │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 12. Key Configuration Parameters

These EEPROM parameters are read/written via RMS GUI or CAN parameter messages. Values persist
through power cycles.


| Address | Parameter          | Description                                               | Typical Value                              |
| ------- | ------------------ | --------------------------------------------------------- | ------------------------------------------ |
| 0x011B  | Inv_Cmd_Mode       | **0** = CAN control; **1** = Analog/switch (VSM)          | 1 for standalone; 0 for vehicle controller |
| 0x0116  | Run_Mode           | **0** = Torque control; **1** = Speed control (demo only) | 0                                          |
| 0x012B  | Key_Switch_Mode    | **0** = Simple on/off; **1** = Ignition + START signal    | 0                                          |
| 0x0110  | Motor_Torque_Limit | Maximum motoring torque (Nm × 10)                         | Application-specific                       |
| 0x0111  | Regen_Torque_Limit | Maximum regen braking torque (Nm × 10)                    | Application-specific                       |
| 0x0107  | Pedal_Lo           | Pedal voltage at which regen begins (V × 100)             | Tune on bench                              |
| 0x010C  | Pedal_Hi           | Pedal voltage at which full torque is reached (V × 100)   | Tune on bench                              |
| 0x011A  | Gamma_Adjust       | Motor magnetic alignment offset (degrees × 10, ±3599)     | Motor-specific                             |
| 0x0119  | Motor_Type         | Motor profile selection (0–255)                           | Motor-specific                             |
| 0x0106  | Inv_OverTemp_Limit | Inverter over-temp shutdown threshold (°C × 10)           | Manufacturer default                       |
| 0x0121  | Mtr_OverTemp_Limit | Motor over-temp shutdown threshold (°C × 10)              | Motor-specific                             |
| 0x0104  | DC_Volt_Limit      | Maximum DC bus voltage before fault (V)                   | ~450V for 400V systems                     |


**CAN-specific parameters:**


| Parameter Name                    | Description                                             |
| --------------------------------- | ------------------------------------------------------- |
| CAN_ID_Offset_EEPROM              | Shifts all CAN IDs by this offset (default: 0x0A0 base) |
| CAN_Bit_Rate_EEPROM               | Bus speed: 125, 250, 500, or 1000 kbps                  |
| CAN_Term_Res_Present_EEPROM       | Enable internal 120 Ω terminator on CAN A               |
| CAN_Command_Message_Active_EEPROM | Enable reception of 0x0C0 command messages              |


---

## 13. Glossary


| Term                             | Definition                                                                                                                                                                          |
| -------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **AGND**                         | Analog Ground — the reference return for low-level sensor signals. Kept separate from power ground to reduce noise interference on signal measurements.                             |
| **AMPSEAL**                      | A family of sealed automotive connectors from TE Connectivity (formerly AMP). They are weatherproof (IP67) and designed for harsh environments.                                     |
| **BMS**                          | Battery Management System — electronics that monitor and protect the high-voltage battery pack (state of charge, cell balancing, over-temperature, etc.).                           |
| **CAN**                          | Controller Area Network — a serial communication bus standard (ISO 11898) widely used in vehicles. Differential pair (CAN_H and CAN_L), very robust to electrical noise.            |
| **CAN 2.0A**                     | The standard frame format of CAN, using 11-bit message identifiers. Allows up to 2048 unique message IDs.                                                                           |
| **DC Bus**                       | The high-voltage DC line inside the inverter that connects the battery pack to the IGBT switching stage. Contains a large capacitor (~500 µF in the PM100).                         |
| **DSP**                          | Digital Signal Processor — a specialized processor optimized for math-intensive control algorithms. The PM100 uses a TI TMS320F28234.                                               |
| **EEPROM**                       | Electrically Erasable Programmable Read-Only Memory — non-volatile storage where configuration parameters are saved and survive power cycles.                                       |
| **Encoder (quadrature)**         | A position feedback sensor with two square-wave outputs (A and B) 90° out of phase, allowing the controller to determine shaft position and direction. Used with induction motors.  |
| **Field-Oriented Control (FOC)** | A motor control technique that independently controls the torque-producing (IQ) and flux-producing (ID) current components. Enables precise, efficient control of AC motors.        |
| **Flux**                         | The magnetic field inside the motor. In FOC, the D-axis current (ID) controls flux magnitude. Induction motors require a minimum flux current to operate.                           |
| **GND**                          | Ground — the 0V reference for the 12V logic system. Distinct from AGND (analog signal returns) and the HV battery negative.                                                         |
| **HV**                           | High Voltage — refers to the main traction battery bus (typically 100–800 VDC in EVs). Always treated as potentially lethal.                                                        |
| **ID / IQ**                      | In Field-Oriented Control: **ID** = D-axis current (controls magnetic flux, little torque), **IQ** = Q-axis current (directly produces torque).                                     |
| **IGBT**                         | Insulated Gate Bipolar Transistor — the high-power switching device in the inverter bridge. Six IGBTs (or equivalent MOSFETs) commutate the 3-phase motor current.                  |
| **Inverter**                     | A power electronics device that converts DC to AC by rapidly switching the IGBT bridge. "Inverter" and "motor controller" are often used interchangeably in EV contexts.            |
| **IP67**                         | Ingress Protection rating — dust-tight (6) and waterproof to 1 meter for 30 minutes (7). The AMPSEAL connector IP rating.                                                           |
| **NTC**                          | Negative Temperature Coefficient — a type of thermistor whose resistance decreases as temperature increases. Commonly used for motor winding temperature sensing.                   |
| **/PROG_ENA**                    | Program Enable — an active-low digital input on J1-7. When grounded at power-up, causes the DSP to enter serial bootloader mode for firmware flashing instead of normal operation.  |
| **Pre-charge**                   | The process of slowly charging the inverter's DC link capacitor through a series resistor before closing the main contactor, to prevent high inrush current.                        |
| **PT100 / PT1000**               | Platinum Resistance Temperature Detectors (RTDs). PT100 = 100 Ω at 0°C, PT1000 = 1000 Ω at 0°C. Resistance increases linearly with temperature (~0.385%/°C).                        |
| **PWM**                          | Pulse Width Modulation — the technique used to synthesize AC waveforms from a DC source by switching transistors on/off at high frequency (typically 8–20 kHz).                     |
| **Regenerative Braking**         | Using the motor as a generator during deceleration to recover kinetic energy back into the battery. The inverter controls this process; current flows back through the DC bus.      |
| **Resolver**                     | A rotary electromagnetic sensor (similar to a transformer) used to measure shaft angle. More robust than encoders for high-vibration environments. Used with PM synchronous motors. |
| **RS-232**                       | A serial communication standard (EIA-232). Point-to-point, ±12V logic, used here for bench service and firmware flashing. Not suitable for multi-drop networks.                     |
| **RTD**                          | Resistance Temperature Detector — a temperature sensor based on precise resistance change with temperature. More accurate than thermistors. See PT100 and PT1000.                   |
| **STB**                          | Switch-To-Battery — a digital input type where the switch connects to battery positive (12V). Input is active when 12V is applied. See DIN5, DIN6.                                  |
| **STG**                          | Switch-To-Ground — a digital input type where the switch connects to ground (0V). Input is active when grounded. Most common type for EV control inputs.                            |
| **Torque**                       | Rotational force, measured in Newton-meters (Nm). In the PM100 CAN protocol, torque values are scaled as Nm × 10 (integer).                                                         |
| **XDCR_PWR**                     | Transducer Power — the PM100's regulated +5V output for powering external sensors. Maximum 80 mA total across all four XDCR_PWR pins on J1.                                         |
| **VSM**                          | Vehicle State Machine — firmware logic in the PM100 that manages power-on sequencing, pre-charge, enable logic, and fault handling.                                                 |


---

*Document compiled from: PM100 Series AC Motor Drive User's Manual (rev h, 2010), PM100 Software User
Manual V3.3, PM Family Data Sheet, PM/RM Hardware Users Manual V3.0, Rinehart PM100 CAN Library
documentation, and TE Connectivity AMPSEAL product datasheets.*

*Authoritative source for production designs: [cascadiamotion.com/documentation](https://www.cascadiamotion.com/documentation)*
