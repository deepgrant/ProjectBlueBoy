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

> **Source \[Ref 1\]:** All byte-level specifications in §7.3–§7.12 are derived from the official
> Cascadia Motion **CAN Protocol, Revision 5.9** (25 Feb 2022) \[Ref 1\], covering PM, RM, PM Gen 5,
> and CM controller families. Some message formats are firmware-version-dependent — see
> [§14 References](#14-references) for a version compatibility table before relying on specific
> byte offsets or parameter addresses.

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
│  • PM100 has a software-configurable internal 120 Ω terminator (CAN A only)  │
│    — enable it if PM100 is at one end of the bus run                         │
│  • RM and CM controllers do NOT have a built-in terminator                   │
│  • No star wiring — short stub connections only                              │
│  • Twisted pair required — do not use untwisted wire                         │
│  • Default bus speed: 250 kbps (software selectable: 125/250/500/1000 kbps)  │
│  • CAN A (J1-11, J1-33) is the active interface; CAN B is reserved           │
│  • All messages: DLC = 8 bytes, little-endian (least significant byte first) │
│  • Standard 11-bit IDs (CAN 2.0A); extended 29-bit and J1939 also supported  │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 7.2 CAN Frame Structure

A CAN 2.0A (standard frame) message has the following layout. Understanding this helps when using a CAN analyser or writing code to send/receive PM100 messages.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                      CAN 2.0A STANDARD FRAME LAYOUT                          │
│                                                                              │
│  ┌─────┬─────────────┬─────┬─────┬─────┬─────────────────────┬──────┬─────┐  │
│  │ SOF │ Identifier  │ RTR │ IDE │ DLC │     Data Bytes      │ CRC  │ EOF │  │
│  │ 1b  │   11 bits   │ 1b  │ 1b  │ 4b  │      0–8 bytes      │ 16b  │ 7b  │  │
│  └─────┴─────────────┴─────┴─────┴─────┴─────────────────────┴──────┴─────┘  │
│                                                                              │
│  SOF         Start of Frame — dominant (0) bit marking start of message      │
│  Identifier  11-bit message ID — defines message type and priority           │
│              Lower ID = higher priority on the bus                           │
│  RTR         Remote Transmission Request — 0 for normal data frames          │
│  IDE         Identifier Extension — 0 for standard (11-bit) CAN 2.0A frames  │
│  DLC         Data Length Code — number of data bytes (0–8)                   │
│  Data        Payload — 0 to 8 bytes of application data                      │
│  CRC         Cyclic Redundancy Check — 15-bit error detection, hardware only │
│  EOF         End of Frame — 7 recessive bits, handled by hardware            │
└──────────────────────────────────────────────────────────────────────────────┘
```

> **Dominant / recessive:** CAN uses two bus states rather than high/low logic. See [Appendix A.1](#a1-dominant-and-recessive-bits) for an explanation.

**For all PM100 messages:**

| Field | Value | Notes |
|---|---|---|
| Frame type | Standard (CAN 2.0A) | 11-bit identifiers |
| DLC | 8 | All PM100 messages are always 8 bytes |
| RTR | 0 | PM100 never uses remote frames |
| Bus speed | 250 kbps (default) | Configurable: 125, 250, 500, or 1000 kbps |

The **11-bit identifier** is what you set as a filter in a CAN analyser or subscribe to in code. For example, to read DC bus voltage you listen for ID `0x0A7`. To command the motor you transmit ID `0x0C0`.

The **8 data bytes** are the application payload — the byte-level breakdown of each message ID is in §7.4 and §7.5.

### 7.3 Data Format Conventions


All multi-byte values in CAN messages are **little-endian**: byte 0 is the least significant byte
(LSB) and byte 1 is the most significant byte (MSB). The format types used across all messages:

| Format Name      | Encoding                              | Scale / Units                     | Range                     |
|------------------|---------------------------------------|-----------------------------------|---------------------------|
| Temperature      | Signed 16-bit int (little-endian)     | Actual °C × 10                    | −3276.8 to +3276.7 °C     |
| Low Voltage      | Signed 16-bit int                     | Actual volts × 100                | −327.68 to +327.67 V      |
| High Voltage     | Signed 16-bit int                     | Actual volts × 10                 | −3276.8 to +3276.7 V      |
| Current          | Signed 16-bit int                     | Actual amps × 10                  | −3276.8 to +3276.7 A      |
| Torque           | Signed 16-bit int                     | Actual N·m × 10                   | −3276.8 to +3276.7 N·m    |
| Angle            | Signed 16-bit int                     | Actual degrees × 10               | 0.0 to ±359.9°            |
| Angular Velocity | Signed 16-bit int                     | Actual RPM (1:1)                  | −32768 to +32767 RPM      |
| Frequency        | Signed 16-bit int                     | Actual Hz × 10                    | −3276.8 to +3276.7 Hz     |
| Flux             | Signed 16-bit int                     | Actual Webers × 1000              | −32.768 to +32.767 Wb     |
| Power            | Signed 16-bit int                     | Actual kW × 10                    | −3276.8 to +3276.7 kW     |
| Boolean          | Unsigned 8-bit                        | 0 = false/off, 1 = true/on        | 0 or 1                    |

**Example — decoding a 16-bit little-endian value:**
If bytes 0,1 of a temperature message read `0x0E 0xEE`, the raw value is `0xEE0E = 60942`.
As a signed 16-bit number this is 60942 → but since temperature range is −3276.8 to +3276.7,
treat as unsigned or check sign bit. For a typical positive temperature: raw = `0x01F4 = 500` →
500 ÷ 10 = **50.0 °C**.

### 7.4 Broadcast Message Map

The PM100 broadcasts all messages continuously regardless of VSM or CAN mode.
The base address 0x0A0 is the default (adjustable via EEPROM parameter CAN ID Offset).
Individual messages can be enabled/disabled using EEPROM parameter address 148
(CAN Active Messages Lo Word).

```
┌────────────────────────────────────────────────────────────────────────────────┐
│              CAN BROADCAST MESSAGES (PM100 → Vehicle Controller)               │
│  Direction: PM100 transmits; vehicle controller reads (receive only)           │
│                                                                                │
│  ID      │ Rate      │ Content                          │ Enable bit (Addr 148)│
│──────────┼───────────┼──────────────────────────────────┼───────────────────── │
│  0x0A0   │ 10 Hz     │ Temperatures #1 (IGBT modules)   │ bit 0  (0x0001)      │
│  0x0A1   │ 10 Hz     │ Temperatures #2 (board + RTDs)   │ bit 1  (0x0002)      │
│  0x0A2   │ 10 Hz     │ Temperatures #3 (motor + shudder)│ bit 2  (0x0004)      │
│  0x0A3   │ 100 Hz    │ Analog Input Voltages            │ bit 3  (0x0008)      │
│  0x0A4   │ 100 Hz    │ Digital Input Status             │ bit 4  (0x0010)      │
│  0x0A5   │ 100 Hz    │ Motor Position Information       │ bit 5  (0x0020)      │
│  0x0A6   │ 100 Hz    │ Current Information              │ bit 6  (0x0040)      │
│  0x0A7   │ 100 Hz    │ Voltage Information              │ bit 7  (0x0080)      │
│  0x0A8   │ 100 Hz    │ Flux Information                 │ bit 8  (0x0100)      │
│  0x0A9   │ 10 Hz     │ Internal Voltages (power rails)  │ bit 9  (0x0200)      │
│  0x0AA   │ 100 Hz    │ Internal States (VSM, faults)    │ bit 10 (0x0400)      │
│  0x0AB   │ 100 Hz    │ Fault Codes (POST + Run faults)  │ bit 11 (0x0800)      │
│  0x0AC   │ 100 Hz    │ Torque & Timer Information       │ bit 12 (0x1000)      │
│  0x0AD   │ 100 Hz    │ Modulation Index & Flux Weak.    │ bit 13 (0x2000)      │
│  0x0AE   │ 10 Hz     │ Firmware Information             │ bit 14 (0x4000)      │
│  0x0AF   │ 100 Hz    │ Diagnostic Data (see diag manual)│ bit 15 (0x8000)      │
│  0x0B0   │ 333 Hz    │ High Speed Message (fw 2042+)    │ Hi Word bit 0=0      │
│          │ (3 ms)    │ Torque cmd/fdbk, speed, DC bus   │ (set Hi Word 0xFFFE) │
└────────────────────────────────────────────────────────────────────────────────┘
```

**Broadcast rate notes:** "Slow" group (10 Hz) = 0x0A0, 0x0A1, 0x0A2, 0x0A9, 0x0AE.
"Fast" group (100 Hz) = all others. Starting firmware version 2025, rates are configurable
via EEPROM parameters CAN Fast Msg Rate (addr 235) and CAN Slow Msg Rate (addr 236).
Setting either rate to 0 disables the entire group.

### 7.5 Broadcast Message Byte Definitions

All messages are 8 bytes, little-endian. Byte pairs are listed as [Lo byte, Hi byte].

---

#### 0x0A0 — Temperatures #1 (IGBT Module Temperatures) — 10 Hz

| Bytes | Signal               | Format      | Scale    | Description                        |
|-------|----------------------|-------------|----------|------------------------------------|
| 0, 1  | Module A Temperature | Temperature | °C × 10  | IGBT module Phase A temperature    |
| 2, 3  | Module B Temperature | Temperature | °C × 10  | IGBT module Phase B temperature    |
| 4, 5  | Module C Temperature | Temperature | °C × 10  | IGBT module Phase C temperature    |
| 6, 7  | Gate Driver Board Temp | Temperature | °C × 10 | Gate driver board temperature     |

Example: bytes `[0xD0, 0x07, 0xD0, 0x07, 0xD0, 0x07, 0xD0, 0x07]` →
all four temperatures = 0x07D0 = 2000 → 200.0 °C.

---

#### 0x0A1 — Temperatures #2 (Control Board + RTD Inputs) — 10 Hz

| Bytes | Signal                | Format      | Scale   | Description                        |
|-------|-----------------------|-------------|---------|-------------------------------------|
| 0, 1  | Control Board Temp    | Temperature | °C × 10 | Control PCB temperature             |
| 2, 3  | RTD #1 Temperature    | Temperature | °C × 10 | Temperature from RTD input #1       |
| 4, 5  | RTD #2 Temperature    | Temperature | °C × 10 | Temperature from RTD input #2       |
| 6, 7  | RTD #3 Temperature    | Temperature | °C × 10 | Temperature from RTD input #3 (Gen 2 only) |

---

#### 0x0A2 — Temperatures #3 (Motor + Shudder) — 10 Hz

| Bytes | Signal                       | Format      | Scale    | Description                                     |
|-------|------------------------------|-------------|----------|-------------------------------------------------|
| 0, 1  | Coolant Temp / RTD #4 Temp   | Temperature | °C × 10  | Gen 2: RTD #4; Gen 5/CM: estimated coolant temp |
| 2, 3  | Hot Spot Temp / RTD #5 Temp  | Temperature | °C × 10  | Gen 2: RTD #5; Gen 5/CM: estimated internal hot spot |
| 4, 5  | Motor Temperature            | Temperature | °C × 10  | Filtered motor temp sensor value                |
| 6, 7  | Torque Shudder               | Torque      | N·m × 10 | Torque value used in shudder compensation       |

---

#### 0x0A3 — Analog Input Voltages — 100 Hz

**For firmware before version 1995 (4 analog inputs, 16-bit each):**

| Bytes | Signal          | Format      | Scale     | Description               |
|-------|-----------------|-------------|-----------|---------------------------|
| 0, 1  | Analog Input #1 | Low Voltage | volts × 100 | Voltage on AIN1 (0–5 V) |
| 2, 3  | Analog Input #2 | Low Voltage | volts × 100 | Voltage on AIN2           |
| 4, 5  | Analog Input #3 | Low Voltage | volts × 100 | Voltage on AIN3           |
| 6, 7  | Analog Input #4 | Low Voltage | volts × 100 | Voltage on AIN4           |

**For firmware version 1995 and later (6 analog inputs, 10-bit each packed into bits):**

| Bits    | Signal          | Format      | Scale     | Description               |
|---------|-----------------|-------------|-----------|---------------------------|
| 0–9     | Analog Input #1 | Low Voltage | volts × 100 | Voltage on AIN1         |
| 10–19   | Analog Input #2 | Low Voltage | volts × 100 | Voltage on AIN2         |
| 20–29   | Analog Input #3 | Low Voltage | volts × 100 | Voltage on AIN3         |
| 32–41   | Analog Input #4 | Low Voltage | volts × 100 | Voltage on AIN4         |
| 42–51   | Analog Input #5 | Low Voltage | volts × 100 | Voltage on AIN5         |
| 52–61   | Analog Input #6 | Low Voltage | volts × 100 | Voltage on AIN6         |

---

#### 0x0A4 — Digital Input Status — 100 Hz

| Byte | Signal          | Format  | Description                                 |
|------|-----------------|---------|---------------------------------------------|
| 0    | Digital Input #1 | Boolean | Forward switch status                       |
| 1    | Digital Input #2 | Boolean | Reverse switch status                       |
| 2    | Digital Input #3 | Boolean | Brake switch status                         |
| 3    | Digital Input #4 | Boolean | REGEN disable switch status                 |
| 4    | Digital Input #5 | Boolean | Ignition switch status                      |
| 5    | Digital Input #6 | Boolean | Start switch status                         |
| 6    | Digital Input #7 | Boolean | Valet mode status                           |
| 7    | Digital Input #8 | Boolean | Digital Input #8 status                    |

---

#### 0x0A5 — Motor Position Information — 100 Hz

| Bytes | Signal                   | Format           | Scale        | Description                                  |
|-------|--------------------------|------------------|--------------|----------------------------------------------|
| 0, 1  | Motor Angle (Electrical) | Angle            | degrees × 10 | Electrical angle from encoder/resolver       |
| 2, 3  | Motor Speed              | Angular Velocity | RPM (1:1)    | Measured motor speed                         |
| 4, 5  | Electrical Output Freq   | Frequency        | Hz × 10      | Actual electrical output frequency           |
| 6, 7  | Delta Resolver Filtered  | Angle            | degrees × 10 | Calibration value; range ±180°; 270° = −90°  |

---

#### 0x0A6 — Current Information — 100 Hz

| Bytes | Signal          | Format  | Scale    | Description                              |
|-------|-----------------|---------|----------|------------------------------------------|
| 0, 1  | Phase A Current | Current | amps × 10 | Measured phase A current                |
| 2, 3  | Phase B Current | Current | amps × 10 | Measured phase B current                |
| 4, 5  | Phase C Current | Current | amps × 10 | Measured phase C current                |
| 6, 7  | DC Bus Current  | Current | amps × 10 | Calculated DC bus current               |

Example: DC bus current bytes `[0x64, 0x00]` → 0x0064 = 100 → 10.0 A.

---

#### 0x0A7 — Voltage Information — 100 Hz

| Bytes | Signal           | Format       | Scale      | Description                                                       |
|-------|------------------|--------------|------------|-------------------------------------------------------------------|
| 0, 1  | DC Bus Voltage   | High Voltage | volts × 10 | Measured DC bus voltage                                           |
| 2, 3  | Output Voltage   | High Voltage | volts × 10 | Calculated output (peak line-neutral)                             |
| 4, 5  | VAB / Vd Voltage | High Voltage | volts × 10 | Phase A–B voltage when disabled; Vd voltage when enabled          |
| 6, 7  | VBC / Vq Voltage | High Voltage | volts × 10 | Phase B–C voltage when disabled; Vq voltage when enabled          |

Example: DC bus voltage bytes `[0xC4, 0x09]` → 0x09C4 = 2500 → 250.0 V.

---

#### 0x0A8 — Flux Information — 100 Hz

| Bytes | Signal        | Format  | Scale          | Description              |
|-------|---------------|---------|----------------|--------------------------|
| 0, 1  | Flux Command  | Flux    | Webers × 1000  | Commanded flux           |
| 2, 3  | Flux Feedback | Flux    | Webers × 1000  | Estimated flux           |
| 4, 5  | Id Feedback   | Current | amps × 10      | D-axis current feedback  |
| 6, 7  | Iq Feedback   | Current | amps × 10      | Q-axis current feedback  |

---

#### 0x0A9 — Internal Voltages (Power Rail Monitor) — 10 Hz

| Bytes | Signal              | Format      | Scale       | Description                  |
|-------|---------------------|-------------|-------------|------------------------------|
| 0, 1  | 1.5 V Reference     | Low Voltage | volts × 100 | Internal 1.5 V rail          |
| 2, 3  | 2.5 V Reference     | Low Voltage | volts × 100 | Internal 2.5 V rail          |
| 4, 5  | 5.0 V Reference     | Low Voltage | volts × 100 | Internal 5.0 V rail          |
| 6, 7  | 12 V System Voltage | Low Voltage | volts × 100 | 12 V system supply           |

---

#### 0x0AA — Internal States — 100 Hz

This message carries packed status information across all 8 bytes. Several fields are bit-level.

| Byte | Bit(s) | Signal                   | Values / Meaning                                                              |
|------|--------|--------------------------|-------------------------------------------------------------------------------|
| 0    | all    | VSM State                | 0=Start, 1=Pre-charge Init, 2=Pre-charge Active, 3=Pre-charge Complete,       |
|      |        |                          | 4=Wait, 5=Ready, 6=Motor Running, 7=Blink Fault Code,                        |
|      |        |                          | 14=Shutdown in Process, 15=Recycle Power                                      |
| 1    | all    | PWM Frequency            | kHz (Gen 5/CM: currently active PWM frequency)                                |
| 2    | all    | Inverter State           | 0=Power On, 1=Stop, 2=Open Loop, 3=Closed Loop, 4=Wait,                      |
|      |        |                          | 8=Idle Run, 9=Idle Stop (5–7, 10–12 = internal states)                       |
| 3    | 0      | Relay 1 Status           | 1 = Relay 1 active                                                            |
|      | 1      | Relay 2 Status           | 1 = Relay 2 active                                                            |
|      | 2      | Relay 3 Status           | 1 = Relay 3 active                                                            |
|      | 3      | Relay 4 Status           | 1 = Relay 4 active                                                            |
|      | 4      | Relay 5 Status           | 1 = Relay 5 active                                                            |
|      | 5      | Relay 6 Status           | 1 = Relay 6 active                                                            |
| 4    | 0      | Inverter Run Mode        | 0 = Torque Mode, 1 = Speed Mode                                               |
|      | 5–7    | Active Discharge State   | 0=Disabled, 1=Enabled/waiting, 2=Speed Check, 3=Active, 4=Complete           |
| 5    | 0      | Inverter Command Mode    | 0 = CAN Mode, 1 = VSM Mode                                                    |
|      | 4–7    | Rolling Counter Value    | Gen 5/CM only: expected rolling counter value (0–15)                          |
| 6    | 0      | Inverter Enable State    | 0 = Inverter disabled, 1 = Inverter enabled                                   |
|      | 6      | Start Mode Active        | 1 = start signal has been activated                                           |
|      | 7      | Inverter Enable Lockout  | 0 = can enable; 1 = cannot enable (must send disable first)                   |
| 7    | 0      | Direction Command        | 1 = Forward, 0 = Reverse (if enabled) or Stopped (if disabled)               |
|      | 1      | BMS Active               | 0 = BMS message not received, 1 = BMS message active                         |
|      | 2      | BMS Limiting Torque      | 1 = torque is being limited by BMS                                            |
|      | 3      | Limit Max Speed          | 1 = torque limiting due to over-speed (Gen 5/CM + Gen 3 v2042+)              |
|      | 4      | Limit Hot Spot           | 1 = current limited to regulate hot spot temperature (Gen 5/CM only)         |
|      | 5      | Low Speed Limiting       | 1 = low-speed current limiting applied (Gen 5/CM + Gen 3 v2042+)             |
|      | 6      | Coolant Temp Limiting    | 1 = current limited due to coolant temperature (Gen 5/CM only)               |

---

#### 0x0AB — Fault Codes — 100 Hz

| Bytes | Signal         | Format   | Description                             |
|-------|----------------|----------|-----------------------------------------|
| 0, 1  | POST Fault Lo  | 32-bit bitmask (lo word) | Power-On Self Test fault bits 0–15 |
| 2, 3  | POST Fault Hi  | 32-bit bitmask (hi word) | Power-On Self Test fault bits 16–31 |
| 4, 5  | Run Fault Lo   | 32-bit bitmask (lo word) | Run-time fault bits 0–15            |
| 6, 7  | Run Fault Hi   | 32-bit bitmask (hi word) | Run-time fault bits 16–31           |

**POST Fault Bits (bits 0–31, bytes 0–3):**

| Bit | Byte | Byte Value | POST Fault Description                           |
|-----|------|------------|--------------------------------------------------|
| 0   | 0    | 0x01       | Hardware Gate / Desaturation Fault               |
| 1   | 0    | 0x02       | HW Over-current Fault                            |
| 2   | 0    | 0x04       | Accelerator Shorted                              |
| 3   | 0    | 0x08       | Accelerator Open                                 |
| 4   | 0    | 0x10       | Current Sensor Low                               |
| 5   | 0    | 0x20       | Current Sensor High                              |
| 6   | 0    | 0x40       | Module Temperature Low                           |
| 7   | 0    | 0x80       | Module Temperature High                          |
| 8   | 1    | 0x01       | Control PCB Temperature Low                      |
| 9   | 1    | 0x02       | Control PCB Temperature High                     |
| 10  | 1    | 0x04       | Gate Drive PCB Temperature Low                   |
| 11  | 1    | 0x08       | Gate Drive PCB Temperature High                  |
| 12  | 1    | 0x10       | 5 V Sense Voltage Low                            |
| 13  | 1    | 0x20       | 5 V Sense Voltage High                           |
| 14  | 1    | 0x40       | 12 V Sense Voltage Low                           |
| 15  | 1    | 0x80       | 12 V Sense Voltage High                          |
| 16  | 2    | 0x01       | 2.5 V Sense Voltage Low                          |
| 17  | 2    | 0x02       | 2.5 V Sense Voltage High                         |
| 18  | 2    | 0x04       | 1.5 V Sense Voltage Low                          |
| 19  | 2    | 0x08       | 1.5 V Sense Voltage High                         |
| 20  | 2    | 0x10       | DC Bus Voltage High                              |
| 21  | 2    | 0x20       | DC Bus Voltage Low                               |
| 22  | 2    | 0x40       | Pre-charge Timeout                               |
| 23  | 2    | 0x80       | Pre-charge Voltage Failure                       |
| 24  | 3    | 0x01       | EEPROM Checksum Invalid                          |
| 25  | 3    | 0x02       | EEPROM Data Out of Range                         |
| 26  | 3    | 0x04       | EEPROM Update Required                           |
| 27  | 3    | 0x08       | Hardware DC Bus Over-Voltage (init) / Gen 5: Gate Driver Init |
| 28  | 3    | 0x10       | Gen 3: Reserved; Gen 5: Gate Driver Initialization |
| 29  | 3    | 0x20       | Reserved                                         |
| 30  | 3    | 0x40       | Brake Shorted                                    |
| 31  | 3    | 0x80       | Brake Open                                       |

**Run Fault Bits (bits 0–31, bytes 4–7):**

| Bit | Byte | Byte Value | Run Fault Description                                         |
|-----|------|------------|---------------------------------------------------------------|
| 0   | 4    | 0x01       | Motor Over-speed Fault                                        |
| 1   | 4    | 0x02       | Over-current Fault                                            |
| 2   | 4    | 0x04       | Over-voltage Fault                                            |
| 3   | 4    | 0x08       | Inverter Over-temperature Fault                              |
| 4   | 4    | 0x10       | Accelerator Input Shorted Fault                               |
| 5   | 4    | 0x20       | Accelerator Input Open Fault                                  |
| 6   | 4    | 0x40       | Direction Command Fault                                       |
| 7   | 4    | 0x80       | Inverter Response Time-out Fault                              |
| 8   | 5    | 0x01       | Hardware Gate / Desaturation Fault                            |
| 9   | 5    | 0x02       | Hardware Over-current Fault                                   |
| 10  | 5    | 0x04       | Under-voltage Fault                                           |
| 11  | 5    | 0x08       | **CAN Command Message Lost Fault** (timeout — see §7.6)       |
| 12  | 5    | 0x10       | Motor Over-temperature Fault                                  |
| 13  | 5    | 0x20       | Reserved                                                      |
| 14  | 5    | 0x40       | Reserved                                                      |
| 15  | 5    | 0x80       | Reserved                                                      |
| 16  | 6    | 0x01       | Brake Input Shorted Fault                                     |
| 17  | 6    | 0x02       | Brake Input Open Fault                                        |
| 18  | 6    | 0x04       | Module A Over-temperature Fault                               |
| 19  | 6    | 0x08       | Module B Over-temperature Fault                               |
| 20  | 6    | 0x10       | Module C Over-temperature Fault                               |
| 21  | 6    | 0x20       | PCB Over-temperature Fault                                    |
| 22  | 6    | 0x40       | Gate Drive Board 1 Over-temperature Fault                     |
| 23  | 6    | 0x80       | Gate Drive Board 2 Over-temperature Fault                     |
| 24  | 7    | 0x01       | Gate Drive Board 3 Over-temperature Fault                     |
| 25  | 7    | 0x02       | Current Sensor Fault                                          |
| 26  | 7    | 0x04       | Gen 3: Reserved; Gen 5: Gate Driver Over-Voltage              |
| 27  | 7    | 0x08       | Gen 3: Hardware DC Bus Over-Voltage; Gen 5: Reserved          |
| 28  | 7    | 0x10       | Gen 3: Reserved; Gen 5: Hardware DC Bus Over-voltage Fault    |
| 29  | 7    | 0x20       | Reserved                                                      |
| 30  | 7    | 0x40       | Resolver Not Connected                                        |
| 31  | 7    | 0x80       | Reserved                                                      |

---

#### 0x0AC — Torque & Timer Information — 100 Hz

| Bytes  | Signal             | Format  | Scale      | Description                                        |
|--------|--------------------|---------|------------|----------------------------------------------------|
| 0, 1   | Commanded Torque   | Torque  | N·m × 10   | Torque command currently in use                    |
| 2, 3   | Torque Feedback    | Torque  | N·m × 10   | Estimated motor torque (from motor model)          |
| 4, 5, 6, 7 | Power On Timer | Unsigned 32-bit | counts × 0.003 sec | Timer updated every 3 ms; rolls over ~5 months |

---

#### 0x0AD — Modulation Index & Flux Weakening Output — 100 Hz

| Bytes | Signal               | Format      | Scale      | Description                                  |
|-------|----------------------|-------------|------------|----------------------------------------------|
| 0, 1  | Modulation Index     | Per-unit    | ÷ 100      | Actual modulation index = raw value / 100    |
| 2, 3  | Flux Weakening Output| Current     | amps × 10  | Output of flux weakening regulator           |
| 4, 5  | Id Command           | Current     | amps × 10  | Commanded D-axis current                     |
| 6, 7  | Iq Command           | Current     | amps × 10  | Commanded Q-axis current                     |

---

#### 0x0AE — Firmware Information — 10 Hz

| Bytes | Signal                  | Format | Description                                      |
|-------|-------------------------|--------|--------------------------------------------------|
| 0, 1  | EEPROM Version / Project Code | NA | Project-specific EEPROM version (factory use)  |
| 2, 3  | Software Version        | NA     | Major.minor firmware version                     |
| 4, 5  | Date Code (MMDD)        | NA     | Month and day of firmware build                  |
| 6, 7  | Date Code (YYYY)        | NA     | Year of firmware build                           |

---

#### 0x0AF — Diagnostic Data — 100 Hz

Content is defined in a separate Cascadia Motion document: "Download Diagnostic Data."
Enable via EEPROM parameter CAN Diagnostic Data Transmit Active (address 158).

---

#### 0x0B0 — High Speed Message — 333 Hz (3 ms) — firmware 2042+

This message is disabled by default. To enable it, set the Low Bit of
CAN Active Messages Hi Word (EEPROM addr 237) to 0 — e.g., set Hi Word to 0xFFFE.

| Bytes | Signal             | Format           | Scale      | Description                      |
|-------|--------------------|------------------|------------|----------------------------------|
| 0, 1  | Torque Command     | Torque           | N·m × 10   | Currently commanded torque       |
| 2, 3  | Torque Feedback    | Torque           | N·m × 10   | Estimated motor torque           |
| 4, 5  | Motor Speed        | Angular Velocity | RPM (1:1)  | Measured motor speed             |
| 6, 7  | DC Bus Voltage     | High Voltage     | volts × 10 | Measured DC bus voltage          |

---

### 7.6 Command Message — 0x0C0 (Vehicle → PM100)

The command message is the only message sent **to** the PM100 to control the motor.
It must be in **CAN mode** (EEPROM Inverter Command Mode = 0).

```
┌───────────────────────────────────────────────────────────────────────────────┐
│            0x0C0 — COMMAND MESSAGE  (Vehicle → PM100)  8 bytes DLC            │
│                  Send at ≥ 2 Hz; 10–50 ms typical; processed every 3 ms       │
│                                                                               │
│  Bytes 0–1  │ Torque Command    │ Signed 16-bit, little-endian                │
│             │                   │ Units: N·m × 10 (e.g. 30 N·m = 300)         │
│             │                   │ Positive = motoring, Negative = regen       │
│             │                   │ In Speed Mode: acts as feedforward torque   │
│             │                   │ In Torque Mode (fw 2048+): if Speed Command │
│             │                   │   is non-zero, overrides Max Speed EEPROM   │
│                                                                               │
│  Bytes 2–3  │ Speed Command     │ Signed 16-bit, little-endian                │
│             │                   │ Units: RPM (1:1)                            │
│             │                   │ Primary setpoint in Speed Mode              │
│             │                   │ In Torque Mode (fw 2048+): overrides        │
│             │                   │   Max Speed EEPROM limit when non-zero      │
│             │                   │ Positive = direction command direction;     │
│             │                   │   Negative = opposite of direction command  │
│                                                                               │
│  Byte 4     │ Direction Command │ 0 = Reverse, 1 = Forward                    │
│             │                   │ Changing direction while enabled auto-      │
│             │                   │ disables inverter (safety lockout)          │
│                                                                               │
│  Byte 5     │ Control Bits      │ Bit 0: Inverter Enable  (0=Off, 1=On)       │
│             │                   │ Bit 1: Inverter Discharge (0=Disable,       │
│             │                   │          1=Enable active discharge)         │
│             │                   │ Bit 2: Speed Mode Enable (0=no override,    │
│             │                   │          1=force Torque→Speed mode change)  │
│             │                   │         (does NOT change Speed→Torque)      │
│             │                   │ Bits 4–7: Rolling Counter (Gen 5/CM only,   │
│             │                   │           U4 value 0–15, must increment)    │
│                                                                               │
│  Bytes 6–7  │ Torque Limit      │ Signed 16-bit, N·m × 10                     │
│             │                   │ 0 = use EEPROM motor/regen limits (default) │
│             │                   │ Positive value = override both Motor and    │
│             │                   │   Regen Torque limits to this value         │
│             │                   │ Added in firmware version 1953;             │
│             │                   │   set to 0x0000 on older firmware           │
└───────────────────────────────────────────────────────────────────────────────┘
```

**Inverter Enable Lockout Safety Feature:**
Before the inverter will accept an Enable command, it must first see a Disable command.
This prevents accidental motor start-up at power-on. Sequence:
1. Send 0x0C0 with Byte 5 Bit 0 = 0 (Disable) — clears lockout
2. Send 0x0C0 with Byte 5 Bit 0 = 1 (Enable) — now accepted

**Sign Convention (Torque Mode, Forward direction):**

| Torque Command | Motor Speed | Result           |
|----------------|-------------|------------------|
| Positive       | Positive    | Motoring (drive) |
| Negative       | Positive    | Regen (brake)    |
| Positive       | Negative    | Regen (brake)    |
| Negative       | Negative    | Do not use       |

**Working example — 30 N·m forward, inverter enabled (from official document):**

```
Byte 0 = 0x2C  (44 decimal)  ─┐ Torque = (1×256)+44 = 300 → 30.0 N·m
Byte 1 = 0x01  (1 decimal)   ─┘
Byte 2 = 0xF4  (244 decimal) ─┐ Speed = (1×256)+244 = 500 RPM (max speed override)
Byte 3 = 0x01  (1 decimal)   ─┘
Byte 4 = 0x01  Direction = Forward (1)
Byte 5 = 0x01  Inverter Enable = 1
Byte 6 = 0x00  ─┐ Torque Limit = 0 (use EEPROM defaults)
Byte 7 = 0x00  ─┘
```

**Startup command sequence in CAN torque mode:**

| Step | Byte 0–1      | Byte 2–3 | Byte 4 | Byte 5 | Byte 6–7 | Notes                              |
|------|---------------|----------|--------|--------|----------|------------------------------------|
| 1    | 0x00 0x00     | any      | 0      | 0      | 0x00 0x00| Send Disable first — clears lockout |
| 2    | 0x64 0x00     | any      | 1      | 1      | 0x00 0x00| Enable + 10 N·m forward            |
| 3    | 0xC8 0x00     | any      | 1      | 1      | 0x00 0x00| Increase to 20 N·m                 |
| 4    | 0x9C 0xFF     | any      | 1      | 1      | 0x00 0x00| −10 N·m = regen (0xFF9C = −100)    |
| 5    | any           | any      | any    | 0      | 0x00 0x00| Disable before direction change    |
| 6    | 0x64 0x00     | any      | 0      | 1      | 0x00 0x00| Enable + 10 N·m reverse            |

---

### 7.7 Parameter Messages — 0x0C1 / 0x0C2

Parameter messages allow reading and writing configuration values in the PM100.
These are **not** the same as 0x0C8/0x0C9 (which do not exist in this protocol —
the correct IDs are 0x0C1 for the request and 0x0C2 for the response).

**Parameters can only be written when the motor is not enabled.**

```
┌──────────────────────────────────────────────────────────────────────────────┐
│         0x0C1 — READ / WRITE PARAMETER COMMAND  (Vehicle → PM100)            │
│                                                                              │
│  Bytes 0–1 │ Parameter Address │ Unsigned 16-bit, little-endian              │
│            │                   │ 0–99   = General / command parameters       │
│            │                   │ 100–499 = EEPROM (non-volatile) parameters  │
│                                                                              │
│  Byte 2    │ R/W Command       │ 0 = Read, 1 = Write                         │
│                                                                              │
│  Byte 3    │ Reserved          │ Send as 0x00                                │
│                                                                              │
│  Bytes 4–5 │ Data (Lo)         │ Data value, format per parameter definition │
│            │                   │ If < 4 bytes: fill from byte 4 upward       │
│                                                                              │
│  Bytes 6–7 │ Reserved          │ Send as 0x00 0x00                           │
└──────────────────────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────────────────────┐
│         0x0C2 — READ / WRITE PARAMETER RESPONSE  (PM100 → Vehicle)            │
│                                                                               │
│  Bytes 0–1 │ Parameter Address │ Echoes the address from 0x0C1                │
│            │                   │ Returns 0x00 0x00 if address not recognized  │
│                                                                               │
│  Byte 2    │ Write Success     │ 0 = not written / read response, 1 = success │
│                                                                               │
│  Byte 3    │ Reserved          │                                              │
│                                                                               │
│  Bytes 4–5 │ Data (Lo)         │ Read data (on read), or echo (on write)      │
│                                                                               │
│  Bytes 6–7 │ Reserved          │                                              │
└───────────────────────────────────────────────────────────────────────────────┘
```

**Example — read parameter address 148 (CAN Active Messages Lo Word):**
```
Send  0x0C1: [0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
              addr=148(0x94), read(0), reserved, data=0
Recv  0x0C2: [0x94, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00]
              addr=148, success=0(read), data=0xFFFF (all messages enabled)
```

**Example — write parameter address 21 (Fault Clear):**
```
Send  0x0C1: [0x15, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
              addr=21(0x15), write(1), reserved, data=0
Recv  0x0C2: [0x15, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
              addr=21, write success=1, data=0
```

### 7.8 Key Command Parameters (Address 0–99, General — read/write via 0x0C1)

| Addr | Name                    | Format         | Description                                          |
|------|-------------------------|----------------|------------------------------------------------------|
| 1    | Relay Command           | Unsigned int   | 0xAA00 = Normal Run; 0x55nn = External Relay Control (nn = relay bitmask: bit 0=relay1…bit5=relay6) |
| 10   | Flux Command            | Flux (Wb×1000) | Override flux command                                |
| 11   | Resolver PWM Delay Cmd  | Unsigned int (0–6250) | Calibration: resolver A/D timing           |
| 12   | Gamma Adjust            | Degrees        | Resolver-motor magnetic field alignment              |
| 20   | GUI Command             | Boolean        | Set to 0 to clear all active faults                  |
| 21   | Fault Clear             | Boolean        | Write 0 to clear active faults (CAN + VSM mode)     |
| 22   | Set PWM Frequency       | Unsigned int (6–24) | Gen 5/CM: override PWM freq (kHz); reverts on power cycle |
| 23   | Shudder Compensation Gain | Unsigned int | 0=disable; >0=enable with gain = value/100 (fw 2048+) |
| 31   | Diag. Data Trigger      | Unsigned int   | Non-zero triggers a diagnostic CAN data dump (fw 651E+) |

### 7.9 Key EEPROM Parameters (Address 100–499 — write only when motor disabled)

These persist through power cycles. Written via 0x0C1, readable via 0x0C1 read command.

**Motor Configuration (addr 150–157)**

| Addr | Name                  | Format          | Description                                      |
|------|-----------------------|-----------------|--------------------------------------------------|
| 150  | Motor Parameter Set   | Unsigned 8-bit  | Selects motor type preset                        |
| 151  | Resolver PWM Delay    | Unsigned int    | Resolver A/D timing calibration (0–6250)         |
| 152  | Gamma Adjust          | Degrees         | Resolver-to-motor field alignment angle          |
| 154  | Sin Offset            | Low Voltage     | Resolver sine channel offset (Encoder calibration) |
| 155  | Cos Offset            | Low Voltage     | Resolver cosine channel offset                   |

**System Configuration (addr 140–174, 241–246)**

| Addr | Name                  | Format   | Values                                           |
|------|-----------------------|----------|--------------------------------------------------|
| 140  | Pre-charge Bypassed   | Boolean  | 0=Pre-charge active, 1=Pre-charge bypassed       |
| 142  | Inverter Run Mode     | Boolean  | 0=Torque Mode, 1=Speed Mode                      |
| 143  | Inverter Command Mode | Boolean  | **0=CAN Mode**, 1=VSM Mode (default)             |
| 149  | Key Switch Mode       | Unsigned | 0=simple on/off, 1=traditional ignition          |
| 170  | Relay Output State    | Unsigned | Normal function vs CAN-controlled relays         |
| 173  | Discharge Enable      | Unsigned | See Inverter Discharge Process manual            |
| 174  | Serial Number         | Unsigned | Unit serial number                               |

**CAN Configuration (addr 141–159, 171–172, 233–240)**

| Addr | Name                        | Format   | Default | Description                                              |
|------|-----------------------------|----------|---------|----------------------------------------------------------|
| 141  | CAN ID Offset               | Unsigned | 0x0A0   | Base address for all CAN messages (range: 0–0x7C0)       |
| 144  | CAN Extended Message ID     | Boolean  | 0       | 0=11-bit standard, 1=29-bit extended                     |
| 145  | CAN Term Resistor Present   | Boolean  | 1       | 1=internal 120 Ω terminator active (PM only)             |
| 146  | CAN Command Message Active  | Boolean  | 0       | 1=enable CAN command timeout watchdog                    |
| 147  | CAN Bit Rate                | Unsigned | 250     | Bus speed in kbps: 125, 250, 500, 1000 (requires power cycle) |
| 148  | CAN Active Messages Lo Word | Unsigned | 0xFFFF  | Bitmask to enable/disable each broadcast message         |
| 158  | CAN Diag Data TX Active     | Boolean  | 1       | 1=broadcast 0x0AF diagnostic data                        |
| 159  | CAN Inverter Enable Switch  | Boolean  | 0       | 1=DIN1 must also be high to allow inverter enable        |
| 171  | CAN J1939 Option Active     | Boolean  | 0       | 1=use J1939 format (requires Extended ID = 1)            |
| 172  | CAN Timeout                 | Unsigned | 333     | Timeout in counts × 3 ms (333 = 999 ms)                  |
| 177  | CAN OBD2 Enable             | Boolean  | 0       | 1–7 enables OBD2 support with address offset             |
| 178  | CAN BMS Limit Enable        | Boolean  | 0       | 1=accept Orion BMS message at 0x202 for torque limiting  |
| 233  | CAN Slave Cmd ID            | Unsigned | 0       | Address of slave controller (0=disabled); = slave CAN offset + 0x20 |
| 234  | CAN Slave Dir               | Unsigned | 0       | 0=slave same direction as master, 1=slave opposite       |
| 235  | CAN Fast Msg Rate (ms)      | Unsigned | 10      | Broadcast period for "Fast" group (100 Hz); 0=disable    |
| 236  | CAN Slow Msg Rate (ms)      | Unsigned | 100     | Broadcast period for "Slow" group (10 Hz); 0=disable     |
| 237  | CAN Active Messages Hi Word | Unsigned | 0xFFFF  | Enable/disable mailboxes (keep 0xFFFF; 0xFFFE enables 0x0B0) |

**Current Parameters (addr 100–109)**

| Addr | Name               | Format  | Description                                     |
|------|--------------------|---------|-------------------------------------------------|
| 100  | Iq Limit           | Current (A×10) | Q-axis (torque-producing) current limit  |
| 101  | Id Limit           | Current (A×10) | D-axis (flux-producing) current limit    |
| 107  | Ia Offset EEPROM   | ADC Count | Phase A current sensor offset (default 2048)  |
| 108  | Ib Offset EEPROM   | ADC Count | Phase B current sensor offset (default 2048)  |
| 109  | Ic Offset EEPROM   | ADC Count | Phase C current sensor offset (default 2048)  |

**Voltage & Flux (addr 102–106)**

| Addr | Name                | Format       | Description                                         |
|------|---------------------|--------------|-----------------------------------------------------|
| 102  | DC Voltage Limit    | High Voltage | Over-voltage protection threshold                   |
| 103  | DC Voltage Hysteresis | High Voltage | Hysteresis for leaving over-voltage condition      |
| 104  | DC Under-voltage Limit | High Voltage | Under-voltage fault threshold (0=disabled)        |
| 106  | Vehicle Flux Command | Flux        | Back-EMF flux constant for the motor               |

**Temperature (addr 112–115, 203)**

| Addr | Name                  | Format      | Description                                              |
|------|-----------------------|-------------|----------------------------------------------------------|
| 112  | Inverter Over-Temp    | Temperature (°C×10) | Shutdown threshold e.g. 85°C = 850             |
| 113  | Motor Over-Temp       | Temperature (°C×10) | Motor shutdown threshold e.g. 150°C = 1500     |
| 114  | Zero Torque Temp      | Temperature (°C×10) | Motor temp at which torque is reduced to zero  |
| 115  | Full Torque Temp      | Temperature (°C×10) | Motor temp at which full torque is available   |
| 203  | RTD Selection         | Unsigned    | Bit 0: RTD1 (0=1000Ω, 1=100Ω); Bit 1: RTD2            |

**Torque (addr 129–131, 164–168)**

| Addr | Name               | Format  | Description                                                 |
|------|--------------------|---------|-------------------------------------------------------------|
| 129  | Motor Torque Limit | Torque (N·m×10) | Maximum motoring torque                             |
| 130  | REGEN Torque Limit | Torque (N·m×10) | Regen torque when pedal released (no brake)         |
| 131  | Braking Torque Limit | Torque (N·m×10) | Regen torque when brake active                    |
| 164  | Kp Torque          | Prop. Gain (×10000) | Torque regulator proportional gain              |
| 165  | Ki Torque          | Int. Gain (×10000)  | Torque regulator integral gain                  |
| 166  | Kd Torque          | Deriv. Gain (×100)  | Torque regulator derivative gain                |
| 167  | Klp Torque         | LP Gain (×10000)    | Torque regulator low-pass filter gain           |
| 168  | Torque Rate Limit  | Torque  | Max torque change per step (0.1–250 N·m); slows ramp       |

**Speed (addr 111, 126–128, 160–163, 169)**

| Addr | Name               | Format           | Description                                            |
|------|--------------------|------------------|--------------------------------------------------------|
| 111  | Motor Over-speed   | Angular Velocity | Over-speed fault threshold (RPM)                       |
| 126  | REGEN Fade Speed   | Angular Velocity | Speed below which regen torque begins to fade          |
| 127  | Break Speed        | Angular Velocity | Speed below which max torque is reduced (field weak.)  |
| 128  | Max Speed          | Angular Velocity | Maximum allowable speed                               |
| 160  | Kp Speed           | Prop. Gain       | Speed regulator proportional gain                      |
| 161  | Ki Speed           | Int. Gain        | Speed regulator integral gain                          |
| 162  | Kd Speed           | Deriv. Gain      | Speed regulator derivative gain                        |
| 163  | Klp Speed          | LP Gain          | Speed regulator low-pass filter gain                   |
| 169  | Speed Rate Limit   | Speed            | Max speed change per step (100–5100 RPM)               |

### 7.10 CAN Timeout / Watchdog

If EEPROM parameter **CAN Command Message Active** (addr 146) = 1:
- The PM100 expects a 0x0C0 message within **CAN Timeout** × 3 ms (default 333 × 3 = 999 ms).
- If the timeout expires: **Run Fault bit 11** (CAN Command Message Lost) is set and the inverter disables.
- Recommendation: send 0x0C0 at 10–50 ms intervals for responsive vehicle control.
- If CAN Command Message Active = 0 (default), the inverter holds the last received command indefinitely.

### 7.11 Rolling Counter (Gen 5 / CM firmware only)

Gen 5 and CM controllers support an optional rolling counter in Byte 5 bits 4–7 of the 0x0C0
command message to detect lost or repeated CAN frames. The counter increments 0→15→0.

| EEPROM Parameter              | Addr | Default | Description                                           |
|-------------------------------|------|---------|-------------------------------------------------------|
| CAN Debounce Counter Max      | 238  | 20      | Debounce count that triggers a fault; 0=disable RC    |
| CAN Debounce Up Count         | 239  | 5       | Counts added per rolling counter error                |
| CAN Debounce Down Count       | 240  | 3       | Counts removed per correct rolling counter message    |

Fault triggered: Run Fault bit 11 (same as CAN Command Timeout).

### 7.12 Orion BMS Integration

Enable via EEPROM CAN BMS Limit Enable (addr 178 = 1). The PM100 then accepts:

| BMS CAN ID | 0x202 (514 decimal)                                  |
|------------|------------------------------------------------------|
| Byte 0–1   | Maximum discharge current in Amps (little-endian, unsigned) |
| Byte 2–3   | Maximum charge current in Amps (little-endian, unsigned)    |
| Byte 4–7   | Unused                                               |

Example: `0x02 0x01 0x04 0x02 0x00 0x00 0x00 0x00` → discharge limit = 258 A, charge limit = 516 A.

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

### 8.2 CM Diagnostic Data Viewer

The **CM Diagnostic Data Viewer** is a separate Windows tool provided by Cascadia Motion for offline analysis of inverter data logs and fault snapshots. It complements RMS GUI but does not require a live RS-232 connection — it works on saved log files captured from the PM100.

| | RMS GUI | CM Diagnostic Data Viewer |
|---|---|---|
| Connection | Live RS-232 | Offline — reads saved log files |
| Use case | Real-time parameters, config, fault monitoring | Replay and inspect fault snapshots after the fact |
| When to use | During bench testing and commissioning | Post-incident fault analysis |

Download (Cascadia Motion hosted): `https://app.box.com/s/u0b3yx87set8b3e41w93io0zrq6bpe1u/file/923752730376`
Release 104 (February 2022), ~2.5 MB ZIP — extract and run on Windows.

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

## 14. References

The table below lists the source documents used to compile this reference guide. Where content is derived from a specific source, the section notes it. If the BlueBoy firmware version differs from the document versions listed here, cross-check the relevant source before relying on specific byte offsets, parameter addresses, or scaling factors.

| Ref | Document | Version / Date | Publisher | Notes |
|-----|----------|---------------|-----------|-------|
| [1] | CAN Protocol | Rev 5.9, 25 Feb 2022 | Cascadia Motion | Primary source for all of §7 (CAN messages, byte definitions, EEPROM parameter addresses). Covers PM, RM, PM Gen 5, and CM families. Firmware-specific differences noted within §7. |
| [2] | PM100 User Manual | 3/8/2011 | Rinehart Motion Systems LLC | Primary source for connector pinouts (§3, §4), power connections (§2), VSM (§10), pre-charge (§3.3.2), RS-232 (§6), and I/O electrical details (§5). |
| [3] | PM100DX Inverter Drawing | 180-100-002.00, Rev 2.3 | Swindon Silicon Systems / SWIND EV | Factory low-voltage wiring schematic (3 sheets). Sheet 1 (general arrangement) publicly available as rev .01 from Swindon Powertrain webshop. Sheets 2–3 not publicly hosted. |
| [4] | PM100 MotoHawk CAN Library | — | New Eagle / Rinehart | Supplementary CAN signal definitions. Available via New Eagle wiki. |
| [5] | AMPSEAL Connector Datasheet | — | TE Connectivity | J1 connector PN 776164-1, J2 connector PN 770680-1, crimp contact PN 770854-1. |

### Firmware version notes for §7 (CAN)

Several CAN message formats changed across firmware releases. If RMS GUI shows a firmware version outside the ranges below, verify the affected messages against [Ref 1] directly.

| Firmware version | Change |
|---|---|
| Before fw 1953 | Bytes 6–7 of 0x0C0 (Torque Limit) not supported — send `0x0000` |
| Before fw 1995 | 0x0A3 uses four 16-bit analog inputs; fw 1995+ packs six 10-bit inputs |
| Before fw 2042 | 0x0B0 (333 Hz high-speed message) does not exist; disabled by default even when present |
| Before fw 2048 | Speed Command bytes (0x0C0 bytes 2–3) only used in Speed Mode; fw 2048+ also applies as Max Speed override in Torque Mode |
| Gen 5 / CM only | Rolling counter (§7.10) and bits 4–7 of 0x0C0 byte 5 not present on PM Gen 2 (PM100) |

**To identify the firmware version on BlueBoy's unit:** connect RMS GUI and read the firmware version displayed on the main screen, or read CAN message 0x0AE bytes 4–7 (software version + build date).

---

## Appendix A — CAN Bus Concepts

### A.1 Dominant and Recessive Bits

Every bit transmitted on a CAN bus is either **dominant** or **recessive** — these are the two logic states of the differential pair.

| State | Logic | Voltage (typical) | Behaviour |
|---|---|---|---|
| Dominant | 0 | CAN_H ~3.5 V, CAN_L ~1.5 V (~2 V differential) | Wins if two nodes transmit simultaneously |
| Recessive | 1 | CAN_H and CAN_L both ~2.5 V (~0 V differential) | Loses to a dominant bit |

The key property is that **dominant overrides recessive** on the wire. If one node transmits dominant (0) and another simultaneously transmits recessive (1), the bus reads dominant. This is how CAN handles arbitration — the node with the lower message ID (more leading zeros in its 11-bit identifier) wins the bus without a collision or retransmission.

**Recessive bits in the EOF field** mean the transmitting node releases the bus — it stops driving the differential voltage and allows the line to float back to its idle state (~0 V differential). The 7 recessive EOF bits signal to all other nodes that the frame is complete and the bus is free.

In practice this is handled entirely by the CAN controller hardware. It is only relevant when debugging at the electrical level with an oscilloscope.
