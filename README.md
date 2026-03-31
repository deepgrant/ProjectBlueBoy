# BlueBoy EV — Cascadia Motion PM100 Integration

This repository documents the integration of the **Cascadia Motion / Rinehart Motion Systems PM100** AC motor inverter into the BlueBoy EV powertrain project.

The primary reference is **[PM100_Reference_Guide.md](PM100_Reference_Guide.md)**, covering:

| Topic | Section |
|---|---|
| System overview and powertrain architecture | [§1 System Architecture Overview](PM100_Reference_Guide.md#1-system-architecture-overview) |
| HV DC and 12V logic power connections | [§2 Power Connections](PM100_Reference_Guide.md#2-power-connections) |
| J1 35-pin AMPSEAL connector pinout | [§3 J1 Connector](PM100_Reference_Guide.md#3-j1-connector--35-pin-ampseal) |
| J2 23-pin AMPSEAL connector pinout | [§4 J2 Connector](PM100_Reference_Guide.md#4-j2-connector--23-pin-ampseal) |
| Analog/digital I/O electrical specs | [§5 I/O Electrical Details](PM100_Reference_Guide.md#5-inputoutput-electrical-details) |
| RS-232 wiring, baud rate, and serial testing | [§6 RS-232 Serial Interface](PM100_Reference_Guide.md#6-rs-232-serial-interface) |
| CAN A/B bus wiring and termination | [§7 CAN Bus Interface](PM100_Reference_Guide.md#7-can-bus-interface) |
| RMS GUI diagnostics software | [§8 RMS GUI Software](PM100_Reference_Guide.md#8-rms-gui-software) |
| C2Prog firmware flashing procedure | [§9 C2Prog Firmware Flashing](PM100_Reference_Guide.md#9-c2prog-firmware-flashing) |
| Enable/disable and fault state machine | [§10 Vehicle State Machine](PM100_Reference_Guide.md#10-vehicle-state-machine-vsm) |
| Regenerative braking energy path | [§11 Regenerative Braking](PM100_Reference_Guide.md#11-regenerative-braking) |
| EEPROM configuration parameters | [§12 Key Configuration Parameters](PM100_Reference_Guide.md#12-key-configuration-parameters) |
| Terminology and abbreviations | [§13 Glossary](PM100_Reference_Guide.md#13-glossary) |

## Where to start

- **New to the system?** Start with [§1 System Architecture Overview](PM100_Reference_Guide.md#1-system-architecture-overview).
- **Wiring the J1 connector?** See [§3 J1 Connector](PM100_Reference_Guide.md#3-j1-connector--35-pin-ampseal) and [§6 RS-232 Serial Interface](PM100_Reference_Guide.md#6-rs-232-serial-interface).
- **Flashing firmware?** See [§9 C2Prog Firmware Flashing](PM100_Reference_Guide.md#9-c2prog-firmware-flashing).
- **Running diagnostics?** See [§8 RMS GUI Software](PM100_Reference_Guide.md#8-rms-gui-software).

Official Cascadia Motion documentation and software downloads: `https://www.cascadiamotion.com/documentation`
