# Cascadia Motion MCU / RS-232 / J1 Connector Summary

## What was covered

This conversation focused on:

- how to use **C2Prog / c2prog.exe**
- how to communicate with a **Cascadia Motion motor control unit (MCU)**
- how to test the **RS-232** connection from **Windows 10** and **macOS**
- how to understand **program mode vs normal run mode**
- how the **35-way AMPSEAL J1** connector maps to signals
- what **RMS GUI** is and where to get it
- how to represent a typical **MCU / battery / motor** system and the **regen path**
- creation of several reference diagrams

---

## Main conclusions

### 1. C2Prog usage
For Cascadia Motion units, **C2Prog** is primarily the firmware flashing tool used when the controller is in **boot/programming mode**.

General flashing workflow discussed:

1. connect to the MCU via **RS-232**
2. ground **Program Enable**
3. power the unit so it enters boot mode
4. select the correct COM port / target in C2Prog
5. load the firmware file
6. flash the unit
7. power down, remove Program Enable from ground, and restart normally

We also discussed that some C2Prog installations expose:
- GUI usage
- newer command-line usage via `c2p-cli`
- older command-line usage via `C2ProgShell.exe`

---

### 2. Communicating with the MCU
Two main communication paths were discussed:

- **RS-232** for setup, service, and diagnostics
- **CAN A** for monitoring and control

For simple bench/service communication, **RS-232** is the easiest place to start.

---

### 3. Best way to check if the MCU is alive
The best functional method discussed was:

- connect with **RS-232**
- leave **Program Enable not grounded**
- use **RMS GUI**
- look for firmware/version info and live parameter updates

A generic serial terminal can help prove the serial port opens, but it may show no readable text even when the link is electrically fine.

---

## RS-232 wiring summary

### RS-232 signal mapping
For the MCU serial connection:

- **PC / adapter TXD** -> **MCU RS232_RXD**
- **PC / adapter RXD** -> **MCU RS232_TXD**
- **PC / adapter GND** -> **MCU RS232_GND**

### 120 ohm resistor question
We established:

- **No 120 ohm resistors are used on RS-232**
- **120 ohm termination is for CAN bus**, not RS-232

---

## Windows 10 serial testing summary

We discussed these typical checks:

1. locate the COM port
2. verify the adapter is visible in Device Manager
3. run loopback on the adapter by shorting TX to RX
4. open the COM port with a terminal tool
5. use **RMS GUI** for a real functional communication test

---

## macOS serial testing summary

On macOS, the basic serial workflow discussed was:

```bash
ls /dev/cu.*
screen /dev/cu.usbserial-FTES73H7 115200
```

Important conclusions:

- when only `/dev/cu.Bluetooth-Incoming-Port` and `/dev/cu.debug-console` appeared, the USB serial adapter was not yet detected
- when `/dev/cu.usbserial-FTES73H7` appeared, the adapter was detected properly
- if `screen` opened with no complaint, the Mac could open the serial port
- seeing **no text** in `screen` does **not** prove failure, because the MCU may not emit plain ASCII data on its own

Recommended next steps discussed:

- do a loopback test on the adapter
- verify TX/RX crossover and GND
- power-cycle the MCU while `screen` is open
- use vendor software rather than relying only on a dumb terminal

---

## Program mode vs normal run mode

### Meaning of `/PROG_ENA`
We clarified that:

- `/PROG_ENA` means **Program Enable**
- the leading slash indicates **active-low**
- the pin is asserted by connecting it to **ground**

### Operational meaning
- **/PROG_ENA not grounded at power-up** -> normal run mode
- **/PROG_ENA grounded at power-up** -> boot/programming mode

This was a key distinction for understanding why a unit might not behave normally on RS-232.

---

## 35-way AMPSEAL J1 connector summary

We discussed the verified key J1 pin mapping used for RS-232 and CAN:

- **J1-7** = `/PROG_ENA`
- **J1-11** = `CANA_L`
- **J1-12** = `TXD`
- **J1-22** = `GND`
- **J1-33** = `CANA_H`
- **J1-35** = `RXD`

### J1 to DB9 RS-232 mapping
The practical serial mapping discussed was:

- **J1-12 TXD** -> **DB9 pin 2**
- **J1-35 RXD** -> **DB9 pin 3**
- **J1-22 GND** -> **DB9 pin 5**

For boot mode only:

- **J1-7 /PROG_ENA** temporarily jumpered to **J1-22 GND**

---

## Complete 35-way AMPSEAL J1 pin chart discussed

| Pin | Signal | Description / notes |
|---|---|---|
| 1 | XDCR_PWR | +5 V @ 80 mA max, accel pedal power |
| 2 | AGND | Analog ground, accel pedal ground |
| 3 | AIN4 | Analog input 4, 0–5 VFS |
| 4 | RTD1 | 1000 Ω RTD input |
| 5 | RTD4 | 100 Ω RTD input |
| 6 | RTD5 | 100 Ω RTD input |
| 7 | /PROG_ENA | Serial boot loader enable |
| 8 | DIN2 | Digital input, reverse enable switch |
| 9 | DIN5 | Digital input, ignition input if used |
| 10 | Reserved | Do not connect |
| 11 | CANA_L | CAN channel A low |
| 12 | TXD | RS-232 transmit |
| 13 | AIN1 | Analog input 1, 0–5 VFS, accel pedal wiper |
| 14 | XDCR_PWR | +5 V @ 80 mA max, spare transducer power |
| 15 | AGND | Analog ground |
| 16 | RTD2 | 1000 Ω RTD input |
| 17 | AGND | Analog ground |
| 18 | Reserved | Do not connect |
| 19 | AGND | Analog ground |
| 20 | DIN3 | Digital input, brake switch |
| 21 | DIN6 | Digital input, start input if used |
| 22 | GND | Ground |
| 23 | CANB_H | CAN channel B high |
| 24 | AIN2 | Analog input 2, 0–5 VFS, motor temperature sensor |
| 25 | AIN3 | Analog input 3, 0–5 VFS, brake pedal |
| 26 | XDCR_PWR | +5 V @ 80 mA max, spare transducer power |
| 27 | RTD3 | 1000 Ω RTD input |
| 28 | XDCR_PWR | +5 V @ 80 mA max, spare transducer power |
| 29 | Reserved | Do not connect |
| 30 | DIN1 | Digital input, forward enable switch |
| 31 | DIN4 | Digital input, regen disable input if used |
| 32 | Reserved | Do not connect |
| 33 | CANA_H | CAN channel A high |
| 34 | CANB_L | CAN channel B low |
| 35 | RXD | RS-232 receive |

---

## RMS GUI summary

We clarified that **RMS GUI** is Cascadia Motion’s normal **Windows service/setup application** for the inverter.

Its role, as discussed:

- communicate with the inverter over **RS-232**
- monitor live parameters in real time
- read/write certain EEPROM configuration values
- inspect firmware/version information

We contrasted it with:

- **C2Prog** = bootloader / firmware flashing tool
- generic terminal tools = useful for port testing, but not protocol-aware

We also provided the official documentation page link where the firmware folder includes RMS GUI:

- `https://www.cascadiamotion.com/documentation`

---

## Wiring and system-diagram discussion

We created or discussed several diagrams, including:

- RS-232 wiring diagram for the MCU
- full 35-pin connector pinout images
- conceptual MCU / battery / inverter / motor wiring diagrams
- a stricter J1 connector diagram
- a flat J1 pin chart image
- a revised powertrain diagram showing the **regen current path through the inverter DC bus**

---

## Regenerative braking / energy flow clarification

A key correction made during the conversation:

The **three-phase motor does not feed directly back to the battery on separate direct wires**.

The physically accurate conceptual path is:

**Motor U/V/W -> Inverter power stage -> Inverter DC bus -> HV battery pack**

We explicitly refined the diagram language to say:

- **regen current through inverter DC bus**
- **motor does not feed battery directly**
- battery charging depends on the inverter, DC bus, contactors, BMS, and battery acceptance

---

## Practical takeaways

If you need a simple service checklist:

1. verify the USB-to-RS232 adapter works
2. verify TX/RX crossover and GND
3. do **not** ground `/PROG_ENA` unless you are intentionally entering boot mode
4. for normal communication, use **RMS GUI**
5. for firmware flashing, use **C2Prog**
6. use the verified J1 pins:
   - J1-7 `/PROG_ENA`
   - J1-11 `CANA_L`
   - J1-12 `TXD`
   - J1-22 `GND`
   - J1-33 `CANA_H`
   - J1-35 `RXD`

---

## Final note

Some generated diagrams were conceptual rather than factory-verified production schematics. The most reliable pin information captured in this conversation is the verified **35-way AMPSEAL J1 pin mapping** and the RS-232 wiring guidance above.
