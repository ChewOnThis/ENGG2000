# MQ Sentinel

This repository contains the ENGG2000 MQ Sentinel Arduino work: infrared
target detection, reaction-wheel alignment, laser firing, encoder feedback,
and the experiments used to tune those behaviours.

## Start Here

- [Project/Documentation.md](Project/Documentation.md) describes the current
  project sketch, wiring, control flow, and limitations.
- [Project/Project.ino](Project/Project.ino) is the simple reference tracker.
- `Goated/` contains the most developed controller experiments, including
  EEPROM-backed flight logging and high-speed precision alignment.
- [Testing/Documentation.txt](Testing/Documentation.txt) records the test
  sketches and their purpose.
- `Psuedocode/` contains the proposed modular design and pseudocode. It is a
  design reference, not the implementation used by the main sketch.

## Repository Map

| Location | Role |
| --- | --- |
| `Project/` | Current simple three-sensor tracker and its documentation |
| `Goated/` | Advanced V2 and V8.8.3 EEPROM/logging controllers |
| Root `MQ_Sentinel_*` folders | Standalone V7/V8 controller iterations |
| `Testing/3SensorMiddleStopSerialPrintElectricBraking/` | Focused motor and sensor behaviour test |
| `Testing/HangTest/` | IR decoding and receiver diagnostics |
| `Testing/SlopTest/` | Alignment, filtering, reaction-wheel, and state-machine experiments |
| `Psuedocode/` | Planned modular architecture and supporting pseudocode |
| `SensorDetector.ino` | Minimal IR receiver and LED diagnostic |

Most Arduino sketches are standalone and must be opened in a folder whose
sketch name matches the `.ino` filename before compiling in the Arduino IDE.
The repository does not currently contain an automated build or unit-test
runner; hardware validation is documented as a manual test activity.

## Common Hardware

The main tracker sketches use an Arduino Nano/ATmega328P with three active-LOW
38 kHz IR receivers, a two-input motor driver, a laser output, and an A/B
encoder. The common pin assignment is:

| Function | Pin |
| --- | --- |
| Left, centre, right IR | D2, D3, D4 |
| Motor driver IN1, IN2 | D6, D7 |
| Laser | D8 |
| Encoder A, B | D9, D10 |
| Status LED (advanced sketches) | D13 |

Check the selected sketch before wiring: advanced variants may use interrupt
capture, PWM, status LED output, EEPROM, and different serial protocols.

## Safety

Treat the laser and motor as live hardware. Test with the laser disabled or
physically shielded, secure the reaction wheel, limit motor power, and use the
sketch's safe-stop/standby controls where provided. Do not assume that a
successful compile proves that the physical alignment or laser aim is safe.
