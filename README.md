# ZVS Driver with Protection

A **Mazzilli ZVS (zero-voltage switching) driver** for wireless energy transfer, built as a complete PCB with hardware overcurrent protection and an ATmega328P supervisor that monitors the oscillation, controls the startup and shows the board status on LEDs.

> **Status:** simulation (LTspice) and firmware (SimulIDE) done · schematic in KiCad in progress · PCB not yet built

---

## Features

- **Mazzilli ZVS power stage** at about **111 kHz**, powered from a **12 V DC** input
- **Soft start** that holds the MOSFET gates off at power-up
- **Hardware overcurrent protection**: shunt → current-sense amplifier → comparator → latch → gates off within microseconds, independent of the firmware
- **Two current limits** set by the ATmega: a high limit during startup (to allow the inrush current) and a normal limit during operation
- **Oscillation watchdog**: the ATmega counts the drain pulses and shuts the stage down if the frequency is missing or outside its window
- **Fault diagnosis on LEDs**: overcurrent and oscillation faults are shown separately
- **Safe by default**: the latch starts in the "off" state at power-up and after any fault, and only a button press restarts the stage

---

## How it works

```
12 V ─ Fuse ─ Shunt (5 mΩ) ─ Soft start ─ Mazzilli ZVS ─ Coupled coils ─ Rectifier ─ Load
                  │                            ▲      │
             INA181A1 (×20)                    │      │ drain signal
                  │                      gates off    │ (divider, clamp, Schmitt trigger)
             RC filter                         │      ▼
                  │                     ┌─ Fault latch ◄─── ATmega328P
             LM393 comparator ──────────┘  (74HC02)        - threshold PWM
                  ▲                                        - frequency watchdog
                  └──── threshold (PWM + RC filter) ◄──────  - LEDs, reset button
```

1. **Overcurrent:** the INA181 amplifies the voltage across the shunt. When it exceeds the threshold, the LM393 sets the latch, and two small MOSFETs pull both main gates to ground.
2. **Oscillation watchdog:** one drain is divided down, clamped and cleaned up by a Schmitt trigger, then counted by Timer1 of the ATmega. A missing or wrong frequency makes the ATmega set the latch.
3. **Startup:** the ATmega first sets the high threshold, waits for it to settle, then resets the latch. After a blanking time and a valid frequency, it switches to the normal threshold.

---

## Key parameters

| Parameter | Value |
|---|---|
| Input voltage | 12 V DC |
| Switching frequency | ~111 kHz (tank: 2 µH, 1 µF) |
| Feed chokes | 2 × 100 µH |
| Shunt | 5 mΩ |
| Current-sense amplifier | INA181A1 (gain 20, 0.1 V/A) |
| Trip current, startup | 30 A (first ~30 ms) |
| Trip current, running | 8 A |
| Frequency window | 90–130 kHz (to be confirmed by measurement) |
| Comparator | LM393, supplied from 12 V, open-collector output |
| Fault latch | 74HC02 (two NOR gates as SR latch) |
| Microcontroller | ATmega328P-PU, 16 MHz crystal, 5 V |
| Logic supply | 7805 linear regulator |

---

## LED indication

| LEDs | State |
|---|---|
| All off | Booting |
| 🟢 Green blinking | Startup, high current limit active |
| 🟢 Green steady | Running normally |
| 🔴 Red | Overcurrent trip |
| 🟡 Yellow | Oscillation fault (missing or wrong frequency) |

After a fault, the stage stays off until the **reset button** is pressed.

---

## Repository structure

```
ZVS-Driver/
├── kicad/        KiCad schematic and PCB
├── ltspice/      LTspice simulations (power stage, protection, coupling sweeps)
├── firmware/
│   └── zvs_supervisor/
│       └── zvs_supervisor.ino
├── docs/         Notes, measurements, photos
└── README.md
```

---

## Firmware

**Toolchain:** Arduino IDE 2.x with the [MiniCore](https://github.com/MCUdude/MiniCore) board package.

| Setting | Value |
|---|---|
| Board | MiniCore → ATmega328 |
| Clock | External 16 MHz |
| BOD | 4.3 V |
| Bootloader | No bootloader (programmed via ISP) |
| Programmer | Arduino as ISP |

The same code also runs unchanged on an Arduino Uno or Nano for testing.

### Pin map

| Arduino | ATmega pin | Function |
|---|---|---|
| D2 | 4 | Reset button (to GND) |
| D3 | 5 | Threshold PWM (Timer2, ~31 kHz) |
| D5 | 11 | Oscillation input (Timer1, T1) |
| D6 | 12 | Latch S (via 1N5819) |
| D7 | 13 | Latch R |
| D8 | 14 | Latch Q (read back) |
| A0 / A1 / A2 | 23 / 24 / 25 | Green / red / yellow LED |
| A3 | 26 | Current-sense output (debug) |
| D0 / D1 | 2 / 3 | Serial debug, 115200 baud |

All thresholds, timings and the frequency window are constants at the top of `zvs_supervisor.ino`.

### Programming

The board has a 6-pin ISP header (MISO, VCC, SCK, MOSI, RESET, GND). An Arduino Uno running the **ArduinoISP** example works as programmer (D10 → RESET, D11 → MOSI, D12 → MISO, D13 → SCK, plus 5 V and GND, and 10 µF between RESET and GND on the Uno).

1. Burn the fuses once: **Tools → Burn Bootloader**.
2. Upload: **Sketch → Upload Using Programmer**.

⚠️ **Program the board with the 12 V supply disconnected.** While the ATmega is being programmed it cannot supervise the power stage.

### Serial debug

A 3-pin header (GND, TX, RX) connects to a 5 V USB-UART adapter (CH340 or CP2102). TX and RX cross over, and the adapter's VCC stays unconnected.

---

## Simulation

- **LTspice:** power stage, soft start, current sensing, comparator, latch and drain sensing, with faults (shorted tank) and coupling-factor sweeps between the coils. The TI models for the INA181 and LM2903B/LM393 are available from [ti.com](https://www.ti.com) and are not included in this repository.
- **SimulIDE:** the ATmega328P running the compiled firmware, with LEDs, button, latch and a test signal on the oscillation input.

---

## Roadmap

- [x] Power stage and protection simulated in LTspice
- [x] Firmware written and tested in SimulIDE
- [ ] Finish KiCad schematic and run ERC
- [ ] Breadboard test of the control and protection circuit
- [ ] Perfboard test of the power stage with a current-limited supply
- [ ] PCB layout, revision A
- [ ] Measurements on the real board and tuning of the thresholds

---

## Safety

This circuit handles high currents, and the drain voltages reach several times the supply voltage. Test new hardware with a **current-limited lab supply**, start at low voltage, and never connect an oscilloscope ground clip to a drain. The coil and workpiece can get hot.
