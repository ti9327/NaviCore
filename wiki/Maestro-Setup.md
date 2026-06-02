# Maestro Setup

Pololu **Maestro** servo controllers are how NaviCore moves servos — locally over a wired bus, and remotely over ESP‑NOW. Getting multiple Maestros to share a bus is the most common stumbling block, so this page walks through it end to end. For the action syntax see [[Actions Reference]]; for pins see [[Hardware and Wiring]].

---

## The model: 8 logical slots

NaviCore has **8 logical Maestro slots (1–8)**. A Maestro action targets a slot; where that slot physically lives is set once in **Config → Maestro (Locations)**:

| Slot `type` | Where the bytes go |
|-------------|--------------------|
| **Local** | Out the wired Maestro bus on **GPIO6** (Serial2), at the local‑Maestro baud. |
| **Remote** | Broadcast over ESP‑NOW; any WCB with **Kyber_Remote** enabled forwards them to *its* Maestro port. |
| **Disabled** | Slot ignored. |

Each slot also stores a **device number** — see below.

In the config schema this is the `maestros[]` array (8 entries) plus `auxBaud.maestro`. See [[Configuration Schema]].

---

## Device numbers are mandatory

NaviCore **always speaks Pololu protocol** (never "compact"), which means every command is addressed to a specific **device number**. This is what lets several Maestros sit on one wire (or one ESP‑NOW broadcast bus) and only the addressed one act.

> **Every Maestro must have a unique device number**, and that number must match what you enter for its slot in NaviCore. Two Maestros with the same number on one bus will both react — and collide.

### Set it in Maestro Control Center

1. Connect the Maestro to a PC via USB, open **Pololu Maestro Control Center**.
2. **Serial Settings** tab:
   - **Serial mode:** *UART, fixed baud rate* (set it to **115200** to match NaviCore's default), **or** *UART, detect baud rate*.
   - **Device Number:** give each Maestro a unique value (0–127).
   - **CRC:** disabled (NaviCore doesn't send CRC).
3. **Apply Settings** (writes to the Maestro's own flash).

Then in NaviCore's **Maestro Locations** panel, set that slot's device number to the same value and choose Local or Remote.

---

## Wiring a local Maestro

- NaviCore **TX = GPIO6** → Maestro **RX**.
- Common **GND**.
- (Maestro TX → NaviCore RX on GPIO7 is optional — NaviCore doesn't need Maestro feedback.)
- Baud defaults to **115200** (`auxBaud.maestro`). Either set the Maestro to fixed 115200 or use *detect baud rate*.
- Multiple local Maestros: daisy‑chain their serial lines on the same bus; unique device numbers keep them independent.

---

## Remote Maestros (over WCB)

No wire from NaviCore. NaviCore broadcasts the Maestro bytes on the Kyber path; a WCB elsewhere with **Kyber_Remote** configured forwards them to the Maestro wired to *that* WCB. Set the slot to **Remote** and give it the right device number. See [[WCB Network]].

---

## Command forms

A Maestro action's `cmd` is one of:

| `cmd` | Effect |
|-------|--------|
| `setTarget,<ch>,<pos>` | Move servo channel `<ch>` (0–31) to `<pos>` quarter‑µs (e.g. `4000`–`8000`). |
| `setSpeed,<ch>,<spd>` | Set channel speed limit. |
| `setAccel,<ch>,<acc>` | Set channel acceleration limit. |
| `goHome` | Send all channels to their configured home positions. |
| `stopScript` | Stop the Maestro's onboard script. |
| `restartScript,<n>` | Start the onboard script at subroutine `<n>`. |

Knobs/sliders/gimbal axes can also drive a Maestro continuously (**Maestro Pass‑Through**) — pick a slot, a servo channel, and the qus range at the stick's min/max. See [[Config Tool Guide]].

---

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| Nothing moves (local) | TX/RX swapped, no common GND, baud mismatch (set Maestro to 115200 or detect‑baud), or slot not set **Local**. |
| Nothing moves (remote) | Receiving WCB lacks **Kyber_Remote**, wrong device number, or slot not set **Remote**. |
| *Every* Maestro reacts to one command | Duplicate device numbers — give each a unique number in Control Center and match it in NaviCore. |
| Servo jitters / wrong range | `setTarget` qus out of the servo's range; clamp `posMin`/`posMax` to a safe travel. |
| Works on the bench, not in the droid | Power — servos can brown out the bus; give the Maestro its own servo‑power supply with shared ground. |

More in [[Troubleshooting]].

---

### Next: [[Actions Reference]] · [[Hardware and Wiring]] · [[WCB Network]]
