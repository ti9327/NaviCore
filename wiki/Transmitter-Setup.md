# Transmitter Setup

NaviCore decodes a standard FrSky SBUS stream. It ships with metadata for three transmitter models, and a ready‑made model file for the X18 so you don't have to build the channel map by hand.

Pick your model in the config tool under **Config → Transmitter**. The model drives which SVG is shown, the default channel assignments, the control labels, and which controls appear in the calibration wizard. (The firmware itself is model‑agnostic — every control is gated on its assigned channel, so unused controls just sit at channel 0.)

---

## Supported models

| Model | Notes |
|-------|-------|
| **FrSky Tandem X18** *(default)* | Full‑size. 10 switches, 2 knobs (S1/S2), 2 side sliders (LS/RS). |
| **FrSky Twin X‑Lite** | Handheld. 6 switches (SE/SF momentary), 2 knobs, no sliders, 4 face buttons. |
| **FrSky Twin X20** | X18 layout **plus** an MS middle slider and, with the optional **3‑axis gimbal** upgrade, J5/J6 twist axes and two stick‑click momentary buttons (matrix slots 19/20). Enable via the *3‑axis gimbals* checkbox on the Transmitter tab. |

---

## Default channel map (X18, Mode 2)

This is the layout the firmware defaults to. You can override any of it per‑control via calibration, so a custom mixer setup still works.

| Channel | Control | Role |
|---------|---------|------|
| CH1 | **J1** | Aileron — right‑stick X |
| CH2 | **J2** | Elevator — right‑stick Y |
| CH3 | **J3** | Throttle — left‑stick Y |
| CH4 | **J4** | Rudder — left‑stick X |
| CH5 | **S1** | Rotary knob |
| CH6 | **S2** | Rotary knob |
| **CH7** | **Button matrix** | All matrix buttons, multiplexed (see below) |
| CH8 | **SA** | 3‑position switch |
| CH9 | **SB** | 3‑position switch |
| CH10 | **SC** | 3‑position switch |
| CH11 | **SD** | 3‑position switch |
| CH12 | **SE** | 3‑position switch — **mode selector** by default |
| CH13 | **SF** | 2‑position switch |
| CH14 | **SG** | 3‑position switch |
| CH15 | **SH** | 2‑position switch |

Disabled by default (channel 0 — assign in the tool if your build has them): **SI, SJ**, sliders **LS/RS**, X20 **MS** slider, X20 **J5/J6** twist axes.

> **The button matrix (CH7)** carries every physical button as a distinct PWM band — pressing a button drives CH7 to that button's value, and NaviCore decodes which one. That's why a single channel can serve ~20 buttons. The default band table (B1–B6, the T1–T6 trims as Left/Right or Up/Down, and the X20 stick‑clicks) is built from measured X18 values; **calibrate** to lock the bands to your radio.

---

## Loading the X18 model file

The repo bundles a matching transmitter model so your radio outputs exactly the channel map above:

1. Find `model/r2 web gui.bin` in the repository (~2.4 KB).
2. Copy it to the **`MODELS/`** folder on your transmitter's SD card.
3. On the radio, open the **model select** screen and load it.

This sets up the stick/switch/knob channel assignments and the CH7 button‑matrix mixer. If you use your own model, that's fine — just **calibrate** in the config tool so NaviCore learns your channels and matrix bands.

---

## Modes

A 3‑position switch selects one of **three modes** (Mode 1 / 2 / 3). Each mode has its own complete set of button mappings — so the same physical button can do different things in each mode, tripling your control surface. The mode switch defaults to **SE (CH12)** and can be reassigned (or disabled) under the config tool's binding settings.

- Switch position thresholds (SBUS counts): **< 582** = Mode 1 (down), **582–1401** = Mode 2 (mid), **> 1401** = Mode 3 (up).

---

### Next: [[Config Tool Guide]] · [[Actions Reference]]
