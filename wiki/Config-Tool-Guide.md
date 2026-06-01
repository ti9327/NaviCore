# Config Tool Guide

Everything is configured from the browser — no IDE, no app, no server. The config tool talks to the board over the **Web Serial API**.

> **Browser:** Chrome or Edge (desktop). Firefox and Safari don't support Web Serial.

---

## Connecting

### Direct USB (normal)

1. Open `config_tool/index.html` (locally, or the hosted GitHub Pages link).
2. Plug the board in over USB.
3. Click **Connect** and pick the board's serial port.

On connect, the tool pings the board (reads its firmware version), pulls the current config (`GET_CONFIG`), and starts the live monitor. The board's version shows in the footer.

### Via WCB (wireless bridge)

If you can't tether the controller directly, the tool can bridge to it **through any USB‑connected WCB** over ESP‑NOW:

- Connect to a tethered WCB, enable **Via WCB**, and the tool relays config/monitor traffic to the RC (which always claims WCB device ID **20**).
- Large `SET_CONFIG`/`GET_CONFIG` payloads are automatically fragmented to fit ESP‑NOW's ~250‑byte packets, so saving over the bridge is slower than USB but works.
- **Firmware flashing is not available** over Via WCB — it needs a direct USB connection.

---

## The live monitor

While connected, the board streams a `PWM_UPDATE` ~20×/second. The tool uses it to:

- show every SBUS channel's live value,
- highlight the active transmitter control on the SVG,
- display **SBUS health** — frames/sec, lost‑frame and failsafe flags, detected channel count (16 or 24),
- light up the matrix button currently being pressed and the current mode.

If SBUS isn't healthy (no frames, lost frames, or stale data), the monitor flags it — see [[Troubleshooting]].

---

## Mapping controls

Open **Config** (gear) to reach the tabs: **Channels**, **Transmitter**, **WCB Network**, **HCR**, **MP3 Trigger**, **Maestro**, **Aux Serial**, **Firmware**.

### Buttons

- **Click a button** on the transmitter graphic to open its mapping editor.
- Each button has **three tap tiers**: single, double, and triple tap. Add up to **5 actions** per tier (see [[Actions Reference]]).
- **Exclusive vs cumulative**:
  - *Exclusive* — only the matched tier fires (a double‑tap fires the double‑tap actions alone).
  - *Cumulative* — every tier up to the match fires (a double‑tap fires single **then** double).
- Mappings are **per mode** — switch the mode tab (1/2/3) to map the same button differently in each mode.
- The **tap window** (how long the board waits to see if more taps are coming) defaults to **500 ms** and is adjustable.

You can also **fire a button from the tool** to test its mapping without touching the radio:
- **Shift+Click** = single tap · **Shift+Ctrl+Click** = double · **Shift+Alt+Click** = triple.

### Switches

- Each switch (SA–SJ) is **2‑** or **3‑position**; assign its SBUS channel and per‑position actions.
- A switch fires its actions when it **changes** to a position.

### Knobs, sliders & joystick axes

Analog sources — knobs **S1/S2**, sliders **LS/RS/MS**, gimbal axes **J1–J6** — continuously map their position to an output. Two functions:

- **Maestro Pass‑Through** — drive a servo: pick a Maestro (1–8), its servo channel (0–31), and the target positions (in quarter‑µs, e.g. 4000–8000) at the stick's min/max. One source can drive **up to 8** outputs at once.
- **HCR Volume** — map position to an HCR audio channel's volume (0–99).
- A **Reverse** toggle inverts the source around centre.

> ⚠️ Analog sources dispatch continuously. With many outputs active at the SBUS frame rate, you can generate a lot of traffic — keep active outputs reasonable.

### Per‑button matrix value

Each matrix button matches a PWM band on the matrix channel (default CH7), with a ±10‑count deadband around its center value. Edit a band in **Config → Channels → Buttons**, or via the button's hardware card — both edit the same table. **Calibration** (below) sets these for you.

---

## Calibration

**Config → 🎯 Calibrate RC** walks you through each control on your model:

1. The wizard highlights one control and shows the live SBUS value.
2. Move/press that control; it captures the value (with optional **auto‑advance**), or use **Force Capture**.
3. For switches/knobs you can also type the channel manually.
4. **Next/Previous** to step through; the step counter shows progress.

While the wizard is open, **all action dispatch is muted** (so nothing fires while you wiggle controls). Closing the wizard — or stopping the monitor — restores normal dispatch.

> Calibration requires a **direct USB** connection. Over Via WCB the channel data arrives at only ~5 Hz, too slow to reliably capture a quick stick/switch move, so the tool blocks calibration in Via WCB mode and asks you to connect over USB.

---

## Saving & loading

- **Refresh** re‑reads the config from the board (`GET_CONFIG`).
- **Save to Board** writes it (`SET_CONFIG`) and persists to NVS. The board replies with an ACK; the tool waits for it and warns if no ACK arrives (see [[Troubleshooting]]).
- Baud changes apply **live** on save — no reboot. **WCB network** changes need a **reboot** to take effect.
- **Restore Defaults** reloads factory defaults on the board.
- Config is also import/exportable for backup and sharing.

---

### Next: [[Actions Reference]] · [[WCB Network]] · [[Troubleshooting]]
