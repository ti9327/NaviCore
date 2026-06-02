# Remote Management over WCB

You don't have to tether a USB cable to the NaviCore board to manage it. If the controller is buried inside a droid, you can reach it **wirelessly** from the config tool by plugging into *any* WCB on the same ESP‑NOW network and letting that WCB relay traffic to the controller. This is **"Via WCB"** mode.

This page is the end‑to‑end walkthrough. For the credential details see [[WCB Network]]; for the connection basics see [[Config Tool Guide]].

---

## How it works (the short version)

```
[Browser config tool] --USB--> [any WCB] --ESP-NOW--> [NaviCore @ device ID 20]
```

- The config tool wraps each JSON command as `;w20,<json>` and the tethered WCB forwards it over ESP‑NOW to the controller, which always lives at **special‑peer device ID 20**.
- The controller continuously broadcasts lightweight telemetry back (heartbeat, channel snapshot, trigger/mode events) so the tool can discover and live‑monitor it.
- Large payloads (a full `GET_CONFIG` / `SET_CONFIG`) are **fragmented** into ESP‑NOW‑sized packets and reassembled on the other end.

---

## One‑time setup

1. **Put the controller on the WCB network.** In [[Config Tool Guide]] over **direct USB**, open **Config → WCB Network** and set the MAC octets, password, and quantity to match your system (read them off a known‑good WCB with `?WCBM` / `?WCBP` / `?WCBQ`). Leave **Device ID = 20**. Save and **reboot** the controller (WCB credentials only take effect at boot).
2. **Confirm it's online.** From any WCB's USB serial, or the config tool's WCB Status panel, the controller should show up as board **20** online.

---

## Connecting Via WCB

1. Plug the config tool into **any** WCB on the network over USB and **Connect** (normal direct connection to *that WCB*).
2. Tick the **Via WCB** checkbox in the top bar.
3. The tool now relays to the controller at ID 20. You'll see the SBUS monitor, mode, and live channels populate from the controller's telemetry — the same UI as a direct connection.

The tool sends a quiet keep‑alive ping every ~10 s so the controller keeps streaming channel data while you're parked on the page.

---

## What works Via WCB

| Task | Via WCB? | Notes |
|------|:--------:|-------|
| Live SBUS monitor (channels, fps, mode) | ✅ | fps is the controller's **real** SBUS rate (from its heartbeat), not the 5 Hz relay rate |
| See button/switch/knob activity | ✅ | event‑driven trigger/mode notices |
| **Trigger** a button remotely | ✅ | fires the mapped actions exactly like a physical press |
| **Load** config (`GET_CONFIG`) | ✅ | fragmented + reassembled |
| **Save** config (`SET_CONFIG`) | ✅ | the tool sends only the **changed** branches (a diff), so a typical save is a handful of fragments, not the whole config |
| **Calibration** | ❌ | needs ~30 Hz sampling; the 5 Hz relay is too slow — use direct USB |
| **Firmware flashing** | ❌ | esptool needs a direct USB bootloader connection; the flash buttons are disabled in Via WCB mode |

---

## Saving config Via WCB

- Saves are **diff‑based**: the tool ships only the top‑level branches you changed since the last load, which keeps the payload small and reliable over the radio.
- A save isn't considered done until the controller returns an `ACK`. If no ACK arrives within ~12 s, the tool warns you — click **Refresh** to see what actually persisted, then re‑save.
- **Network‑identity fields are protected.** The controller refuses to change its own `wcbNetwork` (device ID, MAC octets, password, quantity) from a Via‑WCB save — otherwise a save could cut the very link it arrived on. Change those over direct USB.

> If a save over WCB ever feels flaky (RF congestion, a busy network), just tether USB for that one save — it's always the most reliable path.

---

## Limits & expectations

- **One controller per network at ID 20.** The bridge addresses a single special‑peer slot. Multiple controllers would need distinct IDs and an explicit picker (not the default setup).
- **Telemetry is gated.** The controller only streams 5 Hz channel data while a tool is actively listening (it sees your pings); on an idle network it just sends a 0.5 Hz heartbeat, so it isn't spamming the ESP‑NOW airwaves.
- **Bridges are interchangeable.** The WCB you tether to is just a relay — its own ID doesn't matter, it only needs to be on the same network with the special peer enabled.

---

## Troubleshooting Via WCB

- **Controller not found / no telemetry:** confirm it booted on the network (board 20 online), and that MAC octets/password/quantity match. A failed network init shows an **orange** status LED. See [[WCB Network]].
- **Save says "no ACK":** RF congestion or the controller briefly offline — Refresh and retry, or save over USB.
- **Calibrate / flash greyed out or blocked:** expected — both require direct USB.

See [[Troubleshooting]] for the full list.

---

### Next: [[WCB Network]] · [[Config Tool Guide]] · [[Serial JSON Protocol]]
