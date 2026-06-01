# WCB Network

NaviCore joins your **WCB** system over **ESP‑NOW** (no Wi‑Fi AP, no router — peer‑to‑peer). This is how it reaches remote Maestros, remote WCB boards, HCR/MP3 over WCB, and how the config tool's **Via WCB** bridge works.

---

## Credentials

All four network values are stored in NVS and editable from **Config → WCB Network** in the tool. The compile‑time values in `wcb_config.h` are only the **factory defaults** for a fresh board.

| Setting | What it is | How to find it |
|---------|-----------|----------------|
| **MAC octet 2** | 2nd octet of the shared WCB MAC scheme | query a WCB: `?WCBM` |
| **MAC octet 3** | 3rd octet | `?WCBM` |
| **Password** | ESP‑NOW network password (≤39 chars) | `?WCBP` |
| **Quantity** | Total WCBs in the system | `?WCBQ` |
| **Device ID** | This controller's ID on the network | see below |

> Query any WCB over its own USB serial port with `?WCBM`, `?WCBP`, `?WCBQ` to read the values your system already uses, then enter the same ones here.

### Device ID

By convention an RC controller claims **device ID 20** — the WCB "special‑peer" slot. This keeps IDs 1–19 free for regular WCBs and gives the config tool's **Via WCB** bridge a single, known target (it always addresses the RC at ID 20). The field is editable for unusual multi‑controller setups, but leave it at 20 unless you have a reason not to.

> ⚠️ **WCB network changes require a reboot.** The WCB client initializes once at startup. Saving new credentials writes them to NVS, but the running radio keeps using the old values until the board restarts. The tool shows a "dirty / needs reboot" indicator to reflect this — use **REBOOT** (or power‑cycle) after changing them.

---

## Remote Maestros

Remote Maestro commands are **broadcast** on the Kyber path: NaviCore sends one packet, every WCB receives it, and any WCB configured with **Kyber_Remote** forwards the raw bytes to its local Maestro serial port. There's no per‑slot WCB/port routing to set up on the NaviCore side.

Because it's a shared broadcast bus, **every Maestro must have a unique Pololu device number** (set in Maestro Control Center, and matched in the Maestro Locations panel). The device‑number filter is what stops every Maestro from reacting to every command. See [[Actions Reference]].

---

## HCR & MP3 over WCB

- **HCR over WCB** is **unicast** to one WCB ID (1–20) plus a serial port (1–5) on that receiving WCB, which pipes the bytes to the HCR device.
- **MP3 over WCB** is **unicast** to one WCB ID; that WCB's own MP3 driver (set up there via `?MP3,S<port>`) does the serial work.

Set both destinations in their respective config tabs. See [[Actions Reference]].

---

## "Via WCB" remote management

NaviCore continuously broadcasts lightweight telemetry on the WCB network so the config tool can discover and monitor it without a USB cable:

- a periodic **heartbeat** (~0.5 Hz) and **channel snapshot** (~5 Hz),
- event‑driven **trigger** and **mode‑change** notices the instant they happen.

When you enable **Via WCB** in the tool (while tethered to *any* WCB), it relays `GET_CONFIG` / `SET_CONFIG` / `TRIGGER` / monitoring to the RC at device ID 20. Big config payloads are fragmented to fit ESP‑NOW packets. Firmware flashing still requires direct USB.

---

## Status & liveness

- The tool's **WCB Status** panel polls which boards are online (`GET_WCB_STATUS`).
- CLI `#L11` prints the same up/down list over USB serial. See [[CLI Commands]].
- A failed network init at boot lights the status LED **orange**.

---

### Next: [[Serial JSON Protocol]] · [[Troubleshooting]]
