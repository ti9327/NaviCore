# CLI Commands

Besides the [[Serial JSON Protocol]], the firmware accepts a handful of `#Lxx` text commands on the USB serial port — type them in any serial terminal (115200 baud) for quick diagnostics. They're the fastest way to check SBUS and WCB health without the config tool.

Type the command and press Enter.

| Command | Does |
|---------|------|
| `#L01` | Print board identity (`NaviCore — WCB HW 3.2`). |
| `#L02` | Restart the board. |
| `#L09` | **SBUS state dump** — detected variant (16/24‑ch), frame count, fps, age, lost/failsafe flags, and the first 24 channel values. |
| `#L10` | Toggle **1 Hz live SBUS dump** on/off (repeats `#L09` once a second). |
| `#L11` | **WCB status** — this board's device ID, system quantity, and an up/down list of every WCB. |
| `#L12` | **RC state** — current mode, decoded matrix button, and raw matrix value. |
| `#L13` | **Raw SBUS frame hex dump** — every byte with header/flags/footer annotated (shows whether CH17–24 carry data on an SBUS‑24 link). |
| `#L20` | **HCR test on S3** (GPIO15) — sends `SetEmotion(Happy,80)` straight to the port, bypassing config + button mapping. |
| `#L21` | **HCR test on S4** (GPIO17) — same, on S4. |

---

## Using the HCR tests to isolate a fault

`#L20` / `#L21` drive the serial port directly — no config, no mapping in the way. So:

- **HCR reacts to `#L20` but not to a mapped button** → the problem is in **config or mapping** (HCR destination not saved, or the button isn't mapped to an HCR action), **not** the wiring.
- **`#L20` does nothing either** → it's **wiring** (TX/RX swapped, no common ground, 3V3 vs 5V level) or the aux‑serial port itself.

See [[Troubleshooting]] for more.

---

## Boot banner

At power‑up the board prints its version and a reminder of the available commands, e.g.:

```
=== NaviCore (WCB HW 3.2) ===
[SBUS] IN  on Serial1/UART1 RX (GPIO5)
[SBUS] OUT on Serial0/UART0 TX (GPIO9) — 100k 8E2 inverted, byte passthrough
[WCB] Joined network as device ID 20 (quantity=4)
[NaviCore] Firmware v0.2.0_… — setup complete.
```

If you see `[WCB] ERROR: wcb->begin() failed`, check your [[WCB Network]] settings.

---

### Next: [[Troubleshooting]]
