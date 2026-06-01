# Troubleshooting

Common problems and how to isolate them. For live diagnostics use the [[CLI Commands]] (especially `#L09`, `#L11`, `#L13`, `#L20`/`#L21`) and the config tool's monitor.

---

## The config tool won't connect

- **Use Chrome or Edge** on desktop. Firefox and Safari don't support the Web Serial API.
- Only one program can hold the serial port. Close the Arduino IDE Serial Monitor, other terminals, or another browser tab that's connected.
- Re‑plug the USB cable and click **Connect** again, then pick the port. A **charge‑only cable** won't enumerate a data port — use a data cable.
- After a manual reboot the native‑USB port re‑enumerates — just click **Connect** again. (After a *flash* the tool reconnects on its own; if that ever times out it tells you to click Connect.)

## No SBUS / "SBUS not OK" in the monitor

- Confirm the receiver's SBUS line is on **GPIO5**, with **common ground** and correct receiver power.
- Run `#L09` — if `frames` isn't climbing and `fps` is 0, no frames are arriving (wiring, wrong pin, or receiver not outputting SBUS).
- Make sure the radio is **bound** and actually transmitting, and that the model outputs SBUS (not just PWM/PPM).
- `failsafe: YES` or `lost: YES` means frames are arriving but the link is unhealthy — check RF/binding.
- Use `#L13` to see the raw frame; header byte should be `0F`. A 25‑byte frame = SBUS‑16, 36‑byte = SBUS‑24.

## Buttons don't fire (or fire twice)

- **Calibrate** (Config → 🎯) — the default matrix bands are X18 measurements; your radio's values may differ. Watch the matrix value in the monitor as you press each button and confirm it lands in a band.
- **Phantom double‑presses** on a noisy analog matrix: raise **matrix debounce frames** (Config → Channels) from 1 to 2–4. (1 is best for a clean digital SBUS source.)
- **A very fast tap** that presses+releases inside a single ~9–14 ms SBUS frame can't be seen — that's a hard SBUS limit, not a bug.
- Remember mappings are **per mode** — check you're editing the mode the mode‑switch is actually in (shown live in the monitor).

## Mode never changes / wrong mode

- The mode switch defaults to **SE (CH12)**, 3‑position. Verify that switch is on CH12 (or rebind it), and watch `modeVal` in the monitor: **<582** = Mode 1, **582–1401** = Mode 2, **>1401** = Mode 3.

## Save seems to fail / "no ACK received"

- A `SET_CONFIG` waits for the board's ACK. If you see a no‑ACK warning, click **Refresh** to verify what actually persisted, then re‑Save.
- Over **Via WCB**, large configs are fragmented and saving is slower (several seconds) — give it time before assuming failure.
- If saving over WCB is flaky, save over **direct USB** instead.

## HCR is silent

1. Run **`#L20`** (S3) or **`#L21`** (S4) — sends a test phrase straight to the port, bypassing config.
   - **HCR responds** → the wiring is fine; the issue is config/mapping. Check **Config → HCR** destination (transport + port/WCB ID) is saved, and that your button is actually mapped to an HCR action.
   - **No response** → wiring: confirm NaviCore **TX → HCR RX** (GPIO15 for S3, GPIO17 for S4), common **ground**, correct voltage, and the right **baud** (HCR = 9600) in Config → Aux Serial.
- For **HCR over WCB**: it's **unicast only** — the destination must be a WCB ID 1–20 plus a port 1–5 (broadcast isn't supported).

## MP3 Trigger doesn't play

- **Local (S3/S4):** set that port to **38400** baud (MP3 Trigger v2). Confirm TX→RX wiring and ground.
- **Over WCB:** the destination WCB must have its MP3 driver set up (`?MP3,S<port>` on that WCB), and you target that WCB's ID.
- `PLAY` track range is 1–255; `VOL` is 0 (loudest) … 64 (silent).

## Remote Maestro / servo doesn't move

- The Maestro **slot (1–8)** must be set to **Remote** in Config → Maestro, and the **device number** must match the unit's number in **Maestro Control Center**.
- The receiving WCB must have **Kyber_Remote** enabled so it forwards the broadcast bytes to its Maestro port.
- For a **local** Maestro: it's on Serial2 (GPIO6), and its baud must match **Config → Aux Serial → Maestro** (default 115200) — or set the Maestro to *Detect Baud Rate*.
- Each Maestro on a shared bus needs a **unique** Pololu device number, or they'll collide.

## WCB peers show offline

- Run `#L11` (or the tool's WCB Status panel). If peers are `dn`, check that **MAC octets, password, and quantity** match the rest of your system (`?WCBM` / `?WCBP` / `?WCBQ` on a known‑good WCB).
- **WCB network changes need a reboot** — save, then `REBOOT` (or power‑cycle).
- Orange status LED at boot = WCB init failed; re‑check the credentials.

## "Via WCB" can't flash firmware

- Correct — flashing needs a **direct USB** connection (esptool drives the chip's bootloader). Tether the board over USB to update firmware.

## Board seems frozen when nothing's connected

- It isn't — hot‑path logging is non‑blocking and is dropped when no host is draining USB. Connect the config tool (or a serial terminal) and it resumes normally. Saved config and dispatch keep running regardless.

## Recover a bad config / start fresh

- **Config → Restore Defaults**, or send `RESET_DEFAULTS`.
- For a deeper reset (corrupted NVS / partition trouble), use the flasher's **⚠ Full Wipe & Flash**, which erases NVS and rewrites everything. See [[Flashing the Firmware]].

---

Still stuck? Capture a `#L09` dump and the boot banner, and open an issue on the repository.
