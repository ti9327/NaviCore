# Flashing the Firmware

NaviCore targets the **WCB v3.2** board (ESP32‑S3). There are three ways to get firmware onto a board: the **in‑browser flasher** (easiest), the **Arduino IDE**, or the **build scripts / CI**.

---

## Option 1 — In‑browser flasher (recommended)

The config tool can flash a connected board directly over USB, pulling the latest compiled binaries from the repo's `firmware/` folder.

1. Open `config_tool/index.html` in **Chrome or Edge** (Web Serial is required — Firefox/Safari won't work).
2. Plug the board in over USB.
3. Go to **Config → Firmware**.
4. Choose one of:
   - **⬆ Update Firmware** — routine update. Writes the bootloader, partition table, and app, but **never touches NVS**, so your saved configuration is preserved.
   - **⚠ Full Wipe & Flash** — initial programming or recovery. Writes the same image **and erases NVS** (all saved settings) plus OTA data, returning the board to factory‑fresh. Use this the first time you program a new board, or to recover from a corrupted config.

Both modes write the same three regions (bootloader + partition table + app); the only difference is whether NVS/OTA data is also erased. The flasher **reuses the port you're already connected on** (no second port picker) and **reconnects to the board automatically** once it reboots — no need to re‑select the port afterward.

The page fetches binaries from `main` by default. (Developers can target another branch by setting `localStorage.rc_fw_branch = '<branch>'` in DevTools.)

> The flasher uses esptool over Web Serial and needs a **direct USB connection** — you cannot flash through a "Via WCB" bridge. (The flash buttons are disabled while Via WCB is active.)

---

## Option 2 — Arduino IDE

1. Open `NaviCore.ino` in the Arduino IDE.
2. **Tools → Board →** *ESP32S3 Dev Module* (or your usual WCB v3.2 board definition).
3. **Tools → Partition Scheme →** *Minimal SPIFFS (1.9 MB APP with OTA / 190 KB SPIFFS)*.
4. **Tools → USB CDC On Boot → Enabled** (so `Serial` is native USB‑CDC; the firmware relies on this for the SBUS‑out UART allocation).
5. **Sketch → Upload**, or **Export Compiled Binary** to produce the three `.bin` files.

### Required libraries

Install via **Library Manager** (versions pinned to match CI — ArduinoJson **must be v7**; v6's `DynamicJsonDocument` allocation behavior breaks `SET_CONFIG` parsing):
- **ArduinoJson** (Benoit Blanchon) — **7.4.3**
- **EspSoftwareSerial** — **8.1.0** (for S3/S4)
- **Adafruit NeoPixel** — **1.15.4**
- **PololuMaestro**

Local library (drop into your sketchbook `libraries/` folder):
- **WCB_Client** + **WCBStream** (from the `greghulette/WCBClient` repo)

> The exact library set + versions live in `.github/workflows/build-firmware.yml` — that workflow is the source of truth, so check there if a build ever disagrees with this list.

---

## Option 3 — Build scripts & CI

- **CI (default):** every push triggers `.github/workflows/build-firmware.yml`, which compiles `NaviCore.ino` for ESP32‑S3, reads the version from `fw_version.h`, and commits the three `.bin` files back into `firmware/` (tagged `[skip ci]`). Once that lands on `main`, the in‑browser flasher can use it.
- **Local:** run `tools/build-firmware.ps1` (Windows) or `tools/build-firmware.sh` (Linux/macOS/WSL). Both wrap `arduino-cli` with the correct FQBN + partition scheme and drop versioned bins into `firmware/`. Prereqs: `arduino-cli`, the `esp32` core, and the library set from the CI workflow.

---

## Firmware files & flash addresses

Three files per release, matched by **suffix** (the version prefix can change every build):

| File suffix | Flash address | Purpose |
|-------------|---------------|---------|
| `_ESP32S3.bin` | `0x10000` | Application image (`ota_0`) |
| `_ESP32S3_boot.bin` | `0x0` | Second‑stage bootloader |
| `_ESP32S3_part.bin` | `0x8000` | Partition table |

NVS (saved config) lives at `0x9000`; OTA data at `0xE000`. *Update Firmware* leaves both alone; *Full Wipe & Flash* erases them.

---

## Versioning

The version lives in `fw_version.h` at the repo root:

```c
#define FW_VERSION_BASE  "v0.2.0"         // bump by hand for releases
#define FW_VERSION_DTG   "011009QJUN26"   // auto-stamped by the pre-commit hook
#define FW_VERSION       FW_VERSION_BASE "_" FW_VERSION_DTG
```

- **`FW_VERSION_BASE`** — semver; edit by hand when cutting a release (`v0.2.0 → v0.2.1 → v0.3.0 → v1.0.0`).
- **`FW_VERSION_DTG`** — a Date‑Time‑Group stamped automatically by `tools/git-hooks/pre-commit` on every commit. Don't edit by hand.

The build embeds both into the `.bin` filename, and the firmware reports `FW_VERSION` in its boot banner and in the `PONG` reply — so the file on disk, the running version, and the source header always match. The config tool shows the connected board's version in its footer and on the Firmware tab.

---

### Next: [[Transmitter Setup]] · [[Config Tool Guide]]
