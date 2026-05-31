# Firmware Binaries

This folder hosts the compiled NaviCore firmware for the **WCB v3.2**
hardware (ESP32-S3). The browser-based flasher in `config_tool/index.html`
(Config → Firmware tab) pulls the latest set from here on `main` via the
GitHub Contents API.

## Versioning

The firmware version is stored in `fw_version.h` at the repo root:

```c
#define FW_VERSION_BASE  "v0.2.0"         // bump by hand for releases
#define FW_VERSION_DTG   "211520QMAY26"   // auto-stamped by pre-commit hook
#define FW_VERSION       FW_VERSION_BASE "_" FW_VERSION_DTG
```

- **`FW_VERSION_BASE`** — semver version (MAJOR.MINOR.PATCH).  Edit by hand
  when cutting a new release: `v0.2.0` → `v0.2.1` → `v0.3.0` → `v1.0.0`.
  The build scripts treat the field as opaque — any quoted string works,
  so two-digit forms (`v0.1`) still parse if you need them.
- **`FW_VERSION_DTG`** — stamped automatically by `tools/git-hooks/pre-commit`
  on every commit, same DTG format as the UI footer in `config_tool/index.html`.
  Both files always reflect the same commit time.
- **`FW_VERSION`** — convenience macro the firmware uses to report itself
  in the `PONG` reply and at boot.

The build script reads both `#define`s and embeds them in the bin filename
(e.g. `NaviCore_v0.2.0_211520QMAY26_ESP32S3.bin`), so the file on disk,
the file's reported version at runtime, and the source header all match.

## File naming

Three files per release, all ending with these stable suffixes:

| File suffix             | Flash address | What it is                |
|-------------------------|---------------|---------------------------|
| `_ESP32S3.bin`          | `0x10000`     | Application image (`ota_0`)|
| `_ESP32S3_boot.bin`     | `0x0`         | Second-stage bootloader   |
| `_ESP32S3_part.bin`     | `0x8000`      | Partition table           |

The flasher matches by **suffix**, so the version prefix can change every
build without touching the page. Example set:

```
NaviCore_201500RMAY26_ESP32S3.bin
NaviCore_201500RMAY26_ESP32S3_boot.bin
NaviCore_201500RMAY26_ESP32S3_part.bin
```

The Config → Firmware tab offers two buttons:

- **⬆ Update Firmware** — routine update. NVS at `0x9000` is never touched,
  so the user's saved configuration survives.  The flasher auto-detects
  whether app-only is safe (matching partition table) or a full
  bootloader+partition+app write is needed (layout changed) — either way,
  NVS is preserved.
- **⚠ Full Wipe & Flash** — initial push / recovery.  Writes the full image
  AND erases NVS (`0x9000`, 20 KB) and OTA data (`0xE000`, 8 KB), returning
  the board to factory-fresh state.  Use this for first-time programming on
  a new board, or to recover from corrupted config / bad partition state.
  **Erases all saved settings.**

## How to update the binaries

**Default: GitHub Actions does it automatically.**  Every push to any
branch triggers `.github/workflows/build-firmware.yml`, which compiles
`NaviCore.ino` for ESP32-S3 with the `min_spiffs` partition scheme,
reads `FW_VERSION_BASE` + `FW_VERSION_DTG` out of `fw_version.h`, and
commits the three resulting bins back to `firmware/` (overwriting older
versioned files) under an auto-commit tagged `[skip ci]`.  Once that
commit lands on `main`, the Config → Firmware tab can flash any
connected board.

The two manual options below remain available for offline work or for
producing a one-off bin without going through CI.

### Option A — Arduino IDE (zero scripting)

1. Open `NaviCore.ino` in Arduino IDE.
2. Tools → Board → **ESP32S3 Dev Module** (or whatever you normally use for WCB v3.2).
3. Tools → Partition Scheme → **Minimal SPIFFS (1.9MB APP with OTA / 190KB SPIFFS)**.
4. Sketch → **Export Compiled Binary** (Ctrl/Cmd+Alt+S).
5. The IDE writes three files into `build/<fqbn>/`:
   - `NaviCore.ino.bin`
   - `NaviCore.ino.bootloader.bin`
   - `NaviCore.ino.partitions.bin`
6. Copy them into `firmware/`, renaming with a version prefix + the suffixes
   in the table above. Example:
   ```
   cp build/.../NaviCore.ino.bin             firmware/NaviCore_<DTG>_ESP32S3.bin
   cp build/.../NaviCore.ino.bootloader.bin  firmware/NaviCore_<DTG>_ESP32S3_boot.bin
   cp build/.../NaviCore.ino.partitions.bin  firmware/NaviCore_<DTG>_ESP32S3_part.bin
   ```
   (Old versioned files can be deleted or left as history — the flasher only
   looks at suffixes, so it always picks one of each.)
7. Commit to `main`. The Config → Firmware tab will pick them up automatically.

### Option B — `tools/build-firmware.ps1` (Windows) or `build-firmware.sh` (Linux/macOS/WSL)

Wraps `arduino-cli` so steps 2–6 above happen in one command.  Same
logic the CI workflow uses, just running locally.

```powershell
# Windows:
pwsh tools/build-firmware.ps1
```
```bash
# Linux / macOS / WSL:
tools/build-firmware.sh
```

Both read `FW_VERSION_BASE` + `FW_VERSION_DTG` from `fw_version.h`,
compile with the right FQBN + partition scheme, prune older bins, and
drop the new ones into `firmware/` with the matching version prefix.
You commit + push manually after a local build.

Prereqs: `arduino-cli` on `PATH`, the `esp32` core installed
(`arduino-cli core install esp32:esp32@3.3.4`), and the same library set
the CI workflow installs (see `.github/workflows/build-firmware.yml`).

## Notes

- Only **WCB v3.2** (ESP32-S3) is supported by the in-browser flasher right
  now. If support for older boards is added later, mirror the WCB Wizard's
  per-variant suffix scheme (`_ESP32.bin`, `_ESP32_boot.bin`, etc.) and add
  a board-variant dropdown to the Firmware tab.
- The flasher auto-detects whether the board is blank, has matching
  partitions, or needs a full flash. App-only is the default for any
  already-programmed board with a matching partition table.
- The page fetches from `main` by default. Developers can override the
  branch by setting `localStorage.rc_fw_branch = '<branch-name>'` in DevTools.
