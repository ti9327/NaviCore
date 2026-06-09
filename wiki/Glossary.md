# Glossary

Quick definitions for the jargon scattered across this wiki.

| Term | What it means |
|------|---------------|
| **NaviCore** | This project — the WCB‑based RC controller firmware + browser config tool. (Formerly *RC‑Controller* / *HyperCore*.) |
| **WCB** | **Wireless Communication Board** — the ESP32‑based boards in the droid ecosystem that talk to each other over ESP‑NOW. NaviCore runs on WCB **v3.2** hardware and joins the WCB network as a peer. |
| **WCB v3.2** | The specific board revision NaviCore targets — an Espressif **ESP32‑S3** module on the WCB carrier. |
| **ESP32‑S3** | The microcontroller. Dual‑core, native USB, 3 hardware UARTs — relevant because NaviCore juggles SBUS in, SBUS out, a Maestro bus, and a USB console across them. |
| **SBUS** | The serial protocol FrSky (and others) use to send all channels on one wire: inverted UART, 100 000 baud, 8E2. NaviCore reads it as input. **SBUS‑16** = 25‑byte frames, **SBUS‑24** = 36‑byte frames (auto‑detected). |
| **Matrix channel** | One SBUS channel (default **CH7**) onto which the transmitter multiplexes *all* its push‑buttons as distinct PWM bands. NaviCore decodes which button is pressed from the value. Lets ~20 buttons share one channel. |
| **ESP‑NOW** | Espressif's connectionless peer‑to‑peer radio protocol (no Wi‑Fi router/AP). How NaviCore reaches remote WCBs and Maestros, and how **Via WCB** works. 250‑byte packet limit (hence fragmentation). |
| **ETM** | **Ensured Transmission Mode** — the WCB network's ACK'd/reliable unicast layer over ESP‑NOW. NaviCore's `wcb->send()` rides it. |
| **Via WCB** | Managing the controller *wirelessly* by relaying config‑tool traffic through a USB‑tethered WCB. See [[Remote Management over WCB]]. |
| **Special‑peer slot / Device ID 20** | The WCB network slot a non‑WCB device (like NaviCore) claims. By convention NaviCore is always **ID 20**, leaving 1–19 for real WCBs. |
| **Maestro** | A **Pololu Maestro** servo controller. NaviCore drives one locally (wired) and others remotely (over ESP‑NOW). See [[Maestro Setup]]. |
| **Pololu protocol** | The Maestro command framing that includes a **device number**, so multiple Maestros can share one bus and only the addressed one responds. NaviCore always uses it (never "compact"). |
| **qus** | **Quarter‑microseconds** — Maestro's servo position unit. Typical servo travel is ~4000–8000 qus (1000–2000 µs). |
| **Kyber / Kyber_Remote** | The WCB pass‑through path NaviCore broadcasts remote‑Maestro bytes on; a WCB with **Kyber_Remote** enabled forwards those bytes to its own Maestro port. |
| **HCR** | **Human‑Cyborg Relations** vocalizer — a droid sound/emotion board. NaviCore fires HCR commands over serial or WCB. See [[Actions Reference]]. |
| **MP3 Trigger** | A SparkFun **MP3 Trigger v2.x** sound board. Driven over serial or via a WCB's MP3 driver. |
| **NVS** | **Non‑Volatile Storage** — the ESP32 flash area where your saved configuration lives (survives reboots). *Update Firmware* preserves it; *Full Wipe* erases it. |
| **DTG** | **Date‑Time‑Group** — the timestamp stamped into the firmware version (`v0.2.0_011009QJUN26`) by the pre‑commit hook, so a binary's name matches the running version. |
| **FQBN** | **Fully Qualified Board Name** — the `arduino-cli` board+options string the build uses (`esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PartitionScheme=min_spiffs,PSRAM=opi`). `PSRAM=opi` is **required** — the runtime config lives in external PSRAM. |
| **USB CDC (on boot)** | Native‑USB serial mode on the ESP32‑S3. NaviCore is built with it **enabled**, which frees a hardware UART for SBUS‑out and is why `Serial` is the native USB port. |
| **Mode** | One of **three** complete mapping sets, chosen by a 3‑position switch (default **SE**). The same button does different things per mode — tripling your control surface. |
| **Tap window / multi‑tap** | The window (default **500 ms**) NaviCore waits to see if more taps are coming, so single / double / triple taps can each fire different actions. |
| **Exclusive vs cumulative** | Per‑button tap behavior — *exclusive* fires only the matched tap tier; *cumulative* fires every tier up to it. |
| **Fragmentation / reassembly** | Splitting an oversized JSON payload (a config) into ESP‑NOW‑sized chunks for Via‑WCB transfer, then rebuilding it on the far side. |
| **Failsafe / lostFrame** | SBUS link‑health flags. `failsafe` = link gone (NaviCore freezes outputs); `lostFrame` = one missed frame (ignored). See [[Failsafe and Signal Loss]]. |
| **Config tool** | The single‑file browser app (`config_tool/index.html`) that configures everything over the **Web Serial API** (Chrome/Edge). |

---

### Back to [[Home]]
