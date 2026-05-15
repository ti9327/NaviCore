# RC Controller

[**🚀 Launch the config tool →**](https://greghulette.github.io/RC-Controller/)

WCB-based RC controller firmware for ESP32-S3 (WCB HW 3.2). Reads SBUS from an
RC receiver, dispatches matrix buttons / switches / knobs to local and remote
Maestros and WCB boards over ESP-NOW, and ships with a browser-based config
tool that talks to the board over Web Serial.

## Features

- SBUS input (16 or 24 channel, auto-detected) on Serial1
- SBUS output passthrough on a dedicated pin (~110 µs latency)
- Local Pololu Maestro on Serial2
- Up to 8 remote Maestros via WCBStream broadcast over ESP-NOW
- WCB unicast and broadcast command dispatch
- Multi-mode (3-position switch) button / switch / knob mapping, NVS-backed
- Multi-tap detection (single / double / triple) with exclusive or cumulative
  dispatch
- USB Serial JSON protocol for config tool and CLI debugging

## Layout

```
RC-Controller.ino    Main sketch
rc_config.h          Config struct, JSON serialisation, NVS persistence
wcb_config.h         WCB board ID / port definitions
sbus_reader.h        SBUS decoder + passthrough tee
config_tool/         Browser-based config GUI (Web Serial)
model/               FrSky X18 transmitter model file (download below)
```

## Required libraries

Install via the Arduino Library Manager:

- `ArduinoJson` (Benoit Blanchon) v6.x
- `Adafruit NeoPixel`

Local libraries (drop into your sketchbook `libraries/` folder):

- `WCBClient` + `WCBStream`
- `PololuMaestro`
- `hcr`

## Downloads

### FrSky X18 transmitter model

The RC Controller assumes a specific channel map (SBUS channels 1–15 for
sticks/switches, CH7 for the button matrix, etc.). To save you setting all
that up by hand on your X18, the matching transmitter model file is bundled
in the [`model/`](model) folder.

- [**Download:** r2 web gui.bin](<model/r2 web gui.bin>) — ~2.4 KB

Copy it to the `MODELS/` folder on the X18's SD card, then load it from the
transmitter's model select screen.

## Configuration tool

Open [`config_tool/index.html`](config_tool/index.html) directly in a
Chromium-based browser (Chrome, Edge). Click **Connect**, pick the board's
serial port, and you get live PWM monitoring, button mapping, switch / knob
config, calibration, and CSV import/export.

The tool uses the [Web Serial API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API)
— no install, no server, no Electron app.

## USB serial JSON protocol

Newline-delimited JSON on the USB Serial port:

| Command | Reply |
|---|---|
| `PING` | `{"type":"PONG"}` |
| `GET_CONFIG` | `{"type":"CONFIG","data":{...}}` |
| `{"type":"SET_CONFIG","data":{...}}` | `{"type":"ACK","ok":true}` |
| `{"type":"START_MONITOR"}` | streams `PWM_UPDATE` every 50 ms |
| `{"type":"STOP_MONITOR"}` | `{"type":"ACK","ok":true}` |
| `{"type":"RESET_DEFAULTS"}` | reloads factory defaults, replies ACK |
| `{"type":"TRIGGER","mode":1,"btn":3,"tap":1}` | fires virtual button press |
| `{"type":"WCB_SEND","target":2,"cmd":":PP100"}` | manually fires WCB command |

## License

MIT — see [LICENSE](LICENSE).
