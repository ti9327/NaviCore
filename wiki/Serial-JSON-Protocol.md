# Serial JSON Protocol

NaviCore exposes a newline‑delimited **JSON protocol** over its USB serial port (115200 baud, native USB‑CDC). The config tool uses it, but it's plain text — you can script against it from anything that opens the port. Each command is one line of JSON terminated by `\n`.

> The same protocol is reachable over the WCB network via the **Via WCB** bridge (large payloads are fragmented). See [[WCB Network]].

---

## Command summary

| Send | Board replies | Purpose |
|------|---------------|---------|
| `PING` | `{"type":"PONG","version":"v0.2.0_…"}` | Liveness + firmware version. Also clears a stuck calibration mute. |
| `GET_CONFIG` | `{"type":"CONFIG","data":{…}}` | Full configuration as JSON. |
| `{"type":"SET_CONFIG","data":{…}}` | `{"type":"ACK","ok":true}` | Apply + persist config to NVS. Baud changes take effect live. |
| `{"type":"START_MONITOR"}` | `ACK`, then streams `PWM_UPDATE` | Begin live telemetry (~every 50 ms). |
| `{"type":"STOP_MONITOR"}` | `{"type":"ACK","ok":true}` | Stop telemetry (also clears calibration mute). |
| `{"type":"CALIB","on":true\|false}` | `{"type":"ACK","ok":true}` | Mute/unmute all action dispatch during calibration. |
| `{"type":"RESET_DEFAULTS"}` | `{"type":"ACK","ok":true}` | Reload factory defaults. |
| `{"type":"REBOOT"}` | `{"type":"ACK","ok":true,"msg":"rebooting"}` | ACKs, then restarts after 250 ms. |
| `{"type":"TRIGGER","mode":1,"btn":3,"tap":1}` | `{"type":"ACK","ok":true}` | Fire a virtual button press. |
| `{"type":"WCB_SEND","target":2,"cmd":":PP100"}` | `{"type":"ACK","ok":true}` | Send a WCB command (`target:0` = broadcast). |
| `{"type":"SET_DEBUG_FLAGS","flags":N}` | `{"type":"ACK","ok":true}` | Bitmask gating firmware `[DISPATCH]` logging. |
| `{"type":"GET_WCB_STATUS"}` | `{"type":"WCB_STATUS",…}` | Which WCB IDs are online. |

Parse/format errors come back as `{"type":"ERROR","msg":"…"}` (with `rxLen` on JSON parse failures, so you can spot a truncated line).

---

## PWM_UPDATE (telemetry stream)

While the monitor is active the board emits, ~20×/second:

```json
{
  "type":"PWM_UPDATE",
  "matrixCh":7, "modeCh":12,
  "matrixVal":1024, "modeVal":992,
  "btn":0, "mode":2,
  "sbus":{
    "ok":true, "fps":71, "frames":12345, "ageMs":3,
    "lost":false, "failsafe":false,
    "chCount":16, "frameLen":25,
    "channels":[ … per-channel SBUS values … ]
  }
}
```

- `matrixVal` / `modeVal` — raw SBUS values on the matrix and mode channels.
- `btn` — decoded matrix button (0 = none); `mode` — current mode (1‑3).
- `sbus.ok` — true only when frames are flowing, no lost frame, and data is fresh (<500 ms).
- `sbus.chCount` / `frameLen` — 16/25 = SBUS‑16, 24/36 = SBUS‑24.

---

## TRIGGER

```json
{"type":"TRIGGER","mode":1,"btn":3,"tap":1}
```

- `mode` 1–3, `btn` 1–21 (matrix slot), `tap` 1–3. Out‑of‑range values get a `{"ok":false}` ACK. Fires the mapped actions exactly as a physical press would.

## SET_DEBUG_FLAGS

A bitmask enabling per‑category `[DISPATCH]` log lines on the USB console (default 0 = silent):

| Bit | Category |
|-----|----------|
| `1<<0` | Maestro |
| `1<<1` | WCB (unicast + broadcast) |
| `1<<3` | HCR |
| `1<<4` | MP3 |
| `1<<5` | Serial |

## GET_WCB_STATUS

```json
{"type":"WCB_STATUS","quantity":4,"self":20,"online":[1,0,1,1]}
```

`online[i]` is 1 if board *i+1* is up. The local board (`self`) always reports online.

---

## Notes for scripting

- One JSON object per line, `\n`‑terminated.
- `SET_CONFIG` payloads can be several KB; the firmware sizes its RX buffer for this, but if you bridge over WCB the tool fragments them for you.
- Send a `PING` first — it also clears any stale calibration mute left by a crashed page.

---

### Next: [[CLI Commands]] · [[Troubleshooting]]
