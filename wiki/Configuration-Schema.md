# Configuration Schema

This is the full structure of NaviCore's configuration object — the `data` payload of a `GET_CONFIG` reply and a `SET_CONFIG` command (see [[Serial JSON Protocol]]). It's also exactly what the config tool's **Export** produces and **Import** consumes. Field names and types here are taken straight from the firmware (`rc_config.h`).

> You normally never hand-edit this — the config tool builds it for you. This page is for **scripting, backups, and understanding an exported file**.

---

## Top-level object

```jsonc
{
  "txModel": 0,                 // 0 = Tandem X18, 1 = Twin X-Lite, 2 = Twin X20
  "threeAxisGimbals": false,    // X20 only: enables J5/J6 twist + stick-click slots 19/20
  "tapWindowMs": 500,           // multi-tap detection window (ms)
  "matrixChannel": 7,           // SBUS channel carrying the button matrix
  "matrixDebounceFrames": 1,    // 1-4; raise on a noisy analog matrix
  "funcBindings": { "mode": 4 },// which switch selects mode (index; 4 = SE by default)
  "thresholds": [ … ],          // matrix button PWM bands  (see below)
  "mappings":   { … },          // per-mode, per-button action lists
  "switches":   { … },          // SA-SJ channel + per-position actions
  "knobs":      { … },          // analog sources (knobs / sliders / gimbal axes)
  "hcrDest":    { … },          // global HCR destination
  "mp3Dest":    { … },          // global MP3 Trigger destination
  "maestros":   [ … ],          // 8 Maestro-location slots
  "auxBaud":    { … },          // S3 / S4 / local-Maestro baud
  "wcbNetwork": { … }           // ESP-NOW credentials
}
```

Any top-level key you **omit** in a `SET_CONFIG` is left unchanged on the board — so you can save just the branch you edited. (The config tool does exactly this: it diffs against the last load and ships only what changed.)

---

## `thresholds[]` — matrix button bands

One entry per matrix slot (21 total). A button "presses" when the matrix channel's value lands in `[minPwm, maxPwm]`.

```jsonc
{ "id": 1, "label": "B1", "minPwm": 1799, "maxPwm": 1823 }
```

| Field | Type | Meaning |
|-------|------|---------|
| `id` | int 1-21 | matrix slot number |
| `label` | string | display name ("B1", "T4 Left", …) |
| `minPwm` / `maxPwm` | int | inclusive SBUS-value band; `0/0` = unassigned/inert |

> **Calibration** writes these for your radio. Slots 19/20 are the X20 stick-clicks; 21 is spare.

---

## `mappings{}` — what each button does

Keyed by a **composite string** `"<mode*100 + slot>"`:

- `"107"` → **mode 1**, slot **7**
- `"312"` → **mode 3**, slot **12**

Only buttons that have actions (or `exclusive:true`) are serialized.

```jsonc
"107": {
  "exclusive": false,           // false = cumulative tiers, true = only the matched tier
  "t1": [ <action>, … ],        // single-tap actions
  "t2": [ <action>, … ],        // double-tap
  "t3": [ <action>, … ]         // triple-tap
}
```

Each tier holds up to **5 actions**. See **[Action objects](#action-objects)** below. Exclusive vs. cumulative is explained in [[Config Tool Guide]].

---

## `switches{}` — SA–SJ

Keyed by switch label. A switch fires its position's actions when it **changes** to that position.

```jsonc
"SA": {
  "channel": 8,                 // 0 = unassigned
  "positions": 3,               // 2 or 3
  "p0": [ <action>, … ],        // actions for position 0 (down)
  "p1": [ <action>, … ],        // position 1 (mid, 3-pos only)
  "p2": [ <action>, … ]         // position 2 (up)
}
```

Labels: `SA SB SC SD SE SF SG SH SI SJ`.

---

## `knobs{}` — analog sources

Keyed by label: `S1 S2 LS RS MS J1 J2 J3 J4 J5 J6`. Each continuously maps its position to one or more outputs.

```jsonc
"S1": {
  "channel": 5,                 // 0 = unassigned
  "function": 1,                // 0 = none, 1 = Maestro pass-through, 2 = HCR volume
  "reverse": false,             // invert around centre
  "outputs": [
    { "target": 3, "maestroCh": 5, "posMin": 4000, "posMax": 8000 }
  ]
}
```

**`outputs[]`** (up to 8 per source):

| Field | `function:1` (Maestro) | `function:2` (HCR volume) |
|-------|------------------------|----------------------------|
| `target` | Maestro slot **1-8** | HCR audio chan **0=V, 1=A, 2=B** |
| `maestroCh` | servo channel **0-31** | *unused* |
| `posMin` / `posMax` | quarter-µs at SBUS min/max (e.g. 4000–8000) | volume **0-99** at SBUS min/max |

---

## Action objects

Used inside `mappings` tiers (`t1/t2/t3`) and switch positions (`p0/p1/p2`). Every action may also carry `"delay": <ms>` (fired after the press) and `"note": "<label>"`.

| `type` | Fields | Notes |
|--------|--------|-------|
| `wcb_unicast` | `target` (WCB ID "1"–"20"), `cmd` | one board |
| `wcb_broadcast` | `cmd` | all boards |
| `maestro` | `target` (slot "1"–"8"), `cmd` | location set in `maestros[]`; `maestro_remote` also accepted on input |
| `maestro_local` | `cmd` | legacy/local-only form |
| `serial` | `port` ("S3"/"S4"), `cmd` | CR appended on send |
| `hcr` | `fn`, `chan`, `track` | destination is global (`hcrDest`) |
| `mp3` | `fn`, `track` | destination is global (`mp3Dest`) |

`cmd` formats and the `fn`/`chan`/`track` code tables are in [[Actions Reference]].

```jsonc
// examples
{ "type": "wcb_broadcast", "cmd": ":PP100", "note": "dome spin" }
{ "type": "maestro", "target": "2", "cmd": "setTarget,5,6000", "delay": 600 }
{ "type": "hcr", "fn": 2, "chan": 0, "track": 80 }   // SetEmotion(Happy, 80)
```

---

## Destinations & hardware

```jsonc
"hcrDest": { "transport": "serial", "port": "S3" }
// or:      { "transport": "wcb", "target": "5", "wcbPort": 2 }

"mp3Dest": { "transport": "serial", "port": "S4" }
// or:      { "transport": "wcb", "target": "2" }

"maestros": [                   // exactly 8 entries, slots 1-8
  { "type": 0, "device": 1 },   // type: 0 disabled / 1 local / 2 remote
  …                             // device: Pololu number 0-127 (255 = compact protocol)
],

"auxBaud": { "S3": 9600, "S4": 38400, "maestro": 115200 },

"wcbNetwork": {                 // see [[WCB Network]] — changing these needs a reboot
  "macOct2": 0, "macOct3": 0,
  "password": "yourpw",
  "quantity": 4,
  "deviceId": 20                // RC special-peer slot; leave at 20
}
```

> ⚠️ Over the **Via WCB** bridge the firmware ignores incoming `wcbNetwork.deviceId` / `macOct2` / `macOct3` / `password` / `quantity` — changing the transport you're riding on would cut you off. Edit those over **direct USB**. See [[WCB Network]].

---

### Next: [[Serial JSON Protocol]] · [[Actions Reference]]
