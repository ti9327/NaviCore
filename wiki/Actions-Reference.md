# Actions Reference

An **action** is one thing NaviCore does when a button is tapped or a switch changes. Each button tap‑tier (single/double/triple) and each switch position holds up to **5 actions**, and every action can carry an optional **delay** (ms, fired after the press) and a short **note** (label shown in the tool).

There are six action types.

---

## WCB unicast

Send a WCB command string to **one** board.

- **Target:** WCB board ID **1–20**
- **Command:** any WCB command string, e.g. `:PP100`
- Chained commands are supported (e.g. `;h,play,a,1,fadein,4^;t6000^;h,fadeout,a,4`) up to ~95 characters.

## WCB broadcast

Same, but sent to **every** WCB on the network (no target).

---

## Maestro

Drive a Pololu Maestro servo controller. Maestros are addressed by **logical slot 1–8**; where each slot actually lives is set in **Config → Maestro (Locations)**:

- **Local** — wired to this board's Serial2 bus (GPIO6).
- **Remote** — broadcast over ESP‑NOW; any WCB with **Kyber_Remote** forwards the bytes to its Maestro port.
- **Disabled** — slot ignored.

Each slot also has a **Pololu device number (0–127)**, which **must match** the number set in Maestro Control Center on the physical unit. NaviCore always uses **Pololu protocol** (not compact), so multiple Maestros can share a bus and only the addressed device responds.

**Command forms:**

| Command | Meaning |
|---------|---------|
| `setTarget,<ch>,<pos>` | Move servo channel to position (quarter‑µs) |
| `setSpeed,<ch>,<spd>` | Set channel speed |
| `setAccel,<ch>,<acc>` | Set channel acceleration |
| `goHome` | Send all channels to their home positions |
| `stopScript` | Stop the running Maestro script |
| `restartScript,<n>` | Restart the Maestro script at subroutine *n* |

---

## HCR (Human‑Cyborg Relations vocalizer)

Fire an HCR vocalizer command. The **destination is global** — set once in **Config → HCR**, every HCR action shares it:

- **Serial** — local port **S3** or **S4** (9600 baud typical).
- **WCB** — unicast to a WCB ID **1–20**, plus the **serial port 1–5** on that receiving WCB. (HCR over WCB is unicast only — broadcast isn't supported.)

Each HCR action carries a **function**, an **emotion/audio channel**, and a **track/value**:

| fn | Function | chan | track/value |
|----|----------|------|-------------|
| 2 | SetEmotion | 0–3 emotion | level 0–99 |
| 3 | Trigger | 0–4 | level 0–99 |
| 4 | Stimulate | 0–4 | level 0–99 |
| 5 | Overload | — | — |
| 6 | Muse (one shot) | — | — |
| 7 | Muse gap (auto interval) | min gap 0–99 s | max gap 0–99 s |
| 8 | Stop (all audio + emote) | — | — |
| 9 | StopEmote | — | — |
| 10 | Override emotions | 0 = off / 1 = on | — |
| 11 | ResetEmotions | — | — |
| 13 | Auto-muse (continuous) | — | 0 = off / 1 = on |
| 14 | PlayWAV | 0–2 audio | track 0–9999 |
| 16 | StopWAV | 0–2 audio | — |
| 17 | SetVolume | 0–2 audio | 0–99 |

- **Emotion channels:** 0 = Happy, 1 = Sad, 2 = Mad, 3 = Scared, 4 = Overload.
- **Audio channels:** 0 = V (vocalizer), 1 = A, 2 = B.
- **Override emotions (fn 10):** locks the brain's emotion normalization so a `SetEmotion` level holds instead of decaying back to baseline. Turn it off to resume normal decay.
- **Muse (fn 6 / 7 / 13):** fn 6 plays a single muse now; fn 13 turns the brain's *continuous* random idle musing on/off; fn 7 sets the random gap (min–max seconds) between auto-muses.

> These match the Body Controller's HCR function numbers exactly. Over WCB, fn 7/10/13 are sent as the WCB's readable `;H,MUSE,GAP` / `;H,OVERRIDE` / `;H,MUSE` verbs (no WCB firmware change needed); the other functions use `;H,FN,…`.

> Diagnostic: CLI `#L20` / `#L21` send a test HCR command straight to S3 / S4, bypassing config and mapping — handy for isolating wiring vs. config issues. See [[CLI Commands]].

---

## MP3 Trigger (SparkFun v2.x)

Fire an MP3 Trigger. The **destination is global** — set once in **Config → MP3 Trigger**:

- **WCB** — unicast a `;A,…` command to a WCB ID **1–20** whose own MP3 driver does the serial work (configured there with `?MP3,S<port>`).
- **Serial** — local port **S3/S4**; NaviCore speaks the MP3 Trigger v2 protocol directly (set the port to 38400 baud).

| fn | Function | arg |
|----|----------|-----|
| PLAY | Play track | 1–255 |
| PLAYFS | Play by file index | 0–255 |
| STOP | Start/stop toggle | — |
| NEXT | Next track | — |
| PREV | Previous track | — |
| VOL | Set volume | 0 = loudest … 64 = inaudible |
| VOLUP | Volume up | — |
| VOLDN | Volume down | — |

---

## Serial

Write a raw command string to an aux serial port.

- **Port:** **S3** or **S4** (set its baud in **Config → Aux Serial**).
- **Command:** any string; the dispatcher appends a carriage return.
- *(The old "S5" target is gone — GPIO9 is now SBUS OUT.)*

---

## Delays & sequencing

Any action can carry a **delay** (ms). Delayed actions are queued (up to 8 pending) and fired when their timer elapses — so you can build short sequences within one button, e.g. *play a sound now, move a servo 600 ms later.*

---

### Next: [[WCB Network]] · [[Config Tool Guide]]
