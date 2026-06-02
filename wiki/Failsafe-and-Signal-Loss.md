# Failsafe & Signal Loss

What NaviCore does when the radio link drops — important to understand for an animatronic, where unexpected motion on signal loss is the thing you most want to avoid.

---

## The short version

**On a failsafe frame, NaviCore freezes — it does not act.** Servos and other outputs **hold their last commanded state**; no buttons, switches, or knobs are dispatched. When the link recovers, normal operation resumes, and a button you were physically holding across the dropout won't auto‑fire — it needs a fresh press.

---

## Background: how SBUS signals a loss

Each SBUS frame carries two status flags:

- **`lostFrame`** — *this one frame* was missed/garbled. Transient; a single dropped frame is normal RF jitter.
- **`failsafe`** — the receiver has decided the **link is gone** and is now emitting *failsafe* frames, with the channels parked at whatever failsafe positions the receiver was configured for.

You can watch both live in the config tool monitor and via `#L09` (see [[CLI Commands]]).

---

## What NaviCore does

| Condition | Behavior |
|-----------|----------|
| Normal frames | Decode + dispatch as usual. |
| **`lostFrame` set** (a blip) | Telemetry notes it, but dispatch continues — gating on a single lost frame would make control feel laggy. |
| **`failsafe` set** (link gone) | **Dispatch is skipped entirely.** Mode decode, the button matrix, switches, and knobs are all bypassed. Telemetry keeps updating (so the tool shows the failsafe state), but **no actions fire and no servo targets change** — every output holds its last value. |

When `failsafe` clears (link restored), the matrix is re‑armed from a clean state: it requires a confirmed neutral and then a fresh press before it will fire again. So if you were holding a button when the transmitter died, NaviCore won't replay that press on recovery.

> Why hold instead of go‑to‑failsafe‑pose? Because the receiver's failsafe channel values would otherwise be decoded as real input — driving servos to the failsafe pose and tripping whatever switch/knob thresholds those parked values happen to cross. Freezing is the safe choice for a prop.

---

## What this means for your build

- **Set sensible failsafe on the receiver too.** NaviCore freezing its *outputs* is the firmware half; your **drive/throttle ESC or motor controller** (if any) has its own failsafe and should be configured to stop on signal loss. NaviCore can't stop a motor it isn't directly commanding.
- **Servos hold, they don't relax.** A held servo stays energized at its last target. If you need limbs to go limp on loss, that's a power/ESC decision, not something the "hold" behavior provides.
- **Brief RF blips are fine.** A single `lostFrame` doesn't interrupt anything; only a real `failsafe` state freezes dispatch.

---

## Verifying it

1. Connect the config tool and watch the monitor's SBUS health.
2. Power off the transmitter.
3. You should see the state flip to **FAILSAFE**, the servos **stay put** (no jump), and no actions fire.
4. Power the transmitter back on → state returns to receiving, and control resumes. A button held during the outage does **not** fire on recovery until released and pressed again.

If servos *do* jump to a pose or actions fire on power‑off, your receiver may be sending hold‑last‑position failsafe frames that don't set the failsafe bit — check the receiver's failsafe configuration so it flags failsafe (or stops output) on loss.

---

### Next: [[Troubleshooting]] · [[Config Tool Guide]]
