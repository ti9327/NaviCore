# NaviCore Record / Replay — Design Spec (v2, validated)

Status: **DESIGN — validated against code, ready for phase-1 build.**
Last updated 2026-06-30. v2 folds in a 22-agent validation pass (see §12 changelog).

## Goal

Capture everything the droid *does* (Maestro moves, HCR, WCB commands, MP3, aux serial) over a
bounded window, save it as a named **clip**, replay it on demand, and build a renamable **library**
of clips. Phase 2 adds a **timeline editor** in the config tool. The phase-1 data model is designed
so phase 2 is purely additive.

## Decisions locked

1. **Servo anti-snap:** snapshot last-commanded positions at clip start (`servoHome`) → ease-in.
2. **Storage:** a **dedicated `clips` partition** carved from the 16 MB flash (OTA + config untouched).
3. **Motion smoothness:** prioritized — **dense keyframe capture (no thinning) + replay interpolation**.
4. **Trigger:** **button-first**. `toggle` is the universal button mode; `level`/`hold` need a switch.

---

## 1. Event model — a clip event is a timestamped `RcAction`

Every button/switch maps to an **`RcAction`** (`{type, target[6], cmd[96], fn, chan, track, note}`,
rc_config.h) that the config tool already renders (action picker) and serializes
(`actionToJson`/`actionFromJson`). So **clip event = `{ tMs, RcAction }`**, and the phase-2 editor is
that picker on a timeline.

### Capture taps (verified) — both route through ONE queue

| Tap | Captures | Core |
|---|---|---|
| `rcExecuteActionNow()` (NaviCore.ino:926) | Every discrete action (WCB uni/broadcast, Maestro, Serial, HCR, MP3) | **Core 1 AND Core 0** ⚠ |
| `processKnobs()` sinks — `maestroSetTarget` (:1196), `dispatchHcrVolume` (:1199) | Continuous gimbal/knob streams (synthesized as `setTarget`/`SetVolume` events) | Core 1 |

⚠ **`rcExecuteActionNow` runs on Core 0** for a remote ESP-NOW TRIGGER (`onWCBCommand` →
`rcTelemetry::handle` → `rcDispatch` → `rcExecuteActionNow`, rc_telemetry.h:863-871, no defer to
loop). It also runs on Core 1 (local press, `checkPendingActions`, Web-Serial TRIGGER). This dual-core
reachability **dictates the recorder's concurrency design** (§6) — it is the exact class of bug that
crashed the board during the OTA work.

---

## 2. Clip structure — baseline header + event list

The header snapshots the **global state** the events silently depend on.

| Header field | Why |
|---|---|
| `version, name[32], durationMs, createdDtg` | Metadata (tool stamps real date on save). |
| `mode` (1–3) | Context. |
| `boardType` + `auxBaud[3]` | A clip replayed on the *other* board profile (v2 PCB vs WCB 3.2) would mis-route / mis-speed local serial; record so the tool can warn on mismatch. |
| `hcrDest` {transport, target[6], wcbPort} | HCR actions carry only `fn/chan/track`; destination is **global**. |
| `mp3Dest` {transport, target[6]} | Same — MP3 destination is global. |
| `maestros[8]` {type, device} | Maestro actions carry a **logical slot id**; wiring + Pololu device # live here. |
| `hcrLocalVol[3]`, `mp3LocalVolume` | Volume **shadows** — restore before replay so relative *volume* ops replay deterministically. |
| `servoHome[]` {slot, ch, pos} | Last-commanded position per **slot+ch** (see §6) at clip start → ease-in. Excludes channels never driven or invalidated by `goHome` (sentinel `0xFFFF`). **Key strictly by slot id (1–8), never device#** — two slots may legally share a device# on different buses (NaviCore.ino:432-434), so a device#-keyed shadow would alias two servos. |

### Event records (tagged, timestamped)

| Tag | Record | ~size |
|---|---|---|
| `0x01` Action | `tMs, type, fn, chan, track, targetLen, cmdLen, target[], cmd[]` | 12–40 B |
| `0x02` Maestro keyframe | `tMs, slot, ch, pos(qus)` | ~8 B |
| `0x03` HCR-volume keyframe | `tMs, chan, vol` | ~7 B |

(`tMs` may be a `u16` delta on dense streams — encoding detail; clips ≤30 s.)

---

## 3. Continuous capture & smooth replay

- **Capture: dense, no thinning** — keep every change-gated keyframe (knob deadband 5 / volume 80 ms),
  so replay is at least as smooth as the live move.
- **Replay: interpolated** — the player holds each active `(slot,ch)` curve and emits the
  linearly-interpolated qus position **every loop tick**, decoupling smoothness from loop jitter.
- **Replay preconditions (mandatory) — reset device/cache state so interpolation is authoritative:**
  1. **Force Maestro `speed=0, accel=0`** (unlimited) on every `(slot,ch)` the clip drives before
     easing to the first `servoHome`. Latched Pololu speed/accel limits live *device-side* and would
     **double-smooth** the interpolated stream (distorted, non-deterministic). Read-back is impossible
     on the one-way bus, so we force-reset (don't snapshot). *(setSpeed/setAccel are excluded from
     capture — see §5.)*
  2. **Do NOT emit `0x03` HCR-volume keyframes through `dispatchHcrVolume()`** — its static
     `hcrVolCache` (NaviCore.ino:1128: value-dedupe + 80 ms/channel gate) is a live singleton that
     would drop the leading keyframe and decimate the stream. Emit them via a **raw** HCR path (build
     `RA_HCR fn=17` → `executeHcrAction()` directly), bypassing the cache. (`maestroSetTarget` needs no
     such treatment — it's a cacheless raw write.)
  3. **`goHome` is a curve discontinuity** (see §6) — on replaying a captured `goHome`, clear the
     affected slot's active interpolation curves so the next keyframe re-anchors from home.
- **Position is absolute qus**; the editor changes throw by scaling qus around a pivot.
- **Remote (broadcast) Maestro** smoothness is bounded by `WCBStream`'s 2 ms-gap flush — same as live.

---

## 4. Fidelity rules

- **Replay re-dispatches through the same path** (`rcExecuteActionNow` → `executeHcrAction`/…), so the
  emotion-`chan=4`→Overload rewrite, numeric-vs-readable-verb WCB encoding, and absolute-volume
  derivation re-apply for free. Store the raw `RcAction`; the dispatcher does the rest.
- **Relative *volume* ops are deterministic** (HCR VolumeUp/Down, MP3 Vol±) because the header restores
  the volume shadows.
- **MP3 `NEXT`/`PREV` are NOT deterministic on replay** — they advance the module relative to its live
  track pointer, which NaviCore doesn't shadow and the module won't read back. *Phase-1 policy:* mark
  them best-effort and recommend absolute `Play track` in recorded routines. *(Optional later: capture
  a `lastMp3Track` shadow and rewrite NEXT/PREV→absolute PLAY at capture — with residual drift if the
  module auto-advances on track end.)*
- **Timestamp encodes per-action delay** (we tap post-delay) → stored events use `delayMs=0`; replay
  schedules purely by `tMs`. **Order preserved** (cumulative tap tiers in true fire order).

## 5. Exclusions (never captured)

Telemetry (`rc_hb/rc_ch/rc_trig/rc_mode`), bridge housekeeping (`?WHOAMI`/PONG/STATUS), system
(reboot, config, calibration, debug flags, monitor toggles, OTA), the **Record/Play meta-functions**,
and **Maestro `setSpeed`/`setAccel`** (the player owns motion shaping via interpolation; re-dispatching
a latched limit would fight it). Recording auto-suppresses during calibration.

---

## 6. Recorder mechanics — concurrency is the critical part

**Single producer-safe queue, single PSRAM writer.** Because `rcExecuteActionNow` runs on both cores
(§1), the recorder MUST mirror the proven OTA / remote-CLI pattern:

- **Both taps enqueue** a fixed POD event (`{tMs, RcAction}`, ~144 B) into ONE FreeRTOS queue via a
  `static __attribute__((noinline))` helper doing only `xQueueSend(q, &ev, 0)` (non-blocking,
  drop-if-full). **No alloc, no flash, no mutex, no Serial, no large stack frame** in the tap — the tap
  sits 4 frames deep on the Core-0 ESP-NOW stack on top of the ArduinoJson parse; the `noinline` + small
  POD is exactly the `queueRemoteCli` fix that resolved the prior WiFi-task stack overflow
  (NaviCore.ino:1338-1347). Drop-if-full is acceptable (a dropped keyframe thins one frame).
- **`drainRecorder()` in `loop()` (Core 1)** — placed by `drainOtaPackets()`/`drainRemoteCli()`
  (NaviCore.ino:2246/2252) — is the **sole** writer/serializer of the PSRAM grow buffer (all
  realloc/append on Core 1 only; no mutex on the buffer needed).
- **Create the queue in `setup()` BEFORE `wcb->begin()`** (cf. `remoteCliQueue`) or the first remote
  TRIGGER hits a null queue.
- **Capture-enable flag is `volatile`/atomic** (read once at top of tap), gated off during replay so
  re-dispatched events aren't re-recorded.
- **Live buffer:** PSRAM grow buffer (8 MB — trivial), bounded to **30 s** + a hard byte cap. Flash
  touched only on save.

**`lastPos` shadow (for `servoHome`):** a `volatile uint16_t lastPos[8][32]`, init to `0xFFFF`
(never-commanded sentinel). `maestroSetTarget(slot,ch,pos)` writes `lastPos[slot-1][ch] = pos` — a
single naturally-aligned 16-bit store, **lock-free and torn-write-free on Xtensa** even across cores.
**Do NOT add any bitmap/counter/dirty-list RMW** in that tap (that would be the real race). At
clip-start the recorder snapshots `lastPos` on Core 1 (skipping `0xFFFF`).

**`goHome` invalidates the shadow + curve:** `goHome`/`stopScript`/`restartScript` bypass `setTarget`,
so hook `maestroGoHome(slot)` to set `lastPos[slot-1][*] = 0xFFFF` for all channels (home pose unknown
→ omit from `servoHome`); the player clears that slot's active curves when it re-emits `goHome`.

**Save / replay:** stop → serialize header+events to `clip<N>.ncr` in the clips FS. Replay → restore
shadows → reset speed/accel → ease to `servoHome` → schedule events by `tMs`, interpolating curves;
**suspend live dispatch** and gate capture off.

---

## 7. Trigger model — configurable, button-first

Two new mappable functions (`RA_RECORD` / `RA_PLAY`, excluded from capture), assignable via the
existing mapping UI. Their toggle/active-recording state is new module-level state on the Core-0/Core-1
boundary → keep it `volatile`/atomic.

- **Record**, param `mode`:
  - `toggle` *(default; the only matrix-button-safe mode)* — activate → start; again → stop+save.
  - `level` / `hold` — **require a switch** (or a momentary switch wired as a 2-position channel):
    `processSwitches` surfaces both edges as position changes (NaviCore.ino:1097); **matrix buttons
    deliver only a debounced tap-count, never a release/duration**, so `hold` is infeasible there.
  - param `slot`/`name` — where the capture saves.
- **Play**, params: `clip`, `loop`, `stopOthers` — fires on a button press or switch position.

Handheld (button-only) path: **Record(toggle) + Play(button)** — fully supported. *(A future handheld
on its own protocol could enable `hold` only if it reports button release; the SBUS matrix does not.)*

---

## 8. Storage — dedicated `clips` partition

The shipped table is **stock `min_spiffs`, topping out at 0x400000 (4 MB)** even though the module is
16 MB — leaving **12 MB free** above. Decoded current rows (keep byte-identical) + one appended `clips`
row:

```
# name,    type, subtype,  offset,    size
nvs,       data, nvs,      0x9000,    0x5000
otadata,   data, ota,      0xE000,    0x2000
app0,      app,  ota_0,    0x10000,   0x1E0000
app1,      app,  ota_1,    0x1F0000,  0x1E0000
spiffs,    data, spiffs,   0x3D0000,  0x20000     # config.json — MUST stay byte-identical
coredump,  data, coredump, 0x3F0000,  0x10000
clips,     data, spiffs,   0x400000,  0xC00000    # NEW — 12 MB, fills the 16 MB flash
```

- **Config FS survives only if the `spiffs` row is unmoved.** `rc_config.h` mounts
  `LittleFS.begin(true)` (format-on-fail) bound to the default `spiffs` label — shift/resize that row
  and `config.json` is **silently reformatted**.
- **Mount `clips` as a SECOND LittleFS by label + distinct base path:**
  `LittleFS.begin(true, "/clips", 10, "clips");`. Format is scoped to the `clips` label, so first-boot
  format can never touch config.
- **Build wiring (these files change together — `.github/workflows` is NOT touched, only `tools/`):**
  - Add `NaviCore/partitions.csv` (the 7 rows above).
  - `tools/build-firmware.ps1` **and** `tools/build-firmware.sh`: FQBN `PartitionScheme=custom` +
    `FlashSize=16M`. **`build-firmware.sh` must also copy the custom 16 MB
    `WCB_S3_custom_bootloader_16MB_wdt3s.bin`** (it currently copies arduino-cli's 4 MB stock — the
    `.ps1` already copies the custom one). Otherwise CI republishes a stock table+bootloader and
    **reverts** the change (the workflow auto-commits `firmware/*.bin`).
  - Update doc/comment mentions of `min_spiffs` in `firmware/README.md`, `build-firmware.ps1`, and the
    `flasher.js` layout comments (lines ~91-95 / ~279-284). `flasher.js` *logic* needs no change — it
    writes `part.bin@0x8000` unconditionally and never keys off the config/clips offsets; the otadata
    (0xE000) / NVS (0x9000) erase set is fixed and must **never** be broadened to a full-chip erase.
- **Go/no-go gate:** after building, decode the emitted `_part.bin` and assert rows 0–5 are byte-identical
  to the current shipped `_part.bin` and row 6 is `clips@0x400000`. The 16 MB bootloader is already
  confirmed (`byte[3]=0x4f`; flasher uses `flashSize:'keep'`).
- **Migration:** the `clips` partition arrives via a **full flash** of the locally-built bins (the
  in-browser flasher writes boot+part+app). App-only **OTA** doesn't touch the table, so OTA keeps
  working after migration.

---

## 9. Config-tool integration

A **Clips** panel: list / rename / delete / play, transferred via a small **clip↔JSON bridge** (reuse
`actionToJson` per event) over the existing WCB-bridge/USB JSON transport. Phase 2 (**built**, see §13):
a 📈 button on each clip row opens a **timeline editor** — servo/HCR-volume curves as draggable keyframes,
an easing tool, and an Actions row for adding/editing/deleting discrete commands (WCB/HCR/MP3/serial/
Maestro/record/play/stop) — writing the clip back via a new `EDITBEGIN/EDITEV/EDITEND` upload transport.

## 10. Phase plan

- **Phase 1 (built):** `partitions.csv` + build wiring + clips FS mount; recorder (queue tap → Core-1 drain →
  PSRAM buffer, `lastPos` shadow, `goHome` invalidation, save); player (restore shadows + reset
  speed/accel + ease-in + interpolated replay, HCR-volume cache bypass); `RA_RECORD`/`RA_PLAY`
  functions; config-tool Clips panel.
- **Phase 2 (built, see §13):** timeline editor — download/edit/upload transport + SVG editor UI.

## 11. Resolved risks (from validation) + residual

- ✅ Core-0 capture safety → single non-blocking queue, `noinline` small-POD tap, Core-1 sole writer.
- ✅ Partition brick/wipe → keep `spiffs` byte-identical, 2nd labeled mount, CI scripts emit custom table.
- ✅ Maestro speed/accel double-smoothing → reset on replay + exclude from capture.
- ✅ HCR-volume cache decimation → bypass `dispatchHcrVolume` on replay.
- ✅ `goHome` shadow/curve desync → invalidate shadow + clear curve.
- ✅ `hold` infeasible on matrix buttons → `toggle` for buttons, `hold`/`level` for switches.
- ⚠ Residual (documented, low-impact): MP3 NEXT/PREV non-deterministic; slot/device aliasing affects
  only ease-in pre-roll (config already illegal); cross-board replay mismatch (header warns).

## 13. Phase 2 — timeline editor (built 2026-07-01, config-tool-verified; NOT hardware-tested)

Opened via a new **📈** button on each Clips-panel row (`openTimelineEditor(name)`). Ground truth stays the
on-device `.ncr` file — the editor downloads a clip, edits it client-side, and uploads the result back;
`navicore_record.h` gained no new persistence format, it reuses `_buf`/`_count`/`saveClip()` exactly like
LOAD/SAVE already do.

**Wire protocol — two directions, two different integrity strategies (both `?REC,*` CLI sub-verbs):**

- **Download** (`EDITLOAD,<name>`, read-only against `_buf`): `loadClip()`s, then streams every event as one
  small JSON line — `[CLIPDL:BEGIN]{count,durationMs,mode}` → `[CLIPDL:EV]{t,k,...}` × N → `[CLIPDL:END]`.
  Same "small self-contained line" shape as the [CLIPLIST]/[CLIPITEM] fix (§ earlier bugfix) so a line can't
  be hard-wrapped mid-JSON by the WCB RTERM sink's 160-byte packet limit (a maximal action event — a 95-char
  WCB `cmd` — CAN exceed 160 B, so the config tool applies the SAME per-source fragment-reassembly it already
  uses for the old single [CLIPLIST] marker). No per-line ACK: a lost/incomplete download is read-only (no
  device state mutated) and trivially retried, so the config tool just checks the BEGIN-declared `count`
  against events actually received and shows a clear error + retry prompt on mismatch — full OTA-grade
  ACKing would be overkill for something this cheap to just redo.
- **Upload** (`EDITBEGIN` → `EDITEV,<json>` ×N → `EDITEND,<name>`): this WRITES, so a lost event would
  silently corrupt the saved clip — needs real integrity. Per-event ACK/NAK with retry, mirroring naviota's
  OTA DATA/ACK contract: `[CLIPUL:BEGIN,OK|ERR]`, `[CLIPUL:ACK,<count>]`/`[CLIPUL:NAK,<reason>]` per event
  (3 attempts, then abort), `[CLIPUL:END,OK|ERR,<reason>]`. A NEW firmware state, `ST_EDITING`, brackets the
  whole upload — every existing entry point (`startRecord`/`startReplay`/etc.) already gates on
  `_state==ST_IDLE`, so adding one more state value automatically fences a concurrent live Record/Play
  trigger out of stomping `_buf`/`_count` mid-upload, with zero other call sites needing to change. On
  finalize, `editEnd()` `qsort()`s `_buf` by `tMs` before saving — drag/insert/delete edits can leave events
  out of chronological order, which `clipDurationMs()`, the discrete-event replay cursor, AND the phase-1b
  curve-chain builder all assume is monotonic.

**Editor data model (client-side, `_tlClip`, config_tool/index.html):** flat downloaded events reshape into
`maestro["<slot>-<ch>"] = [{t,pos}]` / `hcr["<chan>"] = [{t,vol}]` (both sorted, keyframe arrays) + a flat
`actions[]` list. `action` objects are the EXACT SAME shape `actionToJson`/`actionFromJson` (firmware) and
`appendActionFields`/`readActionFromFid` (config tool) already use everywhere else — the whole point (per
§1's original "no-throwaway" principle) is that a timeline action is edited by the SAME code as a button
action, just at an arbitrary timestamp instead of a tap tier. `readActionFromUI(tier,ai)` was refactored
into a thin wrapper over a new `readActionFromFid(fid,type,extras)` so the button/switch/knob editors and
the timeline's action popover share one field-reading implementation instead of drifting apart.

**SVG timeline UI:** one row per Maestro `(slot,ch)` / HCR channel actually present in the clip (+ "add
track" buttons for a channel the clip doesn't touch yet), each a draggable-keyframe polyline; a shared
"Actions" row of diamond markers. Maestro positions are STORED in ¼µs (matches the wire `pos` field and the
firmware directly — zero conversion on save) but DISPLAYED/EDITED in µs, same UI convention as the
knob-passthrough Pos min/max fields added earlier this session, converted only at the render/drag boundary.
Click empty track space → add a keyframe/action at that time+value; click a point → select (Delete key
removes it); drag → move it (re-sorted on release); Shift-click a second ADJACENT keyframe on the same
track → arms an **easing** toolbar (linear/easeIn/easeOut/easeInOut + step count) that INSERTS synthesized
intermediate keyframes between the pair — the firmware only ever interpolates LINEARLY between whatever
keyframes exist (phase 1b), so this is the entire mechanism behind "easing": denser keyframes approximating
a curve, not a new firmware curve type. Re-applying easing to the same pair REPLACES the previously
synthesized points (filters anything strictly between the two anchors) rather than stacking more on top. A
draggable scrub cursor shows a live readout that mirrors the firmware's inter-keyframe linear interpolation
(`_tlInterpolateAt`, deliberately parallel to `navicore_record.h`'s `_updateCurves`) plus any actions within
±50 ms, so the user can preview values without hardware attached.

**Verified (browser, no hardware):** full download→model→render pipeline (maestro + HCR + action events);
fragment reassembly for a 191-byte action event split at the exact 160-byte RTERM boundary (reassembles to
the original, unfragmented over Direct USB); add/select/delete keyframes and tracks; easing insert AND
replace-not-stack re-apply; the action-editor popover (open/type-switch/apply/delete) round-tripping through
`readActionFromFid`; the FULL upload sequence (BEGIN→8×EV→END) with simulated ACKs; NAK-then-retry recovery;
a rejected EDITBEGIN failing cleanly with edits preserved (`dirty` stays true, nothing lost); an incomplete
download (declared count vs. received count mismatch) showing a clear error without building a bogus partial
model. Also regression-checked the existing button/switch/knob action editor through the refactored
`readActionFromUI`→`readActionFromFid` path — unaffected. Firmware (`navicore_record.h` + the `?REC,EDIT*`
CLI in `NaviCore.ino`) compiles clean. **NEXT: bench test** — a real clip loaded, edited (drag a servo curve,
add an HCR command, apply easing), saved, and replayed to confirm the round-trip actually drives the Maestro
as edited.

## 12. Changelog

- **v3.4 (2026-07-01):** Timeline **double-click-drag = SOLO (single-point) keyframe drag** (config-tool only) —
  Greg wanted a way to nudge one dot without the smooth-drag pulling its neighbours. A second pointerdown on the
  SAME keyframe id within 350ms (`_tlKfDownStamp`) makes that gesture ignore the falloff (weight 1 for the grabbed
  dot, 0 for all others) regardless of the "Smooth drag" radius. Guards verified: a press on a different dot, or
  past the 350ms window, both stay smooth.
- **v3.3 (2026-07-01):** Timeline-editor delete + smooth-add (config-tool only), from Greg's feedback ("how do I
  delete dots? clicking anywhere makes a new one" + "clicking off the line makes a sharp angle"). Each keyframe
  now renders a display-only visible dot (`pointer-events:none`) under a larger **transparent r=11 hit circle**
  that carries the drag/select events — small dots were easy to miss, so mis-aimed clicks kept ADDING points.
  **Right-click a dot or an action diamond deletes it** (`contextmenu` → `_tlDeleteKeyframe`/`_tlDeleteAction`,
  preventDefault kills the browser menu); select+Delete/🗑 still work. **Snap-to-line on add:** clicking empty
  space to insert a keyframe BETWEEN two existing ones snaps its value onto the interpolated line when the click
  is within 8px of it (so a near-miss doesn't kink the curve — you add a control point then drag to shape);
  a deliberate off-line click still places a peak, and extending past the ends always uses the click value.
- **v3.2 (2026-07-01):** Timeline-editor zoom + trim (config-tool only). **Zoom** now preserves a focal point
  — `_tlZoomAt(factor, clientX)` converts the focal client-x to a clip time, rescales `pxPerMs`, then restores
  that time to the same viewport offset via `scrollLeft` (previously every zoom jumped back to t=0). Wired to
  the +/− buttons (view-centre focal), **Ctrl+scroll** over the timeline (focal = pointer, finer step,
  `preventDefault` suppresses browser page-zoom), and a new **⤢ Fit** button (`_tlFit`, sizes `pxPerMs` to the
  viewport width; guards `usable<=0` so an unlaid-out container can't apply a degenerate zoom). Max zoom raised
  2→4×. **✂ Trim start** (`_tlTrimStart`): shifts every event left by the earliest `t` so the clip starts at 0,
  removing leading empty space — non-destructive (no events deleted, just re-timed; undone by closing without
  saving), reduces `durationMs`/`cursorMs` accordingly, no-op with a status note when there's no leading gap.
- **v3.1 (2026-07-01):** Timeline-editor UX pass (config-tool only) + a review-driven bug sweep.
  - **Proportional ("soft") keyframe drag:** dragging a servo/HCR dot pulls its same-track neighbours by a
    cosine falloff over an adjustable radius (`_tlSoftRadius`, "Smooth drag ___ ms" toolbar field; 0 = classic
    single-point), so you get a smooth hump/warp instead of a spike. Purely editor-side — it just moves more
    keyframes; the firmware still only linear-interpolates. Also FROZE the dragged track's value range for the
    gesture (`_tlFrozenRange`) — fixes a latent jitter where the y-axis rescaled mid-drag so the dot drifted
    off the cursor.
  - **Draggable action markers:** the Actions-row diamonds now drag horizontally to reposition in time (>3px =
    drag, else click-to-edit) — previously click-only.
  - **Review sweep (4-lens adversarial review + verify workflow, 5 confirmed):** exception-safe + idempotent
    drag teardown (`finish()` guard) so window pointer listeners can't leak and a mid-drag editor-close can't
    crash on a null `_tlClip`; shift-select reads the shift state from the pointer**up** (not the stale
    pointerdown); `_tlToggleShiftSelect` recomputes both indices fresh (a re-sort can't stale the adjacency
    test into deleting keyframes between non-adjacent picks); `_tlFlattenEvents` dedupes same-`t` keyframes
    per track (keep-last) so a drag rounding two neighbours onto one ms can't emit a nondeterministic
    duplicate-`t` to firmware. **Plus a bug the review flushed out:** the easing PAIR selection was fully
    broken — `_tlStartDragKeyframe` cleared `selectedPair` up-front, so the 2nd shift-click always wiped the
    1st (easing math worked, but you could never select the pair through the UI). Now the pair clears only on
    a real drag or a plain click. All re-verified in-browser via simulated pointer gestures.
- **v3 (2026-07-01):** Phase 2 timeline editor built — see §13 for the full writeup. New firmware: `ST_EDITING`
  state + `editBegin/editAddEvent/editEnd/editCancel/editStream` in `navicore_record.h`; `?REC,EDITLOAD/
  EDITBEGIN/EDITEV/EDITEND/EDITCANCEL` CLI. New config tool: 📈 Clips-panel button, `tl-modal`/`tl-action-modal`,
  the whole `_tl*` editor module, `_clipDlFeed` (download reassembly), `_tlWaitForMarker`/`_tlFeedMarker`
  (upload ACK waiter), `readActionFromFid` (extracted from `readActionFromUI` for reuse).
- **v2.1 (2026-07-01):** Phase 1b (interpolated replay) implemented in `navicore_record.h`. Per-channel
  `MaestroCurve` + a `_curveNext[]` chain (parallel array, one forward O(n) pass per `startReplay()`) give
  `_updateCurves()` **look-ahead**: it eases from the last *finalized* keyframe (`pPrev`@`tPrev`) toward the
  next *not-yet-finalized* one (read live via `_buf[nextIdx]`), re-anchoring once elapsed reaches that
  keyframe's time — the anchor is always in the past and the target always in the future, which the naive
  "fire event, then look at what already happened" cursor loop can't give you (a keyframe is only knowable
  once its own timestamp arrives). `reanchor` implements the `goHome` discontinuity from §3.3: mid-replay it
  suppresses emission (servo pose is unknown — snapped to a Maestro-side home we can't read back) until the
  next keyframe for that channel lands, which then **snaps** rather than easing from a stale position. Loop
  mode (`_loop`) rewinds via `_rewindCurves()` (cheap — reuses each channel's precomputed `firstIdx`, no
  buffer rescan) rather than a full rebuild, keeping `pPrev` at wherever the previous lap actually left the
  servo so the loop seam eases instead of snapping. Emission is de-duped per channel (`lastEmitted`) so a
  flat curve doesn't spam Maestro serial writes every tick.
- **v2 (2026-06-30):** validated against code by a 22-agent pass. Added: dual-core capture truth +
  single-queue concurrency design; concrete partition CSV + config-FS-wipe guard + CI-revert fix
  (tools-only); replay speed/accel reset; HCR-volume cache bypass; `goHome` shadow/curve invalidation;
  `servoHome` slot-keying rule + `0xFFFF` sentinel; MP3 NEXT/PREV determinism correction; `hold`→switch
  scoping; header `boardType`/`auxBaud`.
- **v1:** initial design.
