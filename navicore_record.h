#pragma once
#include <Arduino.h>
#include <FS.h>
#include "rc_config.h"

// ─────────────────────────────────────────────────────────────────────────────
//  NaviCore record / replay — PHASE 1 (in-PSRAM clip, no persistence yet)
// ─────────────────────────────────────────────────────────────────────────────
// Captures every dispatched droid action as a timestamped event, into a single
// in-PSRAM clip, and replays it by re-dispatching with the original timing.
// See docs/RECORD_REPLAY_DESIGN.md. This module is DISPATCH-AGNOSTIC: NaviCore's
// dispatch helpers are `static`, so we reach them through callbacks registered
// from NaviCore.ino (recBegin).
//
// CONCURRENCY (the load-bearing part): the action tap (rcExecuteActionNow) runs
// on BOTH cores — Core 0 for a remote ESP-NOW TRIGGER (onWCBCommand ->
// rcTelemetry::handle -> rcDispatch, no defer) and Core 1 for local input. So
// captures go through a FreeRTOS queue (non-blocking, drop-if-full, small POD,
// noinline enqueue — same discipline as the OTA/remote-CLI queues that fixed the
// prior WiFi-task stack overflow). loop() drains the queue into the PSRAM buffer
// on Core 1 as the SOLE writer. The lastPos servo shadow is a single atomic
// 16-bit store, safe across cores without a lock.
namespace navirec {

enum RecKind : uint8_t { REC_ACTION = 0, REC_KF_MAESTRO = 1, REC_KF_HCRVOL = 2 };
enum State    : uint8_t { ST_IDLE = 0, ST_RECORDING = 1, ST_REPLAYING = 2 };

struct RecEvent {
  uint32_t tMs;     // ms from record start
  uint8_t  kind;    // RecKind
  union {
    RcAction act;                                    // REC_ACTION
    struct { uint8_t slot, ch; uint16_t pos; } km;   // REC_KF_MAESTRO
    struct { uint8_t chan, vol; } kv;                // REC_KF_HCRVOL
  } u;
};

static const uint16_t LASTPOS_NONE   = 0xFFFF;       // never-commanded sentinel
static const uint32_t REC_MAX_MS     = 30000;        // 30 s capture cap (backstop)

// Callbacks into NaviCore.ino's (static) dispatch layer.
typedef void (*DispatchActionFn)(const RcAction&);          // re-dispatch a captured action
typedef void (*EmitMaestroFn)(uint8_t slot, uint8_t ch, uint16_t pos);
typedef void (*EmitHcrVolFn)(uint8_t chan, uint8_t vol);    // RAW (bypasses dispatchHcrVolume cache)
typedef void (*ResetChanFn)(uint8_t slot, uint8_t ch);      // speed=0/accel=0

// ── Module state ─────────────────────────────────────────────────────────────
inline RecEvent*     _buf      = nullptr;
inline uint32_t      _cap      = 0;          // max events
inline uint32_t      _count    = 0;          // events in clip
inline volatile uint8_t _state = ST_IDLE;
inline volatile bool _capturing = false;     // gate read in the tap (single volatile)
inline uint32_t      _recStart = 0;
inline uint8_t       _mode     = 1;          // mode at capture (context)
inline QueueHandle_t _queue    = nullptr;
inline volatile uint16_t _lastPos[8][32];    // [slot-1][ch], LASTPOS_NONE = unset

// Per-channel replay interpolation curve (phase 1b — see _buildCurveIndex below
// for the full explanation). Declared here (not next to _buildCurveIndex) so
// recBegin() below can allocate _curveNext before first use.
struct MaestroCurve {
  uint16_t pPrev;        // qus of the last finalized keyframe (or servoHome) — ease-from anchor
  uint32_t tPrev;        // its tMs
  int16_t  firstIdx;     // this channel's first keyframe in the clip, -1 = none (set once, for loop rewind)
  int16_t  nextIdx;      // next NOT-YET-finalized keyframe; walked forward as replay advances
  uint16_t lastEmitted;  // de-dupes a redundant _cbEmitMaestro call when the curve hasn't moved
  bool     active;       // channel has a servoHome or at least one keyframe to follow this replay
  bool     reanchor;     // pose is unknown (no home, or a goHome fired mid-replay) — suppress
                          // emission until the next keyframe finalizes and re-anchors the curve
};
inline MaestroCurve _curve[8][32];
inline int16_t*     _curveNext = nullptr;   // parallel to _buf; sized to _cap in recBegin()

inline uint32_t      _replayStart = 0;
inline uint32_t      _cursor   = 0;          // replay event cursor
inline uint32_t      _drops    = 0;          // queue-full drops (diagnostic)
inline volatile bool _doneFlag = false;      // set when a replay completes (loop drains it)
inline volatile bool _loop     = false;      // replay repeats on completion (idle-animation mode)
// Deferred control: a Record/Play trigger can dispatch on Core 0 (remote
// ESP-NOW TRIGGER), but startReplay does Maestro serial writes (ease-in) that
// must run on Core 1. So triggers just set a request here; pollControl() runs it
// from loop(). A single volatile byte is a lock-free edge (last-writer-wins).
enum Ctl : uint8_t { CTL_NONE = 0, CTL_REC_TOGGLE = 1, CTL_PLAY = 2, CTL_STOP = 3 };
inline volatile uint8_t _pendingCtl  = CTL_NONE;
inline volatile uint8_t _pendingMode = 1;    // mode captured at a record request
inline volatile bool    _pendingLoop = false;

inline DispatchActionFn _cbDispatch   = nullptr;
inline EmitMaestroFn    _cbEmitMaestro = nullptr;
inline EmitHcrVolFn     _cbEmitHcrVol  = nullptr;
inline ResetChanFn      _cbResetChan   = nullptr;

// ── Init (Core 1, setup() — BEFORE wcb->begin() so the recv callback can't hit
//    a null queue). cap events ~150 B each → 8192 ≈ 1.2 MB PSRAM. ───────────────
inline void recBegin(uint32_t cap, DispatchActionFn d, EmitMaestroFn m,
                     EmitHcrVolFn v, ResetChanFn r) {
  _cap = cap;
  _buf = (RecEvent*) ps_malloc((size_t)cap * sizeof(RecEvent));
  if (!_buf) _cap = 0;                       // OOM → recorder disabled, never crashes
  _queue = xQueueCreate(32, sizeof(RecEvent));
  // Curve-chain index for replay interpolation (indices must fit int16_t; cap is
  // a fixed 8192 today). OOM here degrades gracefully — replayTick() null-checks
  // _curveNext and each channel just holds at its first keyframe instead of
  // chaining through the rest, rather than crashing.
  _curveNext = (cap && cap <= 32767) ? (int16_t*) ps_malloc((size_t)cap * sizeof(int16_t)) : nullptr;
  _cbDispatch = d; _cbEmitMaestro = m; _cbEmitHcrVol = v; _cbResetChan = r;
  for (int s = 0; s < 8; s++) for (int c = 0; c < 32; c++) _lastPos[s][c] = LASTPOS_NONE;
}

// ── Servo last-position shadow (called from maestroSetTarget — ALL moves) ─────
// Single naturally-aligned 16-bit store → lock-free, torn-write-free on Xtensa
// even though maestroSetTarget runs on both cores. NO bitmap/counter RMW here.
inline void shadowSetTarget(uint8_t slot, uint8_t ch, uint16_t pos) {
  if (slot >= 1 && slot <= 8 && ch < 32) _lastPos[slot - 1][ch] = pos;
}
inline void shadowInvalidateSlot(uint8_t slot) {            // goHome/stopScript: pose unknown
  if (slot < 1 || slot > 8) return;
  for (int c = 0; c < 32; c++) {
    _lastPos[slot - 1][c] = LASTPOS_NONE;
    // Mid-replay this is a curve discontinuity (design §3.3): the servo just
    // snapped to a Maestro-side home position we can't read back, so the
    // interpolation curve's anchor is stale. _updateCurves() (Core 1, same
    // context replay dispatches goHome from) sees reanchor and holds silent
    // until the next keyframe for this channel lands, then snaps straight to
    // it instead of easing from a position that's no longer physically true.
    if (_state == ST_REPLAYING) _curve[slot - 1][c].reanchor = true;
  }
}

// ── Capture taps (noinline + small POD: keep the event off the Core-0 ESP-NOW
//    callback frame, exactly the queueRemoteCli lesson). Non-blocking, drop-if-
//    full — a dropped keyframe just thins one frame; a stall would brick. ───────
inline void __attribute__((noinline)) captureAction(const RcAction& a) {
  if (!_capturing) return;
  if (a.type == RA_RECORD || a.type == RA_PLAY || a.type == RA_STOP) return;   // meta — never record the record/play/stop triggers
  RecEvent ev; ev.tMs = millis() - _recStart; ev.kind = REC_ACTION; ev.u.act = a;
  if (xQueueSend(_queue, &ev, 0) != pdTRUE) _drops++;
}
inline void __attribute__((noinline)) captureMaestroKf(uint8_t slot, uint8_t ch, uint16_t pos) {
  if (!_capturing) return;
  RecEvent ev; ev.tMs = millis() - _recStart; ev.kind = REC_KF_MAESTRO;
  ev.u.km.slot = slot; ev.u.km.ch = ch; ev.u.km.pos = pos;
  if (xQueueSend(_queue, &ev, 0) != pdTRUE) _drops++;
}
inline void __attribute__((noinline)) captureHcrVolKf(uint8_t chan, uint8_t vol) {
  if (!_capturing) return;
  RecEvent ev; ev.tMs = millis() - _recStart; ev.kind = REC_KF_HCRVOL;
  ev.u.kv.chan = chan; ev.u.kv.vol = vol;
  if (xQueueSend(_queue, &ev, 0) != pdTRUE) _drops++;
}

// ── Drain (loop, Core 1 — SOLE writer of the PSRAM buffer) ────────────────────
inline void drain() {
  if (!_queue) return;
  RecEvent ev;
  while (xQueueReceive(_queue, &ev, 0) == pdTRUE) {
    if (_state == ST_RECORDING && _count < _cap) _buf[_count++] = ev;
  }
  // 30 s backstop — auto-stop a runaway capture.
  if (_state == ST_RECORDING && (millis() - _recStart) > REC_MAX_MS) {
    _capturing = false; _state = ST_IDLE;
  }
}

// ── Record control ───────────────────────────────────────────────────────────
inline bool startRecord(uint8_t mode) {
  if (_state != ST_IDLE || !_buf) return false;
  _mode = mode; _count = 0; _drops = 0;
  _recStart = millis();
  _state = ST_RECORDING;
  _capturing = true;                          // arm AFTER recStart so tMs is sane
  return true;
}
inline void stopRecord() {
  if (_state != ST_RECORDING) return;
  _capturing = false;
  drain();                                    // flush in-flight events
  _state = ST_IDLE;
}
inline void clearClip() { if (_state == ST_IDLE) _count = 0; }
inline uint32_t clipDurationMs() { return _count ? _buf[_count - 1].tMs : 0; }
inline uint32_t eventCount()     { return _count; }

// Abort whatever is active — stop a recording OR an in-progress replay. Returns
// what it stopped so the CLI can report it. Replay-abort hands live control back
// next loop (the isReplaying() gates clear).
inline uint8_t stop() {
  uint8_t was = _state;
  if (_state == ST_RECORDING) { _capturing = false; drain(); }
  _state = ST_IDLE;
  return was;
}

// ── Deferred control (set by RA_RECORD/RA_PLAY dispatch, run in loop) ─────────
inline char _pendingName[41] = {0};   // clip name carried by a Record/Play trigger
inline void requestRecordToggle(uint8_t mode, const char* name) {
  _pendingMode = mode; strlcpy((char*)_pendingName, name ? name : "", sizeof(_pendingName)); _pendingCtl = CTL_REC_TOGGLE;
}
inline void requestPlay(const char* name, bool loop) {
  _pendingLoop = loop; strlcpy((char*)_pendingName, name ? name : "", sizeof(_pendingName)); _pendingCtl = CTL_PLAY;
}
inline void requestStop() { _pendingCtl = CTL_STOP; }

// ── Maestro keyframe interpolation (phase 1b) ──────────────────────────────────
// Capture is dense (every knob-deadband keyframe kept), but firing each one AT
// its timestamp still steps servos at capture cadence — this smooths replay by
// emitting a linearly-interpolated qus position for every active (slot,ch)
// EVERY replayTick(), decoupling smoothness from both the original capture
// cadence and loop() jitter (design: docs/RECORD_REPLAY_DESIGN.md §3).
//
// Because a keyframe is only "known" once its own timestamp arrives, true
// look-ahead interpolation needs a channel-scoped chain PARALLEL to the flat,
// time-ordered event buffer: `_curveNext[i]` = index of the next REC_KF_MAESTRO
// event sharing (slot,ch) with event i (-1 = chain end). Built once per
// startReplay() with a single O(n) forward pass (cheap — capped at `_cap`
// events). Each channel eases from `pPrev`@`tPrev` (the last FINALIZED
// keyframe) toward `_buf[nextIdx]` (the next, still-future one); once elapsed
// reaches that keyframe's time it's finalized (becomes the new pPrev/tPrev)
// and the chain advances — so the anchor is always in the past and the target
// always in the future, the invariant real tweening needs. (MaestroCurve and
// _curve/_curveNext are declared up in ── Module state ── so recBegin() can
// allocate _curveNext before this point.)

// Initial build, called once from startReplay(). Also resets speed/accel (0 =
// unlimited) on every touched channel — the Maestro's OWN latched limits live
// device-side and can't be read back, so we force-reset rather than snapshot,
// else they'd double-smooth our interpolated stream — then eases known-home
// channels toward their current position (a no-op if nothing drifted).
inline void _buildCurveIndex() {
  int16_t lastIdx[8][32];
  for (int s = 0; s < 8; s++) {
    for (int c = 0; c < 32; c++) {
      lastIdx[s][c] = -1;
      MaestroCurve& cv = _curve[s][c];
      bool known = (_lastPos[s][c] != LASTPOS_NONE);
      cv.pPrev = known ? _lastPos[s][c] : 0;
      cv.tPrev = 0;
      cv.firstIdx = cv.nextIdx = -1;
      cv.lastEmitted = cv.pPrev;
      cv.active = known;
      cv.reanchor = !known;   // no known home → the first keyframe snaps instead of easing from 0
    }
  }
  for (uint32_t i = 0; i < _count; i++) {
    if (_buf[i].kind != REC_KF_MAESTRO) continue;
    uint8_t s = _buf[i].u.km.slot - 1, c = _buf[i].u.km.ch;
    if (s >= 8 || c >= 32) continue;
    MaestroCurve& cv = _curve[s][c];
    if (lastIdx[s][c] < 0) cv.firstIdx = cv.nextIdx = (int16_t)i;   // first keyframe for this channel
    else if (_curveNext)   _curveNext[lastIdx[s][c]] = (int16_t)i;
    lastIdx[s][c] = (int16_t)i;
    cv.active = true;
  }
  for (int s = 0; s < 8; s++) {
    for (int c = 0; c < 32; c++) {
      if (lastIdx[s][c] >= 0 && _curveNext) _curveNext[lastIdx[s][c]] = -1;   // terminate the chain
      MaestroCurve& cv = _curve[s][c];
      if (!cv.active) continue;
      if (_cbResetChan) _cbResetChan((uint8_t)(s + 1), (uint8_t)c);
      if (!cv.reanchor && _cbEmitMaestro) _cbEmitMaestro((uint8_t)(s + 1), (uint8_t)c, cv.pPrev);
    }
  }
}

// Idle-animation loop restart: rewind each active channel's cursor to its first
// keyframe (cheap — no event-buffer rescan) while KEEPING pPrev at wherever the
// previous lap actually left the servo, so the loop seam eases rather than
// snapping; tPrev resets to 0 to re-anchor against the new lap's elapsed clock.
inline void _rewindCurves() {
  for (int s = 0; s < 8; s++) {
    for (int c = 0; c < 32; c++) {
      MaestroCurve& cv = _curve[s][c];
      if (!cv.active) continue;
      cv.tPrev = 0;
      cv.nextIdx = cv.firstIdx;
      cv.reanchor = false;
    }
  }
}

// Advance + emit every active curve for the current elapsed time. Called once
// per replayTick(), independent of the discrete-event cursor.
inline void _updateCurves(uint32_t elapsed) {
  for (int s = 0; s < 8; s++) {
    for (int c = 0; c < 32; c++) {
      MaestroCurve& cv = _curve[s][c];
      if (!cv.active) continue;
      while (cv.nextIdx >= 0 && _buf[cv.nextIdx].tMs <= elapsed) {
        cv.pPrev = _buf[cv.nextIdx].u.km.pos;
        cv.tPrev = _buf[cv.nextIdx].tMs;
        cv.reanchor = false;               // finalized — pose is known again
        cv.nextIdx = _curveNext ? _curveNext[cv.nextIdx] : (int16_t)-1;
      }
      if (cv.reanchor) continue;           // pose still unknown — hold silent until the next keyframe lands
      uint16_t pos;
      if (cv.nextIdx < 0) {
        pos = cv.pPrev;                    // no more keyframes for this channel — hold position
      } else {
        uint32_t tNext = _buf[cv.nextIdx].tMs;
        uint16_t pNext = _buf[cv.nextIdx].u.km.pos;
        if (tNext <= cv.tPrev) {
          pos = pNext;                     // degenerate/zero-length segment — snap
        } else {
          float frac = (float)(elapsed - cv.tPrev) / (float)(tNext - cv.tPrev);
          pos = (uint16_t)((int32_t)cv.pPrev + (int32_t)(((int32_t)pNext - (int32_t)cv.pPrev) * frac));
        }
      }
      if (pos != cv.lastEmitted) {
        if (_cbEmitMaestro) _cbEmitMaestro((uint8_t)(s + 1), (uint8_t)c, pos);
        cv.lastEmitted = pos;
      }
    }
  }
}

// ── Replay ───────────────────────────────────────────────────────────────────
inline bool startReplay(bool loop = false) {
  if (_state != ST_IDLE || _count == 0) return false;
  _capturing = false;                         // never re-record replayed events
  _loop = loop;
  _buildCurveIndex();                         // resets speed/accel + eases every touched Maestro channel
  _cursor = 0;
  _replayStart = millis();
  _state = ST_REPLAYING;
  return true;
}

// ── Replay tick (loop, Core 1). Discrete actions/HCR-volume fire at their exact
//    timestamp; Maestro motion is driven continuously by _updateCurves(). ──────
inline void replayTick() {
  if (_state != ST_REPLAYING) return;
  uint32_t elapsed = millis() - _replayStart;
  while (_cursor < _count && _buf[_cursor].tMs <= elapsed) {
    const RecEvent& ev = _buf[_cursor++];
    switch (ev.kind) {
      case REC_ACTION:     if (_cbDispatch)   _cbDispatch(ev.u.act); break;
      case REC_KF_MAESTRO: break;   // driven by the interpolated curve follower, not fired directly
      case REC_KF_HCRVOL:  if (_cbEmitHcrVol)  _cbEmitHcrVol(ev.u.kv.chan, ev.u.kv.vol); break;
    }
  }
  _updateCurves(elapsed);
  // Complete when all events have fired, or as a safety net 1 s past the clip's
  // length (so a bad timestamp can never wedge replay + lock out live control).
  if (_cursor >= _count || elapsed > clipDurationMs() + 1000) {
    if (_loop && _count) {                 // idle-animation loop — restart the event stream
      _cursor = 0; _replayStart = millis();
      _rewindCurves();
    } else {
      _state = ST_IDLE;
      _doneFlag = true;   // loop() prints a "playback complete" line on the transition
    }
  }
}

// One-shot "replay just finished" edge — drained by loop() so the message prints
// from Core-1 context (the player completes asynchronously, not inside a CLI call).
inline bool takeReplayDone() { if (_doneFlag) { _doneFlag = false; return true; } return false; }

inline bool isReplaying() { return _state == ST_REPLAYING; }   // gate live dispatch

// ── Persistence: clip library on the dedicated `clips` LittleFS ───────────────
// The host FS is injected from NaviCore.ino (dispatch-agnostic). File format:
// ClipFileHeader + raw RecEvent[count]. All flash I/O runs in loop() context
// (save on record-stop, load on play, both via pollControl / CLI) — never Core 0.
inline fs::FS* _clipFS = nullptr;
inline void setClipFS(fs::FS* fs) { _clipFS = fs; }
inline bool clipFsReady() { return _clipFS != nullptr; }

#pragma pack(push, 1)
struct ClipFileHeader {
  char     magic[4];   // "NCR1"
  uint16_t version;    // 1
  uint16_t mode;       // FunctionSwState at record time (clip context)
  uint32_t count;      // RecEvent entries following the header
  uint32_t durationMs;
};
#pragma pack(pop)

// Sanitize a user clip name into a path "/<name>.ncr" (alnum + _-, <=32 chars).
inline bool _clipPath(char* out, size_t n, const char* name) {
  if (!name || !name[0]) return false;
  char clean[33]; size_t j = 0;
  for (size_t i = 0; name[i] && j < sizeof(clean) - 1; i++) {
    char ch = name[i];
    if (isalnum((int)ch) || ch == '_' || ch == '-') clean[j++] = ch;
  }
  clean[j] = 0;
  if (!j) return false;
  snprintf(out, n, "/%s.ncr", clean);
  return true;
}

inline bool saveClip(const char* name) {
  if (!_clipFS || _state != ST_IDLE || _count == 0) return false;
  char path[48]; if (!_clipPath(path, sizeof(path), name)) return false;
  File f = _clipFS->open(path, "w", true);
  if (!f) return false;
  ClipFileHeader h; memcpy(h.magic, "NCR1", 4); h.version = 1;
  h.mode = _mode; h.count = _count; h.durationMs = clipDurationMs();
  size_t bodyN = (size_t)_count * sizeof(RecEvent);
  bool ok = (f.write((const uint8_t*)&h, sizeof(h)) == sizeof(h)) &&
            (f.write((const uint8_t*)_buf, bodyN) == bodyN);
  f.close();
  return ok;
}

inline bool loadClip(const char* name) {
  if (!_clipFS || _state != ST_IDLE || !_buf) return false;
  char path[48]; if (!_clipPath(path, sizeof(path), name)) return false;
  File f = _clipFS->open(path, "r");
  if (!f) return false;
  ClipFileHeader h;
  if (f.read((uint8_t*)&h, sizeof(h)) != (int)sizeof(h) || memcmp(h.magic, "NCR1", 4) != 0) { f.close(); return false; }
  uint32_t n = (h.count > _cap) ? _cap : h.count;
  size_t got = f.read((uint8_t*)_buf, (size_t)n * sizeof(RecEvent));
  f.close();
  _count = got / sizeof(RecEvent);
  _mode  = h.mode;
  return _count > 0;
}

inline bool deleteClip(const char* name) {
  if (!_clipFS) return false;
  char path[48]; if (!_clipPath(path, sizeof(path), name)) return false;
  return _clipFS->remove(path);
}

inline bool renameClip(const char* from, const char* to) {
  if (!_clipFS) return false;
  char pf[48], pt[48];
  if (!_clipPath(pf, sizeof(pf), from) || !_clipPath(pt, sizeof(pt), to)) return false;
  if (_clipFS->exists(pt)) return false;              // don't clobber an existing clip
  return _clipFS->rename(pf, pt);
}

// Emits the clip library as one small "[CLIPITEM]{...}" line per clip, bracketed
// by "[CLIPLIST:BEGIN]" / "[CLIPLIST:END]", which the config-tool Clips panel
// accumulates.  Each line stays well under the 160-byte RTERM packet limit, so it
// survives the WCB bridge intact — a single long "[CLIPLIST]{...}" line would be
// hard-wrapped mid-JSON at 160 bytes (navicore_rterm.h) into two [TERM:] lines
// and fail to parse.  Per-clip lines also parse independently, so relayed log
// noise landing between them can't corrupt a neighbouring clip's name.  Clip names
// are already sanitised to [A-Za-z0-9_-] on save, so no JSON escaping is needed.
inline void listClips(Print& out) {
  out.println("[CLIPLIST:BEGIN]");
  if (_clipFS) {
    File dir = _clipFS->open("/");
    for (File e = dir ? dir.openNextFile() : File(); e; e = dir.openNextFile()) {
      if (e.isDirectory()) continue;
      String nm = e.name();
      int slash = nm.lastIndexOf('/'); if (slash >= 0) nm = nm.substring(slash + 1);
      if (nm.endsWith(".ncr")) nm = nm.substring(0, nm.length() - 4);
      uint32_t dur = 0; ClipFileHeader h;
      if (e.read((uint8_t*)&h, sizeof(h)) == (int)sizeof(h) && memcmp(h.magic, "NCR1", 4) == 0) dur = h.durationMs;
      out.printf("[CLIPITEM]{\"name\":\"%s\",\"bytes\":%u,\"dur\":%lu}\n",
                 nm.c_str(), (unsigned)e.size(), (unsigned long)dur);
    }
  }
  out.println("[CLIPLIST:END]");
}

// Run a deferred Record/Play trigger from loop() (Core 1). Play toggles: press
// while playing → stop, so one button both starts and stops (incl. a loop).
inline void pollControl() {
  uint8_t c = _pendingCtl;
  if (c == CTL_NONE) return;
  _pendingCtl = CTL_NONE;
  switch (c) {
    case CTL_REC_TOGGLE:
      if (_state == ST_RECORDING) {
        stopRecord();
        if (_pendingName[0]) saveClip((const char*)_pendingName);   // save to the trigger's clip name
      } else if (_state == ST_IDLE) {
        startRecord(_pendingMode);
      }
      break;
    case CTL_PLAY:
      if (_state == ST_REPLAYING) {
        stop();                                                      // toggle off
      } else if (_state == ST_IDLE) {
        // Named clip: load it first; if it's missing, don't play the stale RAM clip.
        if (_pendingName[0] && !loadClip((const char*)_pendingName)) break;
        startReplay(_pendingLoop);
      }
      break;
    case CTL_STOP:
      stop();
      break;
  }
}

inline void info(Print& out) {
  uint32_t durMs = _count ? _buf[_count - 1].tMs : 0;
  const char* st = _state == ST_RECORDING ? "RECORDING" : _state == ST_REPLAYING ? "REPLAYING" : "idle";
  out.printf("[REC] state=%s  events=%lu/%lu  dur=%lums  drops=%lu  buf=%s\n",
             st, (unsigned long)_count, (unsigned long)_cap, (unsigned long)durMs,
             (unsigned long)_drops, _buf ? "ok" : "OOM");
}

}  // namespace navirec
