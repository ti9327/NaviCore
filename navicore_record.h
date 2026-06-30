#pragma once
#include <Arduino.h>
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
inline uint32_t      _replayStart = 0;
inline uint32_t      _cursor   = 0;          // replay event cursor
inline uint32_t      _drops    = 0;          // queue-full drops (diagnostic)

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
  if (slot >= 1 && slot <= 8) for (int c = 0; c < 32; c++) _lastPos[slot - 1][c] = LASTPOS_NONE;
}

// ── Capture taps (noinline + small POD: keep the event off the Core-0 ESP-NOW
//    callback frame, exactly the queueRemoteCli lesson). Non-blocking, drop-if-
//    full — a dropped keyframe just thins one frame; a stall would brick. ───────
inline void __attribute__((noinline)) captureAction(const RcAction& a) {
  if (!_capturing) return;
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

// ── Replay ───────────────────────────────────────────────────────────────────
inline bool startReplay() {
  if (_state != ST_IDLE || _count == 0) return false;
  _capturing = false;                         // never re-record replayed events
  // Reset speed/accel on every touched channel so our timing is authoritative
  // (latched Pololu limits would double-smooth), then ease to servoHome.
  for (int s = 0; s < 8; s++)
    for (int c = 0; c < 32; c++)
      if (_lastPos[s][c] != LASTPOS_NONE) {
        if (_cbResetChan)   _cbResetChan((uint8_t)(s + 1), (uint8_t)c);
        if (_cbEmitMaestro) _cbEmitMaestro((uint8_t)(s + 1), (uint8_t)c, _lastPos[s][c]);
      }
  _cursor = 0;
  _replayStart = millis();
  _state = ST_REPLAYING;
  return true;
}

// ── Replay tick (loop, Core 1). Phase 1: fire each event at its timestamp.
//    (Inter-keyframe interpolation = phase 1b.) ─────────────────────────────────
inline void replayTick() {
  if (_state != ST_REPLAYING) return;
  uint32_t elapsed = millis() - _replayStart;
  while (_cursor < _count && _buf[_cursor].tMs <= elapsed) {
    const RecEvent& ev = _buf[_cursor++];
    switch (ev.kind) {
      case REC_ACTION:     if (_cbDispatch)   _cbDispatch(ev.u.act); break;
      case REC_KF_MAESTRO: if (_cbEmitMaestro) _cbEmitMaestro(ev.u.km.slot, ev.u.km.ch, ev.u.km.pos); break;
      case REC_KF_HCRVOL:  if (_cbEmitHcrVol)  _cbEmitHcrVol(ev.u.kv.chan, ev.u.kv.vol); break;
    }
  }
  if (_cursor >= _count) _state = ST_IDLE;    // replay complete
}

inline bool isReplaying() { return _state == ST_REPLAYING; }   // gate live dispatch

inline void info(Print& out) {
  uint32_t durMs = _count ? _buf[_count - 1].tMs : 0;
  const char* st = _state == ST_RECORDING ? "RECORDING" : _state == ST_REPLAYING ? "REPLAYING" : "idle";
  out.printf("[REC] state=%s  events=%lu/%lu  dur=%lums  drops=%lu  buf=%s\n",
             st, (unsigned long)_count, (unsigned long)_cap, (unsigned long)durMs,
             (unsigned long)_drops, _buf ? "ok" : "OOM");
}

}  // namespace navirec
