// =============================================================================
//  rc_telemetry.h
//
//  WCB-network-side bridge for remote management of the RC-Controller.  The
//  user-facing UI stays in config_tool/index.html (the existing RC config
//  tool); the WCB ESP-NOW network is just a TRANSPORT that lets the tool
//  reach a remote RC through a USB-tethered "bridge" WCB.
//
//  Architecture overview:
//
//    [Browser: config_tool/index.html]
//          │
//          │ Web Serial  (Direct USB)                 (Via WCB)
//          ▼                                              ▼
//    [USB RC-Controller]              [USB WCB] ─ ESP-NOW ─ [remote RC]
//
//  In "Via WCB" mode the config tool sends the SAME JSON messages it already
//  uses over direct USB; the bridge WCB relays them as ESP-NOW packets to
//  the target RC by deviceId.  Replies and live telemetry come back the
//  same way.  The RC firmware doesn't care which transport carried a
//  given JSON message — it dispatches identically.
//
//  This header provides three things:
//
//    1. OUTBOUND TELEMETRY  (rcTelemetry::tick / emitTrig / emitMode)
//       Periodic + event-driven JSON broadcasts the bridge WCB forwards
//       to the config tool.  Mirror of the existing PWM_UPDATE stream
//       but ESP-NOW-sized (≤250 bytes).
//
//         {"type":"rc_hb",  "id", "fw", "up", "mode", "model"}   0.5 Hz
//         {"type":"rc_ch",  "id", "ch":[24 values]}              5 Hz
//         {"type":"rc_trig","id", "mode", "btn", "tap"}          event
//         {"type":"rc_mode","id", "mode"}                        event
//
//    2. INBOUND COMMAND HANDLER  (rcTelemetry::handle)
//       Called from onWCBCommand().  Detects JSON payloads (leading '{')
//       addressed to this RC and routes them to the existing dispatch /
//       mode paths.  Accepts the same message types the config tool
//       already sends over Web Serial:
//
//         {"type":"PING"}                                   → PONG
//         {"type":"TRIGGER","mode","btn","tap"}             → rcDispatch
//         {"type":"SET_MODE","mode"}                        → FunctionSwState
//
//       SET_CONFIG / GET_CONFIG are intentionally NOT handled over the
//       WCB transport — a full config easily exceeds the 250-byte ESP-NOW
//       packet limit.  The config tool surfaces those as "use Direct USB
//       for full-config push/pull" in its UI.
//
//  Required externs (defined in RC-Controller.ino):
//    extern WCB_Client*   wcb;
//    extern int           FunctionSwState;
//    extern int           sbusValues[24];     // signed int per the .ino's declaration
//    void                 rcDispatch(int buttonId, uint8_t tapCount);
// =============================================================================

#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WCB_Client.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "rc_config.h"
#include "fw_version.h"

// Forward declarations from RC-Controller.ino.  Type signature for
// sbusValues MUST match the .ino's `int sbusValues[24]` declaration or
// the linker pulls the wrong symbol layout.
extern WCB_Client*  wcb;
extern int          FunctionSwState;
extern int          sbusValues[24];
// SBUS health metrics from the local reader — surface them in rc_hb so the
// Via-WCB config tool can show the ACTUAL receive rate the RC is seeing,
// not the rate at which we're broadcasting rc_ch over ESP-NOW (5 Hz).
extern int           sbusFps;            // frames/sec from the local SBUS reader
extern unsigned long sbusLastFrameMs;    // millis() at last received frame (0 = none yet)
extern bool          lostFrameOld;       // SBUS lost-frame flag (no signal)
extern bool          sbusFailsafe;       // SBUS failsafe flag (TX lost / disarmed)
void                rcDispatch(int buttonId, uint8_t tapCount);

// Forward declarations from rc_config.h — needed for SET_CONFIG / GET_CONFIG
// fragmentation handlers below.  Both already exist; declared here so
// rc_telemetry.h doesn't have to #include rc_config.h's huge implementation.
String rcConfigToJSON();
bool   rcConfigFromJSON(const String& json);
bool   rcConfigFromJSON(const JsonObject& doc);   // JsonObject overload — skip the double-parse
void   rcConfigSaveNVS();

namespace rcTelemetry {

// ── App-level fragmentation for oversize JSON over WCB transport ────────────
// ESP-NOW packets max out at 252 bytes; SET_CONFIG / GET_CONFIG payloads are
// kilobytes.  Sender slices the JSON into ~160-byte chunks and wraps each in
// a fragment envelope:
//
//     {"f":1,"of":3,"sid":42,"s":"…slice of original JSON…"}
//
// Receiver buffers by sid, concatenates `s` fields in fragment order, parses
// the result as JSON, then routes through the normal dispatch.  A small
// session pool + 5-second timeout handles concurrent transfers and dropped
// packets (the requester re-issues if no response).
// Why 80 (not 160): WCB_Client's _sendPacket() copies the envelope into a
// 200-byte structCommand buffer and snprintf's "|CRC%08X" (12 bytes) onto
// the end.  If envelope > 187 bytes the CRC suffix gets truncated and the
// bridge's CRC verifier silently drops the fragment.  The envelope wraps
// the slice in `{"f":N,"of":M,"sid":S,"s":"…"}` and JSON-escapes every
// quote / backslash, so an 80-byte slice can grow to ~130 bytes once
// escaped + wrapped — comfortably under 187.  18 fragments × 160 = 2.88 KB
// previously fit a typical config; at 80 bytes/fragment we need ~36 for
// the same config, still well under FRAG_MAX_PARTS.
constexpr size_t   FRAG_CHUNK_BYTES  = 80;   // underlying-JSON bytes per fragment
// Bumped 64 → 192 so a fully-populated RcConfig (mappings + per-button
// action chains + maestro slots + ...) fits.  Worst-case envelope size
// per session: 192 × sizeof(String) ≈ 3 KB pointer overhead until any
// fragment actually arrives; reasonable for the ESP32-S3's heap.
constexpr uint8_t  FRAG_MAX_PARTS    = 192;  // 192 × 80 = 15 KB max payload
constexpr uint8_t  FRAG_POOL_SIZE    = 3;    // concurrent reassembly sessions
constexpr uint32_t FRAG_TIMEOUT_MS   = 5000;

struct FragSession {
  uint16_t sid;          // 0 = slot free
  uint8_t  total;        // expected fragment count
  uint8_t  got;          // received so far
  uint8_t  senderID;     // who sent the fragments (for the SET_CONFIG ack)
  uint32_t expireAt;     // millis() when this session is reclaimed
  String   parts[FRAG_MAX_PARTS];
  // Explicit "have we received this fragment yet?" flags.  Previously we
  // used parts[i].length() == 0 as the proxy, but if a sender ever
  // produced an empty `s` field (legitimately empty slice) the duplicate
  // check would treat every duplicate as new, inflate `got`, and
  // potentially complete the session with garbage.  Separate flag array
  // is unambiguous.  192 bools = 192 bytes — fine.
  bool     received[FRAG_MAX_PARTS];
};

inline FragSession _fragPool[FRAG_POOL_SIZE] = {};
inline uint16_t    _nextOutSid = 1;            // sender-side sid generator

inline FragSession* _findOrAllocSession(uint16_t sid, uint8_t total, uint8_t senderID) {
  const uint32_t now = millis();
  FragSession* freeSlot = nullptr;
  // First pass: reclaim expired slots BEFORE the sid-match check.  If we
  // checked sid first, a stale session whose `expireAt` is in the past
  // but whose sid happens to equal the new sid (likely after _nextOutSid
  // wraps from 65535 → 1) would be returned with old `parts[]` content,
  // causing the new fragments to merge into stale data.  Reclaim-first
  // guarantees that any returned slot is either fresh-claimed or genuine.
  for (auto& s : _fragPool) {
    if (s.sid != 0 && (int32_t)(now - s.expireAt) >= 0) {
      s = FragSession{};
    }
  }
  for (auto& s : _fragPool) {
    if (s.sid == sid)             return &s;
    if (!freeSlot && s.sid == 0)  freeSlot = &s;
  }
  if (!freeSlot) return nullptr;                // pool exhausted — drop
  freeSlot->sid      = sid;
  freeSlot->total    = total;
  freeSlot->got      = 0;
  freeSlot->senderID = senderID;
  freeSlot->expireAt = now + FRAG_TIMEOUT_MS;
  // FragSession{} default-construction already zeroed `received[]`, but
  // when we re-claim a slot that previously had a different sid we want
  // to be sure no stale flags carry over.  Cheap belt-and-braces.
  for (uint8_t i = 0; i < FRAG_MAX_PARTS; i++) freeSlot->received[i] = false;
  return freeSlot;
}

// Forward declaration so handle() can recurse on the reassembled message.
inline bool handle(uint8_t senderID, const char* command);

// ── Outbound: fragment + unicast a (potentially large) JSON string ──────────
// Ships a CONFIG response (rcConfigToJSON output) back to a specific
// requester.  Single ESP-NOW packets carry one fragment each; the receiver
// reassembles by sid.
//
// Unicast (wcb->send) instead of broadcast because:
//   • WCBClient's send() path uses ETM — every packet gets an ACK and is
//     retried automatically if the ACK doesn't arrive.
//   • Broadcast had ~90% loss on a 38-fragment payload in real testing
//     (3/38 arrived).  Sessions silently timed out at 5 s, _configLoaded
//     never flipped true, and the user got the "no config loaded" warning
//     every time they tried to save.
//   • Only the requester needs to see the response; other listeners on the
//     network don't care about config dumps.
//
// MUST be called from the main loop, NOT from the WCB_Client receive
// callback — see the _pendingGetConfigSender comment.
inline void fragmentAndSend(uint8_t targetID, const String& json, uint16_t sid) {
  if (!wcb || targetID == 0) return;
  const size_t total = (json.length() + FRAG_CHUNK_BYTES - 1) / FRAG_CHUNK_BYTES;
  if (total == 0 || total > FRAG_MAX_PARTS) {
    Serial.printf("[RC] fragmentAndSend bailed: payload %u bytes needs %u fragments (max %u). "
                  "Use Direct USB for this size.\n",
                  (unsigned)json.length(), (unsigned)total, (unsigned)FRAG_MAX_PARTS);
    return;
  }
  Serial.printf("[RC] fragmentAndSend: %u bytes → %u fragments to W%u (sid=%u)\n",
                (unsigned)json.length(), (unsigned)total, (unsigned)targetID, (unsigned)sid);

  for (size_t i = 0; i < total; i++) {
    const size_t off = i * FRAG_CHUNK_BYTES;
    const size_t len = (off + FRAG_CHUNK_BYTES <= json.length())
                         ? FRAG_CHUNK_BYTES
                         : (json.length() - off);
    String slice = json.substring(off, off + len);

    // Build envelope using ArduinoJson so the `s` field gets properly
    // escaped (quotes, backslashes, control chars inside the slice).
    StaticJsonDocument<300> env;
    env["f"]   = (int)(i + 1);
    env["of"]  = (int)total;
    env["sid"] = sid;
    env["s"]   = slice;

    char buf[252];
    size_t n = serializeJson(env, buf, sizeof(buf));
    // 187-byte cap (not 252!): WCB_Client appends "|CRC%08X" (12 bytes)
    // when sending via _sendPacket() into a 200-byte structCommand[200]
    // buffer.  If envelope > 187, the CRC suffix gets snprintf-truncated
    // and the bridge silently drops the fragment with "missing CRC".
    // Keep this cap in sync with FRAG_CHUNK_BYTES — see comment above.
    constexpr size_t MAX_ENV_BYTES = 187;
    if (n == 0 || n > MAX_ENV_BYTES) {
      Serial.printf("[RC] fragmentAndSend: envelope %u > %u-byte cap "
                    "(would lose CRC suffix in WCB_Client) — reduce "
                    "FRAG_CHUNK_BYTES\n",
                    (unsigned)n, (unsigned)MAX_ENV_BYTES);
      return;
    }
    wcb->send(targetID, buf);
    // Inter-fragment pacing: empirically 40 ms saturated the ESP-NOW MAC
    // layer on bursts (we saw "[SEND CB] MAC-layer FAILED" / queue
    // overflow on the receiving WCB).  150 ms gives the MAC + ETM ACK
    // round-trip enough headroom and matches the config tool's
    // outbound-fragment cadence on the SET_CONFIG path.  A 40-fragment
    // CONFIG response now takes ~6 s — acceptable for a Load.
    delay(150);
    // Pump WCBClient's state machine so ACKs received during this delay
    // get processed and retry timers advance.  Without this the pending
    // table fills up and later fragments are sent without retry.
    if (wcb) wcb->update();
  }
}

// ── Timing ───────────────────────────────────────────────────────────────────
// Heartbeat at 0.5 Hz (2 s) — low-overhead "I'm alive" beacon driving the
// config tool / Wizard's online-RC list.  Channels at 5 Hz (200 ms) — fast
// enough for the live SBUS visualizer to feel responsive, slow enough that
// 24-channel JSON packets (~150 B) don't saturate ESP-NOW airtime.
constexpr uint32_t HB_INTERVAL_MS = 2000;
constexpr uint32_t CH_INTERVAL_MS = 200;

inline uint32_t _lastHb = 0;
inline uint32_t _lastCh = 0;

// ── Subscription gate for high-rate broadcasts ───────────────────────────────
// `rc_hb` is a low-cost (0.5 Hz) presence beacon that ALWAYS runs so the WCB
// Wizard's discovery feature can find this RC even when no config tool is
// open.  `rc_ch` is 10× the bandwidth (5 Hz × 128 B = ~640 B/s of ESP-NOW
// airtime) and only useful when a config tool is actively live-monitoring,
// so we gate it on "have we received an inbound JSON message from a WCB
// sender recently?".  Any inbound JSON (PING / TRIGGER / SET_MODE / fragment)
// counts as a subscription renewal; if 15 s passes with no inbound, we stop
// emitting rc_ch.  The config tool sends a periodic PING in Via WCB mode to
// keep the subscription alive during passive monitoring.
//
// Direct-USB-only setups never PING over WCB, so rc_ch broadcasts simply
// never start — no airtime wasted on networks where no one's listening.
inline uint32_t _lastWcbInbound = 0;
constexpr uint32_t WCB_SUBSCRIPTION_MS = 15000;   // rc_ch off after 15 s idle
inline bool _hasWcbSubscriber(uint32_t now) {
  return (_lastWcbInbound != 0) &&
         (now - _lastWcbInbound < WCB_SUBSCRIPTION_MS);
}

// ── Deferred SET_CONFIG queue (callback → main loop) ────────────────────────
// rcTelemetry::handle() runs in the WCB_Client receive-callback chain, which
// fires from the ESP-NOW WiFi-task context on Core 0.  rcConfigFromJSON()
// can be a multi-KB JSON parse and rcConfigSaveNVS() blocks 100+ ms doing
// a flash erase + write — running either inline blocks the WiFi task and
// trips Core 0's interrupt watchdog ("Guru Meditation Error: Core 0
// panic'ed (Interrupt wdt timeout on CPU0)").
//
// Pattern: handle() stashes the reassembled JSON + sender ID here, then
// tick() (called from loop()) picks it up and applies it on the main task
// where blocking flash I/O is safe.  ACK is also deferred to tick() so it
// reports the actual apply result, not an optimistic "we got it" pre-apply.
//
// We stash the WHOLE reassembled payload (not just the `data` field)
// because the StaticJsonDocument<512> in handle() can't parse multi-KB
// JSON — tick() does the parse with a properly-sized DynamicJsonDocument
// instead.
// ── Cross-core synchronization for the deferred queues ──────────────────────
// `handle()` runs in the WCB_Client receive-callback context (WiFi task
// on Core 0).  `tick()` runs in the main loop (Core 1).  Both read AND
// write `_pendingReassembled` (an Arduino String).  String assignment is
// multi-step (allocate buffer → set length → copy bytes), and an
// interleaved read between steps can yield a length pointing past the
// end of the buffer — `_applyReassembled` then parses garbage or crashes
// in deserializeJson.
//
// The mutex is acquired briefly on each side around the read/write of
// the slot.  Cost is microseconds; safety is absolute.  Initialized
// lazily by `init()` (called from RC-Controller.ino setup() before
// WCB_Client init brings the receive callback online).
inline SemaphoreHandle_t _pendingMutex = nullptr;

// Call from RC-Controller.ino setup() BEFORE wcb->begin() so the mutex
// is ready when the first ESP-NOW packet arrives.  Idempotent.
inline void init() {
  if (!_pendingMutex) _pendingMutex = xSemaphoreCreateMutex();
}

inline String  _pendingReassembled;          // full JSON of a reassembled fragmented payload
inline uint8_t _pendingReassembledSender = 0;

// ── Deferred GET_CONFIG response queue (callback → main loop) ────────────────
// Fragmenting + sending 30-40 packets back to the requester needs to happen
// on the main loop, NOT in the WiFi-task receive callback, because:
//
//   1. WCBClient's pending-ACK table fills up if we fire fragments faster
//      than ACKs can arrive (ACKs come in via the same callback we're
//      blocking, so they queue and the retry logic starves).
//   2. Even with bigger delays between fragments, blocking the WiFi task
//      for >100 ms invites the same interrupt watchdog that ate us on
//      SET_CONFIG.
//
// Pattern: handle() stashes the requester's senderID, tick() builds and
// fragments the response with unicast + ACK-tracked retries.
inline uint8_t _pendingGetConfigSender = 0;

// ── Deferred apply for reassembled fragmented payloads ──────────────────────
// Called from tick() (main loop context) when _pendingReassembled is set.
// Uses a DynamicJsonDocument sized to 2× the input length — enough for the
// expected SET_CONFIG / CONFIG envelopes without the StaticJsonDocument<512>
// truncation that breaks the inline-recursion path.  Safe to call
// rcConfigFromJSON + rcConfigSaveNVS here — flash I/O blocks the main task
// but the WiFi/ESP-NOW task on Core 0 stays unblocked.
inline void _applyReassembled(uint8_t senderID, const String& json) {
  // 2x heuristic: ArduinoJson docs need roughly the input size plus overhead
  // for the parsed-object tree.  2x is comfortable for our config shape and
  // still small enough to fit in heap on the S3.  Cap at 8 KB just in case.
  size_t cap = json.length() * 2;
  if (cap < 1024) cap = 1024;
  if (cap > 8192) cap = 8192;
  DynamicJsonDocument doc(cap);
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.printf("[RC] reassembled JSON parse failed: %s (size=%u cap=%u)\n",
                  err.c_str(), (unsigned)json.length(), (unsigned)cap);
    return;
  }
  const char* type = doc["type"] | "";
  if (!strcmp(type, "SET_CONFIG")) {
    if (!doc["data"].is<JsonObject>()) {
      Serial.println("[RC] reassembled SET_CONFIG missing 'data' object");
      return;
    }
    // ── Self-preservation: strip wcbNetwork.deviceId before applying ──
    // A SET_CONFIG arriving over the WCB transport must NEVER change
    // wcbNetwork.deviceId — doing so would change our broadcast sender
    // ID mid-session, the bridge's special-peer (slot 20) would stop
    // recognising us, and the config tool's `msg.id !== 20` filter
    // would silently drop every subsequent rc_hb / rc_ch / rc_trig.
    // Symptom: "SBUS stopped showing on the webpage after a Save."
    // The config tool's default `config.wcbNetwork.deviceId` is 3, so
    // any Save before a successful Load would otherwise demote us
    // from the special-peer slot.  We also strip macOct2/macOct3/
    // password/quantity for the same reason — those define which
    // network we live on; changing them mid-session is suicide.
    JsonObject data = doc["data"].as<JsonObject>();
    if (data["wcbNetwork"].is<JsonObject>()) {
      JsonObject wnet = data["wcbNetwork"].as<JsonObject>();
      if (wnet.containsKey("deviceId")) {
        Serial.printf("[RC] SET_CONFIG: ignoring incoming wcbNetwork.deviceId=%d "
                      "(WCB-transport saves can't change our own slot)\n",
                      (int)wnet["deviceId"]);
        wnet.remove("deviceId");
      }
      if (wnet.containsKey("macOct2"))  wnet.remove("macOct2");
      if (wnet.containsKey("macOct3"))  wnet.remove("macOct3");
      if (wnet.containsKey("password")) wnet.remove("password");
      if (wnet.containsKey("quantity")) wnet.remove("quantity");
      // If we stripped every field, drop the empty `wcbNetwork: {}` so
      // rcConfigFromJSON's containsKey("wcbNetwork") check short-circuits
      // and we skip the whole "apply each field from JSON with fallback"
      // loop.  Tiny perf win; avoids touching strlcpy(password) etc.
      // with the same values they already hold.
      if (wnet.size() == 0) data.remove("wcbNetwork");
    }
    // Pass the JsonObject directly instead of re-serializing → re-parsing.
    // The old String round-trip allocated 3 KB for `dataJson` AND another
    // ~6 KB inside rcConfigFromJSON(String) to re-parse, on top of the 6 KB
    // doc we're already holding here — heap fragmentation at that point
    // is a real crash risk on the ESP32-S3.  rcConfigFromJSON has a
    // JsonObject overload that reads `data` in-place from our parsed tree.
    Serial.printf("[RC] SET_CONFIG → applying via rcConfigFromJSON(JsonObject) "
                  "(heap=%u bytes free)\n", (unsigned)ESP.getFreeHeap());
    bool ok = rcConfigFromJSON(data);
    Serial.printf("[RC] SET_CONFIG → rcConfigFromJSON returned %s "
                  "(heap=%u bytes free)\n", ok ? "true" : "false",
                  (unsigned)ESP.getFreeHeap());
    if (ok) {
      rcConfigSaveNVS();
      Serial.println("[RC] SET_CONFIG → applied + saved to NVS");
    } else {
      Serial.println("[RC] SET_CONFIG → rcConfigFromJSON returned false");
    }
    char ack[100];
    snprintf(ack, sizeof(ack),
      "{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"id\":%u,\"ok\":%s}",
      rcConfig.wcbNetwork.deviceId, ok ? "true" : "false");
    if (wcb) wcb->send(senderID, ack);
    return;
  }
  Serial.printf("[RC] reassembled payload had unexpected type '%s' — dropping\n", type);
}

// ── Outbound: heartbeat + channel snapshot (periodic) ───────────────────────
// Called from loop().  Emits at most one HB and one CH packet per call,
// throttled to the constants above.  Setting _lastHb = 0 / _lastCh = 0
// forces an immediate emit on the next tick (used by PING handler).
inline void tick() {
  // Drain any pending reassembled fragment payload BEFORE the broadcast
  // work — applying SET_CONFIG blocks 100+ ms doing flash I/O, and we'd
  // rather skip one heartbeat than risk a watchdog by mixing flash and
  // ESP-NOW TX in the same tick.  Clearing the slot before the apply
  // means a new reassembly can start landing while the apply runs.
  // ── Drain pending reassembled payload (mutex-guarded) ─────────────────
  // We take the mutex, swap out the slot contents, release the mutex,
  // THEN call _applyReassembled outside the lock — the apply runs flash
  // I/O for 100+ ms and we don't want to block handle() (Core 0 WiFi
  // task) waiting on the lock for that long.
  String  drainedPayload;
  uint8_t drainedSender = 0;
  if (_pendingMutex && xSemaphoreTake(_pendingMutex, portMAX_DELAY) == pdTRUE) {
    if (_pendingReassembled.length() > 0) {
      drainedPayload            = _pendingReassembled;
      drainedSender             = _pendingReassembledSender;
      _pendingReassembled       = String();   // free heap before apply
      _pendingReassembledSender = 0;
    }
    xSemaphoreGive(_pendingMutex);
  }
  if (drainedPayload.length() > 0) {
    _applyReassembled(drainedSender, drainedPayload);
    return;   // skip heartbeat/channel emit this tick — next call resumes
  }

  // Drain a pending GET_CONFIG response — runs entirely on the main loop
  // so each wcb->send() can be followed by wcb->update() to process ACKs
  // and fire retries.  fragmentAndSend internally delays + pumps update;
  // total send time for a 40-fragment config is ~1.5 s, during which
  // heartbeat/channel emits skip (they resume on the next tick).
  if (_pendingGetConfigSender != 0) {
    uint8_t target = _pendingGetConfigSender;
    _pendingGetConfigSender = 0;
    String body = rcConfigToJSON();
    String wrapped;
    wrapped.reserve(body.length() + 40);
    wrapped += "{\"type\":\"CONFIG\",\"id\":";
    wrapped += rcConfig.wcbNetwork.deviceId;
    wrapped += ",\"data\":";
    wrapped += body;
    wrapped += "}";
    uint16_t sid = _nextOutSid++;
    if (_nextOutSid == 0) _nextOutSid = 1;
    fragmentAndSend(target, wrapped, sid);
    return;   // skip heartbeat/channel emit this tick
  }

  if (!wcb) return;
  const uint32_t now = millis();
  const uint8_t  id  = rcConfig.wcbNetwork.deviceId;

  if (now - _lastHb >= HB_INTERVAL_MS) {
    _lastHb = now;
    // SBUS health: surface what the LOCAL reader is seeing so the config
    // tool's SBUS RECEIVER block shows the true ~70-100 Hz frame rate
    // rather than our 5 Hz rc_ch broadcast cadence.  ageMs is the time
    // since the last SBUS frame arrived at the RC; the tool uses this to
    // distinguish "transmitter on, no signal" from "transmitter off".
    const unsigned long sbusAgeMs =
      (sbusLastFrameMs == 0) ? 99999UL : (now - sbusLastFrameMs);
    char buf[220];
    snprintf(buf, sizeof(buf),
      "{\"type\":\"rc_hb\",\"id\":%u,\"fw\":\"%s\",\"up\":%lu,"
       "\"mode\":%d,\"model\":%u,"
       "\"sbusFps\":%d,\"sbusAge\":%lu,\"sbusLost\":%s,\"sbusFail\":%s}",
      id, FW_VERSION,
      (unsigned long)(now / 1000UL),
      FunctionSwState,
      rcConfig.txModel,
      sbusFps,
      sbusAgeMs,
      lostFrameOld ? "true" : "false",
      sbusFailsafe ? "true" : "false");
    wcb->broadcast(buf);   // WCBClient's broadcast() — wcb->send(0,...) bails (target_wcb < 1 check)
  }

  // rc_ch is gated on subscription — see _hasWcbSubscriber comment above.
  // Without a recent inbound, we don't emit channel updates because they're
  // useless to anyone not actively monitoring (10× the cost of rc_hb).
  if (now - _lastCh >= CH_INTERVAL_MS && _hasWcbSubscriber(now)) {
    _lastCh = now;
    char buf[240];
    int  len = snprintf(buf, sizeof(buf),
      "{\"type\":\"rc_ch\",\"id\":%u,\"ch\":[", id);
    // Always emit 24 channels.  sbusValues[] is sized for the full set; if
    // the receiver hasn't detected that many yet the missing ones sit at
    // their last value (0 at boot) — harmless filler for the config tool's
    // visualizer, which only colours the channels its model claims.
    for (int i = 0; i < 24 && len < (int)sizeof(buf) - 8; i++) {
      len += snprintf(buf + len, sizeof(buf) - len, "%s%d",
        (i ? "," : ""), sbusValues[i]);
    }
    if (len + 2 < (int)sizeof(buf)) {
      buf[len++] = ']';
      buf[len++] = '}';
      buf[len]   = '\0';
    }
    wcb->broadcast(buf);
  }
}

// ── Outbound: event-driven emits ────────────────────────────────────────────
// Call from rcDispatch() right after argument validation so EVERY trigger
// — local matrix press, Web-Serial JSON TRIGGER, remote ESP-NOW TRIGGER —
// surfaces on the network.  The config tool's "Via WCB" mode treats this as
// "something fired, no matter who fired it".
inline void emitTrig(int mode, int btn, int tap) {
  if (!wcb) return;
  char buf[120];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"rc_trig\",\"id\":%u,\"mode\":%d,\"btn\":%d,\"tap\":%d}",
    rcConfig.wcbNetwork.deviceId, mode, btn, tap);
  wcb->broadcast(buf);
}

// Call from the FunctionSwState-update path whenever the active mode
// changes.  Lets the config tool update its mode indicator immediately
// instead of waiting up to 2 s for the next heartbeat.
inline void emitMode(int mode) {
  if (!wcb) return;
  char buf[80];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"rc_mode\",\"id\":%u,\"mode\":%d}",
    rcConfig.wcbNetwork.deviceId, mode);
  wcb->broadcast(buf);
}

// ── Inbound parser ──────────────────────────────────────────────────────────
// Called from onWCBCommand() for every command this RC's WCB stack delivers
// to the application (unicasts addressed to our deviceId + broadcasts).
// Returns true if the message was a JSON command directed at us and was
// dispatched (caller should NOT route it further); false if it's something
// the caller still needs to handle (e.g. another RC's heartbeat we want
// to ignore, or a non-JSON legacy WCB command intended for some other path).
inline bool handle(uint8_t senderID, const char* command) {
  if (!command || command[0] != '{') return false;

  // Parse the JSON payload.  Use a doc large enough to hold fragment
  // envelopes (`s` field carries up to FRAG_CHUNK_BYTES of escaped JSON)
  // plus the normal single-packet commands (PING / TRIGGER / SET_MODE).
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, command);
  if (err) {
    // Not valid JSON — silently ignore.  This is normal traffic (a legacy
    // WCB ;-command from a stage-cue device or another peer) and not an
    // error condition.
    return false;
  }

  // Any valid JSON inbound from a WCB peer counts as "someone is listening"
  // — renew the rc_ch subscription so high-rate channel broadcasts run for
  // the next WCB_SUBSCRIPTION_MS.  Done BEFORE the self-echo filter so a
  // remote PING / TRIGGER / SET_CONFIG fragment all count as activity.
  _lastWcbInbound = millis();

  // ── Fragment envelope ────────────────────────────────────────────────────
  // {"f":<n>,"of":<N>,"sid":<S>,"s":"<chunk>"} — slice of a larger JSON
  // payload.  Buffer by sid; when all N parts arrive, concatenate the `s`
  // fields in order and recurse with the reassembled JSON so the normal
  // dispatch handles it (SET_CONFIG, etc.).
  if (doc.containsKey("f") && doc.containsKey("of") && doc.containsKey("sid")) {
    int f     = doc["f"]   | 0;
    int total = doc["of"]  | 0;
    int sid   = doc["sid"] | 0;
    const char* s = doc["s"] | "";
    if (f < 1 || total < 1 || total > FRAG_MAX_PARTS || f > total || sid == 0) {
      return true;   // malformed envelope — drop silently
    }
    FragSession* sess = _findOrAllocSession((uint16_t)sid, (uint8_t)total, senderID);
    if (!sess) {
      Serial.println("[RC] Fragment pool exhausted — dropping");
      return true;
    }
    // Use the explicit received[] flag rather than parts[].length() so an
    // empty slice (legitimately possible if the sender ever produces one)
    // can't masquerade as "not yet received" on duplicate arrivals.
    bool isNewFragment = !sess->received[f - 1];
    if (isNewFragment) {
      sess->parts[f - 1]    = String(s);
      sess->received[f - 1] = true;
      sess->got++;
      sess->expireAt = millis() + FRAG_TIMEOUT_MS;
    }
    // Visibility — log every fragment receive so the user can see in the
    // RC's serial monitor whether a SET_CONFIG / GET_CONFIG handshake is
    // actually arriving over the WCB bridge.
    Serial.printf("[RC] frag sid=%u f=%d/%d got=%d%s\n",
                  (unsigned)sid, f, total, sess->got,
                  isNewFragment ? "" : " (dup)");
    if (sess->got >= sess->total) {
      // Reassemble — concatenate all parts in order.
      String full;
      full.reserve(sess->total * FRAG_CHUNK_BYTES);
      for (uint8_t i = 0; i < sess->total; i++) full += sess->parts[i];
      uint8_t fromId = sess->senderID;
      *sess = FragSession{};   // free the slot before stashing
      Serial.printf("[RC] frag sid=%u COMPLETE, deferring %u bytes to main loop\n",
                    (unsigned)sid, (unsigned)full.length());
      // ── DEFER to main loop ────────────────────────────────────────────
      // Do NOT recurse into handle() here.  We're in the WCB_Client
      // receive-callback chain (WiFi-task context on Core 0); doing the
      // 1–4 KB JSON parse + rcConfigFromJSON + rcConfigSaveNVS inline
      // blocks the WiFi task long enough to trip Core 0's interrupt
      // watchdog.  Stash the payload; tick() picks it up from loop()
      // where blocking flash I/O is safe.  If a second large message
      // arrives before the first one applies, we drop the new one —
      // SET_CONFIG isn't expected to fire faster than ~1 Hz.
      // Mutex-guarded write — see _pendingMutex declaration for why.
      // We assign the String inside the critical section so a concurrent
      // tick() can't observe a partial buffer.
      bool accepted = false;
      if (_pendingMutex && xSemaphoreTake(_pendingMutex, portMAX_DELAY) == pdTRUE) {
        if (_pendingReassembled.length() == 0) {
          _pendingReassembled       = full;
          _pendingReassembledSender = fromId;
          accepted = true;
        }
        xSemaphoreGive(_pendingMutex);
      }
      if (!accepted) {
        Serial.println("[RC] previous reassembled payload still pending — dropping new one");
      }
    }
    return true;
  }

  const char* type = doc["type"] | "";
  if (!type[0]) return false;

  // Filter out our own broadcasts echoing back — `id` field on outbound
  // telemetry equals our deviceId; if we see that, it's something WE sent.
  // (Optional defensive check — WCB stack usually filters self-echoes.)
  if (doc["id"].is<int>() &&
      (uint8_t)(doc["id"] | 0) == rcConfig.wcbNetwork.deviceId &&
      type[0] == 'r' && type[1] == 'c' && type[2] == '_') {
    return false;
  }

  // ── PING ─────────────────────────────────────────────────────────────────
  // Unicast a PONG back to the sender, then force an immediate heartbeat +
  // channel burst on the next tick so the asker gets a full state snapshot.
  if (!strcmp(type, "PING")) {
    char buf[180];
    snprintf(buf, sizeof(buf),
      "{\"type\":\"PONG\",\"id\":%u,\"version\":\"%s\",\"model\":%u,\"mode\":%d}",
      rcConfig.wcbNetwork.deviceId, FW_VERSION,
      rcConfig.txModel, FunctionSwState);
    wcb->send(senderID, buf);
    _lastHb = 0;   // force re-broadcast
    _lastCh = 0;
    return true;
  }

  // ── TRIGGER ──────────────────────────────────────────────────────────────
  // Same shape the config tool already sends over Web Serial.  Goes through
  // the SAME rcDispatch path as a local matrix press — so any
  // dispatch-side telemetry (emitTrig, dispatch-echo when that lands) fires
  // exactly once per remote trigger, indistinguishable from a local press.
  if (!strcmp(type, "TRIGGER")) {
    int mode = doc["mode"] | 0;
    int btn  = doc["btn"]  | 0;
    int tap  = doc["tap"]  | 1;
    if (mode >= 1 && mode <= 3 &&
        btn  >= 1 && btn  <= RC_NUM_THRESHOLDS) {
      if (tap < 1) tap = 1; else if (tap > 3) tap = 3;
      rcDispatch(mode * 100 + btn, (uint8_t)tap);
    }
    return true;
  }

  // ── SET_MODE ─────────────────────────────────────────────────────────────
  // Forces FunctionSwState immediately.  NOTE: the next SBUS frame's
  // mode-switch decode will overwrite this — so SET_MODE is most useful
  // when the user has the modeSwitch parked at a steady position, or
  // for transient testing.  Persistent mode override would need a separate
  // "muted modeSwitch" config flag — out of scope for Phase 1.
  if (!strcmp(type, "SET_MODE")) {
    int mode = doc["mode"] | 0;
    if (mode >= 1 && mode <= 3 && mode != FunctionSwState) {
      FunctionSwState = mode;
      emitMode(mode);
    }
    return true;
  }

  // ── GET_CONFIG ────────────────────────────────────────────────────────────
  // Defer the actual fragment+send to the main loop — see
  // _pendingGetConfigSender comment.  Calling fragmentAndSend from here
  // (WiFi-task callback context) blocks for ~1.5 s with delays + wcb->update
  // calls, which prevents the very ACKs the retries depend on from being
  // processed.  tick() will pick this up and ship it on the next iteration.
  if (!strcmp(type, "GET_CONFIG")) {
    if (_pendingGetConfigSender == 0) {
      _pendingGetConfigSender = senderID;
      Serial.printf("[RC] GET_CONFIG → queued for W%u, will send from main loop\n",
                    (unsigned)senderID);
    } else {
      Serial.println("[RC] GET_CONFIG dropped — previous response still sending");
    }
    return true;
  }

  // ── SET_CONFIG ────────────────────────────────────────────────────────────
  // Always defer to main loop — rcConfigFromJSON + rcConfigSaveNVS block
  // 100+ ms on flash I/O and we're in the WCB_Client receive-callback
  // chain (WiFi-task on Core 0).  Stash the WHOLE message (caller's JSON
  // string, not our 512-byte truncated parse) so tick() can re-parse with
  // a DynamicJsonDocument sized for the real payload.  This branch fires
  // for a single-packet SET_CONFIG too (rare but possible if the config
  // ever shrinks below the ~200 byte threshold).
  if (!strcmp(type, "SET_CONFIG")) {
    // Mutex-guarded — same reasoning as the fragment-complete branch.
    bool accepted = false;
    if (_pendingMutex && xSemaphoreTake(_pendingMutex, portMAX_DELAY) == pdTRUE) {
      if (_pendingReassembled.length() == 0) {
        _pendingReassembled       = String(command);
        _pendingReassembledSender = senderID;
        accepted = true;
      }
      xSemaphoreGive(_pendingMutex);
    }
    if (accepted) {
      Serial.println("[RC] SET_CONFIG → deferred to main loop");
    } else {
      Serial.println("[RC] SET_CONFIG dropped — apply queue full");
    }
    return true;
  }

  // ── Direct-USB-only commands that mean nothing over the WCB transport ──
  // The config tool fires these on every connect/poll cycle (live-monitor
  // start/stop, debug-flag bitmask, WCB peer status query, calibration
  // mute toggle).  Over USB they have local effects (gating PWM_UPDATE
  // emission, etc.).  Over WCB the equivalent live-state already streams
  // via rc_hb / rc_ch broadcasts, so just silently accept these so the
  // config tool's connect sequence runs cleanly without filling the
  // RC's serial log with "Unknown inbound type" noise on every PING.
  if (!strcmp(type, "START_MONITOR")  || !strcmp(type, "STOP_MONITOR") ||
      !strcmp(type, "SET_DEBUG_FLAGS") || !strcmp(type, "GET_WCB_STATUS") ||
      !strcmp(type, "CALIB")) {
    return true;
  }

  // ── REBOOT (JSON form — mirror of the ;<id>,r short-form above) ──────
  if (!strcmp(type, "REBOOT")) {
    Serial.println("[RC] Remote REBOOT requested via WCB");
    delay(100);
    ESP.restart();
    return true;
  }

  // Unknown JSON type — log and don't consume so the caller can keep
  // logging it for visibility.
  Serial.printf("[RC] Unknown inbound type '%s' from WCB%d\n", type, senderID);
  return false;
}

}   // namespace rcTelemetry
