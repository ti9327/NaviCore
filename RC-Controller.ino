// =============================================================================
//  RC-Controller.ino — WCB-based RC Controller
//  Target hardware: WCB HW 3.2 (ESP32-S3)
//
//  Features:
//    • SBUS input (16 or 24 channel, auto-detected) on Serial1 RX (GPIO5)
//    • SBUS output passthrough on GPIO9 (byte-streaming, ~110 µs latency) —
//      lets a downstream device consume the same SBUS stream
//    • Local Pololu Maestro on Serial2 @ 115200 baud (GPIO6 TX)
//    • Up to 8 remote Maestros via WCBStream broadcast over ESP-NOW
//    • WCB unicast and broadcast command dispatch
//    • Serial3, Serial4 available as general-purpose SoftwareSerial ports
//    • Multi-mode RC button/switch/knob mapping (NVS-backed, GUI-configurable)
//    • USB Serial JSON protocol for config tool and CLI debugging
//      (open config_tool/index.html on your PC, connect via Web Serial API)
//
//  USB Serial JSON Protocol (newline-delimited):
//    PING                        → {"type":"PONG"}
//    GET_CONFIG                  → {"type":"CONFIG","data":{...full config JSON...}}
//    {"type":"SET_CONFIG","data":{...}} → {"type":"ACK","ok":true}
//    {"type":"START_MONITOR"}    → streams PWM_UPDATE every 50 ms until STOP_MONITOR
//    {"type":"STOP_MONITOR"}     → {"type":"ACK","ok":true}
//    {"type":"RESET_DEFAULTS"}   → reloads factory defaults, replies ACK
//    {"type":"REBOOT"}           → ACKs then restarts the board after 250 ms
//    {"type":"TRIGGER","mode":1,"btn":3,"tap":1} → fires virtual button press
//    {"type":"WCB_SEND","target":2,"cmd":":PP100"} → manually fires WCB command
//
//  Required libraries (Library Manager):
//    • ArduinoJson        (Benoit Blanchon) v6.x
//    • Adafruit NeoPixel
//
//  Local libraries (in libraries/ folder):
//    • WCB_Client + WCBStream  (from the greghulette/WCBClient repo)
//    • PololuMaestro
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <WCB_Client.h>   // header in greghulette/WCBClient is WCB_Client.h
#include <WCBStream.h>
// HCR (Human Cyborg Relations Vocalizer): no library dependency — we format
// the same byte string the upstream HCRVocalizer would have written and push
// it directly to the bound aux-serial port.  See hcrFormatCommand() below.
#include "sbus_reader.h"
#include "rc_config.h"
#include "wcb_config.h"
#include "fw_version.h"     // FW_VERSION_BASE / FW_VERSION_DTG / FW_VERSION

// =============================================================================
//  WCB HW 3.2 — Pin Definitions
//  TX pins from wcb_pin_map.cpp v3.2 (matches SBUSController.ino comments).
//  RX pins: GPIO5 confirmed by user; others assumed — verify against schematic.
// =============================================================================
#define SBUS_RX_PIN      5    // Serial1 RX — SBUS from RC receiver (confirmed)
// SBUS OUT now routes through Serial1's hardware TX on SBUS_OUT_PIN below
// (no bit-banging).  Previously this firmware bit-banged SBUS OUT via a
// SoftwareSerial on GPIO9, which monopolised ~31% of one core (113 fps ×
// 25 bytes × 110µs spin per byte) and starved the IDLE0 task → stack-canary
// crashes after a few seconds of SBUS input.  Routing through Serial1's
// hardware UART TX drops the per-byte cost from ~110µs to ~1µs (FIFO push)
// and frees the core completely.  See 2026-05-22 crash discussion.

#define MAESTRO_TX_PIN   6    // Serial2 TX — local Maestro command bus
#define MAESTRO_RX_PIN   7    // Serial2 RX — optional Maestro feedback (verify pin)

#define S3_TX_PIN       15    // Aux serial S3 TX (SoftwareSerial)
#define S3_RX_PIN       16    // Aux serial S3 RX
#define S4_TX_PIN       17    // Aux serial S4 TX (SoftwareSerial)
#define S4_RX_PIN       18    // Aux serial S4 RX

// SBUS OUT — pure passthrough re-emission of the SBUS RX stream so downstream
// devices can consume the same channel data. Bit-banged via SoftwareSerial at
// 100k 8E2 inverted (TX-only); each byte is forwarded ~110 µs after it
// arrives on Serial1 RX. See SbusReader::setPassthroughSink() for the tee.
#define SBUS_OUT_PIN     9    // SoftwareSerial TX → downstream SBUS consumer

#define STATUS_LED_PIN  48    // WCB 3.2 onboard NeoPixel — verify against schematic
#define STATUS_LED_COUNT 1

// =============================================================================
//  WCB Client + Streams
//
//  Single shared broadcast stream:  All remote Maestro bytes go out on one
//  WCBStream targeting `broadcast` (target_wcb=0). The Kyber path delivers
//  them to every WCB on the network. Each receiving WCB with Kyber_Remote
//  forwards the raw bytes to its configured Maestro port — no per-slot
//  WCB/port routing needed on the sender.
//
//  No compile-time Maestro instances: Maestro slots (IDs 1-8) are configured
//  at RUNTIME via the GUI's Maestro Locations panel (rcConfig.maestros[]).
//  Each slot decides whether bytes go to Serial2 (local) or the broadcast
//  stream (remote), and which Pololu device # to embed.  See maestroWrite()
//  below for the dispatch.
// =============================================================================
// Heap-allocated in setup() AFTER NVS config loads, so the values come from
// rcConfig.wcbNetwork (GUI-editable) instead of the wcb_config.h #defines.
// The #defines are still the factory defaults on a fresh device — see
// rcConfigLoadDefaults() in rc_config.h.
WCB_Client* wcb = nullptr;

// One stream, broadcast to all WCBs.  target_port is ignored for broadcast.
//
// MUST be constructed AFTER `wcb` (in setup()): WCBStream's constructor
// self-registers with the WCB_Client singleton via WCB_Client::instance(), and
// that registration is what makes wcb->update() drive this stream's flush.
// If it were a global it would construct at static-init time — before the
// heap-allocated WCB_Client exists — and silently fail to register, so no
// Maestro bytes would ever leave the board over ESP-NOW.
WCBStream* maestroBroadcast = nullptr;

// =============================================================================
//  Aux serial ports S3, S4  +  dedicated SBUS OUT
//
//  ESP32-S3 has only 3 hardware UARTs (Serial / Serial1 / Serial2), all
//  claimed: USB-CDC, SBUS RX, local Maestro TX. Everything else is
//  bit-banged via SoftwareSerial. Aux ports stay reserved for ≤57600 baud
//  command lines; SBUS OUT runs at 100k 8E2 inverted TX-only.
// =============================================================================
SoftwareSerial Serial3;    // Aux command-line port, bound in setup()
SoftwareSerial Serial4;    // Aux command-line port, bound in setup()
// SBUS OUT no longer uses SoftwareSerial — it goes out Serial1's TX pin
// directly (configured in setup()).  See the SBUS_OUT_PIN comment above.

// =============================================================================
//  HCR Vocalizer routing
//
//  HCR command strings are the same regardless of transport (Serial or
//  WCB-ESP-NOW) — short ASCII frames like "<SH3,QEH,QT>\n".  hcrFormatCommand()
//  builds them; executeHcrAction() picks the destination (Serial3, Serial4,
//  or a remote WCB) and writes the formatted payload there.
//
//  We deliberately do NOT use the upstream HCRVocalizer library:
//    • Its begin() is the only place it touches Serial — and we never call it
//      (Serial3/4 baud comes from rcConfig.auxBaud via setup()'s begin call).
//    • Its update()/receive() are no-ops without setRefreshSpeed() — which
//      this sketch doesn't call.
//    • Its I2C transport path has known upstream bugs that don't compile on
//      modern ESP32 + GCC 14, and we never use I2C anyway.
//  Eliminating the dependency keeps CI builds clean and removes a third-party
//  library from the install chain.
//
//  S5 is reserved for SBUS OUT — not a valid HCR destination.
// =============================================================================

// =============================================================================
//  SBUS + RC Config
// =============================================================================
SbusReader sbusRx;
RcConfig   rcConfig;

// =============================================================================
//  Status LED
// =============================================================================
Adafruit_NeoPixel statusLed(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

void setStatusLed(uint32_t color, uint8_t brightness = 20) {
  statusLed.setBrightness(brightness);
  statusLed.setPixelColor(0, color);
  statusLed.show();
}

static const uint32_t C_RED    = 0xFF0000;
static const uint32_t C_GREEN  = 0x00FF00;
static const uint32_t C_BLUE   = 0x0000FF;
static const uint32_t C_YELLOW = 0xFFFF00;
static const uint32_t C_ORANGE = 0xFF8000;
static const uint32_t C_CYAN   = 0x00FFFF;
static const uint32_t C_OFF    = 0x000000;

// =============================================================================
//  SBUS state
// =============================================================================
int           sbusValues[24]        = {0};
unsigned long sbusFrameCount        = 0;
unsigned long sbusLastFrameMs       = 0;
unsigned long sbusFpsLastSecond     = 0;
unsigned long sbusFpsCounter        = 0;
int           sbusFps               = 0;
bool          sbusFailsafe          = false;
bool          lostFrameOld          = false;
bool          sbusLiveDump          = false;
unsigned long sbusLiveDumpLastMs    = 0;

int oldValueMatrix  = 100;   // raw matrix SBUS value — kept fresh for status/diagnostics
int oldValueMode    = 100;

// ── Matrix-button edge state machine ─────────────────────────────────────────
// Replaces the old raw-value-delta gate (abs(mxVal-oldValueMatrix) >= 5) that
// silently dropped fast re-presses of the same button (the value was identical
// across the two sampled SBUS frames so no "edge" was seen).
//
// Three-state debounced edge machine (NEUTRAL · TRANSITION · BUTTON):
//   • A press is COMMITTED only after the decoded button is stable in-band for
//     rcConfig.matrixDebounceFrames consecutive frames — rejects single-frame
//     noise and analog resistor-ladder sweep transients.
//   • A release/re-arm is COMMITTED only after NEUTRAL (decoded == 0) is stable
//     for matrixDebounceFrames consecutive frames — so a brief 1-frame neutral
//     dip mid-press (sweep slew, contact bounce) can no longer falsely re-arm
//     and split one press into a phantom double.
//   • Anything not yet stable (short neutral blips, unsettled band) is the
//     implicit TRANSITION state: it changes nothing — no fire, no re-arm.
// matrixDebounceFrames is runtime-configurable (Config modal): 1 = fastest
// (clean digital SBUS source), 2-4 for a noisy analog transmitter matrix.
// Only a true sub-frame tap (press+release inside one ~9-14 ms frame interval)
// is unrecoverable — a hard SBUS-protocol limit, not logic.
bool matrixArmed       = true; // true → a confirmed-neutral release has armed the next press
int  matrixCandidate   = 0;    // decoded button currently being debounced
int  matrixCandCount   = 0;    // consecutive frames matrixCandidate has held in-band
int  matrixNeutralCount = 0;   // consecutive NEUTRAL (decoded==0) frames — for release debounce

// =============================================================================
//  Mode state
// =============================================================================
int FunctionSwState = 1;   // 1=down, 2=mid, 3=up

// =============================================================================
//  Pending action queue
// =============================================================================
struct PendingAction {
  bool          active;
  unsigned long fireAt;
  RcAction      action;
};
#define PENDING_ACTION_SLOTS 8
PendingAction pendingActions[PENDING_ACTION_SLOTS];

// =============================================================================
//  USB Serial WebSerial monitor state
// =============================================================================
bool          wsMonitorActive   = false;
unsigned long wsMonitorLastSent = 0;
// While the config tool's calibration wizard is open the operator deliberately
// wiggles every stick/knob/switch/button. Suppress ALL action dispatch while
// this is set so nothing (HCR/Maestro/MP3/WCB/Serial) fires during cal.
// Set/cleared via {"type":"CALIB","on":bool}; also force-cleared by
// STOP_MONITOR and a fresh PING so a crashed/closed page can never leave the
// board permanently muted.
bool          calibrationActive = false;

// ── Debug-category bitmask ───────────────────────────────────────────────
// Set by the GUI via {"type":"SET_DEBUG_FLAGS","flags":N}. Default 0 = all
// [DISPATCH] logs silenced (no formatting, no USB-CDC bytes spent). Each
// dispatch log site uses dlog(BIT, ...) which is a no-op when the bit is
// off. Lets the config tool's terminal debug chips actually GATE the
// firmware's output instead of just hiding it client-side.
static uint32_t g_dbgFlags = 0;
#define DBG_MAESTRO    (1u << 0)
#define DBG_WCB        (1u << 1)   // covers both unicast and broadcast sends
#define DBG_HCR        (1u << 3)
#define DBG_MP3        (1u << 4)
#define DBG_SERIAL     (1u << 5)
// Category-gated log. ##__VA_ARGS__ swallows the trailing comma when only
// a fmt is passed. Wraps vlogf so it inherits the same non-blocking USB
// back-pressure handling (see vlogf() definition).
#define dlog(catBit, fmt, ...) do { if (g_dbgFlags & (catBit)) vlogf(fmt, ##__VA_ARGS__); } while (0)
#define WS_MONITOR_INTERVAL_MS  50

// =============================================================================
//  Tap detection state
// =============================================================================
struct TapState {
  int           lastBtn         = 0;   // most recent button that was tapped (sticky across release)
  uint8_t       tapCount        = 0;
  unsigned long lastTapMs       = 0;
  bool          deferredPending = false;
  unsigned long deferredFireAt  = 0;
  int           deferredBtn     = 0;
  uint8_t       deferredTaps    = 0;
};
TapState tapState;

// Last-seen switch positions for change detection
int switchPrevPos[RC_NUM_SWITCHES];

// =============================================================================
//  Helpers
// =============================================================================
// Decode SBUS value (172..1811 for FrSky -100%..+100%) into a switch
// position. 3-pos toggles cluster at ~172 / ~992 / ~1811, so the
// midpoints between min↔mid and mid↔max are the right thresholds:
//   (172+992)/2 = 582,  (992+1811)/2 = 1401.
// (Earlier 340/680 thresholds were for a 0-1023 range and mis-decoded
// the middle position as "up".)
static inline int readSwitchPos(int sbusVal, uint8_t positions) {
  if (positions == 2) return (sbusVal > 900) ? 2 : 0;
  if (sbusVal < 582)  return 0;
  if (sbusVal > 1401) return 2;
  return 1;
}

static inline int readBoundSwitchSbus(int8_t swIdx) {
  if (swIdx < 0 || swIdx >= RC_NUM_SWITCHES) return -1;
  int ch = rcConfig.switches[swIdx].channel;
  if (ch < 1 || ch > 24) return -1;
  return sbusValues[ch - 1];
}

int pwmToButton(int val) {
  for (int i = 0; i < RC_NUM_THRESHOLDS; i++) {
    if (val >= rcConfig.thresholds[i].minPwm && val <= rcConfig.thresholds[i].maxPwm)
      return i + 1;
  }
  return 0;
}

static inline uint16_t sbusToRange(int sbusVal, uint16_t outMin, uint16_t outMax) {
  long mapped = (long)(sbusVal - 172) * (outMax - outMin) / (1811 - 172) + outMin;
  if (mapped < (long)outMin) mapped = outMin;
  if (mapped > (long)outMax) mapped = outMax;
  return (uint16_t)mapped;
}

// =============================================================================
//  Maestro byte-level dispatcher
//
//  Writes a Pololu Maestro command to the destination stream for slot
//  `id` (1-8).  The slot's runtime location (type + device #) is looked up
//  in rcConfig.maestros[].
//
//    type = 1 → bytes go to Serial2 (local wired Maestro)
//    type = 2 → bytes go to the broadcast WCBStream (every WCB forwards)
//    type = 0 → slot disabled, command silently dropped
//
//  This firmware ALWAYS uses Pololu protocol — every Maestro on the network
//  is assumed to have its own device number (0-127), set in Maestro Control
//  Center. Compact protocol is not supported because in our broadcast model
//  multiple Maestros can share the same physical bus; the device-number
//  filter is what keeps them from all responding to every command.
//
//  Pololu protocol frame: 0xAA <device> <cmd_compact & 0x7F> <payload...>
// =============================================================================
static void maestroWrite(uint8_t id, uint8_t cmd_compact,
                         const uint8_t* payload, size_t plen) {
  if (id < 1 || id > RC_NUM_MAESTROS) return;
  const RcMaestroSlot& slot = rcConfig.maestros[id - 1];
  if (slot.type == 0) return;          // disabled
  if (slot.device > 127) return;       // invalid Pololu device #

  Stream* dest = (slot.type == 1) ? (Stream*)&Serial2 : (Stream*)maestroBroadcast;
  if (!dest) return;                    // remote slot but stream not yet up
  uint8_t hdr[3] = { 0xAA, slot.device, (uint8_t)(cmd_compact & 0x7F) };
  dest->write(hdr, 3);
  if (payload && plen) dest->write(payload, plen);
}

// Pololu Maestro command byte values (compact protocol).
//   0x84 SET_TARGET   · 0x87 SET_SPEED  · 0x89 SET_ACCEL
//   0xA2 GO_HOME      · 0xA4 STOP_SCRIPT · 0xA7 RESTART_SCRIPT_AT_SUB
static void maestroSetTarget(uint8_t id, uint8_t ch, uint16_t pos) {
  uint8_t p[3] = { ch, (uint8_t)(pos & 0x7F), (uint8_t)((pos >> 7) & 0x7F) };
  maestroWrite(id, 0x84, p, 3);
}
static void maestroSetSpeed(uint8_t id, uint8_t ch, uint16_t spd) {
  uint8_t p[3] = { ch, (uint8_t)(spd & 0x7F), (uint8_t)((spd >> 7) & 0x7F) };
  maestroWrite(id, 0x87, p, 3);
}
static void maestroSetAccel(uint8_t id, uint8_t ch, uint8_t accel) {
  uint8_t p[2] = { ch, accel };
  maestroWrite(id, 0x89, p, 2);
}
static void maestroGoHome(uint8_t id)        { maestroWrite(id, 0xA2, nullptr, 0); }
static void maestroStopScript(uint8_t id)    { maestroWrite(id, 0xA4, nullptr, 0); }
static void maestroRestartScript(uint8_t id, uint8_t sub) {
  maestroWrite(id, 0xA7, &sub, 1);
}

// Parse and execute a Maestro action command string against slot `id` (1-8).
// cmd: "setTarget,ch,pos" | "goHome" | "stopScript" | "restartScript,n"
//      | "setSpeed,ch,spd" | "setAccel,ch,acc"
static void executeMaestroCmd(uint8_t id, const char* cmd) {
  char buf[32];
  strlcpy(buf, cmd, sizeof(buf));
  char* tok = strtok(buf, ",");
  if (!tok) return;
  if      (strcmp(tok, "goHome")        == 0) maestroGoHome(id);
  else if (strcmp(tok, "stopScript")    == 0) maestroStopScript(id);
  else if (strcmp(tok, "setTarget")     == 0) {
    char* sCh = strtok(nullptr, ","); char* sPos = strtok(nullptr, ",");
    if (sCh && sPos) maestroSetTarget(id, (uint8_t)atoi(sCh), (uint16_t)atoi(sPos));
  }
  else if (strcmp(tok, "setSpeed")      == 0) {
    char* sCh = strtok(nullptr, ","); char* sSpd = strtok(nullptr, ",");
    if (sCh && sSpd) maestroSetSpeed(id, (uint8_t)atoi(sCh), (uint16_t)atoi(sSpd));
  }
  else if (strcmp(tok, "setAccel")      == 0) {
    char* sCh = strtok(nullptr, ","); char* sAcc = strtok(nullptr, ",");
    if (sCh && sAcc) maestroSetAccel(id, (uint8_t)atoi(sCh), (uint8_t)atoi(sAcc));
  }
  else if (strcmp(tok, "restartScript") == 0) {
    char* sN = strtok(nullptr, ",");
    maestroRestartScript(id, sN ? (uint8_t)atoi(sN) : 0);
  }
}

// =============================================================================
//  Serial port write helpers (S3, S4 only — S5 is now SBUS OUT)
// =============================================================================
void writeS3(const String& s) { for (char c : (s + '\r')) Serial3.write(c); }
void writeS4(const String& s) { for (char c : (s + '\r')) Serial4.write(c); }

// =============================================================================
//  HCR command formatter
//
//  Reimplements the byte-string format used by HCRVocalizer::sendCommand() so
//  we can ship the same payload through wcb.sendRaw() for WCB transport. Keep
//  this in sync with the HCR library if its protocol ever changes.
//
//  fn / chan / track follow the same convention as BC firmware's HCRFunction()
//  dispatcher:
//     2  SetEmotion(e, v)
//     3  Trigger(e, v)         (same payload as Stimulate)
//     4  Stimulate(e, v)
//     5  Overload()
//     6  Muse()
//     8  Stop all              (Stops V/A/B audio + emotes)
//     9  StopEmote()
//    11  ResetEmotions()
//    14  PlayWAV(ch, track)
//    16  StopWAV(ch)
//    17  SetVolume(ch, vol)
//
//  Returns an empty String for unknown fn or bad parameters.
// =============================================================================
static String hcrFormatCommand(uint8_t fn, int chan, int track) {
  static const char emoteprefix[] = "HSMC";   // HAPPY / SAD / MAD / sCared
  static const char audioprefix[] = "VAB";    // Vocalizer / A / B
  String inner;
  switch (fn) {
    case 2: {  // SetEmotion(e, v)
      if (chan < 0 || chan > 3 || track < 0 || track > 99) return "";
      inner = String("O") + emoteprefix[chan] + String(track) + ",QE" + emoteprefix[chan];
      break;
    }
    case 3:    // Trigger — same payload as Stimulate
    case 4: {  // Stimulate(e, v)
      if (chan < 0 || chan > 4 || track < 0 || track > 99) return "";
      if (chan == 4) {  // emotion 4 is Overload shortcut
        inner = "SE,QT";
      } else {
        inner = String("S") + emoteprefix[chan] + String(track) + ",QE" + emoteprefix[chan] + ",QT";
      }
      break;
    }
    case 5:  inner = "SE,QT";          break;  // Overload
    case 6:  inner = "MM";             break;  // single Muse
    case 8:  inner = "PSV,PSA,PSB,QT"; break;  // Stop (all audio + emote)
    case 9:  inner = "PSV,QT";         break;  // StopEmote
    case 11: inner = "OR,QE";          break;  // ResetEmotions
    case 14: {  // PlayWAV(ch, track) — file number is 0-padded to 4 digits
      if (chan < 0 || chan > 2 || track < 0 || track > 9999) return "";
      char file[8]; snprintf(file, sizeof(file), "%04d", track);
      inner = String("C") + audioprefix[chan] + file + ",QP" + audioprefix[chan];
      break;
    }
    case 16: {  // StopWAV(ch)
      if (chan < 0 || chan > 2) return "";
      inner = String("PS") + audioprefix[chan] + ",QP" + audioprefix[chan];
      break;
    }
    case 17: {  // SetVolume(ch, vol)
      if (chan < 0 || chan > 2 || track < 0 || track > 99) return "";
      inner = String("PV") + audioprefix[chan] + String(track);
      break;
    }
    default: return "";
  }
  return String("<") + inner + ">\n";
}

// ── Non-blocking hot-path logging ───────────────────────────────────────────
// loop() must NEVER stall on Serial. On WCB HW 3.2 the USB serial path
// back-pressures when no host is draining it, so a plain Serial.printf() on
// the per-action / per-flush hot path freezes the whole controller whenever
// the config tool isn't connected — heartbeats stop and the WCBs see this
// board go OFFLINE. It also caused the webpage Disconnect hang (firmware
// blocked mid-write while the browser stopped reading).
//
// vlogf() formats into a small stack buffer and writes ONLY if the TX buffer
// currently has room; otherwise the line is silently dropped. It therefore
// can never block, so standalone operation — and a mid-stream disconnect —
// can't freeze loop(). Use it for everything on the hot path; reserve plain
// Serial.* for one-shot setup output and JSON replies to host commands (a
// host is by definition present and draining when it sent the command).
static void vlogf(const char* fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof(buf)) n = sizeof(buf);
  if (Serial.availableForWrite() >= n) Serial.write((const uint8_t*)buf, (size_t)n);
}

// Dispatch an HCR action.
//
// The destination is GLOBAL — pulled from rcConfig.hcrDest rather than from
// the action itself. This lets every HCR action share one configured
// vocalizer wiring; the action only carries fn/chan/track.
static void executeHcrAction(const RcAction& a) {
  const RcHcrDest& dest = rcConfig.hcrDest;

  if (dest.transport == 1) {
    // ── WCB transport (raw forward via ESP-NOW) ────────────────────────────
    if (!wcb) { dlog(DBG_HCR, "[DISPATCH] HCR-WCB: wcb not ready — skipped\n"); return; }
    String payload = hcrFormatCommand(a.fn, a.chan, a.track);
    if (payload.length() == 0) {
      dlog(DBG_HCR, "[DISPATCH] HCR-WCB: bad fn=%u chan=%d track=%d — skipped\n",
            a.fn, a.chan, a.track);
      return;
    }
    // HCR over WCB is UNICAST ONLY. It must go through sendRaw() (targetID 97,
    // raw-serial forward) so the receiving WCB pipes the bytes verbatim to the
    // HCR device's serial port. Broadcast is intentionally unsupported: the
    // only broadcast primitives are the command path (which would parse the
    // HCR string as a WCB command and discard it) and Kyber (Maestro ports
    // only) — neither delivers to an arbitrary HCR serial port. An HCR
    // vocalizer is a single device at a known location, so unicast is correct.
    uint8_t target = (uint8_t)atoi(dest.target);
    if (target < 1 || target > WCB_MAX_BOARDS) {
      dlog(DBG_HCR, "[DISPATCH] HCR-WCB: target '%s' invalid — HCR over WCB must be a "
            "unicast WCB ID 1-%d (broadcast is not supported). Fix the HCR "
            "Destination in the config tool. Not sent.\n",
            dest.target, WCB_MAX_BOARDS);
      return;
    }
    if (dest.wcbPort < 1 || dest.wcbPort > 5) {
      // sendRaw() silently rejects a port outside 1-5 — surface it so a
      // misconfigured HCR Destination doesn't fail invisibly.
      dlog(DBG_HCR, "[DISPATCH] HCR-WCB: invalid port %u (must be 1-5) — "
            "check HCR Destination in the config tool, not sent\n",
            dest.wcbPort);
      return;
    }
    dlog(DBG_HCR, "[DISPATCH] HCR→WCB%u:port%u  %s\n",
          target, dest.wcbPort, payload.c_str());
    bool ok = wcb->sendRaw(target, dest.wcbPort,
                           (const uint8_t*)payload.c_str(), payload.length());
    dlog(DBG_HCR, "[DISPATCH] HCR-WCB sendRaw(WCB%u, port%u, %u bytes) %s\n",
          target, dest.wcbPort, payload.length(), ok ? "OK" : "FAIL");
    return;
  }

  // ── Local serial transport ───────────────────────────────────────────────
  // We format the same byte string the HCRVocalizer library would have
  // written and push it directly to the bound aux-serial port.  No library
  // dependency required — see hcrFormatCommand() for the formatter that
  // mirrors HCRVocalizer's protocol.  S5 is reserved for SBUS OUT and is
  // not a valid HCR destination; legacy configs pointing at S5 fall through
  // to the "unknown port" log below.
  Stream* hcrSerial = nullptr;
  if      (!strcmp(dest.target, "S3")) hcrSerial = &Serial3;
  else if (!strcmp(dest.target, "S4")) hcrSerial = &Serial4;
  if (!hcrSerial) {
    dlog(DBG_HCR, "[DISPATCH] HCR: unknown serial port '%s' — skipped\n", dest.target);
    return;
  }

  String payload = hcrFormatCommand(a.fn, a.chan, a.track);
  if (payload.length() == 0) {
    dlog(DBG_HCR, "[DISPATCH] HCR-Serial: bad/unsupported fn=%u chan=%d track=%d — skipped\n",
          a.fn, a.chan, a.track);
    return;
  }
  dlog(DBG_HCR, "[DISPATCH] HCR→%s  fn=%u chan=%d track=%d  %s",
        dest.target, a.fn, a.chan, a.track, payload.c_str());
  hcrSerial->print(payload);
}

// Build the WCB MP3 command string for an RA_MP3 action. Mirrors the WCB's
// ";A,<CMD>" set (WCB_MP3.cpp). Returns "" for an unknown function code.
// No trailing newline: this travels as a WCB command (the WCB strips the ';'
// and routes 'A...' to its MP3 driver) — not a raw serial forward.
static String mp3FormatCommand(uint8_t fn, int16_t arg) {
  switch (fn) {
    case MP3_PLAY:   return String(";A,PLAY,")   + arg;   // track 1-255
    case MP3_PLAYFS: return String(";A,PLAYFS,") + arg;   // index 0-255
    case MP3_STOP:   return ";A,STOP";
    case MP3_NEXT:   return ";A,NEXT";
    case MP3_PREV:   return ";A,PREV";
    case MP3_VOL:    return String(";A,VOL,")    + arg;   // 0=loudest..64=inaudible
    case MP3_VOLUP:  return ";A,VOLUP";
    case MP3_VOLDN:  return ";A,VOLDN";
    default:         return "";
  }
}

// ── Local MP3 Trigger (v2.x) serial driver ──────────────────────────────────
// Used when rcConfig.mp3Dest.transport == 0 (MP3 Trigger wired to this board's
// S3/S4). Mirrors the WCB's WCB_MP3.cpp byte protocol exactly:
//   play track   : 'v' <vol>  then 't' <track>
//   play file idx : 'v' <vol>  then 'p' <idx>
//   stop toggle  : 'O'        next: 'F'   prev: 'R'
//   set volume   : 'v' <vol>  (0=loudest .. 64=inaudible)
// The pre-play volume byte is why we keep a local volume shadow, just like the
// WCB driver (mp3Volume there).
static uint8_t mp3LocalVolume = 20;

static inline void mp3Raw(Stream* p, uint8_t b1, int16_t b2 = -1) {
  p->write(b1);
  if (b2 >= 0) p->write((uint8_t)b2);
  p->flush();
}

static void mp3SendLocal(Stream* p, const char* portName, uint8_t fn, int16_t arg) {
  switch (fn) {
    case MP3_PLAY:
      if (arg < 1 || arg > 255) { dlog(DBG_MP3, "[DISPATCH] MP3-local: track %d out of 1-255 — skipped\n", arg); return; }
      mp3Raw(p, 'v', mp3LocalVolume); mp3Raw(p, 't', arg); break;
    case MP3_PLAYFS:
      if (arg < 0 || arg > 255) { dlog(DBG_MP3, "[DISPATCH] MP3-local: index %d out of 0-255 — skipped\n", arg); return; }
      mp3Raw(p, 'v', mp3LocalVolume); mp3Raw(p, 'p', arg); break;
    case MP3_STOP:  mp3Raw(p, 'O'); break;
    case MP3_NEXT:  mp3Raw(p, 'F'); break;
    case MP3_PREV:  mp3Raw(p, 'R'); break;
    case MP3_VOL:
      if (arg < 0 || arg > 64) { dlog(DBG_MP3, "[DISPATCH] MP3-local: vol %d out of 0-64 — skipped\n", arg); return; }
      mp3LocalVolume = (uint8_t)arg; mp3Raw(p, 'v', mp3LocalVolume); break;
    case MP3_VOLUP:
      mp3LocalVolume = (mp3LocalVolume <= 5) ? 0 : mp3LocalVolume - 5;
      mp3Raw(p, 'v', mp3LocalVolume); break;
    case MP3_VOLDN:
      mp3LocalVolume = (mp3LocalVolume >= 59) ? 64 : mp3LocalVolume + 5;
      mp3Raw(p, 'v', mp3LocalVolume); break;
    default:
      dlog(DBG_MP3, "[DISPATCH] MP3-local: bad fn=%u — skipped\n", fn); return;
  }
  dlog(DBG_MP3, "[DISPATCH] MP3→%s  fn=%u arg=%d vol=%u  OK\n", portName, fn, arg, mp3LocalVolume);
}

// Dispatch an RA_MP3 action. Destination is GLOBAL (rcConfig.mp3Dest).
//   transport 0 = local serial (S3/S4) — drive the MP3 Trigger directly here.
//   transport 1 = WCB unicast — ";A,..." command to one WCB whose own MP3
//                 driver (configured there via ?MP3,S<port>) does the serial.
static void executeMp3Action(const RcAction& a) {
  const RcMp3Dest& dest = rcConfig.mp3Dest;

  if (dest.transport == 0) {
    // ── Local serial transport ───────────────────────────────────────────
    Stream* p = nullptr;
    if      (!strcmp(dest.target, "S3")) p = &Serial3;
    else if (!strcmp(dest.target, "S4")) p = &Serial4;
    if (!p) {
      dlog(DBG_MP3, "[DISPATCH] MP3-local: unknown serial port '%s' — skipped\n", dest.target);
      return;
    }
    mp3SendLocal(p, dest.target, a.fn, a.track);
    return;
  }

  // ── WCB unicast transport ──────────────────────────────────────────────
  if (!wcb) { dlog(DBG_MP3, "[DISPATCH] MP3: wcb not ready — skipped\n"); return; }
  String cmd = mp3FormatCommand(a.fn, a.track);
  if (cmd.length() == 0) {
    dlog(DBG_MP3, "[DISPATCH] MP3: bad fn=%u — skipped\n", a.fn);
    return;
  }
  uint8_t target = (uint8_t)atoi(dest.target);
  if (target < 1 || target > WCB_MAX_BOARDS) {
    dlog(DBG_MP3, "[DISPATCH] MP3: target '%s' invalid — set MP3 Destination to a "
          "WCB ID 1-%d in the config tool. Not sent.\n",
          dest.target, WCB_MAX_BOARDS);
    return;
  }
  bool ok = wcb->send(target, cmd.c_str());
  dlog(DBG_MP3, "[DISPATCH] MP3→WCB%u  %s  %s\n", target, cmd.c_str(), ok ? "OK" : "FAIL");
}

// =============================================================================
//  rcExecuteAction — single-action dispatcher
// =============================================================================
static void scheduleAction(const RcAction& action, unsigned long delayMs) {
  for (int i = 0; i < PENDING_ACTION_SLOTS; i++) {
    if (!pendingActions[i].active) {
      pendingActions[i] = { true, millis() + delayMs, action };
      return;
    }
  }
  vlogf("WARN: pendingActions full — executing immediately\n");
}

void rcExecuteAction(const RcAction& a) {
  // Calibration mode: the operator is intentionally moving every control to
  // set thresholds — drop every action (including delayed/scheduled ones, so
  // nothing queues up to fire the instant calibration ends).
  if (calibrationActive) return;
  if (a.delayMs > 0) { scheduleAction(a, a.delayMs); return; }

  switch (a.type) {
    case RA_WCB_UNICAST: {
      uint8_t boardId = (uint8_t)atoi(a.target);
      if (boardId >= 1 && boardId <= WCB_MAX_BOARDS) {
        dlog(DBG_WCB, "[DISPATCH] WCB→%d  %s\n", boardId, a.cmd);
        wcb->send(boardId, a.cmd);
      }
      break;
    }
    case RA_WCB_BROADCAST:
      dlog(DBG_WCB, "[DISPATCH] WCB broadcast  %s\n", a.cmd);
      wcb->broadcast(a.cmd);
      break;

    case RA_MAESTRO_LOCAL:
      // Legacy "local Maestro" — treat as Maestro ID 1 for backward compat
      // with old configs.  The location of Maestro 1 (and whether it's
      // actually wired locally) is now defined in the Maestro Locations panel.
      dlog(DBG_MAESTRO, "[DISPATCH] Maestro (legacy local → ID 1)  %s\n", a.cmd);
      executeMaestroCmd(1, a.cmd);
      break;

    case RA_MAESTRO_REMOTE: {
      // RA_MAESTRO_REMOTE is now the unified "Maestro" action — target holds
      // the Maestro slot ID (1-8). Wiring per ID is in rcConfig.maestros[].
      int id = atoi(a.target);
      if (id < 1 || id > RC_NUM_MAESTROS) {
        vlogf("WARN: Maestro action with invalid ID %d (target='%s')\n", id, a.target);
        break;
      }
      dlog(DBG_MAESTRO, "[DISPATCH] Maestro %d  %s\n", id, a.cmd);
      executeMaestroCmd((uint8_t)id, a.cmd);
      break;
    }
    case RA_SERIAL: {
      String s(a.cmd);
      dlog(DBG_SERIAL, "[DISPATCH] Serial %s  %s\n", a.target, a.cmd);
      if      (!strcmp(a.target, "S3")) writeS3(s);
      else if (!strcmp(a.target, "S4")) writeS4(s);
      // S5 is reserved for SBUS OUT — actions targeting S5 are silently
      // ignored (legacy configs may still reference it).
      break;
    }
    case RA_HCR:
      executeHcrAction(a);
      break;
    case RA_MP3:
      executeMp3Action(a);
      break;
    default: break;
  }
}

void checkPendingActions() {
  unsigned long now = millis();
  for (int i = 0; i < PENDING_ACTION_SLOTS; i++) {
    if (pendingActions[i].active && now >= pendingActions[i].fireAt) {
      pendingActions[i].active = false;
      rcExecuteAction(pendingActions[i].action);
    }
  }
}

// =============================================================================
//  RC dispatch by buttonId + tap count
// =============================================================================
void rcDispatch(int buttonId, uint8_t tapCount) {
  int mode = buttonId / 100, btn = buttonId % 100;
  if (mode < 1 || mode > 3 || btn < 1 || btn > 19) return;
  if (tapCount < 1 || tapCount > 3) return;
  const RcMapping& mapping = rcConfig.mappings[rcMapIndex(mode, btn)];
  if (mapping.exclusive) {
    // Exclusive: only the matched tier fires (e.g. double-tap fires t2 alone).
    const RcTier& tier = mapping.t[tapCount - 1];
    for (int i = 0; i < tier.count; i++) rcExecuteAction(tier.a[i]);
  } else {
    // Non-exclusive: cumulative — every tier up to and including the matched
    // one fires (e.g. double-tap fires t1 then t2; triple-tap fires t1+t2+t3).
    for (int ti = 0; ti < tapCount; ti++) {
      const RcTier& tier = mapping.t[ti];
      for (int i = 0; i < tier.count; i++) rcExecuteAction(tier.a[i]);
    }
  }
}

// =============================================================================
//  Matrix button tap detection
// =============================================================================
// Called exactly ONCE per discrete, debounced button press by processSbus()
// (the matrixArmed / neutral-frame logic there guarantees one invocation per
// press and a release between presses). So every call here is a genuine new
// press — there is no need to suppress "held" frames internally, and doing so
// (the old prevPollBtn edge check) wrongly dropped a second press of the SAME
// button when no other button was pressed in between.
void RCRadio_Matrix_Buttons(int val) {
  int btn = pwmToButton(val);
  if (btn == 0) return;                 // caller only calls for a real button; defensive
  unsigned long now = millis();

  if (btn != tapState.lastBtn) {
    // Different button than the last gesture — commit any pending fire for the
    // previous button now, then start a fresh single tap on this one.
    if (tapState.deferredPending) {
      tapState.deferredPending = false;
      rcDispatch(tapState.deferredBtn, tapState.deferredTaps);
    }
    tapState.lastBtn   = btn;
    tapState.tapCount  = 1;
    tapState.lastTapMs = now;
  } else {
    // Same button as the last gesture — within the tap window it's another tap
    // of a multi-tap; past the window it's a brand-new single tap.
    if ((now - tapState.lastTapMs) < (unsigned long)rcConfig.tapWindowMs) {
      tapState.tapCount++;
      if (tapState.tapCount > 3) tapState.tapCount = 3;
    } else {
      tapState.tapCount = 1;
    }
    tapState.lastTapMs = now;
  }

  // Arm (or refresh) the deferred fire — both exclusive and non-exclusive modes
  // wait the tap window before dispatching so multi-taps have time to register.
  tapState.deferredPending = true;
  tapState.deferredFireAt  = now + rcConfig.tapWindowMs;
  tapState.deferredBtn     = FunctionSwState * 100 + btn;
  tapState.deferredTaps    = tapState.tapCount;
}

void checkDeferredTap() {
  if (!tapState.deferredPending) return;
  if (millis() >= tapState.deferredFireAt) {
    tapState.deferredPending = false;
    rcDispatch(tapState.deferredBtn, tapState.deferredTaps);
    tapState.tapCount = 0;
    tapState.lastBtn  = 0;
  }
}

// =============================================================================
//  Switch processing
// =============================================================================
void processSwitches() {
  for (int i = 0; i < RC_NUM_SWITCHES; i++) {
    RcSwitch& sw = rcConfig.switches[i];
    if (sw.channel < 1 || sw.channel > 24) continue;
    int pos = readSwitchPos(sbusValues[sw.channel - 1], sw.positions);
    if (pos == switchPrevPos[i]) continue;
    switchPrevPos[i] = pos;
    RcTier& tier = sw.t[pos];
    for (int ai = 0; ai < tier.count; ai++) rcExecuteAction(tier.a[ai]);
  }
}

// =============================================================================
//  Knob processing
//
//  Each knob/joystick-axis samples its SBUS channel once and fans the value
//  out to every configured output. The dispatch path is chosen by the knob's
//  function:
//
//    KF_MAESTRO_PASSTHROUGH  → each output.target is a Maestro ID (1-8) and
//                              maestroCh/posMin/posMax describe the servo.
//    KF_HCR_VOLUME           → each output.target is an HCR audio chan
//                              (0=V, 1=A, 2=B); posMin/posMax are volume
//                              endpoints (0-99). Rate-limited to avoid
//                              saturating ESP-NOW / HCR serial input.
//
//  ⚠ At 8 sources × 8 outputs × ~70Hz SBUS rate this could emit up to 4480
//  Maestro.setTarget() calls/sec. Keep active outputs reasonable.
// =============================================================================

// HCR volume rate-limiter — only re-emit when value changes AND a minimum
// interval has elapsed.  One slot per audio channel (V/A/B).
struct HcrVolCache {
  int8_t        lastVol[3] = { -1, -1, -1 };
  unsigned long lastSent[3] = { 0, 0, 0 };
};
static HcrVolCache hcrVolCache;
static const unsigned long HCR_VOLUME_MIN_INTERVAL_MS = 80;  // ~12 Hz max per chan

static void dispatchHcrVolume(uint8_t audioChan, uint8_t vol) {
  if (audioChan > 2) return;
  if (vol > 99) vol = 99;
  if ((int8_t)vol == hcrVolCache.lastVol[audioChan]) return;
  unsigned long now = millis();
  if (now - hcrVolCache.lastSent[audioChan] < HCR_VOLUME_MIN_INTERVAL_MS) return;
  hcrVolCache.lastVol[audioChan]  = vol;
  hcrVolCache.lastSent[audioChan] = now;
  // Build a synthetic RA_HCR action and reuse the existing dispatcher so
  // it automatically honors rcConfig.hcrDest (serial vs WCB transport).
  RcAction a{};
  a.type  = RA_HCR;
  a.fn    = 17;        // SetVolume
  a.chan  = (int8_t)audioChan;
  a.track = (int16_t)vol;
  executeHcrAction(a);
}

// Per-knob last-processed raw SBUS value. Knobs only re-dispatch when their
// channel moves past KNOB_CHANGE_DEADBAND counts — otherwise a stationary
// stick/knob would spam a Maestro/HCR command on every SBUS frame (~70+/s),
// and SBUS line jitter (±1-2 counts) would do the same. Matches the ≥5
// deadband the matrix/mode selectors already use. Sentinel 0xFFFF forces the
// first frame through so the initial position is always sent.
#define KNOB_CHANGE_DEADBAND 5
static uint16_t lastKnobRaw[RC_NUM_KNOBS] = {
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

void processKnobs() {
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    RcKnob& kn = rcConfig.knobs[i];
    if (kn.channel < 1 || kn.channel > 24) continue;
    if (kn.function == KF_NONE || kn.outputCount == 0) continue;
    uint16_t raw = sbusValues[kn.channel - 1];

    // Only dispatch when the source actually moved (or on the first frame).
    if (abs((int)raw - (int)lastKnobRaw[i]) < KNOB_CHANGE_DEADBAND) continue;
    lastKnobRaw[i] = raw;

    for (uint8_t o = 0; o < kn.outputCount && o < RC_KNOB_MAX_OUTPUTS; o++) {
      const RcKnobOutput& out = kn.outputs[o];
      uint16_t mapped = sbusToRange(raw, out.posMin, out.posMax);
      if (kn.function == KF_MAESTRO_PASSTHROUGH) {
        // out.target is the Maestro slot ID (1-8)
        maestroSetTarget(out.target, out.maestroCh, mapped);
      } else if (kn.function == KF_HCR_VOLUME) {
        // out.target is the HCR audio chan (0/1/2); mapped is volume 0-99
        dispatchHcrVolume(out.target, (uint8_t)mapped);
      }
    }
  }
}

// =============================================================================
//  SBUS frame processing
// =============================================================================
void processSbus() {
  if (!sbusRx.read()) return;

  sbusFrameCount++;
  sbusLastFrameMs = millis();
  sbusFpsCounter++;
  sbusFailsafe  = sbusRx.failsafe;
  lostFrameOld  = sbusRx.lostFrame;
  for (int i = 0; i < 24; i++) sbusValues[i] = sbusRx.channels[i];

  // Mode selector — same SBUS-cluster thresholds as readSwitchPos()
  // (582/1401) so the bound mode switch decodes its three positions
  // correctly. Earlier 340/680 incorrectly mapped middle (~992) → mode 3.
  int modeVal = readBoundSwitchSbus(rcConfig.funcBindings.modeSwitch);
  if (modeVal >= 0 && abs(modeVal - oldValueMode) > 5) {
    oldValueMode = modeVal;
    if      (modeVal < 582)  FunctionSwState = 1;
    else if (modeVal < 1401) FunctionSwState = 2;
    else                     FunctionSwState = 3;
  }

  // Button matrix — edge-detected with an asymmetric debounce (see
  // matrixArmed/Candidate state above). Decodes EVERY frame instead of
  // gating on raw value delta, so a fast re-press of the same button is no
  // longer dropped.
  int mxCh = rcConfig.matrixChannel;
  if (mxCh >= 1 && mxCh <= 24) {
    int mxVal = sbusValues[mxCh - 1];
    oldValueMatrix = mxVal;                 // keep fresh for status/diagnostics
    int decoded = pwmToButton(mxVal);       // 0 = neutral / between-band gap
    int debFrames = rcConfig.matrixDebounceFrames;
    if (debFrames < 1) debFrames = 1;            // safety clamp (config is 1-4)

    if (decoded == 0) {
      // NEUTRAL candidate. A real release sits at rest for many frames; a
      // sweep slew / contact bounce only dips out-of-band for 1-2 frames.
      // Only a *debounced* neutral run counts as a release → re-arm. This is
      // the key reliability fix: a transient neutral can no longer split one
      // physical press into a phantom double.
      matrixCandidate = 0;
      matrixCandCount = 0;
      if (matrixNeutralCount < debFrames) matrixNeutralCount++;
      if (matrixNeutralCount >= debFrames) matrixArmed = true;   // release confirmed
    } else {
      // BUTTON candidate. Any in-band reading breaks a neutral run, so a
      // mid-press sweep transient that briefly crosses a neighbor band does
      // not accumulate toward a release.
      matrixNeutralCount = 0;
      if (decoded == matrixCandidate) {
        if (matrixCandCount < debFrames) matrixCandCount++;
      } else {
        matrixCandidate = decoded;
        matrixCandCount = 1;
      }
      // Fire once the button has been stable in-band AND we're armed
      // (armed only by a confirmed neutral — never by a 1-frame blip).
      if (matrixArmed && matrixCandCount >= debFrames) {
        matrixArmed = false;             // consume — needs a CONFIRMED neutral to re-arm
        RCRadio_Matrix_Buttons(mxVal);
      }
    }
  }

  processSwitches();
  processKnobs();
}

// =============================================================================
//  SBUS FPS tracking
// =============================================================================
void trackSbusFps() {
  unsigned long now = millis();
  if (now - sbusFpsLastSecond >= 1000) {
    sbusFps           = (int)sbusFpsCounter;
    sbusFpsCounter    = 0;
    sbusFpsLastSecond = now;
  }
}

// =============================================================================
//  SBUS state dump (#L09)
// =============================================================================
void dumpSbusState() {
  unsigned long ageMs = (sbusLastFrameMs == 0) ? 0 : (millis() - sbusLastFrameMs);
  Serial.println("---- SBUS STATE ----");
  Serial.printf("  variant=%s (%d ch, %d-byte frame)\n",
                sbusRx.detectedChCount == 24 ? "SBUS-24" :
                sbusRx.detectedChCount == 16 ? "SBUS-16" : "(none yet)",
                sbusRx.detectedChCount, sbusRx.detectedFrameLen);
  Serial.printf("  frames=%lu  fps=%d  ageMs=%lu  lost=%s  failsafe=%s\n",
                sbusFrameCount, sbusFps, ageMs,
                lostFrameOld ? "YES" : "no", sbusFailsafe ? "YES" : "no");
  for (int r = 0; r < 3; r++) {
    int base = r * 8;
    if (base >= sbusRx.detectedChCount) break;
    Serial.printf("  CH%d-%d: ", base + 1, base + 8);
    for (int i = base; i < base + 8 && i < 24; i++)
      Serial.printf("%4d ", sbusValues[i]);
    Serial.println();
  }
}

// =============================================================================
//  WCB receive callback
// =============================================================================
void onWCBCommand(uint8_t senderID, const char* command) {
  Serial.printf("[WCB RX] from WCB%d: %s\n", senderID, command);
  // Extend here to route inbound WCB commands to local actions if needed
}

// =============================================================================
//  PWM monitor — streams to USB Serial while wsMonitorActive
// =============================================================================
void sendPWMUpdate() {
  if (!wsMonitorActive) return;
  unsigned long now = millis();
  if (now - wsMonitorLastSent < WS_MONITOR_INTERVAL_MS) return;
  wsMonitorLastSent = now;

  unsigned long ageMs = (sbusLastFrameMs == 0) ? 99999 : (now - sbusLastFrameMs);
  bool sbusOk = (sbusFps > 0) && !lostFrameOld && (ageMs < 500);

  char channelBuf[300] = "";
  int chCount = sbusRx.detectedChCount > 24 ? 24 : sbusRx.detectedChCount;
  for (int i = 0; i < chCount; i++) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%d%s", sbusValues[i], (i + 1 < chCount) ? "," : "");
    strncat(channelBuf, tmp, sizeof(channelBuf) - strlen(channelBuf) - 1);
  }

  int modeCh = 0;
  if (rcConfig.funcBindings.modeSwitch >= 0 &&
      rcConfig.funcBindings.modeSwitch < RC_NUM_SWITCHES)
    modeCh = rcConfig.switches[rcConfig.funcBindings.modeSwitch].channel;

  char buf[640];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"PWM_UPDATE\",\"matrixCh\":%d,\"modeCh\":%d,\"matrixVal\":%d,"
    "\"modeVal\":%d,\"btn\":%d,\"mode\":%d,"
    "\"sbus\":{\"ok\":%s,\"fps\":%d,\"frames\":%lu,\"ageMs\":%lu,"
    "\"lost\":%s,\"failsafe\":%s,\"chCount\":%d,\"frameLen\":%d,\"channels\":[%s]}}",
    rcConfig.matrixChannel, modeCh, oldValueMatrix, oldValueMode,
    pwmToButton(oldValueMatrix), FunctionSwState,
    sbusOk ? "true" : "false", sbusFps, sbusFrameCount, ageMs,
    lostFrameOld ? "true" : "false", sbusFailsafe ? "true" : "false",
    sbusRx.detectedChCount, sbusRx.detectedFrameLen, channelBuf);
  Serial.println(buf);
}

// =============================================================================
//  Serial-port baud (re)application
//  Serial2 (Maestro) and Serial3/4 (aux SWSerial) are opened from rcConfig.
//  Called once at boot AND again after every SET_CONFIG save, so a baud
//  change in the config tool takes effect immediately — no reboot needed.
//  Only a port whose baud actually changed is re-opened: re-begin() briefly
//  drops the line, so unrelated saves must not disturb live ports.
// =============================================================================
static uint32_t appliedMaestroBaud = 0;
static uint32_t appliedAuxBaud[2]  = { 0, 0 };

static void applySerialBauds(bool initial) {
  if (initial || rcConfig.maestroBaud != appliedMaestroBaud) {
    if (!initial) Serial2.end();
    Serial2.begin(rcConfig.maestroBaud, SERIAL_8N1, MAESTRO_RX_PIN, MAESTRO_TX_PIN);
    appliedMaestroBaud = rcConfig.maestroBaud;
    Serial.printf("[Serial2] Local Maestro %s @ %lu baud  TX=GPIO%d\n",
                  initial ? "open" : "re-open",
                  (unsigned long)rcConfig.maestroBaud, MAESTRO_TX_PIN);
  }
  if (initial || rcConfig.auxBaud[0] != appliedAuxBaud[0]) {
    if (!initial) Serial3.end();
    Serial3.begin(rcConfig.auxBaud[0], SWSERIAL_8N1, S3_RX_PIN, S3_TX_PIN, false, 95);
    appliedAuxBaud[0] = rcConfig.auxBaud[0];
    Serial.printf("[AUX] S3 %s @ %lu baud\n",
                  initial ? "open" : "re-open", (unsigned long)rcConfig.auxBaud[0]);
  }
  if (initial || rcConfig.auxBaud[1] != appliedAuxBaud[1]) {
    if (!initial) Serial4.end();
    Serial4.begin(rcConfig.auxBaud[1], SWSERIAL_8N1, S4_RX_PIN, S4_TX_PIN, false, 95);
    appliedAuxBaud[1] = rcConfig.auxBaud[1];
    Serial.printf("[AUX] S4 %s @ %lu baud\n",
                  initial ? "open" : "re-open", (unsigned long)rcConfig.auxBaud[1]);
  }
}

// =============================================================================
//  USB Serial WebSerial protocol handler
//  Same JSON protocol as Body Controller so the config_tool HTML is compatible.
// =============================================================================
String serialInputBuf;

void handleSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serialInputBuf.trim();
      if (serialInputBuf.length() == 0) { serialInputBuf = ""; return; }

      if (serialInputBuf[0] == '{') {
        // ── JSON WebSerial protocol ──────────────────────────────────────────
        // The header doc has to use a Filter — without one, deserializing
        // a SET_CONFIG payload (tens of KB once mappings/knobs/maestros
        // are populated) into the small hdr buffer returns NoMemory and
        // breaks SET_CONFIG. The filter is a WHITELIST in ArduinoJson v6:
        // every field used by a non-SET_CONFIG handler must be listed
        // explicitly, otherwise it's stripped and hdr["x"] returns the
        // default. SET_CONFIG does its own un-filtered deserialize below.
        StaticJsonDocument<256> filter;
        filter["type"]   = true;
        filter["target"] = true;   // WCB_SEND
        filter["cmd"]    = true;   // WCB_SEND
        filter["on"]     = true;   // CALIB
        filter["flags"]  = true;   // SET_DEBUG_FLAGS
        filter["mode"]   = true;   // TRIGGER
        filter["btn"]    = true;   // TRIGGER
        filter["tap"]    = true;   // TRIGGER
        DynamicJsonDocument hdr(256);
        DeserializationError perr = deserializeJson(
            hdr, serialInputBuf,
            DeserializationOption::Filter(filter));
        if (perr != DeserializationError::Ok) {
          // Include the received-buffer length so the host can spot truncation
          // (host-sent length vs. received length mismatch → USB RX overflow).
          Serial.printf("{\"type\":\"ERROR\",\"msg\":\"JSON parse failed (%s)\",\"rxLen\":%u}\n",
                        perr.c_str(), (unsigned)serialInputBuf.length());
          serialInputBuf = ""; return;
        }
        const char* type = hdr["type"] | "";

        if (strcmp(type,"PING")==0 || strcmp(type,"ping")==0) {
          // A fresh page connect pings first — clear any stale calibration
          // mute left behind by a previously crashed/closed calibration page.
          calibrationActive = false;
          // Report firmware version so the GUI can display it (footer +
          // Firmware tab "currently on board").
          Serial.print("{\"type\":\"PONG\",\"version\":\"");
          Serial.print(FW_VERSION);
          Serial.println("\"}");

        } else if (strcmp(type,"GET_CONFIG")==0) {
          Serial.print("{\"type\":\"CONFIG\",\"data\":");
          Serial.print(rcConfigToJSON());
          Serial.println("}");

        } else if (strcmp(type,"SET_CONFIG")==0) {
          DynamicJsonDocument bigDoc(98304);
          if (deserializeJson(bigDoc, serialInputBuf) != DeserializationError::Ok) {
            Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"parse failed\"}");
          } else if (!bigDoc.containsKey("data")) {
            Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"missing data\"}");
          } else {
            bool ok = rcConfigFromJSON(bigDoc["data"].as<JsonObject>());
            if (ok) {
              rcConfigSaveNVS();
              // Apply baud changes live (Serial2/3/4) so HCR / MP3 / Maestro
              // pick up a new rate immediately — no reboot required.
              applySerialBauds(false);
              Serial.println("{\"type\":\"ACK\",\"ok\":true}");
            } else {
              Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"config apply failed\"}");
            }
          }

        } else if (strcmp(type,"START_MONITOR")==0) {
          wsMonitorActive   = true;
          wsMonitorLastSent = 0;
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");

        } else if (strcmp(type,"STOP_MONITOR")==0) {
          wsMonitorActive = false;
          // Calibration always runs with the monitor on; stopping it is a
          // reliable "calibration is over" signal even if CALIB-off is lost.
          calibrationActive = false;
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");

        } else if (strcmp(type,"CALIB")==0) {
          calibrationActive = hdr["on"] | false;
          Serial.printf("[CALIB] action dispatch %s\n",
                        calibrationActive ? "SUPPRESSED (calibrating)" : "resumed");
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");

        } else if (strcmp(type,"RESET_DEFAULTS")==0) {
          rcConfigLoadDefaults();
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");

        } else if (strcmp(type,"REBOOT")==0) {
          // ACK first so the GUI hears the reply, then restart after a brief
          // delay so the USB TX buffer drains before the reset kills it.
          Serial.println("{\"type\":\"ACK\",\"ok\":true,\"msg\":\"rebooting\"}");
          Serial.flush();
          delay(250);
          ESP.restart();

        } else if (strcmp(type,"TRIGGER")==0) {
          int mode = hdr["mode"] | 1;
          int btn  = hdr["btn"]  | 0;
          int tap  = hdr["tap"]  | 1;
          if (btn < 1 || btn > 19 || mode < 1 || mode > 3 || tap < 1 || tap > 3) {
            Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"bad mode/btn/tap\"}");
          } else {
            Serial.printf("[TRIGGER] mode=%d btn=%d tap=%d\n", mode, btn, tap);
            rcDispatch(mode * 100 + btn, (uint8_t)tap);
            Serial.println("{\"type\":\"ACK\",\"ok\":true}");
          }

        } else if (strcmp(type,"WCB_SEND")==0) {
          int target     = hdr["target"] | 0;
          const char* cmd = hdr["cmd"]   | "";
          if (target == 0)                               wcb->broadcast(cmd);
          else if (target >= 1 && target <= WCB_MAX_BOARDS) wcb->send((uint8_t)target, cmd);
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");

        } else if (strcmp(type,"SET_DEBUG_FLAGS")==0) {
          // GUI debug chips drive this — bitmask of DBG_* categories to
          // enable. Default 0 silences every [DISPATCH] line.
          g_dbgFlags = (uint32_t)(hdr["flags"] | 0);
          Serial.printf("[DBG] flags=0x%02X\n", (unsigned)g_dbgFlags);
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");

        } else if (strcmp(type,"GET_WCB_STATUS")==0) {
          // Lightweight liveness poll for the GUI's sidebar WCB Status
          // panel. Reads wcb->isOnline(i) for boards 1..quantity. The
          // sketch's CLI #L11 prints the same info as human-readable text.
          int q = rcConfig.wcbNetwork.quantity;
          if (q < 0) q = 0;
          if (q > WCB_MAX_BOARDS) q = WCB_MAX_BOARDS;
          // Self is by definition online — wcb->isOnline() tracks REMOTE
          // peer heartbeats via ETM and never returns true for our own
          // deviceId, so we force "1" for the local board.
          int selfId = rcConfig.wcbNetwork.deviceId;
          Serial.printf("{\"type\":\"WCB_STATUS\",\"quantity\":%d,\"self\":%d,\"online\":[",
                        q, selfId);
          for (int i = 1; i <= q; i++) {
            bool up = (i == selfId) ? true : (wcb && wcb->isOnline(i));
            Serial.printf("%s%s", (i > 1) ? "," : "", up ? "1" : "0");
          }
          Serial.println("]}");

        } else {
          Serial.println("{\"type\":\"ERROR\",\"msg\":\"unknown type\"}");
        }

      } else if (serialInputBuf[0] == '#') {
        // ── CLI commands ─────────────────────────────────────────────────────
        if (serialInputBuf.length() >= 3 &&
            (serialInputBuf[1] == 'L' || serialInputBuf[1] == 'l')) {
          int fn = 0;
          if (serialInputBuf.length() >= 4)
            fn = (serialInputBuf[2]-'0')*10 + (serialInputBuf[3]-'0');
          else
            fn = serialInputBuf[2] - '0';
          switch (fn) {
            case 1:  Serial.println("RC-Controller — WCB HW 3.2"); break;
            case 2:  ESP.restart(); break;
            case 9:  dumpSbusState(); break;
            case 10: sbusLiveDump = !sbusLiveDump;
                     Serial.printf("SBUS live dump %s\n", sbusLiveDump ? "ON (1Hz)" : "OFF"); break;
            case 11: {
              Serial.printf("WCB device ID: %d  quantity: %d\n",
                            rcConfig.wcbNetwork.deviceId, rcConfig.wcbNetwork.quantity);
              Serial.print("  Board status: ");
              for (int i = 1; i <= rcConfig.wcbNetwork.quantity; i++)
                Serial.printf("WCB%d=%s ", i, wcb->isOnline(i) ? "UP" : "dn");
              Serial.println();
              break;
            }
            case 12: Serial.printf("Mode=%d  matrixBtn=%d  matrixVal=%d\n",
                                   FunctionSwState, pwmToButton(oldValueMatrix), oldValueMatrix); break;
            // ── HCR local-serial TX diagnostics ──────────────────────────────
            // These bypass rcConfig.hcrDest AND button mapping entirely. They
            // drive Serial3/Serial4 directly so a "nothing on the HCR" report
            // can be split into:  firmware-TX-path/wiring  vs  config/dispatch.
            //   #L20 → HCR on S3 (GPIO15 TX): SetEmotion(HAPPY,80) + raw marker
            //   #L21 → HCR on S4 (GPIO17 TX): same
            // If the HCR reacts to #L20 but not to a mapped button, the bug is
            // in config (hcrDest transport/target not saved) or button mapping,
            // NOT the serial wiring. If #L20 also does nothing, it's wiring
            // (TX/RX swap, ground, 3V3 vs 5V) or the EspSoftwareSerial port.
            case 20:
            case 21: {
              Stream*     h  = (fn == 20) ? (Stream*)&Serial3 : (Stream*)&Serial4;
              const char* pn = (fn == 20) ? "S3 (GPIO15)"     : "S4 (GPIO17)";
              Serial.printf("[HCR TEST] -> %s : SetEmotion(HAPPY,80) via hcrFormatCommand + raw frame\n", pn);
              // Send the SetEmotion(HAPPY,80) payload we'd normally dispatch
              // through hcrFormatCommand, plus a second raw line in case a
              // formatter bug is the cause — both are the exact byte string a
              // working HCR expects, no library logic in between.
              h->print(hcrFormatCommand(2 /*SetEmotion*/, 0 /*HAPPY*/, 80));
              h->print("<OH80,QEH>\n");
              Serial.println("[HCR TEST] sent — watch the HCR; check TX wiring to HCR RX, common ground");
              break;
            }
          }
        }
      }
      serialInputBuf = "";
    } else {
      if (serialInputBuf.length() < 8192) serialInputBuf += c;
    }
  }
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  // Bump the USB-CDC RX buffer well above the 256-byte default. The SBUS OUT
  // byte-streaming tee blocks the main loop for ~2.75 ms per SBUS frame while
  // bit-banging SoftwareSerial; during that window the host can shove ~1-2 KB
  // into us at USB-CDC speed. A 4 KB buffer comfortably absorbs the worst case
  // (e.g. a 3-4 KB SET_CONFIG payload arriving in one shot) without dropping
  // bytes that would otherwise corrupt the JSON. Must be set BEFORE begin().
  Serial.setRxBufferSize(4096);
  // TX ring buffer so vlogf() has headroom to write into when a host is
  // draining the port. When no host is attached this fills once and then
  // vlogf() simply drops further lines — it never blocks the loop.
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n\n=== RC-Controller (WCB HW 3.2) ===");

  // Status LED
  statusLed.begin();
  setStatusLed(C_RED, 255);

  // Serial2 (local Maestro bus) and Serial3-4 (aux SoftwareSerial) are opened
  // AFTER the config loads, below, at their configured baud (rcConfig.maestroBaud
  // / rcConfig.auxBaud). Nothing uses them before setup() finishes, so
  // deferring the open is safe.

  // SBUS reader on Serial1 — RX = SBUS_RX_PIN, TX disabled (-1).
  // The SBUS OUT passthrough is intentionally DISABLED here as a diagnostic:
  // even after moving it from SoftwareSerial to a hardware-UART TX FIFO,
  // the board still ran for ~a minute then panicked under sustained SBUS
  // input.  Disabling the passthrough entirely tells us whether any SBUS-OUT
  // write activity is involved or whether the crash is purely in the SBUS RX
  // / dispatch path.  Re-enable by passing SBUS_OUT_PIN to begin() and
  // pointing the sink at &Serial1 (see chat log 2026-05-22).
  sbusRx.begin(&Serial1, SBUS_RX_PIN, /*txPin=*/-1);
  // sbusRx.setPassthroughSink(&Serial1);   // intentionally not set — see above
  Serial.printf("[SBUS] IN  on Serial1 RX (GPIO%d)\n", SBUS_RX_PIN);
  Serial.println("[SBUS] OUT DISABLED (diagnostic — see source comment)");

  // RC Config: defaults then NVS overrides. Must run BEFORE constructing
  // WCB_Client so the network credentials come from rcConfig.wcbNetwork.
  rcConfigLoadDefaults();
  rcConfigLoadNVS();

  // Open Serial2 (local Maestro) + Serial3/4 (aux SWSerial) at their
  // configured baud. Single source of truth: rcConfig (maestroBaud /
  // auxBaud[]). Same helper runs again after every SET_CONFIG save so a
  // baud change in the config tool applies live — no reboot needed.
  // Keep aux baud ≤ ~57600 (higher rates choke bit-banged SWSerial on
  // ESP32-S3). One port = one device = one baud.
  applySerialBauds(true);

  // WCB Client — sets STA mode + custom MAC + inits ESP-NOW.
  // No WiFi AP or web server — ESP-NOW only.  Credentials come from NVS
  // (editable via the GUI's "WCB Network" sidebar); a reboot is required
  // for credential changes to take effect.
  wcb = new WCB_Client(rcConfig.wcbNetwork.macOct2,
                      rcConfig.wcbNetwork.macOct3,
                      rcConfig.wcbNetwork.password,
                      rcConfig.wcbNetwork.quantity,
                      rcConfig.wcbNetwork.deviceId);
  if (!wcb->begin()) {
    Serial.println("[WCB] ERROR: wcb->begin() failed — check WCB Network settings in GUI");
    setStatusLed(C_ORANGE, 200);
  } else {
    wcb->onCommand(onWCBCommand);
    Serial.printf("[WCB] Joined network as device ID %d (quantity=%d)\n",
                  rcConfig.wcbNetwork.deviceId, rcConfig.wcbNetwork.quantity);
  }

  // Construct the broadcast Maestro stream AFTER wcb exists. WCBStream's
  // constructor self-registers with the WCB_Client singleton; doing this here
  // (rather than at global scope) is what guarantees wcb->update() actually
  // drives the stream's flush so remote Maestro bytes go out over ESP-NOW.
  maestroBroadcast = new WCBStream(/*target_wcb=*/0, /*target_port=*/0);

  // Clear state
  memset(pendingActions, 0, sizeof(pendingActions));
  memset(switchPrevPos, -1, sizeof(switchPrevPos));
  serialInputBuf.reserve(256);

  setStatusLed(C_BLUE, 10);
  Serial.print  ("[RC-Controller] Firmware ");
  Serial.print  (FW_VERSION);
  Serial.println(" — setup complete.");
  Serial.println("  Connect config_tool/index.html via Web Serial for configuration.");
  Serial.println("  CLI: #L01=info  #L09=SBUS dump  #L10=live  #L11=WCB status  #L12=RC state");
  Serial.println("  CLI: #L20=HCR S3 test  #L21=HCR S4 test  (direct, bypasses config+mapping)");
  Serial.println("  Send PING to test. Send GET_CONFIG to read mappings.");
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
  // WCB — heartbeats, ACKs, WCBStream flushes
  if (wcb) wcb->update();

  // SBUS
  processSbus();
  checkDeferredTap();

  // Pending delayed actions
  checkPendingActions();

  // USB Serial WebSerial monitor stream
  sendPWMUpdate();

  // USB Serial input
  handleSerialInput();

  // FPS counter
  trackSbusFps();

  // 1Hz SBUS live dump (#L10)
  if (sbusLiveDump && (millis() - sbusLiveDumpLastMs >= 1000)) {
    sbusLiveDumpLastMs = millis();
    dumpSbusState();
  }
}
