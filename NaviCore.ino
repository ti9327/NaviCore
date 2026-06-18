// =============================================================================
//  NaviCore.ino — WCB-based RC Controller (formerly RC-Controller / HyperCore)
//  Target hardware: WCB HW 3.2 (ESP32-S3)
//
//  Features:
//    • SBUS input (16 or 24 channel, auto-detected) on Serial1/UART1 RX (GPIO5)
//    • SBUS output passthrough on UART0 TX (GPIO9) — byte-streamed re-emit of
//      the RX frame so a downstream device sees the same channel data
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
#include <LittleFS.h>     // config persists as /config.json (replaces the NVS 4000-byte/value limit)
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "esp_timer.h"          // one-shot boot-guard timer (cold-boot auto-recovery)
#include "esp_ota_ops.h"        // esp_ota_get_bootloader_description (boot banner)
#include "rom/rtc.h"            // rtc_get_reset_reason (low-level boot telemetry)
#include <WCB_Client.h>   // header in greghulette/WCBClient is WCB_Client.h
#include <WCBStream.h>
// HCR (Human Cyborg Relations Vocalizer): no library dependency — we format
// the same byte string the upstream HCRVocalizer would have written and push
// it directly to the bound aux-serial port.  See hcrFormatCommand() below.
#include "sbus_reader.h"
#include "rc_config.h"
#include "wcb_config.h"
#include "fw_version.h"     // FW_VERSION_BASE / FW_VERSION_DTG / FW_VERSION
#include "rc_telemetry.h"   // WCB-network remote-management bridge (Phase 1 — see file header)

// =============================================================================
//  WCB HW 3.2 — Pin Definitions
//  TX pins from wcb_pin_map.cpp v3.2 (matches SBUSController.ino comments).
//  RX pins: GPIO5 confirmed by user; others assumed — verify against schematic.
// =============================================================================
#define SBUS_RX_PIN      5    // Serial1/UART1 RX — SBUS from RC receiver (confirmed)
// SBUS OUT runs on its OWN hardware UART (UART0 / Serial0), TX-only on
// SBUS_OUT_PIN below.  This is the third iteration:
//   v1 bit-banged via SoftwareSerial on GPIO9 — blocking ~110µs/byte spin
//      monopolised ~31% of a core and tripped the stack-canary watchdog.
//   v2 teed into Serial1's TX (shared with SBUS RX) — still panicked.
//   v3 (current) uses UART0, which freed up when the debug console moved
//      to native USB CDC (USBMode=hwcdc / CDCOnBoot=cdc — Serial is no
//      longer UART0).  Hardware UART = ~1µs FIFO push per byte, and a
//      separate peripheral from SBUS RX = no FIFO contention.  Configured
//      in setup(); see the SBUS OUT block there.

#define MAESTRO_TX_PIN   6    // Serial2 TX — local Maestro command bus
#define MAESTRO_RX_PIN   7    // Serial2 RX — optional Maestro feedback (verify pin)

#define S3_TX_PIN       15    // Aux serial S3 TX (SoftwareSerial)
#define S3_RX_PIN       16    // Aux serial S3 RX
#define S4_TX_PIN       17    // Aux serial S4 TX (SoftwareSerial)
#define S4_RX_PIN       18    // Aux serial S4 RX

// SBUS OUT — pure passthrough re-emission of the SBUS RX stream so downstream
// devices can consume the same channel data.  Emitted on UART0 (Serial0) at
// 100k 8E2 inverted, TX-only; each byte is teed into the UART TX FIFO as it
// arrives on Serial1 RX (~1 µs push).  See SbusReader::setPassthroughSink().
#define SBUS_OUT_PIN     9    // UART0 (Serial0) TX → downstream SBUS consumer

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
// True ONLY after wcb->begin() succeeds. ESP-NOW is unusable until then, and
// calling send/broadcast/update on a WCB_Client whose begin() failed is
// undefined behavior — so every wcb-> call is gated on this flag.
bool wcbReady = false;

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
//  ESP32-S3 has 3 hardware UARTs (UART0/1/2).  Allocation once the debug
//  console moved to native USB CDC (USBMode=hwcdc, CDCOnBoot=cdc):
//    • Serial  → native USB CDC (debug + config tool)  — no UART consumed
//    • Serial0 → UART0 → SBUS OUT (TX-only, GPIO9, 100k 8E2 inverted)
//    • Serial1 → UART1 → SBUS RX
//    • Serial2 → UART2 → local Maestro TX
//  That leaves no spare hardware UART for the aux command ports, so S3/S4
//  stay bit-banged via SoftwareSerial (fine at ≤57600 baud; they never run
//  the 100k SBUS rate).
// =============================================================================
SoftwareSerial Serial3;    // Aux command-line port, bound in setup()
SoftwareSerial Serial4;    // Aux command-line port, bound in setup()
// SBUS OUT uses the real UART0 (Serial0) — see setup() and SBUS_OUT_PIN above.

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
// rcConfig is heap-allocated in EXTERNAL PSRAM (ESP32-S3-WROOM-1 N16R8 = 8 MB).
// With the 15 logical-button slots the struct is ~210 KB — it will NOT fit in
// internal DRAM alongside the WCB stack + the ~96 KB SET_CONFIG JSON parse.
//
// NOTE: EXT_RAM_BSS_ATTR does NOT help here — the stock Arduino-ESP32 core is
// built without CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY, so the attribute is
// a no-op and the struct lands in .bss → linker "dram0_0_seg overflowed". The
// reliable way on the stock core is to allocate from the PSRAM heap at runtime
// (ps_calloc, at the top of setup()).  The global below is just a 4-byte pointer;
// `rcConfig` is a macro alias (see rc_config.h) so all rcConfig.xxx access is
// unchanged.  Mapping/threshold arrays aren't a hot path (read on button events,
// not per-SBUS-byte), so PSRAM latency is irrelevant.
// REQUIRES Arduino IDE: Tools → PSRAM → "OPI PSRAM" (the N16R8 is OCTAL PSRAM).
// If PSRAM is off, ps_calloc() returns null and setup() halts with a message.
RcConfig* g_rcConfig = nullptr;

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

// LED fault latch — tiny arbiter so a fault outranks the routine SBUS status.
// A non-zero color here means "a persistent fault was detected" (currently:
// wcb->begin() failure in setup; future faults can latch their own color).
// updateStatusLed() displays a latched fault STEADY and skips the SBUS
// indication, so last-writer-wins can't silently erase a fault. 0 = no fault.
uint32_t g_ledFaultColor = 0;

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
    // A 0/0 band is the "Unassigned" sentinel — treat it as inert so it
    // can never match (otherwise pwmToButton(0) returns that slot, e.g. a
    // pre-signal/garbage 0 would decode as a real button press).
    if (rcConfig.thresholds[i].minPwm == 0 && rcConfig.thresholds[i].maxPwm == 0)
      continue;
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
  if (slot.type == 0) return;          // disabled (expected — not an error)
  if (slot.device > 127) {             // invalid Pololu device # (config error)
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u: invalid Pololu device # %u (must be 0-127) — skipped\n",
         id, slot.device);
    return;
  }

  Stream* dest = (slot.type == 1) ? (Stream*)&Serial2 : (Stream*)maestroBroadcast;
  if (!dest) return;                    // remote slot but stream not yet up
  uint8_t hdr[3] = { 0xAA, slot.device, (uint8_t)(cmd_compact & 0x7F) };
  dest->write(hdr, 3);
  if (payload && plen) dest->write(payload, plen);
}

// Valid Maestro channel guard (0-31 covers Micro/Mini Maestro 6/12/18/24).
// An out-of-range channel is a config error; warn + skip so it isn't a silent
// no-op the user has to debug by guessing the servo/wiring is broken.
static bool maestroChanOk(uint8_t id, uint8_t ch) {
  if (ch <= 31) return true;
  dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u: channel %u out of range (0-31) — skipped\n", id, ch);
  return false;
}

// Pololu Maestro command byte values (compact protocol).
//   0x84 SET_TARGET   · 0x87 SET_SPEED  · 0x89 SET_ACCEL
//   0xA2 GO_HOME      · 0xA4 STOP_SCRIPT · 0xA7 RESTART_SCRIPT_AT_SUB
static void maestroSetTarget(uint8_t id, uint8_t ch, uint16_t pos) {
  if (!maestroChanOk(id, ch)) return;
  uint8_t p[3] = { ch, (uint8_t)(pos & 0x7F), (uint8_t)((pos >> 7) & 0x7F) };
  maestroWrite(id, 0x84, p, 3);
}
static void maestroSetSpeed(uint8_t id, uint8_t ch, uint16_t spd) {
  if (!maestroChanOk(id, ch)) return;
  uint8_t p[3] = { ch, (uint8_t)(spd & 0x7F), (uint8_t)((spd >> 7) & 0x7F) };
  maestroWrite(id, 0x87, p, 3);
}
static void maestroSetAccel(uint8_t id, uint8_t ch, uint8_t accel) {
  if (!maestroChanOk(id, ch)) return;
  // Acceleration (0-255) is sent as TWO 7-bit bytes, same framing as
  // speed/target. The old single-byte form set the high bit for accel>127
  // (corrupting the Pololu data stream) and was one byte short of the frame
  // the Maestro expects for 0x89.
  uint8_t p[3] = { ch, (uint8_t)(accel & 0x7F), (uint8_t)((accel >> 7) & 0x7F) };
  maestroWrite(id, 0x89, p, 3);
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
// Write the payload + a trailing CR in as few SoftwareSerial calls as
// possible.  The old form `for (char c : (s + '\r'))` allocated a fresh
// String every call AND wrote one byte at a time; on a bit-banged port
// each write blocks ~1 byte-time, so a long command stalled loop() (and
// thus SBUS) for many ms.  One block write + one CR minimizes the hit.
void writeS3(const String& s) { Serial3.write((const uint8_t*)s.c_str(), s.length()); Serial3.write('\r'); }
void writeS4(const String& s) { Serial4.write((const uint8_t*)s.c_str(), s.length()); Serial4.write('\r'); }

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
//     6  Muse()                (single muse)
//     7  Muse(min, max)        (auto-muse gap in seconds — min=chan, max=track)
//     8  Stop all              (Stops V/A/B audio + emotes)
//     9  StopEmote()
//    10  OverrideEmotions(v)   (v=chan, 0=off/1=on — locks emotion normalization)
//    11  ResetEmotions()
//    13  SetMuse(v)            (v=track, 0=off/1=on — continuous idle musing)
//    14  PlayWAV(ch, track)
//    16  StopWAV(ch)
//    17  SetVolume(ch, vol)
//
//  fn numbers + parameter positions match the BC firmware's HCRFunction()
//  dispatcher exactly (RA_HCR shares rc_config.h with the Body Controller), so a
//  saved action means the same thing on both. The query/poll functions (1, 18+)
//  are intentionally NOT implemented — NaviCore is fire-and-forget.
//
//  Returns an empty String for unknown fn or bad parameters.
// =============================================================================
// Validate + normalize an HCR fn/chan/track triplet IN PLACE. This is the ONE
// place that owns the parameter ranges AND the "emotion 4 = Overload" shortcut
// (fn 3/4 + chan 4 → fn 5). Both transports — local serial (hcrFormatCommand)
// and WCB (hcrFormatWcbCommand) — normalize through here first, so they can
// never drift apart on what an action means. Returns false for an unknown fn
// or out-of-range params (caller skips the action).
static bool hcrNormalizeAction(uint8_t& fn, int& chan, int& track) {
  switch (fn) {
    case 2:   // SetEmotion(e, v)
      return (chan >= 0 && chan <= 3 && track >= 0 && track <= 99);
    case 3:   // Trigger — same payload as Stimulate
    case 4:   // Stimulate(e, v)
      if (chan < 0 || chan > 4 || track < 0 || track > 99) return false;
      if (chan == 4) { fn = 5; chan = 0; track = 0; }   // emotion 4 = Overload shortcut
      return true;
    case 5: case 6: case 8: case 9: case 11:   // no-param functions
      return true;
    case 7:   // Muse(min, max) — auto-muse gap in seconds (min=chan, max=track)
      return (chan >= 0 && chan <= 99 && track >= 0 && track <= 99);
    case 10:  // OverrideEmotions(v) — v=chan (0/1); locks emotion normalization
      return (chan >= 0 && chan <= 1);
    case 13:  // SetMuse(v) — v=track (0/1); continuous idle musing on/off
      return (track >= 0 && track <= 1);
    case 14:  // PlayWAV(ch, track)
      return (chan >= 0 && chan <= 2 && track >= 0 && track <= 9999);
    case 16:  // StopWAV(ch)
      return (chan >= 0 && chan <= 2);
    case 17:  // SetVolume(ch, vol)
      return (chan >= 0 && chan <= 2 && track >= 0 && track <= 99);
    default:
      return false;
  }
}

static String hcrFormatCommand(uint8_t fn, int chan, int track) {
  static const char emoteprefix[] = "HSMC";   // HAPPY / SAD / MAD / sCared
  static const char audioprefix[] = "VAB";    // Vocalizer / A / B
  if (!hcrNormalizeAction(fn, chan, track)) return "";   // ranges + Overload shortcut
  String inner;
  switch (fn) {
    case 2:   // SetEmotion(e, v)
      inner = String("O") + emoteprefix[chan] + String(track) + ",QE" + emoteprefix[chan];
      break;
    case 3:    // Trigger — same payload as Stimulate
    case 4:    // Stimulate(e, v)  (chan==4 already normalized to fn 5 above)
      inner = String("S") + emoteprefix[chan] + String(track) + ",QE" + emoteprefix[chan] + ",QT";
      break;
    case 5:  inner = "SE,QT";          break;  // Overload
    case 6:  inner = "MM";             break;  // single Muse
    case 7:  inner = String("MN") + chan + ",MX" + track; break;  // Muse(min,max) gap
    case 8:  inner = "PSV,PSA,PSB,QT"; break;  // Stop (all audio + emote)
    case 9:  inner = "PSV,QT";         break;  // StopEmote
    case 10: inner = String("O") + chan + ",QO"; break;  // OverrideEmotions(v) — "O<v>" (digit, vs "O<HSMC>" for SetEmotion)
    case 11: inner = "OR,QE";          break;  // ResetEmotions
    case 13: inner = String("M") + track + ",QM"; break;  // SetMuse(v) — "M<v>" (digit, vs "MM" single muse)
    case 14: {  // PlayWAV(ch, track) — file number is 0-padded to 4 digits
      char file[8]; snprintf(file, sizeof(file), "%04d", track);
      inner = String("C") + audioprefix[chan] + file + ",QP" + audioprefix[chan];
      break;
    }
    case 16:  // StopWAV(ch)
      inner = String("PS") + audioprefix[chan] + ",QP" + audioprefix[chan];
      break;
    case 17:  // SetVolume(ch, vol)
      inner = String("PV") + audioprefix[chan] + String(track);
      break;
    default: return "";   // unreachable — hcrNormalizeAction rejects unknown fn
  }
  return String("<") + inner + ">\n";
}

// Build the ";H,FN,<fn>,<chan>,<track>" command an HCR action sends over the
// WCB network. The receiving WCB strips the ';', routes 'H,...' to its
// processHCRRuntimeCommand() FN handler (the "RC-Controller numeric
// convention"), and that drives its locally-wired HCRVocalizer to the WCB's
// own configured HCR port. Gating on hcrFormatCommand() returning non-empty
// reuses the SAME fn/param validation as the local-serial path, so both
// transports accept exactly the same actions. Returns "" for an unknown fn or
// out-of-range params. No trailing newline — this is a WCB command, not a raw
// serial forward (mirrors mp3FormatCommand()).
static String hcrFormatWcbCommand(uint8_t fn, int chan, int track) {
  // Shared validation + normalization (incl. the emotion-4→Overload shortcut)
  // lives in hcrNormalizeAction() — one source of truth for both transports.
  if (!hcrNormalizeAction(fn, chan, track)) return "";
  // fn 7/10/13 are NOT in the WCB's numeric ";H,FN" switch (WCB_HCR.cpp
  // processHCRRuntimeCommand) — it only maps 2,3,4,5,6,8,9,11,14,16,17. Emit the
  // readable verbs the WCB DOES implement so HCR-over-WCB needs no firmware change
  // on the receiving board. All other fns use the numeric convention.
  switch (fn) {
    case 7:  return String(";H,MUSE,GAP,") + chan + "," + track;  // Muse(min,max)
    case 10: return String(";H,OVERRIDE,") + chan;                // OverrideEmotions(v)
    case 13: return String(";H,MUSE,") + track;                   // SetMuse(v)
    default: return String(";H,FN,") + (int)fn + "," + chan + "," + track;
  }
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
    // ── WCB transport (ETM command via ESP-NOW) ────────────────────────────
    // HCR over WCB is UNICAST ONLY and rides the normal WCB command path: we
    // send ";H,FN,<fn>,<chan>,<track>" and the receiving WCB strips the ';',
    // routes 'H,...' to processHCRRuntimeCommand()'s FN handler (the
    // "RC-Controller numeric convention"), which drives its locally-wired
    // HCRVocalizer to the WCB's OWN configured HCR port. So the WCB owns the
    // HCR serial port now — NaviCore no longer specifies it (dest.wcbPort is
    // unused on this path; the receiving WCB must have ?HCR,PORT configured).
    // This replaces the old raw-serial forward (sendRaw, targetID 97), which
    // was best-effort only: wcb->send() defaults to ETM, so a dropped HCR
    // command is retried until ACK'd — HCR can't tolerate a miss. Mirrors the
    // MP3-over-WCB pattern (";A,..." → processMP3AudioCommand). Broadcast is
    // unsupported — an HCR vocalizer is a single device at a known WCB.
    if (!wcb || !wcbReady) { dlog(DBG_HCR, "[DISPATCH] HCR-WCB: WCB not ready — skipped\n"); return; }
    String cmd = hcrFormatWcbCommand(a.fn, a.chan, a.track);
    if (cmd.length() == 0) {
      dlog(DBG_HCR, "[DISPATCH] HCR-WCB: bad/unsupported fn=%u chan=%d track=%d — skipped\n",
            a.fn, a.chan, a.track);
      return;
    }
    uint8_t target = (uint8_t)atoi(dest.target);
    if (target < 1 || target > WCB_MAX_BOARDS) {
      dlog(DBG_HCR, "[DISPATCH] HCR-WCB: target '%s' invalid — HCR over WCB must be a "
            "unicast WCB ID 1-%d (broadcast is not supported). Fix the HCR "
            "Destination in the config tool. Not sent.\n",
            dest.target, WCB_MAX_BOARDS);
      return;
    }
    bool ok = wcb->send(target, cmd.c_str());   // ETM by default — HCR can't tolerate a miss
    dlog(DBG_HCR, "[DISPATCH] HCR→WCB%u  %s  %s\n", target, cmd.c_str(), ok ? "OK" : "FAIL");
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
  // No flush(): on a bit-banged SoftwareSerial port flush() blocks until
  // the TX shift-register drains (~1 byte-time/byte at the aux baud), which
  // stalls loop() — and therefore SBUS read/dispatch/passthrough — on every
  // MP3-local command.  The bytes are already queued in the port's buffer
  // and clock out on their own; nothing here needs to wait for completion.
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
  if (!wcb || !wcbReady) { dlog(DBG_MP3, "[DISPATCH] MP3: WCB not ready — skipped\n"); return; }
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
// Forward declaration: scheduleAction() (queue-full path) and
// checkPendingActions() fire actions through rcExecuteActionNow(), which is
// defined further below. Declared explicitly so the call sites don't depend on
// the Arduino auto-prototype generator handling this static function.
static void rcExecuteActionNow(const RcAction& a);

static void scheduleAction(const RcAction& action, unsigned long delayMs) {
  for (int i = 0; i < PENDING_ACTION_SLOTS; i++) {
    if (!pendingActions[i].active) {
      pendingActions[i] = { true, millis() + delayMs, action };
      return;
    }
  }
  // Queue full — fire the action NOW (dropping its delay) rather than losing
  // it. rcExecuteActionNow bypasses the delay check, so this can't recurse.
  vlogf("WARN: pendingActions full (%d slots) — firing now, delay dropped\n", PENDING_ACTION_SLOTS);
  rcExecuteActionNow(action);
}

// Dispatch an action's EFFECT immediately — no delay handling, no calibration
// gate (callers handle those). Split out from rcExecuteAction so the delayed
// path fires WITHOUT re-entering the delay check: previously checkPendingActions
// called rcExecuteAction, which saw delayMs>0 and re-queued the action forever,
// so a delayed action's real effect never ran.
static void rcExecuteActionNow(const RcAction& a) {
  switch (a.type) {
    case RA_WCB_UNICAST: {
      uint8_t boardId = (uint8_t)atoi(a.target);
      if (boardId >= 1 && boardId <= WCB_MAX_BOARDS) {
        if (!wcb || !wcbReady) { dlog(DBG_WCB, "[DISPATCH] WCB→%d skipped — WCB not ready\n", boardId); break; }
        dlog(DBG_WCB, "[DISPATCH] WCB→%d  %s\n", boardId, a.cmd);
        wcb->send(boardId, a.cmd);
      }
      break;
    }
    case RA_WCB_BROADCAST:
      if (!wcb || !wcbReady) { dlog(DBG_WCB, "[DISPATCH] WCB broadcast skipped — WCB not ready\n"); break; }
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

// Public entry: applies the calibration gate and per-action delay, then
// dispatches the effect via rcExecuteActionNow().
void rcExecuteAction(const RcAction& a) {
  // Calibration mode: the operator is intentionally moving every control to
  // set thresholds — drop every action (including delayed/scheduled ones, so
  // nothing queues up to fire the instant calibration ends).
  if (calibrationActive) return;
  if (a.delayMs > 0) { scheduleAction(a, a.delayMs); return; }
  rcExecuteActionNow(a);
}

void checkPendingActions() {
  unsigned long now = millis();
  for (int i = 0; i < PENDING_ACTION_SLOTS; i++) {
    if (pendingActions[i].active && now >= pendingActions[i].fireAt) {
      pendingActions[i].active = false;
      // Fire the EFFECT directly (NOT rcExecuteAction) so the action's own
      // delayMs can't re-queue it forever. Honor a calibration that started
      // after it was scheduled.
      if (!calibrationActive) rcExecuteActionNow(pendingActions[i].action);
    }
  }
}

// =============================================================================
//  RC dispatch by buttonId + tap count
// =============================================================================
void rcDispatch(int buttonId, uint8_t tapCount) {
  int mode = buttonId / 100, btn = buttonId % 100;
  if (mode < 1 || mode > 3 || btn < 1 || btn > RC_NUM_THRESHOLDS) return;
  if (tapCount < 1 || tapCount > 3) return;

  // Broadcast a "this trigger fired" event over the WCB ESP-NOW network so
  // the config tool's "Via WCB" mode (and any listening Wizard) sees every
  // dispatch — local matrix press, Web-Serial JSON TRIGGER, or remote
  // ESP-NOW TRIGGER — uniformly.  Emitted BEFORE action execution so the
  // event arrives even if a synchronous action stalls.
  rcTelemetry::emitTrig(mode, btn, tapCount);

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
  if (audioChan > 2) {
    dlog(DBG_HCR, "[DISPATCH] HCR volume: audio channel %u out of range (0-2) — "
         "check the knob's HCR output target; skipped\n", audioChan);
    return;
  }
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
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,    // S1, S2, LS, RS
  0xFFFF,                            // MS  (X20)
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,    // J1-J4
  0xFFFF, 0xFFFF                     // J5, J6 (X20 3-axis)
};

void processKnobs() {
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    RcKnob& kn = rcConfig.knobs[i];
    if (kn.channel < 1 || kn.channel > 24) continue;
    if (kn.function == KF_NONE || kn.outputCount == 0) continue;
    uint16_t raw = sbusValues[kn.channel - 1];

    // Per-knob direction reversal — invert around the SBUS centre so a
    // "stick up" reading maps to what would otherwise be "stick down" for
    // all downstream output computations.  SBUS valid range is ~172-1811;
    // reflecting around the midpoint = (172 + 1811) - raw = 1983 - raw.
    // The deadband / change-detection still uses the post-reverse value so
    // moving the source dispatches as expected.
    if (kn.reverse) {
      int flipped = 1983 - (int)raw;
      if (flipped < 0)     flipped = 0;
      if (flipped > 2047)  flipped = 2047;
      raw = (uint16_t)flipped;
    }

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

  // ── Failsafe gate — DO NOT dispatch on a failsafe frame ─────────────────
  // When the transmitter powers off (or link is lost), many receivers keep
  // emitting frames with the failsafe bit set and channels parked at their
  // configured failsafe positions.  Acting on those would drive servos to
  // the failsafe pose and fire any switch/knob thresholds the parked values
  // happen to cross — unexpected motion on signal loss, which is dangerous
  // for an animatronic.  Instead we freeze: keep telemetry/FPS current (so
  // the config tool shows "FAILSAFE"), but skip mode/matrix/switch/knob
  // dispatch entirely, leaving all outputs holding their last commanded
  // state.  We also reset the matrix debounce so that when the link
  // recovers, a button physically held across the dropout requires a fresh
  // confirmed-neutral + press before it can fire (no recovery-transient
  // phantom press).
  // NOTE: this gates on `failsafe` only, NOT `lostFrame` — lostFrame is a
  // single-frame transient and gating on it would make control feel laggy.
  if (sbusRx.failsafe) {
    matrixArmed        = false;   // require a confirmed neutral to re-arm post-recovery
    matrixCandidate    = 0;
    matrixCandCount    = 0;
    matrixNeutralCount = 0;
    return;
  }

  // Mode selector — same SBUS-cluster thresholds as readSwitchPos()
  // (582/1401) so the bound mode switch decodes its three positions
  // correctly. Earlier 340/680 incorrectly mapped middle (~992) → mode 3.
  int modeVal = readBoundSwitchSbus(rcConfig.funcBindings.modeSwitch);
  if (modeVal >= 0 && abs(modeVal - oldValueMode) > 5) {
    oldValueMode = modeVal;
    int newMode = (modeVal < 582) ? 1 : (modeVal < 1401 ? 2 : 3);
    if (newMode != FunctionSwState) {
      FunctionSwState = newMode;
      // Surface the mode change on the WCB network immediately — config
      // tools watching via the bridge get instant feedback instead of
      // waiting up to 2 s for the next rc_hb heartbeat.
      rcTelemetry::emitMode(FunctionSwState);
    }
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
  // Delegate to the telemetry/management bridge first — if it recognised
  // the command as a JSON management message addressed to us (PING /
  // TRIGGER / SET_MODE / GET_CONFIG / SET_CONFIG), it dispatches and
  // returns true.  Otherwise we just log the raw command for visibility
  // (legacy WCB ;-commands intended for other peers or droid hardware).
  if (rcTelemetry::handle(senderID, command)) return;
  // Unhandled (legacy/unknown) WCB command.  This runs in the ESP-NOW
  // receive callback on Core 0; a blocking Serial.printf here can stall the
  // WiFi task (if a USB host is attached but not draining) and interleave
  // with the main loop's Serial output on Core 1.  It's rare (almost all RC
  // traffic is JSON handled above), so gate it behind the same verbose flag
  // used for the fragment logging rather than printing unconditionally.
  if (rcTelemetry::RC_TELEM_VERBOSE)
    Serial.printf("[WCB RX] from WCB%d: %s\n", senderID, command);
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
            Serial.println("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":false,\"msg\":\"parse failed\"}");
          } else if (!bigDoc.containsKey("data")) {
            Serial.println("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":false,\"msg\":\"missing data\"}");
          } else {
            bool ok = rcConfigFromJSON(bigDoc["data"].as<JsonObject>());
            if (ok) {
              bool saved = rcConfigSaveLFS();
              // Apply baud changes live (Serial2/3/4) so HCR / MP3 / Maestro
              // pick up a new rate immediately — no reboot required.
              applySerialBauds(false);
              // rcConfigSaveLFS() (flash write) + applySerialBauds()
              // block loop() for 100+ ms, during which processSbus() can't
              // run.  If the operator was holding a matrix button across
              // that gap, the frozen debounce state could produce a phantom
              // edge when loop() resumes.  Reset the matrix state machine to
              // a clean "must see a confirmed neutral, then a fresh press"
              // condition so the save can't manufacture a button event.
              matrixArmed        = false;
              matrixCandidate    = 0;
              matrixCandCount    = 0;
              matrixNeutralCount = 0;
              if (saved) Serial.println("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":true}");
              else       Serial.println("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":false,\"msg\":\"applied to RAM but could not be saved to flash (LittleFS write error)\"}");
            } else {
              Serial.println("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":false,\"msg\":\"config apply failed\"}");
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
          if (btn < 1 || btn > RC_NUM_THRESHOLDS || mode < 1 || mode > 3 || tap < 1 || tap > 3) {
            Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"bad mode/btn/tap\"}");
          } else {
            Serial.printf("[TRIGGER] mode=%d btn=%d tap=%d\n", mode, btn, tap);
            rcDispatch(mode * 100 + btn, (uint8_t)tap);
            Serial.println("{\"type\":\"ACK\",\"ok\":true}");
          }

        } else if (strcmp(type,"WCB_SEND")==0) {
          int target     = hdr["target"] | 0;
          const char* cmd = hdr["cmd"]   | "";
          if (!wcb || !wcbReady) {
            Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"WCB not ready (init failed)\"}");
          } else {
            if (target == 0)                               wcb->broadcast(cmd);
            else if (target >= 1 && target <= WCB_MAX_BOARDS) wcb->send((uint8_t)target, cmd);
            Serial.println("{\"type\":\"ACK\",\"ok\":true}");
          }

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
            case 1:  Serial.println("NaviCore — WCB HW 3.2"); break;
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
            // #L13 — Raw SBUS frame hex dump.  Shows exactly what bytes the
            // RX is delivering, with channel-decoding offsets annotated so
            // we can see at a glance whether bytes 23-33 (CH17-24 in SBUS-24)
            // carry data or are zero on the wire.
            case 13: {
              const uint8_t* raw = sbusRx.rawFrameBytes();
              uint8_t        len = sbusRx.rawFrameLen();
              if (len == 0) {
                Serial.println("---- SBUS RAW ---- (no frame parsed yet)");
                break;
              }
              Serial.printf("---- SBUS RAW ---- (%u bytes, %s)\n",
                            len, len == 25 ? "SBUS-16" :
                                 len == 36 ? "SBUS-24" : "unknown length");
              for (uint8_t i = 0; i < len; i++) {
                if (i % 8 == 0) Serial.printf("  [%2u] ", i);
                Serial.printf("%02X ", raw[i]);
                if (i % 8 == 7) Serial.println();
              }
              if (len % 8) Serial.println();
              Serial.println("  byte 0       = header (expect 0F)");
              if (len == 25) {
                Serial.println("  bytes 1-22   = CH1-16 data");
                Serial.println("  byte 23      = flags");
                Serial.println("  byte 24      = footer (expect 00)");
              } else if (len == 36) {
                Serial.println("  bytes 1-22   = CH1-16 data");
                Serial.println("  bytes 23-33  = CH17-24 data  ← check these");
                Serial.println("  byte 34      = flags");
                Serial.println("  byte 35      = footer (expect 00)");
              }
              break;
            }
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
      // Must be able to hold a full SET_CONFIG payload. A config with
      // double/triple-tap mappings is tens of KB; the old 8 KB cap silently
      // truncated large configs mid-JSON, so deserializeJson() (bigDoc, 98304
      // in the SET_CONFIG handler above) failed with "parse failed" and the
      // ENTIRE save was rejected — nothing persisted and reload reverted every
      // edit. Keep this in sync with that bigDoc capacity.
      if (serialInputBuf.length() < 98304) serialInputBuf += c;
    }
  }
}

// =============================================================================
//  SETUP
// =============================================================================
// ── Cold-boot auto-recovery (boot guard) ───────────────────────────────────
// The custom short-watchdog bootloader (RTC WDT 9000 → 3000 ms) auto-resets a
// board that stalls in the pre-app boot window — but IDF disables that RTC WDT
// right before setup() runs, so a stall INSIDE setup() (PSRAM alloc, WiFi/USB
// bring-up current spike on a cold rail) would otherwise sit dark until someone
// presses reset. This one-shot esp_timer fires if setup() hasn't finished
// within BOOT_GUARD_TIMEOUT_MS and restarts the board, so a cold boot
// auto-retries. The callback runs in the esp_timer task — independent of the
// loop task running setup() — so a hung setup() can't stop it. Cancelled at the
// end of a healthy setup(). The custom short-WDT bootloader and THIS guard are a
// matched pair: never run that bootloader on a board without this guard.
#define BOOT_GUARD_TIMEOUT_MS 15000
static esp_timer_handle_t _bootGuardTimer = nullptr;
static void _bootGuardFired(void*) { ESP.restart(); }

static void bootGuardArm() {
  esp_timer_create_args_t args = {};
  args.callback        = &_bootGuardFired;
  args.arg             = nullptr;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name            = "bootguard";
  if (esp_timer_create(&args, &_bootGuardTimer) == ESP_OK) {
    esp_timer_start_once(_bootGuardTimer, (uint64_t)BOOT_GUARD_TIMEOUT_MS * 1000ULL);
  }
}

static void bootGuardDisarm() {
  if (_bootGuardTimer) {
    esp_timer_stop(_bootGuardTimer);
    esp_timer_delete(_bootGuardTimer);
    _bootGuardTimer = nullptr;
  }
}

// Report at boot which 2nd-stage bootloader is on the board (stock vs the custom
// short-WDT one). Reads the esp_bootloader_desc_t in IDF 5.2+ bootloader images —
// no flash dump. The custom bootloaders are identified by build timestamp; if
// either is rebuilt, add its new date here (read it from this very banner).
static void printBootloaderInfo() {
  static const char *CUSTOM_BOOT_DATES[] = {
    "Jun  8 2026 16:02:21",   // WCB_S3_custom_bootloader_16MB_wdt3s.bin
    "Jun 10 2026 14:36:20",   // WCB_S3_custom_bootloader_8MB_wdt3s.bin
  };
  esp_bootloader_desc_t desc;
  if (esp_ota_get_bootloader_description(NULL, &desc) == ESP_OK) {
    bool custom = false;
    for (size_t i = 0; i < sizeof(CUSTOM_BOOT_DATES) / sizeof(CUSTOM_BOOT_DATES[0]); i++)
      if (strncmp(desc.date_time, CUSTOM_BOOT_DATES[i], sizeof(desc.date_time)) == 0) { custom = true; break; }
    if (custom)
      Serial.printf("Bootloader: CUSTOM short-WDT (cold-boot auto-retry) — built %s\n", desc.date_time);
    else
      Serial.printf("Bootloader: stock (IDF %s, built %s)\n", desc.idf_ver, desc.date_time);
  } else {
    Serial.println("Bootloader: unknown (no description block)");
  }
}

// ── Boot telemetry ──────────────────────────────────────────────────────────
// Boot-attempt counter in RTC noinit RAM: survives watchdog/software/panic
// resets (and usually the reset button); garbage only after true power loss —
// the magic word detects that and restarts the count. After a "dark board"
// episode this tells you whether the chip had been reset-looping through the
// app (count climbing), brown-outing (RTC code 15), or never reached app code
// at all (count restarts at 1).
#define BOOT_MAGIC 0xB007C0DEUL
RTC_NOINIT_ATTR static uint32_t g_bootMagic;
RTC_NOINIT_ATTR static uint32_t g_bootAttempts;

static void printBootTelemetry() {
  esp_reset_reason_t r = esp_reset_reason();
  const char *name = "other";
  switch (r) {
    case ESP_RST_POWERON:  name = "Power-on / EN reset"; break;
    case ESP_RST_SW:       name = "Software restart (incl. boot-guard retry)"; break;
    case ESP_RST_PANIC:    name = "Crash (panic)"; break;
    case ESP_RST_INT_WDT:  name = "Interrupt watchdog"; break;
    case ESP_RST_TASK_WDT: name = "Task watchdog"; break;
    case ESP_RST_WDT:      name = "RTC watchdog (short-WDT bootloader fired)"; break;
    case ESP_RST_BROWNOUT: name = "BROWNOUT — supply rail sagged"; break;
    default: break;
  }
  // Low-level per-core causes (rom/rtc.h). Key S3 codes:
  //   1 = power-on   15 = RTC-WDT brown-out   16 = RTC-WDT system reset
  //   (16 = the short-WDT bootloader's 3 s watchdog fired — auto-retry)
  Serial.printf("Reset reason: %d - %s  (RTC codes core0=%d core1=%d)\n",
                (int)r, name, (int)rtc_get_reset_reason(0), (int)rtc_get_reset_reason(1));
  if (g_bootMagic != BOOT_MAGIC) {          // true power loss → fresh count
    g_bootMagic = BOOT_MAGIC;
    g_bootAttempts = 0;
  }
  g_bootAttempts++;
  Serial.printf("Boot attempts since power applied: %lu%s\n",
                (unsigned long)g_bootAttempts,
                g_bootAttempts > 1 ? "   <-- board retried/reset before this boot" : "");
}

void setup() {
  // Arm the boot guard FIRST so it covers all of setup() (PSRAM/WiFi/USB
  // bring-up). Disarmed at the very end once the board is confirmed healthy.
  bootGuardArm();

  // Bump the USB-CDC RX buffer well above the 256-byte default. The SBUS OUT
  // byte-streaming tee blocks the main loop for ~2.75 ms per SBUS frame while
  // bit-banging SoftwareSerial; during that window the host can shove ~1-2 KB
  // into us at USB-CDC speed. A 4 KB buffer comfortably absorbs the worst case
  // (e.g. a 3-4 KB SET_CONFIG payload arriving in one shot) without dropping
  // bytes that would otherwise corrupt the JSON. Must be set BEFORE begin().
  Serial.setRxBufferSize(4096);
  // TX ring buffer — sized to hold an entire CONFIG response (rcConfigToJSON
  // can produce 2-4 KB depending on how populated the config is) in one
  // print() so the host has a full window to drain it before any byte gets
  // dropped.  Previously 1024 — too small; a fast Serial.println of the full
  // config overflowed mid-stream and the corrupt JSON appeared at the
  // browser as a truncated line (e.g. "matrixChannel" → "matrixCel" gap).
  // 8 KB is comfortable on the S3's 512 KB SRAM.  Must be set BEFORE begin().
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  // ── ESP32-S3 USB-CDC TX-blocking guard (only when Serial = HWCDC) ──
  // If the board variant is built with ARDUINO_USB_CDC_ON_BOOT=1, Serial is
  // the native-USB HWCDC class.  Default tx timeout is ~100 ms which can
  // make the firmware appear frozen if no host ever attaches — but the
  // OTHER extreme (timeout = 0) drops bytes immediately when the host is
  // just briefly slow to drain, which mangles the CONFIG response.
  //
  // 50 ms is a deliberate middle ground:
  //   • host present + reading at any reasonable speed → buffer drains
  //     within microseconds, the 50 ms ceiling never gets hit, no drops
  //   • host absent / disappeared mid-write → writes give up after 50 ms
  //     instead of blocking the loop indefinitely
  //   • combined with the 8 KB tx buffer above, a full config print fits
  //     entirely in the buffer before any wait is needed
  //
  // When CDC-on-boot is DISABLED, Serial is HardwareSerial (UART0 through
  // a USB-to-serial bridge) and setTxTimeoutMs doesn't exist — the boot
  // latch in that case is the bridge chip's DTR/RTS autoreset circuit,
  // which is a hardware issue software can't directly suppress.
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(50);
#endif
  delay(1500);
  Serial.println("\n\n=== NaviCore (WCB HW 3.2) ===");
  printBootloaderInfo();
  printBootTelemetry();

  // Status LED
  statusLed.begin();
  setStatusLed(C_RED, 255);

  // Serial2 (local Maestro bus) and Serial3-4 (aux SoftwareSerial) are opened
  // AFTER the config loads, below, at their configured baud (rcConfig.maestroBaud
  // / rcConfig.auxBaud). Nothing uses them before setup() finishes, so
  // deferring the open is safe.

  // SBUS reader on Serial1 (UART1) — RX = SBUS_RX_PIN, TX disabled (-1).
  // SBUS RX and SBUS OUT now live on SEPARATE hardware UARTs (see the
  // dedicated UART0 block just below), so Serial1 is RX-only and never
  // shares its FIFO with the re-emit path.
  sbusRx.begin(&Serial1, SBUS_RX_PIN, /*txPin=*/-1);

  // ── SBUS OUT on a DEDICATED hardware UART (UART0 / Serial0) ──────────────
  // See the SBUS_RX_PIN comment block above for the three-attempt history.
  // Short version: bit-banging (v1) and sharing Serial1's TX (v2) both
  // failed; UART0 became free when the debug console moved to native USB
  // CDC, so SBUS OUT now owns a real hardware UART — TX-only, GPIO9,
  // 100k 8E2 INVERTED (matching the SBUS line format), no RX pin.
  //
  // A 256-byte TX ring buffer (≈7 frames) guarantees the byte-tee write in
  // SbusReader::read() never blocks loop(): frames drain at 100k baud
  // (~3-4 ms each) far faster than they arrive (~7-14 ms), so the buffer
  // never fills.  Non-blocking TX is what keeps this off the watchdog.
  // setTxBufferSize() MUST be called before begin().
  Serial0.setTxBufferSize(256);
  Serial0.begin(100000, SERIAL_8E2, /*rxPin=*/-1, /*txPin=*/SBUS_OUT_PIN, /*invert=*/true);
  sbusRx.setPassthroughSink(&Serial0);
  Serial.printf("[SBUS] IN  on Serial1/UART1 RX (GPIO%d)\n", SBUS_RX_PIN);
  Serial.printf("[SBUS] OUT on Serial0/UART0 TX (GPIO%d) — 100k 8E2 inverted, byte passthrough\n", SBUS_OUT_PIN);

  // RC Config lives in PSRAM — allocate it BEFORE any rcConfig.* access.
  // ps_calloc() pulls from the PSRAM heap (Tools → PSRAM must be "OPI PSRAM").
  // If PSRAM is unavailable the ~210 KB struct can't fit internal RAM, so halt
  // with a clear message instead of crashing cryptically on first access.
  g_rcConfig = (RcConfig*) ps_calloc(1, sizeof(RcConfig));
  if (!g_rcConfig) {
    Serial.println("\n[FATAL] PSRAM allocation for rcConfig failed.");
    Serial.println("        Set Arduino IDE: Tools -> PSRAM -> \"OPI PSRAM\", then reflash.");
    setStatusLed(C_RED, 255);
    while (true) { delay(1000); }
  }
  Serial.printf("[MEM] rcConfig (%u bytes) allocated in PSRAM, free PSRAM now %u\n",
                (unsigned)sizeof(RcConfig), (unsigned)ESP.getFreePsram());

  // RC Config: defaults then NVS overrides. Must run BEFORE constructing
  // WCB_Client so the network credentials come from rcConfig.wcbNetwork.
  rcConfigLoadDefaults();
  // Config persistence: LittleFS (/config.json) is primary; the legacy NVS store
  // is a one-time migration source. LittleFS removes the NVS 4000-byte-per-value
  // limit that silently dropped densely-mapped modes.
  rcConfigBeginLFS();
  if (!rcConfigLoadLFS()) {
    if (g_lfsReady && LittleFS.exists(RC_CFG_PATH)) {
      // The file EXISTS but didn't load (parse error / transient low memory).
      // Do NOT migrate-and-save here — that would overwrite a good config with
      // defaults. Keep the file and run on defaults this boot; retry next boot.
      Serial.println("[CONFIG] /config.json present but unreadable — kept; running on defaults this boot");
    } else {
      // No file yet — fresh board, or first boot after the LittleFS upgrade.
      // Load the legacy NVS config and migrate it forward (once) so the next
      // boot reads LittleFS.
      rcConfigLoadNVS();
      if (g_lfsReady && rcConfigSaveLFS())
        Serial.println("[CONFIG] migrated existing config: NVS -> LittleFS");
    }
  }

  // Open Serial2 (local Maestro) + Serial3/4 (aux SWSerial) at their
  // configured baud. Single source of truth: rcConfig (maestroBaud /
  // auxBaud[]). Same helper runs again after every SET_CONFIG save so a
  // baud change in the config tool applies live — no reboot needed.
  // Keep aux baud ≤ ~57600 (higher rates choke bit-banged SWSerial on
  // ESP32-S3). One port = one device = one baud.
  applySerialBauds(true);

  // Initialize rc_telemetry's deferred-queue mutex BEFORE WCB_Client
  // brings the ESP-NOW receive callback online.  Otherwise the very
  // first inbound packet's handle() call would find _pendingMutex==null
  // and skip its critical section.  See rc_telemetry.h::init() for the
  // race details.
  rcTelemetry::init();

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
    // Latch the fault for the LED arbiter: updateStatusLed() (loop) displays a
    // latched fault color STEADY and skips the SBUS indication entirely, so
    // this isn't silently overwritten on the first loop() pass. Steady orange
    // = WCB fault; FLASHING orange = no SBUS — distinguishable at a glance.
    g_ledFaultColor = C_ORANGE;
    setStatusLed(C_ORANGE, 200);   // show immediately for the rest of setup()
  } else {
    wcbReady = true;   // ESP-NOW is up — wcb-> calls are now safe
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
  Serial.print  ("[NaviCore] Firmware ");
  Serial.print  (FW_VERSION);
  Serial.println(" — setup complete.");
  Serial.println("  Connect config_tool/index.html via Web Serial for configuration.");
  Serial.println("  CLI: #L01=info  #L09=SBUS dump  #L10=live  #L11=WCB status  #L12=RC state  #L13=SBUS raw hex");
  Serial.println("  CLI: #L20=HCR S3 test  #L21=HCR S4 test  (direct, bypasses config+mapping)");
  Serial.println("  Send PING to test. Send GET_CONFIG to read mappings.");

  // setup() completed — cancel the boot guard so a healthy board never trips it.
  bootGuardDisarm();
}

// =============================================================================
//  LOOP
// =============================================================================
// ── Status LED: SBUS-aware running indicator ────────────────────────────────
// Once the board is up, the LED reflects SBUS reception:
//   • frames arriving        → steady BLUE  (ready / receiving SBUS)
//   • no frames (no signal)  → slow-flash ORANGE
// Non-blocking: toggles on a millis() timer so loop()/SBUS dispatch never stall,
// and only writes the NeoPixel on a state change or flash edge (not every loop).
// Boot/fault colours (RED, WCB-init-fail ORANGE) are set in setup(); this takes
// over on the first loop(), so once running, FLASHING orange == "no SBUS".
#define SBUS_LED_TIMEOUT_MS 500   // no SBUS frame for this long ⇒ "no signal"
#define SBUS_LED_FLASH_MS   600   // slow-flash half-period (ORANGE on, then off)
#define STATUS_LED_BRIGHT   12    // running-indicator brightness (0-255). Kept low —
                                  // the NeoPixel sits right on the board; bump if you
                                  // want it more visible across a room.
static void updateStatusLed() {
  unsigned long now = millis();
  static int8_t        mode       = -1;   // -1 unset, 0 steady-blue, 1 flashing-orange, 2 latched-fault
  static bool          flashOn    = false;
  static unsigned long lastToggle = 0;

  // Latched fault outranks the routine SBUS indication (see g_ledFaultColor).
  // Shown STEADY — distinct from the FLASHING no-SBUS pattern below.
  if (g_ledFaultColor) {
    if (mode != 2) { setStatusLed(g_ledFaultColor, STATUS_LED_BRIGHT); mode = 2; }
    return;
  }

  bool sbusAlive = (sbusLastFrameMs != 0) && (now - sbusLastFrameMs < SBUS_LED_TIMEOUT_MS);
  if (sbusAlive) {
    if (mode != 0) { setStatusLed(C_BLUE, STATUS_LED_BRIGHT); mode = 0; }   // receiving → steady blue
  } else if (mode != 1) {                                                   // just lost / never had SBUS
    mode = 1; flashOn = true; lastToggle = now; setStatusLed(C_ORANGE, STATUS_LED_BRIGHT);
  } else if (now - lastToggle >= SBUS_LED_FLASH_MS) {                       // slow flash
    lastToggle = now; flashOn = !flashOn;
    setStatusLed(flashOn ? C_ORANGE : C_OFF, flashOn ? STATUS_LED_BRIGHT : 0);
  }
}

void loop() {
  // WCB — heartbeats, ACKs, WCBStream flushes
  if (wcb && wcbReady) wcb->update();

  // WCB-network telemetry bridge — periodic rc_hb (0.5 Hz) + rc_ch (5 Hz)
  // broadcasts so the config tool's "Via WCB" mode can discover and live-
  // monitor this RC.  Event-driven rc_trig / rc_mode are emitted from
  // rcDispatch() and the mode-decode block in processSbus(), respectively.
  rcTelemetry::tick();

  // SBUS
  processSbus();
  checkDeferredTap();

  // Status LED: steady BLUE while receiving SBUS, slow-flash ORANGE when not
  updateStatusLed();

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
