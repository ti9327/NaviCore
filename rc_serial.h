#pragma once
#include <Arduino.h>
#include <HWCDC.h>

// ─────────────────────────────────────────────────────────────────────────────
//  RcSerial — USB-CDC tee for the remote-terminal feature
// ─────────────────────────────────────────────────────────────────────────────
// NaviCore's CLI (#L diagnostics, ?OTALOCAL,*, ?OTA,*) prints to the global
// `Serial`.  To mirror that output back over the WCB ESP-NOW bridge — so the
// config tool's terminal can show NaviCore's CLI alongside the WCB's — we
// interpose a thin Stream that ALWAYS forwards to the real USB-CDC
// (HWCDCSerial) and ALSO copies writes to a capture sink while a remote
// command is running.  This mirrors the WCB firmware's `class WCBSerial` +
// `#define Serial` trick, adapted for the ESP32-S3 hardware USB-CDC.
//
// Capturing at the Serial layer means EVERY existing Serial.printf in the CLI
// (and in navicore_ota.h) is mirrored automatically — no Print& has to be
// threaded through the dozens of print sites.
//
// Include order matters: this header MUST come AFTER the Arduino core headers
// (so `Serial` is already the `#define Serial HWCDCSerial` from
// HardwareSerial.h) and BEFORE any project header that prints, so their
// Serial.* calls route through the tee.
class RcSerial : public Stream {
 public:
  // ── HWCDC-specific pass-throughs used by NaviCore setup() ────────────────
  void begin(unsigned long baud)   { HWCDCSerial.begin(baud); }
  void setRxBufferSize(size_t s)   { HWCDCSerial.setRxBufferSize(s); }
  void setTxBufferSize(size_t s)   { HWCDCSerial.setTxBufferSize(s); }
  void setTxTimeoutMs(uint32_t ms) { HWCDCSerial.setTxTimeoutMs(ms); }

  // ── Stream interface → real USB ──────────────────────────────────────────
  int  available() override { return HWCDCSerial.available(); }
  int  read()      override { return HWCDCSerial.read(); }
  int  peek()      override { return HWCDCSerial.peek(); }
  void flush()     override { HWCDCSerial.flush(); }

  // ── Print interface → USB pass-through ALWAYS, plus capture when armed ────
  // Capture is gated to the core that armed it so a Core-0 (WiFi/ESP-NOW)
  // print landing mid-command can't race the single-threaded sink buffer.
  size_t write(uint8_t c) override {
    Print* cap = _cap;
    if (cap && xPortGetCoreID() == _capCore) cap->write(c);
    return HWCDCSerial.write(c);
  }
  size_t write(const uint8_t* buf, size_t n) override {
    Print* cap = _cap;
    if (cap && xPortGetCoreID() == _capCore) cap->write(buf, n);
    return HWCDCSerial.write(buf, n);
  }
  using Print::write;
  operator bool() { return (bool)HWCDCSerial; }

  // ── Capture control (driven by the remote-CLI drain in loop()) ───────────
  void armCapture(Print* sink) { _capCore = xPortGetCoreID(); _cap = sink; }
  void disarmCapture()         { _cap = nullptr; }

 private:
  Print* _cap     = nullptr;
  int    _capCore = -1;
};

extern RcSerial rcSerial;

#undef Serial
#define Serial rcSerial
