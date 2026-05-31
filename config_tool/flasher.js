// ════════════════════════════════════════════════════════════════
//  NaviCore Firmware Flasher
//
//  Browser-based ESP32-S3 flasher for the WCB v3.2 hardware variant.
//  This is a simplified port of the Wireless_Communication_Board-WCB
//  Wizard flasher (same upstream library, same flash addresses,
//  same NVS-preservation logic) — but stripped to a single board
//  variant since the NaviCore firmware only targets WCB v3.2.
//
//  Uses esptool-js (Espressif's official browser flash library)
//  loaded on-demand from CDN, plus CryptoJS for MD5 verification.
//
//  Firmware binaries are hosted in the NaviCore repo under
//  /firmware/ on GitHub.  The Contents API is used to list the
//  directory so a freshly built binary with a versioned filename
//  (e.g. NaviCore_201500RMAY26_ESP32S3.bin) is picked up
//  automatically — only the suffix needs to be stable.
//
//  Public surface:
//    flashFirmware(port, callbacks)  → Promise<void>
//
//  port is a WebSerial SerialPort that MUST be closed before calling.
// ════════════════════════════════════════════════════════════════

const ESPTOOL_CDN  = 'https://cdn.jsdelivr.net/npm/esptool-js@0.4.7/+esm';
const CRYPTOJS_CDN = 'https://cdnjs.cloudflare.com/ajax/libs/crypto-js/4.2.0/crypto-js.min.js';

// ── Firmware source config ───────────────────────────────────────
// Binaries live in /firmware/ on GitHub with versioned names ending
// in one of three stable suffixes:
//   _ESP32S3.bin       — application image           → 0x10000
//   _ESP32S3_boot.bin  — second-stage bootloader     → 0x0
//   _ESP32S3_part.bin  — partition table            → 0x8000
//
// Only the application image is required.  If boot+part are missing
// we fall back to an app-only flash (works on already-programmed
// boards; blank boards need a full set).
const GITHUB_OWNER          = 'greghulette';
const GITHUB_REPO           = 'NaviCore';
const GITHUB_BRANCH_DEFAULT = 'main';
const GITHUB_BIN_PATH       = 'firmware';

// Branch override (Advanced/dev only) — set via localStorage to test
// unreleased branches without recompiling the page.
function getFirmwareBranch() {
  try {
    const b = (localStorage.getItem('rc_fw_branch') || '').trim();
    return b || GITHUB_BRANCH_DEFAULT;
  } catch (_) {
    return GITHUB_BRANCH_DEFAULT;
  }
}

// ── Script loader ────────────────────────────────────────────────
function loadScript(src) {
  return new Promise((resolve, reject) => {
    if (document.querySelector(`script[src="${src}"]`)) { resolve(); return; }
    const s    = document.createElement('script');
    s.src      = src;
    s.onload   = resolve;
    s.onerror  = () => reject(new Error(`Failed to load CDN script: ${src}`));
    document.head.appendChild(s);
  });
}

// ── Binary fetching ──────────────────────────────────────────────
// Returns [{ buf, address }, ...] in ascending-address order.
//
// Flash map (ESP32-S3, min_spiffs partition scheme):
//   boot   → 0x0       (bootloader)
//   part   → 0x8000    (partition table)
//   nvs    → 0x9000    (NVS — NOT touched here so saved config survives)
//   ota_0  → 0x10000   (application)
async function fetchFirmwareImages(onLog) {
  const branch = getFirmwareBranch();
  const apiUrl = `https://api.github.com/repos/${GITHUB_OWNER}/${GITHUB_REPO}/contents/${GITHUB_BIN_PATH}?ref=${branch}`;

  if (branch !== GITHUB_BRANCH_DEFAULT)
    onLog(`⚠ Firmware source branch: ${branch} (not the released 'main')`);
  onLog(`Scanning ${GITHUB_BIN_PATH}/ on GitHub (${branch})…`);

  const listResp = await fetch(apiUrl);
  if (!listResp.ok) throw new Error(`GitHub API: HTTP ${listResp.status}`);
  const files = await listResp.json();

  async function fetchBySuffix(suffix, required) {
    const match = files.find(f => f.type === 'file' && f.name.endsWith(suffix));
    if (!match) {
      if (required) throw new Error(`No file ending with "${suffix}" found in ${GITHUB_BIN_PATH}/`);
      return null;
    }
    onLog(`Found: ${match.name}`);
    const r = await fetch(match.download_url);
    if (!r.ok) throw new Error(`HTTP ${r.status} fetching ${match.name}`);
    const buf = await r.arrayBuffer();
    if (buf.byteLength === 0) throw new Error(`${match.name} is empty`);
    return buf;
  }

  // App is required; boot + part are optional (paired — either both or neither).
  const appBuf = await fetchBySuffix('_ESP32S3.bin', true);
  const images = [{ buf: appBuf, address: 0x10000 }];

  let hasBootPart = true;
  try {
    const [bootBuf, partBuf] = await Promise.all([
      fetchBySuffix('_ESP32S3_boot.bin', true),
      fetchBySuffix('_ESP32S3_part.bin', true),
    ]);
    images.unshift(
      { buf: bootBuf, address: 0x0    },
      { buf: partBuf, address: 0x8000 },
    );
  } catch (_) {
    hasBootPart = false;
    onLog('Note: bootloader/partition files not on GitHub — flashing app only.');
    onLog('      Blank boards will need a one-time full flash via Arduino IDE.');
  }

  const totalKB = Math.round(images.reduce((s, i) => s + i.buf.byteLength, 0) / 1024);
  onLog(`Loaded ${totalKB} KB (${hasBootPart ? 'boot + partitions + app' : 'app only'})`);
  return images;
}

// ── Buffer → Latin1 string (esptool-js wants strings, not Uint8Array) ─
function bufToLatin1(buf) {
  const u8 = new Uint8Array(buf);
  let s = '';
  const CHUNK = 65536;
  for (let i = 0; i < u8.length; i += CHUNK) {
    s += String.fromCharCode.apply(null, u8.subarray(i, i + CHUNK));
  }
  return s;
}

// Windows detection — CP210x driver has a known issue with mid-flash
// baud-rate changes, so we keep 115200 there.  Mac/Linux can use 460800.
const _isWindowsPlatform = /Win/i.test(navigator.platform || '');

// ════════════════════════════════════════════════════════════════
//  flashFirmware
//
//  port      — WebSerial SerialPort, MUST be CLOSED before calling
//  options   — {
//                onProgress(written, total),
//                onLog(msg),
//                onStatus(msg),
//                eraseNvs  : bool   — true → full wipe (factory state)
//              }
//
//  Modes:
//    eraseNvs = false  (default — "Update" path):
//      Smart auto-detect, NVS preserved.
//        • Blank board (magic 0xFF)       → full flash (boot + part + app)
//        • Existing firmware (magic 0xE9) → app-only flash, NVS preserved
//        • Partition-table mismatch       → full flash, NVS preserved
//        • Anything else / read failure   → full flash, NVS preserved
//
//    eraseNvs = true   ("Full Wipe / Initial Push" path):
//      Unconditional full flash + erase NVS (0x9000, 20 KB) + erase
//      otadata (0xE000, 8 KB).  Use for first-time programming, recovery
//      from a bricked board, or whenever a factory-fresh config is wanted.
//      Skip detection entirely — we don't care what was there before.
// ════════════════════════════════════════════════════════════════
async function flashFirmware(port, { onProgress, onLog, onStatus, eraseNvs = false }) {

  // ── Step 1: load CDN dependencies ──────────────────────────────
  onStatus('Loading flash tool…');
  onLog('Loading CryptoJS…');
  try { await loadScript(CRYPTOJS_CDN); }
  catch (e) { throw new Error(`Could not load CryptoJS from CDN — are you online?\n${e.message}`); }

  onLog('Loading esptool-js…');
  let ESPLoader, Transport;
  try { ({ ESPLoader, Transport } = await import(ESPTOOL_CDN)); }
  catch (e) { throw new Error(`Could not load esptool-js from CDN — are you online?\n${e.message}`); }
  onLog('Flash tool loaded.');

  // ── Step 2: fetch firmware ──────────────────────────────────────
  onStatus('Downloading firmware…');
  let flashImages;
  try { flashImages = await fetchFirmwareImages(onLog); }
  catch (e) { throw new Error(`Firmware download failed: ${e.message}`); }

  // ── Step 3: connect to ESP bootloader ──────────────────────────
  onStatus('Connecting to bootloader…');
  onLog('Connecting to ESP32-S3 bootloader…');
  onLog('► Hold BOOT, tap RST, release BOOT — then watch for sync below.');

  // Route esptool-js internal messages into our log so the user can see sync progress.
  const terminal = {
    clean:     ()    => {},
    writeLine: (msg) => { if (msg?.trim()) onLog(`[esptool] ${msg.trim()}`); },
    write:     (msg) => { if (msg?.trim()) onLog(`[esptool] ${msg.trim()}`); },
  };

  const transport = new Transport(port, false);
  const loader    = new ESPLoader({
    transport,
    baudrate:    _isWindowsPlatform ? 115200 : 460800,
    romBaudrate: 115200,
    enableTracing: false,
    terminal,
  });

  let chip;
  try {
    chip = await loader.main();
    onLog(`Chip identified: ${chip}`);
  } catch (e) {
    try { await transport.disconnect(); } catch (_) {}
    throw new Error(
      `Bootloader connection failed: ${e.message}\n\n` +
      `To enter bootloader mode: hold BOOT, press RST, release BOOT, then click Flash again.`
    );
  }

  // ── Step 3b: decide what to flash ──────────────────────────────
  let imagesToFlash;

  if (eraseNvs) {
    // Full wipe / initial push: always write the full image, regardless of
    // what's on the board. We'll prepend NVS + otadata erase blobs below.
    imagesToFlash = flashImages.slice();
    onLog('Full wipe — flashing bootloader + partitions + app (NVS will be erased).');
  } else {
    // Smart auto-detect: NVS-preserving update.
    let forceFull = false;
    let isBlankBoard = false;
    try {
      const readFlashFn = loader.readFlash ?? loader.read_flash;
      if (typeof readFlashFn === 'function') {
        const sample = await readFlashFn.call(loader, 0x0, 4);
        const view   = new DataView(sample.buffer ?? sample);
        const magic  = view.getUint8(0);
        // 0xFF = blank, 0xE9 = valid ESP image, anything else = corrupted
        if (magic === 0xFF) {
          isBlankBoard = true;
          onLog('Blank board detected — will flash bootloader + partitions + app.');
          onLog('(Tip: for blank/factory-fresh boards, the "Full Wipe" option also clears NVS.)');
        } else if (magic === 0xE9) {
          // Existing firmware. App-only is the fast path — but only if the on-flash
          // partition table matches what we're about to write. Otherwise a layout
          // change (e.g. min_spiffs vs. default) would overrun. Compare and, on
          // mismatch, force a full flash. NVS at 0x9000 is never written here.
          const partImg = flashImages.find(img => img.address === 0x8000);
          if (!partImg) {
            onLog('Existing firmware detected — app-only flash (NVS preserved).');
          } else {
            try {
              const CMP_LEN = 0x200;  // covers all partition-table entries before MD5/padding
              const want = new Uint8Array(partImg.buf, 0, Math.min(CMP_LEN, partImg.buf.byteLength));
              const got0 = await readFlashFn.call(loader, 0x8000, want.length);
              const got  = got0 instanceof Uint8Array ? got0 : new Uint8Array(got0.buffer ?? got0);
              let same = got.length >= want.length;
              for (let i = 0; same && i < want.length; i++) if (got[i] !== want[i]) same = false;
              if (same) {
                onLog('Existing firmware, partition table matches — app-only flash (NVS preserved).');
              } else {
                forceFull = true;
                onLog('⚠ Partition table changed — full flash required (NVS preserved).');
              }
            } catch (_) {
              forceFull = true;
              onLog('Could not read partition table — forcing a safe full flash (NVS preserved).');
            }
          }
        } else {
          forceFull = true;
          onLog(`Corrupted bootloader (0x${magic.toString(16).padStart(2,'0')}) — will full-flash to recover.`);
        }
      } else {
        forceFull = true;
        onLog('Cannot read flash on this esptool-js version — forcing a safe full flash.');
      }
    } catch (_) {
      forceFull = true;
      onLog('Flash read check failed — forcing a safe full flash (NVS preserved).');
    }

    imagesToFlash = (isBlankBoard || forceFull)
      ? flashImages.slice()
      : flashImages.filter(img => img.address === 0x10000);
  }

  // ── Step 3c: optionally prepend NVS + otadata erase images ────
  // NaviCore partition layout (PartitionScheme=min_spiffs):
  //   nvs     @ 0x9000,  size 0x5000 (20 KB)
  //   otadata @ 0xE000,  size 0x2000  (8 KB — two 4 KB flash sectors)
  //   ota_0   @ 0x10000, size 0x1E0000 (~1.9 MB)
  //   ota_1   @ 0x1F0000
  //
  // Writing 0xFF buffers causes esptool to erase then rewrite those sectors,
  // returning them to factory-fresh state. Both otadata sectors MUST be
  // erased: if either still holds a stale OTA state pointing to ota_1, the
  // bootloader will try to boot ota_1, fail (nothing there after a fresh
  // flash to ota_0), and the OTA rollback watchdog fires — endless reboot loop.
  if (eraseNvs) {
    const nvsBlank     = new ArrayBuffer(0x5000);  // NVS: 20 KB @ 0x9000
    const otadataBlank = new ArrayBuffer(0x2000);  // otadata: 8 KB @ 0xE000
    new Uint8Array(nvsBlank).fill(0xFF);
    new Uint8Array(otadataBlank).fill(0xFF);
    // Insert in ascending address order, before the app images.
    imagesToFlash = [
      { buf: nvsBlank,     address: 0x9000 },
      { buf: otadataBlank, address: 0xE000 },
      ...imagesToFlash,
    ];
    onLog('NVS (0x9000, 20 KB) and OTA data (0xE000, 8 KB) will be erased.');
  }

  const totalBytes = imagesToFlash.reduce((sum, img) => sum + img.buf.byteLength, 0);

  // ── Step 4: write flash ────────────────────────────────────────
  onStatus(`Flashing ${chip}…`);
  onLog(`Writing ${Math.round(totalBytes / 1024)} KB across ${imagesToFlash.length} region(s)…`);
  onProgress(0, totalBytes);

  const writeFlashFn = loader.writeFlash ?? loader.write_flash;
  if (typeof writeFlashFn !== 'function') {
    try { await transport.disconnect(); } catch (_) {}
    throw new Error('esptool-js: writeFlash method not found — unexpected library version.');
  }

  let bytesWritten = 0;
  try {
    await writeFlashFn.call(loader, {
      fileArray: imagesToFlash.map(img => ({ data: bufToLatin1(img.buf), address: img.address })),
      flashSize: 'keep',
      flashMode: 'keep',
      flashFreq: 'keep',
      eraseAll:  false,
      compress:  true,
      reportProgress: (_fileIdx, written, total) => {
        // esptool-js resets written/total per file; accumulate for overall progress.
        onProgress(bytesWritten + written, totalBytes);
        if (written === total) bytesWritten += total;
      },
      calculateMD5Hash: (img) =>
        CryptoJS.MD5(CryptoJS.enc.Latin1.parse(img)).toString(),
    });
  } catch (e) {
    try { await transport.disconnect(); } catch (_) {}
    throw new Error(`Flash write failed: ${e.message}`);
  }

  // ── Step 5: reset into firmware ────────────────────────────────
  onLog('Resetting board into firmware…');
  onStatus('Resetting…');
  onProgress(totalBytes, totalBytes);

  const afterFlashFn = loader.afterFlash ?? loader.after_flash;
  try { if (afterFlashFn) await afterFlashFn.call(loader, 'hard_reset'); } catch (_) {}
  try { await transport.disconnect(); }                                   catch (_) {}

  onLog('Flash complete — board rebooting.');
  onStatus('Flash complete!');
}

// Expose globally so index.html's inline JS can call it without a module loader.
window.flashFirmware = flashFirmware;
