#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────
# build-firmware.sh
#
# Bash twin of tools/build-firmware.ps1.  Used by the GitHub Actions
# workflow (.github/workflows/build-firmware.yml); also runnable locally
# on macOS / Linux / WSL.  Windows users should keep using the .ps1 script.
#
# Reads the firmware version from fw_version.h (FW_VERSION_BASE +
# FW_VERSION_DTG), compiles RC-Controller.ino for the WCB v3.2 hardware
# (ESP32-S3), and drops the three artifacts into firmware/ with names
# matching the version baked into the binary:
#
#   firmware/RC-Controller_<TAG>_ESP32S3.bin
#   firmware/RC-Controller_<TAG>_ESP32S3_boot.bin
#   firmware/RC-Controller_<TAG>_ESP32S3_part.bin
#
# Requirements:
#   - arduino-cli on PATH
#   - esp32:esp32 core installed (e.g. esp32:esp32@3.3.4)
#   - All RC-Controller library deps installed (see the workflow file
#     for the canonical list)
# ─────────────────────────────────────────────────────────────────────────

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
FW_DIR="$REPO_ROOT/firmware"
FW_VER_FILE="$REPO_ROOT/fw_version.h"
SKETCH="$REPO_ROOT/RC-Controller.ino"

if [ ! -f "$SKETCH" ]; then
  echo "✗ Cannot find RC-Controller.ino at $SKETCH" >&2
  exit 1
fi
if [ ! -f "$FW_VER_FILE" ]; then
  echo "✗ Cannot find fw_version.h at $FW_VER_FILE" >&2
  exit 1
fi

# ── Parse FW_VERSION_BASE + FW_VERSION_DTG out of fw_version.h ──────────
# grep -oP needs GNU grep; the GitHub Actions ubuntu runner has it. macOS
# users without it should run with `gsed`-style alternatives via `sed`:
#   sed -nE 's|.*#define[[:space:]]+FW_VERSION_BASE[[:space:]]+"([^"]+)".*|\1|p'
FW_BASE=$(sed -nE 's|.*#define[[:space:]]+FW_VERSION_BASE[[:space:]]+"([^"]+)".*|\1|p' "$FW_VER_FILE" | head -1)
FW_DTG=$( sed -nE 's|.*#define[[:space:]]+FW_VERSION_DTG[[:space:]]+"([^"]+)".*|\1|p' "$FW_VER_FILE" | head -1)
if [ -z "${FW_BASE:-}" ] || [ -z "${FW_DTG:-}" ]; then
  echo "✗ Could not parse FW_VERSION_BASE / FW_VERSION_DTG from fw_version.h" >&2
  exit 1
fi
TAG="${FW_BASE}_${FW_DTG}"

echo ""
echo "Building RC-Controller firmware"
echo "  base : $FW_BASE   (edit fw_version.h to bump)"
echo "  dtg  : $FW_DTG   (auto-stamped on commit by pre-commit hook)"
echo "  tag  : $TAG"
echo ""

# ── Compile ─────────────────────────────────────────────────────────────
# FQBN deliberately mirrors what Arduino IDE produces with the
# "ESP32S3 Dev Module" board selected and NO menu options changed —
# all options at their defaults except PartitionScheme=min_spiffs.
# Critically this means USBMode=default + CDCOnBoot=default, i.e. Serial
# is UART0 (routed off-chip via the WCB v3.2's USB-to-UART bridge), NOT
# the ESP32-S3's native USB-Serial-JTAG controller.  Overriding to hwcdc
# routes Serial to native USB pins (GPIO19/20) which the WCB v3.2 board
# doesn't expose for debug — output silently goes nowhere.
FQBN="esp32:esp32:esp32s3:PartitionScheme=min_spiffs"
BUILD_DIR="${TMPDIR:-/tmp}/rc-fw-build"

rm -rf "$BUILD_DIR"
mkdir -p "$FW_DIR"

echo "→ arduino-cli compile  (this can take a minute)…"
arduino-cli compile \
  --fqbn "$FQBN" \
  --output-dir "$BUILD_DIR" \
  "$REPO_ROOT"

# ── Locate the three artifacts arduino-cli produced ─────────────────────
APP_SRC="$BUILD_DIR/RC-Controller.ino.bin"
BOOT_SRC="$BUILD_DIR/RC-Controller.ino.bootloader.bin"
PART_SRC="$BUILD_DIR/RC-Controller.ino.partitions.bin"

for f in "$APP_SRC" "$BOOT_SRC" "$PART_SRC"; do
  if [ ! -f "$f" ]; then
    echo "✗ Expected build artifact missing: $f" >&2
    exit 1
  fi
done

# ── Clean older versioned bins so firmware/ doesn't accumulate history ──
# The flasher only matches on suffix, so a single set is all that's needed.
find "$FW_DIR" -maxdepth 1 -type f -name 'RC-Controller_*_ESP32S3*.bin' -delete 2>/dev/null || true

# ── Copy with versioned names ───────────────────────────────────────────
APP_DST="$FW_DIR/RC-Controller_${TAG}_ESP32S3.bin"
BOOT_DST="$FW_DIR/RC-Controller_${TAG}_ESP32S3_boot.bin"
PART_DST="$FW_DIR/RC-Controller_${TAG}_ESP32S3_part.bin"

cp "$APP_SRC"  "$APP_DST"
cp "$BOOT_SRC" "$BOOT_DST"
cp "$PART_SRC" "$PART_DST"

echo ""
echo "✓ Built and staged:"
ls -lh "$FW_DIR"/RC-Controller_${TAG}_ESP32S3*.bin

rm -rf "$BUILD_DIR"
