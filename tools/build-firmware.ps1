# ─────────────────────────────────────────────────────────────────────────
# build-firmware.ps1
#
# Compile RC-Controller.ino for the WCB v3.2 hardware (ESP32-S3) and copy
# the three resulting artifacts (app, bootloader, partition table) into
# firmware/ with a DTG-stamped prefix so the in-browser flasher
# (config_tool/index.html → Config → Firmware tab) picks them up.
#
# Prereqs:
#   - arduino-cli on PATH  (https://arduino.github.io/arduino-cli/)
#   - esp32 core installed: arduino-cli core install esp32:esp32
#   - Any RC-Controller library dependencies installed
#
# Usage:
#   pwsh tools\build-firmware.ps1                     # default: tag = FW_VERSION from fw_version.h
#   pwsh tools\build-firmware.ps1 -Tag custom-name    # override the tag
#   pwsh tools\build-firmware.ps1 -KeepOld            # don't prune older bins
#
# Outputs (in firmware/):
#   RC-Controller_<TAG>_ESP32S3.bin
#   RC-Controller_<TAG>_ESP32S3_boot.bin
#   RC-Controller_<TAG>_ESP32S3_part.bin
#
# Default <TAG> reads FW_VERSION_BASE and FW_VERSION_DTG from fw_version.h
# and concatenates them with an underscore, e.g.  v0.1_211520QMAY26.
# This means the bin filename matches the version the firmware reports
# at runtime (PONG reply / boot banner) — single source of truth.
#
# The flasher matches by suffix only, so the <TAG> can be anything.
#
# This script does NOT commit or push.  Review in GitHub Desktop and
# commit when you're ready.
# ─────────────────────────────────────────────────────────────────────────

[CmdletBinding()]
param(
    [string] $Tag,
    [switch] $KeepOld
)

$ErrorActionPreference = 'Stop'

# ── Resolve repo root ────────────────────────────────────────────────────
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot  = Split-Path -Parent $ScriptDir
$FwDir     = Join-Path $RepoRoot 'firmware'
$Sketch    = Join-Path $RepoRoot 'RC-Controller.ino'

if (-not (Test-Path $Sketch)) {
    Write-Error "Could not find RC-Controller.ino at $Sketch"
    exit 1
}
if (-not (Test-Path $FwDir)) { New-Item -ItemType Directory -Path $FwDir | Out-Null }

# ── Verify arduino-cli is available ──────────────────────────────────────
$cli = Get-Command arduino-cli -ErrorAction SilentlyContinue
if (-not $cli) {
    Write-Error "arduino-cli not found on PATH.  Install from https://arduino.github.io/arduino-cli/ and run 'arduino-cli core install esp32:esp32' before retrying."
    exit 1
}

# ── Resolve tag from fw_version.h (single source of truth) ───────────────
# The header defines two values we care about:
#   #define FW_VERSION_BASE  "v0.1"
#   #define FW_VERSION_DTG   "211520QMAY26"
# Default Tag = "<BASE>_<DTG>", matching the FW_VERSION macro the firmware
# itself reports at runtime.  Caller can override with -Tag.
$FwVerFile = Join-Path $RepoRoot 'fw_version.h'
if (-not $Tag) {
    if (-not (Test-Path $FwVerFile)) {
        Write-Error "Cannot find fw_version.h at $FwVerFile.  Pass -Tag explicitly, or restore the file."
        exit 1
    }
    $fwSrc = Get-Content $FwVerFile -Raw
    $mBase = [regex]::Match($fwSrc, '#define\s+FW_VERSION_BASE\s+"([^"]+)"')
    $mDtg  = [regex]::Match($fwSrc, '#define\s+FW_VERSION_DTG\s+"([^"]+)"')
    if (-not $mBase.Success -or -not $mDtg.Success) {
        Write-Error "Could not parse FW_VERSION_BASE / FW_VERSION_DTG from fw_version.h."
        exit 1
    }
    $FwBase = $mBase.Groups[1].Value
    $FwDtg  = $mDtg.Groups[1].Value
    $Tag    = "${FwBase}_${FwDtg}"
    Write-Host ""
    Write-Host "Building RC-Controller firmware"
    Write-Host "  base : $FwBase   (edit fw_version.h to bump)"
    Write-Host "  dtg  : $FwDtg   (auto-stamped on commit by pre-commit hook)"
    Write-Host "  tag  : $Tag"
    Write-Host ""
} else {
    Write-Host ""
    Write-Host "Building RC-Controller firmware — explicit tag: $Tag"
    Write-Host ""
}

# ── Compile ──────────────────────────────────────────────────────────────
# FQBN matches the Arduino IDE board "ESP32S3 Dev Module" with USB CDC on
# boot enabled (required for the config-tool serial connection) and the
# Minimal SPIFFS partition scheme used by WCB v3.2.
$Fqbn     = 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,MSCOnBoot=default,DFUOnBoot=default,UploadMode=default,PartitionScheme=min_spiffs,CPUFreq=240,FlashMode=qio,FlashSize=4M,LoopCore=1,EventsCore=1,DebugLevel=none,EraseFlash=none'
$BuildDir = Join-Path ([System.IO.Path]::GetTempPath()) "rc-fw-build"

if (Test-Path $BuildDir) { Remove-Item -Recurse -Force $BuildDir }

Write-Host "→ arduino-cli compile  (this can take a minute)…"
& arduino-cli compile `
    --fqbn $Fqbn `
    --output-dir $BuildDir `
    $RepoRoot
if ($LASTEXITCODE -ne 0) {
    Write-Error "arduino-cli compile failed (exit $LASTEXITCODE)."
    exit $LASTEXITCODE
}

# ── Locate the three output bins ─────────────────────────────────────────
# arduino-cli names them after the sketch directory:
#   RC-Controller.ino.bin                   ← app
#   RC-Controller.ino.bootloader.bin        ← second-stage bootloader
#   RC-Controller.ino.partitions.bin        ← partition table
$AppSrc  = Join-Path $BuildDir 'RC-Controller.ino.bin'
$BootSrc = Join-Path $BuildDir 'RC-Controller.ino.bootloader.bin'
$PartSrc = Join-Path $BuildDir 'RC-Controller.ino.partitions.bin'

foreach ($f in @($AppSrc, $BootSrc, $PartSrc)) {
    if (-not (Test-Path $f)) {
        Write-Error "Expected build artifact missing: $f"
        exit 1
    }
}

# ── Optionally prune older bins so firmware/ doesn't accumulate history ──
if (-not $KeepOld) {
    $existing = Get-ChildItem -Path $FwDir -Filter 'RC-Controller_*_ESP32S3*.bin' -ErrorAction SilentlyContinue
    if ($existing) {
        Write-Host "→ Removing $($existing.Count) older bin(s) from firmware/ (use -KeepOld to retain)…"
        $existing | Remove-Item -Force
    }
}

# ── Copy with versioned names ────────────────────────────────────────────
$AppDst  = Join-Path $FwDir "RC-Controller_${Tag}_ESP32S3.bin"
$BootDst = Join-Path $FwDir "RC-Controller_${Tag}_ESP32S3_boot.bin"
$PartDst = Join-Path $FwDir "RC-Controller_${Tag}_ESP32S3_part.bin"

Copy-Item $AppSrc  $AppDst  -Force
Copy-Item $BootSrc $BootDst -Force
Copy-Item $PartSrc $PartDst -Force

$AppKB  = [math]::Round((Get-Item $AppDst).Length  / 1024)
$BootKB = [math]::Round((Get-Item $BootDst).Length / 1024)
$PartKB = [math]::Round((Get-Item $PartDst).Length / 1024)

Write-Host ""
Write-Host "✓ Built and staged:"
Write-Host "    firmware\$([System.IO.Path]::GetFileName($AppDst))    ($AppKB KB)"
Write-Host "    firmware\$([System.IO.Path]::GetFileName($BootDst))    ($BootKB KB)"
Write-Host "    firmware\$([System.IO.Path]::GetFileName($PartDst))    ($PartKB KB)"
Write-Host ""
Write-Host "Next steps:"
Write-Host "  1. Review the diff in GitHub Desktop."
Write-Host "  2. Commit + push to main."
Write-Host "  3. The Config tool's Firmware tab will pick them up automatically."
