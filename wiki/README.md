# NaviCore Wiki — source pages

These Markdown files are the **GitHub Wiki** content for NaviCore. Each file is one wiki page.

## How to publish to the GitHub Wiki

A repo's wiki is its own git repository at `https://github.com/<you>/NaviCore.wiki.git`.

**Easiest (web UI):**
1. On GitHub, open your repo → **Wiki** tab → **Create the first page** (just save anything to initialize it).
2. For each `.md` file here, create a page whose **title matches the file name** with spaces where the hyphens are:
   - `Hardware-and-Wiring.md` → page title **“Hardware and Wiring”**
   - `Home.md` → the **Home** page
   - `_Sidebar.md` and `_Footer.md` are special — paste them into pages literally named `_Sidebar` and `_Footer`.
3. Paste the file contents and save.

**Faster (git):**
```bash
git clone https://github.com/<you>/NaviCore.wiki.git
cp -r wiki/*.md wiki/images NaviCore.wiki/
cd NaviCore.wiki
git add . && git commit -m "Add NaviCore wiki" && git push
```
GitHub turns `Hardware-and-Wiring.md` filenames into “Hardware and Wiring” page titles automatically, and `[[Page Name]]` links resolve to those pages.

> **Images:** the `images/` folder must be copied into the wiki repo too (the command above does this). Pages reference them as `![alt](images/banner.svg)`. GitHub renders committed SVGs in wikis. Diagrams: `banner.svg` (Home header), `signal-flow.svg` (Home), `pinmap.svg` (Hardware and Wiring).

## Pages

| File | Page |
|------|------|
| `Home.md` | Landing page + index |
| `_Sidebar.md` | Navigation sidebar (every page) |
| `_Footer.md` | Footer (every page) |
| `Hardware-and-Wiring.md` | Board, pin map, wiring |
| `Flashing-the-Firmware.md` | Flasher, Arduino IDE, CI, versioning |
| `Transmitter-Setup.md` | Models, channel map, model file |
| `Config-Tool-Guide.md` | Connecting, monitor, mapping, calibration, save/load |
| `Actions-Reference.md` | WCB / Maestro / HCR / MP3 / Serial actions |
| `WCB-Network.md` | ESP‑NOW credentials, device IDs, Via WCB |
| `Serial-JSON-Protocol.md` | USB JSON command table |
| `CLI-Commands.md` | `#Lxx` serial debug commands |
| `Troubleshooting.md` | Problem → fix |

> Internal links use GitHub Wiki `[[Double Bracket]]` syntax, so they only resolve once published to the wiki (not in the normal file browser).
