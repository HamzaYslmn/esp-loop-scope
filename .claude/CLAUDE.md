# esp-loop-scope

Wi-Fi oscilloscope and event listener on a classic ESP32: one signal at 1 MS/s, a live scope
view and a log of detected bursts, all in a page served by the board.
GitHub: HamzaYslmn/esp-loop-scope.

## Naming

This is a general-purpose scope and event listener. Don't tie it to any vendor, product or
protocol in code, comments, UI or docs (no MaxLogic, VIP, fire-alarm wording).
`programmer.py` is the ESP32 flasher, not a device programmer.

## Layout

- `core/core.ino`: Wi-Fi, mDNS (`loop-scope.local`), TCP tuning, the web server (Core0Task).
- `core/src/LoopCapture.h`: capture module (`capture::setup()/loop()`), 1 MS/s ADC DMA on
  core 1, event detection, frame format and commands. `PROBE_PIN = 36` is the only place the
  pin is set.
- `core/data/index.html`: the whole web UI in **one file** (CSS + HTML + JS). Sections are
  grouped with `MARK:` comments. Components live in `LS.ui`. The board serves it gzipped
  from LittleFS. Don't split it into separate files; the user reversed that once already.
- `core/sketch.yaml`: pins the toolchain (esp32:esp32 3.3.12, WROOM-32 4 MB, default partitions).
- `core/secrets.h` (gitignored, copy from `secrets.h.example`): if present, the board joins
  that network only and never falls back to a hotspot. If absent, it opens its own AP
  `loop-scope` / `loopscope` at 192.168.4.1.
- `programmer.py`: `uv run programmer.py` flashes the firmware and the gzipped page (as a
  LittleFS image), then opens the serial monitor (115200). Use "upload the page" for
  page-only edits.

## Decisions (measured, don't relitigate without new numbers)

- Classic ESP32 only. One ADC1 pin (GPIO36), **1 MS/s, 8-bit**. 1.5 and 2 MS/s saturate core 1
  and lose samples. Adaptive 10-bit stalled at full load, so it was not shipped.
- Transport: one endless HTTP fetch stream plus `GET /set` for commands. TCP, not UDP (UDP
  lost 8–30%). WebSocket is no faster. Bigger batched sends measured worse.
- Web only: no USB firmware, no Python desktop app, no separate library (all deleted on
  request).
- The board sends raw codes. Calibration (two points, pin V ↔ loop V) happens in the page.
- The header shows link quality % and ADC kS/s. Never show "lost".
- `/info` on the board reports the last reset reason (crash vs power) and the calibration,
  nothing else. `?demo` runs the page on fake data.
- The board is silent on USB serial until it hears `debug` (programmer.py's monitor sends it);
  then where it is and a line a second (heap, RSSI). `quiet` stops it. Don't add always-on logs.
- Core 0 has CPU to spare (≥20 % idle at full load, ~78 % at a normal one); the Wi-Fi air is
  the limit, so moving tasks between cores has nothing to win (a try at moving the stream task
  to core 1 at run time killed the stream).
- On the ESP32, a reset through the EN pin (the USB bridge's RTS) reports as "power on".

## Hardware notes

- Board: ESP32-WROOM-32 DevKit, 38-pin, CP2102 on COM3. COM3 sometimes wedges with
  Windows error 31; retry or replug.
- Front end: LM358 stage into GPIO36. The current stage maps 15 V to 3.3 V. For inputs up
  to 35 V the target gain is ≈ 1/14.3 (0.15–2.45 V out).
- The USB PC is earthed and GND joins it to the signal's minus.

## Working style

- YAGNI/DRY, fewer lines, measured numbers behind every performance claim.
- Test with Python/Node scripts (e.g. a headless Chrome CDP harness) or against the board at
  `loop-scope.local`. Avoid the browser pane: its permission prompts annoy the user.
- The UI is Obsidian-style dark with few boxes. The user likes the Events groups and the
  decoding; protect them.
