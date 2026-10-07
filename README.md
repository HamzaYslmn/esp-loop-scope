# esp-loop-scope

A Wi-Fi oscilloscope and event listener built on a classic ESP32. It samples one signal at
1 MS/s (8-bit) through an ADC pin and streams it to a web page served by the board itself: no app
to install, no PC software, just a browser. Made for reading slow serial and pulse-coded lines,
such as two-wire bus or loop protocols, where you want both a live trace and a log of every burst.

**Open it:** https://hamzayslmn.github.io/esp-loop-scope/ ·
[try the demo](https://hamzayslmn.github.io/esp-loop-scope/?demo) (no board needed)

**What you get:**

- **Live scope**: triggered 1 MS/s windows up to 2 ms/div, rolling view beyond, with the usual
  bench controls (RUN/STOP, Single, Autoset, cursors, CSV/PNG export).
- **Event capture**: the board detects bursts of activity on its own; the page groups identical
  messages and decodes pulse widths into bits and bytes, with compare and histogram views.
- **Two-point calibration** in the page, so any front end (gain, offset, inversion) reads true
  input volts.

## Quick start

1. Wire it (below).
2. Wi-Fi: copy `core/secrets.h.example` to `core/secrets.h`, or skip it and join the board's own
   network `loop-scope` (key `loopscope`).
3. Flash: `uv run programmer.py` → `core` (firmware + page). After a page-only edit, "upload the
   page" is enough.
4. Open http://loop-scope.local/ (on the board's own network: http://192.168.4.1/).
5. **⚖ Calibrate** once: two known points, pin volts ↔ input volts.

**From GitHub Pages:** the same page is at https://hamzayslmn.github.io/esp-loop-scope/.
Type the board's address (`loop-scope.local` or its IP) in the header box and **Connect**; it is
remembered. Chrome and Edge ask once to allow local network access. Firefox, Safari and every
iOS browser block an http device from an https page: use **↗** there, which opens the board's own
page. `?demo` runs without a board.

## Wiring

```
signal + ── LM358 stage ── GPIO36 (VP)
signal - ── GND (stage and ESP32)
```

ESP32-WROOM-32 DevKit, 38-pin, CP2102. Any ADC1 pin works (32–36, 39; `PROBE_PIN` in
`core/src/LoopCapture.h`); 36 is the cleanest: input only, no pull-up, no boot role. ADC2 is
unusable (the radio takes it).

```
                             ┌──┐
                     3V3    ─┤  ├─     G         probe GND
                      EN    ─┤  ├─ 23  free
ADC1_0    ^        PROBE 36 ─┤  ├─ 22  free
ADC1_3    ^         free 39 ─┤  ├─ 1   UART0 TX   USB
ADC1_6    ^         free 34 ─┤  ├─ 3   UART0 RX   USB
ADC1_7    ^         free 35 ─┤  ├─ 21  free
ADC1_4              free 32 ─┤  ├─     G
ADC1_5              free 33 ─┤  ├─ 19  free
ADC2_8              free 25 ─┤  ├─ 18  free
ADC2_9              free 26 ─┤  ├─ 5   free *
ADC2_7              free 27 ─┤  ├─ 17  free
ADC2_6              free 14 ─┤  ├─ 16  free
ADC2_5            free * 12 ─┤  ├─ 4   free         ADC2_0
                       G    ─┤  ├─ 0   BOOT *       ADC2_1
ADC2_4              free 13 ─┤  ├─ 2   LED *        ADC2_2
                   FLASH  9 ─┤  ├─ 15  free *       ADC2_3
                   FLASH 10 ─┤  ├─ 8   FLASH
                   FLASH 11 ─┤  ├─ 7   FLASH
                      5V    ─┤  ├─ 6   FLASH
                             └──┘
```

### Front end (LM358)

The page converts raw codes to input volts with the two calibration points, so the stage's gain,
offset or inversion doesn't matter. Its output must stay in range:

| Limit | Why |
|---|---|
| 0.15–2.45 V out for 0–35 V in (gain ≈ 1/14.3) | the ADC's 12 dB range clips outside it |
| Never above 3.3 V | the pin's limit; the LM358 isn't rail-to-rail, so power it from 3.3 V or add 1k + clamp |
| ~1k + 3.3 nF RC on the output | filters noise and gives the ADC a low source impedance |

> **Match the gain to your signal:** the current stage maps 15 V → 3.3 V, so anything above
> ~14 V reads flat and overdrives the pin. For signals up to 35 V cut the gain to ~1/14.3 and
> recalibrate.

LM358 edges are slow (~0.3 V/µs, ~7 µs per full-scale edge): fine for slow serial lines, but
edges look sloped.

A USB-connected PC is earthed, and GND ties it to the signal's minus. Probe battery-powered or
floating circuits, or isolate the board, when the system under test is earth-fault monitored.

## Using it

- **Live**: a bench scope. RUN/STOP, Single, Autoset, Auto-range, time/div, V/div, trigger;
  the rest is under **⋯ More**. Up to 2 ms/div it shows triggered 1 MS/s windows; from 5 ms/div
  it rolls.
- **Events**: what the board detected, zoomed to the activity: levels, pulse widths, groups of
  identical messages, bits/bytes, compare, histogram. **Detect ≥ Auto** sets the threshold
  above the noise. Nothing is saved automatically: **Download** writes `.jsonl`, **Import** reads
  it back.
- Open `#live` and `#events` in two tabs (up to 3 viewers). `?demo` runs on fake data without the
  board. `/info` shows why the board last restarted.
