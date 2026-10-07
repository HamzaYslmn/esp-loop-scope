// LoopCapture: the board's capture, core.ino's module. Core 1 runs the ADC's DMA at RATE into
// a ring and judges it block by block; a block that swings, or sits off the idle level, opens an
// event, and POST of quiet closes it. Core 0 then sends only events, framed, to the open pages:
// the line's idle noise never leaves the board, so the link idles, nothing is dropped for lack of
// air, and the page sees exactly the communication.
//
// Frame, little endian (core/data/index.html reads it):
//    0  "LS"  magic
//    2  u8    type: 'E' event samples, 'W' scope window, 'L' live envelope, 'S' status (once a second)
//    3  u8    flags, 'E' only: 1 first piece of an event, 2 last piece, 4 samples lost just before
//    4  u32   payload bytes
//    8  u64   sample index since boot of the payload's first sample ('E'), or of now ('S')
//   16  ...   'E': raw samples, one byte each, the ADC's top 8 bits, at most PIECE.
//             'W' (scope, only after "scope ..."): a triggered window, the way a real scope works:
//                  u32 trigger offset in samples from index (~0 = free run), u32 window samples,
//                  then the window: raw samples when flags = 1, else (lo, hi) pairs of flags
//                  samples each (a window longer than PIECE is reduced on the board).
//             'L' (live, while a page is open): (lo, hi) code pairs, each the min and max of
//                  flags = LIVE samples; index is the first sample the first pair covers.
//             'S': u32 rate, u8 lo, u8 hi (codes in the last second), u8 base (idle level),
//                  u8 thr, u32 events since boot, u32 samples lost since boot.
// The page may send lines (GET /set?...): "thr <codes>", "pre <samples>", "post <samples>", "gen <n>",
// "scope <window> <pre> <level> <edge> <mode>": window and pre in samples, level in codes,
// edge 0 none / 1 rising / 2 falling, mode 0 off / 1 auto / 2 normal / 3 single.
//
// Probe: GPIO36 (VP), the LM358 stage's output; GND to the signal's minus. README.md has more.
#pragma once
#include <Arduino.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>

namespace capture {

// MARK: Settings

constexpr int PROBE_PIN = 36;  // change it here and nowhere else
// 0.15..2.45 V at the pin: the LM358 stage must land the 0..35 V input inside it.
constexpr adc_atten_t ATTEN = ADC_ATTEN_DB_12;
// Fixed: 1.5 and 2 MS/s lost samples under load (core 1 saturates), 1 MS/s does not.
constexpr uint32_t RATE = 1000000;
constexpr uint32_t RING = 1 << 16;  // 64 KB, 65 ms: how far the link may lag (no 128 KB block exists in the heap)
constexpr uint32_t LONGEST = RING / 2;  // samples an event may span before it is cut in two
constexpr uint32_t BLOCK = 256;     // samples judged at a time, 0.26 ms
constexpr uint32_t PIECE = 4096;    // samples in a frame at most
constexpr uint32_t FRAME = 2048;    // bytes in a DMA conversion frame, FRAME / 2 samples; a read takes two
constexpr uint32_t HEAD = 16, STATUS = 16, OUT = HEAD + 8 + PIECE;  // OUT: the largest frame
// Live view: a min/max pair per LIVE samples, ~31 KB/s beside the events. Every spike still
// shows; events stay full rate.
constexpr uint32_t LIVE = 64, LIVE_RING = 4096;  // pairs kept: 262 ms
constexpr uint64_t OPEN = ~0ull;    // the end of an event still going on
constexpr int EVENTS = 16;          // events waiting for the link; more are dropped and counted

constexpr int adc1(int gpio) {  // ADC1 channel of a classic ESP32 GPIO, or -1: only ADC1 has DMA
  return gpio == 36 ? 0 : gpio == 37 ? 1 : gpio == 38 ? 2 : gpio == 39 ? 3
       : gpio >= 32 && gpio <= 35 ? gpio - 28 : -1;
}
static_assert(adc1(PROBE_PIN) >= 0, "the probe pin must be ADC1: 32..36 or 39");

// MARK: State, shared by both cores

struct Event { uint64_t start, end; };

inline uint8_t *ring;  // RING bytes; heap, from setup(): static RAM is too small beside Wi-Fi
inline uint8_t *live;  // LIVE_RING pairs, heap too; pair k covers samples k * LIVE onwards
inline portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;  // guards what both cores touch below
struct Locked {  // holds the lock for a scope
  Locked() { taskENTER_CRITICAL(&lock); }
  ~Locked() { taskEXIT_CRITICAL(&lock); }
};
inline uint64_t written;               // samples ever put in the ring, published once a read
inline Event events[EVENTS];
inline uint32_t added, taken;          // events opened by core 1, and finished by core 0
inline volatile uint32_t thr = 8, pre = 2048, post = 4096;  // the page may change them
inline volatile uint32_t detected, lost;
inline volatile uint32_t dropped;  // samples the DMA had no room for (core 1 fell behind): lost too
inline volatile bool liveOn;
// Test signal in place of the probe ("gen <n>"), so every part can be checked on the bench without
// wiring: 0 the probe; 1 a burst of 27 pulses of 200/400 µs every 1.2 s (protocol-like; odd: it ends high, so its last pulse has an edge after it); 2 a 1 kHz
// square, 30 % high; 3 pulses of 100/200 µs without end (the link at full load). ±1 code of noise.
inline volatile uint8_t gen;
// Pin millivolts of each 8-bit code (at its middle), from IDF's line fitting (with its table above
// raw 2880); calSrc: the eFuse it rests on, "two_point", "vref" or "default"; null when none.
inline int16_t calMv[256];
inline const char *calSrc;

inline uint64_t published() { Locked l; return written; }

// MARK: Test signal

inline uint8_t synth() {  // one sample a call; 32-bit counters only (a 64-bit modulo a sample cost ~20 % of core 1)
  static uint32_t x = 1, left, t, ph;
  static uint8_t k = 28;
  static bool high;
  constexpr uint32_t US = RATE / 1000000;   // samples a microsecond
  x ^= x << 13, x ^= x >> 17, x ^= x << 5;  // xorshift: the noise, -1..1
  int noise = int(x & 1) - int(x >> 1 & 1);
  if (gen == 2) return (++ph >= 1000 * US ? ph = 0 : ph) < 300 * US ? 150 + noise : 45 + noise;
  if (gen == 1 && ++t >= 1200000 * US) t = 0, k = 0, left = 0, high = false;  // a burst starts
  if (gen == 1 && k >= 27 && left == 0) return 40 + noise;                    // between bursts
  if (left == 0) {                                                            // the next pulse
    constexpr uint32_t BITS = 0x0b5a3c96;
    bool longer = BITS >> (k % 28) & 1;
    left = (gen == 3 ? (longer ? 200 : 100) : (longer ? 400 : 200)) * US;
    high = !high, k = gen == 3 ? (k + 1) % 28 : k + 1;
  }
  left--;
  return (high ? 130 : 40) + noise;
}

// MARK: Scope and events (core 1)

// Scope: Core 1 watches each sample for the edge while armed and notes where it fired; core 0
// sends the window once it is all written, then arms again. Gapless 1 MS/s does not fit the air
// here (UDP sweep: 8-30% lost at 1-1.5 MS/s), but windows at ~50 a second over TCP do, whole.
constexpr uint32_t WINDOW_MAX = RING / 2, FPS = 50, HYST = 3;  // HYST codes each side of level
inline volatile uint32_t sWindow = 2048, sPre = 409, sMode;
inline volatile uint8_t sLevel = 128, sEdge = 1;
inline volatile bool armed, rearm;  // rearm: core 1 forgets the low side it saw before
inline uint64_t fired = OPEN;  // the trigger's sample index, under lock
inline volatile uint8_t secLo = 255, secHi = 0, base = 0;   // for the status frame
inline adc_continuous_handle_t adc;

// Opens an event at start, or counts it lost when the link is EVENTS behind; false then.
inline bool open(uint64_t start) {
  Locked l;
  bool room = added - taken < EVENTS;
  if (room) events[added % EVENTS] = {start, OPEN}, added++;
  detected++;
  return room;
}

inline void close(uint64_t end) {
  Locked l;
  events[(added - 1) % EVENTS].end = end;
}

// One block's verdict, on core 1. Busy: it swings by more than thr, or sits more than thr off the
// idle level. A flat block makes its own level the idle one, so a step to a new steady level is
// one short event. Learning only from quiet blocks left a stepped line busy for good: one endless
// event at 1 MS/s that no link carries, and 10 M samples lost in a minute (measured).
// An event opens PRE before its first busy block, ends after POST of quiet, and is cut at LONGEST
// and carried on as the next one, so none outruns the ring.
inline void judge(uint64_t end, uint8_t lo, uint8_t hi, uint8_t mean) {
  static int idle = -1;
  static bool active;
  static uint64_t quiet, lastEnd, opened;
  bool flat = uint32_t(hi - lo) <= thr;
  if (idle < 0) idle = mean;
  bool busy = !flat || uint32_t(abs(mean - idle)) > thr;
  if (flat) idle = mean;
  base = idle;
  if (active && end - opened >= LONGEST) {  // cut, and go on seamlessly in a new event
    close(end), lastEnd = end;
    active = open(end), opened = end, quiet = 0;
    return;
  }
  if (!active && busy) {
    uint64_t start = end - BLOCK > pre ? end - BLOCK - pre : 0;
    start = max(start, max(lastEnd, end > RING - PIECE ? end - (RING - PIECE) : 0));
    active = open(start), opened = start, quiet = 0;
  } else if (active) {
    quiet = busy ? 0 : quiet + BLOCK;
    if (quiet >= post) close(end), active = false, lastEnd = end;
  }
}

// MARK: Core 1

// What loop() carries from one DMA read to the next.
inline uint64_t n;                                    // samples made
inline uint8_t lo = 255, hi = 0, llo = 255, lhi = 0;  // this block's and this live pair's extremes
inline uint32_t sum;

// One DMA read's worth, on core 1: sample, keep, judge, watch for the trigger. Never waits on a link.
// O2 for this loop alone: the build's -Os left core 1 at 41 % idle, O2 33 % (measured).
__attribute__((optimize("O2"))) inline void loop() {
  static uint8_t raw[2 * FRAME];
  uint32_t got = 0;
  if (adc_continuous_read(adc, raw, sizeof raw, &got, 100) != ESP_OK) return;
  // Locals for the hot loop: every store into the ring may alias any global, so working on the
  // globals made the compiler reload them each sample (core 1 at 65 % instead of ~44 %, measured).
  const uint64_t n0 = n;
  uint32_t n_ = n0;  // 32 bits here; the index since boot is n0 + uint32_t(n_ - n0)
  uint32_t sum_ = sum;
  uint8_t lo_ = lo, hi_ = hi, llo_ = llo, lhi_ = lhi, *ring_ = ring;
  const uint8_t g = gen, level = sLevel;
  const bool up = sEdge == 1;
  static bool low;   // the trigger saw the low side since it was armed; armed read first: core 0 sets rearm, then armed
  bool arm = armed;  // copies, once a read: read per sample, the trigger alone cost core 1 ~10 % (measured)
  if (rearm) low = false, rearm = false;
  // The classic ESP32's I2S-fed DMA hands each pair of conversions over reversed (measured with
  // a 3-channel test pattern: 0,3,6 arrived 3,0,0,6,6,3): taking every two backwards fixes it.
  for (uint32_t k = 0; k + 3 < got; k += 4) {
    for (uint32_t at : {k + 2, k}) {
      uint8_t v = g ? synth() : ((adc_digi_output_data_t *)&raw[at])->type1.data >> 4;
      ring_[n_ & (RING - 1)] = v;
      lo_ = min(lo_, v), hi_ = max(hi_, v), sum_ += v;
      llo_ = min(llo_, v), lhi_ = max(lhi_, v);
      if (arm) {  // the edge, with hysteresis: low side first, then the high side
        if (up ? v + HYST <= level : v >= level + HYST) low = true;
        else if (low && (up ? v >= level + HYST : v + HYST <= level)) {
          low = false, arm = armed = false;
          Locked l;
          fired = n0 + uint32_t(n_ - n0);
        }
      }
      if ((++n_ & (LIVE - 1)) == 0) {
        uint32_t at = (n_ / LIVE - 1) % LIVE_RING * 2;
        live[at] = llo_, live[at + 1] = lhi_, llo_ = 255, lhi_ = 0;
        if ((n_ & (BLOCK - 1)) == 0) {
          judge(n0 + uint32_t(n_ - n0), lo_, hi_, sum_ / BLOCK);
          if (lo_ < secLo) secLo = lo_;
          if (hi_ > secHi) secHi = hi_;
          lo_ = 255, hi_ = 0, sum_ = 0;
        }
      }
    }
  }
  n = n0 + uint32_t(n_ - n0), sum = sum_, lo = lo_, hi = hi_, llo = llo_, lhi = lhi_;
  { Locked l; written = n; }
}

// Core 1 does nothing else: the ADC started here (start takes a lock this task must own), then loop().
inline void Core1Task(void *) {
  ESP_ERROR_CHECK(adc_continuous_start(adc));
  for (;;) loop();
}

// The DMA's pool was full: a frame's samples dropped (they never reach the ring or its count).
inline bool overflow(adc_continuous_handle_t, const adc_continuous_evt_data_t *, void *) {
  dropped += FRAME / 2;
  return false;
}

// The pin's millivolts for each code, from the chip's eFuse (calMv, calSrc above).
inline void calibrate() {
  adc_cali_line_fitting_efuse_val_t src;
  adc_cali_line_fitting_config_t c = {ADC_UNIT_1, ATTEN, ADC_BITWIDTH_12, 1100};
  adc_cali_handle_t h;
  if (adc_cali_scheme_line_fitting_check_efuse(&src) != ESP_OK || adc_cali_create_scheme_line_fitting(&c, &h) != ESP_OK) return;
  for (int k = 0, mv = 0; k < 256; k++) adc_cali_raw_to_voltage(h, k * 16 + 8, &mv), calMv[k] = mv;
  adc_cali_delete_scheme_line_fitting(h);
  calSrc = src == ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP ? "two_point" : src == ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF ? "vref" : "default";
}

// The buffers and the ADC first (they want internal RAM before Wi-Fi takes it), then core 1.
inline void setup() {
  ring = (uint8_t *)heap_caps_malloc(RING, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  live = (uint8_t *)heap_caps_malloc(LIVE_RING * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  while (!ring || !live) Serial.println("LoopCapture: no RAM for the ring"), delay(1000);
  calibrate();
  adc_continuous_handle_cfg_t handle = {};
  handle.max_store_buf_size = 8 * FRAME;  // 8 ms at 1 MS/s while core 1 judges
  handle.conv_frame_size = FRAME;
  ESP_ERROR_CHECK(adc_continuous_new_handle(&handle, &adc));
  adc_digi_pattern_config_t pattern = {ATTEN, (uint8_t)adc1(PROBE_PIN), ADC_UNIT_1, ADC_BITWIDTH_12};
  adc_continuous_config_t cfg = {};
  cfg.pattern_num = 1, cfg.adc_pattern = &pattern, cfg.sample_freq_hz = RATE;
  cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1, cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
  ESP_ERROR_CHECK(adc_continuous_config(adc, &cfg));
  adc_continuous_evt_cbs_t cbs = {};
  cbs.on_pool_ovf = overflow;
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc, &cbs, nullptr));
  xTaskCreatePinnedToCore(Core1Task, "Core1Task", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
}

// MARK: Frames (core 0)

inline void header(uint8_t *out, char type, uint8_t flags, uint32_t n, uint64_t index) {
  out[0] = 'L', out[1] = 'S', out[2] = type, out[3] = flags;
  memcpy(out + 4, &n, 4), memcpy(out + 8, &index, 8);
}

// Whether core 1 may have overwritten sample start by now: its ring runs up to a read ahead of
// written. Asked after a copy too: core 0 may be held up mid-copy (a request, Wi-Fi).
inline bool lapped(uint64_t start) { return published() + FRAME > start + RING; }

// n ring samples from start into out; false if core 1 overwrote some meanwhile.
inline bool copyRing(uint8_t *out, uint64_t start, uint32_t n) {
  uint32_t at = start & (RING - 1), first = min(n, RING - at);
  memcpy(out, ring + at, first), memcpy(out + first, ring, n - first);
  return !lapped(start);
}

// The next event piece into out (HEAD + PIECE bytes): its length, or 0 when nothing waits.
// Core 0 only. A piece the ring has already overwritten is skipped and flagged as lost.
inline size_t next(uint8_t *out) {
  static uint64_t pos, current = OPEN;
  bool any;
  Event e;
  uint64_t w;
  {
    Locked l;
    any = taken != added, e = events[taken % EVENTS], w = written;
  }
  // Core 1 opens an event before it publishes the samples it judged: with a short pre the start
  // lies past w, and w - pos below wrapped (garbage lost, the piece rewound into the last event).
  if (!any || e.start > w) return 0;
  uint8_t flags = 0;
  if (e.start != current) current = e.start, pos = max(pos, e.start), flags |= 1;
  if (w - pos > RING - PIECE) {  // the ring lapped what was not sent: on from what is still there
    uint64_t to = w - (RING - PIECE), gone = min(to, e.end);
    if (gone > pos) lost += gone - pos;  // this event's samples only: the quiet after it was never to go
    pos = to, flags |= 4;
  }
  uint64_t stop = min(e.end, w);
  if (pos >= stop) {
    if (e.end != OPEN) {  // all of it went, or the ring lapped past its end: done
      Locked l;
      taken++;
      if (flags & 1) current = OPEN;  // never began: let a later look start it afresh
    }
    return 0;
  }
  uint32_t n = min<uint64_t>(stop - pos, PIECE);
  if (pos + n == e.end) flags |= 2;
  header(out, 'E', flags, n, pos);
  if (!copyRing(out + HEAD, pos, n)) {  // overwritten while copying: the next look skips it as lost
    if (flags & 1) current = OPEN;
    return 0;
  }
  pos += n;
  if (flags & 2) {
    Locked l;
    taken++;
  }
  return HEAD + n;
}

// The next scope window into out (OUT bytes), or 0. Core 0, often: it rate-limits
// itself to FPS. Auto sends a free-running window when no edge came for 2 windows or 50 ms;
// normal waits; single sends one and stops.
inline size_t scopeNext(uint8_t *out) {
  static TickType_t last;
  static bool wasOn;
  uint32_t mode = sMode, win = sWindow, pre = sPre;
  if (!mode) return armed = false, wasOn = false, 0;
  if (!wasOn) wasOn = true, rearm = true, armed = sEdge != 0, fired = OPEN;  // just switched on: arm
  TickType_t now = xTaskGetTickCount();
  if (now - last < pdMS_TO_TICKS(1000 / FPS)) return 0;
  uint64_t w, f;
  {
    Locked l;
    w = written, f = fired;
  }
  uint64_t start;
  uint32_t trig;
  if (f != OPEN) {
    if (w < f + (win - pre)) return 0;  // the window's tail is not written yet
    start = f > pre ? f - pre : 0, trig = f - start;
  } else if (mode == 1 && (sEdge == 0 || now - last > pdMS_TO_TICKS(max<uint32_t>(50, 2000 * win / RATE)))) {
    if (w < win) return 0;
    start = w - win, trig = ~0u;  // auto: no edge in time, run free
  } else {
    return 0;
  }
  last = now;
  {
    Locked l;
    fired = OPEN;
  }
  if (mode == 3 && trig != ~0u) sMode = 0;  // single: this one, then stop
  else rearm = true, armed = sEdge != 0;
  if (lapped(start)) return 0;  // a window far too long for the link: skip it
  uint32_t group = win <= PIECE ? 1 : (win + PIECE / 2 - 1) / (PIECE / 2), k = win / group;  // raw when it fits, else pairs
  uint8_t *p = out + HEAD + 8;
  if (group == 1 && !copyRing(p, start, k)) return 0;
  for (uint32_t i = 0; group > 1 && i < k; i++) {
    uint8_t lo = 255, hi = 0;
    for (uint32_t j = 0; j < group; j++) {
      uint8_t v = ring[(start + i * group + j) & (RING - 1)];
      lo = min(lo, v), hi = max(hi, v);
    }
    p[2 * i] = lo, p[2 * i + 1] = hi;
  }
  if (lapped(start)) return 0;
  uint32_t n = 8 + (group == 1 ? k : 2 * k);
  header(out, 'W', group, n, start);
  memcpy(out + HEAD, &trig, 4), memcpy(out + HEAD + 4, &win, 4);
  return HEAD + n;
}

// The live pairs made since the last call into out (HEAD + PIECE bytes), or 0: off, or none
// new. Core 0, every ~20 ms. Pairs the ring already overwrote are skipped; the index shows it.
inline size_t liveNext(uint8_t *out) {
  static uint64_t pos;
  static bool was;
  if (!liveOn) return was = false, 0;
  uint64_t p = published() / LIVE;
  if (!was) was = true, pos = p;  // live starts now, not with what piled up while it was off
  if (p - pos > LIVE_RING) pos = p - LIVE_RING;
  uint32_t k = min<uint64_t>(p - pos, PIECE / 2);
  if (!k) return 0;
  header(out, 'L', LIVE, 2 * k, pos * LIVE);
  uint32_t at = (pos & (LIVE_RING - 1)) * 2, first = min(2 * k, LIVE_RING * 2 - at);
  memcpy(out + HEAD, live + at, first), memcpy(out + HEAD + first, live, 2 * k - first);
  pos += k;
  return HEAD + 2 * k;
}

// The status frame into out (HEAD + STATUS bytes); starts the next second's lo..hi.
inline size_t status(uint8_t *out) {
  header(out, 'S', 0, STATUS, published());
  uint32_t r = RATE, e = detected, l = lost + dropped;
  memcpy(out + HEAD, &r, 4);
  out[HEAD + 4] = secLo, out[HEAD + 5] = secHi, out[HEAD + 6] = base, out[HEAD + 7] = thr;
  memcpy(out + HEAD + 8, &e, 4), memcpy(out + HEAD + 12, &l, 4);
  secLo = 255, secHi = 0;
  return HEAD + STATUS;
}

// MARK: Control (core 0)

// Nobody is reading: what waits would only be stale when someone comes.
inline void drop() {
  Locked l;
  bool closing = taken != added && events[(added - 1) % EVENTS].end == OPEN;
  taken = closing ? added - 1 : added;  // an open event stays, it may still be read whole
}

// A line from the page (the commands at the top); anything else is ignored.
inline void command(const char *line) {
  unsigned v;
  if (sscanf(line, "thr %u", &v) == 1) thr = v < 1 ? 1 : v > 255 ? 255 : v;
  else if (sscanf(line, "pre %u", &v) == 1) pre = min<unsigned>(v, RING / 2);
  else if (unsigned w, p, l, e, m; sscanf(line, "scope %u %u %u %u %u", &w, &p, &l, &e, &m) == 5) {
    w = w < 16 ? 16 : w > WINDOW_MAX ? WINDOW_MAX : w;
    sWindow = w, sPre = p > w ? w : p, sLevel = l > 255 ? 255 : l, sEdge = e > 2 ? 0 : e, sMode = m > 3 ? 0 : m;
  }
  else if (sscanf(line, "post %u", &v) == 1) post = v < BLOCK ? BLOCK : v > RATE ? RATE : v;
  else if (sscanf(line, "gen %u", &v) == 1) gen = v > 3 ? 0 : v;
}

}  // namespace capture
