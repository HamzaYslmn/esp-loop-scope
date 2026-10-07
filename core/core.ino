// esp-loop-scope, Wi-Fi. LoopCapture samples the loop at 1 MS/s on core 1 and finds
// the communication; Core0Task serves the board's own page (data/: index.html with live and
// event views, kept in the LittleFS partition: `uv run programmer.py` writes it) at http://loop-scope.local/
// and streams to each open page, over TCP, the events, a status a second, the live envelope and,
// for its live tab, the scope's triggered windows. TCP: lossless, and a lost piece would break
// exactly what we are trying to decode. UDP looked faster only because it never slows down: on
// this air it lost 8-30% at 1-1.5 MS/s. Frames and commands: src/LoopCapture.h.
// USB serial, 115200: silent until it hears "debug" (programmer.py's monitor sends it), then the
// address and a line per second; "quiet" stops it.

#include <ESPmDNS.h>
#include <LittleFS.h>
#include "src/LoopCapture.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <lwip/api.h>
extern "C" {
#include <lwip/priv/sockets_priv.h>  // lwip_socket_dbg_get_socket: the header lacks extern "C"
}
#include <lwip/sockets.h>
#include <lwip/tcp.h>
#include <lwip/tcpip.h>

// MARK: Wi-Fi

#if __has_include("secrets.h")
#include "secrets.h"  // WIFI_SSID / WIFI_PASS of the router's 2.4 GHz band; gitignored
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""  // none: the board opens its own network, AP_SSID, at 192.168.4.1
#define WIFI_PASS ""
#endif
constexpr char AP_SSID[] = "loop-scope", AP_PASS[] = "loopscope";  // a WPA2 key needs 8+
constexpr char HOST[] = "loop-scope";  // loop-scope.local
constexpr uint16_t WEB = 80;

// A network in secrets.h is the one: join it and keep trying, never open a hotspot instead (a
// typo in the key should not leave a second network on the bench). No secrets.h: the hotspot.
static void startWifi() {
  if (*WIFI_SSID) {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);  // automatic: 40 MHz when the router offers it, else 20. Set every boot: the driver keeps the last value in flash
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);  // the strongest AP of that name (a router and its repeaters), on every (re)join; the default took the first heard
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int k = 1; WiFi.status() != WL_CONNECTED; k++)
      if (delay(1000); k % 10 == 0) WiFi.begin(WIFI_SSID, WIFI_PASS);
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
  }
  esp_wifi_set_ps(WIFI_PS_NONE);  // modem sleep would gate every packet to the AP's beacons
  MDNS.begin(HOST);
  MDNS.addService("http", "tcp", WEB);
}

// MARK: TCP

// A connection that streams: no Nagle delay, a reader gone silent frees the board in 3 s, and the
// send buffer lifted. The Arduino build fixes TCP's send buffer at 5,744 B: over an 8 ms round trip
// that caps it near 0.5 MB/s. lwIP sets it per connection only at birth, so lift this one 4x, the
// most its 16-segment queue holds. Measured, raw 1 MS/s for 8 s: stock 532 kS/s with 46% skipped,
// 2x 949k (4.3%), 4x 1,000,026 S/s with 0% skipped.
// Each frame goes in its own send: gathering them into 16 KB sends measured ~700 KB/s against
// ~1,040 (lwIP takes more only once half its queue has drained, so the pipe ran half empty).
static void tune(int fd) {
  int on = 1;
  timeval wait = {3, 0};
  lwip_setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof on);
  lwip_setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &wait, sizeof wait);
  LOCK_TCPIP_CORE();
  if (lwip_sock *sk = lwip_socket_dbg_get_socket(fd)) sk->conn->pcb.tcp->snd_buf = 4 * TCP_SND_BUF;
  UNLOCK_TCPIP_CORE();
}

static bool sendAll(int fd, const void *data, size_t n) {
  for (auto p = (const uint8_t *)data; n;) {
    int k = lwip_send(fd, p, n, 0);
    if (k <= 0) return false;
    p += k, n -= k;
  }
  return true;
}

// 0: the other side closed. Noticing only when a send timed out kept a gone reader 3 s.
static bool closed(int fd) {
  char c;
  return lwip_recv(fd, &c, 1, MSG_DONTWAIT) == 0;
}

// MARK: HTTP

// A response's head; len < 0 for the stream, which has no end. extra: more header lines.
// Any origin may read the answers, so the page also works from GitHub Pages (or a file).
static bool head(int fd, const char *status, const char *type, int len, const char *extra = "") {
  char h[256], size[32] = "";
  if (len >= 0) snprintf(size, sizeof size, "Content-Length: %d\r\n", len);
  int k = snprintf(h, sizeof h, "HTTP/1.1 %s\r\nAccess-Control-Allow-Origin: *\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Type: %s\r\n%s%s\r\n", status, type, extra, size);
  return sendAll(fd, h, min<int>(k, sizeof h - 1));
}

// The page, kept gzipped in LittleFS by programmer.py: it goes as it is (the browser unpacks it),
// a TCP segment's worth at a time, no RAM copy. Not there (the partition never written): how to.
static void sendPage(int fd) {
  constexpr char PAGE[] = "/index.html.gz";
  File f = LittleFS.exists(PAGE) ? LittleFS.open(PAGE, "r") : File();
  if (!f) {
    static const char NO[] = "The page is not on the board yet: run `uv run programmer.py` and pick \"upload the page\".\n";
    head(fd, "503 Service Unavailable", "text/plain", sizeof NO - 1) && sendAll(fd, NO, sizeof NO - 1);
    return;
  }
  static uint8_t buf[1460];
  if (head(fd, "200 OK", "text/html; charset=utf-8", f.size(), "Content-Encoding: gzip\r\n"))
    for (size_t n; (n = f.read(buf, sizeof buf)) > 0 && sendAll(fd, buf, n);) {}
  f.close();
}

// Why the board last started, so a crash on the bench can be read afterwards instead of guessed.
static const char *resetReason() {
  static const char *const WHY[] = {"other", "power on", "reset pin", "restart", "crash (panic)", "crash (interrupt watchdog)",
                                    "crash (task watchdog)", "crash (watchdog)", "deep sleep", "brownout (supply dipped)"};
  unsigned r = esp_reset_reason();
  return r < sizeof WHY / sizeof *WHY ? WHY[r] : "other";
}

// GET /info: the board's health, and the ADC's calibration (cal: pin mV of each code, see
// capture::calMv). Only what the page reads: the rest of the board's health is the serial log's.
static void sendInfo(int fd) {
  static char body[2048];  // the calibration's 256 numbers are most of it (~1.5 KB)
  int n = snprintf(body, sizeof body, "{\"reset\":\"%s\",\"uptime_s\":%lu", resetReason(), millis() / 1000);
  if (capture::calSrc) {
    n += snprintf(body + n, sizeof body - n, ",\"cal_src\":\"%s\",\"cal\":[%d", capture::calSrc, capture::calMv[0]);
    for (int k = 1; k < 256; k++) n += snprintf(body + n, sizeof body - n, ",%d", capture::calMv[k]);
    n += snprintf(body + n, sizeof body - n, "]");
  }
  n = min<int>(n + snprintf(body + n, sizeof body - n, "}"), sizeof body - 1);  // fits by ~500 B: the min only guards
  head(fd, "200 OK", "application/json", n) && sendAll(fd, body, n);
}

// One browser request: the page ("/" or "/index.html"); "/info" (above); "/stream?scope" the
// frames for as long as it reads (returns that fd, the new viewer); "/set?thr=8" a setting
// (capture::command). Reads the whole header: lwIP answers a close with unread bytes by a reset,
// which can cut the page short.
static int serve(int fd) {
  tune(fd);
  timeval wait = {1, 0};
  lwip_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &wait, sizeof wait);
  char req[96];
  int have = 0;
  uint32_t tail = 0;
  for (char c; tail != 0x0d0a0d0a && lwip_recv(fd, &c, 1, 0) == 1;) {  // ponytail: byte by byte, ~500 B a request
    if (have < (int)sizeof req - 1) req[have++] = c;
    tail = tail << 8 | (uint8_t)c;
  }
  req[have] = 0;
  if (!have) {  // a browser's spare connection that asked nothing (yet): close it quietly, the browser
    lwip_close(fd);  // retries on a fresh one. Answering 404 here was read as the reply to its next /set.
    return -1;
  }
  if (!strncmp(req, "GET /stream", 11)) {
    if (head(fd, "200 OK", "application/octet-stream", -1)) return fd;
  } else if (!strncmp(req, "GET /info", 9)) {
    sendInfo(fd);
  } else if (!strncmp(req, "GET /set?", 9)) {
    char *line = req + 9;  // "scope=2000+400+110+1+1 HTTP/1.1" -> "scope 2000 400 110 1 1"
    line[strcspn(line, " ")] = 0;
    for (char *e = line; *e; e++)
      if (*e == '=' || *e == '+') *e = ' ';
    capture::command(line);
    head(fd, "204 No Content", "text/plain", 0);
  } else if (!strncmp(req, "OPTIONS ", 8)) {  // Chrome's preflight before an https page may reach a local host
    head(fd, "204 No Content", "text/plain", 0, "Access-Control-Allow-Private-Network: true\r\nAccess-Control-Allow-Methods: GET\r\nAccess-Control-Allow-Headers: *\r\n");
  } else if (!strncmp(req, "GET / ", 6) || !strncmp(req, "GET /?", 6) || !strncmp(req, "GET /index.html", 15)) {
    sendPage(fd);
  } else {
    head(fd, "404 Not Found", "text/plain", 0);
  }
  lwip_close(fd);
  return -1;
}

// MARK: Tasks

// Browsers watching at once: a phone and a PC, the live tab and the events tab. Each costs one
// more send of every frame: ~31 KB/s of live, up to ~200 KB/s more of scope windows, and a slow
// one slows them all.
constexpr int VIEWERS = 3;
static int views[VIEWERS] = {-1, -1, -1};

static bool debug;  // the serial log: off until asked for, so a board nobody watches spends nothing on it

static void leave(int &v) {
  lwip_close(v), v = -1;
  if (debug) Serial.println("browser out");
}

static void toViews(const void *p, size_t n) {
  for (int &v : views)
    if (v >= 0 && !sendAll(v, p, n)) leave(v);
}

// Once a second: "debug" or "quiet" from the USB serial. On: where the board is, then a line a second.
static void listen() {
  static char line[8];
  static uint8_t n;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (n < sizeof line - 1) line[n++] = c; continue; }
    line[n] = 0, n = 0;
    if (!strcmp(line, "quiet")) debug = false;
    if (strcmp(line, "debug") || debug) continue;
    debug = true;
    Serial.printf("loop-scope: started after %s, up %lu s; ", resetReason(), millis() / 1000);
    if (*WIFI_SSID) Serial.printf("on %s via %s (%d dBm): %s, %s.local\n", WIFI_SSID, WiFi.BSSIDstr().c_str(), WiFi.RSSI(), WiFi.localIP().toString().c_str(), HOST);
    else Serial.printf("own network %s, key %s, board at 192.168.4.1\n", AP_SSID, AP_PASS);
  }
}

// Requests, on core 0 beside the stream: files, /info, /set, and new streams handed to Core0Task
// through joins. Apart because a request may take seconds (a page to a slow client: measured up
// to 5.8 s), and in the streaming loop that froze every open page.
static QueueHandle_t joins;  // new viewers' fds

static void WebTask(void *) {
  int server = lwip_socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in me = {};
  me.sin_family = AF_INET, me.sin_port = htons(WEB), me.sin_addr.s_addr = htonl(INADDR_ANY);
  lwip_bind(server, (sockaddr *)&me, sizeof me);
  lwip_listen(server, 4);
  for (;;) {
    int fd = lwip_accept(server, nullptr, nullptr);
    if (fd < 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
    fd = serve(fd);
    if (fd >= 0 && xQueueSend(joins, &fd, 0) != pdTRUE) lwip_close(fd);
  }
}

// Core 0, beside Wi-Fi and lwIP: send events as they come, a status each second, live and scope
// windows to every open page. Blocking sends only ever slow this task.
// The scope's trigger is one for all: pages share it, the last one set wins.
static void Core0Task(void *) {
  static uint8_t out[capture::OUT];
  for (TickType_t due = 0, liveDue = 0;;) {
    bool anyView = false;
    for (int fd; xQueueReceive(joins, &fd, 0) == pdTRUE;) {
      int *v = std::find(views, views + VIEWERS, -1);
      if (v == views + VIEWERS) lwip_close(fd);  // full: that page retries each second
      else if (*v = fd; debug) Serial.println("browser in");
    }
    for (int &v : views) {
      if (v >= 0 && closed(v)) leave(v);  // a closed tab frees its place at once
      anyView |= v >= 0;
    }
    if ((int32_t)(xTaskGetTickCount() - due) >= 0) {
      due = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
      toViews(out, capture::status(out));
      listen();
      if (debug) Serial.printf("events %lu  lost %lu  base %u  heap %lu  rssi %d\n", capture::detected, capture::lost, capture::base, ESP.getFreeHeap(), WiFi.RSSI());
    }
    capture::liveOn = anyView;
    if (!anyView) {  // nobody to tell: what waits would only be stale later; no trigger either
      capture::drop(), capture::sMode = 0;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if ((int32_t)(xTaskGetTickCount() - liveDue) >= 0) {  // live: a frame each 20 ms
      liveDue = xTaskGetTickCount() + pdMS_TO_TICKS(20);
      if (size_t k = capture::liveNext(out)) toViews(out, k);
    }
    // Events last: putting a waiting event piece before the scope's window measured worse (571 vs
    // 648 KB/s of events at full load, 4 runs each).
    if (size_t k = capture::scopeNext(out)) toViews(out, k);  // limits itself to FPS
    if (size_t k = capture::next(out)) toViews(out, k);  // an event piece: to all, so each sees every event whole
    else vTaskDelay(1);
  }
}

// MARK: Setup

void setup() {
  Serial.begin(115200);  // silent: it only listens for "debug"
  capture::setup();
  LittleFS.begin(false);  // nothing there yet: "/" answers how to upload the page
  startWifi();
  joins = xQueueCreate(VIEWERS, sizeof(int));
  xTaskCreatePinnedToCore(Core0Task, "Core0Task", 4096, nullptr, 5, nullptr, 0);
  // Above the stream: it sleeps in accept() and takes the core only for a request; below it, a
  // stream with events always waiting starved it (/set timed out under full load, measured).
  xTaskCreatePinnedToCore(WebTask, "WebTask", 4096, nullptr, 6, nullptr, 0);
}

// Everything runs in Core0Task, WebTask and capture's Core1Task (capture::loop()); Arduino still
// links a loop(), so this one removes Arduino's own loop task the first time it runs.
void loop() { vTaskDelete(nullptr); }
