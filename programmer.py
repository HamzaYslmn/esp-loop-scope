# /// script
# dependencies = ["pyserial"]
# ///
"""Flash over the cable, then listen to it: `uv run programmer.py`.

Flashes core/ and opens a serial monitor, or just the monitor. Board and options are
`sketch.yaml`'s, so this passes no flags and knows no boards. core/data/ (the web page) goes
too, gzipped, as a LittleFS image in the partition table's spiffs partition; "upload the page"
does only that, so a page edit needs no firmware build.
"""
import gzip
import subprocess
import tempfile
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

SKETCH = Path(__file__).parent / "core"  # this file sits at the repo's top
BAUD = 115200
BOLD, DIM, END = "\033[1m", "\033[2m", "\033[0m"


def alias(text):
    """COM3, com3 and 3 are the same port to anyone typing it."""
    return text.strip().lower().removeprefix("com")


def pick(kind, items, label, name=None):
    """One step of the menu. name(item) numbers the list itself, COM3 as 3, so there is no
    second number to translate."""
    print(f"\n{BOLD}{kind}{END}")
    if not items:
        sys.exit(f"  no {kind} found")
    tags = [alias(name(item)) if name else str(i) for i, item in enumerate(items, 1)]
    for tag, item in zip(tags, items):
        print(f"  {tag:>3}  {label(item)}")
    if len(items) == 1:
        return items[0]  # nothing to choose
    while True:
        try:
            answer = input(f"{DIM}>{END} ")
        except (EOFError, KeyboardInterrupt):
            sys.exit("\ncancelled")
        if alias(answer) in tags:
            return items[tags.index(alias(answer))]


def run(cmd):
    if subprocess.run(cmd).returncode:
        sys.exit(1)  # the tool said why


def tool(name):
    """The newest copy of an ESP32 core tool (mklittlefs, esptool) where arduino-cli keeps them."""
    data = subprocess.run(["arduino-cli", "config", "get", "directories.data"], capture_output=True, text=True).stdout.strip()
    found = sorted((Path(data) / "packages" / "esp32" / "tools").glob(f"{name}*/*/{name.removesuffix('_py')}*"))
    found = [f for f in found if f.suffix in ("", ".exe")]
    if not found:
        sys.exit(f"no {name} in {data}: install the esp32 core with arduino-cli")
    return str(found[-1])


def upload_page(sketch, port):
    """data/ as a LittleFS image, written where the build's partition table puts spiffs. Each
    file goes gzipped (name.gz); the board sends it as it is and the browser unpacks it, so a page
    crosses the air at a fraction of its size."""
    build = sketch / "build"
    table = build / "partitions.csv"
    if not table.exists():  # never built here: the table comes from a build
        run(["arduino-cli", "compile", "--build-path", str(build), str(sketch)])
    row = next(r.split(",") for r in table.read_text().splitlines() if r.startswith("spiffs"))
    offset, size = row[3].strip(), int(row[4].strip(), 16)
    image = build / "littlefs.bin"
    with tempfile.TemporaryDirectory() as packed:
        for f in (sketch / "data").rglob("*"):
            if f.is_file():
                out = Path(packed) / f"{f.relative_to(sketch / 'data')}.gz"
                out.parent.mkdir(parents=True, exist_ok=True)
                out.write_bytes(gzip.compress(f.read_bytes(), 9, mtime=0))  # mtime 0: the same page, the same image
                print(f"  {f.name}: {f.stat().st_size:,} B -> {out.stat().st_size:,} B gzipped")
        print(f"\n{sketch.name}/data -> LittleFS at {offset} ({size // 1024} KB)\n")
        run([tool("mklittlefs"), "-c", packed, "-s", str(size), "-p", "256", "-b", "4096", str(image)])
    run([tool("esptool_py"), "--chip", "esp32", "--port", port, "--baud", "921600", "write-flash", offset, str(image)])


def port_pick():
    """The USB serial ports (Bluetooth COMs have no vid), by number, so a list number means the
    same twice running."""
    found = sorted((p for p in list_ports.comports() if p.vid), key=lambda p: alias(p.device).rjust(4, "0"))
    return pick("port", found, lambda p: f"{p.device}  {p.description}", lambda p: p.device).device


def link(port, seconds=5):
    """DTR and RTS low before opening: on a devkit's auto-reset pair, a toggle on open would
    reset the board or hold it in the bootloader. Windows also keeps a port claimed for a
    moment after the flasher lets go, so wait out 'busy'."""
    line = serial.Serial(baudrate=BAUD, timeout=0.1)
    line.port, line.dtr, line.rts = port, False, False
    deadline = time.time() + seconds
    while True:
        try:
            line.open()
            return line
        except serial.SerialException:
            if time.time() > deadline:
                raise
            time.sleep(0.2)


def monitor(port):
    """The board stays silent until asked: "debug" turns its log on (a line a second), until ctrl+c."""
    line = link(port)
    print(f"\n{port} at {BAUD}, ctrl+c to leave\n")
    heard, asked = False, 0.0
    try:
        while True:
            if not heard and time.time() - asked > 2:  # a board still booting misses it: ask again till it talks
                line.write(b"debug\n")
                asked = time.time()
            got = line.read(line.in_waiting or 1)
            heard |= b"loop-scope:" in got
            sys.stdout.write(got.decode(errors="replace"))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        line.close()


def main():
    subprocess.run("", shell=True)  # Windows: any shell call turns the console's escape codes on
    print(f"{BOLD}ESP32 programmer{END}")
    todo = pick("do", ["flash the firmware and the page, then the monitor", "upload the page", "serial monitor"], str)
    port = port_pick()
    if todo.startswith("flash"):
        print(f"\n{SKETCH.name} -> {port}\n")
        run(["arduino-cli", "compile", "-u", "-p", port, "--build-path", str(SKETCH / "build"), str(SKETCH)])
    if todo != "serial monitor":
        upload_page(SKETCH, port)
    monitor(port)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass  # ctrl+c is how you leave a terminal, not a crash
