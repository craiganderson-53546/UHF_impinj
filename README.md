# RRU7182M Antenna Mapping Tool (ESP32-S3)

PlatformIO project for a RoyalRay RRU7182M UHF RFID reader from an
ESP32-S3-DevKitC-1, per *UHF RFID Reader Series User Manual V2.25*.
Started as a basic connectivity demo; now a continuous 4-antenna
mapping tool for characterizing bag detection zones on/around the
cornhole board layout.

Craig Anderson

## What it does

1. On boot, sends **Obtain reader information** (Cmd `0x21`) and
   prints firmware version, model code, supported protocols, RF
   power, and inventory time — confirms the link is alive.
2. Enters a continuous **antenna mapping cycle**: steps through all 4
   antennas one at a time, runs a single-antenna **Tags inventory**
   (Cmd `0x01`) on each, and streams CSV
   (`timestamp_ms,label,antenna,epc,rssi`) over the USB serial
   console — the data needed to map detection zones and tune settings
   per antenna. See "Antenna mapping mode" below for the full picture.

## Wiring

| ESP32-S3 (DevKitC-1) | RRU7182M |
|---|---|
| GPIO43 (U0TXD) | RXD |
| GPIO44 (U0RXD) | TXD |
| GND | GND |

115200 8N1, no parity — confirmed by bench testing this unit over
the RRU7182EVB-IP-NL's RS232 port (the manual's stated series default
of 57600 did not respond; 115200 did). These pins are ESP32-S3's
default UART0 pins — free to reuse here because
`ARDUINO_USB_CDC_ON_BOOT=1` routes the `Serial` console over native
USB-CDC instead, the same build-flag setup already used on the
load-cell board. GPIO 38 (onboard WS2812), 19-20 (USB D-/D+), 26-37
(flash/PSRAM), and the 1-17 boot-strap range are avoided per prior
project experience — 43/44 fall outside all of those.

If you're bringing this up against the RRU7182EVB-IP-NL eval board
first (RS232 + TCP/IP), the frame format is identical — only the
byte pipe changes; you'd swap `Serial1` for a TCP socket in
`sendAndReceive()` once RoyalRay confirms the eval board's default
IP/login.

## Build

```
pio run -t upload
pio device monitor
```

## Bench-verified against real hardware

Confirmed via a Python/pyserial bench test over the eval board's
RS232-to-USB cable, and cross-checked over the board's TCP/IP (RJ45)
interface (see below), bypassing the ESP32 entirely to isolate
protocol from wiring:

- Actual baud rate: **115200** (not the manual's stated 57600 default)
- Firmware version **2.8**, model code `0x75`
- Protocol support: **EPC C1G2 (18000-6C) only** — confirm any tags
  you buy for bag ID are Gen2/6C
- Frequency band: **US band**
- Antenna check: on
- Response frame `Len` field = `dataLen + 5`, confirmed by CRC match
  on a live 18-byte response — the manual's own worked example for
  this command has a typo (states `Len=0x0F` for what is actually a
  12-byte payload, i.e. `Len=0x11`)
- Reported RF power value of **33** is outside the manual's stated
  0-30 range — not yet resolved, worth a check with RoyalRay if power
  control matters for the build

## TCP/IP transport (RJ45)

Neither the protocol manual nor the RRUx182M/RRUx182EVB-IP-NL
datasheets document TCP/IP setup — the datasheets just list the RJ45
interface as a bullet point with no elaboration, and RoyalRay's own
manual library has nothing dedicated to it either.

**Confirmed working:**
- Eval board's factory-default network config, printed on a sticker
  on the board: `192.168.1.190:6000` (static IP, no DHCP)
- Sent the identical "Obtain reader information" command frame
  (`04 00 21 D9 6A`) over a raw TCP socket to that IP:port and
  received a response **byte-for-byte identical** to the RS232
  response — confirms the manual's claim that frame format is
  transport-independent holds true here: TCP is a raw pass-through
  of the same command/response protocol, no extra handshake/envelope
  needed.

**Practical note:** `192.168.1.190` was outside your normal network's subnet
(confirmed: your PC's `192.168.4.x`/eero network needed a secondary adapter
IP in `192.168.1.0/24` to reach it). Initial decision was to leave the
reader's IP as factory default rather than risk a write with no documented
reset procedure, using a dedicated USB-Ethernet dongle for bench access
instead.

**That decision was revisited once ESP32/WiFi integration started.** A PC
can bind a second, arbitrary IP to work around a subnet mismatch; a WiFi
station cannot — it gets exactly one IP, issued within whatever subnet its
AP manages. There was no clean way for an ESP32 joined to the eero network
to also reach `192.168.1.190`. Given that, the reader's IP was moved onto
the actual deployment network instead:

- **New reader IP: `192.168.4.190`**, netmask `255.255.252.0`, gateway
  `192.168.4.1` — set via `rru_netconfig.py set-ip` (dry-run reviewed
  before `--apply`, per the tool's built-in confirmation gate)
- A DHCP reservation for the reader's MAC (`00:22:70:01:21:97`) pinned to
  `192.168.4.190` is recommended in the eero app, so the address can't
  later be handed to another device by DHCP
- Once moved, the dongle workaround is no longer needed for routine bench
  access — `192.168.4.190` is directly reachable from the normal Wi-Fi
  network
- `main.cpp`'s `READER_IP` and `bench_scripts/bench_test_tcp.py`'s default
  IP are both updated to `192.168.4.190`

**Config protocol discovered (separate from the port-6000 command
protocol):** ASCII-over-UDP on port 65535, reverse-engineered from the
`UHFReader288Demo` C# source in the Ex10 SDK (`Network_NL/` +
`Form1.cs`). Implemented in `rru_netconfig.py`. Full field/command map is
documented in that script's header docstring.

**Queried values from the live unit** (via `rru_netconfig.py query`, prior
to the IP move): `portnum=6000` (matches sticker), `br=7` (consistent with
the bench-confirmed 115200 baud on the RS232 side), `pr=0`/`bb=1` (no
parity/8 data bits, matches our 8N1 assumption), `rm=0` (server mode —
reader listens for connections, matching how we've connected so far),
`mac=00:22:70:01:21:97`. Gateway/remote-IP fields (`gi`, `di`) were
unconfigured loopback placeholders before the move; `gi` is now set to
`192.168.4.1` as part of the IP change.

## WiFi/TCP transport

`main.cpp` now supports both RS232 and WiFi/TCP, selected by the
`RRU_USE_WIFI_TRANSPORT` `#define` near the top of the file (currently
set to `1`, i.e. WiFi/TCP is active). Both paths use the identical
binary frame protocol — bench-confirmed byte-for-byte identical
responses over RS232 vs raw TCP — so switching transports is just a
different byte pipe, no protocol changes.

**Before building:** fill in `WIFI_SSID` / `WIFI_PASSWORD` in
`main.cpp` (currently placeholders). Consider moving these to
`platformio.ini` build flags instead if you plan to commit this to
version control, so credentials aren't sitting in tracked source.

**Resolved:** the reader's static IP was originally `192.168.1.190`
(factory default), which was on a different subnet than the ESP32's WiFi
network — a WiFi station can't bind a second IP the way a PC can to work
around that. Fixed by moving the reader's own IP onto the deployment
network instead: it's now `192.168.4.190` (see "Bench-verified against
real hardware" above for the full change and the DHCP-reservation
recommendation). `READER_IP` in this file reflects the new address.

## Bug fixed while adding WiFi/TCP support

While restructuring the transport layer, found that `main.cpp`'s
`sendAndReceive()` had its own independent copy of the response
frame-length calculation (needed to know how many bytes to pull off
the wire before handing off to `parseResponseFrame()`), and that copy
still had the same `Len` formula bug that was fixed in
`rru7182m.h`/`parseResponseFrame()` earlier — it read `1 + len + 2`
instead of the correct `1 + len`. This had never been caught because
all protocol validation up to this point went through the Python bench
scripts, not this actual firmware — first time this exact code path
would have run against real hardware. Both locations are now
consistent.

## Bug fixed after first live ESP32 run

First live run over WiFi/TCP printed `RF power = 128` and
`Antenna check = off`, contradicting the RS232-bench-captured values
(`Power=33`, `CheckAnt=on`) — not a reader-side change, a field-offset
bug in `obtainReaderInfo()`. `dmaxfre` and `dminfre` are two *separate*
single-byte fields (`view.data[4]` and `view.data[5]`), but the
original code only skipped one slot between `trType` and `power`, so
every field from `power` onward was read one byte early (`power` was
actually reading `dminfre`'s `0x80`, `Scntm` was actually reading the
real `power` value `0x21`=33, `CheckAnt` was reading a reserved byte).
Confirmed by matching `0x80`/`0x21` against the exact bytes captured
in the earlier Python cross-check, which used the correct offsets —
that check never got carried back into this C++ function, which is
why it went unnoticed until real hardware exposed it. Fixed; now also
prints the antenna mask and raw frequency-band bits for completeness.

## Antenna mapping mode

`main.cpp` now runs a continuous antenna-mapping cycle instead of the
earlier single-antenna connectivity demo: it steps through all 4
antennas one at a time (Cmd `0x3F`, "Setup antenna multiplexing"),
runs a single-antenna inventory pass on each (Cmd `0x01`), and streams
CSV over the USB serial console:

```
timestamp_ms,label,antenna,epc,rssi
```

One row per tag found each cycle, or a `NONE` row if nothing found
(so the log stays a dense, uniform time series). Type a label + Enter
into the serial console at any point to tag subsequent rows with a
board position; leave it as `unlabeled` and correlate by timestamp
afterward if typing while handling a tag isn't practical.

**Why one antenna at a time:** confirmed from the manual that the
inventory response's `Ant` field is a single value for the *whole*
response, not per-tag — so there's no way to tell which antenna found
which tag if more than one is active simultaneously. Enabling exactly
one antenna bit per cycle (`ANT_CONFIG_NOT_PERSISTED` set, so rapid
switching never gets written to persistent flash) is necessary for
unambiguous per-antenna data, not just a safety choice.

**Companion tool:** `bench_scripts/mapping_logger.py` captures the CSV
stream to a timestamped file on disk (`pip install pyserial`, then
`python mapping_logger.py COM11`), ready for pandas/matplotlib
analysis. You can also type labels directly into that script's console
instead of the PlatformIO monitor — it forwards them to the ESP32
the same way.

**Two things not yet bench-validated** (no tag on hand yet at time of
writing):
- **50ms settle delay** after switching antennas, before running
  inventory — not documented anywhere in the manual, a conservative
  guess. If antenna N's readings look like they're bleeding over from
  antenna N-1 right after a switch, increase this; if data looks clean
  without it, it can come down.
- **300ms inventory scan time** (`0x03`, the manual's stated minimum)
  — chosen for a responsive mapping cycle, but shorter scan time may
  reduce read reliability. If reads look flaky/inconsistent, try a
  longer scan time and accept the slower full-cycle rate.

Both are simple constants to tune (`delay(50)` in `runMappingCycle()`,
`setInventoryScanTime(0x03)` in `loop()`) once real data is in hand.

## Testing strategy and analysis notebook

See `analysis/TESTING_STRATEGY.md` for the full data-collection
protocol (coordinate system, label convention, grid layout, dwell
time, orientation testing) and `analysis/antenna_mapping_analysis.ipynb`
for a JupyterLab notebook that turns `mapping_logger.py`'s CSV output
into per-antenna RSSI/detection-rate heatmaps, an on-board vs.
off-board RSSI separation analysis, per-antenna threshold
recommendations, and an effective-coverage plot. The notebook's
analysis logic was validated against synthetic data before delivery —
see `analysis/README.md`.

## Known assumption to verify against real hardware

The manual is ambiguous about whether the per-tag "data length" byte
in an inventory response counts EPC bytes or EPC words (Section
8.2.1). This code assumes **bytes**, matching common firmware in this
reader family. If EPCs print truncated or garbled once a tag is in
range, that's the first thing to check — capture a raw response
frame and compare the length byte against the actual EPC byte count,
consistent with confirming physical/protocol behavior before
adjusting more code.

## Not yet implemented (natural next steps)

- Multi-frame inventory responses (`Status = 0x03`, "further data in
  next frame")
- FastID (EPC+TID in one pass) — currently detected but not decoded
- Writing to / reading tag memory, EPC programming (Sections 8.2.2-8.2.4)
- TCP/IP transport for the eval board
