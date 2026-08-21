# RRU7182M Antenna Mapping Tool (ESP32-S3)

PlatformIO project for a RoyalRay RRU7182M UHF RFID reader from an
ESP32-S3-DevKitC-1, per *UHF RFID Reader Series User Manual V2.25*.
Started as a basic connectivity demo; now a continuous 4-antenna
mapping tool for characterizing bag detection zones on/around the
cornhole board layout.

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

## Bugs found and fixed after moving to a new network

On a different network (same physical setup, different IP), startup
showed a cluster of mixed timeouts, failed antenna switches, and a
"Response too large for buffer" error, then self-recovered after a
few seconds — plus the reader was audibly beeping. Three separate
things going on, diagnosed rather than just patched over:

- **The buzzer beeping is meaningful, not a fault alert.** Confirmed
  from the manual (Section 8.4.9): it beeps on every *successful* tag
  read. Continuous beeping means the reader is genuinely, repeatedly
  detecting something — worth checking for a tag or tag-like object
  left in range, since that directly explains the next two points.
- **`respFrame[MAX_FRAME_LEN]` was sized at 255 bytes, but the true
  worst case is 256** (1 Len byte + up to 255 more, since Len's value
  already accounts for the rest of the frame). A large response —
  exactly what a backlog of repeated tag detections would produce —
  overflowed that by one byte. Fixed via a new `MAX_TOTAL_FRAME_BYTES`
  constant (`rru7182m.h`), used for the inventory response buffer.
- **The RS232 manual's <15ms inter-byte gap requirement was being
  applied to TCP too.** TCP can legitimately deliver one logical frame
  across multiple network segments with larger gaps, especially a
  large frame on a network with different latency than the one this
  was bench-tested on. When that gap check fired mid-frame, the
  function returned immediately *without draining the rest of that
  frame's bytes still in flight* — so the next call read one of those
  stale bytes as if it were a fresh Len byte, misaligning parsing
  entirely. That explains the specific pattern observed: different
  commands failing back-to-back in a burst, then recovering on their
  own once alignment happened to resync by chance. Fixed two ways:
  `INTERBYTE_TIMEOUT_MS` is now transport-conditional (300ms for TCP,
  the original 15ms preserved for RS232), and a new
  `drainStaleBytes()` helper is called from every failure exit of
  `sendAndReceive()` to actively consume lingering bytes before the
  next exchange, rather than relying on chance resync.

**Also improved while in there:** a `STATUS_0x03` ("more buffered
results than fit in this frame") response was previously discarded
entirely, throwing away real tag data that *did* fit in the frame just
to avoid dealing with the unfetched remainder. Now parses and prints
what's present, with a note that the remainder wasn't fetched. Full
continuation-frame fetching still isn't implemented — worth doing
properly once real gameplay means multiple tagged bags in the field
at once, since `0x03` will then be routine, not an edge case.

**Confirmed real tag, and a fourth issue found on the next test run.**
The buzzer theory checked out — `302832AB0CE502869172838A` came through
cleanly on all 4 antennas (RSSI 56-63). But timeout/failed-switch churn
was still substantial even with the fixes above, pointing at something
beyond TCP framing. The manual states directly: "an unreasonable short
inventory time setting may cause tag inventory failure" (Section
8.2.1) — and the scan time was set to `0x03` (300ms), the documented
*minimum* of the valid range, specifically chosen for a responsive
mapping cycle. Combined with a real tag being continuously, rapidly
detected (heavy internal buffering), this is very likely the dominant
cause of the instability, not just a secondary factor. Backed off to
`MAPPING_SCAN_TIME_100MS = 0x08` (800ms), with `RESPONSE_TIMEOUT_MS`
raised to 1500ms to match — these two are coupled (response can't
possibly arrive before the scan itself completes, plus the manual
notes actual scan time can run up to 75ms over the configured value),
so changing one without the other will make every exchange fail by
construction. Not yet re-validated against real hardware at time of
writing — first thing to check on the next run.

**That re-validation surfaced a different, more specific pattern**:
exactly one successful exchange (reader info) after connecting, then
every single subsequent command failed, with no recovery in the whole
log window — a tighter signature than general timing pressure. Most
likely explanation: the TCP connection went dead-in-practice (reader
silently stopped answering without sending a FIN/RST), which
`reader.connected()` can't detect on its own since the socket-level
state still looks fine. Root cause not yet fully confirmed — an RS232
test would help isolate whether this is TCP/WiFi-stack-specific or a
reader-side command-processing issue, independent of transport (see
"WiFi/TCP transport" section for the transport switch).

Regardless of root cause, the firmware had no way to recover from this
state on its own, which is a real gap for anything meant to run
unattended. Added: `inventoryOnceCSV()` and the antenna-switch call in
`runMappingCycle()` now feed a consecutive-failure counter
(`MAX_CONSECUTIVE_FAILURES = 6`, roughly one full antenna cycle); if
exceeded, the code forces `reader.stop()` (WiFi/TCP only) and resets
`readerOnline`/`mappingModeReady`, so `loop()` re-establishes the
connection from scratch — including re-fetching reader info and
re-setting scan time — rather than looping forever against a
connection nothing will ever answer on again.

## Antenna mapping mode

`main.cpp` runs a continuous antenna-mapping cycle instead of the
earlier single-antenna connectivity demo: it steps through antennas
one at a time (Cmd `0x3F`, "Setup antenna multiplexing"), runs a
single-antenna inventory pass on each (Cmd `0x01`), and streams CSV
over the USB serial console:

```
timestamp_ms,label,antenna,epc,rssi
```

**Currently limited to antenna 1 only** (`MAPPING_SINGLE_ANTENNA_ONLY`
in `main.cpp`, set to `1`) while confirming comms reliability with
fewer variables in play. Set it to `0` to resume cycling through all 4
antennas once satisfied with stability.

**RF power is set to 10 at startup** (`MAPPING_RF_POWER`, non-persistent
— overrides the reader's own configured 33/~2W for this session only).
Originally added to test whether a foil-wrapped tag in another room
was genuinely still being read over the air (a lot of transmit power
paired with a sensitive receiver can punch through casual shielding
and an interior wall more easily than intuition suggests), but it's
not just a diagnostic value — lower power should also shrink the
off-board false-read radius, which is the actual point of the mapping
exercise, and it directly helps battery life for field deployment
(current draw scales steeply with power). Once real mapping data says
what the board actually needs, update `MAPPING_RF_POWER` to that value
instead of this test number. Valid range is 0-30 — this is a linear
scale, not a percentage, so "10" is roughly a third of max, not "10%."

**`rf_power` is now a real column in the CSV output**, not just a
firmware constant to remember separately — every row records the power
level it was captured at. This means sessions collected at different
power levels can be loaded into the analysis notebook together and
compared directly, which is exactly what's needed to answer "what's
the lowest power that still covers the board." See
`analysis/TESTING_STRATEGY.md`'s "RF power sweep" section for the
procedure, and Section 7 of the notebook for the resulting
coverage-vs-power comparison. Older CSVs collected before this change
won't have the column — the notebook flags and skips them rather than
guessing a value.

**EPC Gen2 Session changed from S0 to S1** (in `inventoryOnceCSV()`).
Cause: the manual confirms a tag read multiple times within one
inventory pass isn't deduplicated, and with the current 2.0s scan
window, a reliably-present tag was getting read dozens of times per
pass — filling the response frame with repeats of the same tag,
which is exactly why the `STATUS_0x03` "more buffered results" note
was showing up on essentially every detection. Harmless for single-tag
testing, but a real risk once multiple different tagged bags are in
play simultaneously: one tag's repeats could fill the frame before a
second, genuinely different tag ever gets reported. S1 makes an
already-read tag go quiet for the rest of that window (standard Gen2
behavior), freeing frame capacity for other distinct tags.

Tradeoff: S1's quiet period is a real-time timer on the tag itself,
not reset between back-to-back inventory calls during continuous
mapping-mode polling — a tag read near the end of one call may still
read as quiet at the start of the next, which can show up as extra
`NONE` results specifically in this continuous-polling test pattern.
Not a problem for the actual deployment use case (one on-demand read
at end of round, not continuous polling) — just don't mistake an
uptick in `NONE` rows during mapping-mode testing for a regression in
the scan-time work above; it's this tradeoff, not a new bug.

**Fixed a silent-failure gap in `setAntennaMultiplexing()` /
`setInventoryScanTime()` / `setRFPower()`.** A `# WARNING: failed to
switch to antenna N` with no `[RRU] ...` diagnostic line before it
meant the frame was received completely and cleanly at the transport
level — `sendAndReceive()` already logs every timeout/oversized/
gap-exceeded failure it detects, and none of those fired. The actual
failure was happening silently one layer up: these three functions
returned `false` on a CRC/format mismatch or an unexpected
`reCmd`/`status` without ever printing why, making every such failure
indistinguishable from every other. Most likely explanation for this
specific pattern: a large inventory response (common under the old
Session S0 duplicate-heavy responses) leaving trailing bytes that the
very next, smaller exchange misreads as its own response start — the
same class of issue as the earlier TCP inter-byte-gap desync fix, just
a case that fix didn't catch since the frame the *following* command
read was still fully well-formed by our size check, just not the
content expected. All three functions now print the raw response
bytes on a CRC/format failure, and the actual `reCmd`/`status`
received vs. expected on a mismatch — if it's stale data, this will
show a `reCmd` matching a previous command (e.g. `0x01` where `0x3F`
was expected) rather than garbage, confirming the theory directly
instead of leaving it as a guess.

**Confirmed on real hardware, and root-caused, not just diagnosed.**
The new logging immediately caught it: `unexpected reCmd=0x01
status=0x01 (wanted reCmd=0x3F status=0x00)` — a complete, CRC-valid
`CMD_TAG_INVENTORY`/`STATUS_OK` response arriving where the antenna-mux
response was expected. Not corruption; a genuine leftover response to
the *previous* command. The reader appears to push backlogged data
(the remainder behind a `STATUS_0x03` response) unprompted, shortly
after the initial response, without ever being asked for a
continuation — late enough to land in the gap between
`sendAndReceive()`'s instantaneous start-of-call flush and sending the
next command, where it then gets misread as that command's response.

Fixed by closing the race rather than continuing to detect it after
the fact: the start-of-call flush now waits for a genuine 100ms quiet
window (reusing `drainStaleBytes()`) instead of just flushing whatever
happens to be sitting in the buffer at that instant. Adds up to 100ms
per command, which is a non-issue given there's no latency requirement
for this system (see the `MAPPING_SCAN_TIME_100MS` reasoning above).

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

**One thing not yet bench-validated:**
- **50ms settle delay** after switching antennas, before running
  inventory — not documented anywhere in the manual, a conservative
  guess. If antenna N's readings look like they're bleeding over from
  antenna N-1 right after a switch, increase this; if data looks clean
  without it, it can come down.

**Scan time history:** 300ms (`0x03`, the documented minimum) produced
widespread timeouts and failed antenna switches once a real tag was
present and being repeatedly detected — the manual directly warns
short scan times can cause this. Backed off to 800ms, which fixed the
timeouts. Then reconsidered entirely: this system only reads at the
end of a round, not continuously, so there's no latency requirement to
optimize for in the first place — a short scan trades reliability for
responsiveness that isn't needed here, and single-shot misses in the
CSV log (alternating `NONE`/detected) were really just a symptom of
too little time-per-scan for EPC Gen2's anti-collision slotting to
consistently catch the tag, not a bug. Now `MAPPING_SCAN_TIME_100MS =
0x14` (2.0s), with `RESPONSE_TIMEOUT_MS` raised to match — the two are
coupled, not independent tunables; see the comment above
`MAPPING_SCAN_TIME_100MS` in `main.cpp` before changing either one.
Push higher (documented max `0xFF`/25.5s) if `NONE` results are still
frequent — see that comment for the full reasoning, including why this
isn't guaranteed to eliminate every miss (momentary RF fades are a
separate, real phenomenon a longer window helps but doesn't fully
prevent). Dwell-time guidance in `analysis/TESTING_STRATEGY.md` was
updated to match this pacing.

The `delay(50)` settle constant is in `runMappingCycle()`.

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
