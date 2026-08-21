# Antenna Mapping — Testing Strategy

Goal: characterize where each of the 4 antennas actually detects a tag,
well enough to (a) confirm bags on the board get reliably read and (b)
tune settings (RSSI threshold, and later RF power/scan time) to
minimize false reads from bags/tags off the board.

## Coordinate system (must match the notebook's config)

Origin `(0,0)` = front-left corner of the playing surface (the edge
closest to the thrower). `+x` = right, `+y` = away from the thrower,
toward the hole/back edge. Standard board: 24"(x) x 48"(y), hole
centered at `x=12, y=39` (9" from the back edge). Update
`BOARD_WIDTH`/`BOARD_LENGTH`/hole position in the notebook's config
cell if your board differs.

Antenna positions go in the same coordinate system, in the notebook's
`ANTENNA_POSITIONS` dict — update it to match wherever you actually
mount them (corner-mounted, matching the load-cell layout, is the
working assumption but edit to match reality).

## Label convention

The firmware's mapping mode lets you type a free-text label at any
time; for this exercise, use **`x<X>_y<Y>`** in inches, e.g. `x12_y24`,
`x-12_y-6` for off-board points. The notebook's `parse_label()` parses
exactly this format — anything else (including the default
`unlabeled`) gets excluded from the position-based plots, though it's
still usable for basic RSSI checks.

## Grid layout — two phases

**Phase A: coarse, full coverage.** 12" spacing across the board
surface (`x = 0,12,24`; `y = 0,12,...,48` → 15 points) plus a matching
12" grid extending into the off-board margins — most importantly the
**front margin** (`y = -12, -24, -36`, roughly matching `x` range plus
a bit extra), since that's where thrown bags travel, land short, or
where players/spectators stand with their own tagged items. Cover the
side and back margins too, more thinly (one or two rows), since a
second board or foot traffic could realistically be nearby.

This phase is about the big picture — is there a large blind spot on
the board, or a large false-read zone off it — before spending time on
fine resolution.

**Phase B: fine, boundary-focused.** Once Phase A shows roughly where
the transition happens, drop to 3-6" spacing specifically straddling
the board edges (a band from ~12" inside the edge to ~12" outside it,
all the way around). This is the data that actually determines your
RSSI threshold — the board-interior and far-off-board points from
Phase A already tell you those are safely "on" or "off"; the edge band
is where the real decision boundary lives.

## Per-position procedure

1. Place the tag at the grid position. Bag placement matters more than
   an isolated tag — if you have a bag with the tag embedded, use it;
   a bare tag taped flat to the board surface is a reasonable stand-in
   if that's what's on hand for early passes.
2. Type the position label (`x<X>_y<Y>`) into the serial console
   (PlatformIO monitor or `mapping_logger.py`'s console — both forward
   typed text to the ESP32 the same way).
3. **Dwell at least 20 seconds** before moving to the next position.
   Scan time is currently 2.0s per antenna (`MAPPING_SCAN_TIME_100MS`
   in `main.cpp`) — deliberately generous, since this system only
   needs to read at the end of a round, not continuously, so there's
   no reason to trade reliability for responsiveness here. A full
   4-antenna cycle runs roughly 9-10s depending on comms overhead, so
   a 20s dwell comfortably covers 2+ full cycles, giving several
   samples per antenna at that position — worth keeping even with a
   longer per-scan window, since EPC Gen2's anti-collision slotting
   and momentary RF fades mean even a well-placed tag won't register
   on literally every cycle. More samples also let the notebook's
   `detection_rate` metric mean something rather than being a
   single noisy coin flip.
4. Move to the next position and repeat.

## Tag orientation

UHF tag read range depends heavily on polarization alignment between
tag and antenna — a bag landing flat, on edge, or crumpled will read
very differently. Rather than a full second grid, test **2-3
orientations at a reduced subset of positions** (e.g., a diagonal
cross-section through the board, or just the boundary band from Phase
B) to get a sense of how much orientation matters. If it turns out to
matter a lot, that's a finding worth having before committing to RFID
as a sole detection method — it may point toward a different tag
form-factor, or accepting some orientation-dependent blind spots.

## Environmental controls

- Keep other RFID-tagged items away from the test area during
  sessions (any nearby Gen2 tag will show up in inventory results too).
- Note whether testing is indoors/outdoors and any nearby metal
  objects (folding chairs, coolers, etc.) that could reflect or
  attenuate RF — cornhole is often played outdoors, and reflective
  surfaces can behave differently than a clean bench test.
- Keep scan time and Q-value constant across a full session. RF power
  is the one setting you'll deliberately vary between sessions — see
  below — since it's now recorded per-row in the CSV itself and the
  notebook is built to compare across it directly.

## RF power sweep

Unlike scan time and Q-value, RF power is worth deliberately varying
across multiple full sessions, because the actual goal is finding the
lowest power that still reliably covers the board — lower power both
tightens the off-board false-read radius and meaningfully extends
battery life for field deployment.

**Scale note:** the reader's valid range is 0-30 (not a percentage —
a setting of 10 is roughly a third of max power, not "10%"). The
reader's own configured default is 33, which is actually outside that
documented range (~2W); the mapping firmware overrides this at startup
via `MAPPING_RF_POWER` in `main.cpp`.

**Procedure:**

1. Set `MAPPING_RF_POWER` to a chosen value, reflash, and run a full
   mapping session (Phase A grid at minimum) — `rf_power` is captured
   automatically in every row, so there's no need to track it
   separately or keep sessions in different folders.
2. Repeat at 2-4 more power levels, spanning a reasonable range (e.g.
   10, 15, 20, 25) — you don't need to test every value, just enough
   points to see the trend.
3. Load all the sessions together in the notebook (they merge
   automatically) and check Section 7, "Coverage vs. power level" —
   it plots on-board detection rate against off-board detection rate
   (false-read risk) across every power level in your data. The power
   where on-board coverage starts dropping meaningfully, or where
   off-board bleed stops improving much further, is your practical
   floor.
4. Once you've picked a value, that's what `MAPPING_RF_POWER` should
   be set to going forward — not the diagnostic value (10) used while
   debugging comms earlier this session.

Older CSV files collected before this notebook version won't have an
`rf_power` column — the notebook flags and skips these automatically
rather than guessing a value, so they need to be re-collected if you
want them included in a comparison.

## What "done" looks like

Run `mapping_logger.py`, walk the grid per the procedure above, then
open `antenna_mapping_analysis.ipynb`. You're looking for:

- Heatmaps (Section 4) showing each antenna strongest near its own
  corner and falling off with distance, without odd dead zones inside
  the board.
- A visible gap between on-board and off-board RSSI distributions
  (Section 5) — if the histograms overlap heavily, that's the signal
  to either accept a wider tolerance, shorten scan time, lower RF
  power (see the power sweep above), or reconsider antenna
  placement/angle.
- The effective-coverage plot (Section 6) roughly matching the true
  board rectangle at your chosen threshold, with false positives/
  negatives called out explicitly in the printed counts.
- If you've run the power sweep, a clear knee in the coverage-vs-power
  plot (Section 7) — that's your practical power floor.

If Phase A alone gives a clean answer, Phase B may not be necessary —
but for tuning a real threshold with confidence, the boundary-focused
data is usually what actually matters.
