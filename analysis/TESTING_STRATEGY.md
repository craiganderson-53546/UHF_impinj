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
3. **Dwell at least 10 seconds** before moving to the next position.
   At the mapping firmware's current settings (~300ms scan time per
   antenna, ~50ms settle, roughly 1.5-2.5s per full 4-antenna cycle
   depending on comms overhead), this yields several samples per
   antenna at that position — important because EPC Gen2's
   anti-collision slotting means a tag well within range still won't
   answer *every* query, so single-shot reads are noisy. More samples
   also let the notebook's `detection_rate` metric mean something.
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
- Keep the reader's settings (RF power, scan time, Q-value) constant
  across a full session; if you deliberately change one to test its
  effect, start a new labeled session/file rather than mixing settings
  mid-dataset — the notebook doesn't currently track per-row settings.

## What "done" looks like

Run `mapping_logger.py`, walk the grid per the procedure above, then
open `antenna_mapping_analysis.ipynb`. You're looking for:

- Heatmaps (Section 4) showing each antenna strongest near its own
  corner and falling off with distance, without odd dead zones inside
  the board.
- A visible gap between on-board and off-board RSSI distributions
  (Section 5) — if the histograms overlap heavily, that's the signal
  to either accept a wider tolerance, shorten scan time, lower RF
  power (once that firmware command exists — not yet implemented as
  of this session), or reconsider antenna placement/angle.
- The effective-coverage plot (Section 6) roughly matching the true
  board rectangle at your chosen threshold, with false positives/
  negatives called out explicitly in the printed counts.

If Phase A alone gives a clean answer, Phase B may not be necessary —
but for tuning a real threshold with confidence, the boundary-focused
data is usually what actually matters.
