# Antenna Mapping Analysis

- **`TESTING_STRATEGY.md`** — the data collection protocol: coordinate
  system, label convention, grid layout, dwell time, orientation
  testing, RF power sweep procedure. Read this before collecting data,
  not after.
- **`antenna_mapping_analysis.ipynb`** — JupyterLab notebook that
  loads `mapping_*.csv` files (from `bench_scripts/mapping_logger.py`)
  and produces per-antenna RSSI/detection-rate heatmaps, an on-board
  vs. off-board RSSI separation analysis, per-antenna threshold
  recommendations, an effective-coverage plot at a chosen threshold,
  and a coverage-vs-power comparison if your data spans multiple power
  levels.

## Quick start

1. Collect data per `TESTING_STRATEGY.md`, using
   `bench_scripts/mapping_logger.py` to capture one or more
   `mapping_<timestamp>.csv` files. `rf_power` is recorded
   automatically in every row — no separate tracking needed, and
   sessions collected at different power levels can be loaded and
   compared together in the same notebook run.
2. Copy (or symlink) those CSV files into this `analysis/` folder, or
   point the notebook's `CSV_FILES` glob at wherever they live.
3. Open `antenna_mapping_analysis.ipynb` in JupyterLab, edit the
   config cell (board dimensions, antenna positions, which power
   level to focus the single-power plots on) to match your actual
   layout, and run all cells top to bottom.

## Validated, not just written

The notebook's analysis logic (parsing, aggregation, heatmaps,
threshold recommendation, coverage plot, power comparison) was tested
end-to-end against synthetic multi-power-level data shaped like real
mapping output before being handed over — including a synthetic
old-format file (no `rf_power` column) to confirm it gets flagged and
skipped rather than silently mishandled. Not just checked for syntax.
Real data will surely surface things synthetic data didn't (see the
"not yet bench-validated" notes in the main README for the firmware
side), but the analysis pipeline itself runs clean.

