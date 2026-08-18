"""
Captures the antenna-mapping CSV stream from the ESP32 (main.cpp's mapping
mode) over USB serial, and writes it to a timestamped CSV file on disk --
ready for pandas/matplotlib analysis (e.g. building an RSSI heatmap per
antenna across board positions).

The firmware prints:
    timestamp_ms,label,antenna,epc,rssi
one row per tag found each cycle (or a "NONE" row if nothing found), plus
occasional "# ..." comment/warning lines. This script passes data rows
straight through to the CSV file, prints comment lines to the console for
visibility, and echoes everything live so you can watch it while testing.

You can also type directly into this script's console while it's running --
anything you type gets forwarded to the ESP32 as a position label, exactly
like typing into the PlatformIO serial monitor would. (Optional: if that's
awkward while your hands are busy with a tag, just don't type anything --
rows will stay under "unlabeled" and you can correlate by timestamp using
your own notes afterward.)

Usage:
    pip install pyserial
    python mapping_logger.py COM11
    python mapping_logger.py COM11 --out my_mapping_session.csv
"""

import argparse
import csv
import datetime
import sys
import threading

import serial

BAUD = 115200  # matches Serial.begin(115200) in main.cpp (USB-CDC, value mostly nominal)


def stdin_forwarder(ser):
    """Reads lines typed into this script's console and forwards them to
    the ESP32 as position labels, non-blocking relative to the read loop."""
    for line in sys.stdin:
        line = line.strip()
        if line:
            ser.write((line + "\n").encode("ascii"))


def main():
    parser = argparse.ArgumentParser(description="RRU7182M antenna mapping CSV logger")
    parser.add_argument("port", help="ESP32 USB serial port, e.g. COM11")
    parser.add_argument("--out", default=None,
                         help="Output CSV path (default: mapping_<timestamp>.csv)")
    args = parser.parse_args()

    out_path = args.out or f"mapping_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"

    print(f"Connecting to {args.port} at {BAUD} baud...")
    ser = serial.Serial(args.port, BAUD, timeout=1)

    # Forward typed labels to the ESP32 in the background so this script
    # doesn't block waiting on stdin while data is streaming in.
    forwarder = threading.Thread(target=stdin_forwarder, args=(ser,), daemon=True)
    forwarder.start()

    print(f"Logging to {out_path}. Type a label + Enter to tag rows on the ESP32; "
          f"Ctrl+C to stop.\n")

    rows_written = 0
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        header_written = False

        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue

                if line.startswith("#"):
                    print(line)
                    continue

                if line.startswith("timestamp_ms"):
                    # Firmware's own CSV header -- write it once, ignore repeats
                    # (e.g. if the ESP32 reconnects/reboots mid-session).
                    if not header_written:
                        writer.writerow(line.split(","))
                        header_written = True
                    continue

                fields = line.split(",")
                if len(fields) != 5:
                    print(f"(unrecognized line, skipped) {line}")
                    continue

                writer.writerow(fields)
                rows_written += 1
                print(line)

        except KeyboardInterrupt:
            pass
        finally:
            f.flush()
            ser.close()

    print(f"\nStopped. Wrote {rows_written} data rows to {out_path}.")


if __name__ == "__main__":
    main()
