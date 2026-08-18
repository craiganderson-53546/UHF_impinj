"""
Bench test: send commands to the RRU7182M over the eval board's RS232-to-USB
cable, and print the raw response. Confirmed against real hardware at
115200 baud (the manual's stated 57600 default did not respond on this unit).

Usage:
    pip install pyserial

    # Obtain reader information (Cmd 0x21)
    python bench_test.py COM9 --cmd info

    # Tags inventory (Cmd 0x01) -- place a tag near the antenna first
    python bench_test.py COM9 --cmd inventory

    # Override baud if needed
    python bench_test.py COM9 --cmd info --baud 57600

    # Loopback self-test: short DB9 pins 2+3 together, then run this to
    # confirm the cable/driver/COM port round-trip your own bytes correctly,
    # with the reader out of the picture entirely.
    python bench_test.py COM9 --loopback
"""

import argparse
import serial

CONFIRMED_BAUD = 115200  # verified against real hardware; manual states 57600

FRAMES = {
    # Len=0x04, Adr=0x00, Cmd=0x21 (obtain reader info), no Data[]
    "info": bytes([0x04, 0x00, 0x21, 0xD9, 0x6A]),
    # Len=0x06, Adr=0x00, Cmd=0x01 (tags inventory), Data=[QValue=0x04, Session=0x00]
    "inventory": bytes([0x06, 0x00, 0x01, 0x04, 0x00, 0xAC, 0x36]),
}


def main():
    parser = argparse.ArgumentParser(description="RRU7182M RS232 bench test")
    parser.add_argument("port", help="COM port, e.g. COM9")
    parser.add_argument("--baud", type=int, default=CONFIRMED_BAUD,
                         help=f"Baud rate (default {CONFIRMED_BAUD}, bench-confirmed)")
    parser.add_argument("--cmd", choices=FRAMES.keys(), default="info",
                         help="Which command to send (default: info)")
    parser.add_argument("--loopback", action="store_true",
                         help="Loopback mode: expects DB9 pins 2+3 bridged, "
                              "verifies the cable echoes bytes back unchanged "
                              "(ignores --cmd, always sends the 'info' frame)")
    args = parser.parse_args()

    frame = FRAMES["info"] if args.loopback else FRAMES[args.cmd]

    with serial.Serial(args.port, args.baud, bytesize=8, parity='N', stopbits=1,
                        timeout=1) as ser:
        ser.dtr = True
        ser.rts = True

        mode = "loopback" if args.loopback else args.cmd
        print(f"Port: {args.port}  Baud: {args.baud}  Mode: {mode}")
        print(f"Sending: {frame.hex(' ').upper()}")
        ser.write(frame)

        response = ser.read(128)  # inventory responses can run longer than info

        if not response:
            print("No response received.")
            if args.loopback:
                print("Loopback failed -- check the jumper and COM port.")
            else:
                print("Try --loopback first, or double-check --baud.")
            return

        print(f"Received ({len(response)} bytes): {response.hex(' ').upper()}")

        if args.loopback:
            status = "OK" if response == frame else "MISMATCH -- check jumper/pins"
            print(f"Loopback {status}")

if __name__ == "__main__":
    main()
