"""
Bench test: send commands to the RRU7182M eval board over TCP/IP (RJ45),
using the IP:port printed on the board's sticker. Tests whether the
command/response frame format is a raw pass-through identical to the
RS232 path we already validated.

Usage:
    python bench_test_tcp.py                       (uses default 192.168.1.190:6000)
    python bench_test_tcp.py --cmd inventory
    python bench_test_tcp.py --ip 192.168.1.190 --port 6000 --cmd info
"""

import argparse
import socket

DEFAULT_IP = "192.168.4.190"  # moved here from 192.168.1.190 via rru_netconfig.py
DEFAULT_PORT = 6000

FRAMES = {
    # Len=0x04, Adr=0x00, Cmd=0x21 (obtain reader info), no Data[]
    "info": bytes([0x04, 0x00, 0x21, 0xD9, 0x6A]),
    # Len=0x06, Adr=0x00, Cmd=0x01 (tags inventory), Data=[QValue=0x04, Session=0x00]
    "inventory": bytes([0x06, 0x00, 0x01, 0x04, 0x00, 0xAC, 0x36]),
}


def main():
    parser = argparse.ArgumentParser(description="RRU7182M TCP/IP bench test")
    parser.add_argument("--ip", default=DEFAULT_IP, help=f"Reader IP (default {DEFAULT_IP})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                         help=f"Reader TCP port (default {DEFAULT_PORT})")
    parser.add_argument("--cmd", choices=FRAMES.keys(), default="info",
                         help="Which command to send (default: info)")
    parser.add_argument("--timeout", type=float, default=2.0,
                         help="Socket timeout in seconds (default 2.0)")
    args = parser.parse_args()

    frame = FRAMES[args.cmd]

    print(f"Connecting to {args.ip}:{args.port} ...")
    with socket.create_connection((args.ip, args.port), timeout=args.timeout) as sock:
        sock.settimeout(args.timeout)
        print(f"Sending: {frame.hex(' ').upper()}")
        sock.sendall(frame)

        try:
            response = sock.recv(256)
        except socket.timeout:
            response = b""

        if not response:
            print("No response received within timeout.")
            return

        print(f"Received ({len(response)} bytes): {response.hex(' ').upper()}")

if __name__ == "__main__":
    main()
