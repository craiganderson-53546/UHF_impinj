"""
RRU7182EVB-IP-NL network configuration tool.

Implements the ASCII-over-UDP discovery/config protocol used by RoyalRay's
UHFReader288Demo (C# demo, Network_NL module + Form1.cs). This is a SEPARATE
protocol from the binary command protocol on port 6000 (Len/Adr/Cmd/Data/CRC)
that bench_test.py / bench_test_tcp.py exercise -- this one lives on UDP
port 65535 and is plain ASCII text, used only for discovery and for setting
network/serial parameters on the reader itself.

Reverse-engineered field/command map (G = get, S = set, followed by the
value for S, then "|<index>"):

    Field    Get      Set        Meaning
    -----    ---      ---        -------
    usename  GON|1    SON..|18   Owner/user name
    dsname   GDN|2    SDN..|19   Device name
    mac      GFE|3    (n/a)      MAC address -- read-only, never set
    IP       GIP|4    SIP..|37   Reader's own IP address
    portnum  GPN|5    SPN..|21   Reader's own listening port (6000 on this unit)
    tup      GTP|6    STP..|20   Protocol (TCP/UDP mode)
    rm       GRM|7    SRM..|22   Work mode (server/client)
    cm       GCM|8    SCM..|29   Network advanced option
    ct       GCT|9    SCT..|30   Network advanced option
    fc       GFC|10   SFC..|23   Serial: flow control / RTS
    dt       GDT|11   SDT..|24   Serial: DTR mode
    br       GBR|12   SBR..|25   Serial: baud rate
    pr       GPR|13   SPR..|26   Serial: parity
    bb       GBB|14   SBB..|27   Serial: data bits
    rc       GRC|15   SRC..|28   Serial advanced option
    ml       GML|16   SML..|31   Serial advanced option
    md       GMD|17   SMD..|32   Serial advanced option
    di       GDI|18   SDI..|33   Remote/destination IP (client work mode only)
    dp       GDP|19   SDP..|34   Remote/destination port (client work mode only)
    gi       GGI|20   SGI..|35   Gateway IP
    nm       GNM|21   SNM..|36   Netmask

Sequencing observed in the demo:
    Query:  "L" (lock/select device) -> each G command, one at a time
    Write:  "L" -> each S command, one at a time -> "E" (commit/apply)

IMPORTANT: this writes persistent configuration to the reader over UDP,
with no documented reset procedure if something goes wrong. This script
always does a full read-modify-write (queries every field before changing
only the ones you asked for) and requires typed confirmation before
sending anything. Query/discover freely; be deliberate about --apply.

Usage:
    python rru_netconfig.py discover
    python rru_netconfig.py query 192.168.1.190
    python rru_netconfig.py set-ip 192.168.1.190 --new-ip 192.168.4.190 \\
        --netmask 255.255.252.0 --gateway 192.168.4.1
    python rru_netconfig.py set-ip 192.168.1.190 --new-ip 192.168.4.190 \\
        --netmask 255.255.252.0 --gateway 192.168.4.1 --apply
"""

import argparse
import socket
import time

CONFIG_PORT = 65535
RETRIES = 3
RECV_TIMEOUT = 0.3  # seconds; demo uses 50ms but that's aggressive for Wi-Fi/Python overhead

# field -> (get_cmd, get_idx, set_cmd, set_idx)
FIELDS = {
    "usename": ("GON", 1, "SON", 18),
    "dsname":  ("GDN", 2, "SDN", 19),
    "IP":      ("GIP", 4, "SIP", 37),
    "portnum": ("GPN", 5, "SPN", 21),
    "tup":     ("GTP", 6, "STP", 20),
    "rm":      ("GRM", 7, "SRM", 22),
    "cm":      ("GCM", 8, "SCM", 29),
    "ct":      ("GCT", 9, "SCT", 30),
    "fc":      ("GFC", 10, "SFC", 23),
    "dt":      ("GDT", 11, "SDT", 24),
    "br":      ("GBR", 12, "SBR", 25),
    "pr":      ("GPR", 13, "SPR", 26),
    "bb":      ("GBB", 14, "SBB", 27),
    "rc":      ("GRC", 15, "SRC", 28),
    "ml":      ("GML", 16, "SML", 31),
    "md":      ("GMD", 17, "SMD", 32),
    "di":      ("GDI", 18, "SDI", 33),
    "dp":      ("GDP", 19, "SDP", 34),
    "gi":      ("GGI", 20, "SGI", 35),
    "nm":      ("GNM", 21, "SNM", 36),
}
MAC_GET = ("GFE", 3)  # read-only, reported but never written back


def send_recv(sock, addr, message, expect_prefix="A"):
    """Send an ASCII request, retry up to RETRIES times, return the reply
    string (without leading 'A') or None if no reply arrived."""
    for _ in range(RETRIES):
        sock.sendto(message.encode("ascii"), addr)
        sock.settimeout(RECV_TIMEOUT)
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            continue
        text = data.decode("ascii", errors="replace")
        if text.startswith(expect_prefix):
            return text[len(expect_prefix):]
    return None


def discover(bind_ip="0.0.0.0", timeout=2.0, broadcast_addr="255.255.255.255"):
    """Broadcast for readers on the local subnet. Returns a list of dicts.

    Default broadcast_addr is the global broadcast, but with more than one
    active network path (e.g. Wi-Fi + a dedicated USB-Ethernet adapter),
    Windows may route a global broadcast out the wrong interface. Passing
    the subnet-directed broadcast instead (e.g. "192.168.1.255" for a
    192.168.1.0/24 network) is more reliable in that situation, since it
    matches a specific route rather than requiring the OS to guess.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind((bind_ip, 0))
    sock.settimeout(timeout)

    sock.sendto(b"X", (broadcast_addr, CONFIG_PORT))

    found = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            break
        text = data.decode("ascii", errors="replace")
        if text.startswith("A"):
            found.append({"ip": addr[0], "raw": text})
    sock.close()
    return found


def query_all(ip):
    """Query every known field from the reader at `ip`. Returns a dict."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(RECV_TIMEOUT)
    addr = (ip, CONFIG_PORT)

    ack = send_recv(sock, addr, "L")
    if ack is None:
        sock.close()
        raise RuntimeError(f"No response from {ip} on the config port ({CONFIG_PORT}). "
                            f"Device may be offline, wrong IP, or not on this subnet.")

    values = {}

    mac_cmd, mac_idx = MAC_GET
    reply = send_recv(sock, addr, f"{mac_cmd}|{mac_idx}")
    if reply is not None:
        pipe = reply.find("|")
        if pipe != -1 and reply[pipe + 1:] == str(mac_idx):
            values["mac"] = reply[:pipe]

    for field, (get_cmd, get_idx, _set_cmd, _set_idx) in FIELDS.items():
        reply = send_recv(sock, addr, f"{get_cmd}|{get_idx}")
        if reply is None:
            values[field] = None
            continue
        pipe = reply.find("|")
        if pipe != -1 and reply[pipe + 1:] == str(get_idx):
            values[field] = reply[:pipe]
        else:
            values[field] = None
        time.sleep(0.01)

    sock.close()
    return values


def write_all(ip, values):
    """Send every field in `values` back to the reader (S commands), then E."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(RECV_TIMEOUT)
    addr = (ip, CONFIG_PORT)

    ack = send_recv(sock, addr, "L")
    if ack is None:
        sock.close()
        raise RuntimeError(f"No response from {ip} when starting write session.")

    for field, (_get_cmd, _get_idx, set_cmd, set_idx) in FIELDS.items():
        value = values.get(field, "")
        if value is None:
            value = ""
        sock.sendto(f"{set_cmd}{value}|{set_idx}".encode("ascii"), addr)
        time.sleep(0.01)

    sock.sendto(b"E", addr)
    time.sleep(0.5)  # demo waits 500ms here, presumably for apply/reboot
    sock.close()


def cmd_discover(args):
    print(f"Broadcasting discovery request to {args.broadcast} (timeout {args.timeout}s)...")
    results = discover(timeout=args.timeout, broadcast_addr=args.broadcast)
    if not results:
        print("No readers responded.")
        return
    for r in results:
        print(f"  {r['ip']}  raw: {r['raw']!r}")


def cmd_query(args):
    print(f"Querying {args.ip} ...")
    values = query_all(args.ip)
    for field, value in values.items():
        print(f"  {field:10s} = {value}")


def cmd_set_ip(args):
    print(f"Reading current configuration from {args.ip} ...")
    current = query_all(args.ip)
    print("Current values:")
    for field, value in current.items():
        print(f"  {field:10s} = {value}")

    new_values = dict(current)
    new_values.pop("mac", None)  # never written back
    new_values["IP"] = args.new_ip
    if args.netmask:
        new_values["nm"] = args.netmask
    if args.gateway:
        new_values["gi"] = args.gateway

    print("\nProposed changes:")
    print(f"  IP: {current.get('IP')} -> {new_values['IP']}")
    if args.netmask:
        print(f"  nm (netmask): {current.get('nm')} -> {new_values['nm']}")
    if args.gateway:
        print(f"  gi (gateway): {current.get('gi')} -> {new_values['gi']}")
    print("  All other fields will be re-sent unchanged.")

    if not args.apply:
        print("\nDry run only -- no changes sent. Re-run with --apply to write "
              "these settings to the reader.")
        return

    print(f"\nAbout to overwrite network configuration on {args.ip}.")
    print("There is no documented reset procedure for this reader if this goes wrong.")
    confirm = input("Type APPLY (all caps) to proceed: ")
    if confirm != "APPLY":
        print("Not confirmed -- aborting, no changes sent.")
        return

    write_all(args.ip, new_values)
    print(f"Sent. The reader may now be reachable at {args.new_ip} instead of {args.ip} "
          f"-- try `query {args.new_ip}` to confirm.")


def main():
    parser = argparse.ArgumentParser(description="RRU7182EVB-IP-NL network config tool")
    sub = parser.add_subparsers(dest="command", required=True)

    p_discover = sub.add_parser("discover", help="Broadcast for readers on this subnet")
    p_discover.add_argument("--timeout", type=float, default=2.0)
    p_discover.add_argument("--broadcast", default="255.255.255.255",
                             help="Broadcast address to use. If you have multiple "
                                  "active network adapters, try the subnet-directed "
                                  "broadcast instead (e.g. 192.168.1.255) for more "
                                  "reliable interface selection.")
    p_discover.set_defaults(func=cmd_discover)

    p_query = sub.add_parser("query", help="Read all current settings from a known IP")
    p_query.add_argument("ip")
    p_query.set_defaults(func=cmd_query)

    p_set_ip = sub.add_parser("set-ip", help="Change the reader's IP (and optionally "
                                              "netmask/gateway); dry-run unless --apply")
    p_set_ip.add_argument("ip", help="Reader's CURRENT IP")
    p_set_ip.add_argument("--new-ip", required=True)
    p_set_ip.add_argument("--netmask")
    p_set_ip.add_argument("--gateway")
    p_set_ip.add_argument("--apply", action="store_true",
                           help="Actually send the change (default is dry-run)")
    p_set_ip.set_defaults(func=cmd_set_ip)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
