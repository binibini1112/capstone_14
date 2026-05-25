#!/usr/bin/env python3
import argparse
import socket
import time


def request(host, port, cmd, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto((cmd.rstrip() + "\n").encode("ascii"), (host, port))
        data, _ = sock.recvfrom(1024)
        return data.decode("ascii", errors="replace").strip()
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.3.1")
    parser.add_argument("--port", type=int, default=5016)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--torque-off", action="store_true")
    parser.add_argument("--torque-on", action="store_true")
    parser.add_argument("--read", action="store_true")
    parser.add_argument("--set-current", action="store_true")
    parser.add_argument("--set", nargs=2, metavar=("PAN", "TILT"), type=int)
    parser.add_argument("--center", action="store_true")
    parser.add_argument("--pause", type=float, default=0.0)
    args = parser.parse_args()

    commands = []
    if args.torque_off:
        commands.append("TORQUE 0")
    if args.pause > 0:
        commands.append(None)
    if args.read:
        commands.append("READPOS")
    if args.set_current:
        commands.append("SETCENTER")
    if args.set:
        commands.append(f"SETCENTER {args.set[0]} {args.set[1]}")
    if args.torque_on:
        commands.append("TORQUE 1")
    if args.center:
        commands.append("CENTER")

    if not commands:
        commands = ["READPOS"]

    for cmd in commands:
        if cmd is None:
            time.sleep(args.pause)
            continue
        print(f"{cmd} => {request(args.host, args.port, cmd, args.timeout)}")


if __name__ == "__main__":
    main()
