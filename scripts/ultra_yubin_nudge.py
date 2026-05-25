#!/usr/bin/env python3
import argparse
import socket
import sys
import termios
import tty


def request(host, port, cmd, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto((cmd.rstrip() + "\n").encode("ascii"), (host, port))
        data, _ = sock.recvfrom(2048)
        return data.decode("ascii", errors="replace").strip()
    finally:
        sock.close()


def parse_pos(reply):
    fields = {}
    for item in reply.split(","):
        if "=" in item:
            key, value = item.split("=", 1)
            fields[key.strip()] = value.strip()
    return int(fields.get("pan", "2048")), int(fields.get("tilt", "2772"))


def clamp_goal(value):
    return max(0, min(4095, int(value)))


def read_pos(args):
    reply = request(args.host, args.port, "READPOS", args.timeout)
    pan, tilt = parse_pos(reply)
    return pan, tilt, reply


def send_goal(args, pan, tilt):
    pan = clamp_goal(pan)
    tilt = clamp_goal(tilt)
    reply = request(args.host, args.port, f"G {pan} {tilt}", args.timeout)
    return pan, tilt, reply


def run_once(args):
    pan, tilt, read_reply = read_pos(args)
    print(f"READPOS => {read_reply}")
    pan, tilt, goal_reply = send_goal(args, pan + args.pan_delta, tilt + args.tilt_delta)
    print(f"G {pan} {tilt} => {goal_reply}")


def run_interactive(args):
    pan, tilt, read_reply = read_pos(args)
    print(f"READPOS => {read_reply}")
    print("keys: h/l pan -/+ | j/k tilt +/- | H/L/J/K big step | c center | r read | q quit")
    print(f"step={args.step} big={args.big_step}")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch in ("q", "\x03", "\x04"):
                print("\nquit")
                return
            if ch == "r":
                pan, tilt, reply = read_pos(args)
                print(f"\nREADPOS => {reply}")
                continue
            if ch == "c":
                reply = request(args.host, args.port, "CENTER", args.timeout)
                pan, tilt = parse_pos(reply)
                print(f"\nCENTER => {reply}")
                continue

            dp = 0
            dt = 0
            if ch == "h":
                dp = -args.step
            elif ch == "l":
                dp = args.step
            elif ch == "j":
                dt = args.step
            elif ch == "k":
                dt = -args.step
            elif ch == "H":
                dp = -args.big_step
            elif ch == "L":
                dp = args.big_step
            elif ch == "J":
                dt = args.big_step
            elif ch == "K":
                dt = -args.big_step
            else:
                continue

            pan, tilt, reply = send_goal(args, pan + dp, tilt + dt)
            print(f"\rpan={pan:4d} tilt={tilt:4d} reply={reply[:90]}   ", end="", flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.3.1")
    parser.add_argument("--port", type=int, default=5016)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--step", type=int, default=12)
    parser.add_argument("--big-step", type=int, default=48)
    parser.add_argument("--pan-delta", type=int, default=0)
    parser.add_argument("--tilt-delta", type=int, default=0)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    if args.once or args.pan_delta or args.tilt_delta:
        run_once(args)
    else:
        run_interactive(args)


if __name__ == "__main__":
    main()
