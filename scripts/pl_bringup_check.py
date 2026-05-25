#!/usr/bin/env python3
"""Run quick ultra_yubin UDP/PL/USB bridge checks."""
import argparse
import datetime as dt
import os
import socket
import sys
import time


def udp_cmd(host, port, payload, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    started = time.perf_counter()
    try:
        sock.sendto((payload.rstrip() + "\n").encode("ascii"), (host, port))
        data, _ = sock.recvfrom(1024)
        return True, data.decode("ascii", errors="replace").strip(), (time.perf_counter() - started) * 1000
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}", (time.perf_counter() - started) * 1000
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=os.getenv("ULTRA96_HOST", "192.168.3.1"))
    parser.add_argument("--port", type=int, default=int(os.getenv("ULTRA_YUBIN_PORT", "5016")))
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--log-dir", default="benchmark_logs")
    parser.add_argument("--no-save", action="store_true")
    args = parser.parse_args()

    commands = [
        ("PING", "PING"),
        ("PLPING_BEFORE", "PLPING"),
        ("DIRECT_G", "G 2048 2772"),
        ("PLPING_AFTER_G", "PLPING"),
        ("TRACK_T", "T 650 340 100 80 1280 720 870 1"),
        ("PLPING_AFTER_T", "PLPING"),
        ("AUDIO_A", "A 30 850 1"),
        ("PLPING_AFTER_A", "PLPING"),
    ]

    lines = [
        "=== ultra_yubin Bring-up Check ===",
        f"time: {dt.datetime.now().isoformat(timespec='seconds')}",
        f"target: {args.host}:{args.port}",
    ]
    print("\n".join(lines))

    failures = 0
    for label, cmd in commands:
        ok, reply, elapsed = udp_cmd(args.host, args.port, cmd, args.timeout)
        if not ok:
            failures += 1
        line = f"[{label}] {'OK' if ok else 'FAIL'} {elapsed:.3f} ms cmd={cmd!r} reply={reply}"
        lines.append(line)
        print(line)

    summary = f"summary: failures={failures}"
    lines.append(summary)
    print(summary)

    if not args.no_save:
        os.makedirs(args.log_dir, exist_ok=True)
        path = os.path.join(args.log_dir, f"ultra_yubin_bringup_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        with open(path, "w", encoding="utf-8") as fp:
            fp.write("\n".join(lines) + "\n")
        print(f"log: {path}")

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
