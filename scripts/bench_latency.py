#!/usr/bin/env python3
"""Benchmark ultra_yubin UDP -> PL compute -> PS USB bridge round-trip latency."""
import argparse
import statistics
import socket
import time


def send(host, port, payload, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    start = time.perf_counter()
    try:
        sock.sendto((payload + "\n").encode("ascii"), (host, port))
        data, _ = sock.recvfrom(1024)
        return True, (time.perf_counter() - start) * 1000.0, data.decode(errors="replace").strip()
    except Exception as exc:
        return False, (time.perf_counter() - start) * 1000.0, f"{type(exc).__name__}: {exc}"
    finally:
        sock.close()


def summarize(name, values, failures):
    print(f"\n[{name}]")
    print(f"  success: {len(values)}")
    print(f"  failure: {failures}")
    if values:
        print(f"  avg_ms: {statistics.mean(values):.3f}")
        print(f"  min_ms: {min(values):.3f}")
        print(f"  max_ms: {max(values):.3f}")
        print(f"  p95_ms: {statistics.quantiles(values, n=20)[18] if len(values) >= 20 else max(values):.3f}")
        print(f"  jitter_stdev_ms: {statistics.pstdev(values):.3f}")


def run_case(host, port, label, command, samples, timeout):
    values = []
    failures = 0
    for i in range(samples):
        ok, elapsed, reply = send(host, port, command, timeout)
        if i < 3:
            print(f"{label} sample reply: {reply}")
        if ok:
            values.append(elapsed)
        else:
            failures += 1
    return values, failures


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.3.1")
    parser.add_argument("--port", type=int, default=5016)
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    print("=== ultra_yubin Latency Benchmark ===")
    print(f"target: {args.host}:{args.port}")
    print(f"samples: {args.samples}")

    cases = [
        ("PING UDP round-trip", "PING"),
        ("PLPING AXI read round-trip", "PLPING"),
        ("T bbox PL compute plus PS USB send ACK", "T 650 340 100 80 1280 720 870 1"),
        ("A audio PS compute plus PS USB send ACK", "A 30 850 1"),
        ("G direct PS USB send ACK", "G 2048 2772"),
    ]

    results = []
    for label, cmd in cases:
        values, failures = run_case(args.host, args.port, label, cmd, args.samples, args.timeout)
        results.append((label, values, failures))
        time.sleep(0.1)

    print("\n=== Summary ===")
    for label, values, failures in results:
        summarize(label, values, failures)

    print("\nNotes:")
    print("  T includes UDP, PL AXI writes/reads, PL computation, and PS USB serial write calls.")
    print("  A includes UDP, PS angle-to-pan computation, and PS USB serial write calls.")
    print("  In dry-run mode, USB writes are skipped, so results measure control path overhead only.")


if __name__ == "__main__":
    main()
