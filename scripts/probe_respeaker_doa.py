#!/usr/bin/env python3
import argparse
import os
import struct
import subprocess
import time

import numpy as np


def gcc_phat_debug(sig, refsig, fs=16000, max_tau=None, interp=16):
    n = sig.shape[0] + refsig.shape[0]
    sig_fft = np.fft.rfft(sig, n=n)
    ref_fft = np.fft.rfft(refsig, n=n)
    cross = sig_fft * np.conj(ref_fft)
    corr = np.fft.irfft(cross / (np.abs(cross) + 1e-15), n=interp * n)
    max_shift = int(interp * n / 2)
    if max_tau is not None:
        max_shift = min(int(interp * fs * max_tau), max_shift)
    window = np.concatenate((corr[-max_shift:], corr[: max_shift + 1]))
    abs_window = np.abs(window)
    peak_idx = int(np.argmax(abs_window))
    shift = peak_idx - max_shift
    peak = float(abs_window[peak_idx])
    median = float(np.median(abs_window) + 1e-12)
    return {
        "tau": shift / float(interp * fs),
        "shift": shift / float(interp),
        "peak": peak,
        "ratio": peak / median,
    }


def estimate_simple_4ch(audio_4ch, fs=16000, mic_distance=0.065):
    c = 343.0
    max_tau = mic_distance / c
    tau_x = gcc_phat_debug(audio_4ch[:, 0], audio_4ch[:, 2], fs, max_tau)["tau"]
    tau_y = gcc_phat_debug(audio_4ch[:, 1], audio_4ch[:, 3], fs, max_tau)["tau"]
    x = np.clip(tau_x * c / mic_distance, -1.0, 1.0)
    y = np.clip(tau_y * c / mic_distance, -1.0, 1.0)
    return float(np.degrees(np.arctan2(y, x)) % 360.0)


def read_usb_doa(offset=0):
    try:
        import usb.core

        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if dev is None:
            return None, "not_found"
        data = dev.ctrl_transfer(0xC0, 0, 0xC0, 21, 8, 100000)
        val = struct.unpack(b"ii", bytes(data))[0]
        if 0 <= val <= 359:
            return int((val + offset) % 360), "ok"
        return None, f"bad_value:{val}"
    except Exception as exc:
        return None, f"{type(exc).__name__}:{exc}"


def capture_arecord(device, rate, channels, seconds):
    cmd = [
        "arecord",
        "-q",
        "-D",
        device,
        "-f",
        "S16_LE",
        "-r",
        str(rate),
        "-c",
        str(channels),
        "-t",
        "raw",
        "-d",
        str(seconds),
    ]
    raw = subprocess.check_output(cmd)
    data = np.frombuffer(raw, dtype=np.int16)
    data = data[: (data.size // channels) * channels]
    return data.reshape(-1, channels).astype(np.float32) / 32768.0


def main():
    parser = argparse.ArgumentParser(description="Probe ReSpeaker channel map and GCC-PHAT DOA")
    parser.add_argument("--device", default="plughw:1,0")
    parser.add_argument("--rate", type=int, default=16000)
    parser.add_argument("--channels", type=int, default=6)
    parser.add_argument("--seconds", type=int, default=2)
    parser.add_argument("--mic-channels", default=os.getenv("TELLO_AUDIO_MIC_CHANNELS", "1,2,3,4"))
    parser.add_argument("--mic-distance", type=float, default=float(os.getenv("TELLO_AUDIO_MIC_DISTANCE", "0.045")))
    args = parser.parse_args()

    mic_channels = tuple(int(x.strip()) for x in args.mic_channels.split(",") if x.strip())
    if len(mic_channels) != 4:
        raise SystemExit("--mic-channels must contain exactly 4 indexes, e.g. 0,1,2,3")

    usb_doa, usb_status = read_usb_doa()
    print(f"usb_doa={usb_doa} status={usb_status}")
    print(
        f"capture device={args.device} rate={args.rate} channels={args.channels} "
        f"seconds={args.seconds} mic_channels={mic_channels}"
    )
    audio = capture_arecord(args.device, args.rate, args.channels, args.seconds)
    print(f"frames={audio.shape[0]}")

    rms = np.sqrt(np.mean(audio * audio, axis=0))
    peak = np.max(np.abs(audio), axis=0)
    for ch in range(audio.shape[1]):
        print(f"ch{ch}: rms={rms[ch]:.6f} peak={peak[ch]:.6f}")

    mic = audio[:, mic_channels]
    simple = estimate_simple_4ch(mic, fs=args.rate, mic_distance=args.mic_distance)
    print(f"simple_gcc_doa={simple:.1f}")
    print("pair_gcc:")
    c = 343.0
    max_tau = args.mic_distance / c
    for i in range(4):
        for j in range(i + 1, 4):
            info = gcc_phat_debug(mic[:, i], mic[:, j], fs=args.rate, max_tau=max_tau)
            print(
                f"  mic{i}-mic{j} raw_ch{mic_channels[i]}-{mic_channels[j]} "
                f"tau_us={info['tau'] * 1e6:+.1f} shift_samples={info['shift']:+.2f} "
                f"peak_ratio={info['ratio']:.2f}"
            )

    time.sleep(0.1)
    usb_doa2, usb_status2 = read_usb_doa()
    print(f"usb_doa_after={usb_doa2} status={usb_status2}")


if __name__ == "__main__":
    main()
