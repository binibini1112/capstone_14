#!/usr/bin/env python3
import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import resample_poly

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from jetson.src.audio_fallback import _find_respeaker_alsa_device, _load_tflite_interpreter


DEFAULT_MODEL_DIR = ROOT / "models" / "audio_direction" / "colab_direction_model_1s"


def read_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def read_audio(path, target_sr):
    sr, data = wavfile.read(str(path))
    data = np.asarray(data)
    if data.ndim == 1:
        data = data[:, None]
    if np.issubdtype(data.dtype, np.integer):
        audio = data.astype(np.float32) / float(np.iinfo(data.dtype).max)
    else:
        audio = data.astype(np.float32)
    if sr != target_sr:
        gcd = math.gcd(int(sr), int(target_sr))
        audio = resample_poly(audio, int(target_sr) // gcd, int(sr) // gcd, axis=0).astype(np.float32)
        sr = target_sr
    return np.clip(audio, -1.0, 1.0), sr


def capture_arecord(device, sample_rate, channels, seconds):
    if not device or device == "auto":
        device = _find_respeaker_alsa_device()
    seconds = float(seconds)
    cmd = [
        "arecord",
        "-q",
        "-D",
        device,
        "-f",
        "S16_LE",
        "-r",
        str(sample_rate),
        "-c",
        str(channels),
        "-t",
        "raw",
    ]
    if seconds.is_integer():
        cmd.extend(["-d", str(int(seconds))])
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    else:
        proc = subprocess.run(
            ["timeout", "--signal=INT", f"{seconds:.3f}s", *cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if proc.returncode == 124:
            proc.returncode = 0
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.decode("utf-8", errors="replace"))
    samples = np.frombuffer(proc.stdout, dtype="<i2")
    frames = samples.size // channels
    if frames <= 0:
        raise RuntimeError("arecord returned no samples")
    audio = samples[: frames * channels].reshape(frames, channels).astype(np.float32) / 32768.0
    return np.clip(audio, -1.0, 1.0), sample_rate, device


def select_4ch(audio, channels_text):
    selected = tuple(int(part.strip()) for part in str(channels_text).split(",") if part.strip())
    if len(selected) != 4:
        raise ValueError("--mic-channels must contain exactly 4 channel indexes")
    if audio.shape[1] <= max(selected):
        raise ValueError(f"audio has {audio.shape[1]} channels, cannot select {selected}")
    return audio[:, selected]


def mel_filterbank(sr, n_fft, n_mels, fmin, fmax):
    try:
        import librosa

        return librosa.filters.mel(sr=sr, n_fft=n_fft, n_mels=n_mels, fmin=fmin, fmax=fmax).astype(np.float32)
    except Exception:
        return manual_mel_filterbank(sr, n_fft, n_mels, fmin, fmax)


def hz_to_mel(freq):
    return 2595.0 * np.log10(1.0 + np.asarray(freq) / 700.0)


def mel_to_hz(mel):
    return 700.0 * (10.0 ** (np.asarray(mel) / 2595.0) - 1.0)


def manual_mel_filterbank(sr, n_fft, n_mels, fmin, fmax):
    fftfreqs = np.linspace(0.0, sr / 2.0, n_fft // 2 + 1)
    mel_points = np.linspace(hz_to_mel(fmin), hz_to_mel(fmax), n_mels + 2)
    hz_points = mel_to_hz(mel_points)
    fb = np.zeros((n_mels, fftfreqs.size), dtype=np.float32)
    for i in range(n_mels):
        left, center, right = hz_points[i : i + 3]
        up = (fftfreqs - left) / max(center - left, 1e-9)
        down = (right - fftfreqs) / max(right - center, 1e-9)
        fb[i] = np.maximum(0.0, np.minimum(up, down))
    return fb


def power_to_db_common_ref(mel, ref):
    return 10.0 * np.log10(np.maximum(mel, 1e-10)) - 10.0 * np.log10(max(float(ref), 1e-10))


def make_feature(audio_4ch, config, mean, std, mel_basis):
    sr = int(config["sample_rate"])
    segment_sec = float(config.get("segment_sec", config.get("window_sec", 1.0)))
    segment_samples = int(round(segment_sec * sr))
    n_fft = int(config["n_fft"])
    hop = int(config["hop_length"])
    if audio_4ch.shape[0] < segment_samples:
        audio_4ch = np.pad(audio_4ch, ((0, segment_samples - audio_4ch.shape[0]), (0, 0)))
    elif audio_4ch.shape[0] > segment_samples:
        audio_4ch = audio_4ch[-segment_samples:]

    window = np.hanning(n_fft + 1)[:-1].astype(np.float32)
    mel_list = []
    power_max = 1e-10
    for ch in range(4):
        y = audio_4ch[:, ch].astype(np.float32)
        y = np.pad(y, (n_fft // 2, n_fft // 2), mode="constant")
        frames = []
        for start in range(0, len(y) - n_fft + 1, hop):
            frame = y[start : start + n_fft] * window
            frames.append(np.abs(np.fft.rfft(frame, n=n_fft)) ** 2)
        power = np.asarray(frames, dtype=np.float32).T
        mel = np.maximum(mel_basis @ power, 1e-10)
        mel_list.append(mel)
        power_max = max(power_max, float(np.max(mel)))

    feat = np.stack([power_to_db_common_ref(mel, power_max) for mel in mel_list], axis=-1).astype(np.float32)
    feat = (feat - mean.squeeze(axis=0)) / np.maximum(std.squeeze(axis=0), 1e-6)
    return feat[np.newaxis, ...].astype(np.float32)


def _mapping_get(mapping, key, default=None):
    if mapping is None:
        return default
    return mapping.get(str(key), mapping.get(key, default))


def predict(interpreter, input_info, output_info, feat, class_to_angle, class_to_name=None):
    interpreter.set_tensor(input_info["index"], feat)
    interpreter.invoke()
    probs = np.ravel(interpreter.get_tensor(output_info["index"])).astype(np.float32)
    pred_class = int(np.argmax(probs))
    angle_value = _mapping_get(class_to_angle, pred_class)
    angle = None if angle_value is None else int(angle_value)
    name = _mapping_get(class_to_name, pred_class, f"class_{pred_class}")
    return pred_class, angle, name, probs


def load_model_bundle(model_dir):
    model_dir = Path(model_dir)
    config = read_json(model_dir / "feature_config.json")
    label_json = read_json(model_dir / "label_mapping.json")
    labels = label_json.get("class_to_angle", {})
    names = label_json.get("class_to_name", {})
    if not labels:
        labels = {}
        for key, value in label_json.items():
            if isinstance(value, dict):
                labels[str(key)] = value.get("angle_deg")
                names[str(key)] = value.get("name", f"{value.get('angle_deg')}deg")
            else:
                labels[str(key)] = value
                names[str(key)] = f"{value}deg"
    mean_path = model_dir / "feature_mean.npy"
    std_path = model_dir / "feature_std.npy"
    if mean_path.exists() and std_path.exists():
        mean = np.load(mean_path).astype(np.float32)
        std = np.load(std_path).astype(np.float32)
    else:
        norm = config.get("normalization", {})
        mean = np.asarray([[[float(norm.get("mean", config.get("mean", 0.0)))]]], dtype=np.float32)
        std = np.asarray([[[float(norm.get("std", config.get("std", 1.0)))]]], dtype=np.float32)
    candidates = [
        model_dir / "audio_angle_cnn_final.tflite",
        model_dir / "drone_direction_7class_1s.tflite",
        model_dir / "direction_6sector_cnn_1s_float32.tflite",
        model_dir / "direction_6sector_cnn_1s_compat.tflite",
        model_dir / "direction_6sector_cnn_1s.tflite",
    ]
    model_path = next((path for path in candidates if path.exists()), candidates[-1])
    interpreter = _load_tflite_interpreter(str(model_path))
    input_info = interpreter.get_input_details()[0]
    output_info = interpreter.get_output_details()[0]
    mel_basis = mel_filterbank(
        int(config["sample_rate"]),
        int(config["n_fft"]),
        int(config["n_mels"]),
        float(config["fmin"]),
        float(config["fmax"]),
    )
    return config, labels, names, mean, std, interpreter, input_info, output_info, mel_basis


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-dir", default=str(DEFAULT_MODEL_DIR))
    parser.add_argument("--wav", default="")
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--channels", type=int, default=6)
    parser.add_argument("--mic-channels", default="1,2,3,4")
    parser.add_argument("--seconds", type=float, default=1.0)
    parser.add_argument("--repeat", type=int, default=0, help="0 means run forever in live mode")
    args = parser.parse_args()

    config, labels, names, mean, std, interpreter, input_info, output_info, mel_basis = load_model_bundle(args.model_dir)
    sr = int(config["sample_rate"])
    print(f"[cnn6-test] model_dir={args.model_dir}")
    print(f"[cnn6-test] input_shape={input_info['shape']} labels={labels} names={names}")

    if args.wav:
        audio, file_sr = read_audio(args.wav, sr)
        mic = select_4ch(audio, "0,1,2,3" if audio.shape[1] == 4 else args.mic_channels)
        feat = make_feature(mic, config, mean, std, mel_basis)
        pred_class, angle, name, probs = predict(interpreter, input_info, output_info, feat, labels, names)
        prob_text = " ".join(
            f"{_mapping_get(names, i, _mapping_get(labels, i, i))}:{float(probs[i]):.3f}" for i in range(len(probs))
        )
        angle_text = "noise" if angle is None else f"{angle:03d}deg"
        print(f"[cnn6-test] wav={args.wav} pred_class={pred_class} pred={angle_text} name={name} probs={prob_text}")
        return 0

    if not args.live:
        raise SystemExit("Use --wav FILE or --live")

    count = 0
    while True:
        audio, _, device = capture_arecord(args.device, sr, args.channels, args.seconds)
        mic = select_4ch(audio, args.mic_channels)
        feat = make_feature(mic, config, mean, std, mel_basis)
        pred_class, angle, name, probs = predict(interpreter, input_info, output_info, feat, labels, names)
        rms = float(np.sqrt(np.mean(mic * mic)))
        prob_text = " ".join(
            f"{_mapping_get(names, i, _mapping_get(labels, i, i))}:{float(probs[i]):.2f}" for i in range(len(probs))
        )
        angle_text = "noise" if angle is None else f"{angle:03d}deg"
        print(
            f"[cnn6-test] device={device} rms={rms:.4f} pred={angle_text} name={name} class={pred_class} {prob_text}",
            flush=True,
        )
        count += 1
        if args.repeat and count >= args.repeat:
            break
        time.sleep(0.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
