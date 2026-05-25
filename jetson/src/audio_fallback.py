import json
import math
import os
import queue
import re
import struct
import subprocess
import sys
import threading
import time
from collections import deque

import numpy as np


_RESPEAKER_KEYS = ("respeaker", "arrayuac10", "seeed")
_BLOCKED_AUDIO_KEYS = ("fhd60f",)


def handle_drone_detected(doa_angle, tello_prob):
    # TODO: Dynamixel pan/tilt motor control 연결
    print(f"[DRONE DETECTED] doa={doa_angle}, prob={tello_prob}", flush=True)


def _parse_arecord_cards(text):
    cards = {}
    for line in text.splitlines():
        card_match = re.search(r"(?:card|카드)\s+(\d+)", line, re.IGNORECASE)
        if card_match is None:
            continue
        cards[card_match.group(1)] = line
    return cards


def _is_respeaker_text(text):
    lowered = str(text).lower()
    return any(key in lowered for key in _RESPEAKER_KEYS)


def _is_blocked_audio_text(text):
    lowered = str(text).lower()
    return any(key in lowered for key in _BLOCKED_AUDIO_KEYS)


def _read_arecord_list_retried():
    tries = int(os.getenv("TELLO_AUDIO_ARECORD_LIST_RETRIES", "6"))
    delay = float(os.getenv("TELLO_AUDIO_ARECORD_LIST_RETRY_SEC", "0.25"))
    last = ""
    for _ in range(max(1, tries)):
        try:
            last = subprocess.check_output(
                ["arecord", "-l"],
                text=True,
                stderr=subprocess.STDOUT,
                timeout=1.5,
            )
        except Exception as exc:
            last = str(exc)
        if _is_respeaker_text(last):
            return last
        time.sleep(max(0.0, delay))
    return last


def _validate_respeaker_alsa_device(device, arecord_text=None):
    requested = str(device or "").strip()
    lowered = requested.lower()
    if not requested or lowered in ("auto", "default"):
        return requested
    if _is_blocked_audio_text(requested):
        raise RuntimeError(f"Refusing camera audio device for ReSpeaker fallback: {requested}")
    if any(key in lowered for key in ("default", "pulse", "sysdefault:card=fhd60f", "front:card=fhd60f")):
        raise RuntimeError(f"Refusing ambiguous/non-ReSpeaker audio device: {requested}")
    if _is_respeaker_text(requested):
        return requested

    numeric_match = re.search(r"(?:^|:)(?:CARD=)?(\d+)(?:,|$)", requested, re.IGNORECASE)
    if numeric_match is not None:
        if arecord_text is None:
            arecord_text = _read_arecord_list_retried()
        card_line = _parse_arecord_cards(arecord_text).get(numeric_match.group(1), "")
        if not card_line:
            arecord_text = _read_arecord_list_retried()
            card_line = _parse_arecord_cards(arecord_text).get(numeric_match.group(1), "")
        if _is_respeaker_text(card_line) and not _is_blocked_audio_text(card_line):
            return requested
        if not card_line and numeric_match.group(1) == "1" and lowered in ("plughw:1,0", "hw:1,0"):
            return requested
        raise RuntimeError(
            f"Refusing non-ReSpeaker numeric audio device {requested}; card line: {card_line or 'not found'}"
        )

    raise RuntimeError(f"Refusing non-ReSpeaker audio device for fallback: {requested}")


def _find_respeaker_alsa_device():
    out = _read_arecord_list_retried()
    for line in out.splitlines():
        text = line.lower()
        if _is_blocked_audio_text(text):
            continue
        if not _is_respeaker_text(text):
            continue
        card_match = re.search(r"(?:card|카드)\s+(\d+)", line, re.IGNORECASE)
        dev_match = re.search(r"(?:device|장치)\s+(\d+)", line, re.IGNORECASE)
        if card_match is not None and dev_match is not None:
            return f"plughw:{card_match.group(1)},{dev_match.group(1)}"
        return "plughw:CARD=ArrayUAC10,DEV=0"
    raise RuntimeError("ReSpeaker ALSA capture card not found in `arecord -l`; refusing FHD60F camera audio")


class ReSpeakerDOA:
    def __init__(self, offset=0):
        import usb.core

        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if dev is None:
            raise RuntimeError("ReSpeaker Mic Array not found")
        self._dev = dev
        self._offset = int(offset)
        self._angle = 0
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        prev = 0
        while self._running:
            try:
                data = self._dev.ctrl_transfer(0xC0, 0, 0xC0, 21, 8, 100000)
                val = struct.unpack(b"ii", bytes(data))[0]
                if 0 <= val <= 359:
                    prev = int((val + self._offset) % 360)
            except Exception:
                pass
            with self._lock:
                self._angle = prev
            time.sleep(0.02)

    def read(self):
        with self._lock:
            return int(self._angle)

    def stop(self):
        self._running = False


def _load_tflite_interpreter(model_path):
    errors = []
    try:
        from tflite_runtime.interpreter import Interpreter
        interpreter = Interpreter(model_path=model_path)
        interpreter.allocate_tensors()
        return interpreter
    except Exception as exc:
        errors.append(f"tflite_runtime: {exc}")

    try:
        from tensorflow.lite.python.interpreter import Interpreter
        interpreter = Interpreter(model_path=model_path)
        interpreter.allocate_tensors()
        print("[audio] loaded TFLite model with tensorflow.lite fallback", flush=True)
        return interpreter
    except Exception as exc:
        errors.append(f"tensorflow.lite: {exc}")

    raise RuntimeError(
        "Failed to load TFLite model. " + " | ".join(errors)
    )


def _hz_to_mel(freqs):
    freqs = np.asanyarray(freqs)
    f_sp = 200.0 / 3
    mels = freqs / f_sp
    min_log_hz = 1000.0
    min_log_mel = min_log_hz / f_sp
    logstep = np.log(6.4) / 27.0
    log_t = freqs >= min_log_hz
    mels = np.array(mels, dtype=np.float64)
    mels[log_t] = min_log_mel + np.log(freqs[log_t] / min_log_hz) / logstep
    return mels


def _mel_to_hz(mels):
    mels = np.asanyarray(mels)
    f_sp = 200.0 / 3
    freqs = f_sp * mels
    min_log_hz = 1000.0
    min_log_mel = min_log_hz / f_sp
    logstep = np.log(6.4) / 27.0
    log_t = mels >= min_log_mel
    freqs = np.array(freqs, dtype=np.float64)
    freqs[log_t] = min_log_hz * np.exp(logstep * (mels[log_t] - min_log_mel))
    return freqs


def _mel_filterbank(sr, n_fft, n_mels, fmin, fmax):
    fftfreqs = np.linspace(0, sr / 2, 1 + n_fft // 2)
    mel_f = _mel_to_hz(np.linspace(_hz_to_mel(fmin), _hz_to_mel(fmax), n_mels + 2))
    fdiff = np.diff(mel_f)
    ramps = mel_f[:, np.newaxis] - fftfreqs[np.newaxis, :]
    lower = -ramps[:-2] / fdiff[:-1, np.newaxis]
    upper = ramps[2:] / fdiff[1:, np.newaxis]
    weights = np.maximum(0, np.minimum(lower, upper))
    weights *= (2.0 / (mel_f[2 : n_mels + 2] - mel_f[:n_mels]))[:, np.newaxis]
    return weights.astype(np.float32)


def _gcc_phat_info(sig, refsig, fs=16000, max_tau=None, interp=16):
    n = sig.shape[0] + refsig.shape[0]
    sig_fft = np.fft.rfft(sig, n=n)
    ref_fft = np.fft.rfft(refsig, n=n)
    cross = sig_fft * np.conj(ref_fft)
    corr = np.fft.irfft(cross / (np.abs(cross) + 1e-15), n=interp * n)

    max_shift = int(interp * n / 2)
    if max_tau is not None:
        max_shift = min(int(interp * fs * max_tau), max_shift)

    corr = np.concatenate((corr[-max_shift:], corr[: max_shift + 1]))
    abs_corr = np.abs(corr)
    peak_idx = int(np.argmax(abs_corr))
    shift = int(peak_idx - max_shift)
    peak = float(abs_corr[peak_idx])
    median = float(np.median(abs_corr) + 1e-12)
    return {
        "tau": shift / float(interp * fs),
        "shift_samples": shift / float(interp),
        "peak": peak,
        "peak_ratio": peak / median,
    }


def _gcc_phat(sig, refsig, fs=16000, max_tau=None, interp=16):
    return _gcc_phat_info(sig, refsig, fs=fs, max_tau=max_tau, interp=interp)["tau"]


def estimate_direction_gcc_phat(
    audio_4ch,
    fs=16000,
    mic_distance=0.045,
    sound_speed=340.0,
    interp=16,
    min_peak_ratio=1.15,
    return_debug=False,
):
    """Estimate azimuth from 4 microphone channels using all six GCC-PHAT pairs.

    The selected 4 channels are treated as a cross array in this order:
    mic0=(+x), mic1=(+y), mic2=(-x), mic3=(-y).  Pairwise TDOA values are
    clamped to the physical maximum delay, then solved as a small least-squares
    direction-vector problem.
    """
    audio_4ch = np.asarray(audio_4ch, dtype=np.float32)
    if audio_4ch.ndim != 2 or audio_4ch.shape[1] < 4:
        raise ValueError("4-channel audio is required for GCC-PHAT DOA")

    d = float(mic_distance)
    c = float(sound_speed)
    max_tau = d / c
    positions = np.asarray(
        [
            [d / 2.0, 0.0],
            [0.0, d / 2.0],
            [-d / 2.0, 0.0],
            [0.0, -d / 2.0],
        ],
        dtype=np.float32,
    )

    rows = []
    delays_m = []
    pair_debug = []
    for i in range(4):
        for j in range(i + 1, 4):
            info = _gcc_phat_info(
                audio_4ch[:, i],
                audio_4ch[:, j],
                fs=fs,
                max_tau=max_tau,
                interp=interp,
            )
            tau = float(np.clip(info["tau"], -max_tau, max_tau))
            info = dict(info)
            info.update({"i": i, "j": j, "tau": tau, "tau_us": tau * 1e6})
            pair_debug.append(info)
            if info["peak_ratio"] < float(min_peak_ratio):
                continue
            rows.append(positions[i] - positions[j])
            delays_m.append(tau * c)

    if len(rows) < 2:
        # Fall back to the two opposite pairs if the confidence gate is too strict.
        pair02 = _gcc_phat_info(audio_4ch[:, 0], audio_4ch[:, 2], fs=fs, max_tau=max_tau, interp=interp)
        pair13 = _gcc_phat_info(audio_4ch[:, 1], audio_4ch[:, 3], fs=fs, max_tau=max_tau, interp=interp)
        x = np.clip(pair02["tau"] * c / max(d, 1e-9), -1.0, 1.0)
        y = np.clip(pair13["tau"] * c / max(d, 1e-9), -1.0, 1.0)
        angle = float(np.degrees(np.arctan2(y, x)) % 360.0)
        debug = {"pairs": pair_debug, "used_pairs": 2, "fallback": "opposite_pairs"}
        return (angle, debug) if return_debug else angle

    a = np.asarray(rows, dtype=np.float32)
    b = np.asarray(delays_m, dtype=np.float32)
    direction, *_ = np.linalg.lstsq(a, b, rcond=None)
    norm = float(np.linalg.norm(direction))
    if norm > 1e-9:
        direction = direction / norm
    angle = float(np.degrees(np.arctan2(direction[1], direction[0])) % 360.0)
    debug = {"pairs": pair_debug, "used_pairs": len(rows), "fallback": ""}
    return (angle, debug) if return_debug else angle


def _estimate_direction_4ch(audio_4ch, fs=16000, mic_distance=0.065):
    if audio_4ch.ndim != 2 or audio_4ch.shape[1] < 4:
        raise ValueError("4-channel audio is required for GCC-PHAT DOA")
    return estimate_direction_gcc_phat(audio_4ch, fs=fs, mic_distance=mic_distance)


def _direction_gcc_pair_features(a, b, sr, max_tau_sec, interp=8):
    n = int(a.shape[0] + b.shape[0])
    nfft = 1 << int(np.ceil(np.log2(max(1, n))))
    fa = np.fft.rfft(a, n=nfft)
    fb = np.fft.rfft(b, n=nfft)
    cross = fa * np.conj(fb)
    cross /= np.maximum(np.abs(cross), 1e-12)
    cc = np.fft.irfft(cross, n=nfft * interp)
    max_shift = min(int(interp * sr * max_tau_sec), cc.shape[0] // 2)
    if max_shift <= 0:
        return 0.0, 0.0, 0.0
    cc = np.concatenate((cc[-max_shift:], cc[: max_shift + 1]))
    abs_cc = np.abs(cc)
    peak_idx = int(np.argmax(abs_cc))
    shift = peak_idx - max_shift
    tau = float(shift) / float(interp * sr)
    peak = float(abs_cc[peak_idx])
    mean = float(np.mean(abs_cc) + 1e-9)
    return tau / max_tau_sec, peak, peak / mean


def _parse_channel_list(value, fallback=(0, 1, 2, 3)):
    text = str(value or "").strip()
    if not text:
        return tuple(fallback)
    channels = tuple(int(part.strip()) for part in text.split(",") if part.strip() != "")
    if len(channels) != 4:
        raise ValueError("TELLO_AUDIO_MIC_CHANNELS must contain exactly 4 comma-separated channel indexes")
    if len(set(channels)) != 4:
        raise ValueError("TELLO_AUDIO_MIC_CHANNELS must not contain duplicate indexes")
    if min(channels) < 0:
        raise ValueError("TELLO_AUDIO_MIC_CHANNELS indexes must be non-negative")
    return channels


class TelloAudioFallback:
    def __init__(
        self,
        model_path,
        config_path,
        alsa_device="auto",
        channels=6,
        threshold=0.70,
        min_avg_score=None,
        consecutive=3,
        cooldown_sec=1.2,
        min_rms=0.008,
        doa_offset=0,
        doa_method="gcc",
        verbose=False,
    ):
        self.model_path = model_path
        self.config_path = config_path
        self.alsa_device = str(alsa_device or "auto")
        self.input_device = None
        self.channels = int(channels)
        self.threshold = float(threshold)
        self.min_avg_score = float(0.55 if min_avg_score is None else min_avg_score)
        self.cooldown_sec = float(cooldown_sec)
        self.min_rms = float(min_rms)
        self.doa_offset = float(doa_offset)
        self.mic_distance = float(os.getenv("TELLO_AUDIO_MIC_DISTANCE", "0.045"))
        self.mic_channels = _parse_channel_list(os.getenv("TELLO_AUDIO_MIC_CHANNELS", "1,2,3,4"))
        self.mic_debug = os.getenv("TELLO_AUDIO_MIC_DEBUG", "0") == "1"
        self.gcc_debug = os.getenv("TELLO_AUDIO_GCC_DEBUG", "0") == "1"
        self.timing_debug = os.getenv("TELLO_AUDIO_TIMING_DEBUG", "0") == "1"
        self.skip_gcc_with_cnn6 = os.getenv("TELLO_AUDIO_SKIP_GCC_WITH_CNN6", "1") == "1"
        self.gcc_min_peak_ratio = float(os.getenv("TELLO_AUDIO_GCC_MIN_PEAK_RATIO", "1.15"))
        self.preprocess_enabled = os.getenv("TELLO_AUDIO_PREPROCESS", "1") == "1"
        self.bandpass_low = float(os.getenv("TELLO_AUDIO_BANDPASS_LOW", "1656.2"))
        self.bandpass_high = float(os.getenv("TELLO_AUDIO_BANDPASS_HIGH", "2656.2"))
        self.bandpass_order = int(os.getenv("TELLO_AUDIO_BANDPASS_ORDER", "5"))
        self.pre_gain_noise_floor = float(os.getenv("TELLO_AUDIO_PRE_GAIN_NOISE_FLOOR", "0.003"))
        self.pre_gain_target = float(os.getenv("TELLO_AUDIO_PRE_GAIN_TARGET", "0.9"))
        self.preprocess_target = os.getenv("TELLO_AUDIO_PREPROCESS_TARGET", "doa").strip().lower()
        self._sos_filter = None
        self._sosfilt = None
        self.audio_backend = os.getenv("TELLO_AUDIO_BACKEND", "arecord").strip().lower()
        self.requested_doa_method = str(doa_method or "gcc").lower()
        self.direction_mode = os.getenv("TELLO_AUDIO_DIRECTION_MODE", "legacy").strip().lower()
        self.direction_model_path = os.getenv(
            "TELLO_AUDIO_DIRECTION_MODEL",
            os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                "models",
                "audio_direction",
                "direction_6sector_lr.npz",
            ),
        )
        self._direction_model = None
        self.stride_sec = float(os.getenv("TELLO_AUDIO_STRIDE_SEC", "0.25"))
        self.verbose = bool(verbose)
        self._queue = queue.Queue(maxsize=16)
        self._running = False
        self._paused = False
        self._latest = None
        self._status = None
        self._lock = threading.Lock()
        self._capture_error = ""
        self._load_config()
        smoothing = self.audio_config.get("smoothing", {})
        self.window_size = int(os.getenv("TELLO_AUDIO_WINDOW_SIZE", str(smoothing.get("window_size", 5))))
        self.required_tello_count = int(consecutive or smoothing.get("required_tello_count", 3))
        if min_avg_score is None:
            self.min_avg_score = float(smoothing.get("avg_threshold", self.min_avg_score))
        self.window_size = max(1, self.window_size)
        self.required_tello_count = max(1, min(self.required_tello_count, self.window_size))
        self.stride_sec = max(0.05, min(self.stride_sec, float(self.clip_samples) / float(self.sample_rate)))
        self.block_samples = max(1, int(round(self.sample_rate * self.stride_sec)))
        if self.channels < 4:
            raise RuntimeError("ReSpeaker DOA/classification split needs at least 4 microphone channels")
        self._interpreter = _load_tflite_interpreter(model_path)
        self._input = self._interpreter.get_input_details()[0]
        self._output = self._interpreter.get_output_details()[0]
        self._validate_model_input()
        self._init_preprocess_filter()
        self._load_direction_model()
        self._doa = None
        self._want_usb_doa = self.requested_doa_method in ("usb", "auto")
        self._doa_method = "gcc"
        if self.requested_doa_method not in ("gcc", "usb", "auto"):
            raise RuntimeError(f"Unsupported TELLO_AUDIO_DOA_METHOD={self.requested_doa_method}")

    def _load_direction_model(self):
        if self.direction_mode in ("cnn6", "cnn_6sector", "cnn_direction", "tflite6"):
            self._load_cnn_direction_model()
            return
        if self.direction_mode not in ("learned6", "learned_6sector", "direction_model"):
            return
        if not os.path.exists(self.direction_model_path):
            raise RuntimeError(f"TELLO_AUDIO_DIRECTION_MODEL not found: {self.direction_model_path}")
        data = np.load(self.direction_model_path)
        self._direction_model = {
            "weights": data["weights"].astype(np.float32),
            "bias": data["bias"].astype(np.float32),
            "mean": data["mean"].astype(np.float32),
            "std": data["std"].astype(np.float32),
            "angle_labels": data["angle_labels"].astype(np.float32),
            "feature_dim": int(np.ravel(data["feature_dim"])[0]),
        }
        print(
            f"[audio] learned direction model loaded: {self.direction_model_path} "
            f"angles={','.join(str(int(v)) for v in self._direction_model['angle_labels'])}",
            flush=True,
        )

    def _load_cnn_direction_model(self):
        if not os.path.exists(self.direction_model_path):
            raise RuntimeError(f"TELLO_AUDIO_DIRECTION_MODEL not found: {self.direction_model_path}")
        model_dir = os.path.dirname(os.path.abspath(self.direction_model_path))
        config_path = os.getenv(
            "TELLO_AUDIO_DIRECTION_FEATURE_CONFIG",
            os.path.join(model_dir, "feature_config.json"),
        )
        label_path = os.getenv(
            "TELLO_AUDIO_DIRECTION_LABEL_MAPPING",
            os.path.join(model_dir, "label_mapping.json"),
        )
        if not os.path.exists(config_path):
            raise RuntimeError(f"TELLO_AUDIO_DIRECTION_FEATURE_CONFIG not found: {config_path}")
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        if os.path.exists(label_path):
            with open(label_path, "r", encoding="utf-8") as f:
                label_cfg = json.load(f)
        else:
            label_cfg = {}
            if not any(key in cfg for key in ("class_to_angle_deg", "class_to_deg")):
                raise RuntimeError(f"TELLO_AUDIO_DIRECTION_LABEL_MAPPING not found: {label_path}")

        interpreter = _load_tflite_interpreter(self.direction_model_path)
        input_info = interpreter.get_input_details()[0]
        output_info = interpreter.get_output_details()[0]
        input_shape = tuple(int(v) for v in input_info["shape"])
        if len(input_shape) not in (3, 4):
            raise RuntimeError(f"direction CNN expected logmel NHWC or raw 1D input, got shape={input_shape}")

        norm = cfg.get("normalization", {})
        angle_labels = []
        if isinstance(label_cfg, dict) and "class_to_angle" in label_cfg:
            source = label_cfg.get("class_to_angle", {})
            max_class = max(int(k) for k in source.keys()) if source else input_shape[-1] - 1
            for i in range(max_class + 1):
                value = source.get(str(i), source.get(i))
                angle_labels.append(None if value is None else float(value))
        elif isinstance(label_cfg, dict) and label_cfg:
            max_class = max(int(k) for k in label_cfg.keys()) if label_cfg else 5
            for i in range(max_class + 1):
                item = label_cfg.get(str(i), label_cfg.get(i, {}))
                if isinstance(item, dict):
                    value = item.get("angle_deg")
                else:
                    value = item
                angle_labels.append(None if value is None else float(value))
        elif "class_to_angle_deg" in cfg or "class_to_deg" in cfg:
            source = cfg.get("class_to_angle_deg", cfg.get("class_to_deg", {}))
            max_class = max(int(k) for k in source.keys()) if source else 5
            for i in range(max_class + 1):
                value = source.get(str(i), source.get(i))
                angle_labels.append(None if value is None else float(value))
        else:
            angle_labels = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]

        if len(input_shape) == 3:
            cfg_shape = cfg.get("input_shape", list(input_shape))
            clip_samples = int(cfg.get("segment_samples", cfg_shape[1] if len(cfg_shape) >= 2 else input_shape[1]))
            self._direction_model = {
                "type": "cnn6",
                "feature_type": "raw1d",
                "interpreter": interpreter,
                "input": input_info,
                "output": output_info,
                "sample_rate": int(cfg.get("sample_rate", self.sample_rate)),
                "clip_samples": clip_samples,
                "angle_labels": angle_labels,
            }
            print(
                f"[audio] raw-1D CNN direction model loaded: {self.direction_model_path} "
                f"angles={','.join('noise' if v is None else str(int(v)) for v in angle_labels)}",
                flush=True,
            )
            return

        self._direction_model = {
            "type": "cnn6",
            "feature_type": "logmel",
            "interpreter": interpreter,
            "input": input_info,
            "output": output_info,
            "sample_rate": int(cfg.get("sample_rate", self.sample_rate)),
            "clip_samples": int(round(float(cfg.get("window_sec", cfg.get("segment_sec", 1.0))) * int(cfg.get("sample_rate", self.sample_rate)))),
            "n_mels": int(cfg.get("n_mels", input_shape[1])),
            "n_fft": int(cfg.get("n_fft", 1024)),
            "hop_length": int(cfg.get("hop_length", 256)),
            "fmin": float(cfg.get("fmin", 50.0)),
            "fmax": float(cfg.get("fmax", 8000.0)),
            "target_frames": int(cfg.get("input_shape", [input_shape[1], input_shape[2], input_shape[3]])[1]),
            "mean": float(norm.get("mean", cfg.get("mean", 0.0))),
            "std": float(norm.get("std", cfg.get("std", 1.0))),
            "angle_labels": angle_labels,
            "mel_basis": None,
            "hann": None,
        }
        self._direction_model["mel_basis"] = _mel_filterbank(
            self._direction_model["sample_rate"],
            self._direction_model["n_fft"],
            self._direction_model["n_mels"],
            self._direction_model["fmin"],
            self._direction_model["fmax"],
        )
        self._direction_model["hann"] = np.hanning(self._direction_model["n_fft"] + 1)[:-1].astype(np.float32)
        print(
            f"[audio] CNN direction model loaded: {self.direction_model_path} "
            f"angles={','.join('noise' if v is None else str(int(v)) for v in angle_labels)}",
            flush=True,
        )

    def _init_preprocess_filter(self):
        if not self.preprocess_enabled:
            return
        try:
            from scipy.signal import butter, sosfilt

            nyq = 0.5 * float(self.sample_rate)
            low = max(1.0, min(float(self.bandpass_low), nyq - 1.0))
            high = max(low + 1.0, min(float(self.bandpass_high), nyq - 1.0))
            self._sos_filter = butter(
                max(1, int(self.bandpass_order)),
                [low / nyq, high / nyq],
                btype="band",
                output="sos",
            )
            self._sosfilt = sosfilt
            print(
                f"[audio] preprocess=on bandpass={low:.1f}-{high:.1f}Hz "
                f"order={self.bandpass_order} gain_target={self.pre_gain_target:.2f} "
                f"floor={self.pre_gain_noise_floor:.4f} target={self.preprocess_target}",
                flush=True,
            )
        except Exception as exc:
            self.preprocess_enabled = False
            print(f"[audio] preprocess disabled: {exc}", flush=True)

    def _preprocess_mic_audio(self, mic_4ch):
        audio = np.asarray(mic_4ch, dtype=np.float32)
        if not self.preprocess_enabled or self._sos_filter is None or self._sosfilt is None:
            return audio
        filtered = self._sosfilt(self._sos_filter, audio, axis=0).astype(np.float32)
        max_amp = float(np.max(np.abs(filtered))) if filtered.size else 0.0
        if max_amp <= self.pre_gain_noise_floor:
            return np.zeros_like(filtered, dtype=np.float32)
        normalized = (filtered / max(max_amp, 1e-9)) * float(self.pre_gain_target)
        return np.clip(normalized, -1.0, 1.0).astype(np.float32)

    def _init_usb_doa(self):
        if not self._want_usb_doa or self._doa is not None:
            return
        try:
            self._doa = ReSpeakerDOA(offset=self.doa_offset)
            self._doa_method = "usb"
            print("[audio] ReSpeaker USB DOA enabled", flush=True)
        except Exception as exc:
            if self.requested_doa_method == "usb":
                raise
            self._doa_method = "gcc"
            print(f"[audio] USB DOA unavailable: {exc}; using GCC-PHAT from raw 4ch stream", flush=True)

    def _resolve_sounddevice_input(self, sd):
        requested = str(self.alsa_device or "").strip()
        lowered = requested.lower()
        if requested and lowered not in ("auto", "default"):
            if _is_blocked_audio_text(requested):
                raise RuntimeError(f"Refusing camera audio device for ReSpeaker fallback: {requested}")
            if requested.isdigit():
                idx = int(requested)
                dev = sd.query_devices(idx)
                if dev["max_input_channels"] < self.channels:
                    raise RuntimeError(f"Audio device {idx} has only {dev['max_input_channels']} input channels")
                return idx
            return requested

        for idx, dev in enumerate(sd.query_devices()):
            name = str(dev.get("name", ""))
            if _is_blocked_audio_text(name):
                continue
            if _is_respeaker_text(name) and int(dev.get("max_input_channels", 0)) >= self.channels:
                print(f"[audio] ReSpeaker sounddevice input={idx} ({name})", flush=True)
                return idx
        raise RuntimeError("ReSpeaker sounddevice input not found; refusing default/camera audio input")

    def _load_config(self):
        with open(self.config_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.audio_config = data
        self.sample_rate = int(data.get("sample_rate", data.get("sr", 16000)))
        self.clip_samples = int(data.get("clip_samples", data.get("clip_len", 16000)))
        if "clip_sec" in data:
            self.clip_samples = int(round(float(data["clip_sec"]) * self.sample_rate))
        self.n_fft = int(data.get("n_fft", 1024))
        self.hop_length = int(data.get("hop_length", 256))
        self.n_mels = int(data.get("n_mels", 64))
        self.fmin = float(data.get("fmin", 50.0))
        self.fmax = float(data.get("fmax", 8000.0))
        self.target_frames = int(data.get("target_frames", data.get("input_shape", [1, self.n_mels, 63, 1])[2]))
        self._mel_basis = _mel_filterbank(
            self.sample_rate, self.n_fft, self.n_mels, self.fmin, self.fmax
        )
        self._hann = np.hanning(self.n_fft + 1)[:-1].astype(np.float32)

    def _validate_model_input(self):
        shape = tuple(int(v) for v in self._input["shape"])
        dtype = self._input["dtype"]
        expected = (1, self.n_mels, self.target_frames, 1)
        if shape != expected:
            print(f"[audio] warning: model input shape={shape}, expected={expected}", flush=True)
        if dtype != np.float32:
            print(f"[audio] warning: model input dtype={dtype}, expected=float32", flush=True)

    def _audio_to_logmel(self, audio):
        y = np.asarray(audio, dtype=np.float32)
        if y.size < self.clip_samples:
            y = np.pad(y, (0, self.clip_samples - y.size))
        elif y.size > self.clip_samples:
            y = y[-self.clip_samples:]
        y = np.pad(y, (self.n_fft // 2, self.n_fft // 2), mode="constant")
        frames = []
        for start in range(0, len(y) - self.n_fft + 1, self.hop_length):
            frame = y[start : start + self.n_fft] * self._hann
            frames.append(np.abs(np.fft.rfft(frame, n=self.n_fft)) ** 2)
        power = np.asarray(frames, dtype=np.float32).T
        mel = np.maximum(np.dot(self._mel_basis, power), 1e-10)
        ref = float(np.max(mel))
        logmel = 10.0 * np.log10(mel) - 10.0 * np.log10(max(ref, 1e-10))
        if logmel.shape[1] < self.target_frames:
            pad = self.target_frames - logmel.shape[1]
            logmel = np.pad(logmel, ((0, 0), (0, pad)), mode="edge")
        elif logmel.shape[1] > self.target_frames:
            logmel = logmel[:, : self.target_frames]
        logmel = (logmel - np.mean(logmel)) / (np.std(logmel) + 1e-6)
        return logmel.astype(np.float32)[..., np.newaxis]

    def _predict_probs(self, mono):
        feat = self._audio_to_logmel(mono)
        arr = feat[np.newaxis, ...].astype(np.float32)
        self._interpreter.set_tensor(self._input["index"], arr)
        self._interpreter.invoke()
        out = self._interpreter.get_tensor(self._output["index"])
        probs = np.ravel(out).astype(np.float32)
        if probs.size < 2:
            raise RuntimeError(f"Expected 2-class TFLite output, got shape={out.shape}")
        return float(probs[0]), float(probs[1])

    def _learned_direction_features(self, raw_mic_4ch, pre_mic_4ch):
        raw = np.asarray(raw_mic_4ch, dtype=np.float32)
        pre = np.asarray(pre_mic_4ch, dtype=np.float32)
        raw_rms = np.sqrt(np.mean(raw * raw, axis=0) + 1e-12)
        rms = np.sqrt(np.mean(pre * pre, axis=0) + 1e-12)
        feats = []
        feats.extend(np.log(raw_rms + 1e-6).tolist())
        feats.extend(np.log(rms + 1e-6).tolist())
        for i in range(4):
            for j in range(i + 1, 4):
                feats.append(float(np.log((rms[i] + 1e-6) / (rms[j] + 1e-6))))

        max_tau = float(self.mic_distance) / 340.0
        tx, px, rx = _direction_gcc_pair_features(pre[:, 0], pre[:, 2], self.sample_rate, max_tau)
        ty, py, ry = _direction_gcc_pair_features(pre[:, 1], pre[:, 3], self.sample_rate, max_tau)
        angle = np.degrees(np.arctan2(ty, tx)) % 360.0
        feats.extend([tx, ty, px, py, rx, ry])
        feats.extend([float(np.sin(np.deg2rad(angle))), float(np.cos(np.deg2rad(angle)))])

        for i in range(4):
            x = pre[:, i]
            zcr = np.mean(np.abs(np.diff(np.signbit(x).astype(np.float32))))
            feats.append(float(zcr))
        mono = pre.mean(axis=1)
        feats.append(float(np.sqrt(np.mean(mono * mono) + 1e-12)))
        return np.asarray(feats, dtype=np.float32)

    def _predict_learned_direction(self, raw_mic_4ch, pre_mic_4ch):
        model = self._direction_model
        if not model:
            return None
        if model.get("type") == "cnn6":
            return self._predict_cnn_direction(raw_mic_4ch)
        feat = self._learned_direction_features(raw_mic_4ch, pre_mic_4ch)
        if feat.shape[0] != int(model["feature_dim"]):
            raise RuntimeError(f"direction feature dim mismatch: {feat.shape[0]} != {model['feature_dim']}")
        x = (feat - model["mean"]) / np.maximum(model["std"], 1e-6)
        logits = x @ model["weights"] + model["bias"]
        logits = logits - np.max(logits)
        probs = np.exp(logits)
        probs = probs / max(float(np.sum(probs)), 1e-12)
        order = np.argsort(probs)[::-1]
        best = int(order[0])
        second = int(order[1]) if order.size > 1 else best
        return {
            "angle": float(model["angle_labels"][best]) % 360.0,
            "confidence": float(probs[best]),
            "margin": float(probs[best] - probs[second]),
            "scores": {str(int(model["angle_labels"][i])): float(probs[i]) for i in range(len(probs))},
        }

    def _cnn_direction_feature(self, raw_mic_4ch):
        model = self._direction_model
        if model.get("feature_type") == "raw1d":
            return self._raw1d_direction_feature(raw_mic_4ch)
        y4 = np.asarray(raw_mic_4ch, dtype=np.float32)
        target_samples = int(model["clip_samples"])
        if y4.shape[0] < target_samples:
            y4 = np.pad(y4, ((0, target_samples - y4.shape[0]), (0, 0)), mode="constant")
        elif y4.shape[0] > target_samples:
            y4 = y4[-target_samples:, :]

        n_fft = int(model["n_fft"])
        hop = int(model["hop_length"])
        hann = model["hann"]
        mel_basis = model["mel_basis"]
        mel_list = []
        common_ref = 1e-10
        for ch in range(4):
            y = y4[:, ch].astype(np.float32)
            y = np.pad(y, (n_fft // 2, n_fft // 2), mode="constant")
            frames = []
            for start in range(0, len(y) - n_fft + 1, hop):
                frame = y[start : start + n_fft] * hann
                frames.append(np.abs(np.fft.rfft(frame, n=n_fft)) ** 2)
            power = np.asarray(frames, dtype=np.float32).T
            mel = np.maximum(np.dot(mel_basis, power), 1e-10)
            mel_list.append(mel)
            common_ref = max(common_ref, float(np.max(mel)))

        feats = []
        for mel in mel_list:
            feats.append(10.0 * np.log10(mel) - 10.0 * np.log10(max(common_ref, 1e-10)))
        feat = np.stack(feats, axis=-1).astype(np.float32)
        target_frames = int(model["target_frames"])
        if feat.shape[1] < target_frames:
            feat = np.pad(feat, ((0, 0), (0, target_frames - feat.shape[1]), (0, 0)), mode="edge")
        elif feat.shape[1] > target_frames:
            feat = feat[:, :target_frames, :]
        feat = (feat - float(model["mean"])) / max(float(model["std"]), 1e-6)
        return feat[np.newaxis, ...].astype(np.float32)

    def _raw1d_direction_feature(self, raw_mic_4ch):
        model = self._direction_model
        y4 = np.asarray(raw_mic_4ch, dtype=np.float32)
        target_samples = int(model["clip_samples"])
        if y4.shape[0] < target_samples:
            y4 = np.pad(y4, ((0, target_samples - y4.shape[0]), (0, 0)), mode="constant")
        elif y4.shape[0] > target_samples:
            y4 = y4[-target_samples:, :]
        if y4.shape[1] != 4:
            raise RuntimeError(f"raw-1D direction model needs 4 channels, got {y4.shape}")
        mean = np.mean(y4, axis=0, keepdims=True)
        std = np.std(y4, axis=0, keepdims=True)
        y4 = (y4 - mean) / np.maximum(std, 1e-6)
        return y4[np.newaxis, ...].astype(np.float32)

    def _predict_cnn_direction(self, raw_mic_4ch):
        model = self._direction_model
        feat = self._cnn_direction_feature(raw_mic_4ch)
        interpreter = model["interpreter"]
        interpreter.set_tensor(model["input"]["index"], feat)
        interpreter.invoke()
        probs = np.ravel(interpreter.get_tensor(model["output"]["index"])).astype(np.float32)
        if probs.size < 2:
            raise RuntimeError(f"direction CNN output too small: {probs.shape}")
        if np.any(probs < 0.0) or abs(float(np.sum(probs)) - 1.0) > 0.05:
            logits = probs - float(np.max(probs))
            probs = np.exp(logits).astype(np.float32)
            probs = probs / max(float(np.sum(probs)), 1e-12)
        order = np.argsort(probs)[::-1]
        best = int(order[0])
        second = int(order[1]) if order.size > 1 else best
        labels = model["angle_labels"]
        angle = labels[best] if best < len(labels) else None
        if angle is None:
            return {
                "angle": None,
                "confidence": float(probs[best]),
                "margin": float(probs[best] - probs[second]),
                "scores": {
                    ("noise" if i < len(labels) and labels[i] is None else str(int(labels[i] if i < len(labels) else i))): float(probs[i])
                    for i in range(len(probs))
                },
            }
        return {
            "angle": float(angle) % 360.0,
            "confidence": float(probs[best]),
            "margin": float(probs[best] - probs[second]),
            "scores": {
                ("noise" if i < len(labels) and labels[i] is None else str(int(labels[i] if i < len(labels) else i))): float(probs[i])
                for i in range(len(probs))
            },
        }

    def start(self):
        if self._running:
            return self
        self._running = True
        self._paused = False
        threading.Thread(target=self._stream_loop, daemon=True).start()
        return self

    def _clear_queue(self):
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break

    def pause(self):
        if not self._running:
            return self
        self._paused = True
        self._clear_queue()
        with self._lock:
            self._latest = None
            self._status = None
        return self

    def resume(self):
        if self._running:
            self._paused = False
        return self

    def is_paused(self):
        return bool(self._paused)

    def _queue_audio(self, chunk, captured_at):
        if self._paused:
            return
        try:
            self._queue.put_nowait((chunk, captured_at))
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.put_nowait((chunk, captured_at))
            except queue.Empty:
                pass

    def _stream_loop(self):
        if self.audio_backend == "arecord":
            self._stream_loop_arecord()
            return

        try:
            import sounddevice as sd
        except Exception as exc:
            self._capture_error = f"sounddevice import failed: {exc}"
            print(f"[audio] {self._capture_error}", flush=True)
            return

        try:
            self.input_device = self._resolve_sounddevice_input(sd)
        except Exception as exc:
            self._capture_error = str(exc)
            print(f"[audio] {self._capture_error}", flush=True)
            return

        def callback(indata, frames, time_info, status):
            if status:
                print(f"[audio] input status: {status}", flush=True)
            chunk = np.asarray(indata, dtype=np.float32).copy()
            if chunk.ndim == 1:
                chunk = chunk[:, np.newaxis]
            self._queue_audio(chunk, time.perf_counter())

        print(
            f"[audio] sounddevice stream device={self.input_device} channels={self.channels} "
            f"sr={self.sample_rate} block={self.block_samples} stride={self.stride_sec:.2f}s "
            f"model={self.model_path}",
            flush=True,
        )
        try:
            with sd.InputStream(
                samplerate=self.sample_rate,
                channels=self.channels,
                device=self.input_device,
                blocksize=self.block_samples,
                dtype="float32",
                callback=callback,
            ):
                self._init_usb_doa()
                self._infer_loop()
        except Exception as exc:
            self._capture_error = f"sounddevice stream failed: {exc}"
            print(f"[audio] {self._capture_error}", flush=True)

    def _resolve_arecord_input(self):
        requested = str(self.alsa_device or "").strip()
        lowered = requested.lower()
        if not requested or lowered in ("auto", "default"):
            device = _find_respeaker_alsa_device()
            if device.startswith("plughw:"):
                device = "hw:" + device[len("plughw:") :]
            return device
        return _validate_respeaker_alsa_device(requested)

    def _stream_loop_arecord(self):
        try:
            self.input_device = self._resolve_arecord_input()
        except Exception as exc:
            self._capture_error = str(exc)
            print(f"[audio] {self._capture_error}", flush=True)
            return

        cmd = [
            "arecord",
            "-q",
            "-D",
            str(self.input_device),
            "-f",
            "S16_LE",
            "-r",
            str(int(self.sample_rate)),
            "-c",
            str(int(self.channels)),
            "-t",
            "raw",
        ]
        bytes_per_sample = 2
        bytes_per_chunk = int(self.block_samples) * int(self.channels) * bytes_per_sample
        print(
            f"[audio] arecord stream device={self.input_device} channels={self.channels} "
            f"sr={self.sample_rate} block={self.block_samples} stride={self.stride_sec:.2f}s "
            f"model={self.model_path}",
            flush=True,
        )
        proc = None
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=bytes_per_chunk * 2,
            )
            self._init_usb_doa()
            infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
            infer_thread.start()
            while self._running:
                data = proc.stdout.read(bytes_per_chunk) if proc.stdout is not None else b""
                if not data:
                    err = ""
                    if proc.stderr is not None:
                        try:
                            err = proc.stderr.read().decode("utf-8", errors="replace").strip()
                        except Exception:
                            err = ""
                    self._capture_error = f"arecord stream ended: {err}"
                    print(f"[audio] {self._capture_error}", flush=True)
                    break
                samples = np.frombuffer(data, dtype="<i2")
                frame_count = samples.size // int(self.channels)
                if frame_count <= 0:
                    continue
                samples = samples[: frame_count * int(self.channels)]
                chunk = samples.reshape(frame_count, int(self.channels)).astype(np.float32) / 32768.0
                self._queue_audio(chunk, time.perf_counter())
            infer_thread.join(timeout=0.2)
        except Exception as exc:
            self._capture_error = f"arecord stream failed: {exc}"
            print(f"[audio] {self._capture_error}", flush=True)
        finally:
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=1.0)
                except Exception:
                    proc.kill()

    def _infer_loop(self):
        ring = np.zeros((self.clip_samples, self.channels), dtype=np.float32)
        ring_count = 0
        write_pos = 0
        latest_capture_time = time.perf_counter()
        candidates = deque(maxlen=self.window_size)
        tello_scores = deque(maxlen=self.window_size)
        last_emit = 0.0
        while self._running:
            if self._paused:
                self._clear_queue()
                time.sleep(0.05)
                continue
            try:
                chunk, captured_at = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if self._paused:
                self._clear_queue()
                continue

            chunks = [(chunk, captured_at)]
            drained = 0
            while True:
                try:
                    chunks.append(self._queue.get_nowait())
                    drained += 1
                except queue.Empty:
                    break

            for chunk, captured_at in chunks:
                chunk = np.asarray(chunk, dtype=np.float32)
                if chunk.ndim == 1:
                    chunk = chunk[:, np.newaxis]
                if chunk.shape[1] < 4:
                    self._capture_error = f"ReSpeaker stream has {chunk.shape[1]} channels; need at least 4"
                    print(f"[audio] {self._capture_error}", flush=True)
                    continue
                if max(self.mic_channels) >= chunk.shape[1]:
                    self._capture_error = (
                        f"TELLO_AUDIO_MIC_CHANNELS={self.mic_channels} exceeds "
                        f"stream channel count={chunk.shape[1]}"
                    )
                    print(f"[audio] {self._capture_error}", flush=True)
                    continue
                if chunk.shape[1] != self.channels:
                    chunk = chunk[:, : self.channels]
                latest_capture_time = float(captured_at)
                if chunk.shape[0] >= self.clip_samples:
                    ring[:, :] = chunk[-self.clip_samples :, : self.channels]
                    write_pos = 0
                    ring_count = self.clip_samples
                else:
                    n = chunk.shape[0]
                    end = write_pos + n
                    if end <= self.clip_samples:
                        ring[write_pos:end, :] = chunk[:, : self.channels]
                    else:
                        first = self.clip_samples - write_pos
                        ring[write_pos:, :] = chunk[:first, : self.channels]
                        ring[: end % self.clip_samples, :] = chunk[first:, : self.channels]
                    write_pos = end % self.clip_samples
                    ring_count = min(self.clip_samples, ring_count + n)
            if ring_count < self.clip_samples:
                continue

            loop_t0 = time.perf_counter()
            if write_pos == 0:
                raw_audio = ring.copy()
            else:
                raw_audio = np.concatenate((ring[write_pos:, :], ring[:write_pos, :]), axis=0)

            raw_mic_4ch = raw_audio[:, self.mic_channels]
            pre_mic_4ch = self._preprocess_mic_audio(raw_mic_4ch)
            use_pre_for_model = self.preprocess_target in ("model", "all", "both", "model_doa")
            use_pre_for_doa = self.preprocess_target in ("doa", "all", "both", "model_doa")
            model_mic_4ch = pre_mic_4ch if use_pre_for_model else raw_mic_4ch
            doa_mic_4ch = pre_mic_4ch if use_pre_for_doa else raw_mic_4ch
            audio_mono = model_mic_4ch.mean(axis=1).astype(np.float32)
            audio_mono = np.clip(audio_mono, -1.0, 1.0)
            rms = float(np.sqrt(np.mean(audio_mono * audio_mono)))
            if self.mic_debug:
                mic_rms = np.sqrt(np.mean(model_mic_4ch * model_mic_4ch, axis=0))
                pre_mic_rms = np.sqrt(np.mean(pre_mic_4ch * pre_mic_4ch, axis=0))
                raw_mic_rms = np.sqrt(np.mean(raw_mic_4ch * raw_mic_4ch, axis=0))
                mic_text = ", ".join(
                    f"Ch{ch}: model={mic_rms[idx]:.5f} pre={pre_mic_rms[idx]:.5f} raw={raw_mic_rms[idx]:.5f}"
                    for idx, ch in enumerate(self.mic_channels)
                )
                print(f"[MIC DEBUG] {mic_text}", flush=True)
            prep_ms = (time.perf_counter() - loop_t0) * 1000.0
            infer_t0 = time.perf_counter()
            noise_prob, tello_prob = self._predict_probs(audio_mono)
            infer_ms = (time.perf_counter() - infer_t0) * 1000.0
            candidate = bool(tello_prob >= self.threshold and rms >= self.min_rms)
            candidates.append(candidate)
            tello_scores.append(tello_prob)
            count = int(sum(candidates))
            avg_score = float(np.mean(tello_scores)) if tello_scores else 0.0
            detected = (
                len(candidates) >= self.window_size
                and count >= self.required_tello_count
                and avg_score >= self.min_avg_score
            )

            learned_direction = None
            doa_t0 = time.perf_counter()
            if self._direction_model is not None and (candidate or detected):
                try:
                    learned_direction = self._predict_learned_direction(raw_mic_4ch, pre_mic_4ch)
                except Exception as exc:
                    if self.verbose:
                        print(f"[audio] learned direction failed: {exc}", flush=True)
            if learned_direction is not None and self.skip_gcc_with_cnn6:
                doa_raw = float(learned_direction["angle"]) % 360.0
                doa_method = "cnn6"
            elif self._doa is not None:
                doa_raw = float(self._doa.read())
                doa_method = self._doa_method
            else:
                doa_method = "gcc"
                if candidate or detected:
                    doa_raw, gcc_debug = estimate_direction_gcc_phat(
                        doa_mic_4ch,
                        fs=self.sample_rate,
                        mic_distance=self.mic_distance,
                        min_peak_ratio=self.gcc_min_peak_ratio,
                        return_debug=True,
                    )
                    doa_raw = (doa_raw + self.doa_offset) % 360.0
                    if self.gcc_debug:
                        pair_text = " ".join(
                            (
                                f"{p['i']}-{p['j']}:"
                                f"{p['tau_us']:+.0f}us/"
                                f"{p['peak_ratio']:.1f}x"
                            )
                            for p in gcc_debug["pairs"]
                        )
                        print(
                            f"[GCC DEBUG] angle={doa_raw:.1f} used={gcc_debug['used_pairs']} "
                            f"{gcc_debug['fallback']} {pair_text}",
                            flush=True,
                        )
                else:
                    doa_raw = 0.0
                    doa_method = "gcc_idle"
            doa_ms = (time.perf_counter() - doa_t0) * 1000.0
            doa = self._normalize_relative_angle(doa_raw)
            now = time.time()
            latency_ms = int(round((time.perf_counter() - latest_capture_time) * 1000.0))
            timing_text = ""
            if self.timing_debug:
                timing_text = (
                    f", prep={prep_ms:.1f}ms, infer={infer_ms:.1f}ms, "
                    f"doa_calc={doa_ms:.1f}ms, drained={drained}"
                )
            print(
                f"tello_prob={tello_prob:.3f}, noise_prob={noise_prob:.3f}, "
                f"candidate={candidate}, detected={detected}, count={count}/{self.window_size}, "
                f"avg={avg_score:.3f}, latency={latency_ms}ms, "
                f"doa={int(round(doa_raw)) % 360}, doa_method={doa_method}{timing_text}",
                flush=True,
            )

            with self._lock:
                self._status = {
                    "angle": doa,
                    "raw_angle": doa_raw,
                    "score": tello_prob,
                    "noise_prob": noise_prob,
                    "avg_score": avg_score,
                    "hit_count": count,
                    "rms": rms,
                    "quiet": rms < self.min_rms,
                    "detected": detected,
                    "candidate": candidate,
                    "latency_ms": latency_ms,
                    "doa_method": doa_method,
                    "time": now,
                }
                if learned_direction is not None:
                    self._status.update(
                        {
                            "learned_direction_deg": learned_direction["angle"],
                            "learned_direction_conf": learned_direction["confidence"],
                            "learned_direction_margin": learned_direction["margin"],
                            "learned_direction_scores": learned_direction["scores"],
                        }
                    )

            if detected and now - last_emit >= self.cooldown_sec:
                last_emit = now
                handle_drone_detected(doa_raw, tello_prob)
                with self._lock:
                    self._latest = {
                        "angle": doa,
                        "raw_angle": doa_raw,
                        "raw_doa_deg": doa_raw,
                        "score": tello_prob,
                        "noise_prob": noise_prob,
                        "avg_score": avg_score,
                        "hit_count": count,
                        "rms": rms,
                        "time": now,
                        "latency_ms": latency_ms,
                        "doa_method": doa_method,
                    }
                    if learned_direction is not None:
                        self._latest.update(
                            {
                                "learned_direction_deg": learned_direction["angle"],
                                "learned_direction_conf": learned_direction["confidence"],
                                "learned_direction_margin": learned_direction["margin"],
                                "learned_direction_scores": learned_direction["scores"],
                            }
                        )

    def _normalize_relative_angle(self, angle):
        angle = ((float(angle) + 180.0) % 360.0) - 180.0
        return angle

    def get_detection(self, max_age_sec=1.5):
        with self._lock:
            latest = dict(self._latest) if self._latest else None
        if latest and time.time() - latest["time"] <= max_age_sec:
            return latest
        return None

    def get_status(self, max_age_sec=1.5):
        with self._lock:
            status = dict(self._status) if self._status else None
        if status and time.time() - status["time"] <= max_age_sec:
            return status
        return None

    def get_error(self):
        return self._capture_error

    def stop(self):
        self._running = False
        self._paused = False
        self._clear_queue()
        if self._doa is not None:
            self._doa.stop()


class JunmoDroneAudioFallback:
    """Run junmoyolo26's drone-audio detector and expose recent DOA detections.

    The original junmoyolo26 path directly controlled the Jetson-side Dynamixel
    motor. In ultra_yubin we keep the detection pipeline on Jetson, but forward
    its detected DOA to Ultra96 PS through the normal `A angle conf valid` path.
    """

    def __init__(
        self,
        model_path,
        project_root=None,
        config_path=None,
        alsa_device="auto",
        channels=6,
        threshold=0.70,
        min_avg_score=None,
        consecutive=2,
        cooldown_sec=0.6,
        min_rms=0.008,
        doa_offset=0,
        doa_method="auto",
        mic_distance=0.065,
        audio_backend="sounddevice",
        stride_sec=None,
        verbose=False,
    ):
        self.project_root = project_root or os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        self.model_path = model_path
        self.config_path = config_path
        self.alsa_device = self._resolve_alsa_device(alsa_device)
        self.channels = int(channels)
        self.threshold = float(threshold)
        self.min_avg_score = float(self.threshold if min_avg_score is None else min_avg_score)
        self.consecutive = int(consecutive)
        self.cooldown_sec = float(cooldown_sec)
        self.min_rms = float(min_rms)
        self.doa_offset = float(doa_offset)
        self.doa_method = str(doa_method)
        self.mic_distance = float(mic_distance)
        self.audio_backend = str(audio_backend)
        self.stride_sec = float(os.getenv("TELLO_AUDIO_STRIDE_SEC", "0.25") if stride_sec is None else stride_sec)
        self.verbose = bool(verbose)
        self._latest = None
        self._paused = False
        self._lock = threading.Lock()
        self._thread = None

    def _resolve_alsa_device(self, alsa_device):
        requested = str(alsa_device or "").strip()
        if requested and requested.lower() not in ("auto", "default"):
            return _validate_respeaker_alsa_device(requested)
        return _find_respeaker_alsa_device()

    def start(self):
        if self._thread and self._thread.is_alive():
            self._paused = False
            return self
        self._paused = False
        local_audio_dir = os.path.join(os.path.dirname(__file__), "audio")
        local_pipeline = os.path.join(local_audio_dir, "junmo_pipeline_drone.py")
        audio_dir = local_audio_dir if os.path.exists(local_pipeline) else os.path.join(self.project_root, "src", "audio")
        pipeline_name = "junmo_pipeline_drone" if os.path.exists(local_pipeline) else "pipeline_drone"
        if not os.path.exists(os.path.join(audio_dir, f"{pipeline_name}.py")):
            raise FileNotFoundError(os.path.join(audio_dir, f"{pipeline_name}.py"))
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(self.model_path)
        if audio_dir not in sys.path:
            sys.path.insert(0, audio_dir)
        pipeline_drone = __import__(pipeline_name)

        def on_detect(text, doa, section, from_partial, stage, action):
            if self._paused:
                return
            score = 0.0
            avg_score = 0.0
            rms = 0.0
            hit_count = 0
            if isinstance(action, dict):
                score = float(action.get("confidence", 0.0))
                avg_score = float(action.get("avg_score", score))
                rms = float(action.get("rms", 0.0))
                hit_count = int(action.get("hit_count", 0))
            with self._lock:
                self._latest = {
                    "angle": float(doa),
                    "raw_angle": float(doa),
                    "raw_doa_deg": float(doa),
                    "section": int(section),
                    "score": score,
                    "avg_score": avg_score,
                    "hit_count": hit_count,
                    "rms": rms,
                    "mode": "junmo",
                    "time": time.time(),
                    "text": text,
                }

        def runner():
            pipeline_drone.run(
                None,
                self.channels,
                on_detect,
                self.model_path,
                config_path=self.config_path,
                threshold=self.threshold,
                min_avg_score=self.min_avg_score,
                consecutive=self.consecutive,
                cooldown=self.cooldown_sec,
                min_rms=self.min_rms,
                doa_offset=self.doa_offset,
                doa_method=self.doa_method,
                mic_distance=self.mic_distance,
                audio_backend=self.audio_backend,
                alsa_device=self.alsa_device,
                stride_sec=self.stride_sec,
                verbose=self.verbose,
            )

        self._thread = threading.Thread(target=runner, daemon=True)
        self._thread.start()
        return self

    def pause(self):
        self._paused = True
        with self._lock:
            self._latest = None
        return self

    def resume(self):
        self._paused = False
        return self

    def is_paused(self):
        return bool(self._paused)

    def get_detection(self, max_age_sec=1.5):
        with self._lock:
            latest = dict(self._latest) if self._latest else None
        if latest and time.time() - latest["time"] <= max_age_sec:
            return latest
        return None

    def stop(self):
        # junmoyolo26's pipeline has no cooperative stop flag. The worker is a
        # daemon thread, so process shutdown stops it. This keeps Ctrl+C fast.
        pass
