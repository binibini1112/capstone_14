import cv2
import glob
import re
import subprocess
import time
from threading import Thread
import numpy as np
import src.config as config


class CameraStream:
    """CCTV 카메라 스트림 (스레드 분리 + 워치독 재연결)."""

    def __init__(self, camera_id=0):
        self.camera_id       = camera_id
        self.active_source   = None
        self.stream          = None
        self.frame           = None
        self.grabbed         = False
        self.stopped         = False
        self.max_retries     = 5
        self.last_frame_time = time.time()
        self._miss_count     = 0
        self._connect()

    def _candidate_sources(self):
        if self.camera_id not in (None, "auto", "AUTO"):
            return [self.camera_id]

        candidates = []
        candidates.extend(sorted(glob.glob("/dev/v4l/by-id/*Camera*video-index0")))
        candidates.extend(sorted(glob.glob("/dev/v4l/by-path/*video-index0")))
        candidates.extend(sorted(glob.glob("/dev/video*")))
        candidates.extend([0, 1, 2, 3])

        seen = set()
        deduped = []
        for item in candidates:
            key = str(item)
            if key in seen:
                continue
            seen.add(key)
            deduped.append(item)
        return deduped

    def _open_source(self, source):
        if isinstance(source, str) and not source.isdigit():
            return cv2.VideoCapture(source, cv2.CAP_V4L2)
        return cv2.VideoCapture(int(source), cv2.CAP_V4L2)

    def _control_device(self):
        source = self.active_source if self.active_source is not None else self.camera_id
        if isinstance(source, int):
            return f"/dev/video{source}"
        if isinstance(source, str):
            if source.isdigit():
                return f"/dev/video{source}"
            return source
        return None

    def _run_v4l2(self, args):
        device = self._control_device()
        if not device:
            return None
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", device] + list(args),
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=1.5,
            )
        except Exception as exc:
            print(f"[CAMERA] glare control failed: {exc}")
            return None
        if result.returncode != 0:
            msg = result.stdout.strip().replace("\n", " | ")
            print(f"[CAMERA] glare control failed: {msg}")
            return None
        return result.stdout

    def _get_control(self, name, default=None):
        out = self._run_v4l2([f"--get-ctrl={name}"])
        if not out:
            return default
        match = re.search(r":\s*(-?\d+)", out)
        if not match:
            return default
        return int(match.group(1))

    def _set_controls(self, controls):
        ctrl_arg = ",".join(f"{name}={int(value)}" for name, value in controls.items())
        return self._run_v4l2([f"--set-ctrl={ctrl_arg}"]) is not None

    def _set_control_if_supported(self, name, value):
        return self._set_controls({name: value})

    def _apply_startup_settings(self):
        if not bool(getattr(config, "CAMERA_APPLY_SETTINGS", False)):
            return
        ordered_controls = [
            ("auto_exposure", config.CAMERA_AUTO_EXPOSURE),
            ("exposure_time_absolute", config.CAMERA_EXPOSURE),
            ("gain", config.CAMERA_GAIN),
            ("brightness", config.CAMERA_BRIGHTNESS),
            ("contrast", config.CAMERA_CONTRAST),
            ("saturation", config.CAMERA_SATURATION),
            ("sharpness", config.CAMERA_SHARPNESS),
            ("backlight_compensation", config.CAMERA_BACKLIGHT),
            ("gamma", config.CAMERA_GAMMA),
        ]
        applied = []
        for name, value in ordered_controls:
            if self._set_control_if_supported(name, value):
                applied.append(f"{name}={int(value)}")
        self._set_stream_prop("CAP_PROP_AUTO_EXPOSURE", config.CAMERA_AUTO_EXPOSURE)
        self._set_stream_prop("CAP_PROP_EXPOSURE", config.CAMERA_EXPOSURE)
        self._set_stream_prop("CAP_PROP_GAIN", config.CAMERA_GAIN)
        self._set_stream_prop("CAP_PROP_BRIGHTNESS", config.CAMERA_BRIGHTNESS)
        self._set_stream_prop("CAP_PROP_CONTRAST", config.CAMERA_CONTRAST)
        if applied:
            print(f"[CAMERA] startup settings {' '.join(applied)}")
            print(f"[CAMERA] startup status {self._status_text()}")

    def _set_stream_prop(self, prop_name, value):
        if self.stream is None:
            return False
        prop = getattr(cv2, prop_name, None)
        if prop is None:
            return False
        try:
            return bool(self.stream.set(prop, float(value)))
        except Exception:
            return False

    def _status_text(self):
        auto_exp = self._get_control("auto_exposure", -1)
        exposure = self._get_control(
            "exposure_time_absolute",
            config.CAMERA_GLARE_RESET_EXPOSURE,
        )
        gain = self._get_control("gain", config.CAMERA_GLARE_RESET_GAIN)
        backlight = self._get_control(
            "backlight_compensation",
            config.CAMERA_GLARE_RESET_BACKLIGHT,
        )
        mode = "manual" if auto_exp == 1 else "auto" if auto_exp == 3 else str(auto_exp)
        return (
            f"auto={mode} exposure={exposure} "
            f"gain={gain} backlight={backlight}"
        )

    def adjust_glare(self, action):
        """Runtime keypad controls for glare-only UVC settings."""
        if action in ("exposure_down", "exposure_up"):
            current = self._get_control(
                "exposure_time_absolute",
                config.CAMERA_GLARE_RESET_EXPOSURE,
            )
            direction = -1 if action == "exposure_down" else 1
            value = current + direction * config.CAMERA_GLARE_EXPOSURE_STEP
            value = max(
                config.CAMERA_GLARE_EXPOSURE_MIN,
                min(config.CAMERA_GLARE_EXPOSURE_MAX, value),
            )
            if self._set_controls({
                    "auto_exposure": 1,
                    "exposure_time_absolute": value,
            }):
                self._set_stream_prop("CAP_PROP_AUTO_EXPOSURE", 1)
                self._set_stream_prop("CAP_PROP_EXPOSURE", value)
                print(f"[CAMERA] glare {self._status_text()}")
            return

        if action in ("gain_down", "gain_up"):
            current = self._get_control("gain", config.CAMERA_GLARE_RESET_GAIN)
            direction = -1 if action == "gain_down" else 1
            value = current + direction * config.CAMERA_GLARE_GAIN_STEP
            value = max(
                config.CAMERA_GLARE_GAIN_MIN,
                min(config.CAMERA_GLARE_GAIN_MAX, value),
            )
            if self._set_controls({"gain": value}):
                self._set_stream_prop("CAP_PROP_GAIN", value)
                print(f"[CAMERA] glare {self._status_text()}")
            return

        if action == "backlight_toggle":
            current = self._get_control(
                "backlight_compensation",
                config.CAMERA_GLARE_RESET_BACKLIGHT,
            )
            value = 0 if current else 1
            if self._set_controls({"backlight_compensation": value}):
                self._set_stream_prop("CAP_PROP_BACKLIGHT", value)
                print(f"[CAMERA] glare {self._status_text()}")
            return

        if action == "reset":
            # Return only glare-related controls to the camera defaults.
            if self._set_controls({
                    "gain": config.CAMERA_GLARE_RESET_GAIN,
                    "backlight_compensation": config.CAMERA_GLARE_RESET_BACKLIGHT,
                    "auto_exposure": 3,
            }):
                self._set_controls({
                    "auto_exposure": 1,
                    "exposure_time_absolute": config.CAMERA_GLARE_RESET_EXPOSURE,
                })
                self._set_controls({"auto_exposure": 3})
                self._set_stream_prop("CAP_PROP_GAIN", config.CAMERA_GLARE_RESET_GAIN)
                self._set_stream_prop(
                    "CAP_PROP_BACKLIGHT",
                    config.CAMERA_GLARE_RESET_BACKLIGHT,
                )
                self._set_stream_prop("CAP_PROP_EXPOSURE", config.CAMERA_GLARE_RESET_EXPOSURE)
                self._set_stream_prop("CAP_PROP_AUTO_EXPOSURE", 3)
                print(
                    f"[CAMERA] glare reset {self._status_text()}"
                )

    def _connect(self):
        for i in range(self.max_retries):
            if self.stream is not None:
                try:
                    self.stream.release()
                except Exception:
                    pass
            for source in self._candidate_sources():
                print(f"[INFO] 카메라 {source} 연결 시도 중... ({i+1}/{self.max_retries})")
                stream = self._open_source(source)
                if stream.isOpened():
                    if config.CAMERA_FOURCC:
                        fourcc = cv2.VideoWriter_fourcc(*config.CAMERA_FOURCC)
                        stream.set(cv2.CAP_PROP_FOURCC, fourcc)
                    stream.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
                    stream.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
                    stream.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)
                    stream.set(cv2.CAP_PROP_BUFFERSIZE,   1)
                    grabbed, frame = stream.read()
                    if grabbed and frame is not None:
                        self.stream = stream
                        self.frame = frame
                        self.grabbed = True
                        self.active_source = source
                        self._miss_count = 0
                        self._apply_startup_settings()
                        h, w = self.frame.shape[:2]
                        actual_fps = stream.get(cv2.CAP_PROP_FPS)
                        actual_fourcc = int(stream.get(cv2.CAP_PROP_FOURCC))
                        actual_fourcc = "".join(
                            chr((actual_fourcc >> 8 * j) & 0xFF)
                            for j in range(4)
                        ).strip()
                        print(
                            f"[INFO] 카메라 연결 성공 source={source} "
                            f"({w}x{h}) fps={actual_fps:.1f} "
                            f"fourcc={actual_fourcc or 'unknown'}"
                        )
                        self.last_frame_time = time.time()
                        return True
                try:
                    stream.release()
                except Exception:
                    pass
            print(f"[WARN] 카메라 {self.camera_id} 연결 실패. 1.5초 후 재시도...")
            time.sleep(1.5)
        print(f"[ERROR] 카메라 {self.camera_id} 최종 연결 실패.")
        return False

    def start(self):
        Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            if (self.stream is None or not self.stream.isOpened()
                    or (time.time() - self.last_frame_time > 3.0)):
                if not self.stopped:
                    print("[WARN] 카메라 응답 없음 → 재연결...")
                    self._connect()
                    self.last_frame_time = time.time()
                time.sleep(1.0)
                continue
            grabbed, frame = self.stream.read()
            if grabbed and frame is not None:
                self.frame           = frame
                self.grabbed         = True
                self.last_frame_time = time.time()
                self._miss_count     = 0
            else:
                self.grabbed = False
                self._miss_count += 1
                if self._miss_count < 30:
                    time.sleep(0.02)
                    continue
            sleep_sec = float(getattr(config, "CAMERA_THREAD_SLEEP_SEC", 0.001))
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)

    def read(self):
        if self.frame is None:
            return np.zeros((config.CAMERA_HEIGHT, config.CAMERA_WIDTH, 3), np.uint8)
        frame = self.frame.copy()
        if getattr(config, "CAMERA_ROTATE_180", False):
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        if getattr(config, "CAMERA_FLIP_VERTICAL", False):
            frame = cv2.flip(frame, 0)
        if getattr(config, "CAMERA_FLIP_HORIZONTAL", False):
            frame = cv2.flip(frame, 1)
        return frame

    def stop(self):
        self.stopped = True
        if self.stream and self.stream.isOpened():
            self.stream.release()
