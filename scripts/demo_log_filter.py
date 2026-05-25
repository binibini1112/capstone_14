#!/usr/bin/env python3
"""Convert noisy runtime logs into concise demo-facing status lines."""

import re
import sys


ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

DROP_PATTERNS = [
    r"^\d{4}-\d{2}-\d{2} .*onnxruntime:",
    r"^\d{4}-\d{2}-\d{2} .* \[W:onnxruntime:",
    r"^\[\d{2}/\d{2}/\d{4}-.*\] \[TRT\]",
    r"^Loading .+ for TensorRT inference",
    r"^INFO: Created TensorFlow Lite",
    r"^/.*(site|dist)-packages/.*UserWarning:",
    r"^\s*(warnings\.warn|setattr\(self|return self\._float_to_str)",
    r"^Expression '.*' failed in 'src/hostapi/alsa/",
    r"^Corrupt JPEG data:",
    r"^\[MIC DEBUG\]",
    r"^tello_prob=",
    r"^ultra_yubin_v1\.(bit|hwh)\s+",
    r"^pl_udp_usb_dxl_bridge\.c\s+",
    r"^operating$",
    r"^\d+$",
    r"^=== ultra_yubin Bring-up Check ===$",
    r"^time: ",
    r"^target: ",
    r"^\[(PING|PLPING_|DIRECT_G|TRACK_T|AUDIO_A)",
    r"^summary: failures=0$",
    r"^log: benchmark_logs/ultra_yubin_bringup_",
]

drop_re = [re.compile(pattern) for pattern in DROP_PATTERNS]


def _pct(value: str) -> str:
    try:
        return f"{float(value) * 100.0:.1f}%"
    except Exception:
        return value


def transform(line: str) -> str | None:
    clean = ANSI_RE.sub("", line)
    if any(regex.search(clean) for regex in drop_re):
        return None

    if clean.startswith("[DEMO] clean terminal log enabled; full log="):
        log_path = clean.split("full log=", 1)[1]
        return f"[DEMO] 데모 로그 모드 ON | 전체 로그: {log_path}"

    if clean.startswith("[run_demo_pl_drive] FINAL "):
        return "[READY] 데모 프로파일 로드 완료: YOLO + ReSpeaker + Ultra96 팬틸트"

    match = re.search(r"target=([^ ]+) remote=.* port=([0-9]+)", clean)
    if clean.startswith("[ultra-yubin-v1] target=") and match:
        return f"[READY] Ultra96 연결 준비: {match.group(1)}:{match.group(2)}"

    if clean.startswith("[ultra-yubin-v1] service="):
        return "[READY] Ultra96 브리지 재시작 확인"

    if clean.startswith("[ultra_yubin] connected"):
        return "[READY] Ultra96 통신 OK"

    if clean.startswith("[ultra_yubin] centered"):
        return "[READY] 팬틸트 중앙 정렬 OK"

    if clean.startswith("[LASER]"):
        return "[READY] 레이저 GPIO OK"

    if clean.startswith("[JETSON-SENDER]"):
        return "[READY] 대시보드 텔레메트리 ON"

    if "카메라 연결 성공" in clean:
        size = re.search(r"\(([0-9]+x[0-9]+)\).*fps=([0-9.]+)", clean)
        if size:
            return f"[READY] 카메라 연결 OK: {size.group(1)} @ {size.group(2)}fps"
        return "[READY] 카메라 연결 OK"

    if clean.startswith("[VISION] Loading:"):
        return "[READY] YOLO 모델 로딩 중"

    if clean.startswith("[VISION] ✓ Loaded:") or clean.startswith("[VISION] Loaded:"):
        return "[READY] YOLO 모델 로딩 OK"

    if clean.startswith("[vision] TensorRT warmup complete"):
        return "[READY] TensorRT 워밍업 OK"

    if clean.startswith("[audio] preprocess=on"):
        band = re.search(r"bandpass=([^ ]+)", clean)
        if band:
            return f"[READY] 오디오 전처리 ON: {band.group(1)}"
        return "[READY] 오디오 전처리 ON"

    if clean.startswith("[audio] fallback=on"):
        threshold = re.search(r"threshold=([0-9.]+)", clean)
        if threshold:
            return f"[READY] 드론소리 감지 모델 ON: threshold={threshold.group(1)}"
        return "[READY] 드론소리 감지 모델 ON"

    if clean.startswith("[audio] paused while vision tracking"):
        return "[AUDIO] 대기: YOLO 추적 중"

    if clean.startswith("[audio] resumed after vision loss"):
        return "[AUDIO] 시작: 화면에서 드론 놓침"

    if clean.startswith("[audio] arecord stream") or clean.startswith("[audio] sounddevice stream"):
        device = re.search(r"device=([^ ]+)", clean)
        channels = re.search(r"channels=([0-9]+)", clean)
        if device and channels:
            return f"[READY] ReSpeaker 입력 OK: {device.group(1)}, {channels.group(1)}ch"
        return "[READY] ReSpeaker 입력 OK"

    if clean.startswith("[audio] ReSpeaker USB DOA enabled"):
        return "[READY] ReSpeaker 방향 추정 OK"

    state = re.search(r"^\[STATE\] ([A-Z_]+) .* ([A-Z_]+)\s+\(([^)]+)\)", clean)
    if state:
        names = {
            "SCANNING": "탐색",
            "DETECTED": "탐지",
            "TRACKING": "추적",
            "LOCKED": "조준",
        }
        src = names.get(state.group(1), state.group(1))
        dst = names.get(state.group(2), state.group(2))
        return f"[STATE] {src} -> {dst} ({state.group(3)})"

    detected = re.search(r"^\[DRONE DETECTED\] doa=([0-9.+-]+), prob=([0-9.]+)", clean)
    if detected:
        return f"[AUDIO] 드론 소리 감지: 방향={float(detected.group(1)):.0f} deg, 신뢰도={_pct(detected.group(2))}"

    motor = re.search(
        r"^\[AUDIO->MOTOR\] angle=([0-9.+-]+).*raw_doa=([0-9.+-]+).*corrected=([0-9.+-]+).*sector=([0-9.+-]+).*score=([0-9.]+)",
        clean,
    )
    if motor:
        return (
            f"[MOTOR] 소리 기반 팬 회전: 명령각={float(motor.group(1)):.1f} deg, "
            f"raw={float(motor.group(2)):.0f} deg, "
            f"보정섹터={float(motor.group(4)):.0f} deg, "
            f"신뢰도={_pct(motor.group(5))}"
        )

    if clean.startswith("[jetson] Ctrl+C received"):
        return "[STOP] 사용자 종료 요청"

    done = re.search(r"^\[jetson\] done frames=([0-9]+).*avg_fps=([0-9.]+)", clean)
    if done:
        return f"[STOP] 데모 종료: frames={done.group(1)}, avg_fps={done.group(2)}"

    if re.search(r"(ERROR|Error|error|failed|failed:|WARNING|Warning|Traceback|Exception|세그멘테이션|오류)", clean):
        return f"[WARN] {clean}"

    return None


def main() -> int:
    try:
        for line in sys.stdin:
            output = transform(line.rstrip("\n"))
            if output:
                sys.stdout.write(output + "\n")
                sys.stdout.flush()
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
