"""
vision_tracker.py — 드론 전용 YOLO 탐지기 (속도 최적화)
========================================================
FPS 향상 3가지:
  1. botsort → bytetrack (옵티컬 플로우 제거 → CPU 부하 대폭 감소)
  2. 입력 해상도 imgsz=320 (640 대비 4배 빠름)
  3. 프레임 스킵 (매 N프레임만 추론, 중간은 이전 결과 재사용)
"""
import os
import gc
import time
import torch
import numpy as np
from ultralytics import YOLO

MIN_BOX_HEIGHT = int(os.getenv("YOLO_MIN_BOX_HEIGHT", "4"))
TRACKER_CFG = os.getenv(
    "YOLO_TRACKER_CFG",
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "bytetrack_ultra_chan.yaml"),
)


def _config_default(name, default):
    try:
        import src.config as config
        return getattr(config, name, default)
    except Exception:
        return default


def _safe_cuda_empty_cache():
    if not torch.cuda.is_available():
        return
    try:
        torch.cuda.empty_cache()
    except RuntimeError as e:
        print(f"[VISION] CUDA cache cleanup skipped: {e}")


class VisionDetector:
    def __init__(self, engine_path=None, tracker='bytetrack'):
        # ── CUDA 정리 ──
        # Jetson에서는 dummy tensor 워밍업도 NvMap OOM을 만들 수 있어서
        # 실제 모델 로딩 전 별도 cuda tensor 할당은 하지 않는다.
        gc.collect()
        _safe_cuda_empty_cache()

        self.tracker_type = tracker
        self.conf_thr = float(os.getenv("YOLO_CONF", str(_config_default("YOLO_CONF", 0.18))))
        self.infer_img_size = int(os.getenv("YOLO_IMGSZ", "640"))
        self.skip_frames = max(1, int(os.getenv("YOLO_SKIP_FRAMES", "2")))
        self.fast_detect = os.getenv("YOLO_FAST_DETECT", "0") == "1"
        self._tracker_cfg = TRACKER_CFG if os.path.exists(TRACKER_CFG) else "bytetrack.yaml"
        try:
            torch.backends.cudnn.benchmark = True
        except Exception:
            pass

        # ── 모델 탐색 ──
        _project = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))  # antidrone_yubin/
        default_candidates = [
            os.path.join(_project, "models", "drone_best_final_0520.engine"),
            os.path.join(_project, "models", "drone_best_final_0520.pt"),
        ]
        candidates = []
        if engine_path:
            candidates.append(engine_path)
        for path in default_candidates:
            if path not in candidates:
                candidates.append(path)

        self.model = None
        self._model_name = "unknown"

        for path in candidates:
            real = os.path.realpath(path)
            if not os.path.exists(real):
                continue
            name = os.path.basename(path)
            try:
                print(f"[VISION] Loading: {name} ...")
                gc.collect()
                _safe_cuda_empty_cache()
                self.model = YOLO(path, task="detect")
                self._model_name = name
                if name.endswith(".engine"):
                    engine_imgsz = int(os.getenv("YOLO_ENGINE_IMGSZ", "640"))
                    if self.infer_img_size != engine_imgsz:
                        print(
                            f"[VISION] TensorRT engine input is fixed at {engine_imgsz}; "
                            f"using imgsz={engine_imgsz} instead of {self.infer_img_size}"
                        )
                        self.infer_img_size = engine_imgsz
                print(f"[VISION] ✓ Loaded: {name}")
                break
            except Exception as e:
                print(f"[VISION] ✗ Failed ({name}): {e}")
                self.model = None
                gc.collect()
                _safe_cuda_empty_cache()

        if self.model is None:
            raise RuntimeError("[VISION] 모델을 하나도 로드할 수 없습니다!")

        self._frame_count = 0
        self._last_result = None
        self._last_infer_ms = 0.0
        self._last_step_ms = 0.0
        self._last_track_was_skipped = False
        self._last_box_count = 0

        mode = "predict" if self.fast_detect else f"track:{tracker}"
        print(f"[VISION] Conf>={self.conf_thr} | imgsz={self.infer_img_size} | "
              f"Skip={self.skip_frames} | Mode={mode} cfg={self._tracker_cfg}")

    def track(self, frame, persist=True):
        self._frame_count += 1
        
        # ★ 프레임 스킵 부활: 130MB 대형 모델의 끔찍한 랙 방지 (2프레임당 1번만 추론)
        if self._frame_count % self.skip_frames != 0 and self._last_result is not None:
            self._last_track_was_skipped = True
            self._last_step_ms = 0.0
            return self._last_result

        started = time.perf_counter()
        common_kwargs = {
            "verbose": False,
            "conf": self.conf_thr,
            "device": os.getenv("YOLO_DEVICE", "cuda:0"),
            "imgsz": self.infer_img_size,
            "classes": [0],
        }
        if self.fast_detect:
            res = self.model.predict(frame, **common_kwargs)[0]
        else:
            res = self.model.track(
                frame,
                persist=persist,
                tracker=self._tracker_cfg,
                **common_kwargs,
            )[0]
        self._last_infer_ms = (time.perf_counter() - started) * 1000.0
        self._last_step_ms = self._last_infer_ms
        self._last_track_was_skipped = False
        
        filtered_res = self._filter_boxes(res)
        self._last_box_count = (
            0 if filtered_res.boxes is None else len(filtered_res.boxes)
        )
        self._last_result = filtered_res
        return filtered_res

    def _filter_boxes(self, res):
        if res.boxes is None or len(res.boxes) == 0:
            return res
        boxes = res.boxes.xyxy.cpu().numpy()
        keep = []
        for i, box in enumerate(boxes):
            h = float(box[3] - box[1])
            if h < MIN_BOX_HEIGHT:
                continue
            keep.append(i)

        if len(keep) == len(boxes):
            return res
        idx = torch.tensor(keep, dtype=torch.long)
        if len(idx) == 0:
            res.boxes = None
        else:
            res.boxes = res.boxes[idx]
        return res
