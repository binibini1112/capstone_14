"""Minimal demo UI: camera, reticle, drone bbox, and audio detection only."""
import time

import cv2

from src.core.state import SystemState
import src.config as config


WHITE = (245, 245, 245)
MUTED = (150, 155, 160)
GREEN = (80, 230, 90)
AMBER = (40, 190, 255)
RED = (60, 70, 255)
CYAN = (230, 190, 70)
BLACK = (0, 0, 0)
FONT_MAIN = cv2.FONT_HERSHEY_SIMPLEX


class AntiDroneDisplay:
    """Small, fast overlay for the live camera feed."""

    def __init__(self, state):
        self.state = state
        self.window_name = "ULTRA YUBIN V1 CCTV"
        self.ui_buttons = {}
        self._window_created = False
        self._last_clicked = None
        self.line_type = cv2.LINE_8 if getattr(config, "UI_FAST_DRAW", False) else cv2.LINE_AA

    def _ensure_window(self):
        if self._window_created:
            return
        flags = cv2.WINDOW_NORMAL if config.UI_FULLSCREEN else cv2.WINDOW_AUTOSIZE
        try:
            cv2.namedWindow(self.window_name, flags)
            if config.UI_FULLSCREEN:
                cv2.setWindowProperty(
                    self.window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN,
                )
            cv2.setMouseCallback(self.window_name, self._mouse_cb)
        except cv2.error:
            pass
        self._window_created = True

    def draw(self, frame, bboxes, fps, threat_info, decision_maker,
             fire_status=None, motor_info=None):
        self._ensure_window()
        self._draw_overlay(frame, bboxes, fps, motor_info, fire_status)
        cv2.imshow(self.window_name, frame)
        return frame

    def draw_light(self, frame, bboxes, fps, threat_info,
                   fire_status=None, motor_info=None):
        return self.draw(frame, bboxes, fps, threat_info, None, fire_status, motor_info)

    def _draw_overlay(self, frame, bboxes, fps, motor_info, fire_status=None):
        h, w = frame.shape[:2]
        target = next((b for b in (bboxes or []) if b.get("is_target", False)), None)
        state = self.state.system_state
        target_visible = target is not None

        with self.state.lock:
            audio_detected = bool(self.state.audio_detected)
            audio_status = str(self.state.audio_status)
            audio_score = float(self.state.audio_score)
            audio_rms = float(self.state.audio_rms)
            audio_doa = self.state.audio_doa
            audio_age = time.time() - self.state.audio_updated_at if self.state.audio_updated_at else 999.0
            audio_last_detected_at = getattr(self.state, "audio_last_detected_at", 0.0)
            audio_last_detection_doa = getattr(self.state, "audio_last_detection_doa", None)
            audio_detection_age = time.time() - audio_last_detected_at if audio_last_detected_at else 999.0

        reticle_x = w // 2
        reticle_y = h // 2
        if target_visible and getattr(config, "UI_RETICLE_FOLLOW_TARGET", True):
            x1, y1, x2, y2 = [int(v) for v in target["box"]]
            reticle_x = (x1 + x2) // 2
            reticle_y = (y1 + y2) // 2
            if getattr(config, "UI_SHOW_CAMERA_CENTER", False):
                self._draw_camera_center(frame, w // 2, h // 2)

        reticle_color = GREEN if target_visible else AMBER if audio_detected and audio_age < 2.0 else MUTED
        self._draw_target_box(frame, target)
        self._draw_fire_bbox(frame, fire_status)
        self._draw_laser_dot(frame, fire_status)
        if getattr(config, "UI_SHOW_LASER_CENTER_MARKER", False):
            self._draw_laser_center_marker(frame, w // 2, h // 2)
        if getattr(config, "UI_SHOW_RETICLE", True):
            self._draw_reticle(frame, reticle_x, reticle_y, reticle_color)
        self._draw_top_status(
            frame,
            fps,
            state,
            target,
            audio_detected,
            audio_status,
            audio_score,
            audio_rms,
            audio_doa,
            audio_age,
            audio_detection_age,
            audio_last_detection_doa,
            motor_info=motor_info,
            fire_status=fire_status,
        )
        self._draw_motor_hint(frame, motor_info)

    def _draw_reticle(self, frame, cx, cy, color):
        gap = 14
        arm = 62
        cv2.line(frame, (cx - arm, cy), (cx - gap, cy), color, 2, self.line_type)
        cv2.line(frame, (cx + gap, cy), (cx + arm, cy), color, 2, self.line_type)
        cv2.line(frame, (cx, cy - arm), (cx, cy - gap), color, 2, self.line_type)
        cv2.line(frame, (cx, cy + gap), (cx, cy + arm), color, 2, self.line_type)
        cv2.circle(frame, (cx, cy), 4, color, -1, self.line_type)
        cv2.circle(frame, (cx, cy), 34, color, 1, self.line_type)

    def _draw_camera_center(self, frame, cx, cy):
        cv2.line(frame, (cx - 10, cy), (cx + 10, cy), MUTED, 1, self.line_type)
        cv2.line(frame, (cx, cy - 10), (cx, cy + 10), MUTED, 1, self.line_type)
        cv2.circle(frame, (cx, cy), 18, MUTED, 1, self.line_type)

    def _draw_laser_center_marker(self, frame, cx, cy):
        gap = 22
        arm = 20
        color = WHITE
        shadow = BLACK
        thickness = 2
        segments = [
            ((cx - gap - arm, cy - gap), (cx - gap, cy - gap)),
            ((cx - gap, cy - gap - arm), (cx - gap, cy - gap)),
            ((cx + gap, cy - gap), (cx + gap + arm, cy - gap)),
            ((cx + gap, cy - gap - arm), (cx + gap, cy - gap)),
            ((cx - gap - arm, cy + gap), (cx - gap, cy + gap)),
            ((cx - gap, cy + gap), (cx - gap, cy + gap + arm)),
            ((cx + gap, cy + gap), (cx + gap + arm, cy + gap)),
            ((cx + gap, cy + gap), (cx + gap, cy + gap + arm)),
        ]
        for start, end in segments:
            cv2.line(frame, start, end, shadow, thickness + 2, self.line_type)
        for start, end in segments:
            cv2.line(frame, start, end, color, thickness, self.line_type)

    def _draw_target_box(self, frame, target):
        if not target:
            return
        x1, y1, x2, y2 = [int(v) for v in target["box"]]
        conf = float(target.get("conf", 0.0))
        color = GREEN if conf >= 0.60 else AMBER
        self._corner_box(frame, x1, y1, x2, y2, color)
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        if getattr(config, "UI_SHOW_BBOX_CROSSHAIR", True):
            cv2.line(frame, (cx - 12, cy), (cx + 12, cy), color, 2, self.line_type)
            cv2.line(frame, (cx, cy - 12), (cx, cy + 12), color, 2, self.line_type)
        if getattr(config, "UI_SHOW_BBOX_LABEL", True):
            label = f"DRONE {max(0, min(100, int(round(conf * 100))))}%"
            self._label(frame, label, x1, max(26, y1 - 8), color)

    def _corner_box(self, frame, x1, y1, x2, y2, color):
        w = max(1, x2 - x1)
        h = max(1, y2 - y1)
        length = max(14, min(34, int(min(w, h) * 0.32)))
        thickness = 2
        segments = [
            ((x1, y1), (x1 + length, y1)),
            ((x1, y1), (x1, y1 + length)),
            ((x2, y1), (x2 - length, y1)),
            ((x2, y1), (x2, y1 + length)),
            ((x1, y2), (x1 + length, y2)),
            ((x1, y2), (x1, y2 - length)),
            ((x2, y2), (x2 - length, y2)),
            ((x2, y2), (x2, y2 - length)),
        ]
        for start, end in segments:
            cv2.line(frame, start, end, BLACK, thickness + 2, self.line_type)
        for start, end in segments:
            cv2.line(frame, start, end, color, thickness, self.line_type)

    def _draw_fire_bbox(self, frame, fire_status):
        if not fire_status or not fire_status.get("fire_active"):
            return
        box = fire_status.get("fire_bbox")
        if not box:
            laser_dot = fire_status.get("laser_dot") or {}
            box = laser_dot.get("hit_bbox")
        if not box:
            return
        try:
            x1, y1, x2, y2 = [int(v) for v in box]
        except (TypeError, ValueError):
            return
        cv2.rectangle(frame, (x1, y1), (x2, y2), AMBER, 2, self.line_type)

    def _draw_laser_dot(self, frame, fire_status):
        laser_dot = (fire_status or {}).get("laser_dot") if fire_status else None
        if not laser_dot or not laser_dot.get("detected"):
            return
        try:
            x = int(laser_dot.get("x"))
            y = int(laser_dot.get("y"))
        except (TypeError, ValueError):
            return
        h, w = frame.shape[:2]
        if x < 0 or y < 0 or x >= w or y >= h:
            return
        color = GREEN if laser_dot.get("inside_hit_bbox") else RED
        cv2.circle(frame, (x, y), 9, BLACK, 2, self.line_type)
        cv2.circle(frame, (x, y), 7, color, 2, self.line_type)
        cv2.circle(frame, (x, y), 2, color, -1, self.line_type)

    def _draw_top_status(self, frame, fps, state, target, audio_detected,
                         audio_status, audio_score, audio_rms, audio_doa, audio_age,
                         audio_detection_age=999.0, audio_last_detection_doa=None,
                         motor_info=None, fire_status=None):
        h, w = frame.shape[:2]
        cv2.rectangle(frame, (0, 0), (w, 40), BLACK, -1)
        vision_text = "LOCKED" if target else "SEARCHING"
        vision_color = GREEN if target else MUTED
        if state in (SystemState.TRACKING, SystemState.LOCKED):
            vision_text = state.value
            vision_color = GREEN if state == SystemState.LOCKED else CYAN

        cv2.putText(frame, "JETSON-FPGA DRONE TRACKER", (14, 26), FONT_MAIN, 0.50, WHITE, 1, self.line_type)
        cv2.putText(frame, f"VISION: {vision_text}", (310, 26), FONT_MAIN, 0.52, vision_color, 1, self.line_type)

        audio_fresh = audio_detection_age < float(getattr(config, "UI_AUDIO_HOLD_SEC", 5.0))
        audio_color = AMBER if audio_fresh else MUTED
        if audio_fresh:
            held_doa = audio_last_detection_doa if audio_last_detection_doa is not None else audio_doa
            doa_text = "" if held_doa is None else f" {held_doa:.0f}deg"
            audio_text = f"AUDIO: FOUND{doa_text}"
        else:
            audio_text = "AUDIO: STANDBY"
        (audio_tw, _), _ = cv2.getTextSize(audio_text, FONT_MAIN, 0.48, 1)
        audio_x = max(500, w - 124 - audio_tw)
        cv2.putText(frame, audio_text, (audio_x, 26), FONT_MAIN, 0.48, audio_color, 1, self.line_type)
        cv2.putText(frame, f"FPS {int(fps)}", (w - 88, 26), FONT_MAIN, 0.48, MUTED, 1, self.line_type)

        if target or state in (SystemState.TRACKING, SystemState.LOCKED):
            self._label(frame, "MODE: VISION TRACKING", 14, h - 18, GREEN)
        elif audio_fresh:
            self._label(frame, "MODE: AUDIO SEARCH", 14, h - 18, AMBER)
        distance_mm = (motor_info or {}).get("distance_mm") if motor_info else None
        if distance_mm not in (None, ""):
            try:
                rounded_m = round((float(distance_mm) / 1000.0) * 2.0) / 2.0
                self._label(frame, f"DIST {rounded_m:.1f}m", max(14, w - 150), h - 18, CYAN)
            except Exception:
                pass
        if (
            (fire_status or {}).get("hit_detected")
            or (fire_status or {}).get("fire_result") == "hit"
        ):
            self._center_label(frame, "HIT", h - 28, RED, scale=1.10, thickness=2)
        elif (fire_status or {}).get("fire_active"):
            self._center_label(frame, "FIRE CHECK", h - 28, AMBER, scale=0.92, thickness=2)
        elif (fire_status or {}).get("fire_result") == "miss":
            self._center_label(frame, "MISS", h - 28, MUTED, scale=0.92, thickness=2)

    def _draw_motor_hint(self, frame, motor_info):
        return

    def _center_label(self, frame, text, y, color, scale=0.90, thickness=2):
        (tw, _), _ = cv2.getTextSize(text, FONT_MAIN, scale, thickness)
        x = (frame.shape[1] - tw) // 2 - 12
        self._label(frame, text, x, y, color, scale=scale, thickness=thickness)

    def _label(self, frame, text, x, y, color, scale=0.54, thickness=1):
        (tw, th), _ = cv2.getTextSize(text, FONT_MAIN, scale, thickness)
        x = max(0, min(frame.shape[1] - tw - 16, int(x)))
        y = max(th + 8, min(frame.shape[0] - 8, int(y)))
        cv2.rectangle(frame, (x, y - th - 10), (x + tw + 18, y + 7), BLACK, -1)
        cv2.rectangle(frame, (x, y - th - 10), (x + tw + 18, y + 7), color, 1, self.line_type)
        cv2.putText(frame, text, (x + 9, y), FONT_MAIN, scale, color, thickness, self.line_type)

    def _mouse_cb(self, event, x, y, flags, param):
        return

    def get_clicked_button(self):
        clicked = self._last_clicked
        self._last_clicked = None
        return clicked
