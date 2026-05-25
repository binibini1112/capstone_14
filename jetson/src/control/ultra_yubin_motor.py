import os
import socket
import threading
import time


def _get_config_value(name, default):
    try:
        import src.config as config
        return getattr(config, name, default)
    except Exception:
        return default


def _clamp_tick(value):
    return max(0, min(4095, int(round(float(value)))))


def _laser_image_offset_ticks(cy, frame_h):
    frame_h = max(1, int(frame_h))
    fov_deg = float(_get_config_value("ULTRA_CHAN_LASER_VERTICAL_FOV_DEG", 43.0))
    err_y = int(cy) - (frame_h // 2)
    return int((-err_y * fov_deg * 1024.0) / (90.0 * float(frame_h)))


class UltraYubinMotorController:
    """MotorController-compatible adapter for the ultra_yubin architecture.

    The Jetson keeps running YOLO and audio detection. Instead of writing to a
    local U2D2, this adapter sends bbox/audio cues to Ultra96 PS. Ultra96 writes
    them into PL, reads the PL-computed pan/tilt goal, then sends Dynamixel
    commands through its own USB/U2D2 port.
    """

    def __init__(self):
        self.host = os.getenv("ULTRA_CHAN_HOST", os.getenv("ULTRA_YUBIN_HOST", "192.168.3.1"))
        self.port = int(os.getenv("ULTRA_CHAN_PORT", os.getenv("ULTRA_YUBIN_PORT", "5016")))
        self.timeout = float(os.getenv("ULTRA_CHAN_TIMEOUT_SEC", os.getenv("ULTRA_YUBIN_TIMEOUT_SEC", "0.08")))
        self.default_conf = int(os.getenv("ULTRA_CHAN_DEFAULT_CONF", os.getenv("ULTRA_YUBIN_DEFAULT_CONF", str(_get_config_value("ULTRA_CHAN_DEFAULT_CONF", 1000)))))
        self.default_bbox_w = int(os.getenv("ULTRA_CHAN_DEFAULT_BBOX_W", os.getenv("ULTRA_YUBIN_DEFAULT_BBOX_W", str(_get_config_value("ULTRA_CHAN_DEFAULT_BBOX_W", 0)))))
        self.default_bbox_h = int(os.getenv("ULTRA_CHAN_DEFAULT_BBOX_H", os.getenv("ULTRA_YUBIN_DEFAULT_BBOX_H", str(_get_config_value("ULTRA_CHAN_DEFAULT_BBOX_H", 0)))))
        self.control_period_sec = float(os.getenv("ULTRA_CHAN_CONTROL_PERIOD_SEC", os.getenv("ULTRA_YUBIN_CONTROL_PERIOD_SEC", str(_get_config_value("ULTRA_CHAN_CONTROL_PERIOD_SEC", 0.025)))))
        self.deadband_px = int(os.getenv("ULTRA_CHAN_DEADBAND_PX", os.getenv("ULTRA_YUBIN_DEADBAND_PX", str(_get_config_value("ULTRA_CHAN_DEADBAND_PX", 16)))))
        self.deadband_x_px = int(os.getenv("ULTRA_CHAN_DEADBAND_X_PX", os.getenv("ULTRA_YUBIN_DEADBAND_X_PX", str(self.deadband_px))))
        self.deadband_y_px = int(os.getenv("ULTRA_CHAN_DEADBAND_Y_PX", os.getenv("ULTRA_YUBIN_DEADBAND_Y_PX", str(self.deadband_px))))
        self.smooth_alpha = float(os.getenv("ULTRA_CHAN_SMOOTH_ALPHA", os.getenv("ULTRA_YUBIN_SMOOTH_ALPHA", str(_get_config_value("ULTRA_CHAN_SMOOTH_ALPHA", 0.38)))))
        self.smooth_alpha_x = float(os.getenv("ULTRA_CHAN_SMOOTH_ALPHA_X", os.getenv("ULTRA_YUBIN_SMOOTH_ALPHA_X", str(self.smooth_alpha))))
        self.smooth_alpha_y = float(os.getenv("ULTRA_CHAN_SMOOTH_ALPHA_Y", os.getenv("ULTRA_YUBIN_SMOOTH_ALPHA_Y", str(self.smooth_alpha))))
        self.aim_offset_x = int(os.getenv("ULTRA_CHAN_AIM_OFFSET_X", os.getenv("ULTRA_YUBIN_AIM_OFFSET_X", "0")))
        self.aim_offset_y = int(os.getenv("ULTRA_CHAN_AIM_OFFSET_Y", os.getenv("ULTRA_YUBIN_AIM_OFFSET_Y", "0")))
        self.invert_x_to_pl = os.getenv("ULTRA_CHAN_INVERT_X_TO_PL", os.getenv("ULTRA_YUBIN_INVERT_X_TO_PL", "1")) == "1"
        self.invert_y_to_pl = os.getenv("ULTRA_CHAN_INVERT_Y_TO_PL", os.getenv("ULTRA_YUBIN_INVERT_Y_TO_PL", "1")) == "1"
        self.center_on_start = os.getenv("ULTRA_CHAN_CENTER_ON_START", os.getenv("ULTRA_YUBIN_CENTER_ON_START", "1")) == "1"
        self.async_send = os.getenv("ULTRA_CHAN_ASYNC_SEND", os.getenv("ULTRA_YUBIN_ASYNC_SEND", "1" if _get_config_value("ULTRA_CHAN_ASYNC_SEND", True) else "0")) == "1"
        self.readpos_hz = max(
            0.0,
            float(os.getenv("ULTRA_CHAN_READPOS_HZ", os.getenv("ULTRA_YUBIN_READPOS_HZ", "5.0"))),
        )
        self.laser_id = int(os.getenv("ULTRA_CHAN_LASER_ID", os.getenv("ULTRA_YUBIN_LASER_ID", str(_get_config_value("ULTRA_CHAN_LASER_ID", 3)))))
        self.cur_pan = int(_get_config_value("PAN_CENTER", 2048))
        self.cur_tilt = int(os.getenv("ULTRA_CHAN_CENTER_TILT", os.getenv("ULTRA_YUBIN_CENTER_TILT", "2772")))
        self.pan_min = int(_get_config_value("PAN_MIN", 0))
        self.pan_max = int(_get_config_value("PAN_MAX", 4095))
        self.tilt_min = int(_get_config_value("TILT_MIN", 2340))
        self.tilt_max = int(_get_config_value("TILT_MAX", 3048))
        self.pan_dir = int(_get_config_value("PAN_DIR", 1))
        self.tilt_dir = int(_get_config_value("TILT_DIR", 1))
        self.ready = False
        self._command_lock = threading.Lock()
        self._last_control_sent = 0.0
        self._smooth_cx = None
        self._smooth_cy = None
        self._readpos_stop = threading.Event()
        self._readpos_thread = None
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(self.timeout)
        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_telemetry = {
            "ready": False,
            "mode": "ultra_yubin",
            "err_x": 0,
            "err_y": 0,
            "cmd_x": 0.0,
            "cmd_y": 0.0,
            "aim_cx": 0,
            "aim_cy": 0,
            "send_cx": 0,
            "send_cy": 0,
            "aim_offset_x": self.aim_offset_x,
            "aim_offset_y": self.aim_offset_y,
            "invert_x_to_pl": int(self.invert_x_to_pl),
            "invert_y_to_pl": int(self.invert_y_to_pl),
            "pan": self.cur_pan,
            "tilt": self.cur_tilt,
            "usb_ok": 0,
            "fpga_reply": "",
            "motor_ms": 0.0,
            "target_active": 0,
            "async_send": int(self.async_send),
            "distance_mm": None,
            "laser": None,
            "laser_base_tick": None,
            "present_pan": None,
            "present_tilt": None,
            "present_ms": 0.0,
            "present_timestamp": 0.0,
            "present_usb_ok": 0,
        }

    def _request(self, cmd):
        with self._command_lock:
            started = time.perf_counter()
            self._sock.sendto((cmd.rstrip() + "\n").encode("ascii"), (self.host, self.port))
            data, _ = self._sock.recvfrom(512)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
        return data.decode("ascii", errors="replace").strip(), elapsed_ms

    def _try_request(self, cmd):
        if not self._command_lock.acquire(blocking=False):
            return None, 0.0
        try:
            started = time.perf_counter()
            self._sock.sendto((cmd.rstrip() + "\n").encode("ascii"), (self.host, self.port))
            data, _ = self._sock.recvfrom(512)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            return data.decode("ascii", errors="replace").strip(), elapsed_ms
        finally:
            self._command_lock.release()

    def _send_async(self, cmd):
        with self._command_lock:
            started = time.perf_counter()
            self._tx_sock.sendto((cmd.rstrip() + "\n").encode("ascii"), (self.host, self.port))
        return (time.perf_counter() - started) * 1000.0

    def _parse_reply(self, reply):
        parsed = {}
        for item in reply.split(","):
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            parsed[key.strip()] = value.strip()
        return parsed

    def _update_from_reply(
        self,
        reply,
        elapsed_ms,
        err_x=0,
        err_y=0,
        active=1,
        tx_cmd="",
        send_cx=0,
        send_cy=0,
        aim_cx=0,
        aim_cy=0,
        distance_mm=None,
    ):
        parsed = self._parse_reply(reply)
        pan = int(parsed.get("pan", self.cur_pan))
        tilt = int(parsed.get("tilt", self.cur_tilt))
        laser = int(parsed.get("laser", "-1")) if parsed.get("laser", "-1").lstrip("-").isdigit() else None
        laser_base = int(parsed.get("laser_base", "-1")) if parsed.get("laser_base", "-1").lstrip("-").isdigit() else None
        self.cur_pan = pan
        self.cur_tilt = tilt
        reply_kind = reply.split(",", 1)[0] if reply else ""
        src = parsed.get("src", "")
        if parsed.get("ps_audio", "0") == "1":
            src = parsed.get("src", "audio_direct")
        self.last_telemetry.update({
            "ready": True,
            "mode": "ultra_yubin",
            "tx_cmd": tx_cmd,
            "rx_reply": reply,
            "reply_kind": reply_kind,
            "err_x": int(err_x),
            "err_y": int(err_y),
            "cmd_x": float(err_x),
            "cmd_y": float(err_y),
            "aim_cx": int(aim_cx),
            "aim_cy": int(aim_cy),
            "send_cx": int(send_cx),
            "send_cy": int(send_cy),
            "aim_offset_x": self.aim_offset_x,
            "aim_offset_y": self.aim_offset_y,
            "invert_x_to_pl": int(self.invert_x_to_pl),
            "invert_y_to_pl": int(self.invert_y_to_pl),
            "pan": pan,
            "tilt": tilt,
            "usb_ok": int(parsed.get("usb", "0")) if parsed.get("usb", "0").isdigit() else 0,
            "compute_count": int(parsed.get("count", "0")) if parsed.get("count", "0").isdigit() else 0,
            "fpga_reply": reply,
            "motor_ms": float(elapsed_ms),
            "target_active": int(active),
            "async_send": int(self.async_send),
            "distance_mm": distance_mm,
            "laser": laser,
            "laser_base_tick": laser_base,
            "src": src,
            "ctrl": parsed.get("ctrl", ""),
            "dry": parsed.get("dry", ""),
            "no_pl": parsed.get("no_pl", ""),
            "pan_limited": pan <= self.pan_min or pan >= self.pan_max,
            "tilt_limited": tilt <= self.tilt_min or tilt >= self.tilt_max,
        })
        return dict(self.last_telemetry)

    def _update_present_position(self, reply, elapsed_ms):
        parsed = self._parse_reply(reply)
        try:
            pan = int(parsed.get("pan", ""))
            tilt = int(parsed.get("tilt", ""))
        except ValueError:
            return
        self.cur_pan = pan
        self.cur_tilt = tilt
        self.last_telemetry.update({
            "ready": True,
            "present_pan": pan,
            "present_tilt": tilt,
            "present_ms": float(elapsed_ms),
            "present_timestamp": time.time(),
            "present_usb_ok": int(parsed.get("usb", "0")) if parsed.get("usb", "0").isdigit() else 0,
            "present_reply": reply,
            "present_source": "READPOS",
        })

    def _start_readpos_polling(self):
        if self.readpos_hz <= 0.0:
            return
        if self._readpos_thread is not None and self._readpos_thread.is_alive():
            return
        self._readpos_stop.clear()
        self._readpos_thread = threading.Thread(target=self._readpos_loop, daemon=True)
        self._readpos_thread.start()
        print(f"[ultra_yubin] READPOS polling {self.readpos_hz:.1f}Hz")

    def _readpos_loop(self):
        period = 1.0 / max(0.1, self.readpos_hz)
        while not self._readpos_stop.wait(period):
            if not self.ready:
                continue
            if time.perf_counter() - self._last_control_sent < self.control_period_sec:
                continue
            try:
                reply, elapsed_ms = self._try_request("READPOS")
                if reply:
                    self._update_present_position(reply, elapsed_ms)
            except Exception as exc:
                self.last_telemetry.update({
                    "present_usb_ok": 0,
                    "present_error": str(exc),
                    "present_timestamp": 0.0,
                })

    def set_laser_tick(self, tick):
        tick = max(0, min(4095, int(tick)))
        try:
            reply, elapsed_ms = self._request(f"D {self.laser_id} {tick}")
            parsed = self._parse_reply(reply)
            ok = parsed.get("usb", "0") == "1"
            goal = int(parsed.get("goal", tick)) if ok else tick
            self.last_telemetry.update({
                "ready": True,
                "tx_cmd": f"D {self.laser_id} {tick}",
                "rx_reply": reply,
                "reply_kind": reply.split(",", 1)[0] if reply else "",
                "fpga_reply": reply,
                "motor_ms": float(elapsed_ms),
                "usb_ok": int(ok),
                "laser": goal,
                "laser_base_tick": goal,
            })
            return ok, goal, reply
        except Exception as exc:
            reply = f"ERR,{exc}"
            self.last_telemetry.update({
                "ready": self.ready,
                "tx_cmd": f"D {self.laser_id} {tick}",
                "rx_reply": reply,
                "reply_kind": "ERR",
                "fpga_reply": reply,
                "usb_ok": 0,
            })
            return False, tick, reply

    def start(self):
        try:
            tx_cmd = "PLPING"
            reply, elapsed_ms = self._request(tx_cmd)
            if not (
                reply.startswith("PONG,PL,ULTRA_YUBIN")
                or reply.startswith("PONG,PL,ULTRA_CHAN")
            ):
                print(f"[ultra_yubin] unexpected PLPING reply: {reply}")
                return self
            self.ready = True
            self._update_from_reply(reply, elapsed_ms, active=0, tx_cmd=tx_cmd)
            print(f"[ultra_yubin] connected {self.host}:{self.port} {reply}")
            if self.center_on_start:
                center_reply, center_ms = self._request("CENTER")
                self._update_from_reply(center_reply, center_ms, active=0, tx_cmd="CENTER")
                print(f"[ultra_yubin] centered {center_reply}")
            self._start_readpos_polling()
        except Exception as exc:
            print(f"[ultra_yubin] connection failed host={self.host} port={self.port}: {exc}")
        self.last_telemetry["ready"] = self.ready
        return self

    def control(
        self,
        cx,
        cy,
        img_width=1280,
        img_height=720,
        bbox_width=None,
        bbox_height=None,
        distance_mm=None,
        laser_base_tick=None,
        laser_center_lock_tick=None,
        aim_center_x=None,
        aim_center_y=None,
    ):
        if not self.ready:
            self.last_telemetry["ready"] = False
            return dict(self.last_telemetry)

        img_width = max(1, int(img_width))
        img_height = max(1, int(img_height))
        cx = int(cx)
        cy = int(cy)
        bbox_width = self.default_bbox_w if bbox_width is None else max(0, int(bbox_width))
        bbox_height = self.default_bbox_h if bbox_height is None else max(0, int(bbox_height))
        aim_cx = (img_width // 2 if aim_center_x is None else int(aim_center_x)) + self.aim_offset_x
        aim_cy = (img_height // 2 if aim_center_y is None else int(aim_center_y)) + self.aim_offset_y
        err_x = cx - aim_cx
        err_y = cy - aim_cy
        quiet_x = abs(err_x) < self.deadband_x_px
        quiet_y = abs(err_y) < self.deadband_y_px
        if quiet_x and quiet_y:
            self.last_telemetry.update({
                "ready": True,
                "tx_cmd": "",
                "rx_reply": "SKIP,deadband",
                "reply_kind": "SKIP",
                "err_x": int(err_x),
                "err_y": int(err_y),
                "cmd_x": float(err_x),
                "cmd_y": float(err_y),
                "aim_cx": int(aim_cx),
                "aim_cy": int(aim_cy),
                "fpga_reply": "SKIP,deadband",
                "motor_ms": 0.0,
                "target_active": 1,
                "usb_ok": 0,
                "distance_mm": distance_mm,
                "laser_base_tick": laser_base_tick,
            })
            return dict(self.last_telemetry)

        now = time.perf_counter()
        if now - self._last_control_sent < self.control_period_sec:
            self.last_telemetry.update({
                "ready": True,
                "tx_cmd": "",
                "rx_reply": "SKIP,rate",
                "reply_kind": "SKIP",
                "err_x": int(err_x),
                "err_y": int(err_y),
                "cmd_x": float(err_x),
                "cmd_y": float(err_y),
                "aim_cx": int(aim_cx),
                "aim_cy": int(aim_cy),
                "fpga_reply": "SKIP,rate",
                "motor_ms": 0.0,
                "target_active": 1,
                "usb_ok": 0,
                "distance_mm": distance_mm,
                "laser_base_tick": laser_base_tick,
            })
            return dict(self.last_telemetry)

        input_cx = aim_cx if quiet_x else cx
        input_cy = aim_cy if quiet_y else cy
        if self._smooth_cx is None:
            self._smooth_cx = float(input_cx)
            self._smooth_cy = float(input_cy)
        else:
            alpha_x = max(0.0, min(1.0, self.smooth_alpha_x))
            alpha_y = max(0.0, min(1.0, self.smooth_alpha_y))
            self._smooth_cx = alpha_x * float(input_cx) + (1.0 - alpha_x) * self._smooth_cx
            self._smooth_cy = alpha_y * float(input_cy) + (1.0 - alpha_y) * self._smooth_cy
        send_cx = int(round(self._smooth_cx))
        send_cy = int(round(self._smooth_cy))
        control_cx = send_cx - self.aim_offset_x
        control_cy = send_cy - self.aim_offset_y
        if self.invert_x_to_pl:
            control_cx = img_width - control_cx
        if self.invert_y_to_pl:
            control_cy = img_height - control_cy
        control_cx = max(0, min(img_width, control_cx))
        control_cy = max(0, min(img_height, control_cy))
        dist_arg = 0 if distance_mm is None else max(0, int(distance_mm))
        if laser_center_lock_tick is not None:
            laser_img_ticks = _laser_image_offset_ticks(control_cy, img_height)
            laser_arg = _clamp_tick(int(laser_center_lock_tick) - laser_img_ticks)
        else:
            laser_img_ticks = None
            laser_arg = 0 if laser_base_tick is None else _clamp_tick(laser_base_tick)
        cmd = (
            f"T {control_cx} {control_cy} {bbox_width} {bbox_height} "
            f"{img_width} {img_height} {self.default_conf} 1 {dist_arg} {laser_arg}"
        )
        try:
            if self.async_send:
                elapsed_ms = self._send_async(cmd)
                self._last_control_sent = time.perf_counter()
                self.last_telemetry.update({
                    "ready": True,
                    "mode": "ultra_yubin",
                    "tx_cmd": cmd,
                    "rx_reply": "ASYNC,sent",
                    "reply_kind": "ASYNC",
                    "err_x": int(err_x),
                    "err_y": int(err_y),
                    "cmd_x": float(err_x),
                    "cmd_y": float(err_y),
                    "aim_cx": int(aim_cx),
                    "aim_cy": int(aim_cy),
                    "send_cx": int(control_cx),
                    "send_cy": int(control_cy),
                    "aim_offset_x": self.aim_offset_x,
                    "aim_offset_y": self.aim_offset_y,
                    "invert_x_to_pl": int(self.invert_x_to_pl),
                    "invert_y_to_pl": int(self.invert_y_to_pl),
                    "usb_ok": 1,
                    "fpga_reply": "ASYNC,sent",
                    "motor_ms": float(elapsed_ms),
                    "target_active": 1,
                    "src": "pl",
                    "async_send": 1,
                    "distance_mm": distance_mm,
                    "laser_base_tick": laser_base_tick,
                    "laser_center_lock_tick": laser_center_lock_tick,
                    "laser_img_ticks": laser_img_ticks,
                })
                return dict(self.last_telemetry)
            reply, elapsed_ms = self._request(cmd)
            self._last_control_sent = time.perf_counter()
            return self._update_from_reply(
                reply,
                elapsed_ms,
                err_x,
                err_y,
                active=1,
                tx_cmd=cmd,
                send_cx=control_cx,
                send_cy=control_cy,
                aim_cx=aim_cx,
                aim_cy=aim_cy,
                distance_mm=distance_mm,
            )
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({
                "ready": False,
                "fpga_reply": f"ERR,{exc}",
                "target_active": 0,
            })
            return dict(self.last_telemetry)

    def turn_to_doa(self, doa_angle):
        if not self.ready:
            self.last_telemetry["ready"] = False
            return dict(self.last_telemetry)
        angle = int(round(float(doa_angle)))
        try:
            cmd = f"A {angle} {self.default_conf} 1"
            reply, elapsed_ms = self._request(cmd)
            return self._update_from_reply(reply, elapsed_ms, 0, 0, active=0, tx_cmd=cmd)
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({"ready": False, "fpga_reply": f"ERR,{exc}"})
            return dict(self.last_telemetry)

    def manual_move(self, dx, dy):
        pan = max(self.pan_min, min(self.pan_max, self.cur_pan + int(dx) * self.pan_dir))
        tilt = max(self.tilt_min, min(self.tilt_max, self.cur_tilt + int(dy) * self.tilt_dir))
        return self._send_goal(pan, tilt)

    def center(self):
        if not self.ready:
            self.last_telemetry["ready"] = False
            return dict(self.last_telemetry)
        try:
            cmd = "CENTER"
            reply, elapsed_ms = self._request(cmd)
            return self._update_from_reply(reply, elapsed_ms, 0, 0, active=0, tx_cmd=cmd)
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({"ready": False, "fpga_reply": f"ERR,{exc}"})
            return dict(self.last_telemetry)

    def _send_goal(self, pan, tilt):
        if not self.ready:
            self.last_telemetry["ready"] = False
            return dict(self.last_telemetry)
        try:
            cmd = f"G {int(pan)} {int(tilt)}"
            reply, elapsed_ms = self._request(cmd)
            return self._update_from_reply(reply, elapsed_ms, 0, 0, active=0, tx_cmd=cmd)
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({"ready": False, "fpga_reply": f"ERR,{exc}"})
            return dict(self.last_telemetry)

    def read_status(self):
        if not self.ready:
            return dict(self.last_telemetry)
        try:
            tx_cmd = "PLPING"
            reply, elapsed_ms = self._request(tx_cmd)
            return self._update_from_reply(reply, elapsed_ms, 0, 0, active=0, tx_cmd=tx_cmd)
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({"ready": False, "fpga_reply": f"ERR,{exc}"})
            return dict(self.last_telemetry)

    def stop(self):
        self._readpos_stop.set()
        if self._readpos_thread is not None and self._readpos_thread.is_alive():
            self._readpos_thread.join(timeout=0.5)
        try:
            self._sock.close()
        except Exception:
            pass
        try:
            self._tx_sock.close()
        except Exception:
            pass


MotorController = UltraYubinMotorController
