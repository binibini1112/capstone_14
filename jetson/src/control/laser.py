import threading
import time
import os
import sys

_GPIO_AVAILABLE = False
_GPIO_IMPORT_ERROR = None
try:
    import Jetson.GPIO as GPIO
    _GPIO_AVAILABLE = True
except Exception as exc:
    _GPIO_IMPORT_ERROR = exc
    for site_dir in (
        "/usr/local/lib/python3.10/dist-packages",
        "/usr/lib/python3/dist-packages",
    ):
        if site_dir not in sys.path:
            sys.path.append(site_dir)
    try:
        import Jetson.GPIO as GPIO
        _GPIO_AVAILABLE = True
        _GPIO_IMPORT_ERROR = None
    except Exception as fallback_exc:
        GPIO = None
        _GPIO_IMPORT_ERROR = fallback_exc


class LaserController:
    def __init__(self, pin=7, enabled=True, pin_mode="BOARD", active_high=True):
        self.pin = int(pin)
        self.pin_mode = str(pin_mode or "BOARD").upper()
        self.active_high = bool(active_high)
        self.enabled = bool(enabled) and _GPIO_AVAILABLE
        self.active = False
        self._desired_active = False
        self._pattern_active = False
        self._fire_start = 0.0
        self._timer = None
        self._pattern_thread = None
        self._lock = threading.Lock()
        self.verbose = os.getenv("LASER_VERBOSE", "0").strip().lower() in ("1", "true", "yes", "on")

        if self.enabled:
            try:
                gpio_mode = GPIO.BOARD if self.pin_mode == "BOARD" else GPIO.BCM
                GPIO.setwarnings(True)
                GPIO.setmode(gpio_mode)
                GPIO.setup(self.pin, GPIO.OUT, initial=self._inactive_level())
                polarity = "active-high" if self.active_high else "active-low"
                print(f"[LASER] GPIO {self.pin_mode} pin {self.pin} initialized ({polarity})")
            except Exception as exc:
                print(f"[LASER] GPIO init failed: {exc} -> software-only mode")
                self.enabled = False
        else:
            if not _GPIO_AVAILABLE and _GPIO_IMPORT_ERROR is not None:
                mode = f"GPIO unavailable: {_GPIO_IMPORT_ERROR}"
            else:
                mode = "disabled in config"
            print(f"[LASER] Software-only mode ({mode})")

    def fire(self, duration=3.0):
        with self._lock:
            if self._timer and self._timer.is_alive():
                self._timer.cancel()
            self.active = True
            self._fire_start = time.time()
            self._write_gpio_locked(True)
            self._timer = threading.Timer(float(duration), self.cease_fire)
            self._timer.daemon = True
            self._timer.start()
        print(f"[LASER] FIRING ({duration:.1f}s)")

    def tick(self, duration=3.0):
        if not self.active:
            return False
        if time.time() - self._fire_start >= duration:
            self.cease_fire()
            return False
        return True

    def cease_fire(self):
        timer = None
        with self._lock:
            self.active = False
            self._write_gpio_locked(False)
            timer = self._timer
            self._timer = None
        if timer and timer.is_alive() and threading.current_thread() is not timer:
            timer.cancel()

    def set_active(self, active, reason="target"):
        active = bool(active)
        with self._lock:
            self._desired_active = active
            if self._pattern_active:
                return
            if self.active == active:
                return
            self.active = active
            self._write_gpio_locked(active)
        if self.verbose:
            print(f"[LASER] {'ON' if active else 'OFF'} ({reason})")

    def pattern(self, bits="110111", unit_sec=0.12, gap_sec=0.04, reason="manual-pattern"):
        bits = "".join(ch for ch in str(bits or "") if ch in "01")
        if not bits:
            return False
        unit_sec = max(0.02, float(unit_sec))
        gap_sec = max(0.0, float(gap_sec))

        def worker():
            try:
                print(f"[LASER] PATTERN {bits} ({reason})")
                for bit in bits:
                    with self._lock:
                        self.active = bit == "1"
                        self._write_gpio_locked(self.active)
                    time.sleep(unit_sec)
                    if gap_sec > 0:
                        with self._lock:
                            self.active = False
                            self._write_gpio_locked(False)
                        time.sleep(gap_sec)
            finally:
                with self._lock:
                    self._pattern_active = False
                    self.active = bool(self._desired_active)
                    self._write_gpio_locked(self.active)

        with self._lock:
            if self._pattern_thread and self._pattern_thread.is_alive():
                return False
            self._pattern_active = True
            self._pattern_thread = threading.Thread(target=worker, daemon=True)
            self._pattern_thread.start()
        return True

    def cleanup(self):
        self.cease_fire()
        if self.enabled:
            try:
                GPIO.cleanup(self.pin)
            except Exception:
                pass

    def status(self):
        return {
            "laser_available": bool(self.enabled),
            "laser_auto_enabled": True,
            "laser_output": bool(self.active),
            "laser_ready": bool(self.active),
            "laser_confirmed": bool(self.active),
            "laser_pattern_active": bool(self._pattern_active),
        }

    def _active_level(self):
        return GPIO.HIGH if self.active_high else GPIO.LOW

    def _inactive_level(self):
        return GPIO.LOW if self.active_high else GPIO.HIGH

    def _write_gpio_locked(self, active):
        if not self.enabled:
            return
        try:
            GPIO.output(self.pin, self._active_level() if active else self._inactive_level())
        except Exception as exc:
            level = "active" if active else "inactive"
            print(f"[LASER] GPIO {level} output failed: {exc}")
