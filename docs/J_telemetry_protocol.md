# Jetson to Raspberry Pi Telemetry Protocol

This document is the shared contract between the Jetson tracker and the Raspberry Pi Tello dashboard.

## Transport

- Direction: Jetson -> Raspberry Pi
- Protocol: UDP
- Default Raspberry Pi listen port: `5005`
- Encoding: UTF-8 JSON
- Send rate: 10-30Hz clamp, default 20Hz
- Packet loss is acceptable; each packet must be a complete latest-state snapshot.

Current Jetson sender configuration:

- CLI: `--jetson-sender-host`, `--jetson-sender-port`, `--jetson-sender-rate`
- Environment: `JETSON_SENDER_HOST`, `JETSON_SENDER_PORT`, `JETSON_SENDER_RATE_HZ`
- Host fallback: `JETSON_SENDER_HOST -> PI_IP -> RASPBERRY_PI_IP -> "192.168.0.7"`
- Port fallback: `JETSON_SENDER_PORT -> PI_PORT -> 5005`

Default network assumption:

```text
Jetson Orin Nano  ->  Raspberry Pi eth0 / secondary network
UDP JSON          ->  <pi-ip>:5005

Raspberry Pi wlan0 -> Tello Wi-Fi
```

The Raspberry Pi dashboard does not send Tello control commands based on this telemetry. The Pi drone controller can still react to `laser.hit_detected=true` in the CLI path and execute `flip_forward` followed by `land`.

## Required Fields

```json
{
  "timestamp": 1715600000.123,
  "target_found": true,
  "state": "TRACKING"
}
```

- `timestamp`: Unix time seconds from `time.time()`.
- `target_found`: boolean.
- `state`: tracking state string.

Allowed `state` values:

- `SCANNING`
- `DETECTED`
- `TRACKING`
- `LOCKED`
- `ENGAGED`
- `NEUTRALIZED`

The current Jetson main loop normally emits `SCANNING`, `DETECTED`, `TRACKING`, and `LOCKED`. `ENGAGED` and `NEUTRALIZED` are defined in the state model but are not entered by the current main loop. Lost target handling returns to `SCANNING`.

## Optional Fields

```json
{
  "timestamp": 1715600000.123,
  "frame_id": 10234,
  "fps": 24.8,
  "target_found": true,
  "confidence": 0.87,
  "bbox": {
    "x1": 420,
    "y1": 210,
    "x2": 520,
    "y2": 290,
    "cx": 470,
    "cy": 250,
    "w": 100,
    "h": 80
  },
  "frame": {
    "width": 1280,
    "height": 720
  },
  "error": {
    "x_px": -170,
    "y_px": -110,
    "x_norm": -0.266,
    "y_norm": -0.306
  },
  "ptz": {
    "pan_deg": 12.5,
    "tilt_deg": -4.2,
    "pan_cmd": 18,
    "tilt_cmd": -7
  },
  "audio": {
    "enabled": true,
    "drone_detected": false,
    "confidence": 0.62,
    "detected": false,
    "fallback_active": false,
    "status": "VISION_HOLD"
  },
  "ultra_ps": {
    "motor_deg": 90.0
  },
  "laser": {
    "armed": false,
    "fired": false,
    "shot_count": 0,
    "hit_detected": false,
    "dot": {
      "detected": false,
      "x": null,
      "y": null,
      "score": 0.0,
      "area": 0,
      "inside_bbox": false,
      "hit_count": 0,
      "hit_window": 5
    },
    "fire": {
      "active": false,
      "result": "idle",
      "id": 0,
      "hit_frames": 0,
      "sample_frames": 0,
      "window_sec": 1.0
    }
  },
  "state": "TRACKING"
}
```

## Representative Current Packets

Scanning without target:

```json
{
  "timestamp": 1715600000.123,
  "target_found": false,
  "state": "SCANNING",
  "frame": {"width": 1280, "height": 720},
  "frame_id": 12,
  "fps": 24.0,
  "ptz": {"pan_deg": 359.978021978022, "tilt_cmd": 2772, "pan_cmd": 2048},
  "ultra_ps": {"motor_deg": 359.978021978022},
  "audio": {"enabled": true, "drone_detected": true, "sector": "FRONT_LEFT", "sector_index": 1, "sector_count": 6, "target_motor_deg": 60.0, "confidence": 0.72, "detected": true, "fallback_active": true, "status": "AUDIO_SEARCH"},
  "laser": {
    "armed": false,
    "fired": false,
    "shot_count": 0,
    "hit_detected": false,
    "dot": {"detected": false, "x": null, "y": null, "score": 0.0, "area": 0, "inside_bbox": false, "hit_count": 0, "hit_window": 5},
    "fire": {"active": false, "result": "idle", "id": 0, "hit_frames": 0, "sample_frames": 0, "window_sec": 1.0}
  }
}
```

Tracking target:

```json
{
  "timestamp": 1715600001.123,
  "target_found": true,
  "state": "TRACKING",
  "frame": {"width": 1280, "height": 720},
  "frame_id": 48,
  "fps": 24.7,
  "confidence": 0.86,
  "bbox": {"x1": 650.0, "y1": 340.0, "x2": 730.0, "y2": 400.0, "cx": 690.0, "cy": 370.0, "w": 80.0, "h": 60.0},
  "error": {"x_px": 50.0, "y_px": 10.0, "x_norm": 0.078125, "y_norm": 0.027777777777777776},
  "ptz": {"pan_deg": 4.989010989010989, "tilt_cmd": 2772, "pan_cmd": 1934},
  "ultra_ps": {"motor_deg": 4.989010989010989},
  "audio": {"enabled": true, "drone_detected": false, "confidence": 0.72, "detected": false, "fallback_active": false, "status": "VISION_HOLD"},
  "laser": {
    "armed": true,
    "fired": false,
    "shot_count": 0,
    "hit_detected": false,
    "dot": {"detected": false, "x": null, "y": null, "score": 0.0, "area": 0, "inside_bbox": false, "hit_count": 0, "hit_window": 5},
    "fire": {"active": false, "result": "idle", "id": 0, "hit_frames": 0, "sample_frames": 0, "window_sec": 1.0}
  }
}
```

Locked/hit pulse:

```json
{
  "timestamp": 1715600002.123,
  "target_found": true,
  "state": "LOCKED",
  "frame": {"width": 1280, "height": 720},
  "frame_id": 96,
  "fps": 25.1,
  "confidence": 0.91,
  "bbox": {"x1": 600.0, "y1": 330.0, "x2": 680.0, "y2": 390.0, "cx": 640.0, "cy": 360.0, "w": 80.0, "h": 60.0},
  "error": {"x_px": 0.0, "y_px": 0.0, "x_norm": 0.0, "y_norm": 0.0},
  "ptz": {"pan_deg": 359.978021978022, "tilt_cmd": 2772, "pan_cmd": 2048},
  "ultra_ps": {"motor_deg": 359.978021978022},
  "audio": {"enabled": true, "drone_detected": false, "confidence": 0.65, "detected": false, "fallback_active": false, "status": "VISION_HOLD"},
  "laser": {
    "armed": false,
    "fired": false,
    "shot_count": 1,
    "hit_detected": true,
    "dot": {"detected": true, "x": 642, "y": 361, "score": 210.5, "area": 4, "inside_bbox": true, "hit_count": 1, "hit_window": 5},
    "fire": {"active": false, "result": "hit", "id": 1, "hit_frames": 1, "sample_frames": 3, "window_sec": 1.0}
  }
}
```

## Shot Count

The dashboard counts shots only when Jetson reports a fire action:

- Preferred: increment `laser.shot_count` every time the Jetson fire key/action runs.
- Alternative: send `laser.fired=true` for the fire event, then return it to `false`.

The dashboard treats the first received `laser.shot_count` as the session baseline and displays only increases after that baseline.

`laser.armed=true` currently means the laser output is active; it does not increase shots. `laser.hit_detected=true` increases hits, not shots.

## Hit Response

When `laser.hit_detected` changes from `false` to `true`, the default `./run_drone.sh` path on Raspberry Pi:

1. stops RC input
2. sends `flip_forward`
3. sends `land`

Jetson may keep `hit_detected=true` in subsequent packets for the same hit event. The Pi reacts only to the rising edge.

## Coordinate Contract

### Image Bounding Box

- `bbox.x1/y1/x2/y2`: image pixel coordinates.
- `bbox.cx/cy`: target center in pixels.
- `bbox.w/h`: bbox width and height in pixels.

### Error

The dashboard treats `error` as relative target offset from the current PTZ/motor center.

```text
frame_cx = frame.width / 2
frame_cy = frame.height / 2
x_px = bbox.cx - frame_cx
y_px = bbox.cy - frame_cy
x_norm = clamp(x_px / frame_cx, -1.0, 1.0)
y_norm = clamp(y_px / frame_cy, -1.0, 1.0)
```

- `x_norm = -1`: left edge
- `x_norm = 0`: motor/PTZ center
- `x_norm = 1`: right edge
- `y_norm = -1`: top edge
- `y_norm = 0`: motor/PTZ center
- `y_norm = 1`: bottom edge

The dashboard graph history uses:

- `fps`
- `confidence`
- `error.x_px`
- Tello battery percentage, when available
- telemetry receive rate in Hz, computed on the Raspberry Pi
- `audio.confidence`

The dashboard radar uses compass-like direction angles:

- north/up is `0` degrees
- west/left is `90` degrees
- south/down is `180` degrees
- east/right is `270` degrees

Displayed radar directions:

- motor/drone direction: `ultra_ps.motor_deg` preferred, with `ptz.pan_deg` as a dashboard fallback
- audio search target: `audio.sector` and `audio.target_motor_deg` only while `audio.fallback_active=true`

The radar does not display real distance in meters.

### PTZ

- `ptz.pan_deg`: current pan angle in degrees. Jetson derives this from Ultra96 PS `READPOS` present position when available.
- `ptz.tilt_deg`: current tilt angle in degrees.
- `ptz.pan_cmd`: pan tick used for telemetry. With `READPOS` active this is the present Dynamixel tick; otherwise it falls back to the last known command/goal tick.
- `ptz.tilt_cmd`: tilt tick used for telemetry. With `READPOS` active this is the present Dynamixel tick; otherwise it falls back to the last known command/goal tick.

### Audio

- `audio.enabled`: whether Jetson audio fallback was enabled at launch.
- `audio.drone_detected`: first-stage audio classifier result. True means the audio subsystem classified the sound as drone-like.
- `audio.sector`: second-stage 6-direction sector result. Recommended values: `FRONT`, `FRONT_LEFT`, `BACK_LEFT`, `BACK`, `BACK_RIGHT`, `FRONT_RIGHT`.
- `audio.sector_index`: zero-based sector index.
- `audio.sector_count`: currently `6`.
- `audio.target_motor_deg`: dashboard compass motor heading target for the selected sector. This must use the same coordinate convention as `ultra_ps.motor_deg`.
- `audio.confidence`: `0.0` to `1.0`.
- `audio.detected`: whether the latest audio direction is a valid hint.
- `audio.fallback_active`: true only while audio fallback is actively driving/searching the motor because vision has no target or lost the target.
- `audio.status`: short UI/debug status string. Recommended values: `OFF`, `IDLE`, `LISTENING`, `AUDIO_SEARCH`, `VISION_HOLD`.

Dashboard behavior:

- If `audio.fallback_active=true`, show Audio Assist and display `sector`, `target_motor_deg`, confidence, and motor alignment error.
- If vision reacquires a bbox and Jetson state returns to `DETECTED`, `TRACKING`, or `LOCKED`, Jetson should send `audio.fallback_active=false`; the dashboard hides audio search details and returns to Vision-focused display.
- `audio.target_motor_deg` and `ultra_ps.motor_deg` should converge while audio fallback is controlling the motor.
- Legacy `audio.direction_deg`, if present, is debug-only. The dashboard should not use it as the primary audio search direction for the 6-sector classifier.

### Ultra PS

- `ultra_ps.motor_deg`: motor pan/drone direction angle in degrees.

The current Pi telemetry payload sends only `ultra_ps.motor_deg` inside `ultra_ps`. It does not send `front_pan`, `pan_tick`, `motor_direction_deg`, `fan_deg`, `heading_deg`, `direction_deg`, `confidence`, `source`, or `timestamp`.

The dashboard may compute motor heading from `ultra_ps.pan_tick` or `ptz.pan_cmd` first, using the 360-degree wrap formula below. This keeps the Pi display correct when `ptz.pan_cmd` carries the fresh `READPOS` present pan tick. If no tick is available, the dashboard displays `ultra_ps.motor_deg` as-is, then falls back to `ptz.pan_deg`.

Jetson now polls Ultra96 PS `READPOS` in a separate background thread. Ultra96 PS reads the Dynamixel present position and returns the actual current pan/tilt ticks. The default polling rate is `5Hz`; set `ULTRA_CHAN_READPOS_HZ` or `ULTRA_YUBIN_READPOS_HZ` to change it, or `0` to disable it.

Telemetry uses fresh `READPOS` `present_pan`/`present_tilt` first. Fresh currently means the `READPOS` timestamp is within about `2s` of the telemetry packet timestamp. If a fresh present position is unavailable, Jetson falls back to the last known command/goal tick and uses the same 360-degree wrap calculation:

```text
motor_deg = ((front_pan - pan_tick) * 360 / 4096 * PAN_DIR) % 360
```

With default `front_pan=2048` and `PAN_DIR=1`: `2048 -> 0 deg/front`, `1024 -> 90 deg/left`, `0 -> 180 deg/back`, and `3072 -> 270 deg/right`. Do not treat the full tick range as only `-90..+90`; that compresses 360 degrees into 180 degrees and makes a rear-facing motor appear near left/right on the dashboard.

### Laser

- `laser.armed`: current laser output active state.
- `laser.fired`: short fire pulse after fire starts.
- `laser.shot_count`: cumulative fire action count.
- `laser.hit_detected`: short hit pulse after hit assessment, currently about `FIRE_HIT_PULSE_SEC`.
- `laser.dot`: laser spot detector debug/status object.
- `laser.fire`: fire assessment debug/status object.

Recommended user-facing UI should prioritize `shot_count`, `fired`, `hit_detected`, and `fire.result`. `dot.hit_count`, `dot.hit_window`, `fire.hit_frames`, `fire.sample_frames`, and `fire.window_sec` are debug details.

## Safety Contract

Telemetry is display/logging data only, except for the explicit hit response above.

Do not implement these behaviors on either side in v1:

- automatic Tello movement from bbox/error
- automatic target-following flight
- automatic laser/fire action
- movement based only on audio direction
- automatic emergency from tracking telemetry

Allowed in v1:

- status display
- logging
- operator decision support
- target lost warning
- low battery warning
- hit-confirmed `flip_forward` + `land`

## Jetson Sender Skeleton

```python
import json
import socket
import time


class TelemetrySender:
    def __init__(self, pi_ip: str, pi_port: int = 5005):
        self.addr = (pi_ip, pi_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, payload: dict) -> None:
        try:
            payload.setdefault("timestamp", time.time())
            data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.sock.sendto(data, self.addr)
        except OSError as exc:
            print(f"[telemetry] send failed: {exc}")

    def close(self) -> None:
        self.sock.close()


def clamp(value: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))
```

## Minimal Packet

```python
sender.send({
    "timestamp": time.time(),
    "target_found": True,
    "state": "TRACKING",
})
```

## Full Packet Build Example

```python
def build_payload(frame, target, fps, frame_id, state, pan_deg, tilt_deg):
    frame_h, frame_w = frame.shape[:2]
    payload = {
        "timestamp": time.time(),
        "frame_id": frame_id,
        "fps": fps,
        "target_found": target is not None,
        "state": state,
        "frame": {"width": frame_w, "height": frame_h},
        "ptz": {"pan_deg": pan_deg, "tilt_deg": tilt_deg},
    }

    if target is not None:
        x1, y1, x2, y2 = map(float, target.xyxy)
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        frame_cx = frame_w / 2.0
        frame_cy = frame_h / 2.0
        x_px = cx - frame_cx
        y_px = cy - frame_cy

        payload["confidence"] = float(target.confidence)
        payload["bbox"] = {
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
            "cx": cx,
            "cy": cy,
            "w": x2 - x1,
            "h": y2 - y1,
        }
        payload["error"] = {
            "x_px": x_px,
            "y_px": y_px,
            "x_norm": clamp(x_px / frame_cx) if frame_cx else 0.0,
            "y_norm": clamp(y_px / frame_cy) if frame_cy else 0.0,
        }

    return payload
```
