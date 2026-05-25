# Jetson -> Raspberry Pi Sender Contract

Jetson에서 Raspberry Pi 대시보드로 보내야 하는 UDP JSON 형식입니다.

## 목적

Jetson은 YOLO/tracking 결과와 방향 정보를 Raspberry Pi로 보냅니다. Raspberry Pi는 이 데이터를 대시보드 표시와 로그 저장에 쓰고, 기본 설정에서는 `laser.hit_detected=true` 상승 엣지를 받으면 `flip_forward` 뒤 `land`를 실행합니다.

자동 Tello 이동, 자동 발사, 자동 emergency는 사용하지 않습니다. 피격 반응만 예외적으로 허용합니다.

## 네트워크

```text
Jetson Orin Nano  ->  Raspberry Pi
UDP JSON          ->  <raspberry-pi-ip>:5005
```

기본값:

- Protocol: UDP
- Encoding: UTF-8 JSON
- Raspberry Pi listen port: `5005`
- Send rate: `10-30Hz`로 clamp, 기본 `20Hz`

현재 Jetson 설정 위치:

- CLI: `--jetson-sender-host`, `--jetson-sender-port`, `--jetson-sender-rate`
- 환경 변수: `JETSON_SENDER_HOST`, `JETSON_SENDER_PORT`, `JETSON_SENDER_RATE_HZ`
- host fallback: `JETSON_SENDER_HOST -> PI_IP -> RASPBERRY_PI_IP -> "192.168.0.7"`
- port fallback: `JETSON_SENDER_PORT -> PI_PORT -> 5005`

예시:

```text
Raspberry Pi dashboard output:
  Jetson UDP target candidates:
    <pi-ip-from-dashboard>:5005
Jetson sends to one of those candidates.
```

Jetson 자신의 IP가 따로 있어도, 패킷 목적지는 Raspberry Pi IP입니다.
캡스톤실, 체육관처럼 장소가 바뀌면 Raspberry Pi IP도 바뀔 수 있으므로,
Jetson sender 코드에 IP를 고정하지 말고 실행 시 CLI 옵션이나 위 환경 변수로 전달합니다.

## 최소 필수 패킷

아래 3개 필드는 반드시 포함해야 합니다.

```json
{
  "timestamp": 1715600000.123,
  "target_found": true,
  "state": "TRACKING"
}
```

필드 의미:

- `timestamp`: Jetson 기준 Unix time seconds, Python `time.time()`
- `target_found`: 타겟 검출 여부, boolean
- `state`: 추적 상태 문자열

허용 상태값:

- `SCANNING`
- `DETECTED`
- `TRACKING`
- `LOCKED`
- `ENGAGED`
- `NEUTRALIZED`

현재 main loop에서 일반적으로 송신되는 전이는 `SCANNING -> DETECTED -> TRACKING -> LOCKED`이며, lost 상황은 `SCANNING`으로 돌아갑니다. `ENGAGED`, `NEUTRALIZED`는 상태 머신에 정의되어 있지만 현재 자동 진입 경로는 없습니다.

## 발사 횟수

대시보드의 `Shots`는 Jetson에서 fire 키/액션이 실행된 횟수입니다. 현재 Jetson payload의 `laser.armed`는 이름과 달리 “무장 가능”이 아니라 laser output active 상태입니다. `laser.armed=true`는 shot으로 세지 않습니다. `laser.hit_detected=true`는 `Hits`만 올립니다.

권장 방식은 Jetson이 f 입력마다 `laser.shot_count`를 1씩 증가시켜 누적값으로 보내는 것입니다. UDP 패킷 손실이 있어도 누적값이면 대시보드가 최종 횟수를 맞출 수 있습니다.

Pi 대시보드는 처음 받은 `laser.shot_count`를 기준점으로 삼고, 현재 Pi 대시보드 실행 세션에서 증가한 양만 `Shots`에 표시합니다. 따라서 Jetson이 이전 실행의 누적값을 계속 보내도 Pi 대시보드는 새로 켜면 `Shots=0`에서 시작합니다.

```json
{
  "timestamp": 1715600000.123,
  "target_found": true,
  "state": "LOCKED",
  "laser": {
    "armed": true,
    "fired": true,
    "shot_count": 7,
    "hit_detected": false
  }
}
```

`shot_count`를 보내기 어렵다면 f 입력 순간에 `laser.fired=true`를 보낸 뒤 다음 패킷에서 `false`로 내려도 됩니다.

## 피격 이벤트

피격 판정은 `laser.hit_detected=true`로 보냅니다. Raspberry Pi는 이 값이 `false -> true`로 바뀌는 순간을 피격으로 보고, 기본 설정에서 다음 순서로 동작합니다.

1. RC 입력 정지
2. `flip_forward`
3. `land`

같은 피격 동안 `hit_detected=true`가 계속 반복되어도 Pi는 한 번만 반응합니다.

권장 피격 패킷:

```json
{
  "timestamp": 1715600000.123,
  "target_found": true,
  "state": "LOCKED",
  "laser": {
    "armed": true,
    "fired": false,
    "shot_count": 7,
    "hit_detected": true
  }
}
```

## 권장 전체 패킷

```json
{
  "timestamp": 1715600000.123,
  "frame_id": 1234,
  "target_found": true,
  "state": "TRACKING",
  "fps": 24.5,
  "confidence": 0.82,
  "frame": {
    "width": 1280,
    "height": 720
  },
  "bbox": {
    "x1": 520,
    "y1": 260,
    "x2": 620,
    "y2": 340,
    "cx": 570,
    "cy": 300,
    "w": 100,
    "h": 80
  },
  "error": {
    "x_px": -70,
    "y_px": -60,
    "x_norm": -0.109,
    "y_norm": -0.167
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
    "motor_deg": 92.0
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
  }
}
```

## 좌표 규칙

`bbox`는 이미지 픽셀 좌표입니다.

```text
x1, y1: bbox left-top
x2, y2: bbox right-bottom
cx, cy: bbox center
w, h: bbox width / height
```

`error`는 프레임 중심 대비 타겟 중심 오차입니다.

```text
frame_cx = frame.width / 2
frame_cy = frame.height / 2
x_px = bbox.cx - frame_cx
y_px = bbox.cy - frame_cy
x_norm = clamp(x_px / frame_cx, -1.0, 1.0)
y_norm = clamp(y_px / frame_cy, -1.0, 1.0)
```

의미:

- `x_norm = -1`: 왼쪽 끝
- `x_norm = 0`: 화면 중심
- `x_norm = 1`: 오른쪽 끝
- `y_norm = -1`: 위쪽 끝
- `y_norm = 0`: 화면 중심
- `y_norm = 1`: 아래쪽 끝

## 방향 / 오디오 규칙

현재 Pi telemetry payload에서 방향 관련 필드는 아래가 전부입니다.

- `ultra_ps.motor_deg`
- `ptz.pan_deg`
- `ptz.tilt_deg`
- `ptz.pan_cmd`
- `ptz.tilt_cmd`
- `audio.sector`
- `audio.target_motor_deg`

`ultra_ps`에는 현재 `motor_deg` 하나만 송신됩니다. `front_pan`, `pan_tick`, `motor_direction_deg`, `fan_deg`, `heading_deg`, `direction_deg`, `confidence`, `source`, `timestamp`는 현재 최종 Pi payload에 들어오지 않습니다.

Pi 대시보드의 motor heading 표시는 아래 순서만 신뢰합니다.

1. `ultra_ps.pan_tick` 또는 `ptz.pan_cmd`를 360도식으로 계산
2. `ultra_ps.motor_deg`
3. fallback: `ptz.pan_deg`

현재 Jetson의 `ultra_ps.motor_deg`는 Ultra96 PS `READPOS`로 읽은 Dynamixel present position을 우선 사용해서 계산됩니다. `READPOS`가 아직 없거나 오래된 경우에만 마지막 command/goal tick으로 fallback합니다. 현재 fresh 기준은 telemetry packet timestamp 대비 약 `2s` 이내입니다.

`READPOS` polling은 `jetson/src/control/ultra_yubin_motor.py`에서 별도 daemon thread로 실행됩니다. 기본값은 `5Hz`이며 `ULTRA_CHAN_READPOS_HZ` 또는 `ULTRA_YUBIN_READPOS_HZ`로 변경할 수 있고, `0`으로 끌 수 있습니다. polling은 command lock이 바쁘면 건너뛰므로 tracking loop가 직접 present position을 기다리지 않습니다.

`ptz.pan_cmd`와 `ptz.tilt_cmd`는 payload 호환을 위해 이름을 유지하지만, `READPOS`가 활성화되어 fresh하면 실제 present tick입니다. `READPOS`가 없을 때만 기존 마지막 command/goal tick입니다.

tick -> dashboard angle 계산식은 아래와 같습니다. 전체 pan tick 범위를 `-90..+90`이 아니라 `0..360` wrap으로 해석해야 합니다.

```python
motor_deg = ((front_pan - pan_tick) * 360.0 / 4096.0 * PAN_DIR) % 360.0
```

기본 `front_pan=2048`, `PAN_DIR=1` 기준:

- `pan_tick=2048` -> `0 deg` / 정면
- `pan_tick=1024` -> `90 deg` / 왼쪽
- `pan_tick=0` -> `180 deg` / 뒤
- `pan_tick=3072` -> `270 deg` / 오른쪽

이전 `center/half * 90` 방식은 전체 tick 범위를 180도로 압축해서 실제 모터가 뒤를 보고 있어도 대시보드에 오른쪽/왼쪽처럼 표시될 수 있으므로 쓰면 안 됩니다.

Pi 대시보드는 받은 `ultra_ps.motor_deg`를 그대로 표시합니다. `front_pan`과 `pan_tick`은 Jetson 내부 계산 기준이며 현재 Pi payload에는 넣지 않습니다.

단, `READPOS` 적용 이후 `ptz.pan_cmd`가 fresh present pan tick으로 들어오면 Pi UI는 `front_pan=2048` 기본값으로 위 360도식을 우선 적용해 표시할 수 있습니다. 이는 Jetson의 `ultra_ps.motor_deg`가 예전 `-90..+90` 압축식으로 남아 있을 때 대시보드 표시를 보정하기 위한 호환 경로입니다.

`audio`는 비전이 타겟 bbox를 잃었을 때만 모터 탐색을 보조하는 fallback 상태입니다. UI와 제어가 같은 기준을 보려면 Jetson은 오디오 결과를 “연속 상대각”보다 “6방향 섹터와 목표 모터각”으로 보내야 합니다.

```json
{
  "audio": {
    "enabled": true,
    "drone_detected": true,
    "sector": "FRONT_LEFT",
    "sector_index": 1,
    "sector_count": 6,
    "target_motor_deg": 60.0,
    "confidence": 0.62,
    "detected": true,
    "fallback_active": true,
    "status": "AUDIO_SEARCH"
  }
}
```

- `enabled`: Jetson 실행 인자 기준 audio fallback 기능 활성 여부
- `drone_detected`: 1차 오디오 분류 결과. 드론 소리로 판정되면 `true`
- `sector`: 2차 6방향 분류 결과. 권장값은 `FRONT`, `FRONT_LEFT`, `BACK_LEFT`, `BACK`, `BACK_RIGHT`, `FRONT_RIGHT`
- `sector_index`: 0부터 시작하는 섹터 번호
- `sector_count`: 현재는 `6`
- `target_motor_deg`: 해당 sector를 따라 모터가 바라봐야 할 dashboard compass 목표각. `ultra_ps.motor_deg`와 같은 좌표계여야 함
- `confidence`: 오디오 방향/섹터 신뢰도
- `detected`: 최신 audio sector를 유효한 힌트로 볼 수 있는지 여부
- `fallback_active`: 비전 target이 없거나 lost 상태라 audio direction이 실제 motor 탐색/회전에 쓰이는 동안만 `true`
- `status`: UI/debug용 짧은 문자열. 권장값은 `OFF`, `IDLE`, `LISTENING`, `AUDIO_SEARCH`, `VISION_HOLD`

Pi UI는 `fallback_active=true`일 때만 Audio Assist를 표시/강조합니다. bbox가 다시 잡혀 비전이 `DETECTED`, `TRACKING`, `LOCKED` 상태로 돌아오면 Jetson은 `fallback_active=false`, `status="VISION_HOLD"`를 보내고 Pi UI는 오디오 탐색 정보를 숨깁니다.

`target_motor_deg`는 모터 제어 목표이므로 `ultra_ps.motor_deg`와 같은 dashboard compass 기준이어야 합니다. 예를 들어 오디오가 `FRONT_LEFT` sector를 선택하고 목표가 `60 deg`라면, 모터가 회전한 뒤 `ultra_ps.motor_deg`도 `60 deg` 근처로 수렴해야 합니다.

기존 `audio.direction_deg`가 있다면 debug/legacy 값으로만 취급합니다. 6방향 분류 구조에서는 Pi UI 핵심 표시와 제어 확인에 `sector`와 `target_motor_deg`를 사용합니다.

Pi 레이더의 compass 규칙은 계속 아래와 같습니다.

- `0 deg`: 위쪽 / north
- `90 deg`: 왼쪽 / west
- `180 deg`: 아래쪽 / south
- `270 deg`: 오른쪽 / east

## Jetson Python 송신 예시

```python
import json
import os
import socket
import time


PI_IP = os.environ["PI_IP"]
PI_PORT = int(os.environ.get("PI_PORT", "5005"))


def clamp(value, lo=-1.0, hi=1.0):
    return max(lo, min(hi, value))


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

frame_id = 0

while True:
    frame_w = 1280
    frame_h = 720

    # TODO: replace these values with YOLO output.
    target_found = True
    x1, y1, x2, y2 = 520.0, 260.0, 620.0, 340.0
    confidence = 0.82

    payload = {
        "timestamp": time.time(),
        "frame_id": frame_id,
        "target_found": target_found,
        "state": "TRACKING" if target_found else "SCANNING",
        "fps": 24.5,
        "frame": {
            "width": frame_w,
            "height": frame_h,
        },
        "audio": {
            "enabled": True,
            "drone_detected": False,
            "confidence": 0.62,
            "detected": False,
            "fallback_active": False,
            "status": "VISION_HOLD",
        },
        "ultra_ps": {
            "motor_deg": 92.0,
        },
        "laser": {
            "armed": False,
            "fired": False,
            "shot_count": 0,
            "hit_detected": False,
            "dot": {
                "detected": False,
                "x": None,
                "y": None,
                "score": 0.0,
                "area": 0,
                "inside_bbox": False,
                "hit_count": 0,
                "hit_window": 5,
            },
            "fire": {
                "active": False,
                "result": "idle",
                "id": 0,
                "hit_frames": 0,
                "sample_frames": 0,
                "window_sec": 1.0,
            },
        },
    }

    if target_found:
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        frame_cx = frame_w / 2.0
        frame_cy = frame_h / 2.0
        x_px = cx - frame_cx
        y_px = cy - frame_cy

        payload["confidence"] = confidence
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
            "x_norm": clamp(x_px / frame_cx),
            "y_norm": clamp(y_px / frame_cy),
        }

    data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    sock.sendto(data, (PI_IP, PI_PORT))

    frame_id += 1
    time.sleep(0.05)
```

## Raspberry Pi에서 확인

Pi에서 dashboard 실행:

```bash
python -m tello_control.dashboard --host 0.0.0.0 --telemetry-host 0.0.0.0 --telemetry-port 5005
```

실행하면 현재 네트워크에서 Jetson이 보낼 목적지 후보를 출력합니다.

```text
Dashboard URLs:
  http://127.0.0.1:8000
  http://<pi-ip-from-dashboard>:8000
Jetson UDP target candidates:
  <pi-ip-from-dashboard>:5005
```

Jetson에서는 위 후보 중 Raspberry Pi와 같은 네트워크의 주소를 넣어 실행합니다.

```bash
PI_IP=<pi-ip-from-dashboard> PI_PORT=5005 python jetson_sender.py
```

현장별 ipTIME 고정 IP 참고값:

| 현장 | IP address | SN | GW | DNS | 보조 DNS |
| --- | --- | --- | --- | --- | --- |
| 220호 | `113.198.84.249` | `255.255.255.0` | `113.198.84.254` | `113.198.74.100` | `209.248.252.2` |
| 시현장 | `223.194.146.28` | `255.255.255.0` | `223.194.146.254` | `113.198.74.100` | `203.248.252.2` |

장소를 옮긴 뒤에는 ipTIME 리셋 버튼을 누르고 ipTIME 설치도우미에서 해당 현장의 고정 IP 주소를 다시 할당합니다. 이 값은 Pi 대시보드 실행 또는 CLI의 `--dashboard` 옵션 사용 시 참고용으로도 출력됩니다.

브라우저:

```text
http://<raspberry-pi-ip>:8000
```

Jetson 패킷이 들어오면 UI의 Jetson status가 `CONNECTED`가 됩니다.

`./run_drone.sh`는 기본으로 이 피격 반응을 켠 상태로 실행됩니다.

수신 상태 기준:

- `0.0~1.0s`: `CONNECTED`
- `1.0~3.0s`: `STALE`
- `3.0s+`: `DISCONNECTED`
