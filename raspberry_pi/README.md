# Tello Control Dashboard

DJI Tello / Tello EDU 키보드 수동 조종과 Raspberry Pi 대시보드를 함께 실행하는 Python 프로젝트입니다.

## 먼저 실행할 것

1. Raspberry Pi/조종 컴퓨터를 Tello Wi-Fi에 연결합니다.
2. 터미널 1에서 대시보드:

```bash
./run_dashboard.sh
```

3. 터미널 2에서 드론 조종:

```bash
./run_drone.sh
```

`./run_drone.sh` 실행 중 `Command 'command' was unsuccessful` 또는 `Did not receive a response after 7 seconds`가 나오면 드론이 SDK 시작 명령에 응답하지 않는 상태입니다. Tello 전원이 켜져 있는지, 현재 장비가 Tello Wi-Fi에 붙어 있는지, `192.168.10.1:8889/udp`로 갈 수 있는지 확인합니다.

그냥 `python -m tello_control.dashboard`가 안 되면 현재 Python이 이 프로젝트의 `.venv` 또는 editable install을 쓰지 않는 상태입니다. 이 저장소는 `src` 레이아웃이라 시스템 Python(`/usr/bin/python`)에서는 `tello_control` 패키지를 바로 못 찾을 수 있습니다. 위 스크립트들은 `.venv/bin/python`으로 실행합니다.

대시보드는 Jetson Orin Nano에서 보내는 추적 telemetry와 PS 방향 데이터를 받아 브라우저에 표시하고 로그로 저장합니다. 현재 버전에서 이 데이터는 **표시와 기록 전용**입니다. Tello 자동 이동, 자동 추적 비행, 자동 발사, 자율 교전, tracking 데이터 기반 emergency 동작은 구현하지 않습니다.

## 전체 구조

```text
Jetson Orin Nano
  - YOLO / tracking
  - PTZ 상태
  - audio direction
  - PS / motor direction
        |
        | UDP JSON, default :5005
        v
Raspberry Pi
  - Tello keyboard controller
  - UDP telemetry receiver
  - FastAPI dashboard server
        |
        | HTTP + WebSocket, default :8000
        v
Browser dashboard
```

기본 네트워크 가정:

- Raspberry Pi `wlan0`: Tello Wi-Fi 연결
- Tello IP: `192.168.10.1`
- Tello SDK command port: UDP `8889`
- Raspberry Pi `eth0` 또는 별도 네트워크: Jetson telemetry 수신 및 dashboard 접속
- Jetson -> Raspberry Pi telemetry 기본 수신 포트: UDP `5005`
- Browser -> Raspberry Pi dashboard 기본 접속 포트: TCP `8000`

인터페이스 이름은 OS마다 다를 수 있으므로 코드는 특정 인터페이스에 고정하지 않습니다. 실행 옵션의 host/port로 조정합니다.

현장별 ipTIME 고정 IP 참고값:

| 현장 | IP address | SN | GW | DNS | 보조 DNS |
| --- | --- | --- | --- | --- | --- |
| 220호 | `113.198.84.249` | `255.255.255.0` | `113.198.84.254` | `113.198.74.100` | `209.248.252.2` |
| 시현장 | `223.194.146.28` | `255.255.255.0` | `223.194.146.254` | `113.198.74.100` | `203.248.252.2` |

장소를 옮긴 뒤에는 ipTIME 리셋 버튼을 누르고 ipTIME 설치도우미에서 해당 현장의 고정 IP 주소를 다시 할당합니다. 대시보드 실행 또는 CLI의 `--dashboard` 옵션 사용 시에도 이 표와 같은 값이 참고용으로 출력됩니다.

## 라즈베리에서 띄우는 정보

라즈베리 대시보드는 크게 세 종류의 정보를 보여줍니다.

| 구분 | 출처 | 어떻게 받나 | UI에서 쓰는 내용 |
| --- | --- | --- | --- |
| Tello 상태 | Raspberry Pi의 Tello controller | Tello SDK 명령/응답과 조종 상태 | 연결 여부, 이륙 여부, 배터리, 속도, 마지막 명령 |
| Jetson tracking | Jetson Orin Nano | Jetson이 Pi IP의 UDP `5005`로 JSON 전송 | 추적 상태, YOLO FPS, confidence, bbox/error, target found |
| PS / 방향 정보 | Jetson이 포함해서 보내는 `ultra_ps` payload | 같은 UDP JSON 안에 포함 | motor direction 또는 heading 방향을 원형 레이더에 표시 |

### Jetson이 보내는 것

Jetson은 매 프레임 또는 일정 주기마다 “현재 추적 상태 스냅샷”을 UDP JSON으로 보냅니다. 권장 전송 주기는 10-30Hz입니다.

필수 필드:

- `timestamp`: Jetson 기준 Unix time seconds
- `target_found`: 타겟 검출 여부
- `state`: `SCANNING`, `DETECTED`, `TRACKING`, `LOCKED`, `ENGAGED`, `NEUTRALIZED`, `LOST`

주요 선택 필드:

- `frame_id`: frame 번호
- `fps`: YOLO / tracking FPS
- `confidence`: 검출 신뢰도
- `bbox`: 타겟 bounding box 픽셀 좌표
- `frame`: 영상 width/height
- `error`: 화면 중심 대비 타겟 오차 (`x_px`, `y_px`, `x_norm`, `y_norm`)
- `ptz`: pan/tilt 각도와 명령값
- `audio`: 소리 방향과 confidence
- `ultra_ps`: PS / motor direction 계열 값
- `laser`: armed, fired, shot_count, hit_detected 상태

Jetson에서 `laser.hit_detected=true`를 보내면 Pi의 드론 제어가 기본으로 `flip_forward` 뒤 `land`를 실행합니다. 이 동작을 끄려면 `python -m tello_control.cli --no-auto-hit-response`를 사용합니다. 피격 시 모터를 즉시 끄는 자유낙하 반응은 `--hit-response free-fall`로 명시해서 켭니다.

대시보드의 `Shots`는 Pi 대시보드가 켜진 뒤 증가한 fire 횟수입니다. Jetson이 이전 실행의 누적 `laser.shot_count`를 보내도 첫 수신값은 기준점으로만 쓰고 화면은 `0`에서 시작합니다.

### PS 데이터가 의미하는 것

현재 Jetson -> Pi payload에서 PS는 `ultra_ps` JSON 객체로 받습니다. 현재 실제 송신 필드는 `ultra_ps.motor_deg` 하나입니다.

현재 Jetson의 `ultra_ps.motor_deg`는 motor pan tick을 360도 wrap으로 해석해서 계산됩니다.

```text
motor_deg = ((front_pan - pan_tick) * 360 / 4096 * PAN_DIR) % 360
```

기본 `front_pan=2048`, `PAN_DIR=1` 기준으로 `2048=0 deg`, `1024=90 deg`, `0=180 deg`, `3072=270 deg`입니다. 전체 tick 범위를 `-90..+90`으로 압축하면 실제 뒤쪽 방향이 대시보드에서 왼쪽/오른쪽처럼 보이므로 쓰면 안 됩니다.

Pi 대시보드는 Jetson이 보낸 `motor_deg`를 그대로 표시합니다. 현재 payload에는 `front_pan`, `pan_tick`, `motor_direction_deg`, `fan_deg`, `heading_deg`, `direction_deg`가 들어오지 않습니다.

우선순위:

1. `ultra_ps.motor_deg`
2. 값이 없으면 fallback으로 `ptz.pan_deg`

각도 규칙:

- `0 deg`: 위쪽 / north
- `90 deg`: 왼쪽 / west
- `180 deg`: 아래쪽 / south
- `270 deg`: 오른쪽 / east

레이더는 실제 거리(m)를 표시하지 않고, 방향만 표시합니다.

### Raspberry Pi가 받는 방법

Pi에서 dashboard 또는 controller를 실행하면 `TelemetryReceiver`가 UDP 소켓을 열고 기다립니다.

기본값:

```text
listen host: 0.0.0.0
listen port: 5005
packet: UTF-8 JSON
```

Jetson은 Pi의 같은 네트워크 IP로 UDP를 보내면 됩니다.

```text
Jetson -> <pi-ip-from-dashboard>:5005
```

장소나 핫스팟이 바뀌면 Pi IP도 바뀔 수 있습니다. dashboard를 실행하면 현재 장비의
접속 URL과 Jetson이 보낼 UDP 목적지 후보가 출력됩니다.

```text
Dashboard URLs:
  http://127.0.0.1:8000
  http://<pi-ip-from-dashboard>:8000
Jetson UDP target candidates:
  <pi-ip-from-dashboard>:5005
```

Jetson sender에는 IP를 하드코딩하지 말고 실행 시 `PI_IP`로 넘깁니다.

잘못된 JSON, UTF-8이 아닌 패킷, 필수 필드가 빠진 패킷은 무시됩니다.

## 설치

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -e ".[dev]"
```

## Tello 키보드 조종

실행 전 컴퓨터를 Tello Wi-Fi에 연결합니다.

```bash
python -m tello_control.cli
```

키보드 조종은 Tello RC mode를 사용합니다. 백그라운드 루프가 기본 20Hz로 `send_rc_control`을 보내므로 화면 표시나 telemetry 수신이 조종 명령을 막지 않습니다. 키 입력이 멈추면 자동으로 hover 명령으로 돌아갑니다.
`ARM` 입력 후에는 SSH 터미널 안에 조종용 상태 UI가 자동으로 뜹니다. 이 화면은 `dashboard.py`의 브라우저 대시보드와 별개이며, 현재 SSH로 접속 중인 노트북 터미널에서 배터리, 이륙 상태, RC 채널, 마지막 명령, 최근 이벤트를 보여줍니다.

키:

| 키 | 동작 |
| --- | --- |
| `t` | take off |
| `l` | land |
| `w` / `s` | forward / back |
| `a` / `d` | left / right |
| arrow up / down | up / down |
| arrow left / right | yaw left / right |
| `1` / `2` / `3` / `4` | flip left / forward / back / right |
| `p` | 시나리오 선택 모드. 누른 뒤 `1`, `2`, `3` 번호 선택 |
| `x` | immediate landing. 시나리오 중에는 시나리오 중단 후 착륙 요청 |
| `e` | emergency stop |
| `q` | quit, airborne이면 landing 먼저 수행 |

기본 안전/조종값:

- minimum battery: 25%
- RC send rate: 20Hz
- RC horizontal/vertical speed: 35
- RC yaw speed: 55
- RC key hold timeout: 0.22 seconds
- flip minimum battery: 50%

조종감 조정:

```bash
python -m tello_control.cli --rc-speed 30 --rc-yaw-speed 45 --rc-rate-hz 20 --rc-hold-seconds 0.22
```

터미널 UI 없이 예전처럼 텍스트 출력만 쓰려면:

```bash
python -m tello_control.cli --no-control-ui
```

## 저장된 시나리오 실행

드론 조종 CLI에서 수동 조종하다가 원하는 높이와 위치에서 `p`를 누르면 시나리오 선택 모드가 됩니다. 그 다음 `1`, `2`, `3` 번호를 누르면 해당 시나리오가 실행됩니다. 시나리오 실행 중에는 RC 루프를 잠깐 멈추고 Tello SDK 명령을 순서대로 보냅니다.

기본 예제 실행:

```bash
./run_drone.sh
```

반복 횟수 지정:

```bash
./run_drone.sh --scenario-loops 3
```

반복 횟수와 선택:

```bash
./run_drone.sh --scenario-loops 5
```

실행 중 `p`를 누른 뒤 번호를 고릅니다.

- `1`: RC 기반 전면 무한대
- `2`: 50cm 꼭짓점 정지 사각 흐름
- `3`: 시연용 중앙 복귀 경로

시나리오를 끄려면:

```bash
./run_drone.sh --no-scenario
```

시나리오 JSON 형식:

```json
{
  "name": "1_rc_front_view_infinity",
  "loops": 1,
  "steps": [
    { "command": "rc", "left_right": 22, "forward_back": 0, "up_down": 18, "yaw": 0, "seconds": 0.7 },
    { "command": "rc", "left_right": 22, "forward_back": 0, "up_down": -18, "yaw": 0, "seconds": 0.7 },
    { "command": "rc", "left_right": -22, "forward_back": 0, "up_down": -18, "yaw": 0, "seconds": 0.7 },
    { "command": "rc", "left_right": -22, "forward_back": 0, "up_down": 18, "yaw": 0, "seconds": 0.7 },
    { "command": "rc", "left_right": -22, "forward_back": 0, "up_down": 18, "yaw": 0, "seconds": 0.7 },
    { "command": "rc", "left_right": -22, "forward_back": 0, "up_down": -18, "yaw": 0, "seconds": 0.7 },
    { "command": "rc", "left_right": 22, "forward_back": 0, "up_down": -18, "yaw": 0, "seconds": 0.7 },
    { "command": "rc", "left_right": 22, "forward_back": 0, "up_down": 18, "yaw": 0, "seconds": 0.7 },
    { "command": "wait", "seconds": 0.2 }
  ]
}
```

지원 명령:

| command | 파라미터 | 설명 |
| --- | --- | --- |
| `forward` / `back` / `left` / `right` / `up` / `down` | `cm` | 20-500cm 단일 축 이동 |
| `rc` | `left_right`, `forward_back`, `up_down`, `yaw`, `seconds` | RC 속도 명령. 1번 시나리오는 `forward_back: 0`을 계속 보내 전후 명령을 막음 |
| `go` | `x`, `y`, `z`, `speed` | 상대 좌표 이동. 현재 실기 기준 `y`가 전후축, `x`가 좌우축, `z`가 상하축, `speed` 10-100 |
| `wait` | `seconds` | 대기 |

사람이 드론과 마주 보고 볼 때의 화면 평면에서만 움직이려면 `forward_back`을 항상 `0`으로 둡니다. 1번 무한대 시나리오는 `go` 대신 `rc`를 사용해서 전후 명령을 계속 0으로 보내고, 좌우와 높이만 움직입니다.

## 대시보드 실행

실제 운용 순서:

1. Raspberry Pi에서 대시보드를 먼저 실행합니다.
2. Jetson에서 `ultrachan` 또는 sender 메인을 실행해서 Pi IP의 UDP `5005`로 JSON을 보냅니다.
3. 브라우저에서 Pi dashboard 주소로 접속합니다.

Tello 조종과 함께 실행:

```bash
python -m tello_control.cli --dashboard
```

Dashboard만 실행:

```bash
python -m tello_control.dashboard
```

외부 기기에서 접속하려면 dashboard host를 `0.0.0.0`으로 열고, 브라우저에서는 Pi의 실제 IP로 접속합니다.

```bash
python -m tello_control.dashboard --host 0.0.0.0 --port 8000
```

Pi 모니터의 로컬 GUI 세션에서 브라우저까지 자동으로 열려면:

```bash
python -m tello_control.dashboard --host 0.0.0.0 --port 8000 --open-browser
```

SSH로 Pi에 접속해서 실행해도 `--open-browser`는 기본적으로 Raspberry Pi의 `:0` 디스플레이에 브라우저를 열려고 시도합니다. Pi 모니터가 다른 display 번호를 쓰면 `--browser-display`로 바꿉니다.

```bash
python -m tello_control.dashboard --host 0.0.0.0 --port 8000 --open-browser --browser-display :0
```

브라우저:

```text
http://localhost:8000
http://<raspberry-pi-ip>:8000
```

Telemetry 수신 포트까지 지정:

```bash
python -m tello_control.dashboard \
  --host 0.0.0.0 \
  --port 8000 \
  --telemetry-host 0.0.0.0 \
  --telemetry-port 5005
```

CLI와 함께 실행할 때:

```bash
python -m tello_control.cli \
  --dashboard \
  --dashboard-host 0.0.0.0 \
  --dashboard-port 8000 \
  --telemetry-host 0.0.0.0 \
  --telemetry-port 5005
```

Jetson 피격에 반응해서 뒤집고 바로 착륙하는 동작은 기본으로 켜져 있습니다:

```bash
python -m tello_control.cli \
  --dashboard \
  --telemetry-host 0.0.0.0 \
  --telemetry-port 5005
```

끄려면 `--no-auto-hit-response`를 추가합니다.

피격 시 뒤집기/착륙 대신 모터를 즉시 끄고 자유낙하시키려면:

```bash
python -m tello_control.cli \
  --dashboard \
  --hit-response free-fall
```

## Dry Run

실제 드론이나 Jetson 없이 UI와 로그를 확인합니다.

```bash
python -m tello_control.cli --dry-run --dashboard --fake-jetson
```

Dashboard만 fake Jetson으로 실행:

```bash
python -m tello_control.dashboard --fake-jetson
```

`--fake-jetson`은 `SCANNING -> DETECTED -> TRACKING -> LOCKED -> LOST` 상태를 순환하고, `x_norm/y_norm`, FPS, confidence, audio direction, `ultra_ps.motor_deg` 값을 움직여 레이더와 그래프를 갱신합니다.

## Dashboard 표시 항목

상태:

- Tello connected, airborne, battery, speed mode, last command
- Jetson status: `CONNECTED`, `STALE`, `DISCONNECTED`
- Last received age
- Tracking state
- Laser armed, fired, shot count, hit detected
- Motor direction, audio direction

상단 미터:

- Battery
- YOLO FPS
- Confidence
- X error / Y error
- Telemetry receive rate
- Audio confidence
- Motor direction
- Audio direction

그래프:

- YOLO FPS
- Confidence
- X error px
- Battery
- Telemetry receive rate
- Audio confidence

원형 레이더:

- motor direction: `ultra_ps` 방향값 우선, 없으면 `ptz.pan_deg`
- audio direction: `audio.direction_deg`

Jetson 수신 상태 timeout:

- `0.0~1.0s`: `CONNECTED`
- `1.0~3.0s`: `STALE`
- `3.0s+`: `DISCONNECTED`

## Logs

기본 로그 디렉토리:

```text
logs/
  flight_YYYYMMDD_HHMMSS.csv
  tracking_YYYYMMDD_HHMMSS.jsonl
```

조종 명령과 이벤트는 CSV로, Jetson tracking telemetry는 JSONL로 저장됩니다.

## Jetson UDP Sender 예시

Jetson에서 Raspberry Pi dashboard가 출력한 IP로 UDP JSON을 보냅니다. 더 자세한 통신 규약은 [docs/telemetry_protocol.md](docs/telemetry_protocol.md)에 정리되어 있습니다.

```python
import json
import os
import socket
import time

PI_IP = os.environ["PI_IP"]
PI_PORT = int(os.environ.get("PI_PORT", "5005"))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    payload = {
        "timestamp": time.time(),
        "frame_id": 1234,
        "target_found": True,
        "state": "TRACKING",
        "fps": 24.5,
        "confidence": 0.82,
        "frame": {
            "width": 1280,
            "height": 720,
        },
        "bbox": {
            "x1": 520,
            "y1": 260,
            "x2": 620,
            "y2": 340,
            "cx": 570,
            "cy": 300,
            "w": 100,
            "h": 80,
        },
        "error": {
            "x_px": -70,
            "y_px": -60,
            "x_norm": -0.109,
            "y_norm": -0.167,
        },
        "ptz": {
            "pan_deg": 12.5,
            "tilt_deg": -4.2,
        },
        "audio": {
            "enabled": True,
            "direction_deg": 35.0,
            "confidence": 0.62,
        },
        "ultra_ps": {
            "motor_deg": 90.0,
        },
        "laser": {
            "armed": False,
            "fired": False,
            "shot_count": 0,
            "hit_detected": False,
        },
    }

    sock.sendto(json.dumps(payload).encode("utf-8"), (PI_IP, PI_PORT))
    time.sleep(0.05)
```

최소 패킷:

```json
{
  "timestamp": 1715600000.123,
  "target_found": true,
  "state": "TRACKING"
}
```

## Tests

```bash
PYTHONPATH=src python3 -m unittest discover -s tests
```
