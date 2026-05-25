# ultra_yubin 시현 인수인계

이 문서는 조원들이 같은 Jetson/Ultra96 환경에서 시현을 이어받기 위한 운영 기준이다.

## 1. 현재 목표

시현에서 가장 중요한 것은 드론이 화면 밖으로 튀지 않게 하면서, DJI/Ryze Tello의 빠른 움직임을 따라갈 만큼 pan/tilt가 반응하는 것이다. 과정 설명은 다음 논리로 방어한다.

- Jetson은 카메라 입력과 YOLO/TensorRT bbox 검출을 담당한다.
- Ultra96 PS bridge는 UDP 수신, AXI register write/read, USB/U2D2 Dynamixel 송신을 담당한다.
- Ultra96 PL은 bbox 중심과 화면 중심의 오차로 pan/tilt goal을 계산한다.
- ReSpeaker DOA는 YOLO를 놓쳤을 때의 탐색/재획득 보조 경로로 사용한다.

## 2. 핵심 데이터 경로

비전 추적 경로:

```text
Jetson camera -> YOLO bbox -> UDP 5016 -> Ultra96 PS bridge
-> AXI 0xA0000000 -> Ultra96 PL goal compute
-> PS readback -> USB/U2D2 -> Dynamixel pan/tilt
```

오디오 보조 경로:

```text
ReSpeaker DOA -> Jetson angle -> UDP A command
-> Ultra96 PS angle-to-pan -> USB/U2D2 -> Dynamixel pan
```

교수님 질문에는 이렇게 말하면 된다.

> Jetson은 무거운 YOLO 추론만 수행하고 bbox 좌표를 Ultra96으로 보냅니다. Ultra96 PS는 bbox를 AXI register로 PL에 전달하고, PL Verilog가 이미지 오차 기반 pan/tilt goal을 계산합니다. PS는 계산된 goal을 읽어 Dynamixel로 보냅니다. 즉 제어 목표 계산 경로가 FPGA PL에 들어가 있습니다.

## 3. 현재 튜닝 상태

현재 가장 괜찮았던 구조:

- 기본 UI는 CCTV 화면, 표적지, bbox, 탐지 상태, PL/USB 상태만 보이는 미니멀 모드
- 카메라 강제 manual glare 설정 OFF
- YOLO conf는 `0.35`
- target filter는 `TRACK_TARGET_MIN_CONF=0.55`, `TRACK_TARGET_MIN_AREA=1000`
- low confidence bbox는 모터 명령에서 제외
- 화면 가장자리 bbox는 모터 명령에서 제외
- hold된 이전 bbox는 모터 명령에서 제외
- PL step table은 팬은 빠르게 유지하고, 틸트는 bbox 높이 안에서 크게 둔감하게 하도록 조정
- GUI 표적지는 실제 `aim_cx/aim_cy`를 따라가도록 수정

주의:

- 최신 RTL 소스는 PL 내부에서 축별 step table, bbox 기반 lock radius, acceleration limit을 다르게 적용한다.
- 팬은 Tello 이동을 따라갈 만큼 빠르게 유지하고, 틸트는 작은 상하 흔들림에 반응하지 않도록 더 넓은 lock radius와 낮은 step을 쓴다.
- RTL을 바꾼 뒤에는 반드시 Windows Vivado에서 bitstream을 다시 빌드해야 한다.
- 로컬 `bitstream/ultra_yubin.bit` 파일이 항상 최신 RTL과 일치한다고 가정하지 말 것.

## 4. 빌드와 배포 순서

Windows에서 bitstream 빌드 및 Jetson 전송:

```powershell
cd C:\Users\hansung\examples\ultra_yubin
.\build_and_send.ps1
```

Jetson에서 Ultra96에 bitstream load 및 bridge 재시작:

```bash
ULTRA_YUBIN_NO_PL=0 ULTRA_YUBIN_SKIP_PL_LOAD=0 ULTRA_YUBIN_SKIP_PL_INIT=0 ULTRA_YUBIN_SKIP_DXL_INIT=0 ULTRA_YUBIN_SKIP_CHECK=1 ULTRA_YUBIN_RESTART=1 ./tools/deploy_ultra96_ps_usb.sh
```

PL 확인:

```bash
python3 -c "import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.settimeout(2); s.sendto(b'PLTEST\n',('192.168.3.1',5016)); print(s.recvfrom(2048)[0].decode().strip()); s.sendto(b'PLPING\n',('192.168.3.1',5016)); print(s.recvfrom(2048)[0].decode().strip())"
```

현재 PL 감쇠 RTL이면 `PLTEST`에서 대략 `before_pan=2074`, `after_pan=2086`, `src=pl` 계열이 나온다.

## 5. 2026-05-17 현재 최우선 실행 후보

현재까지 실제 비행 체감 기준으로 가장 안정적인 기본 후보는 PL-drive이다.

```bash
cd /home/jetson/ultra_yubin_v1
./run_demo_pl_drive.sh --pipeline-echo --pipeline-echo-every 30
```

이 명령은 다음 의미를 가진다.

- Jetson: 카메라 캡처, YOLO/TensorRT bbox 검출, 추적 상태 판단
- Ultra96 PL: bbox 중심 오차 기반 pan/tilt goal 계산
- Ultra96 PS: UDP/AXI/USB-U2D2 브릿지
- Dynamixel: PL 계산 결과를 기준으로 pan/tilt 구동

교수님 발표에서는 이 버전을 기준으로 “PS에서 잘 되던 추적 제어식을 RTL로 옮겨 PL에서 결정론적으로 goal을 계산한다”고 설명한다. 단, YOLO 추론 자체를 FPGA로 돌리는 것은 아니므로 그렇게 말하면 안 된다.

현재 기준 백업 커밋:

```text
f3f4a72 pl: mirror ps best tracking profile
```

Jetson 쪽에서만 lead 보상을 추가한 실험 후보는 아래 명령이다.

```bash
cd /home/jetson/ultra_yubin_v1
./run_demo_pl_drive_lead1.sh --pipeline-echo --pipeline-echo-every 30
```

이 버전은 PL-drive는 그대로 쓰고, Jetson에서 bbox 이동 속도를 보고 약 1프레임 앞의 중심점을 PL로 보내는 실험이다. 2026-05-17 16:51 테스트에서 `avg_fps=36.9`가 나왔고 체감도 괜찮았다. 다만 lead 보상은 bbox가 튀면 앞질러 갈 수 있으므로, 시연 직전에는 반드시 기본 PL-drive와 비교한 뒤 선택한다.

롤백 기준:

```bash
./run_demo_pl_drive.sh --pipeline-echo --pipeline-echo-every 30
```

위 명령으로 돌아오면 lead 보상 없이 현재 가장 검증된 PL-drive 상태로 복귀한다.

## 6. 기존 보수 실행

긴 명령을 실수 없이 쓰기 위해 `run_demo.sh`를 사용한다.

```bash
./run_demo.sh
```

현재 `run_demo.sh` 내용은 보수적 안정 버전이다.

- `UI_MINIMAL=1`
- `ULTRA_YUBIN_CONTROL_PERIOD_SEC=0.06`
- `ULTRA_YUBIN_DEADBAND_PX=14`

Tello가 너무 작게 잡히거나 탐지가 자주 끊기면 중앙 크롭 시현 모드를 먼저 시험한다.

```bash
./run_demo_crop.sh
```

이 모드는 카메라 중앙 70% 영역을 잘라 다시 1280x720으로 키운다. YOLO 엔진 입력 크기 `640`은 그대로 두고, PL/모터 좌표계도 기존 1280x720 기준을 유지한다. 드론이 화면 가장자리에서 자주 사라지면 `CAMERA_CROP_SCALE=0.80 ./run_demo_crop.sh`처럼 크롭을 약하게 한다.

YOLO가 잡히면 PL bbox 추적을 쓰고, YOLO가 놓치면 ReSpeaker 방향으로 탐색하게 하려면:

```bash
./run_demo_audio.sh
```

현재 기본은 `TELLO_AUDIO_MODE=doa`다. 준영이 Tello audio 모델 파일은 포함되어 있지만 이 Jetson의 ReSpeaker PCM 캡처가 불안정해서, 시현 기본값은 실제 동작 확인된 ReSpeaker USB DOA를 쓴다. 모델 판별까지 강제로 켜려면 `TELLO_AUDIO_MODE=model ./run_demo_audio.sh`를 사용한다.

기존 우측 패널 HUD를 다시 보고 싶으면 실행 앞에 `UI_MINIMAL=0`을 붙인다.

조금 더 빠르게 하려면:

```bash
sed -i 's/ULTRA_YUBIN_CONTROL_PERIOD_SEC=0.06 ULTRA_YUBIN_DEADBAND_PX=14/ULTRA_YUBIN_CONTROL_PERIOD_SEC=0.05 ULTRA_YUBIN_DEADBAND_PX=12/' run_demo.sh
```

더 빠르게 하려면:

```bash
sed -i 's/ULTRA_YUBIN_CONTROL_PERIOD_SEC=0.05 ULTRA_YUBIN_DEADBAND_PX=12/ULTRA_YUBIN_CONTROL_PERIOD_SEC=0.04 ULTRA_YUBIN_DEADBAND_PX=10/' run_demo.sh
```

너무 지나치면:

```bash
sed -i 's/ULTRA_YUBIN_CONTROL_PERIOD_SEC=0.04 ULTRA_YUBIN_DEADBAND_PX=10/ULTRA_YUBIN_CONTROL_PERIOD_SEC=0.05 ULTRA_YUBIN_DEADBAND_PX=12/' run_demo.sh
```

## 6. 결과 분석

시현 후 전체 로그를 복사해서 보여줄 필요는 없다. 이 명령 결과만 보면 된다.

```bash
./tools/analyze_pipeline_tuning.py
```

주요 판단값:

- `src={'pl': ...}`: PL 경로 사용 여부
- `usb_sent`: Dynamixel 명령 송신 여부
- `mean_abs`, `last_80_target_frames`: 표적지 중심 오차
- `weak_conf_lt_0.60`: detection 품질
- `reply={'SKIP': ...}`: low_conf/edge/deadband/rate skip이 많은지

## 7. 표적지/센터 보정

GUI 표적지는 실제 제어 aim point를 따라간다. 수동으로 pan/tilt를 조금씩 움직여 bbox 중앙에 표적지를 맞출 수 있다.

자동 추적을 막고 GUI만 띄우는 실행:

```bash
CAMERA_APPLY_GLARE_DEFAULTS=0 ULTRA_YUBIN_CENTER_ON_START=0 ULTRA_YUBIN_ASYNC_SEND=0 TRACK_HOLD_LAST_FRAMES=0 TRACK_MOTOR_MIN_CONF=1.10 ./run_jetson.sh --camera auto --conf 0.30 --pipeline-echo --pipeline-echo-every 60
```

다른 터미널에서 수동 nudge:

```bash
./tools/ultra_yubin_nudge.py --step 8 --big-step 32
```

키:

```text
h/l: pan -/+
j/k: tilt +/-
H/L/J/K: 크게 이동
r: 현재 위치 읽기
c: center
q: 종료
```

한 번 맞춰본 후보값:

```text
pan=1855
tilt=2777
```

다만 이후 실행에서 `CENTER`가 `2074,3054`로 돌아가도 표적지가 괜찮게 맞은 적이 있으므로, 이 값은 필요할 때만 저장한다.

저장 명령:

```bash
./tools/ultra_yubin_calibrate_front.py --set 1855 2777 --center
```

## 8. ReSpeaker 보조 시현

ReSpeaker는 주 추적이 아니라 탐색/재획득 보조 경로로 설명한다.

통합 시현은 터미널 하나에서 실행한다.

```bash
./run_demo_audio.sh
```

동작 우선순위:

- YOLO target 있음: Jetson bbox -> Ultra96 PL -> pan/tilt
- YOLO target 없음: ReSpeaker DOA -> Ultra96 PS audio command -> pan
- 두 경로가 동시에 모터를 잡지 않도록 YOLO가 우선이다.

1회 dry-run 확인:

```bash
./tools/respeaker_to_ultra96_ps.py --once --dry-run
```

실제 Ultra96 PS로 angle 전달:

```bash
./tools/respeaker_to_ultra96_ps.py --center-on-start --period-sec 0.10 --min-change-deg 4 --clamp-deg 90
```

주의:

- 이 경로는 PL bbox 계산이 아니라 PS audio angle-to-pan 경로다.
- 교수님 설명에서는 비전 PL 추적과 오디오 fallback을 분리해서 말할 것.

## 9. 문제별 조치

`./run_jetson.sh: No such file or directory`:

- `/home/jetson/ultra_yubin` 안에서 실행해야 한다.

`ValueError: invalid literal for int()`:

- 긴 환경변수 명령을 복붙하다가 공백이 사라진 경우다.
- `run_demo.sh`를 사용한다.

`--conf expected one argument`:

- 줄바꿈 때문에 `--conf`와 `0.35`가 분리된 경우다.
- `run_demo.sh`를 사용한다.

`src=pl`이 안 나옴:

- deploy를 다시 한다.
- `ULTRA_YUBIN_NO_PL=0`인지 확인한다.

모터가 화면 밖으로 튐:

- `TRACK_MOTOR_EDGE_MARGIN_X/Y`를 올린다.
- `TRACK_MOTOR_MIN_CONF`를 올린다.
- `CONTROL_PERIOD`를 0.05 또는 0.06으로 올린다.

너무 느림:

- `CONTROL_PERIOD`를 0.04 또는 0.05로 낮춘다.
- `DEADBAND`를 10 또는 12로 낮춘다.
- 그래도 느리면 RTL step table을 다시 조정해야 한다.

## 10. 최신 소스 변경 요약

- `hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v`
  - bbox width/height register 추가
  - bbox-aware lock radius 추가
  - scale-aware step cap 추가
  - 공격형 step table 적용
- `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c`
  - bbox width/height를 PL로 전달
  - PLTEST 확장
  - front 기준 pan/tilt safety clamp 추가
- `jetson/jetson_node.py`
  - pipeline jsonl logging
  - low_conf/edge/held bbox는 모터 명령 skip
- `jetson/src/ui/display.py`
  - GUI 표적지가 실제 aim point를 따라가게 수정
- `tools/analyze_pipeline_tuning.py`
  - 로그 분석 및 offset/속도 추천
- `tools/ultra_yubin_nudge.py`
  - 수동 pan/tilt nudge
- `tools/respeaker_to_ultra96_ps.py`
  - ReSpeaker DOA -> Ultra96 PS audio command
