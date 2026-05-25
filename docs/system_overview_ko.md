# ultra_yubin_v1 전체 시스템 구조 정리

이 문서는 현재 최종 데모 기준 실행 명령인 아래 명령을 기준으로 한다.

```bash
cd /home/jetson/ultra_yubin_v1
./run_demo_pl_drive.sh --pipeline-echo --pipeline-echo-every 30
```

## 1. 시스템 한 줄 요약

Jetson이 카메라 영상에서 DJI Tello 드론을 YOLOv8 TensorRT 모델로 검출하고, 검출된 bbox 정보를 UDP로 Ultra96에 보낸다. Ultra96의 PS는 UDP/AXI/USB 제어를 담당하고, PL은 bbox 기반 pan/tilt 목표 위치 계산을 담당한다. 계산된 목표 위치는 Ultra96 PS가 U2D2 USB 어댑터를 통해 Dynamixel 모터 3개로 전달한다.

전체 구조는 다음과 같다.

```text
카메라
  -> Jetson Nano
     -> YOLOv8 TensorRT 추론
     -> bbox 추적/lead 보상
     -> UDP 5016
  -> Ultra96 PS
     -> UDP 수신
     -> AXI-Lite 레지스터 write/read
  -> Ultra96 PL
     -> bbox 중심 오차 기반 pan/tilt goal 계산
  -> Ultra96 PS
     -> Dynamixel Protocol 2.0 패킷 생성
     -> U2D2 USB serial
  -> Dynamixel 모터
     -> Pan(ID=1), Tilt(ID=2), Laser(ID=3)
```

## 2. 하드웨어 역할

### Jetson Nano

Jetson은 무거운 연산과 상위 판단을 담당한다.

- USB 카메라 입력 수신
- YOLOv8 기반 DJI Tello 드론 검출
- TensorRT `.engine` 모델 로드
- bbox 중심 좌표, bbox 크기, confidence 계산
- bbox velocity 기반 lead 보상
- 추적 상태 관리: `SCANNING`, `DETECTED`, `TRACKING`, `LOCKED`
- 목표가 없을 때 hold/reacquire 처리
- Ultra96으로 UDP 명령 송신
- 선택적으로 Tello 오디오 fallback 판단

현재 최종 모델은 다음 파일이다.

```text
/home/jetson/ultra_yubin_v1/models/drone_best_final_0520.engine
```

이 파일은 아래 흐름으로 만든 TensorRT FP16 엔진이다.

```text
best_final_0520.pt
  -> drone_best_final_0520.onnx
  -> drone_best_final_0520.engine
```

`run_demo_pl_drive.sh` 안에서 기본 모델 경로가 이 엔진으로 고정되어 있다.

```bash
YOLO_MODEL_PATH="${ROOT}/models/drone_best_final_0520.engine"
```

### Ultra96 PS

Ultra96의 PS는 Linux/C 프로그램 영역이다. 여기서는 실시간 제어 주변부를 담당한다.

- Jetson에서 오는 UDP 5016 명령 수신
- PL AXI-Lite 레지스터에 bbox 입력값 write
- PL 계산 결과 pan/tilt goal read
- U2D2 USB serial 포트 제어
- Dynamixel 모터 패킷 송신
- audio fallback 명령은 PS에서 직접 angle-to-pan 변환
- PL 상태 확인용 `PLPING` 응답
- 시스템 bring-up check 처리

관련 핵심 파일:

```text
hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c
```

### Ultra96 PL

Ultra96의 PL은 FPGA fabric 영역이다. 발표에서 말하는 하드웨어 가속 부분이다.

PL은 Jetson에서 넘어온 bbox 기반 입력을 받아서 pan/tilt 목표 위치를 계산한다.

입력 예시:

```text
cx, cy, bbox_w, bbox_h, frame_w, frame_h, confidence, valid
```

PL이 하는 일:

- bbox 중심과 카메라 중심의 오차 계산
- pan 방향 오차 계산
- tilt 방향 오차 계산
- gain/limit/deadband 반영
- pan/tilt goal position 산출
- AXI-Lite 레지스터로 결과 제공

Jetson이나 Ultra96 PS가 모든 추적 계산을 직접 하는 것이 아니라, bbox 기반 pan/tilt goal 계산을 PL 쪽으로 넘긴 구조다.

관련 산출물:

```text
bitstream/ultra_yubin_v1.bit
bitstream/ultra_yubin_v1.hwh
```

AXI base address:

```text
0xA0000000
```

## 3. 통신 흐름

### Jetson -> Ultra96

Jetson은 Ultra96 PS의 UDP 서버로 명령을 보낸다.

기본 대상:

```text
Ultra96 IP: 192.168.3.1
UDP port: 5016
```

대표 명령:

```text
PING
PLPING
G pan tilt
T cx cy bw bh fw fh conf valid
A angle conf valid
D motor_id goal
CENTER
```

현재 추적 중 가장 중요한 명령은 `T` 명령이다.

```text
T cx cy bw bh fw fh conf valid
```

의미:

- `cx`, `cy`: 드론 bbox 중심 좌표
- `bw`, `bh`: bbox 너비/높이
- `fw`, `fh`: 카메라 프레임 크기
- `conf`: YOLO confidence를 정수화한 값
- `valid`: 유효 검출 여부

### Ultra96 PS -> PL

Ultra96 PS의 C bridge가 UDP로 받은 bbox 값을 PL AXI-Lite 레지스터에 쓴다.

```text
UDP T 명령 수신
  -> AXI register write
  -> PL 계산
  -> AXI register read
```

이때 PL은 pan/tilt goal을 계산하고, PS는 그 결과를 읽어서 모터 명령으로 바꾼다.

### Ultra96 PS -> Dynamixel

Ultra96 PS는 U2D2 USB serial을 통해 Dynamixel Protocol 2.0 패킷을 보낸다.

모터 ID:

```text
Pan motor:   ID=1
Tilt motor:  ID=2
Laser motor: ID=3
```

체인 구성:

```text
U2D2 -> Pan(ID=1) -> Tilt(ID=2) -> Laser(ID=3)
```

현재 pan/tilt 제어는 PL 계산 결과를 PS가 읽어서 ID=1, ID=2에 write하는 구조다. Laser(ID=3)는 현재 중심 tick과 bbox 높이 기반 보정을 적용할 수 있도록 코드가 들어가 있지만, 레이저 정밀 보정은 아직 실험 단계다.

## 4. 현재 최종 실행 프로파일

현재 최종 데모 스크립트:

```text
run_demo_pl_drive.sh
```

핵심 설정:

```text
TRACK_DIRECT_PS=0
TRACK_PL_SHADOW=0
YOLO_MODEL_PATH=models/drone_best_final_0520.engine
YOLO_CONF=0.35
YOLO_FAST_DETECT=1
TRACK_LEAD_FRAMES=1.0
TRACK_LEAD_MAX_PX=70
ULTRA_CHAN_ASYNC_SEND=1
ULTRA_CHAN_CONTROL_PERIOD_SEC=0.007
ULTRA_YUBIN_V1_PROFILE_ACCEL=170
ULTRA_YUBIN_V1_PROFILE_VELOCITY=370
```

의미:

- `TRACK_DIRECT_PS=0`: bbox 추적 명령을 PS 직접 계산이 아니라 PL 계산 경로로 사용
- `TRACK_PL_SHADOW=0`: shadow 비교 모드가 아니라 PL-drive 실구동 모드
- `YOLO_MODEL_PATH`: 최신 0518 Tello 드론 TensorRT 엔진 사용
- `TRACK_LEAD_FRAMES`: Jetson에서 bbox 이동 속도를 보고 약 1프레임 앞을 예측
- `TRACK_LEAD_MAX_PX`: lead 보상 최대 픽셀 제한
- `ULTRA_CHAN_ASYNC_SEND=1`: 모터 명령 송신을 비동기화해서 영상 루프 지연을 줄임

## 5. Jetson에서 하는 추적 보정

PL은 bbox 기반 pan/tilt goal 계산을 담당하지만, Jetson도 아무것도 안 하는 것은 아니다. Jetson은 PL에 넘기기 전에 입력 bbox를 더 안정적으로 만든다.

Jetson 쪽 처리:

- YOLO 검출 결과 중 target 선택
- confidence threshold 적용
- bbox 중심 smoothing
- lost hold 처리
- reacquire 처리
- bbox velocity 기반 lead 보상
- 너무 큰 jump는 lead reset
- low confidence면 모터 명령 skip

즉 구조적으로는 다음처럼 분담되어 있다.

```text
Jetson:
  "어떤 bbox를 믿고 보낼지" 결정
  "움직임을 조금 앞질러 보낼지" 결정

Ultra96 PL:
  "bbox 중심 오차를 pan/tilt goal로 어떻게 바꿀지" 계산

Ultra96 PS:
  "계산된 goal을 실제 Dynamixel 패킷으로 어떻게 보낼지" 처리
```

## 6. 오디오 fallback

현재 스크립트는 `--audio-fallback`을 켠 상태로 `run_demo.sh`를 실행한다.

오디오 fallback은 YOLO가 드론을 못 잡거나 탐색 상태일 때 Tello 소리 기반 방향 추정을 보조로 쓰기 위한 기능이다.

현재 설정:

```text
TELLO_AUDIO_FALLBACK=1
TELLO_AUDIO_MODE=junmo
TELLO_AUDIO_JUNMO_MODEL=/home/jetson/junmoyolo26/tello_detector.tflite
```

오디오 경로:

```text
Jetson audio model
  -> direction angle
  -> UDP A 명령
  -> Ultra96 PS angle-to-pan 변환
  -> U2D2
  -> Pan motor
```

오디오는 PL이 아니라 Ultra96 PS에서 처리한다. 이유는 bbox 기반 2D 추적과 다르게, 오디오 fallback은 angle 값을 pan 목표로 바꾸는 단순 보조 기능이기 때문이다.

## 7. 레이저 제어 현재 상태

레이저는 카메라 렌즈 위에 장착되어 있고, 카메라와 레이저 사이의 수직 오프셋은 약 3.7cm다.

현재 코드에는 레이저 중심 lock 관련 설정이 들어가 있다.

```text
LASER_CAMERA_CENTER_LOCK=1
LASER_CAMERA_CENTER_TICK=1965
LASER_CAMERA_CENTER_RANGE_COMP=1
LASER_CAMERA_CENTER_NEAR_BBOX_H=64
LASER_CAMERA_CENTER_FAR_BBOX_H=19
LASER_CAMERA_CENTER_FAR_OFFSET_TICK=36
```

현재 의도:

- 1m 기준으로 레이저가 카메라 중앙에 오도록 `1965` tick 사용
- bbox 높이가 작아질수록 먼 거리로 보고 C모터 tick을 보정
- 다만 실제 레이저 명중 보정은 아직 실험/튜닝 단계

발표에서는 레이저는 "추적된 목표 지점을 향하도록 보정 가능한 구조" 정도로 설명하는 것이 안전하다. pan/tilt 추적 성능이 현재 핵심이다.

## 8. 발표용 기술 키워드

사용 기술을 정리하면 다음과 같다.

- YOLOv8 object detection
- TensorRT FP16 inference on Jetson
- Real-time bbox tracking
- bbox velocity lead compensation
- UDP-based Jetson-to-Ultra96 communication
- Heterogeneous embedded system
- Xilinx Ultra96 PS/PL partitioning
- AXI-Lite register interface
- FPGA-based bbox-to-goal computation
- Dynamixel Protocol 2.0 motor control
- U2D2 USB serial motor bridge
- Pan/Tilt closed-loop style visual tracking
- Audio fallback direction search

## 9. 현재 기준으로 가장 중요한 파일

```text
run_demo_pl_drive.sh
  최종 데모 실행 프로파일

jetson/jetson_node.py
  Jetson 메인 노드. 카메라, YOLO, 추적 상태, UDP 송신 담당

jetson/src/config.py
  환경변수 기반 설정값

jetson/src/vision/vision_tracker.py
  YOLO/TensorRT 모델 로드 및 검출

jetson/src/control/ultra_yubin_motor.py
  Jetson에서 Ultra96 UDP 제어 명령을 보내는 클라이언트

hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c
  Ultra96 PS UDP/AXI/U2D2 bridge

bitstream/ultra_yubin_v1.bit
bitstream/ultra_yubin_v1.hwh
  Ultra96 PL bitstream 및 HWH

models/drone_best_final_0520.engine
  현재 최종 Tello 드론 검출 TensorRT 모델
```

## 10. 한 문장 발표 버전

본 시스템은 Jetson Nano에서 YOLOv8 TensorRT 모델로 DJI Tello 드론을 실시간 검출하고, 검출 bbox를 UDP로 Ultra96에 전달한 뒤, Ultra96의 PL에서 bbox 중심 오차를 pan/tilt 목표 위치로 계산하고 PS가 U2D2를 통해 Dynamixel 모터를 제어하는 Jetson-Ultra96 이기종 분산 안티드론 추적 시스템이다.
