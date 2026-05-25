# Capstone 14 - Jetson/FPGA Drone Tracking Demo

Jetson Orin Nano, Ultra96-V2 FPGA, ReSpeaker 4 Mic Array, Raspberry Pi dashboard, U2D2/Dynamixel pan-tilt hardware를 통합한 실시간 드론 탐지 및 추적 시스템입니다.

## 시연 흐름

1. 카메라 화면 밖 또는 가려진 상태에서 드론 소리가 들어오면 ReSpeaker 기반 오디오 fallback이 드론 방향을 추정합니다.
2. Jetson이 Ultra96-V2로 오디오 방향 명령을 보내 팬틸트를 대략적인 방향으로 회전시킵니다.
3. 카메라에 드론이 잡히면 YOLO11n TensorRT 추론 결과를 기준으로 팬틸트가 드론을 화면 중앙에 추적합니다.
4. Ultra96-V2 PS/PL 브리지는 UDP 명령을 수신하고, PL goal compute 및 U2D2 Dynamixel 제어 경로로 pan/tilt/laser 목표값을 전달합니다.
5. Raspberry Pi dashboard는 Jetson/Ultra96/Audio/Motor/Laser 상태를 관람자에게 표시합니다.

## 저장소 구성

```text
jetson/                  Jetson Orin Nano 메인 추적 노드 및 Python 모듈
ultra96/pl_goal_compute/ Ultra96-V2 PS bridge, PL RTL, testbench
raspberry_pi/            Raspberry Pi dashboard 및 Tello scenario control
scripts/                 데모 실행, 배포, 벤치마크, 장치 점검 스크립트
models/                  최종 데모에 필요한 대표 모델 파일
docs/                    시스템 설명, 프로토콜, 발표/보고서용 문서
```

## 최종 데모 실행

Jetson에서:

```bash
cd /home/jetson/ultra_yubin_v1
./run_demo_pl_drive.sh
```

Raspberry Pi dashboard:

```bash
cd ~/jh
./run_dashboard.sh
```

Tello scenario control:

```bash
cd ~/jh
./run_drone.sh
```

## 핵심 구현 파일

- `jetson/jetson_node.py`: Jetson 메인 루프. 카메라, YOLO, 오디오 fallback, Ultra96 제어, dashboard telemetry를 통합합니다.
- `jetson/src/audio_fallback.py`: ReSpeaker 오디오 입력, 드론소리 분류, CNN6 방향 추정, 오디오 fallback 상태 관리를 수행합니다.
- `jetson/src/vision/vision_tracker.py`: YOLO11n/TensorRT 기반 영상 검출과 타겟 선택을 담당합니다.
- `jetson/src/control/ultra_yubin_motor.py`: Jetson에서 Ultra96-V2 PS bridge로 UDP 제어 명령을 송신합니다.
- `ultra96/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c`: Ultra96 PS에서 UDP/AXI/U2D2/Dynamixel 제어를 연결합니다.
- `ultra96/pl_goal_compute/rtl/pl_goal_compute_axi.v`: PL 영역의 AXI-Lite 기반 goal compute RTL입니다.
- `raspberry_pi/src/tello_control/dashboard.py`: Raspberry Pi 웹 대시보드입니다.
- `raspberry_pi/src/tello_control/scenario.py`: Tello 시나리오 비행 명령 처리입니다.

## 참고 문서

- `docs/code_overview_ko.md`
- `docs/demo_runbook_ko.md`
- `docs/architecture.md`
- `docs/pl_vs_ps_tracking_latency_analysis_ko.md`
- `raspberry_pi/docs/dashboard_reference.md`

