# 주요 소스코드 설명

## Jetson 인식 및 통합 제어

`jetson/jetson_node.py`는 최상위 실행 루프입니다. 카메라 프레임을 읽고 YOLO11n TensorRT 모델로 드론을 검출한 뒤, 목표 중심 좌표와 bbox 크기 정보를 Ultra96-V2로 전달합니다. 드론이 화면에서 사라진 경우에는 ReSpeaker 오디오 fallback 결과를 사용해 팬틸트를 소리 방향으로 먼저 회전시킵니다.

`jetson/src/audio_fallback.py`는 ReSpeaker 4채널 오디오를 사용합니다. 드론소리 분류 모델로 drone/noise를 판별하고, CNN6 방향 모델로 0/60/120/180/240/300도 sector를 추정합니다. 현재 데모 구조에서는 CNN6 방향 결과가 있으면 GCC-PHAT 계산을 생략하여 오디오 fallback 지연을 줄입니다.

`jetson/src/vision/vision_tracker.py`는 YOLO 추론 결과에서 드론 후보를 선택하고 bbox 중심을 계산합니다. 메인 루프는 bbox 중심과 화면 중심의 오차를 Ultra96 제어부로 넘겨 팬틸트가 드론을 중앙에 유지하도록 합니다.

`jetson/src/control/ultra_yubin_motor.py`는 Jetson과 Ultra96-V2 사이의 UDP 제어 인터페이스입니다. `T` 명령은 영상 추적용 bbox 좌표를 보내고, `A` 명령은 오디오 fallback 방향을 보냅니다.

## Ultra96-V2 PS/PL 제어

`ultra96/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c`는 Ultra96 PS에서 동작하는 C bridge입니다. Jetson UDP 명령을 수신하고 PL AXI-Lite 레지스터 또는 PS fallback 계산 결과를 통해 pan/tilt 목표값을 생성합니다. 이후 U2D2를 통해 Dynamixel pan/tilt/C모터로 goal position을 송신합니다.

`ultra96/pl_goal_compute/rtl/pl_goal_compute_axi.v`는 PL 영역의 AXI-Lite goal compute RTL입니다. bbox 중심과 frame 중심의 오차를 기반으로 deadband, step clamp, 비례 보정, 최종 goal clamp를 수행합니다.

## Raspberry Pi Dashboard 및 Tello Scenario

`raspberry_pi/src/tello_control/dashboard.py`는 Jetson에서 수신한 telemetry를 웹 대시보드로 표시합니다. Tello 상태, Jetson 연결 상태, Ultra96 응답, tracking error, latency, audio bearing, motor direction, laser 상태 등을 보여줍니다.

`raspberry_pi/src/tello_control/scenario.py`와 `raspberry_pi/scenarios/*.json`은 시나리오 기반 Tello 이동 명령을 정의합니다. 발표 시 정해진 드론 이동 패턴을 반복적으로 실행하기 위한 구조입니다.

## 모델 파일

`models/vision/drone0525jh.engine`은 현재 최종 데모 기본값으로 사용하는 TensorRT 변환 YOLO11n 드론 검출 모델입니다. `models/vision/drone_best_final_0520.engine`은 이전 안정형 백업 엔진입니다.

`models/audio/tello_detector_cnn_retrained_jetson.tflite`는 드론소리 여부를 판별하는 TFLite 모델입니다.

`models/audio_direction/junyoung_cnn6/audio_angle_cnn_final.tflite`는 ReSpeaker 4채널 입력 기반 6방향 coarse direction classifier입니다.
